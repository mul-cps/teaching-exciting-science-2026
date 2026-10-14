"""Explicit eye-to-hand transforms. Distances are metres; rotations radians.

``a_from_b`` maps points expressed in b into a. Synthetic closure is a
solver check, never evidence of measured camera/arm accuracy.
"""
import numpy as np


def validate_se3(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-7)):
        raise ValueError("Transform must have a proper orthogonal rotation")
    return matrix.copy()


def base_from_camera_from_board(base_from_target, camera_from_target):
    return validate_se3(base_from_target) @ np.linalg.inv(validate_se3(camera_from_target))


def _pairs(base_from_gripper, camera_from_target):
    base = [validate_se3(p) for p in base_from_gripper]
    camera = [validate_se3(p) for p in camera_from_target]
    if len(base) != len(camera):
        raise ValueError("Pose lists must have equal lengths")
    return base, camera


def eye_to_hand(base_from_gripper, camera_from_target, heldout=None,
                measured_validation_tol=None, provenance=None):
    """Return a serializable calibration report using OpenCV PARK.

    heldout is a pair of independent pose lists. Qualification requires both
    heldout observations and explicit measured (translation_m, angle_rad)
    tolerances; callers must supply genuinely measured observations.
    """
    import cv2
    base, camera = _pairs(base_from_gripper, camera_from_target)
    if len(base) < 3:
        raise ValueError("At least three poses are required")
    axes = []
    for index in range(1, len(base)):
        relative = np.linalg.inv(base[0]) @ base[index]
        vector = cv2.Rodrigues(relative[:3, :3])[0].ravel()
        norm = np.linalg.norm(vector)
        if norm > 1e-5:
            axes.append(vector / norm)
    if len(axes) < 2 or np.linalg.matrix_rank(np.asarray(axes), tol=1e-3) < 2:
        raise ValueError("Rotation axes are degenerate; use diverse wrist poses")
    inverse = [np.linalg.inv(p) for p in base]
    rotation, translation = cv2.calibrateHandEye(
        [p[:3, :3] for p in inverse], [p[:3, 3] for p in inverse],
        [p[:3, :3] for p in camera], [p[:3, 3] for p in camera],
        method=cv2.CALIB_HAND_EYE_PARK)
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = rotation
    extrinsic[:3, 3] = np.asarray(translation).ravel()
    extrinsic = validate_se3(extrinsic)
    mounts = [np.linalg.inv(b) @ extrinsic @ c for b, c in zip(base, camera)]
    # A training pose supplies the mount reference, not a validation score.
    reference = mounts[0]
    residuals = []
    if heldout is not None:
        held_base, held_camera = _pairs(*heldout)
        for b, c in zip(held_base, held_camera):
            observed_mount = np.linalg.inv(b) @ extrinsic @ c
            delta = np.linalg.inv(reference) @ observed_mount
            residuals.append({"translation_m": float(np.linalg.norm(delta[:3, 3])),
                              "angle_rad": float(np.linalg.norm(cv2.Rodrigues(delta[:3, :3])[0]))})
    qualified = False
    if measured_validation_tol is not None:
        tolerance = np.asarray(measured_validation_tol, dtype=float)
        if tolerance.shape != (2,) or not np.isfinite(tolerance).all() or (tolerance <= 0).any():
            raise ValueError("Measured validation tolerances must be positive finite translation_m, angle_rad")
        qualified = bool(residuals) and all(r["translation_m"] <= tolerance[0]
                                            and r["angle_rad"] <= tolerance[1] for r in residuals)
    return {"schema": "camera-arm-calibration-v1", "transform_direction": "base_from_camera",
            "units": {"translation": "metres", "rotation": "radians"},
            "method": "opencv-PARK-eye-to-hand", "base_from_camera": extrinsic.tolist(),
            "gripper_from_target_estimates": [p.tolist() for p in mounts],
            "heldout_closure": residuals, "qualified": qualified,
            "provenance": {} if provenance is None else provenance}
