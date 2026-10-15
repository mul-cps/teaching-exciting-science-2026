"""Strict loaders for ROS calibration diagnostics and frame-safe transforms."""

import json
from pathlib import Path

import numpy as np


def rigid_matrix(value, label="transform"):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{label} must be a finite 4x4 matrix")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-4)):
        raise ValueError(f"{label} must be a proper rigid SE(3) transform")
    return matrix


def _read(path):
    if not str(path):
        return None
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f"Calibration root must be an object: {path}")
    return data


def load_calibration_frames(target_path="", arm_path="", *, qualified_mode=False,
                            base_frame="follower/base_link", optical_frame="camera_color_optical_frame"):
    """Load exact transform directions and return (transforms, use_driver_tf).

    Unqualified calibration frames live under ``so101_diag_*``. A qualified
    base-to-camera transform is allowed only after explicit opt-in and exact
    ``qualified: true`` provenance in the camera-arm JSON.
    """
    target = _read(target_path)
    arm = _read(arm_path)
    transforms = []
    if qualified_mode and (arm is None or arm.get("qualified") is not True):
        raise ValueError("qualified frames require camera-arm calibration qualified: true")

    if arm is not None:
        if arm.get("schema") != "camera-arm-calibration-v1" or arm.get("transform_direction") != "base_from_camera":
            raise ValueError("camera-arm calibration must declare camera-arm-calibration-v1 / base_from_camera")
        matrix = rigid_matrix(arm.get("base_from_camera"), "base_from_camera")
        is_qualified = arm.get("qualified") is True
        if qualified_mode:
            parent, child = base_frame, optical_frame
            transforms.append({"parent": parent, "child": child, "matrix": matrix,
                               "qualified": True, "source": str(arm_path), "kind": "arm"})
        else:
            transforms.extend([
                {"parent": base_frame, "child": "so101_diag_base", "matrix": np.eye(4),
                 "qualified": False, "source": "visual-only identity anchor", "kind": "anchor"},
                {"parent": "so101_diag_base", "child": "so101_diag_camera_optical",
                 "matrix": matrix, "qualified": False, "source": str(arm_path), "kind": "arm"},
            ])

    if target is not None:
        if target.get("schema") != "camera-target-pose-v1" or target.get("transform_direction") != "camera_from_target":
            raise ValueError("target calibration must declare camera-target-pose-v1 / camera_from_target")
        matrix = rigid_matrix(target.get("camera_from_target"), "camera_from_target")
        # The target candidate is diagnostic even when the arm transform is
        # qualified; do not attach it to the live camera or planning tree.
        target_parent = optical_frame if qualified_mode else "so101_diag_camera_optical"
        transforms.append({"parent": target_parent,
                           "child": "so101_diag_charuco_target",
                           "matrix": matrix, "qualified": target.get("qualified") is True,
                           "source": str(target_path), "kind": "target"})
    # Keep the full native RealSense camera-link/optical/depth tree. In the
    # qualified case the calibrated edge is attached at camera_link after the
    # broadcaster composes the official camera_link-to-optical transform.
    return transforms, True


def matrix_quaternion(matrix):
    """Return ROS x,y,z,w quaternion from a validated rotation matrix."""
    r = rigid_matrix(matrix)[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = [0.25 * s, (r[0, 1] + r[1, 0]) / s,
                 (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif i == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s,
                 (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
                 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    return q.tolist()
