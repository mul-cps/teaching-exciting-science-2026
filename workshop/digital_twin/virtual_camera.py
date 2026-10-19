"""Pure synthetic tool-camera geometry; never physical camera calibration."""
import math
import numpy as np


def tool_from_virtual_optical(forward_axis="+Z", backoff_m=.04):
    axes = {"+X": (1., 0., 0.), "-X": (-1., 0., 0.),
            "+Y": (0., 1., 0.), "-Y": (0., -1., 0.),
            "+Z": (0., 0., 1.), "-Z": (0., 0., -1.)}
    if forward_axis not in axes or not np.isfinite(backoff_m) or not 0 <= backoff_m <= .15:
        raise ValueError("virtual eye requires an axis and 0..0.15 m backoff")
    forward = np.asarray(axes[forward_axis])
    up = np.array([0., 0., 1.]) if forward_axis.endswith("Y") else np.array([0., -1., 0.])
    result = np.eye(4)
    # ROS optical axes: X right, Y down, Z forward.
    result[:3, :3] = np.column_stack((np.cross(forward, up), -up, forward))
    result[:3, 3] = -forward * backoff_m
    return result


def virtual_intrinsics(width=320, height=240, vertical_fov_deg=60.):
    if (not 1 <= width <= 640 or not 1 <= height <= 480 or
            not 10 <= vertical_fov_deg <= 120):
        raise ValueError("virtual image bounds are 640x480 and 10..120 degree FOV")
    focal = height / (2 * math.tan(math.radians(vertical_fov_deg) / 2))
    return np.array([[focal, 0., (width - 1) / 2],
                     [0., focal, (height - 1) / 2], [0., 0., 1.]])


def optical_projection(intrinsics, width, height, near=.01, far=2.):
    """GLES clip coordinates from optical metres, including pixel-center offset."""
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or not np.isfinite([near, far]).all() or not 0 < near < far or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("finite intrinsics and 0 < near < far required")
    return np.array([[2*k[0, 0]/width, 0., 2*(k[0, 2]+.5)/width-1, 0.],
                     [0., -2*k[1, 1]/height, 1-2*(k[1, 2]+.5)/height, 0.],
                     [0., 0., (far+near)/(far-near), -2*far*near/(far-near)],
                     [0., 0., 1., 0.]], dtype=np.float32)


def validate_pinch_overlay(artifact):
    if (artifact.get("schema") != "so101-virtual-pinch-overlay-v1" or
            artifact.get("parent_frame") != "urdf:gripper_frame_link" or
            artifact.get("qualified") is not False):
        raise ValueError("NOMINAL pinch artifact schema/frame/qualification mismatch")
    pose = np.asarray(artifact.get("parent_from_pinch"), dtype=float)
    angle = artifact.get("opening_angle_rad")
    if (pose.shape != (4, 4) or not np.isfinite(pose).all() or
            not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-7) or
            not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-7) or
            not isinstance(angle, (int, float)) or not np.isfinite(angle)):
        raise ValueError("NOMINAL pinch requires proper finite SE3 and opening angle")
    return artifact


def project_pinch_overlay(artifact, tool_from_optical, intrinsics, width, height,
                          *, opening_angle_rad, joint_stamp_ns, acquisition_stamp_ns, now_ns,
                          near=.01, far=2.):
    """Project configured nominal pinch point; never assume image-center alignment."""
    validate_pinch_overlay(artifact)
    if (joint_stamp_ns is None or opening_angle_rad is None or
            not 0 <= now_ns - joint_stamp_ns <= 500_000_000 or
            abs(joint_stamp_ns - acquisition_stamp_ns) > 50_000_000 or
            not np.isfinite(opening_angle_rad) or
            abs(opening_angle_rad - artifact["opening_angle_rad"]) > 1e-5):
        return None, "opening_mismatch_or_stale"
    point = (np.linalg.inv(tool_from_optical) @
             np.asarray(artifact["parent_from_pinch"], dtype=float))[:3, 3]
    if not near <= point[2] <= far:
        return None, "pinch_outside_depth_frustum"
    uvh = np.asarray(intrinsics) @ point
    uv = uvh[:2] / uvh[2]
    if not np.isfinite(uv).all() or not (0 <= uv[0] < width and 0 <= uv[1] < height):
        return None, "pinch_outside_image"
    return uv, "nominal_uncalibrated"
