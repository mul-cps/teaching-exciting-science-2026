import cv2
import numpy as np
import pytest

pytest.importorskip("rclpy", reason="Optional ROS perception tests require RoboStack")

from so101_station_perception.charuco_plane import (camera_from_world_from_plane,
                                                     fit_depth_plane,
                                                     plane_tilt_from_pnp_degrees)


def _scene():
    width, height = 640, 480
    k = np.array([[520., 0, 320.], [0, 520., 240.], [0, 0, 1.]])
    d = np.zeros(5)
    rvec = np.array([[0.16], [-0.1], [0.04]])
    tvec = np.array([[0.19], [0.12], [0.65]])
    rotation, _ = cv2.Rodrigues(rvec)
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, tvec[:, 0]
    yy, xx = np.mgrid[:height, :width]
    rays = np.column_stack(((xx.ravel() - k[0, 2]) / k[0, 0],
                            (yy.ravel() - k[1, 2]) / k[1, 1],
                            np.ones(width * height)))
    normal = rotation[:, 2]
    scale = np.dot(normal, tvec[:, 0]) / (rays @ normal)
    depth = scale.reshape(height, width).astype(np.float32)
    corners, _ = cv2.projectPoints(np.array([[0, 0, 0], [.175, 0, 0],
                                              [.175, .125, 0], [0, .125, 0]], float),
                                   rvec, tvec, k, d)
    polygon = np.rint(corners.reshape(-1, 2)).astype(np.int32)
    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, polygon, 255)
    depth[mask == 0] = 0
    # Elevated colored LEGO pieces and invalid holes are common board occluders.
    cv2.rectangle(depth, tuple(polygon[0] + [35, 30]), tuple(polygon[0] + [95, 82]), .625, -1)
    depth[polygon[2, 1] - 18:polygon[2, 1], polygon[2, 0] - 25:polygon[2, 0]] = np.nan
    return depth, k, d, pose


def test_fit_robust_tilted_board_plane_with_objects_and_missing_depth():
    depth, k, d, pose = _scene()
    color = np.full((*depth.shape, 3), 200, dtype=np.uint8)
    color[366:418, 507:567] = [20, 35, 240]
    center, normal, metrics = fit_depth_plane(depth, k, d, pose, color_image=color)
    expected = -pose[:3, 2]
    assert np.dot(normal, expected) > .999
    assert metrics["plane_inliers"] >= 80
    assert metrics["plane_coverage_cells"] >= 8
    assert metrics["plane_median_residual_mm"] < 1.0
    assert metrics["plane_p95_residual_mm"] < 2.5
    assert metrics["plane_vivid_color_rejected"] > 0
    assert abs(float(np.dot(center - pose[:3, 3], normal))) < .003


def test_world_axes_are_right_handed_z_up_and_origin_lies_on_plane():
    depth, k, d, pose = _scene()
    center, normal, _ = fit_depth_plane(depth, k, d, pose)
    optical_from_world = camera_from_world_from_plane(pose, center, normal)
    axes = optical_from_world[:3, :3]
    assert np.linalg.det(axes) == pytest.approx(1.0, abs=1e-8)
    assert np.dot(axes[:, 2], normal) > .999999
    assert abs(float(np.dot(optical_from_world[:3, 3] - center, normal))) < 1e-8
    assert np.dot(axes[:, 0], pose[:3, 0]) > .999


def test_fit_rejects_low_coverage_or_missing_depth():
    depth, k, d, pose = _scene()
    sparse = np.zeros_like(depth)
    sparse[335:430, 475:520] = depth[335:430, 475:520]
    with pytest.raises(ValueError, match="board regions"):
        fit_depth_plane(sparse, k, d, pose)
    with pytest.raises(ValueError, match="no valid depth"):
        fit_depth_plane(np.zeros_like(depth), k, d, pose)


def test_small_projected_board_uses_its_pixel_budget_not_full_image_area():
    # The board has hundreds of valid interior pixels, but a full-image stride
    # of seven drops most of them before fitting and falsely rejects support.
    depth = np.full((480, 640), 2.0, dtype=np.float32)
    k = np.array([[520., 0, 320.], [0, 520., 240.], [0, 0, 1.]])
    pose = np.eye(4)
    pose[:3, 3] = [-.0875, -.0625, 2.0]
    _, _, metrics = fit_depth_plane(depth, k, np.zeros(5), pose)
    assert 80 <= metrics["plane_inliers"] <= 2000
    assert metrics["plane_coverage_cells"] >= 8
    assert metrics["plane_p95_residual_mm"] < .001


def test_plane_tilt_refuses_inconsistent_depth_normal():
    _, _, _, pose = _scene()
    normal = -pose[:3, 2]
    tangent = pose[:3, 0]
    inconsistent = normal * np.cos(np.deg2rad(18)) + tangent * np.sin(np.deg2rad(18))
    assert plane_tilt_from_pnp_degrees(pose, normal) == pytest.approx(0, abs=1e-5)
    assert plane_tilt_from_pnp_degrees(pose, inconsistent) == pytest.approx(18, abs=1e-5)
    with pytest.raises(ValueError, match="disagrees with ChArUco PnP"):
        plane_tilt_from_pnp_degrees(pose, inconsistent, max_tilt_degrees=12)


def test_depth_encoding_conversion_is_explicit():
    from so101_station_perception.charuco_plane import decode_depth
    assert decode_depth(np.array([[1250]], np.uint16), "16UC1")[0, 0] == pytest.approx(1.25)
    assert decode_depth(np.array([[1.25]], np.float32), "32FC1")[0, 0] == pytest.approx(1.25)
    with pytest.raises(ValueError, match="unsupported"):
        decode_depth(np.zeros((2, 2), np.uint8), "8UC1")
