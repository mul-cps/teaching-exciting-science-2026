"""Bounded offline checks for the optional board/arm registration overlay."""
import numpy as np
import pytest

from workshop.digital_twin.registration_diagnostics import (
    arm_roi_cloud, detect_charuco_depth, diagnostic_pair_is_fresh,
    diagnostic_source_is_fresh, evaluate_charuco_depth,
)


def test_arm_roi_includes_upper_arm_and_preserves_exact_world_points():
    rows, cols = np.indices((480, 640))
    valid = np.zeros((480, 640), dtype=bool)
    valid[100, 400] = valid[200, 400] = valid[380, 400] = True
    camera_points = np.zeros((480, 640, 3), dtype=float)
    camera_points[100, 400] = [.1, .2, .5]
    camera_points[200, 400] = [.3, .4, .6]
    camera_points[380, 400] = [.5, .6, .7]
    world_from_camera = np.eye(4)
    world_from_camera[:3, 3] = [.02, -.03, .01]
    world_points = camera_points @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[100, 400] = [20, 30, 40]

    points, colors = arm_roi_cloud(world_points, rgb, valid)

    assert len(points) == 2  # y=100 upper arm and y=200 wrist; y=380 is outside the ROI.
    np.testing.assert_allclose(points[0], world_from_camera[:3, :3] @ camera_points[100, 400]
                                + world_from_camera[:3, 3])
    assert colors[0].tolist() == [20, 30, 40]


def test_stale_or_unpaired_rgbd_hides_registration_diagnostics():
    pair = {'image': 'frame.png', 'depth': 'depth.npy'}
    assert diagnostic_pair_is_fresh(pair, {'freshness': 'fresh', 'pairing_status': 'paired'})
    assert not diagnostic_pair_is_fresh(pair, {'freshness': 'stale', 'pairing_status': 'paired'})
    assert not diagnostic_pair_is_fresh(pair, {'freshness': 'fresh', 'pairing_status': 'unpaired'})
    assert not diagnostic_pair_is_fresh(None, {'freshness': 'fresh', 'pairing_status': 'paired'})
    source = {'timestamp_wall': 100.0}
    assert diagnostic_source_is_fresh(source, 100.4)
    assert not diagnostic_source_is_fresh(source, 100.6)
    assert not diagnostic_source_is_fresh({}, 100.0)


def test_synthetic_charuco_rgb_pnp_matches_depth_plane_and_board_world_transform():
    cv2 = pytest.importorskip('cv2')
    object_points = np.array([[(x + 1) * .025, (y + 1) * .025, 0.]
                              for y in range(4) for x in range(6)])
    intrinsics = {'fx': 600., 'fy': 600., 'ppx': 320., 'ppy': 240.}
    camera_matrix = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    rotation_vector = np.array([.02, -.03, .01])
    translation = np.array([-.075, -.05, .6])
    pixels, _ = cv2.projectPoints(object_points, rotation_vector, translation,
                                   camera_matrix, np.zeros(5))
    pixels = pixels.reshape(-1, 2)
    rotation, _ = cv2.Rodrigues(rotation_vector)
    rows, columns = np.indices((480, 640))
    rays = np.stack(((columns - 320.) / 600., (rows - 240.) / 600.,
                     np.ones((480, 640))), axis=-1)
    normal = rotation[:, 2]
    ray_dot = rays @ normal
    z = (normal @ translation) / ray_dot
    depth = np.where((z > 0) & np.isfinite(z), np.rint(z * 1000), 0).astype(np.uint16)
    world_from_camera = np.eye(4)
    world_from_camera[:3, :3] = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    world_from_camera[:3, 3] = [.1, -.2, .3]
    registered = np.eye(4)
    registered[:3, :3] = rotation
    registered[:3, 3] = translation

    result = evaluate_charuco_depth(object_points, pixels, depth, intrinsics, .001,
                                     world_from_camera, registered)

    assert result['qualified'] is False
    assert result['corner_count'] == 24
    assert result['depth_corner_count'] == 24
    assert result['rgb_pnp_depth_residual_rms_mm'] < 1.0
    assert result['depth_plane_rms_mm'] < 1.0
    assert result['registered_board_pnp_residual']['translation_mm'] < 1e-3
    np.testing.assert_allclose(result['world_from_detected_board'],
        world_from_camera @ registered @ np.diag([1., -1., -1., 1.]), atol=1e-5)
    camera_points = np.asarray(result['observed_depth_points_camera'])
    expected_world_points = camera_points @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    np.testing.assert_allclose(result['observed_depth_points_world'], expected_world_points, atol=1e-6)
    assert 'table_fit' not in result  # board/depth residuals stay separate from the table fit.


def test_charuco_detector_api_finds_bounded_board_corners_and_depth_plane():
    cv2 = pytest.importorskip('cv2')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((7, 5), .025, .018, dictionary)
    board_image = board.generateImage((560, 400), marginSize=0)
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[40:440, 40:600] = np.repeat(board_image[..., None], 3, axis=2)
    depth = np.full((480, 640), 188, dtype=np.uint16)

    result = detect_charuco_depth(rgb, depth,
        {'fx': 600., 'fy': 600., 'ppx': 320., 'ppy': 240.}, .001, np.eye(4))

    assert result['corner_count'] == 24
    assert result['depth_corner_count'] == 24
    assert result['depth_plane_rms_mm'] == pytest.approx(0., abs=.01)
    assert result['qualified'] is False
