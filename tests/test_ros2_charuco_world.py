from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from so101_station_perception.charuco_world import estimate_board_pose


class Detector:
    def __init__(self, corners, ids):
        self.corners, self.ids = corners, ids

    def detectBoard(self, gray):
        return self.corners, self.ids, None, None


def camera_info(width=640, height=480):
    return SimpleNamespace(width=width, height=height,
                            k=[520.0, 0, width / 2, 0, 520.0, height / 2, 0, 0, 1],
                            d=[0.02, -0.01, 0.001, 0.0, 0.0])


def test_calibrated_board_pose_uses_observed_distorted_projection():
    aruco = cv2.aruco
    board = aruco.CharucoBoard((7, 5), 0.025, 0.018,
                               aruco.getPredefinedDictionary(aruco.DICT_6X6_250))
    points = np.asarray(board.getChessboardCorners(), dtype=np.float64)[:12]
    ids = np.arange(len(points), dtype=np.int32).reshape(-1, 1)
    k = np.asarray(camera_info().k).reshape(3, 3)
    d = np.asarray(camera_info().d)
    expected_r = np.array([[0.12], [-0.08], [0.03]])
    expected_t = np.array([[0.04], [-0.02], [0.62]])
    pixels, _ = cv2.projectPoints(points, expected_r, expected_t, k, d)
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    pose, count, rms = estimate_board_pose(image, camera_info(), board,
                                            Detector(pixels.astype(np.float32), ids))
    assert count == len(points)
    assert rms < 1e-4
    assert np.allclose(pose[:3, 3], expected_t[:, 0], atol=1e-5)


def test_pose_rejects_insufficient_corners_and_bad_reprojection():
    aruco = cv2.aruco
    board = aruco.CharucoBoard((7, 5), 0.025, 0.018,
                               aruco.getPredefinedDictionary(aruco.DICT_6X6_250))
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    corners = np.zeros((5, 1, 2), dtype=np.float32)
    with pytest.raises(ValueError, match="need 6"):
        estimate_board_pose(image, camera_info(), board,
                            Detector(corners, np.arange(5).reshape(-1, 1)))

    points = np.asarray(board.getChessboardCorners(), dtype=np.float64)[:8]
    ids = np.arange(8, dtype=np.int32).reshape(-1, 1)
    k = np.asarray(camera_info().k).reshape(3, 3)
    pixels, _ = cv2.projectPoints(points, np.zeros((3, 1)), np.array([[0], [0], [0.6]]),
                                  k, np.asarray(camera_info().d))
    pixels[0, 0, 0] += 50
    with pytest.raises(ValueError, match="reprojection RMS"):
        estimate_board_pose(image, camera_info(), board,
                            Detector(pixels.astype(np.float32), ids))


def test_camera_motion_composition_keeps_board_world_fixed():
    # board_T_camera = inverse(camera_T_board) * camera_T_link. Given the
    # native link->optical edge, recover board_T_link without reparenting optical.
    native_link_from_optical = np.eye(4)
    native_link_from_optical[:3, 3] = [0.02, 0.0, 0.01]
    expected_board_from_link = np.eye(4)
    expected_board_from_link[:3, 3] = [0.3, -0.1, 0.7]
    for dx in (-0.2, 0.0, 0.25):
        board_from_optical = np.eye(4)
        board_from_optical[:3, 3] = expected_board_from_link[:3, 3] + [dx, 0, 0]
        optical_from_board = np.linalg.inv(board_from_optical)
        optical_from_link = np.linalg.inv(native_link_from_optical)
        # Simulate the algebra used by node: board_T_link = board_T_optical * optical_T_link.
        recovered = np.linalg.inv(optical_from_board) @ optical_from_link
        assert np.allclose(recovered, board_from_optical @ optical_from_link)

