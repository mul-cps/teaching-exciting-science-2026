import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "ros2/src/so101_station_objects"))

from so101_station_objects.objects import (  # noqa: E402
    StableObjectTracker,
    analyze_rgbd,
    camera_points,
    transform_points,
)


def tabletop_scene():
    rgb = np.full((480, 640, 3), 128, dtype=np.uint8)
    rgb[220:250, 305:335] = [255, 0, 0]
    depth = np.full((480, 640), 800, dtype=np.uint16)
    depth[220:250, 305:335] = 780
    intrinsics = {"fx": 500.0, "fy": 500.0, "ppx": 320.0, "ppy": 240.0}
    distortion = np.zeros(5)
    camera_to_board = np.eye(4)
    camera_to_board[:3, :3] = np.diag([1.0, -1.0, -1.0])
    camera_to_board[:3, 3] = [0.0, 0.0, 0.8]
    return rgb, depth, intrinsics, distortion, camera_to_board


def test_rgbd_scene_fits_measured_table_and_publishes_unqualified_grasp_geometry():
    rgb, depth, intrinsics, distortion, camera_to_board = tabletop_scene()
    report = analyze_rgbd(rgb, depth, intrinsics, distortion, .001, camera_to_board)
    assert report["support_plane"]["plane_source"] == "measured_table_consistent_with_board_z0"
    assert report["support_plane"]["support_points"] <= 2000
    assert len(report["objects"]) == 1
    obj = report["objects"][0]
    assert obj["color"] == "red"
    assert obj["dimensions_m"][2] == pytest.approx(.02, abs=.004)
    assert obj["geometric_only"] and not obj["ik_validated"] and not obj["collision_checked"]
    assert obj["grasps"]
    assert obj["grasps"][0]["opening_m"] < .055


def test_black_and_gray_regions_do_not_create_color_detections():
    rgb, depth, intrinsics, distortion, camera_to_board = tabletop_scene()
    rgb[80:120, 80:120] = [20, 20, 20]
    report = analyze_rgbd(rgb, depth, intrinsics, distortion, .001, camera_to_board)
    assert [obj["color"] for obj in report["objects"]] == ["red"]


def test_subcentimeter_color_fragment_is_not_reported_as_object():
    rgb, depth, intrinsics, distortion, camera_to_board = tabletop_scene()
    # At this camera/depth scale this 6 px patch is below the 10 mm footprint
    # threshold, while the existing 30 px object remains a valid brick-sized
    # footprint.
    rgb[220:226, 350:356] = [255, 0, 0]
    depth[220:226, 350:356] = 780
    report = analyze_rgbd(rgb, depth, intrinsics, distortion, .001, camera_to_board)
    assert [obj["color"] for obj in report["objects"]] == ["red"]
    assert np.min(report["objects"][0]["dimensions_m"][:2]) >= .010


def test_camera_motion_transform_backprojection_preserves_world_position():
    intrinsics = {"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0}
    world_point = np.array([[0.08, -0.04, 0.12]])
    first = np.eye(4)
    first[:3, :3] = np.diag([1.0, -1.0, -1.0])
    first[:3, 3] = [0.0, 0.0, .8]
    angle = .17
    second = np.eye(4)
    second[:3, :3] = np.diag([1.0, -1.0, -1.0]) @ np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    second[:3, 3] = [.05, -.02, .82]
    for board_from_camera in (first, second):
        camera = transform_points(world_point, np.linalg.inv(board_from_camera))
        z = camera[0, 2]
        pixel = np.array([[intrinsics["fx"] * camera[0, 0] / z + intrinsics["ppx"],
                           intrinsics["fy"] * camera[0, 1] / z + intrinsics["ppy"]]])
        reconstructed_camera = camera_points(pixel, [z], intrinsics)
        reconstructed_world = transform_points(reconstructed_camera, board_from_camera)
        np.testing.assert_allclose(reconstructed_world, world_point, atol=1e-10)


def test_stable_ids_follow_color_and_world_position_across_camera_updates():
    tracker = StableObjectTracker()
    first = tracker.update([{"color": "red", "position": np.array([.1, .2, .03])}], 1.0)
    moved_view = tracker.update([{"color": "red", "position": np.array([.102, .201, .03])}], 1.2)
    other = tracker.update([{"color": "blue", "position": np.array([.1, .2, .03])}], 1.4)
    assert moved_view[0]["track_id"] == first[0]["track_id"]
    assert moved_view[0]["velocity"].shape == (3,)
    assert other[0]["track_id"] != first[0]["track_id"]


def test_invalid_nonfinite_transform_and_depth_are_rejected():
    rgb, depth, intrinsics, distortion, camera_to_board = tabletop_scene()
    camera_to_board[0, 0] = np.nan
    with pytest.raises(ValueError, match="transform"):
        analyze_rgbd(rgb, depth, intrinsics, distortion, .001, camera_to_board)
    with pytest.raises(ValueError, match="depth scale"):
        analyze_rgbd(rgb, depth, intrinsics, distortion, float("nan"), np.eye(4))
