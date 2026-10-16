"""Native ROS contract test for the camera placement selection service."""
import time

import numpy as np
import pytest


pytest.importorskip("rclpy")
pytest.importorskip("so101_station_workflow")

import rclpy
from so101_station_workflow.srv import SelectPlacementRegion

from workshop.digital_twin.ros_state_node import RosStatePublisher


def test_selection_service_uses_pinned_rgbd_frame_and_returns_identity(tmp_path):
    if not rclpy.ok():
        rclpy.init(args=None)
    node = RosStatePublisher(str(tmp_path / "state.json"), root_frame="charuco_board",
        placement_directory=str(tmp_path / "placement-camera"))
    try:
        assert node.placement_selection_timeout_s == 30.
        rng = np.random.default_rng(91)
        image = np.zeros((160, 200, 3), dtype=np.uint8)
        image[25:135, 25:175] = rng.integers(10, 245, (110, 150, 3), dtype=np.uint8)
        stamp = 1_100_000_000
        generation = f"{stamp}-native"
        received = time.monotonic()
        k = np.array([[300., 0., 100.], [0., 300., 80.], [0., 0., 1.]])
        node.placement_frames.append({"generation": generation, "stamp_ns": stamp,
            "received_monotonic": received, "frame_id": "camera_color_optical_frame",
            "root_frame": "charuco_board", "root_from_camera": None,
            "base_from_camera": None, "camera_matrix": k, "rgb": image,
            "depth_m": np.ones((160, 200), dtype=np.float32),
            "image_path": "", "image_shape": [160, 200]})

        feeder = SelectPlacementRegion.Request()
        feeder.generation = generation
        feeder.mode = "feeder"
        feeder.pixel_rectangle = [20, 20, 180, 140]
        anchor_response = node._placement_selection_service(
            feeder, SelectPlacementRegion.Response())
        assert anchor_response.success, anchor_response.message
        assert anchor_response.selection_id
        selection_id = anchor_response.selection_id

        area = SelectPlacementRegion.Request()
        area.generation = generation
        area.mode = "goal"
        area.pixel_rectangle = [70, 55, 130, 105]
        goal_response = node._placement_selection_service(
            area, SelectPlacementRegion.Response())
        assert goal_response.success, goal_response.message
        assert goal_response.selection_id == selection_id
        assert node.placement_status["camera_from_goal_stamp_ns"] == stamp
        assert node.placement_status["plane_metrics"]["depth_samples"] >= 20
        assert node.placement_status["physical_placement_admission"] is False

        clear = SelectPlacementRegion.Request()
        clear.generation = generation
        clear.mode = "clear"
        cleared = node._placement_selection_service(clear, SelectPlacementRegion.Response())
        assert cleared.success
        assert node.placement_status["valid"] is False
        assert node.placement_tracker.anchor_points is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
