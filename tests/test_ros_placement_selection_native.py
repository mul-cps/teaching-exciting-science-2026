"""Native ROS contract test for the camera placement selection service."""
import time
import json

import numpy as np
import pytest


pytest.importorskip("rclpy")
pytest.importorskip("so101_station_workflow")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from so101_station_workflow.srv import SelectPlacementRegion

from workshop.digital_twin.ros_state_node import RosStatePublisher


def test_ui_pins_latest_fresh_frame_and_publishes_exact_display_ack(tmp_path):
    if not rclpy.ok():
        rclpy.init(args=None)
    node = RosStatePublisher(str(tmp_path / "state.json"), root_frame="charuco_board",
        placement_directory=str(tmp_path / "placement-camera"))
    try:
        now = time.monotonic()
        base = {"stamp_ns": 1_000_000_000, "received_monotonic": now - 1.0,
            "frame_id": "camera_color_optical_frame", "root_frame": "charuco_board",
            "root_from_camera": None, "base_from_camera": None,
            "camera_matrix": np.eye(3), "rgb": np.zeros((8, 12, 3), dtype=np.uint8),
            "depth_m": np.ones((8, 12), dtype=np.float32),
            "image_shape": [8, 12], "image_path": str(tmp_path / "old.jpg")}
        node.placement_frames.append(dict(base, generation="old-frame"))
        fresh = dict(base, generation="fresh-frame", stamp_ns=2_000_000_000,
            received_monotonic=time.monotonic(), image_path=str(tmp_path / "fresh.jpg"))
        node.placement_frames.append(fresh)

        node._process_placement_request({"request_id": "ui-1",
            "action": "pin_latest_frame", "mode": "select_feeder_anchor"})

        assert node.placement_status["state"] == "selection_frame_pinned"
        assert node.placement_pinned_frame["generation"] == "fresh-frame"
        assert node.placement_status["selection_request_id"] == "ui-1"
        node._write()
        snapshot = json.loads((tmp_path / "state.json").read_text())
        ack = snapshot["placement_selection_frame"]
        assert ack["request_id"] == "ui-1"
        assert ack["mode"] == "select_feeder_anchor"
        assert ack["generation"] == "fresh-frame"
        assert ack["image_path"] == fresh["image_path"]
        assert ack["image_shape"] == [8, 12]
        assert "rgb" not in ack and "depth_m" not in ack
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


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


def test_native_pin_service_survives_human_delay_and_selects_exact_generation(tmp_path):
    """Exercise ROS client/server transport, not only the Python callback."""
    if not rclpy.ok():
        rclpy.init(args=None)
    server = RosStatePublisher(str(tmp_path / "state.json"), root_frame="charuco_board",
        placement_directory=str(tmp_path / "placement-camera"))
    client_node = rclpy.create_node("placement_pin_contract_test")
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    client = client_node.create_client(SelectPlacementRegion, "/station/placement/select")

    def call(mode, generation="", rectangle=(0, 0, 0, 0)):
        request = SelectPlacementRegion.Request()
        request.generation = generation
        request.mode = mode
        request.pixel_rectangle = list(rectangle)
        assert client.wait_for_service(timeout_sec=2.)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.done(), f"service call timed out: {mode}"
        return future.result()

    try:
        rng = np.random.default_rng(716)
        image = np.zeros((160, 200, 3), dtype=np.uint8)
        image[25:135, 25:175] = rng.integers(10, 245, (110, 150, 3), dtype=np.uint8)
        k = np.array([[300., 0., 100.], [0., 300., 80.], [0., 0., 1.]])
        stamp = 3_100_000_000
        generation = f"{stamp}-ros-service-pin"
        server.placement_frames.append({"generation": generation, "stamp_ns": stamp,
            "received_monotonic": time.monotonic(), "frame_id": "camera_color_optical_frame",
            "root_frame": "charuco_board", "root_from_camera": None,
            "base_from_camera": None, "camera_matrix": k, "rgb": image,
            "depth_m": np.ones((160, 200), dtype=np.float32),
            "image_path": "", "image_shape": [160, 200]})

        pinned = call("pin_feeder")
        assert pinned.success, pinned.message
        ack = json.loads(pinned.message)
        assert ack["state"] == "selection_frame_pinned"
        assert ack["mode"] == "pin_feeder"
        assert ack["generation"] == generation
        assert ack["expires_in_s"] == 30.
        assert ack["request_id"]

        # The live RGB-D frame ages out of the ordinary selection window while
        # the user drags the rectangle. The server must use the acknowledged
        # pinned generation, never a newer image with old screen coordinates.
        time.sleep(.60)
        selected = call("feeder", ack["generation"], (20, 20, 180, 140))
        assert selected.success, selected.message
        assert selected.selection_id
        assert server.placement_status["generation"] == generation
        assert server.placement_status["state"] == "anchor_selected"

        # Goal pinning is gated by a tracked feeder anchor. Model one fresh
        # tracker update and a newly received camera frame, then verify the
        # second human selection also remains bound to its acknowledged frame.
        server.placement_status["anchor_tracking_valid"] = True
        goal_stamp = stamp + 100_000_000
        goal_generation = f"{goal_stamp}-ros-service-goal-pin"
        server.placement_frames.append({"generation": goal_generation,
            "stamp_ns": goal_stamp, "received_monotonic": time.monotonic(),
            "frame_id": "camera_color_optical_frame", "root_frame": "charuco_board",
            "root_from_camera": None, "base_from_camera": None,
            "camera_matrix": k, "rgb": image,
            "depth_m": np.ones((160, 200), dtype=np.float32),
            "image_path": "", "image_shape": [160, 200]})
        tracked = server.placement_frontend.update(image, np.ones((160, 200), dtype=np.float32),
            k, np.eye(4), stamp_ns=goal_stamp, frame_id="camera_color_optical_frame",
            generation=goal_generation, received_monotonic=time.monotonic())
        assert tracked.reason == "tracked", tracked.reason
        goal_pin = call("pin_goal")
        assert goal_pin.success, goal_pin.message
        goal_ack = json.loads(goal_pin.message)
        assert goal_ack["state"] == "selection_frame_pinned"
        assert goal_ack["mode"] == "pin_goal"
        assert goal_ack["generation"] == goal_generation
        assert goal_ack["expires_in_s"] == 30.
        time.sleep(.60)
        goal = call("goal", goal_ack["generation"], (70, 55, 130, 105))
        assert goal.success, goal.message
        assert goal.selection_id == selected.selection_id
        assert server.placement_status["generation"] == goal_generation
        assert server.placement_status["state"] == "goal_selected"

        cancel_generation = f"{goal_stamp + 100_000_000}-cancel-pin"
        server.placement_frames.append({"generation": cancel_generation,
            "stamp_ns": goal_stamp + 100_000_000, "received_monotonic": time.monotonic(),
            "frame_id": "camera_color_optical_frame", "root_frame": "charuco_board",
            "root_from_camera": None, "base_from_camera": None,
            "camera_matrix": k, "rgb": image,
            "depth_m": np.ones((160, 200), dtype=np.float32),
            "image_path": "", "image_shape": [160, 200]})
        cancellable = call("pin_feeder", cancel_generation)
        assert cancellable.success, cancellable.message
        cancel_ack = json.loads(cancellable.message)
        cancelled = call("cancel_pin", cancel_ack["generation"])
        assert cancelled.success, cancelled.message
        assert json.loads(cancelled.message)["state"] == "selection_cancelled"
        assert server.placement_pinned_frame is None
    finally:
        executor.remove_node(client_node)
        executor.remove_node(server)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.try_shutdown()
