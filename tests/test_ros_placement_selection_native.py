"""Native ROS contract test for the camera placement selection service."""
import time
import json
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import cv2


pytest.importorskip("rclpy")
pytest.importorskip("so101_station_workflow")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from so101_station_workflow.srv import SelectPlacementRegion
from sensor_msgs.msg import CameraInfo, Image

from workshop.digital_twin.ros_state_node import RosStatePublisher
from workshop.digital_twin.placement_region import PlacementRegionTracker, SparseRGBDAnchor


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


def test_native_pin_replays_bounded_camera_history_for_delayed_anchor_selection(
        tmp_path, monkeypatch):
    """Pin, wait like a human, then replay only bounded color history to fresh depth."""
    if not rclpy.ok():
        rclpy.init(args=None)
    server = RosStatePublisher(str(tmp_path / "state.json"), root_frame="charuco_board",
        placement_directory=str(tmp_path / "placement-camera"))
    client_node = rclpy.create_node("placement_history_replay_contract_test")
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
        rng = np.random.default_rng(801)
        image = np.zeros((160, 220, 3), dtype=np.uint8)
        texture = rng.integers(10, 245, (90, 130), dtype=np.uint8)
        image[35:125, 35:165] = np.repeat(texture[..., None], 3, axis=2)
        depth = np.ones((160, 220), dtype=np.float32)
        k = np.array([[300., 0., 110.], [0., 300., 80.], [0., 0., 1.]])
        stamp = 5_100_000_000
        generation = f"{stamp}-delayed-pin"
        pin_received = time.monotonic()
        first = {"generation": generation, "stamp_ns": stamp,
            "received_monotonic": pin_received, "frame_id": "camera_color_optical_frame",
            "root_frame": "charuco_board", "root_from_camera": None,
            "base_from_camera": None, "camera_matrix": k, "rgb": image,
            "depth_m": depth, "image_path": "", "image_shape": [160, 220]}
        server.placement_frames.append(first)
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        ok, jpg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 82])
        assert ok
        server.placement_gray_history.append({"stamp_ns": stamp,
            "generation": generation, "frame_id": first["frame_id"],
            "received_monotonic": pin_received, "jpeg": jpg.tobytes()})
        pin = call("pin_feeder")
        assert pin.success, pin.message
        ack = json.loads(pin.message)
        assert ack["generation"] == generation

        # While the user chooses the rectangle, retain compressed gray only.
        # Model live 20 Hz camera arrivals for longer than the tracker's direct
        # 500 ms reference window.
        for index in range(1, 16):
            time.sleep(.05)
            dx = index
            current = np.zeros_like(image)
            current[35:125, 35 + dx:165 + dx] = image[35:125, 35:165]
            current_stamp = stamp + index * 50_000_000
            current_generation = f"{current_stamp}-replay-{index}"
            received = time.monotonic()
            gray = cv2.cvtColor(current, cv2.COLOR_RGB2GRAY)
            ok, jpg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 82])
            assert ok
            server.placement_gray_history.append({"stamp_ns": current_stamp,
                "generation": current_generation, "frame_id": first["frame_id"],
                "received_monotonic": received, "jpeg": jpg.tobytes()})
            server.placement_frames.append({**first, "generation": current_generation,
                "stamp_ns": current_stamp, "received_monotonic": received,
                "rgb": current})

        assert (server.placement_frames[-1]["stamp_ns"] - stamp) / 1e9 > .5
        assert len(server.placement_gray_history) == 16
        assert all(set(item) == {"stamp_ns", "generation", "frame_id",
                                  "received_monotonic", "jpeg"}
                   for item in server.placement_gray_history)
        monkeypatch.setattr(SparseRGBDAnchor, "_descriptor_observations",
            lambda *args: (_ for _ in ()).throw(
                AssertionError("anchor-history replay must not use descriptor reacquisition")))
        replay_entered = threading.Event()
        tail_entered = threading.Event()
        allow_replay = threading.Event()
        allow_tail = threading.Event()
        original_replay = SparseRGBDAnchor.replay_to_current
        replay_threads = []

        def blocked_worker(self, *args, **kwargs):
            replay_threads.append(threading.get_ident())
            if not replay_entered.is_set():
                replay_entered.set()
                assert allow_replay.wait(2.)
            else:
                tail_entered.set()
                assert allow_tail.wait(2.)
            return original_replay(self, *args, **kwargs)

        monkeypatch.setattr(SparseRGBDAnchor, "replay_to_current", blocked_worker)
        selected = call("feeder", ack["generation"], (30, 30, 170, 130))
        assert selected.success, selected.message
        assert selected.selection_id
        assert server.placement_status["state"] == "selection_replay_pending"
        assert replay_entered.wait(1.)
        # A camera callback can append while the isolated replay is still busy.
        dx = 16
        tail_rgb = np.zeros_like(image)
        tail_rgb[35:125, 35 + dx:165 + dx] = image[35:125, 35:165]
        tail_stamp = stamp + 800_000_000
        tail_generation = f"{tail_stamp}-replay-tail"
        tail_record = {**first, "generation": tail_generation,
            "stamp_ns": tail_stamp, "received_monotonic": time.monotonic(),
            "rgb": tail_rgb}
        server.placement_frames.append(tail_record)
        server._append_placement_gray_frame(tail_record, tail_rgb)
        allow_replay.set()
        deadline = time.monotonic() + 2.
        while not tail_entered.is_set() and time.monotonic() < deadline:
            server._finish_placement_replay()
            time.sleep(.002)
        assert tail_entered.is_set()
        assert len(set(replay_threads)) == 1, "all replay work must stay on the worker"

        # Many newer frames arrive while a costly tail is replaying. Polling the
        # completion from the ROS thread must stay bounded and must retain all
        # newest frames for another worker catch-up pass.
        for index in range(17, 37):
            dx = index
            current = np.zeros_like(image)
            current[35:125, 35 + dx:165 + dx] = image[35:125, 35:165]
            current_stamp = stamp + index * 50_000_000
            current_generation = f"{current_stamp}-tail-{index}"
            current_record = {**first, "generation": current_generation,
                "stamp_ns": current_stamp, "received_monotonic": time.monotonic(),
                "rgb": current}
            server.placement_frames.append(current_record)
            server._append_placement_gray_frame(current_record, current)
        start = time.monotonic()
        server._finish_placement_replay()
        assert time.monotonic() - start < .05, "timer callback performed blocking catch-up"
        allow_tail.set()
        deadline = time.monotonic() + 2.
        while server.placement_replay_active and time.monotonic() < deadline:
            server._finish_placement_replay()
            time.sleep(.005)
        assert not server.placement_replay_active
        assert server.placement_status["state"] == "anchor_tracked"
        assert server.placement_status["anchor_tracking_valid"] is True
        latest_generation = server.placement_frames[-1]["generation"]
        latest_stamp = server.placement_frames[-1]["stamp_ns"]
        assert server.placement_status["generation"] == latest_generation
        assert server.placement_status["stamp_ns"] == latest_stamp
        assert server.placement_status["feature_count"] >= 6
        assert len(server.placement_gray_history) == 0
    finally:
        executor.remove_node(client_node)
        executor.remove_node(server)
        client_node.destroy_node()
        server.destroy_node()
        rclpy.try_shutdown()


def test_failed_replacement_replay_cannot_resurrect_previous_anchor_or_goal(tmp_path,
        monkeypatch):
    """An accepted B selection retires A even if B's history replay fails."""
    if not rclpy.ok():
        rclpy.init(args=None)
    server = RosStatePublisher(str(tmp_path / "state.json"), root_frame="charuco_board",
        placement_directory=str(tmp_path / "placement-camera"))
    try:
        rng = np.random.default_rng(49)
        image = rng.integers(0, 255, (160, 200, 3), dtype=np.uint8)
        depth = np.ones((160, 200), dtype=np.float32)
        k = np.array([[300., 0., 100.], [0., 300., 80.], [0., 0., 1.]])
        old = server.placement_frontend
        assert old.select_anchor(image, depth, k, np.eye(4), (20, 20, 180, 140),
            stamp_ns=1_000_000_000, frame_id="camera_color_optical_frame",
            generation="old-A", received_monotonic=time.monotonic())
        old_id = old.tracker.selection_id
        assert old_id
        old.tracker.goal_corners_local = np.array([[0., 0., 1.], [.1, 0., 1.],
                                                   [.1, .1, 1.], [0., .1, 1.]])
        server.placement_status = {"state": "goal_selected", "valid": False,
            "selection_id": old_id, "goal_corners_camera": old.tracker.goal_corners_local.tolist()}

        stamp = 2_000_000_000
        first = {"generation": "replacement-B-pinned", "stamp_ns": stamp,
            "received_monotonic": time.monotonic(),
            "frame_id": "camera_color_optical_frame", "root_frame": "charuco_board",
            "root_from_camera": None, "base_from_camera": None,
            "camera_matrix": k, "rgb": image, "depth_m": depth,
            "image_path": "", "image_shape": [160, 200]}
        server.placement_frames.append(first)
        server._process_placement_request({"action": "pin_latest_frame",
            "mode": "select_feeder_anchor", "request_id": "replacement-B"})
        assert server.placement_status["state"] == "selection_frame_pinned"
        latest = {**first, "generation": "replacement-B-latest",
            "stamp_ns": stamp + 100_000_000,
            "received_monotonic": time.monotonic()}
        server.placement_frames.append(latest)
        server._append_placement_gray_frame(latest, image)
        monkeypatch.setattr(SparseRGBDAnchor, "replay_to_current",
            lambda *args, **kwargs: SimpleNamespace(reason="insufficient_features"))
        server._process_placement_request({"action": "select_feeder_anchor",
            "generation": first["generation"], "rect_xyxy": (20, 20, 180, 140)})
        assert server.placement_replay_active
        assert server.placement_frontend is not old
        assert server.placement_frontend.tracker.anchor_points is None
        assert server.placement_tracker.goal_corners_local is None
        deadline = time.monotonic() + 1.
        while server.placement_replay_active and time.monotonic() < deadline:
            server._finish_placement_replay()
            time.sleep(.002)
        assert server.placement_status["reason"] == "history_replay_insufficient_features"

        assert server.placement_status["reason"] == "history_replay_insufficient_features"
        assert server.placement_status.get("selection_id") != old_id
        assert server.placement_frontend is not old
        assert server.placement_frontend.tracker.anchor_points is None
        assert server.placement_tracker.goal_corners_local is None

        # A valid fresh camera pair after the failure must not update the old
        # frontend or make its UUID/goal valid again.
        stamp = 3_000_000_000
        color = server.cv_bridge.cv2_to_imgmsg(image, encoding="rgb8")
        depth_msg = server.cv_bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        info = CameraInfo()
        for msg in (color, depth_msg, info):
            msg.header.stamp.sec = stamp // 1_000_000_000
            msg.header.stamp.nanosec = stamp % 1_000_000_000
            msg.header.frame_id = "camera_color_optical_frame"
        info.width, info.height = 200, 160
        info.k = k.reshape(-1).tolist()
        info.d = []
        server._placement_rgbd(color, depth_msg, info)
        assert server.placement_status["state"] == "invalid"
        assert server.placement_status["reason"] == "history_replay_insufficient_features"
        assert server.placement_status.get("selection_id") != old_id
    finally:
        server.destroy_node()
        rclpy.try_shutdown()
