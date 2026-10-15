"""ROS action preflight integration with a fake owner and fake MoveIt scene."""
import json
import time
from pathlib import Path

import rclpy
import pytest
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_station_control.trajectory_server import (ARM_JOINTS, TrajectoryBridge,
                                                      calibration_ready_raw_legs,
                                                      _arm_session_command,
                                                      _owner_state_is_ready)


class _FakeProcess:
    def poll(self):
        return None

    def wait(self, timeout=None):
        return 0


class _ExitedProcess(_FakeProcess):
    def poll(self):
        return 1


def _spin_until(executor, future, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def _goal(*, nondefault_path_tolerance=False):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(ARM_JOINTS)
    for seconds, position in ((0, 0.0), (1, 0.005)):
        point = JointTrajectoryPoint()
        point.positions = [position] * len(ARM_JOINTS)
        point.time_from_start = Duration(sec=seconds)
        goal.trajectory.points.append(point)
    if nondefault_path_tolerance:
        tolerance = JointTolerance()
        tolerance.name = ARM_JOINTS[0]
        tolerance.position = 0.01
        goal.path_tolerance.append(tolerance)
    return goal


def test_ros_owner_uses_persistent_idle_and_five_minute_timeout():
    command = _arm_session_command("/station/python", "/station/config.json",
                                   "/run/arm-state.json", "/run/arm-logs",
                                   "/run/station.lock", 300)
    assert command == ["/station/python", "-m", "workshop.station.arm_session",
                       "--config", "/station/config.json",
                       "--state-file", "/run/arm-state.json",
                       "--logdir", "/run/arm-logs", "--lock", "/run/station.lock",
                       "--persistent-idle", "--idle-timeout", "300.0"]


def test_owner_startup_accepts_fresh_first_sample_with_matching_identity():
    state = {"event": "sample", "monotonic": 12.1,
             "arm_id": "station-arm", "calibration_sha256": "a" * 64,
             "torque": {"joint": 0}}
    assert _owner_state_is_ready(state, arm_id="station-arm",
                                 calibration_sha256="a" * 64, owner_started=12.0)
    assert not _owner_state_is_ready(state, arm_id="wrong-arm",
                                     calibration_sha256="a" * 64, owner_started=12.0)
    assert not _owner_state_is_ready({**state, "monotonic": 11.9}, arm_id="station-arm",
                                     calibration_sha256="a" * 64, owner_started=12.0)
    assert not _owner_state_is_ready({**state, "torque": {"joint": 1}}, arm_id="station-arm",
                                     calibration_sha256="a" * 64, owner_started=12.0)


def test_late_teleop_step_error_after_watchdog_keeps_bridge_disabled(monkeypatch, tmp_path):
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-teleop-watchdog-test"])
    root = Path(__file__).resolve().parents[4]
    runtime_state = tmp_path / "owner-state.json"

    def start_fake_owner(node):
        node._state_file = runtime_state
        node._process = _FakeProcess()
        node._owner_started = time.monotonic()

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", lambda _node, _command: None)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(runtime_state)),
    ])
    session_id = "expired-session"
    with bridge._active_lock:
        bridge._teleop_state = "active"
        bridge._teleop_session = session_id
        bridge._teleop_inflight = True
        bridge._teleop_inflight_sequence = 7
        bridge._active_goal = "teleop"
        bridge._teleop_step_done.clear()
    try:
        watchdog = {"command": "teleop_watchdog", "session_id": session_id,
                    "ok": False, "state": {"event": "teleop_watchdog_hold"}}
        assert bridge._route_teleop_response(watchdog)
        assert bridge._teleop_state == "disabled"
        assert bridge._active_goal is None
        assert bridge._teleop_session is None
        assert bridge._teleop_step_done.is_set()

        late_error = {"command": "teleop_step", "session_id": session_id,
                      "ok": False,
                      "error": "teleop watchdog expired; measured pose held"}
        assert bridge._route_teleop_response(late_error)
        assert bridge._teleop_state == "disabled"
        assert bridge._active_goal is None
        assert bridge._teleop_session is None
        assert not bridge._teleop_inflight
        assert bridge._response_queue.empty()
        assert "teleop_step" not in bridge._teleop_acknowledgements
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_action_preflight_success_collision_tolerance_and_cancel(monkeypatch, tmp_path):
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-action-test"])
    root = Path(__file__).resolve().parents[4]
    runtime_state = tmp_path / "owner-state.json"
    durable_artifacts = tmp_path / "trajectory-artifacts"
    commands = []
    scene = {"mode": "valid", "calls": 0}
    hold = {"enabled": False}
    state = {
        "schema": 1, "event": "sample", "arm_id": "offline-fake",
        "calibration_sha256": "offline-fake", "timestamp_utc": "offline",
        "monotonic": time.monotonic(),
        "raw": {joint: 2048 for joint in (*ARM_JOINTS, "gripper")},
        "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
        "torque": dict.fromkeys((*ARM_JOINTS, "gripper"), 1),
    }

    def start_fake_owner(node):
        runtime_state.write_text(json.dumps(state))
        node._state_file = runtime_state
        node._process = _FakeProcess()
        node._owner_started = time.monotonic()

    def send_to_fake_owner(node, command):
        commands.append(command)
        if command["command"] == "execute_trajectory" and not hold["enabled"]:
            node._response_queue.put({"ok": True, "state": state,
                                      "trajectory": {"feedback_count": 21}})
        elif command["command"] == "cancel_hold":
            node._response_queue.put({"cancelled": True, "state": state})

    def refresh_state():
        state["monotonic"] = time.monotonic()
        runtime_state.write_text(json.dumps(state))

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_fake_owner)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(runtime_state)),
        Parameter("trajectory_log_dir", Parameter.Type.STRING, str(durable_artifacts)),
    ])

    def validity_callback(_request, response):
        scene["calls"] += 1
        response.valid = scene["mode"] == "valid" or scene["calls"] < 3
        return response

    bridge.create_service(GetStateValidity, "/follower/check_state_validity", validity_callback)
    client_node = Node("so101_offline_action_client")
    client = ActionClient(
        client_node, FollowJointTrajectory,
        "/follower/arm_trajectory_controller/follow_joint_trajectory")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    try:
        assert client.wait_for_server(timeout_sec=3.0)

        refresh_state()
        accepted = _spin_until(executor, client.send_goal_async(_goal()))
        assert accepted.accepted
        result = _spin_until(executor, accepted.get_result_async())
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        assert len([item for item in commands if item["command"] == "execute_trajectory"]) == 1
        assert scene["calls"] >= 2
        artifact = next(durable_artifacts.glob("trajectory-*.json"))
        assert json.loads(artifact.read_text())["status"] == "succeeded"

        commands.clear()
        scene.update(mode="collision", calls=0)
        refresh_state()
        accepted = _spin_until(executor, client.send_goal_async(_goal()))
        assert accepted.accepted
        result = _spin_until(executor, accepted.get_result_async())
        assert result.result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
        assert not any(item["command"] == "execute_trajectory" for item in commands)

        refresh_state()
        rejected = _spin_until(executor, client.send_goal_async(
            _goal(nondefault_path_tolerance=True)))
        assert not rejected.accepted

        commands.clear()
        scene.update(mode="valid", calls=0)
        hold["enabled"] = True
        refresh_state()
        accepted = _spin_until(executor, client.send_goal_async(_goal()))
        assert accepted.accepted
        deadline = time.monotonic() + 5.0
        while not any(item["command"] == "execute_trajectory" for item in commands):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.01)
        cancel_response = _spin_until(executor, accepted.cancel_goal_async())
        assert cancel_response.goals_canceling
        result = _spin_until(executor, accepted.get_result_async())
        assert result.status == 5  # action_msgs/GoalStatus.STATUS_CANCELED
        assert any(item["command"] == "cancel_hold" for item in commands)
    finally:
        executor.remove_node(client_node)
        executor.remove_node(bridge)
        client_node.destroy_node()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_calibration_endpoint_is_aborted_and_recorded_without_faulting_owner(monkeypatch, tmp_path):
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-calibration-endpoint-test"])
    root = Path(__file__).resolve().parents[4]
    runtime_state = tmp_path / "owner-state.json"
    artifacts = tmp_path / "trajectory-artifacts"
    commands = []
    state = {
        "schema": 1, "event": "sample", "arm_id": "offline-fake",
        "calibration_sha256": "offline-fake", "timestamp_utc": "offline",
        "monotonic": time.monotonic(),
        "raw": {joint: 2048 for joint in (*ARM_JOINTS, "gripper")},
        "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
        "torque": dict.fromkeys((*ARM_JOINTS, "gripper"), 1),
    }

    def start_fake_owner(node):
        runtime_state.write_text(json.dumps(state))
        node._state_file = runtime_state
        node._process = _FakeProcess()
        node._owner_started = time.monotonic()

    def send_to_fake_owner(node, command):
        commands.append(command)
        if command["command"] == "execute_trajectory":
            node._response_queue.put({
                "ok": True, "status": "calibration_measured_endpoint_held",
                "target_reached": False, "goal_error_rad": {"elbow_flex": 0.0399},
                "state": state, "trajectory": {"feedback_count": 70},
            })

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_fake_owner)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(runtime_state)),
        Parameter("trajectory_log_dir", Parameter.Type.STRING, str(artifacts)),
        Parameter("calibration_endpoint_mode", Parameter.Type.BOOL, True),
    ])
    def validity_callback(_request, response):
        response.valid = True
        return response

    bridge.create_service(GetStateValidity, "/follower/check_state_validity", validity_callback)
    client_node = Node("so101_offline_calibration_endpoint_client")
    client = ActionClient(client_node, FollowJointTrajectory,
                          "/follower/arm_trajectory_controller/follow_joint_trajectory")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    try:
        assert client.wait_for_server(timeout_sec=3.0)
        state["monotonic"] = time.monotonic()
        runtime_state.write_text(json.dumps(state))
        accepted = _spin_until(executor, client.send_goal_async(_goal()))
        assert accepted.accepted
        result = _spin_until(executor, accepted.get_result_async())
        assert result.result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        assert result.result.error_string.startswith("calibration_measured_endpoint_held")
        command = next(item for item in commands if item["command"] == "execute_trajectory")
        assert command["calibration_endpoint"] is True
        artifact = json.loads(next(artifacts.glob("trajectory-*.json")).read_text())
        assert artifact["status"] == "calibration_measured_endpoint_held"
        assert artifact["target_reached"] is False
        assert artifact["result"]["goal_error_rad"]["elbow_flex"] == pytest.approx(0.0399)
        assert bridge._owner_faulted is False
        assert bridge._process.poll() is None
    finally:
        executor.remove_node(client_node)
        executor.remove_node(bridge)
        client_node.destroy_node()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_home_service_uses_owner_and_denies_busy_or_unavailable(monkeypatch, tmp_path):
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-home-service-test"])
    root = Path(__file__).resolve().parents[4]
    runtime_state = tmp_path / "owner-state.json"
    state = {"schema": 1, "event": "sample", "monotonic": time.monotonic(),
             "raw": {joint: 2048 for joint in (*ARM_JOINTS, "gripper")},
             "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
             "torque": dict.fromkeys((*ARM_JOINTS, "gripper"), 0)}
    commands = []

    def start_fake_owner(node):
        runtime_state.write_text(json.dumps(state))
        node._state_file = runtime_state
        node._process = _FakeProcess()
        node._owner_started = time.monotonic()

    def send_to_fake_owner(node, command):
        commands.append(command)
        if command == {"command": "home"}:
            node._response_queue.put({"ok": True, "state": {"event": "home_complete"}})

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_fake_owner)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(runtime_state)),
    ])
    client_node = Node("so101_offline_home_service_client")
    client = client_node.create_client(Trigger, "/follower/home")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    try:
        assert client.wait_for_service(timeout_sec=3.0)
        future = client.call_async(Trigger.Request())
        result = _spin_until(executor, future)
        assert result.success
        assert result.message == "measured home completed"
        assert commands == [{"command": "home"}]
        assert bridge._active_goal is None

        with bridge._active_lock:
            bridge._active_goal = "teleop"
        busy = Trigger.Response()
        bridge._home(Trigger.Request(), busy)
        assert not busy.success
        assert "active" in busy.message
        assert commands == [{"command": "home"}]
        with bridge._active_lock:
            bridge._active_goal = None

        bridge._process = _ExitedProcess()
        unavailable = Trigger.Response()
        bridge._home(Trigger.Request(), unavailable)
        assert not unavailable.success
        assert "unavailable" in unavailable.message
        assert commands == [{"command": "home"}]
        assert bridge._active_goal is None
    finally:
        executor.remove_node(client_node)
        executor.remove_node(bridge)
        client_node.destroy_node()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_calibration_ready_service_is_explicit_identity_checked_and_bounded(monkeypatch, tmp_path):
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-calibration-ready-service-test"])
    root = Path(__file__).resolve().parents[4]
    runtime_state = tmp_path / "owner-state.json"
    pose_file = tmp_path / "observation-pose.json"
    joints = (*ARM_JOINTS, "gripper")
    state = {"schema": 1, "event": "sample", "monotonic": time.monotonic(),
             "arm_id": "rig01", "calibration_sha256": "a" * 64,
             "raw": dict.fromkeys(joints, 2000),
             "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
             "torque": dict.fromkeys(joints, 0)}
    target = {"shoulder_pan": 2807, "shoulder_lift": 1594, "elbow_flex": 2151,
              "wrist_flex": 2825, "wrist_roll": 2320, "gripper": 2168}
    commands = []

    def start_fake_owner(node):
        runtime_state.write_text(json.dumps(state))
        node._state_file = runtime_state
        node._process = _FakeProcess()
        node._owner_started = time.monotonic()

    def send_to_fake_owner(node, command):
        commands.append(command)
        if command.get("command") == "move_plan":
            node._response_queue.put({"ok": True, "state": {"event": "plan_complete"}})

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_fake_owner)
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(runtime_state)),
        Parameter("calibration_ready_pose_file", Parameter.Type.STRING, str(pose_file)),
    ])
    client_node = Node("so101_offline_calibration_ready_client")
    client = client_node.create_client(Trigger, "/follower/calibration_ready")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)

    def invoke():
        return _spin_until(executor, client.call_async(Trigger.Request()))

    try:
        assert client.wait_for_service(timeout_sec=3.0)
        pose_file.write_text(json.dumps({"arm_id": "rig01",
                                         "calibration_sha256": "a" * 64,
                                         "raw": target}))
        # The mode defaults off even when the pose file is configured.
        disabled = invoke()
        assert not disabled.success
        assert "calibration_endpoint_mode" in disabled.message
        assert not commands
        assert bridge.set_parameters([
            Parameter("calibration_endpoint_mode", Parameter.Type.BOOL, True)]
        )[0].successful

        pose_file.write_text(json.dumps({"arm_id": "other-arm",
                                         "calibration_sha256": "a" * 64,
                                         "raw": target}))
        mismatch = invoke()
        assert not mismatch.success
        assert "identity" in mismatch.message
        assert not commands

        pose_file.write_text(json.dumps({"arm_id": "rig01",
                                         "calibration_sha256": "b" * 64,
                                         "raw": target}))
        calibration_mismatch = invoke()
        assert not calibration_mismatch.success
        assert "identity" in calibration_mismatch.message
        assert not commands

        invalid_target = {**target, "elbow_flex": 4096}
        pose_file.write_text(json.dumps({"arm_id": "rig01",
                                         "calibration_sha256": "a" * 64,
                                         "raw": invalid_target}))
        invalid = invoke()
        assert not invalid.success
        assert "0..4095" in invalid.message
        assert not commands

        pose_file.write_text(json.dumps({"arm_id": "rig01",
                                         "calibration_sha256": "a" * 64,
                                         "raw": target}))
        ready = invoke()
        assert ready.success
        assert "not MoveIt- or collision-qualified" in ready.message
        assert len(commands) == 1
        command = commands[0]
        assert command["command"] == "move_plan"
        assert len(command["legs"]) <= 100
        assert command["legs"][-1]["target_raw"] == target
        previous = state["raw"]
        for leg in command["legs"]:
            assert all(abs(leg["target_raw"][joint] - previous[joint]) <= 100
                       for joint in joints)
            previous = leg["target_raw"]
        assert bridge._active_goal is None

        with bridge._active_lock:
            bridge._active_goal = "home"
        busy = invoke()
        assert not busy.success
        assert "active" in busy.message
        assert len(commands) == 1
        with bridge._active_lock:
            bridge._active_goal = None
    finally:
        executor.remove_node(client_node)
        executor.remove_node(bridge)
        client_node.destroy_node()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_calibration_ready_raw_legs_reject_out_of_range_and_make_bounded_steps():
    joints = (*ARM_JOINTS, "gripper")
    start = dict.fromkeys(joints, 0)
    target = dict.fromkeys(joints, 4095)
    legs = calibration_ready_raw_legs(start, target)
    assert len(legs) == 41
    previous = start
    for leg in legs:
        assert all(abs(leg["target_raw"][joint] - previous[joint]) <= 100
                   for joint in joints)
        previous = leg["target_raw"]
    invalid = {**target, "gripper": -1}
    with pytest.raises(ValueError, match="0..4095"):
        calibration_ready_raw_legs(start, invalid)
