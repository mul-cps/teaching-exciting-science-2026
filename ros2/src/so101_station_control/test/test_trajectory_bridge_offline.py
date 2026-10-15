"""ROS action preflight integration with a fake owner and fake MoveIt scene."""
import json
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_station_control.trajectory_server import (ARM_JOINTS, TrajectoryBridge,
                                                      _arm_session_command,
                                                      _owner_state_is_ready)


class _FakeProcess:
    def poll(self):
        return None

    def wait(self, timeout=None):
        return 0


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
