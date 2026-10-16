"""Native ROS GripperCommand action roundtrips with a fake resident owner."""
import json
import threading
import time
from pathlib import Path

import rclpy
import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from so101_station_control.trajectory_server import ARM_JOINTS, TELEOP_JOINTS, TrajectoryBridge


JAW_CALIBRATION = {
    "closed_gap_m": 0.004,
    "open_gap_m": 0.075,
    "closed_percent": 0.0,
    "open_percent": 100.0,
}
ARM_ID = "offline-fake-arm"
CALIBRATION_SHA256 = "a" * 64


class _FakeProcess:
    def poll(self):
        return None

    def wait(self, timeout=None):
        return 0

    def terminate(self):
        return None


def _spin_until(executor, future, timeout=8.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(), "ROS future did not complete"
    return future.result()


def _sample_state():
    return {
        "schema": 1,
        "event": "sample",
        "arm_id": ARM_ID,
        "calibration_sha256": CALIBRATION_SHA256,
        "monotonic": time.monotonic(),
        "raw": {joint: 1000 for joint in TELEOP_JOINTS},
        "pose": {**dict.fromkeys(ARM_JOINTS, 0.0), "gripper": 50.0},
    }


def _start_action_bridge(monkeypatch, tmp_path, send_to_owner):
    root = Path(__file__).resolve().parents[4]
    state_file = tmp_path / "owner-state.json"
    state_file.write_text(json.dumps(_sample_state()))

    def start_fake_owner(bridge):
        bridge._state_file = state_file
        bridge._process = _FakeProcess()
        bridge._owner_started = time.monotonic()
        bridge._expected_arm_id = ARM_ID
        bridge._expected_calibration_sha256 = CALIBRATION_SHA256
        bridge._gripper_raw_limits = (500, 1500)
        bridge._gripper_jaw = JAW_CALIBRATION.copy()

    monkeypatch.setattr(TrajectoryBridge, "_start_owner", start_fake_owner)
    monkeypatch.setattr(TrajectoryBridge, "_send", send_to_owner)
    if not rclpy.ok():
        rclpy.init(args=["so101-offline-gripper-action-test"])
    bridge = TrajectoryBridge(parameter_overrides=[
        Parameter("enable_hardware", Parameter.Type.BOOL, False),
        Parameter("station_root", Parameter.Type.STRING, str(root)),
        Parameter("state_file", Parameter.Type.STRING, str(state_file)),
        Parameter("gripper_action_name", Parameter.Type.STRING,
                  "/follower/gripper_controller/gripper_cmd"),
    ])
    client_node = Node("so101_offline_gripper_action_client")
    client = ActionClient(client_node, GripperCommand,
                          "/follower/gripper_controller/gripper_cmd")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(client_node)
    assert client.wait_for_server(timeout_sec=3.0)
    return bridge, client_node, client, executor


def _goal(gap_m, *, max_effort=0.0):
    goal = GripperCommand.Goal()
    goal.command.position = gap_m
    goal.command.max_effort = max_effort
    return goal


def _close_action_bridge(bridge, client_node, executor):
    executor.remove_node(client_node)
    executor.remove_node(bridge)
    client_node.destroy_node()
    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_open_and_contact_inferred_close_roundtrip_through_gripper_action(
        monkeypatch, tmp_path):
    commands = []
    def send_to_owner(bridge, command):
        commands.append(command)
        if command["command"] != "move_plan":
            return
        leg = command["legs"][0]
        target_raw = leg["target_raw"]["gripper"]
        contact = target_raw == 500
        final_percent = 68.0 if contact else 100.0
        bridge._response_queue.put({
            "ok": True,
            "state": {
                "event": "plan_complete",
                "pose": {"gripper": final_percent},
                "legs": [{"label": "gripper_command",
                          "contact_inferred": contact}],
            },
        })

    bridge, client_node, client, executor = _start_action_bridge(
        monkeypatch, tmp_path, send_to_owner)
    try:
        opened = _spin_until(executor, client.send_goal_async(_goal(0.075)))
        assert opened.accepted
        open_result = _spin_until(executor, opened.get_result_async())
        assert open_result.status == GoalStatus.STATUS_SUCCEEDED
        assert open_result.result.reached_goal
        assert not open_result.result.stalled
        assert open_result.result.position == 0.075
        assert commands[-1]["legs"][0]["target_raw"]["gripper"] == 1500
        assert commands[-1]["legs"][0]["grasp"] is False

        closed = _spin_until(executor, client.send_goal_async(_goal(0.004)))
        assert closed.accepted
        close_result = _spin_until(executor, closed.get_result_async())
        assert close_result.status == GoalStatus.STATUS_SUCCEEDED
        assert close_result.result.stalled
        assert not close_result.result.reached_goal
        assert close_result.result.position == pytest.approx(0.05228)
        assert commands[-1]["legs"][0]["target_raw"]["gripper"] == 500
        assert commands[-1]["legs"][0]["grasp"] is True
        assert not bridge._owner_faulted
    finally:
        _close_action_bridge(bridge, client_node, executor)


def test_nonzero_force_is_rejected_by_action_server(monkeypatch, tmp_path):
    commands = []
    bridge, client_node, client, executor = _start_action_bridge(
        monkeypatch, tmp_path,
        lambda _bridge, command: commands.append(command))
    try:
        rejected = _spin_until(executor, client.send_goal_async(
            _goal(0.04, max_effort=1.0)))
        assert not rejected.accepted
        assert commands == []
    finally:
        _close_action_bridge(bridge, client_node, executor)


def test_cancel_before_owner_submission_cancels_action_without_motion(
        monkeypatch, tmp_path):
    entered_submit = threading.Event()
    allow_submit = threading.Event()
    commands = []
    bridge, client_node, client, executor = _start_action_bridge(
        monkeypatch, tmp_path,
        lambda _bridge, command: commands.append(command))
    original_submit = bridge._submit_gripper_plan

    def gated_submit(command):
        entered_submit.set()
        assert allow_submit.wait(timeout=4.0)
        return original_submit(command)

    monkeypatch.setattr(bridge, "_submit_gripper_plan", gated_submit)
    try:
        handle = _spin_until(executor, client.send_goal_async(_goal(0.004)))
        assert handle.accepted
        assert entered_submit.wait(timeout=3.0)
        cancel_response = _spin_until(executor, handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        allow_submit.set()
        result = _spin_until(executor, handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert commands == []
        assert not bridge._owner_faulted
    finally:
        allow_submit.set()
        _close_action_bridge(bridge, client_node, executor)


def test_cancel_during_closure_holds_owner_and_cancels_action(monkeypatch, tmp_path):
    move_submitted = threading.Event()
    commands = []
    bridge = None

    def send_to_owner(owner, command):
        commands.append(command)
        if command["command"] == "move_plan":
            move_submitted.set()
        elif command["command"] == "cancel_hold":
            owner._response_queue.put({
                "ok": True,
                "state": {"event": "trajectory_cancel_hold"},
            })

    bridge, client_node, client, executor = _start_action_bridge(
        monkeypatch, tmp_path, send_to_owner)
    try:
        handle = _spin_until(executor, client.send_goal_async(_goal(0.004)))
        assert handle.accepted
        assert move_submitted.wait(timeout=3.0)
        cancel_response = _spin_until(executor, handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        result = _spin_until(executor, handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert [command["command"] for command in commands] == [
            "move_plan", "cancel_hold"]
        assert not bridge._owner_faulted
        assert bridge._active_goal is None
    finally:
        _close_action_bridge(bridge, client_node, executor)
