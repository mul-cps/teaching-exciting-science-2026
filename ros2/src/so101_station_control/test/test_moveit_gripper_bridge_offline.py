"""Native ROS angular gripper roundtrips; never accesses physical hardware."""
import time
import threading
from types import SimpleNamespace

import pytest
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from so101_station_control.trajectory_server import TrajectoryBridge
from test_gripper_bridge_offline import (
    _start_action_bridge, _close_action_bridge, _spin_until, _goal, _sample_state,
    ARM_ID, CALIBRATION_SHA256,
)

JOINT = dict(closed_joint_rad=-.1, open_joint_rad=1.5,
             closed_percent=0., open_percent=100., arm_id=ARM_ID,
             calibration_sha256=CALIBRATION_SHA256)


def _angular_state():
    state = _sample_state()
    state["arm_bounds"] = dict(schema="arm-bounds-observation-v1", classification="normal",
        position_monotonic=state["monotonic"],
        admitted_min=dict.fromkeys(state["raw"],500), admitted_max=dict.fromkeys(state["raw"],1500),
        firmware_min=dict.fromkeys(state["raw"],0), firmware_max=dict.fromkeys(state["raw"],4095))
    return state


def test_missing_angle_calibration_rejects_without_owner_command(monkeypatch, tmp_path):
    commands = []
    bridge, node, _metric, executor = _start_action_bridge(monkeypatch, tmp_path,
        lambda self, command: commands.append(command))
    client = ActionClient(node, GripperCommand, "/follower/moveit_gripper_controller/gripper_cmd")
    try:
        assert client.wait_for_server(timeout_sec=3.)
        handle = _spin_until(executor, client.send_goal_async(_goal(.7)))
        assert not handle.accepted
        assert commands == []
        assert bridge._active_goal is None
    finally:
        client.destroy()
        _close_action_bridge(bridge, node, executor)


def test_moveit_intermediate_angle_converts_units_and_binds_owner(monkeypatch, tmp_path):
    commands = []
    def send(bridge, command):
        commands.append(command)
        final = _sample_state()
        final.update(event="plan_complete", pose={**final["pose"], "gripper": 50.})
        bridge._response_queue.put(dict(ok=True, state=final))
    bridge, node, _metric, executor = _start_action_bridge(monkeypatch, tmp_path, send)
    bridge._gripper_joint = JOINT.copy()
    monkeypatch.setattr(bridge, "_read_state", _angular_state)
    monkeypatch.setattr(bridge, "_gripper_collision_binding",
        lambda raw: dict(expected_start_raw={"gripper": 1000}, owner_session_id="fake-session"))
    client = ActionClient(node, GripperCommand, "/follower/moveit_gripper_controller/gripper_cmd")
    try:
        assert client.wait_for_server(timeout_sec=3.)
        handle = _spin_until(executor, client.send_goal_async(_goal(.7)))
        assert handle.accepted
        result = _spin_until(executor, handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.position == pytest.approx(.7)
        assert result.result.reached_goal
        assert commands == [dict(command="move_gripper", target_raw=1000,
            expected_start_raw={"gripper": 1000}, owner_session_id="fake-session")]
    finally:
        client.destroy()
        _close_action_bridge(bridge, node, executor)


def test_cancel_before_angular_dispatch_sends_no_motion(monkeypatch, tmp_path):
    commands = []
    bridge, node, _metric, executor = _start_action_bridge(monkeypatch, tmp_path,
        lambda self, command: commands.append(command))
    bridge._gripper_joint = JOINT.copy()
    monkeypatch.setattr(bridge, "_read_state", _angular_state)
    checking = threading.Event()
    def cancel_during_preflight(_target):
        checking.set()
        deadline = time.monotonic()+3.
        while not bridge._gripper_goal_cancelled and time.monotonic() < deadline:
            time.sleep(.005)
        assert bridge._gripper_goal_cancelled
        raise InterruptedError("cancelled during preflight")
    monkeypatch.setattr(bridge, "_gripper_collision_binding", cancel_during_preflight)
    client = ActionClient(node, GripperCommand, "/follower/moveit_gripper_controller/gripper_cmd")
    try:
        assert client.wait_for_server(timeout_sec=3.)
        handle = _spin_until(executor, client.send_goal_async(_goal(.7)))
        assert handle.accepted
        assert checking.wait(timeout=3.)
        assert _spin_until(executor, handle.cancel_goal_async()).goals_canceling
        result = _spin_until(executor, handle.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert commands == []
        assert bridge._active_goal is None
    finally:
        client.destroy()
        _close_action_bridge(bridge, node, executor)


def test_collision_binding_checks_all_raw_samples_and_rejects_changed_start(monkeypatch, tmp_path):
    bridge, node, _metric, executor = _start_action_bridge(monkeypatch, tmp_path, lambda *_: None)
    bridge._gripper_joint = JOINT.copy()
    monkeypatch.setattr(bridge, "_read_state", _angular_state)
    bridge._active_goal = "gripper"
    bridge._teleop_state = "disabled"
    state = _sample_state()
    state["owner_session_id"] = "same-owner"
    state["raw"]["gripper"] = 1000
    state["arm_bounds"] = dict(schema="arm-bounds-observation-v1", classification="normal",
        position_monotonic=state["monotonic"],
        admitted_min=dict.fromkeys(state["raw"], 500), admitted_max=dict.fromkeys(state["raw"], 1500),
        firmware_min=dict.fromkeys(state["raw"], 0), firmware_max=dict.fromkeys(state["raw"], 4095))
    monkeypatch.setattr(bridge, "_assert_owner_reservation", lambda *_: None)
    monkeypatch.setattr(bridge, "_prepare_owner_state", lambda *_args, **_kw: "normal")
    monkeypatch.setattr(bridge, "_read_state", lambda: state)
    samples = []
    class Future:
        def add_done_callback(self, callback): callback(self)
        def result(self): return SimpleNamespace(valid=True)
    class Validity:
        def wait_for_service(self, **_kw): return True
        def call_async(self, request):
            assert request.group_name == ""
            samples.append(request.robot_state.joint_state.position[-1])
            return Future()
    bridge._state_validity = Validity()
    try:
        binding = bridge._gripper_collision_binding(1002)
        assert binding["expected_start_raw"] == state["raw"]
        assert samples == pytest.approx([.7, .7016, .7032])
        calls = iter([state, {**state, "raw": {**state["raw"], "gripper": 1001}}])
        monkeypatch.setattr(bridge, "_read_state", lambda: next(calls))
        with pytest.raises(RuntimeError, match="start changed"):
            bridge._gripper_collision_binding(1002)
        monkeypatch.setattr(bridge, "_prepare_owner_state",
            lambda *_args, **_kw: (_ for _ in ()).throw(RuntimeError("stale owner")))
        monkeypatch.setattr(bridge, "_read_state", lambda: None)
        with pytest.raises(RuntimeError, match="stale owner"):
            bridge._gripper_collision_binding(1002)
    finally:
        _close_action_bridge(bridge, node, executor)


def test_feedback_uses_fresh_identity_generation_source_cache_without_file_reads(monkeypatch,tmp_path):
    bridge,node,_metric,executor=_start_action_bridge(monkeypatch,tmp_path,lambda *_:None)
    state=_angular_state()
    state.update(observe_only=False,owner_source_sha256={"fake":"b"*64},torque_monotonic=state["monotonic"])
    bridge._owner_source_expected={"fake":"b"*64}
    bridge._cached_owner_state=state
    bridge._cached_owner_generation=bridge._owner_started
    monkeypatch.setattr(bridge,"_read_state",lambda:(_ for _ in ()).throw(AssertionError("filesystem read forbidden")))
    try:
        assert bridge._gripper_feedback_state() is state
        bridge._cached_owner_generation-=1.
        assert bridge._gripper_feedback_state() is None
        bridge._cached_owner_generation=bridge._owner_started
        for key,value in (("monotonic",time.monotonic()-1.),("arm_id","wrong"),
                          ("owner_source_sha256",{"fake":"c"*64})):
            before=state[key];state[key]=value
            assert bridge._gripper_feedback_state() is None
            state[key]=before
    finally:
        _close_action_bridge(bridge,node,executor)
