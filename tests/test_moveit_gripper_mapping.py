"""The MoveIt revolute jaw interface is distinct from the metric jaw action."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control"))
from so101_station_control.gripper_action import (  # noqa: E402
    joint_to_percent, percent_to_joint, validate_joint_calibration,
)

LIMITS = (math.radians(-10), math.radians(100))
CALIBRATION = dict(schema="so101-gripper-joint-calibration-v1", qualified=True,
    closed_joint_rad=-.1, open_joint_rad=1.5, closed_percent=5., open_percent=95.,
    arm_id="fake-arm", calibration_sha256="a"*64)


def test_continuous_joint_angle_mapping_does_not_treat_radians_as_gap_metres():
    c = validate_joint_calibration(CALIBRATION, LIMITS)
    assert joint_to_percent(.7, c) == pytest.approx(50.)
    assert percent_to_joint(50., c) == pytest.approx(.7)
    assert joint_to_percent(1.5, c) == 95.


@pytest.mark.parametrize("patch,reason", [
    ({"qualified": False}, "qualified"),
    ({"open_joint_rad": 1.8}, "model bounds"),
    ({"calibration_sha256": ""}, "identity"),
    ({"closed_joint_rad": float("nan")}, "finite"),
])
def test_unqualified_or_outside_model_endpoints_fail_closed(patch, reason):
    with pytest.raises(ValueError, match=reason):
        validate_joint_calibration({**CALIBRATION, **patch}, LIMITS)


def test_observations_are_not_clamped_into_qualified_range():
    c = validate_joint_calibration(CALIBRATION, LIMITS)
    with pytest.raises(ValueError, match="outside"):
        percent_to_joint(100., c)
    with pytest.raises(ValueError, match="outside"):
        joint_to_percent(-.15, c)


def test_owner_start_binding_rejects_changed_pose_or_session_before_writes(tmp_path):
    import importlib.util
    support_path = Path(__file__).with_name("test_arm_session.py")
    spec = importlib.util.spec_from_file_location("jaw_binding_test_support", support_path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    session, arm, _clock = support.make_session(tmp_path)
    session.initialize()
    start = arm.bus.data["Present_Position"].copy()
    writes = list(arm.bus.writes)
    for expected, owner in (({**start, "gripper": start["gripper"]+1}, session._owner_session_id),
                            (start, "replaced-owner")):
        with pytest.raises(ValueError, match="start or owner session changed"):
            session.move_gripper(start["gripper"]+5, expected_start_raw=expected, owner_session_id=owner)
        assert arm.bus.writes == writes

    accepted = session.handle(dict(command="move_gripper", target_raw=start["gripper"]+5,
        expected_start_raw=start, owner_session_id=session._owner_session_id))
    assert accepted["ok"] is True
    assert arm.bus.goal_history[-1]["gripper"] == start["gripper"]+5


@pytest.mark.parametrize("closed,opened", [(-.1,1.5), (1.5,-.1)])
def test_fractional_percent_endpoints_quantize_inward(closed, opened):
    from so101_station_control.gripper_action import joint_target_raw
    c = {**CALIBRATION, "closed_percent":5.04, "open_percent":95.04,
         "closed_joint_rad":closed, "open_joint_rad":opened}
    assert joint_target_raw(closed,c,(1872,3366),(1872,3366)) == 1948
    assert joint_target_raw(opened,c,(1872,3366),(1872,3366)) == 3291
    for q in (closed,opened):
        raw = joint_target_raw(q,c,(1872,3366),(1872,3366))
        percent_to_joint(100.*(raw-1872)/1494,c)
    with pytest.raises(ValueError,match="outside calibrated"):
        joint_target_raw(min(closed,opened)-.0001,c,(1872,3366),(1872,3366))
    with pytest.raises(ValueError,match="admitted representable"):
        joint_target_raw(closed,c,(1872,3366),(2500,3000))


def test_quantization_handles_single_empty_and_exact_integer_intervals():
    from so101_station_control.gripper_action import joint_target_raw
    c = {**CALIBRATION,"closed_percent":5.04,"open_percent":5.1}
    assert joint_target_raw(-.1,c,(1872,3366),(1872,3366)) == 1948
    with pytest.raises(ValueError,match="no admitted integer"):
        joint_target_raw(-.1,{**c,"open_percent":5.05},(1872,3366),(1872,3366))
    c={**CALIBRATION,"closed_percent":100.*75/1494,"open_percent":95.04}
    assert joint_target_raw(-.1,c,(1872,3366),(1872,3366)) == 1947


def test_bound_angular_jaw_disables_trim_and_stops_on_body_drift(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location("static_body_test_support",Path(__file__).with_name("test_arm_session.py"))
    support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
    session,arm,_clock=support.make_session(tmp_path)
    session.initialize()
    start=arm.bus.data["Present_Position"].copy()
    def drift():
        arm.bus.stalled_joints.add("shoulder_lift")
        arm.bus.data["Present_Position"]["shoulder_lift"]=start["shoulder_lift"]-20
    arm.bus.on_goal_write=drift
    with pytest.raises(RuntimeError,match="held body drift exceeds 2"):
        session.move_gripper(2100,expected_start_raw=start,owner_session_id=session._owner_session_id)
    assert all(all(g[j]==start[j] for j in support.JOINTS[:-1]) for g in arm.bus.goal_history)
    assert set(arm.bus.data["Torque_Enable"].values()) == {0}


def test_bound_contact_hold_preserves_certified_body_goals(tmp_path):
    import importlib.util
    spec=importlib.util.spec_from_file_location("static_contact_test_support",Path(__file__).with_name("test_arm_session.py"))
    support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
    session,arm,_clock=support.make_session(tmp_path)
    session.initialize()
    start=arm.bus.data["Present_Position"].copy()
    arm.bus.blocked_gripper=True
    def slight_drift():
        arm.bus.stalled_joints.add("shoulder_lift")
        arm.bus.data["Present_Position"]["shoulder_lift"]=start["shoulder_lift"]-2
    arm.bus.on_goal_write=slight_drift
    completed=session.move_gripper(start["gripper"]-100,expected_start_raw=start,owner_session_id=session._owner_session_id)
    assert completed["legs"][-1]["contact_inferred"]
    assert all(all(g[j]==start[j] for j in support.JOINTS[:-1]) for g in arm.bus.goal_history)
    assert arm.bus.data["Goal_Position"]["gripper"]==start["gripper"]


def test_gripper_feedback_hotpath_has_no_snapshot_or_json_io():
    import ast
    source=Path(__file__).resolve().parents[1]/"ros2/src/so101_station_control/so101_station_control/trajectory_server.py"
    methods={n.name:n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.FunctionDef)}
    forbidden={"self._read_state","read_text","read_bytes","open","json.loads","json.load"}
    getter_calls={ast.unparse(n.func) for n in ast.walk(methods["_gripper_feedback_state"]) if isinstance(n,ast.Call)}
    assert not getter_calls & forbidden
    for loop in (n for n in ast.walk(methods["_execute_gripper"]) if isinstance(n,ast.While)):
        calls={ast.unparse(n.func) for n in ast.walk(loop) if isinstance(n,ast.Call)}
        assert not calls & forbidden
