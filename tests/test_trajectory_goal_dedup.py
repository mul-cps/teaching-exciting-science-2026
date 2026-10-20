"""Goal deduplication never skips feedback or repairs an altered goal register."""
import pytest

from test_arm_session import make_session
from workshop.station.ros_trajectory import ARM_JOINTS, path_digest, prepare_trajectory


def trajectory(session, arm, *, delta=.06, duration=2., pan_delta=0.):
    raw = dict(arm.bus.data["Present_Position"])
    start = session._arm_radians(raw)
    end = dict(start, elbow_flex=start["elbow_flex"] + delta,
               shoulder_pan=start["shoulder_pan"] + pan_delta)
    points = [{"time": 0., "positions": [start[j] for j in ARM_JOINTS]},
              {"time": duration, "positions": [end[j] for j in ARM_JOINTS]}]
    prepared = prepare_trajectory(ARM_JOINTS, points, start_radians=start,
                                 max_velocity=.125, max_acceleration=.2, max_jerk=2.)
    command = dict(command="execute_trajectory", goal_id="dedup-test",
                   joint_names=list(ARM_JOINTS), points=points,
                   path_sha256=path_digest(prepared, held_gripper_raw=raw["gripper"]))
    targets = [session._trajectory_raw(sample.positions, raw["gripper"])
               for sample in prepared.samples]
    return command, prepared, targets


def test_duplicate_raw_goals_skip_writes_but_keep_all_feedback(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    command, prepared, targets = trajectory(session, arm)
    before = dict(arm.bus.sync_read_calls)
    result = session.handle(command)
    expected = [dict.fromkeys(arm.bus.data["Present_Position"], 2000)]
    for target in targets:
        if target != expected[-1]:
            expected.append(target)
    assert len(expected) < len(targets)
    assert arm.bus.goal_history == expected
    assert arm.bus.goal_write_payloads[0] == expected[0]
    assert all(set(payload) == {"elbow_flex"} for payload in arm.bus.goal_write_payloads[1:])
    assert all(arm.bus.goal_write_payloads)  # Never dispatch an empty subset.
    feedback = result["trajectory"]["feedback_count"]
    assert feedback == len(prepared.samples) + 3
    assert arm.bus.sync_read_calls["Goal_Position"] - before.get("Goal_Position", 0) == feedback + 1
    # Start acquisition and enable checks are additional to every feedback tick.
    assert arm.bus.sync_read_calls["Present_Position"] - before.get("Present_Position", 0) >= feedback + 1
    assert arm.bus.sync_read_calls["Torque_Enable"] - before.get("Torque_Enable", 0) >= feedback
    assert set(arm.bus.data["Torque_Enable"].values()) == {1}
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_profile_reset_on_goal_write_can_settle_without_resubmission(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    command, _, targets = trajectory(session, arm)
    original_write, original_read = arm.bus.sync_write, arm.bus.sync_read
    ready_at = 0.
    def write(register, values, **kwargs):
        nonlocal ready_at
        if register == "Goal_Position":
            ready_at = clock.value + .06
        return original_write(register, values, **kwargs)
    def read(register, **kwargs):
        # Hypothetical servo: a write restarts its 60 ms motion preparation.
        was_stalled = set(arm.bus.stalled_joints)
        if register == "Present_Position" and clock.value < ready_at:
            arm.bus.stalled_joints.update(ARM_JOINTS)
        try:
            return original_read(register, **kwargs)
        finally:
            arm.bus.stalled_joints = was_stalled
    arm.bus.sync_write, arm.bus.sync_read = write, read
    result = session.handle(command)
    assert result["ok"] is True
    assert "status" not in result
    assert arm.bus.data["Present_Position"] == targets[-1]
    assert sum(goal == targets[-1] for goal in arm.bus.goal_history) == 1
    session.shutdown(reason="test_cleanup")
    session.close_log()


@pytest.mark.parametrize("phase", ["duplicate_tick", "settle"])
def test_altered_goal_register_fails_without_resubmission(tmp_path, phase):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    command, prepared, targets = trajectory(session, arm, delta=0., duration=.2)
    original_publish = session.publish_state
    injected = False
    writes_at_corruption = None
    def publish(**kwargs):
        nonlocal injected, writes_at_corruption
        state = original_publish(**kwargs)
        inject = kwargs.get("event") == "trajectory_feedback" and (
            phase == "duplicate_tick" or kwargs["trajectory_time"] == prepared.samples[-1].time)
        if inject and not injected:
            injected = True
            writes_at_corruption = len(arm.bus.goal_history)
            arm.bus.data["Goal_Position"]["elbow_flex"] += 5
        return state
    session.publish_state = publish
    with pytest.raises(RuntimeError, match="goal readback mismatch"):
        session.handle(command)
    assert injected
    assert len(arm.bus.goal_history) == writes_at_corruption
    assert set(arm.bus.data["Torque_Enable"].values()) == {0}
    assert session._failed_endpoint_hold is None
    session.close_log()


@pytest.mark.parametrize("phase", ["duplicate_tick", "settle"])
@pytest.mark.parametrize("fault", ["torque", "stale_goal_read"])
def test_feedback_guards_remain_active_without_goal_writes(tmp_path, phase, fault):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    command, prepared, _ = trajectory(session, arm, delta=0., duration=.2)
    original_publish, original_read = session.publish_state, arm.bus.sync_read
    injected = False
    delay_next_goal_read = False
    def publish(**kwargs):
        nonlocal injected, delay_next_goal_read
        state = original_publish(**kwargs)
        inject = kwargs.get("event") == "trajectory_feedback" and (
            phase == "duplicate_tick" or kwargs["trajectory_time"] == prepared.samples[-1].time)
        if inject and not injected:
            injected = True
            if fault == "torque":
                arm.bus.data["Torque_Enable"]["gripper"] = 0
            else:
                delay_next_goal_read = True
        return state
    def read(register, **kwargs):
        nonlocal delay_next_goal_read
        if register == "Goal_Position" and delay_next_goal_read:
            delay_next_goal_read = False
            clock.sleep(.3)
        return original_read(register, **kwargs)
    session.publish_state, arm.bus.sync_read = publish, read
    message = "torque state changed" if fault == "torque" else "feedback freshness"
    with pytest.raises(RuntimeError, match=message):
        session.handle(command)
    assert injected
    assert len(arm.bus.goal_write_payloads) == 1  # Initial full-six enable only.
    assert set(arm.bus.data["Torque_Enable"].values()) == {0}
    session.close_log()



def test_quantized_multiaxis_targets_write_exactly_changed_joints(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    command, _, targets = trajectory(session, arm, pan_delta=.035)
    initial = dict(arm.bus.data["Present_Position"])
    result = session.handle(command)
    assert result["ok"] is True
    expected_payloads, expected_states = [initial], [initial]
    previous = initial
    for target in targets:
        changed = {joint: value for joint, value in target.items()
                   if value != previous[joint]}
        if changed:
            expected_payloads.append(changed)
            expected_states.append(target)
        previous = target
    assert arm.bus.goal_write_payloads == expected_payloads
    assert arm.bus.goal_history == expected_states
    masks = {frozenset(payload) for payload in expected_payloads[1:]}
    assert frozenset({"elbow_flex"}) in masks
    assert frozenset({"shoulder_pan"}) in masks
    assert frozenset({"elbow_flex", "shoulder_pan"}) in masks
    reconstructed = dict(initial)
    for payload, expected in zip(arm.bus.goal_write_payloads, expected_states):
        reconstructed.update(payload)
        assert reconstructed == expected
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_trajectory_loop_static_guards_preserve_feedback_only_settle():
    import ast
    import inspect
    import textwrap
    from workshop.station.arm_session import ArmSession
    tree = ast.parse(textwrap.dedent(inspect.getsource(ArmSession.execute_trajectory)))
    schedule = next(node for node in ast.walk(tree) if isinstance(node, ast.For)
                    and isinstance(node.target, ast.Tuple)
                    and isinstance(node.target.elts[0], ast.Name)
                    and node.target.elts[0].id == "index"
                    and any(isinstance(item, ast.Name) and item.id == "last_verified_goal"
                            for item in ast.walk(node)))
    settle = next(node for node in ast.walk(tree) if isinstance(node, ast.While))
    for loop in (schedule, settle):
        reads = set()
        for node in ast.walk(loop):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                assert node.func.attr not in {"write", "_ensure_profile", "ping"}
                if node.func.attr == "_read":
                    reads.add(node.args[0].value)
        assert reads == {"Goal_Position", "Present_Position", "Torque_Enable"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and node.func.attr == "sync_write" for node in ast.walk(settle))
    writes = [node for node in ast.walk(schedule) if isinstance(node, ast.Call)
              and isinstance(node.func, ast.Attribute) and node.func.attr == "sync_write"]
    assert len(writes) == 1
    write = writes[0]
    assert write.args[0].value == "Goal_Position"
    assert isinstance(write.args[1], ast.Name) and write.args[1].id == "changed"
    assert any(isinstance(node, ast.If) and isinstance(node.test, ast.Name)
               and node.test.id == "changed" and write in list(ast.walk(node))
               for node in ast.walk(schedule))
