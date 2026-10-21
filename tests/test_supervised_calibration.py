import pytest
from workshop.station import ros_trajectory as rt
from test_arm_session import make_session


def target(session, arm):
    raw = dict(arm.bus.data['Present_Position'])
    end = dict(raw, elbow_flex=raw['elbow_flex']-114)
    return raw, end


def test_supervised_helper_slow_held_path(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    start, end = target(session, arm)
    points, prepared = rt.prepare_supervised_calibration(start, end,
        session._arm_radians(start), session.admitted_min, session.admitted_max)
    assert prepared.duration >= 13
    assert all(s.positions[0] == prepared.samples[0].positions[0] for s in prepared.samples)
    result = session.execute_trajectory('supervised', rt.ARM_JOINTS, points,
        expected_path_sha256=rt.path_digest(prepared, held_gripper_raw=start['gripper']),
        supervised_commissioning=True, expected_start_raw=start)
    assert result['trajectory']['qualification'] == rt.SUPERVISED_QUALIFICATION
    assert result['trajectory']['qualified'] is False


def test_supervised_helper_rejects_jaw_change(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    start, end = target(session, arm)
    end['gripper'] += 1
    with pytest.raises(ValueError, match='gripper'):
        rt.prepare_supervised_calibration(start, end, session._arm_radians(start),
                                         session.admitted_min, session.admitted_max)


def command(session, arm):
    start, end = target(session, arm)
    points, prepared = rt.prepare_supervised_calibration(start, end,
        session._arm_radians(start), session.admitted_min, session.admitted_max)
    return dict(command='execute_trajectory', goal_id='test', joint_names=list(rt.ARM_JOINTS),
                points=points, path_sha256=rt.path_digest(prepared, held_gripper_raw=start['gripper']),
                supervised_commissioning=True, expected_start_raw=start,
                ros_motion_profile=session._ros_motion_profile.name)


@pytest.mark.parametrize('bad', [True, 2000.0, -1, 4096, None])
def test_bad_raw_target_rejected(tmp_path, bad):
    session, arm, _ = make_session(tmp_path); session.initialize()
    start, end = target(session, arm); end['wrist_roll'] = bad
    with pytest.raises(ValueError):
        rt.prepare_supervised_calibration(start, end, session._arm_radians(start),
                                         session.admitted_min, session.admitted_max)
    assert not arm.bus.goal_history


@pytest.mark.parametrize('failure', ['drift', 'torque', 'cancel_hold', 'source', 'source_stall'])
def test_final_predispatch_guard_has_no_goal_or_enable(tmp_path, monkeypatch, failure):
    from workshop.station import arm_session as module
    session, arm, _ = make_session(tmp_path); session.initialize()
    request = command(session, arm)
    original = session._ensure_profile
    def profile():
        original()
        if failure == 'drift': arm.bus.data['Present_Position']['shoulder_pan'] += 1
        if failure == 'torque': arm.bus.data['Torque_Enable']['gripper'] = 1
        if failure == 'cancel_hold':
            session._cancel_requested = True
            session._cancel_reason = 'trajectory_cancel_hold'
        if failure == 'source': monkeypatch.setattr(module, 'owner_source_sha256', lambda path: {})
        if failure == 'source_stall':
            def stalled(path):
                session.sleep(7.)
                return module.RUNTIME_OWNER_SOURCE_SHA256
            monkeypatch.setattr(module, 'owner_source_sha256', stalled)
    monkeypatch.setattr(session, '_ensure_profile', profile)
    with pytest.raises(Exception): session.handle(request)
    assert not arm.bus.goal_history
    assert not any(reg == 'Torque_Enable' and value == 1 for reg, joint, value in arm.bus.writes)


@pytest.mark.parametrize('joint', ['gripper', 'shoulder_pan', 'wrist_roll'])
def test_held_axis_drift_stops_without_trim(tmp_path, joint):
    session, arm, _ = make_session(tmp_path); session.initialize()
    request = command(session, arm)
    def drift():
        arm.bus.data['Present_Position'][joint] += 5
        arm.bus.stalled_joints.add(joint)
    arm.bus.on_goal_write = drift
    with pytest.raises(RuntimeError, match='held axis drift'): session.handle(request)
    assert all(goal[joint] == request['expected_start_raw'][joint] for goal in arm.bus.goal_history)
    assert all(v == 0 for v in arm.bus.data['Torque_Enable'].values())


def test_supervised_rejects_endpoint_fallback_and_nonboolean_flag(tmp_path):
    session, arm, _ = make_session(tmp_path); session.initialize()
    request = command(session, arm)
    with pytest.raises(ValueError): session.handle(dict(request, supervised_commissioning=1))
    with pytest.raises(ValueError): session.handle(dict(request, calibration_endpoint=True))
    assert not arm.bus.goal_history


@pytest.mark.parametrize('failure', ['identity', 'bounds'])
def test_home_reload_rejects_before_goal_preserving_cache(tmp_path, failure):
    import json
    from pathlib import Path
    session, arm, _ = make_session(tmp_path); session.initialize()
    old_pose, old_raw = dict(session.home_pose), dict(session.home_raw)
    path = Path(arm.settings['home_file']); home = json.loads(path.read_text())
    if failure == 'identity': home['arm_id'] = 'other'
    else: home['position']['elbow_flex'] = 180
    path.write_text(json.dumps(home))
    with pytest.raises(ValueError): session.home()
    assert session.home_pose == old_pose and session.home_raw == old_raw
    assert not arm.bus.goal_history


def test_home_reload_adopts_new_valid_measured_pose(tmp_path):
    import json
    from pathlib import Path
    session, arm, _ = make_session(tmp_path); session.initialize()
    path = Path(arm.settings['home_file']); home = json.loads(path.read_text())
    home['position']['elbow_flex'] -= 1
    path.write_text(json.dumps(home))
    session.home()
    assert session.home_pose['elbow_flex'] == home['position']['elbow_flex']
    assert session.home_raw == arm._raw_goal(session.home_pose)


def test_supervised_completion_does_not_mint_fresh_clocks(tmp_path, monkeypatch):
    session, arm, clock=make_session(tmp_path); session.initialize()
    request=command(session,arm)
    publish=session.publish_state
    def delayed(**kwargs):
        value=publish(**kwargs)
        if kwargs.get('event')=='trajectory_settle': clock.sleep(.15)
        return value
    monkeypatch.setattr(session,'publish_state',delayed)
    with pytest.raises(RuntimeError,match='stale'): session.handle(request)
    assert all(v==0 for v in arm.bus.data['Torque_Enable'].values())


def test_supervised_active_tracking_stall_fails_no_retry(tmp_path):
    session,arm,_=make_session(tmp_path); session.initialize()
    request=command(session,arm); arm.bus.stalled_joints.add('elbow_flex')
    with pytest.raises(RuntimeError,match='tracking error'): session.handle(request)
    assert all(v==0 for v in arm.bus.data['Torque_Enable'].values())
    assert all(goal['gripper']==2000 for goal in arm.bus.goal_history)


def test_supervised_cached_guard_rejects_all_six_bounds(tmp_path):
    session,arm,_=make_session(tmp_path); session.initialize()
    for joint in (*rt.ARM_JOINTS,'gripper'):
        actual=dict(arm.bus.data['Present_Position']);actual[joint]=session.admitted_max[joint]+1
        with pytest.raises(RuntimeError,match='admitted bounds'):
            session._guard_supervised_actual(actual,{})


def test_supervised_tick_guard_has_no_io_or_profile_operations():
    import ast
    import inspect
    import textwrap
    from workshop.station.arm_session import ArmSession
    tree=ast.parse(textwrap.dedent(inspect.getsource(ArmSession._guard_supervised_actual)))
    calls={ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)}
    assert calls <= {'type', 'abs', 'RuntimeError'}
    assert len([node for node in ast.walk(tree) if isinstance(node,ast.For)]) == 1


def test_actual_91_degree_elbow_114_count_extension_keeps_other_five(tmp_path):
    from test_ros_elbow_bounds import owner_at_actual_elbow
    session,arm,_=owner_at_actual_elbow(tmp_path);session.initialize()
    request=command(session,arm)
    result=session.handle(request)
    assert result['ok'] and result['trajectory']['qualified'] is False
    assert request['expected_start_raw']['elbow_flex']==2993
    for goal in arm.bus.goal_history:
        for joint in (*rt.ARM_JOINTS,'gripper'):
            if joint!='elbow_flex': assert goal[joint]==request['expected_start_raw'][joint]
    assert abs(arm.bus.data['Present_Position']['elbow_flex']-2879)<=1
