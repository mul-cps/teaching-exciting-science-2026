"""Source-bound bounded reentry policy; synthetic feedback, never hardware."""
import pytest
from test_arm_session import make_session, target
from workshop.station.arm_session import JOINTS
import workshop.station.arm_session as owner


def real_edge_session(tmp_path, raw):
    session, arm, clock = make_session(tmp_path, initial=target(elbow_flex=raw), recover_sag=True)
    # Source station's admitted max3052, firmware max3055, represented by the
    # synthetic arm's own unchanged calibration/configured-limit intersection.
    arm.settings['limits']['elbow_flex']['max'] = (3052.25 - 2027.5) * 360 / 4095
    return session, arm, clock


def test_current_3069_start_targets3020_with_49count_first_step_and_fixed_other_goals(tmp_path):
    session, arm, _ = real_edge_session(tmp_path, 3069)
    session.initialize()
    assert session.admitted_max['elbow_flex'] == 3052
    session.home()
    first = arm.bus.goal_history[0]
    assert first['elbow_flex'] == 3020
    assert 3069 - first['elbow_flex'] == 49
    assert all(first[j] == 2000 for j in JOINTS if j != 'elbow_flex')
    assert session._home_completed_this_session
    session.shutdown(); session.close_log()


@pytest.mark.parametrize('overrun', [16, 17])
def test_firmware_startup_allowance_remains16_and_transition_cannot_bypass_it(tmp_path, overrun):
    session, arm, _ = real_edge_session(tmp_path, 3055 + overrun)
    if overrun == 17:
        with pytest.raises(ValueError, match='outside firmware'):
            session.initialize()
        assert not arm.bus.goal_history and set(arm.bus.data['Torque_Enable'].values()) == {0}
    else:
        session.initialize()
        assert session.home()['arm_bounds']['classification'] == 'normal'
        start = target(elbow_flex=3055 + 17)
        goal = target(elbow_flex=3020)
        with pytest.raises(ValueError, match='outside firmware sag allowance'):
            session._validate_transition(start, goal, 'home_sag_recovery', allow_sag=True)
        session.shutdown()
    session.close_log()


@pytest.mark.parametrize('inset', [16, 32])
def test_synthetic25count_static_bias_needs_larger_inset_for_measured_reentry(tmp_path, monkeypatch, inset):
    session, arm, _ = real_edge_session(tmp_path, 3066)
    session.initialize()
    monkeypatch.setattr(owner, 'SAG_INSET_COUNTS', inset)
    start = dict(arm.bus.data['Present_Position'])
    goal = dict(start, elbow_flex=3052-inset)
    arm.bus.stalled_joints.add('elbow_flex')
    def bias():
        arm.bus.data['Present_Position']['elbow_flex'] = goal['elbow_flex'] + 25
    arm.bus.on_present_read = bias
    session._ensure_profile()
    if inset == 16:
        assert goal['elbow_flex'] + 25 == 3061 > session.admitted_max['elbow_flex']
        with pytest.raises(TimeoutError, match='6 second deadline'):
            session._run_leg('home_sag_recovery', goal, start, allow_sag=True)
    else:
        record = session._run_leg('home_sag_recovery', goal, start, allow_sag=True)
        assert record['settled_raw']['elbow_flex'] == 3045 <= session.admitted_max['elbow_flex']
        assert record['settlement_purpose'] == 'admitted_reentry'
        assert not record['target_reached'] and not record['grasp_confirmed']
        assert record['gravity_trim_raw'] == dict.fromkeys(JOINTS,0)
    assert arm.bus.goal_history == [goal]
    assert all(goal[j] == start[j] for j in JOINTS if j != 'elbow_flex')
    session.shutdown()
    assert set(arm.bus.data['Torque_Enable'].values()) == {0}
    session.close_log()


def test_recovery_budget64_and_actual_guard66_are_separate_from_ordinary_motion(tmp_path):
    session, arm, clock = make_session(tmp_path, recover_sag=True)
    arm.settings['limits']['elbow_flex'].update(min=-70., max=80.)
    session.initialize()
    high = session.admitted_max['elbow_flex']
    start = target(elbow_flex=high+32)
    goal = target(elbow_flex=high-32)
    recovery = session._validate_transition(start,goal,'home_sag_recovery',allow_sag=True)
    assert start['elbow_flex'] - goal['elbow_flex'] == owner.SAG_STEP_COUNTS == 64
    for api in (lambda: session.move_plan([dict(label='ordinary',target_raw=goal)]),
                lambda: session.move_gripper(2200)):
        arm.bus.data['Present_Position'] = dict(start)
        writes = list(arm.bus.writes)
        with pytest.raises(ValueError): api()
        assert arm.bus.writes == writes and not arm.bus.goal_history
    inside = dict(start,elbow_flex=start['elbow_flex']-66)
    session._guard_actual(inside,start,start,{'elbow_flex'},recovery,clock.monotonic())
    with pytest.raises(RuntimeError, match='excursion exceeds 66'):
        session._guard_actual(dict(start,elbow_flex=start['elbow_flex']-67),inside,start,
                              {'elbow_flex'},recovery,clock.monotonic())
    session.shutdown();session.close_log()
