"""No hardware: current-pose activation and bounded absolute leader tracking."""
import math
from types import SimpleNamespace

import pytest

from test_arm_session import make_session, target
from test_ros_teleop_mode_switch import bridge, enable, JOINTS
from workshop.station.arm_session import _teleop_slew_step


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
@pytest.mark.parametrize('torque', [0, 1])
def test_nonhome_activation_first_goal_is_exact_current_and_never_grants_home(tmp_path, mode, torque):
    session, arm, _ = make_session(tmp_path, initial=target(shoulder_pan=2140, wrist_roll=2110, gripper=2200))
    session.initialize()
    actual = dict(arm.bus.data['Present_Position'])
    arm.bus.data['Torque_Enable'] = dict.fromkeys(JOINTS, torque)
    response = session.handle({'command': 'teleop_start', 'session_id': 'no-home', 'mode': mode})
    assert arm.bus.goal_history == [actual]
    assert response['state']['raw'] == actual
    assert session._teleop['mode'] == mode
    assert session._home_completed_this_session is False
    session.shutdown(); session.close_log()


def test_absolute_six_joint_transition_respects_existing_limits_and_starts_at_rest(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    state = session.handle({'command': 'teleop_start', 'session_id': 'absolute', 'mode': 'absolute'})['state']
    initial = dict(state['raw'])
    pose = {j: state['pose'][j] + (20 if i % 2 else -20) for i, j in enumerate(JOINTS)}
    for sequence in range(300):
        active = session._teleop
        previous, velocity = dict(active['slew_position']), dict(active['slew_velocity'])
        clock.sleep(.02)
        session.handle({'command': 'teleop_step', 'session_id': 'absolute', 'sequence': sequence, 'pose': pose})
        for joint in JOINTS:
            low, high, vmax, amax = active['slew_limits'][joint]
            assert low <= active['slew_position'][joint] <= high
            assert abs(active['slew_velocity'][joint]) <= vmax + 1e-8
            assert abs(active['slew_velocity'][joint] - velocity[joint]) <= amax * .02 + 1e-8
            assert abs(active['slew_position'][joint] - previous[joint]) <= vmax * .02 + 1e-8
        if sequence == 0:
            assert arm.bus.goal_history[-1] == initial  # no initial absolute jump
    assert all(arm.bus.goal_history[-1][j] != initial[j] for j in JOINTS)
    expected = session.arm._raw_goal(pose)
    assert all(abs(arm.bus.goal_history[-1][j] - expected[j]) <= 1 for j in JOINTS)
    assert not session._home_completed_this_session
    assert set(arm.bus.data['Acceleration'].values()) == {8}
    assert set(arm.bus.data['Goal_Velocity'].values()) == {0}
    session.shutdown(); session.close_log()


def test_slew_brakes_at_both_bounds_and_handles_leader_reversals():
    position, velocity = dict.fromkeys(JOINTS, 50.), dict.fromkeys(JOINTS, 0.)
    limits = {j: (0., 100., 30., 20.) for j in JOINTS}
    for tick in range(2000):
        target = {j: float(100 if (tick // 137 + i) % 2 else 0) for i, j in enumerate(JOINTS)}
        following, speeds = _teleop_slew_step(position, velocity, target, limits, .02)
        for joint in JOINTS:
            assert 0 <= following[joint] <= 100
            assert abs(speeds[joint]) <= 30 + 1e-8
            assert abs(speeds[joint] - velocity[joint]) <= .4 + 1e-8
        position, velocity = following, speeds


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
def test_unsafe_current_pose_still_rejects_before_any_writes(tmp_path, mode):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.data['Present_Position']['elbow_flex'] = session.admitted_max['elbow_flex'] + 17
    writes = list(arm.bus.writes)
    with pytest.raises(ValueError, match='limits'):
        session.handle({'command': 'teleop_start', 'session_id': 'unsafe', 'mode': mode})
    assert arm.bus.writes == writes and not arm.bus.goal_history
    session.close_log()


def select(node, absolute):
    return node._set_teleop_absolute_mode(SimpleNamespace(data=absolute), SimpleNamespace(success=False, message=''))


def test_mode_selection_is_nonactuating_and_relative_reenable_reanchors(bridge):
    bridge._home_completed_owner_started = None
    assert select(bridge, True).success and bridge._teleop_mode == 'absolute'
    assert not bridge.commands
    assert select(bridge, False).success and bridge._teleop_mode == 'relative'
    assert not bridge.commands
    assert enable(bridge).success
    assert bridge.commands[0]['mode'] == 'relative'
    assert not select(bridge, True).success
    assert bridge._teleop_mode == 'relative'
    assert bridge._home_completed_owner_started is None


def test_absolute_bridge_maps_calibrated_angles_and_jaw_without_relative_offset(bridge):
    bridge._home_completed_owner_started = None
    assert select(bridge, True).success
    assert enable(bridge).success
    assert bridge.commands[0]['mode'] == 'absolute'
    bridge._teleop_latest.update(sequence=3, pose={j: float(i * 5) for i, j in enumerate(JOINTS)})
    bridge._teleop_tick()
    assert bridge.commands[-1]['pose'] == bridge._teleop_latest['pose']
    assert bridge._home_completed_owner_started is None


@pytest.mark.parametrize('fault', ['busy', 'active', 'stale', 'fault', 'stop', 'observe'])
def test_mode_selection_preserves_idle_fresh_owner_guards(bridge, fault):
    from test_ros_teleop_mode_switch import state
    if fault == 'busy': bridge._active_goal = 'home'
    if fault == 'active': bridge._teleop_state = 'active'
    if fault == 'stale': bridge._read_state = lambda: state(monotonic=0)
    if fault == 'fault': bridge._owner_faulted = True
    if fault == 'stop': bridge._stop_latched = True
    if fault == 'observe': bridge._reconnect_observe = True
    result = select(bridge, True)
    assert not result.success
    assert bridge._teleop_mode == 'relative' and not bridge.commands


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
@pytest.mark.parametrize('barrier', ['fault', 'stop', 'stale_leader'])
def test_active_modes_freeze_on_owner_fault_stop_and_stale_leader(bridge, mode, barrier):
    bridge._teleop_mode = mode
    assert enable(bridge).success
    sent = len(bridge.commands)
    bridge._teleop_latest['sequence'] += 1
    if barrier == 'fault': bridge._owner_faulted = True
    if barrier == 'stop': bridge._stop_latched = True
    if barrier == 'stale_leader': bridge._teleop_latest['stamp_s'] -= 1
    if barrier == 'stale_leader':
        bridge.get_logger = lambda: SimpleNamespace(error=lambda *args: None)
    bridge._teleop_tick()
    if barrier == 'stale_leader':
        assert bridge.commands[-1]['command'] == 'teleop_stop'
        assert bridge._teleop_state == 'stopping'
    else:
        assert len(bridge.commands) == sent


def test_absolute_stale_owner_stream_holds_measured_pose_without_jumping_to_leader(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({'command': 'teleop_start', 'session_id': 'stale', 'mode': 'absolute'})
    measured = dict(arm.bus.data['Present_Position'])
    clock.sleep(.31)
    with pytest.raises(ValueError, match='watchdog expired'):
        session.handle({'command': 'teleop_step', 'session_id': 'stale', 'sequence': 0,
                        'pose': dict.fromkeys(JOINTS, 90.)})
    assert arm.bus.goal_history[-1] == measured
    assert session._teleop is None and not session._home_completed_this_session
    session.shutdown(); session.close_log()


def test_disable_change_mode_and_reenable_capture_a_new_current_reference(bridge):
    from test_ros_teleop_mode_switch import state
    import time
    assert enable(bridge).success
    response = bridge._set_teleop_enabled(SimpleNamespace(data=False), SimpleNamespace(success=False, message=''))
    assert response.success and bridge.commands[-1]['command'] == 'teleop_stop'
    assert bridge._teleop_state == 'disabled'
    count = len(bridge.commands)
    assert select(bridge, True).success
    assert len(bridge.commands) == count
    def ack(key):
        bridge._teleop_latest = dict(stamp_s=time.monotonic(), sequence=10, pose=dict.fromkeys(JOINTS, 20.))
        return {'ok': True, 'state': state(pose=dict.fromkeys(JOINTS, 40.))}
    bridge._wait_teleop_ack = ack
    assert enable(bridge).success
    assert bridge.commands[-1]['mode'] == 'absolute'
    assert bridge._teleop_origin == dict.fromkeys(JOINTS, 20.)
    assert bridge._teleop_follower_start == dict.fromkeys(JOINTS, 40.)


@pytest.mark.parametrize('bad', ['unknown', True, None])
def test_owner_unknown_mode_is_rejected_before_any_goal_or_profile_write(tmp_path, bad):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    writes = list(arm.bus.writes)
    with pytest.raises(ValueError, match='relative or absolute'):
        session.handle({'command': 'teleop_start', 'session_id': 'bad-mode', 'mode': bad})
    assert arm.bus.writes == writes and not arm.bus.goal_history
    session.close_log()
