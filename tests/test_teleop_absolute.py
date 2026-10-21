"""No hardware: current-pose activation and bounded absolute leader tracking."""
from types import SimpleNamespace

import pytest

from test_arm_session import make_session, target
from test_ros_teleop_mode_switch import bridge, enable, JOINTS


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


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
@pytest.mark.parametrize('gripper', [0., 100.])
def test_both_owner_modes_dispatch_mapped_targets_directly_with_identical_bounds(tmp_path, mode, gripper):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    state = session.handle({'command': 'teleop_start', 'session_id': 'direct', 'mode': mode})['state']
    initial = dict(state['raw'])
    assert arm.bus.goal_history == [initial]
    # Already mapped by the bridge: alternate body endpoints beyond the bounds.
    pose = {j: (1000. if i % 2 else -1000.) for i, j in enumerate(JOINTS)}
    pose['gripper'] = gripper
    expected = {j: session.admitted_max[j] if pose[j] > 0 else session.admitted_min[j] for j in JOINTS}
    clock.sleep(.01)
    session.handle({'command': 'teleop_step', 'session_id': 'direct', 'sequence': 0, 'pose': pose})
    assert arm.bus.goal_history == [initial, expected]
    assert not session._home_completed_this_session
    assert set(arm.bus.data['Acceleration'].values()) == {8}
    assert set(arm.bus.data['Goal_Velocity'].values()) == {0}
    assert arm.bus.data['Torque_Limit'] == {j: 250 if j == 'gripper' else 700 for j in JOINTS}
    assert not any(key.startswith('slew_') for key in session._teleop)
    # STOP holds the measured follower position, then rejects any further target.
    session.handle({'command': 'teleop_stop', 'session_id': 'direct'})
    assert arm.bus.goal_history[-1] == arm.bus.data['Present_Position']
    count = len(arm.bus.goal_history)
    with pytest.raises(ValueError, match='not active'):
        session.handle({'command': 'teleop_step', 'session_id': 'direct', 'sequence': 1, 'pose': pose})
    assert len(arm.bus.goal_history) == count
    session.shutdown(); session.close_log()


def test_owner_modes_send_identical_raw_positions_for_the_same_mapped_pose(tmp_path):
    histories = []
    for mode in ('relative', 'absolute'):
        directory = tmp_path / mode
        directory.mkdir()
        session, arm, clock = make_session(directory)
        session.initialize()
        session.handle({'command': 'teleop_start', 'session_id': mode, 'mode': mode})
        pose = {joint: (73.25 if joint == 'gripper' else (-1) ** index * 12.345)
                for index, joint in enumerate(JOINTS)}
        expected = {joint: int(round(1000 + pose[joint] * 20)) if joint == 'gripper' else
                    int(round((arm.calibration[joint]['range_min'] + arm.calibration[joint]['range_max']) / 2
                              + pose[joint] * 4095 / 360)) for joint in JOINTS}
        clock.sleep(.01)
        session.handle({'command': 'teleop_step', 'session_id': mode, 'sequence': 0, 'pose': pose})
        assert arm.bus.goal_history[-1] == expected
        histories.append(list(arm.bus.goal_history))
        session.shutdown(); session.close_log()
    assert histories[0] == histories[1]


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
def test_direct_owner_step_preserves_feedback_packet_budget(tmp_path, mode):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({'command': 'teleop_start', 'session_id': 'budget', 'mode': mode})
    before = dict(arm.bus.sync_read_calls)
    for sequence in range(10):
        clock.sleep(.025)
        session.handle({'command': 'teleop_step', 'session_id': 'budget', 'sequence': sequence,
                        'pose': {j: 75. if j == 'gripper' else 10. for j in JOINTS}})
    reads = {j: value - before.get(j, 0) for j, value in arm.bus.sync_read_calls.items()}
    assert reads['Present_Position'] == 10
    assert 2 <= reads['Goal_Position'] <= 3
    assert reads['Torque_Enable'] == reads['Goal_Position']
    assert sum(reads.values()) == 10 + 2 * reads['Goal_Position']
    session.shutdown(); session.close_log()


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


@pytest.mark.parametrize('mode', ['relative', 'absolute'])
def test_stale_owner_stream_holds_measured_pose_without_jumping_to_leader(tmp_path, mode):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({'command': 'teleop_start', 'session_id': 'stale', 'mode': mode})
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
