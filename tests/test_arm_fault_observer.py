"""Fault cleanup keeps passive owner telemetry available without motion authority."""
import json
import os
import select

import pytest

from test_arm_session import make_session, target
from workshop.station.arm_session import JOINTS, ArmPlanPreflightError


def test_stalled_home_keeps_fresh_passive_fault_observation_and_stop_cannot_arm(tmp_path):
    session, arm, clock = make_session(tmp_path, initial=target(elbow_flex=3066), recover_sag=True)
    session.initialize()
    arm.bus.stalled_joints.add('elbow_flex')
    session.idle_timeout = .1
    reader, writer = os.pipe()
    emitted, snapshots = [], []
    commands = [
        {'command': name} for name in ('home', 'move_plan', 'move_gripper', 'teleop_start',
            'teleop_step', 'teleop_stop', 'native_damping', 'native_damping_renew',
            'native_damping_inventory', 'rearm_motion_profile', 'recover_camera',
            'commission_gripper_step', 'commission_gripper_held_step', 'acknowledge_held_jaw',
            'joint_diagnostics', 'jaw_diagnostics')]
    commands += [{'command': 'read_state'}, {'command': 'stop'}, {'command': 'home'},
                 {'command': 'read_state'}, {'command': 'exit'}]
    started = False
    fault_writes = None
    fault_goals = None

    def wait(readers, writers, errors, timeout):
        nonlocal started, fault_writes, fault_goals
        clock.sleep(timeout)
        if not started:
            started = True
            os.write(writer, b'{"command":"home"}\n')
        elif session._fault_observation is not None:
            snapshots.append(json.loads(session.state_file.read_text()))
            if fault_writes is None:
                fault_writes = list(arm.bus.writes)
                fault_goals = list(arm.bus.goal_history)
            assert arm.bus.writes == fault_writes
            assert arm.bus.goal_history == fault_goals
            assert set(arm.bus.data['Torque_Enable'].values()) == {0}
            arm.bus.data['Present_Position']['wrist_roll'] += 1
            os.write(writer, (json.dumps(commands.pop(0)) + '\n').encode())
        return select.select(readers, writers, errors, 0)

    try:
        assert session.serve(fd=reader, emit=emitted.append, wait=wait) == 0
    finally:
        os.close(reader)
        os.close(writer)
        session.close_log()
    assert len(fault_goals) == 1
    assert snapshots[-1]['monotonic'] > snapshots[0]['monotonic']
    assert snapshots[-1]['raw']['wrist_roll'] > snapshots[0]['raw']['wrist_roll']
    assert all(row['fault_observation'] == snapshots[0]['fault_observation'] for row in snapshots)
    assert all(row['write_scope'] == 'fault_observation' for row in snapshots)
    assert not session._home_completed_this_session
    assert any(row.get('stopped') for row in emitted)
    assert sum(row.get('error_type') == 'ArmPlanPreflightError' for row in emitted) == 17
    assert sum('6 second deadline' in row.get('error', '') for row in emitted) == 1
    with pytest.raises(TypeError):
        session._fault_observation['error'] = 'cleared'
    with pytest.raises(ArmPlanPreflightError, match='reconnect'):
        session._ensure_profile()


def test_fault_discards_buffered_motion_and_requires_new_owner(tmp_path, monkeypatch):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    def fail():
        raise RuntimeError('synthetic fault')
    monkeypatch.setattr(session, 'home', fail)
    reader, writer = os.pipe()
    emitted = []
    stage = 0
    def wait(readers, writers, errors, timeout):
        nonlocal stage
        clock.sleep(timeout)
        if stage == 0:
            os.write(writer, b'{"command":"home"}\n{"command":"move_plan","legs":[]}\n')
        else:
            assert session._fault_observation is not None
            assert not session._input_buffer
            os.write(writer, b'{"command":"exit"}\n')
        stage += 1
        return select.select(readers, writers, errors, 0)
    try:
        assert session.serve(fd=reader, emit=emitted.append, wait=wait) == 0
    finally:
        os.close(reader); os.close(writer); session.close_log()
    assert not arm.bus.goal_history
    assert len(emitted) == 2  # fault reply and explicit exit; queued motion was discarded
    new, _, _ = make_session(tmp_path)
    new.initialize()
    assert new._fault_observation is None and not new._home_completed_this_session
    new.close_log()


def test_failed_cleanup_remains_fatal_and_never_claims_fault_observation(tmp_path, monkeypatch):
    session, _, clock = make_session(tmp_path)
    session.initialize()
    monkeypatch.setattr(session, 'home', lambda: (_ for _ in ()).throw(RuntimeError('motion fault')))
    monkeypatch.setattr(session, 'shutdown', lambda **kw: (_ for _ in ()).throw(RuntimeError('cleanup failed')))
    reader, writer = os.pipe()
    os.write(writer, b'{"command":"home"}\n')
    try:
        with pytest.raises(RuntimeError, match='cleanup failed'):
            session.serve(fd=reader, emit=lambda reply: None,
                          wait=lambda *args: select.select(*args))
        assert session._fault_observation is None
    finally:
        os.close(reader); os.close(writer); session.close_log()


def test_oversized_command_cannot_amplify_immutable_fault_telemetry(tmp_path):
    session, arm, clock = make_session(tmp_path)
    normal = session.initialize()
    assert normal['fault_observation'] is None
    reader, writer = os.pipe()
    huge = 'x' * 900_000
    # Prebuffer the complete request without filling an OS pipe. The request is
    # below the existing input ceiling, and fails ordinary command validation.
    stage = 0
    fault_samples = []
    def wait(readers, writers, errors, timeout):
        nonlocal stage
        clock.sleep(timeout)
        if stage == 0:
            os.write(writer, b'{"command":"read_state"}\n')
        elif stage == 1:
            session._input_buffer.extend((json.dumps({'command': huge}) + '\n').encode())
            os.write(writer, b'\n')
        else:
            state = json.loads(session.state_file.read_text())
            fault_samples.append(state)
            assert len(json.dumps(state['fault_observation'])) < 1024
            assert len(state['fault_observation']['command']) <= 128
            assert len(state['fault_observation']['error']) <= 512
            assert len(state['fault_observation']['error_type']) <= 128
            os.write(writer, b'{"command":"exit"}\n' if stage == 3 else b'{"command":"read_state"}\n')
        stage += 1
        return select.select(readers, writers, errors, 0)
    try:
        assert session.serve(fd=reader, wait=wait, emit=lambda reply: None) == 0
    finally:
        os.close(reader); os.close(writer); session.close_log()
    assert len(fault_samples) == 2
    assert fault_samples[0]['fault_observation'] == fault_samples[1]['fault_observation']
    assert not arm.bus.goal_history


@pytest.mark.parametrize('safety', ['stop_before', 'stop_after', 'exit', 'exit_before', 'partial_exit', 'partial_stop', 'partial_motion_then_stop'])
def test_fault_boundary_preserves_queued_safety_ack_and_drops_actuation(tmp_path, monkeypatch, safety):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    request_id = 'a' * 32
    stop = (json.dumps({'command': 'stop', 'request_id': request_id}) + '\n').encode()
    motion = b'{"command":"move_plan","legs":[]}\n'
    exit_line = b'{"command":"exit"}\n'
    if safety == 'stop_before': tail = stop + motion
    elif safety == 'stop_after': tail = motion + stop
    elif safety == 'exit': tail = motion + exit_line
    elif safety == 'exit_before': tail = exit_line + motion
    elif safety == 'partial_exit': tail = motion + exit_line[:8]
    elif safety == 'partial_stop': tail = motion + stop[:30]
    else: tail = motion[:20]
    def fail():
        # The fault happens after the final busy-input poll. Commands arriving
        # at this boundary have not been acknowledged by the busy handler.
        session._input_buffer.extend(tail)
        raise RuntimeError('fault at final poll boundary')
    monkeypatch.setattr(session, 'home', fail)
    reader, writer = os.pipe()
    emitted = []
    stage = 0
    fault_writes = None
    def wait(readers, writers, errors, timeout):
        nonlocal stage, fault_writes
        clock.sleep(timeout)
        if stage == 0:
            os.write(writer, b'{"command":"home"}\n')
        elif stage == 1 and safety == 'partial_exit':
            fault_writes = list(arm.bus.writes)
            os.write(writer, exit_line[8:])
        elif stage == 1 and safety == 'partial_stop':
            fault_writes = list(arm.bus.writes)
            os.write(writer, stop[30:])
        elif stage == 1 and safety == 'partial_motion_then_stop':
            fault_writes = list(arm.bus.writes)
            os.write(writer, motion[20:] + stop)
        else:
            fault_writes = list(arm.bus.writes) if fault_writes is None else fault_writes
            assert arm.bus.writes == fault_writes
            os.write(writer, exit_line)
        stage += 1
        return select.select(readers, writers, errors, 0)
    try:
        assert session.serve(fd=reader, wait=wait, emit=emitted.append) == 0
    finally:
        os.close(reader); os.close(writer); session.close_log()
    assert session._fault_observation is not None
    assert not session._home_completed_this_session and not arm.bus.goal_history
    if safety not in ('exit', 'exit_before', 'partial_exit'):
        replies = [row for row in emitted if row.get('request_id') == request_id]
        assert len(replies) == 1 and replies[0]['stopped'] and replies[0]['ok']
        state = replies[0]['state']
        assert state['fault_observation'] == dict(session._fault_observation)
        assert set(state['torque'].values()) == {0}
        assert 0 <= clock.monotonic() - state['monotonic'] <= .1
    assert emitted[-1].get('exit') is True
    assert set(arm.bus.data['Torque_Enable'].values()) == {0}


@pytest.mark.parametrize('schema', ['failed-endpoint-hold-v1', 'failed-held-jaw-v1'])
def test_verified_off_fault_observer_retires_enabled_hold_health_bookkeeping(tmp_path, schema):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    arm.bus.data['Torque_Enable'] = dict.fromkeys(JOINTS, 1)
    anchor = dict(arm.bus.data['Present_Position'])
    session._failed_endpoint_hold = dict(schema=schema, held_raw=anchor)
    if schema == 'failed-held-jaw-v1':
        session._failed_endpoint_hold['body_anchor_raw'] = {j: anchor[j] for j in JOINTS[:-1]}
        session._failed_jaw_ack_context = json.dumps({'hold': session._failed_endpoint_hold})
    session._failed_hold_last_tick = clock.monotonic()
    reader, writer = os.pipe()
    snapshots = []
    stage = 0
    fault_writes = None
    def wait(readers, writers, errors, timeout):
        nonlocal stage, fault_writes
        clock.sleep(min(timeout, .01))
        if stage == 0:
            os.write(writer, b'{"invalid":"command"}\n')
        else:
            state = json.loads(session.state_file.read_text())
            snapshots.append(state)
            assert session._failed_endpoint_hold is None
            assert session._failed_hold_last_tick is None
            assert session._failed_jaw_ack_context is None
            assert state['failed_endpoint_hold'] is None
            assert state['fault_observation'] is not None
            assert set(state['torque'].values()) == {0}
            if fault_writes is None: fault_writes = list(arm.bus.writes)
            assert arm.bus.writes == fault_writes
            os.write(writer, b'{"command":"exit"}\n' if stage == 4 else b'{"command":"read_state"}\n')
        stage += 1
        return select.select(readers, writers, errors, 0)
    try:
        assert session.serve(fd=reader, wait=wait, emit=lambda reply: None) == 0
    finally:
        os.close(reader); os.close(writer); session.close_log()
    assert len(snapshots) == 4 and snapshots[-1]['monotonic'] > snapshots[0]['monotonic']
    assert all(state['fault_observation'] == snapshots[0]['fault_observation'] for state in snapshots)
    assert not arm.bus.goal_history and not session._home_completed_this_session
