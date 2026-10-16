"""Hardware-free proof that bounded jaw commissioning never writes body servos."""
import pytest
from test_arm_session import make_session, target
from workshop.station.arm_session import ArmSessionCancelled


def setup(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    arm.bus.data['Present_Position']['elbow_flex'] = 3093
    arm.bus.data['Goal_Position']['elbow_flex'] = 3093
    arm.bus.writes.clear()
    return session, arm.bus, clock


def jaw_only(bus):
    assert bus.writes
    assert all(joint == 'gripper' for _, joint, _ in bus.writes)
    assert not bus.goal_history
    assert all(reg in {'Torque_Enable', 'Acceleration', 'Goal_Velocity', 'Torque_Limit', 'Goal_Position'} for reg, _, _ in bus.writes)


def test_component_success_and_later_shutdown_stays_scoped(tmp_path):
    session, bus, _ = setup(tmp_path)
    state = session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})['state']
    assert state['event'] == 'commission_gripper_complete'
    assert state['write_scope'] == 'gripper_only'
    assert state['component_command']['status'] == 'complete'
    assert state['component_command']['target_raw'] == 2008
    assert state['component_command']['profile_raw']['Goal_Velocity'] == 1
    assert not state['component_restore_pending']
    assert bus.data['Present_Position']['gripper'] == 2008
    assert not any(bus.data['Torque_Enable'].values())
    session.shutdown(reason='idle_timeout')
    session.handle({'command': 'stop'})
    session.handle({'command': 'exit'})
    jaw_only(bus)
    assert ('Goal_Velocity', 'gripper', 1) in bus.writes
    assert ('Acceleration', 'gripper', 1) in bus.writes
    assert ('Torque_Limit', 'gripper', 50) in bus.writes
    with pytest.raises(ValueError):
        session._ensure_profile()
    jaw_only(bus)


@pytest.mark.parametrize('delta', [0, 17, -17, True, 1.5])
def test_component_invalid_delta_no_writes(tmp_path, delta):
    session, bus, _ = setup(tmp_path)
    with pytest.raises(ValueError):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': delta})
    assert not bus.writes


@pytest.mark.parametrize('fault', ['timeout', 'cancel', 'drift', 'body_torque', 'read', 'disconnect', 'partial_write'])
def test_component_failure_cleanup_is_jaw_only(tmp_path, fault):
    session, bus, clock = setup(tmp_path)
    original_write = bus.write
    def write(reg, joint, value, **kwargs):
        original_write(reg, joint, value, **kwargs)
        if reg == 'Torque_Enable' and value == 1:
            if fault == 'cancel': session._cancel('test_cancel')
            if fault == 'drift': bus.data['Present_Position']['wrist_roll'] += 3
            if fault == 'body_torque': bus.data['Torque_Enable']['wrist_roll'] = 1
            if fault == 'read': bus.sync_read_failures['Present_Position'] = 100
            if fault == 'disconnect': bus.is_connected = False; raise ConnectionError('disconnected')
        if fault == 'partial_write' and reg == 'Goal_Velocity' and value == 1:
            raise ConnectionError('partial write')
    bus.write = write
    if fault == 'timeout': bus.blocked_gripper = True
    with pytest.raises((RuntimeError, TimeoutError, ConnectionError, ArmSessionCancelled)):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    try: session.shutdown(reason='later_global_shutdown')
    except RuntimeError: pass
    jaw_only(bus)
    assert clock.value < 3
    if fault == 'body_torque':
        assert bus.data['Torque_Enable']['wrist_roll'] == 1
        with pytest.raises(RuntimeError): session.handle({'command': 'stop'})


def test_component_scope_clears_only_on_clean_normal_admission(tmp_path):
    session, bus, _ = setup(tmp_path)
    session.handle({'command': 'commission_gripper_step', 'delta_counts': -8})
    assert session._component_scope
    bus.data['Present_Position']['elbow_flex'] = 2000
    session._ensure_profile()
    assert not session._component_scope
    assert any(joint != 'gripper' for _, joint, _ in bus.writes)


@pytest.mark.parametrize('action', ['stop', 'cancel_hold', 'exit', 'eof'])
def test_component_busy_owner_protocol_cleanup(tmp_path, action):
    import os
    session, bus, _ = setup(tmp_path)
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b'{"command":"commission_gripper_step","delta_counts":8}\n')
    sent = False
    def on_read():
        nonlocal sent
        if not sent and bus.data['Torque_Enable']['gripper'] == 1:
            sent = True
            if action == 'eof':
                os.close(write_fd)
            else:
                import json
                command = {'command': action}
                if action == 'stop': command['request_id'] = 'c' * 32
                os.write(write_fd, (json.dumps(command)+'\n').encode())
    bus.on_present_read = on_read
    replies = []
    def emit(reply):
        replies.append(reply)
        if reply.get('stopped') and action in {'stop', 'cancel_hold'}:
            os.write(write_fd, b'{"command":"exit"}\n')
    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
    finally:
        os.close(read_fd)
        if action != 'eof': os.close(write_fd)
    assert sent
    jaw_only(bus)
    assert not any(bus.data['Torque_Enable'].values())
    assert any(reply.get('stopped') for reply in replies)
    if action == 'stop': assert replies[0]['request_id'] == 'c' * 32


@pytest.mark.parametrize('failure', ['initial_read', 'profile_readback', 'jaw_off', 'restore'])
def test_component_admission_and_cleanup_failures_never_escape_scope(tmp_path, failure):
    session, bus, _ = setup(tmp_path)
    original_write, original_read = bus.write, bus.read
    if failure == 'initial_read': bus.sync_read_failures['Present_Position'] = 100
    def write(reg, joint, value, **kwargs):
        original_write(reg, joint, value, **kwargs)
        if failure == 'jaw_off' and reg == 'Torque_Enable' and value == 0:
            bus.data[reg][joint] = 1
        if failure == 'restore' and reg == 'Goal_Velocity' and value == 3:
            raise ConnectionError('restore failed')
    def read(reg, joint, **kwargs):
        value = original_read(reg, joint, **kwargs)
        if failure == 'profile_readback' and reg == 'Goal_Velocity' and value == 1: return 2
        return value
    bus.write, bus.read = write, read
    with pytest.raises((RuntimeError, ConnectionError)):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    jaw_only(bus)
    assert session._component_scope
    if failure == 'jaw_off':
        assert ('Goal_Velocity', 'gripper', 3) not in bus.writes


@pytest.mark.parametrize('register', ['Acceleration', 'Goal_Velocity', 'Torque_Limit', 'Goal_Position', 'Torque_Enable'])
def test_component_partial_each_command_write_remains_scoped(tmp_path, register):
    session, bus, _ = setup(tmp_path)
    original_write = bus.write
    fired = False
    def write(reg, joint, value, **kwargs):
        nonlocal fired
        original_write(reg, joint, value, **kwargs)
        if reg == register and not fired:
            fired = True
            raise ConnectionError('partial '+register)
    bus.write = write
    with pytest.raises(ConnectionError):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    session.shutdown(reason='later_stop')
    jaw_only(bus)
    assert not any(bus.data['Torque_Enable'].values())


def test_component_no_body_hold_write_after_success(tmp_path):
    session, bus, _ = setup(tmp_path)
    session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    state = session.handle({'command': 'cancel_hold'})['state']
    assert state['event'] == 'component_cancel_disarmed'
    jaw_only(bus)


def test_component_freshness_fault_before_enabling(tmp_path):
    session, bus, clock = setup(tmp_path)
    bus.present_read_delays = [.101]
    with pytest.raises(RuntimeError, match='feedback'):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    assert ('Torque_Enable', 'gripper', 1) not in bus.writes
    jaw_only(bus)


@pytest.mark.parametrize('failure', ['slow_snapshot', 'cancel_snapshot'])
def test_component_snapshot_cannot_stale_or_cancel_enable_admission(tmp_path, monkeypatch, failure):
    session, bus, clock = setup(tmp_path)
    original = session.publish_state
    def publish(**kwargs):
        state = original(**kwargs)
        if (kwargs.get('event') == 'commission_gripper_sample' and
                bus.data['Goal_Position']['gripper'] == 2008 and
                bus.data['Torque_Enable']['gripper'] == 0):
            if failure == 'slow_snapshot': clock.value += .101
            else: session._cancel('stop_during_snapshot')
        return state
    monkeypatch.setattr(session, 'publish_state', publish)
    with pytest.raises((RuntimeError, ArmSessionCancelled)):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    assert ('Torque_Enable', 'gripper', 1) not in bus.writes
    jaw_only(bus)


@pytest.mark.parametrize('terminal', ['exit', 'eof', 'fault'])
def test_component_real_arm_close_semantics_never_escape_process_teardown(tmp_path, terminal):
    from contextlib import nullcontext
    from workshop.station.arm import LeRobotArm
    from workshop.station.arm_session import ArmSession, run_locked_session
    from test_arm_session import FakeArm, FakeClock
    import os
    arm, clock = FakeArm(tmp_path), FakeClock()
    arm.bus.clock = clock
    # Use actual production close/stop, not a permissive fake close.
    arm.close = lambda: LeRobotArm.close(arm)
    arm.stop = lambda: LeRobotArm.stop(arm)
    arm.config = {'timeouts': {'stop_s': 2}}
    arm.saved, arm.cleanup_diagnostics = {}, []
    disconnected = []
    def disconnect(*, disable_torque=True):
        disconnected.append(disable_torque)
        if disable_torque:
            for joint in arm.bus.motors: arm.bus.write('Torque_Enable', joint, 0)
        arm.bus.is_connected = False
    arm.bus.disconnect = disconnect
    class ComponentSession(ArmSession):
        def initialize(self):
            state = super().initialize()
            arm.bus.data['Present_Position']['elbow_flex'] = 3093
            arm.bus.data['Goal_Position']['elbow_flex'] = 3093
            return state
    def factory(instance, state, logs, **kwargs):
        return ComponentSession(instance, state, logs, clock=clock.monotonic,
                                wall_clock=clock.utc, sleep=clock.sleep, **kwargs)
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b'{"command":"commission_gripper_step","delta_counts":8}\n')
    write_open = True
    def emit(reply):
        nonlocal write_open
        if reply.get('state', {}).get('event') == 'commission_gripper_complete':
            if terminal == 'exit': os.write(write_fd, b'{"command":"exit"}\n')
            else:
                os.close(write_fd)
                write_open = False
    # Buffered exit is consumed as cancellation; EOF exercises the same outer
    # production finally path. Fault fails within the component scope as well.
    if terminal == 'fault':
        original_write = arm.bus.write
        def fail_write(reg, joint, value, **kwargs):
            original_write(reg, joint, value, **kwargs)
            if reg == 'Goal_Velocity' and value == 1: raise ConnectionError('partial profile')
        arm.bus.write = fail_write
    try:
        try:
            run_locked_session({}, tmp_path/'state', tmp_path/'logs2', lock_path=tmp_path/'lock',
                arm_factory=lambda _: arm, session_factory=factory,
                lock_factory=lambda _: nullcontext(), fd=read_fd, emit=emit)
        except (RuntimeError, ConnectionError):
            pass
    finally:
        os.close(read_fd)
        if write_open: os.close(write_fd)
    # Actual completion/fault traverses the production finally path; the final
    # SDK disconnect must be explicit and must not call the global arm stop.
    jaw_only(arm.bus)
    assert disconnected == [False]


def test_component_measured_jaw_overshoot_never_bypasses_normal_bounds(tmp_path):
    session, bus, _ = setup(tmp_path)
    high = session.admitted_max['gripper']
    bus.data['Present_Position']['gripper'] = high-8
    bus.data['Goal_Position']['gripper'] = high-8
    def on_read():
        if bus.data['Torque_Enable']['gripper']:
            bus.data['Present_Position']['gripper'] = high+1
    bus.on_present_read = on_read
    with pytest.raises(RuntimeError, match='jaw actual'):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    jaw_only(bus)


def test_component_write_guards_encoder_age_including_torque_read_and_snapshot(tmp_path, monkeypatch):
    session, bus, clock = setup(tmp_path)
    original_read = bus.sync_read
    def read(register, **kwargs):
        result = original_read(register, **kwargs)
        if register == 'Torque_Enable': clock.value += .019
        return result
    bus.sync_read = read
    original_publish = session.publish_state
    def publish(**kwargs):
        result = original_publish(**kwargs)
        if (kwargs.get('event') == 'commission_gripper_sample' and
                bus.data['Goal_Position']['gripper'] == 2008 and
                bus.data['Torque_Enable']['gripper'] == 0):
            clock.value += .090
        return result
    monkeypatch.setattr(session, 'publish_state', publish)
    with pytest.raises(RuntimeError, match='write admission'):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    assert ('Torque_Enable', 'gripper', 1) not in bus.writes
    jaw_only(bus)


def test_component_rejected_torque_sample_and_goal_write_survive_cleanup_log(tmp_path):
    import json
    session, bus, _ = setup(tmp_path)
    original_write = bus.write
    def write(register, joint, value, **kwargs):
        original_write(register, joint, value, **kwargs)
        # Synthetic unexpected state only; does not assert real firmware does it.
        if register == 'Goal_Position': bus.data['Torque_Enable']['gripper'] = 1
    bus.write = write
    with pytest.raises(RuntimeError, match='expected=0 actual=1'):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    events = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    failed = [e for e in events if e['event'] == 'component_raw_observation' and e['torque']['gripper'] == 1]
    assert len(failed) == 1
    assert failed[0]['expected_gripper_torque'] == 0
    assert set(failed[0]['raw']) == set(bus.motors)
    assert failed[0]['position_monotonic'] <= failed[0]['torque_monotonic']
    goal = [e for e in events if e['event'] == 'component_write_readback' and e['register'] == 'Goal_Position']
    assert len(goal) == 1 and goal[0]['actual_raw_value'] == 2008
    assert goal[0]['actual_type'] == 'int'
    assert not any(e['event'] == 'component_write_intent' and e['register'] == 'Torque_Enable' for e in events)
    assert events[-1]['event'] == 'component_shutdown'
    assert not any(events[-1]['torque'].values())
    jaw_only(bus)


@pytest.mark.parametrize('failure', ['slow', 'cancel'])
def test_component_new_write_diagnostics_cannot_bypass_admission(tmp_path, monkeypatch, failure):
    session, bus, clock = setup(tmp_path)
    original = session._event
    def event(name, **fields):
        original(name, **fields)
        if name == 'component_write_intent' and fields['register'] == 'Torque_Enable':
            if failure == 'slow': clock.value += .101
            else: session._cancel('cancel_during_write_diagnostic')
    monkeypatch.setattr(session, '_event', event)
    with pytest.raises((RuntimeError, ArmSessionCancelled)):
        session.handle({'command': 'commission_gripper_step', 'delta_counts': 8})
    assert ('Torque_Enable', 'gripper', 1) not in bus.writes
    jaw_only(bus)
