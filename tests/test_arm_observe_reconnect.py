"""No hardware: passive owner connection and explicit disarm boundaries."""
from pathlib import Path
import hashlib
import pytest
from test_arm_session import make_session
from workshop.station.arm_session import ArmPlanPreflightError


def observer(tmp_path, torque=0):
    session, arm, clock = make_session(tmp_path)
    session._observe_only = True
    arm.bus.data['Torque_Enable']['elbow_flex'] = torque
    arm.connect = lambda **kw: setattr(arm.bus, 'is_connected', True)
    return session, arm, clock


@pytest.mark.parametrize('torque', [0, 1])
def test_observe_owner_never_writes_until_explicit_stop(tmp_path, torque):
    session, arm, _ = observer(tmp_path, torque)
    state = session.initialize()
    assert state['observe_only'] is True
    assert state['torque']['elbow_flex'] == torque
    assert state['owner_source_sha256']['arm_session'] == hashlib.sha256(Path('workshop/station/arm_session.py').read_bytes()).hexdigest()
    assert not arm.bus.writes
    session.shutdown(reason='idle_timeout')
    session.handle({'command': 'exit'})
    assert not arm.bus.writes
    for command in ({'command': 'home'}, {'command': 'commission_gripper_step', 'delta_counts':8}, {'command':'teleop_start'}):
        with pytest.raises(ArmPlanPreflightError): session.handle(command)
    assert not arm.bus.writes
    result = session.handle({'command': 'stop', 'request_id': 'b'*32})
    assert result['stopped'] and not any(result['state']['torque'].values())
    assert not session._observe_only
    assert all(reg=='Torque_Enable' and value==0 for reg, _, value in arm.bus.writes)


def test_observe_failed_initialization_cleanup_never_writes(tmp_path):
    session, arm, _ = observer(tmp_path)
    arm.bus.sync_read_failures['Present_Position'] = 100
    with pytest.raises(ConnectionError): session.initialize()
    assert not arm.bus.writes


from test_station_arm import setup


@pytest.mark.parametrize('failure', [None, 'model', 'timeout_setup', 'disconnect'])
def test_production_arm_observe_connect_and_close_no_writes(setup, monkeypatch, failure):
    arm, bus, _ = setup
    if failure == 'model': bus.ping = lambda *_a, **_kw: 123
    if failure == 'timeout_setup': monkeypatch.setattr(arm, '_timeouts', lambda: (_ for _ in ()).throw(RuntimeError('timeout setup')))
    closed = []
    def disconnect(*, disable_torque):
        assert disable_torque is False
        closed.append(True)
        if failure == 'disconnect': raise RuntimeError('disconnect failure')
        bus.is_connected = False
    bus.disconnect = disconnect
    if failure in {'model', 'timeout_setup'}:
        with pytest.raises((ValueError, RuntimeError)): arm.connect(observe_only=True)
    else:
        arm.connect(observe_only=True)
        if failure == 'disconnect':
            with pytest.raises(RuntimeError): arm.close()
        else: arm.close()
    assert not bus.writes
    assert closed


@pytest.mark.parametrize('failure', [None, 'initialization', 'serve'])
def test_observe_run_locked_production_teardown_never_global_stop(tmp_path, failure):
    from contextlib import nullcontext
    from workshop.station.arm_session import ArmSession, run_locked_session
    from workshop.station.arm import LeRobotArm
    from test_arm_session import FakeArm, FakeClock
    arm, clock = FakeArm(tmp_path), FakeClock()
    arm.bus.clock = clock
    arm.connect = lambda **kwargs: setattr(arm.bus, 'is_connected', True)
    arm.close = lambda: LeRobotArm.close(arm)
    arm.stop = lambda: (_ for _ in ()).throw(AssertionError('global stop escaped observer'))
    disconnects = []
    arm.bus.disconnect = lambda **kw: (disconnects.append(kw), setattr(arm.bus, 'is_connected', False))
    class Observer(ArmSession):
        def initialize(self):
            if failure == 'initialization': arm.bus.sync_read_failures['Present_Position'] = 100
            return super().initialize()
        def serve(self, **kwargs):
            if failure == 'serve': raise RuntimeError('read failure')
            return 0
    def factory(instance, state, logs, **kwargs):
        return Observer(instance, state, logs, clock=clock.monotonic, wall_clock=clock.utc, sleep=clock.sleep, **kwargs)
    try:
        run_locked_session({}, tmp_path/'state', tmp_path/'logs2', lock_path=tmp_path/'lock', observe_only=True,
            arm_factory=lambda _: arm, session_factory=factory, lock_factory=lambda _: nullcontext())
    except (RuntimeError, ConnectionError):
        assert failure is not None
    assert not arm.bus.writes
    assert disconnects == [{'disable_torque': False}]


def test_reconnect_provenance_stays_cached_on_ticks(tmp_path, monkeypatch):
    import ast
    import inspect
    import textwrap
    from workshop.station.arm_session import ArmSession
    source = ast.parse(textwrap.dedent(inspect.getsource(ArmSession.publish_state)))
    assert not any(isinstance(n, ast.Attribute) and n.attr in {'read_bytes', 'sha256', 'read_text'} for n in ast.walk(source))
    session, arm, _ = observer(tmp_path)
    session.initialize()
    def forbidden(*_a, **_kw): raise AssertionError('source filesystem access on telemetry tick')
    monkeypatch.setattr(Path, 'read_bytes', forbidden)
    sources = session._owner_source_sha256
    for _ in range(20):
        state = session.publish_state()
        assert state['owner_source_sha256'] is sources
    assert not arm.bus.writes


def test_observe_direct_controller_methods_cannot_bypass_protocol_gate(tmp_path):
    session, arm, _ = observer(tmp_path)
    session.initialize()
    for call in (lambda: session.commission_gripper_step(8), session.cancel_hold,
                 session._ensure_profile, lambda: session._enable_current_goal({},0,'invalid')):
        with pytest.raises(ArmPlanPreflightError): call()
    assert not arm.bus.writes


def test_observe_initialization_preserves_actual_torque_age_through_slow_home_load(tmp_path, monkeypatch):
    import workshop.station.arm_session as module
    session, arm, clock = observer(tmp_path)
    original = module.load_home
    def slow(*args, **kwargs):
        result=original(*args,**kwargs)
        clock.value+=.15
        return result
    monkeypatch.setattr(module,'load_home',slow)
    state=session.initialize()
    assert state['monotonic'] - state['torque_monotonic'] >= .15
    assert not arm.bus.writes


def test_in_range_observer_nonfatal_motion_rejection_then_explicit_stop(tmp_path):
    import os,json
    session,arm,_=observer(tmp_path)
    session.initialize()
    read_fd,write_fd=os.pipe()
    for command in ({'command':'home'}, {'command':'commission_gripper_step','delta_counts':8},
                    {'command':'teleop_start'}, {'command':'stop','request_id':'a'*32}, {'command':'exit'}):
        os.write(write_fd,(json.dumps(command)+'\n').encode())
    os.close(write_fd)
    replies=[]
    try: assert session.serve(fd=read_fd,emit=replies.append)==0
    finally: os.close(read_fd)
    rejected=replies[:3]
    assert all(r['ok'] is False and r['error_type']=='ArmPlanPreflightError' for r in rejected)
    assert replies[3]['stopped'] is True and replies[4]['exit'] is True
    assert all(reg=='Torque_Enable' and value==0 for reg,_,value in arm.bus.writes)
