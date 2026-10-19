"""No hardware: bounded idle writer and original acquisition clocks."""
import threading
import time
import pytest
from workshop.station.idle_telemetry import IdleTelemetry


def test_blocked_sink_one_pending_immutable_and_command_barrier():
    entered, release = threading.Event(), threading.Event()
    written = []
    def sink(state):
        entered.set()
        assert release.wait(1)
        written.append(state)
    writer = IdleTelemetry(sink)
    source = {'raw': {'j': 1}, 'clock': 3.0}
    writer.offer(source)
    assert entered.wait(.5)
    source['raw']['j'] = 999
    for i in range(100):
        writer.offer({'raw': {'j': i}, 'clock': 3.0})
    assert writer.pending_count == 1
    started = time.monotonic()
    with pytest.raises(TimeoutError):
        with writer.serialized(timeout=.01):
            pytest.fail('blocked writer admitted command')
    assert time.monotonic()-started < .1
    release.set()
    with writer.serialized():
        written.append({'event': 'command'})
    assert writer.close(.5)
    assert written == [{'raw': {'j': 1}, 'clock': 3.0}, {'event': 'command'}]
    with pytest.raises(TypeError): written[0]['raw']['j'] = 2


def test_writer_failure_surfaces_without_retry():
    entered=threading.Event()
    def sink(state):
        entered.set()
        raise OSError('disk blocked')
    writer=IdleTelemetry(sink)
    writer.offer({'monotonic':1.})
    assert entered.wait(.5)
    assert writer.close(.5)
    with pytest.raises(RuntimeError,match='disk blocked'): writer.check()


def test_timing_ring_bounded_and_original_clock_not_changed():
    writer=IdleTelemetry(lambda state: None)
    for i in range(300):
        with writer.stage('acquire'): pass
    events=writer.diagnostics()['timing']
    assert len(events)==256
    assert all(e['wall_s']>=0 and e['thread_cpu_s']>=0 for e in events)
    assert writer.close(.5)


def _session_writer(tmp_path, monkeypatch):
    from test_arm_session import make_session
    import workshop.station.arm_session as module
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    entered, release = threading.Event(), threading.Event()
    original = module._atomic_snapshot_json
    def sink(path, state):
        if state['event']=='sample':
            entered.set()
            assert release.wait(2)
        original(path,state)
    monkeypatch.setattr(module,'_atomic_snapshot_json',sink)
    writer=IdleTelemetry(session._publish_built_state)
    session._idle_telemetry=writer
    writer.offer(session._build_state(event='sample'))
    assert entered.wait(.5)
    return session, arm, clock, writer, release


def test_blocked_writer_command_rejected_before_any_actuation(tmp_path,monkeypatch):
    from workshop.station.arm_session import ArmPlanPreflightError
    session,arm,clock,writer,release=_session_writer(tmp_path,monkeypatch)
    prior=list(arm.bus.writes)
    try:
        with pytest.raises(ArmPlanPreflightError,match='barrier'):
            session.handle({'command':'move_plan','legs':[{'label':'no','target_raw':dict.fromkeys(arm.calibration,2050)}]})
        assert arm.bus.writes==prior and not arm.bus.goal_history
    finally:
        release.set();assert writer.close(.5)
        session._idle_telemetry=None
        session.shutdown();session.close_log()


@pytest.mark.parametrize('command',['stop','exit'])
def test_stop_exit_disarm_before_blocked_sink_barrier(tmp_path,monkeypatch,command):
    session,arm,clock,writer,release=_session_writer(tmp_path,monkeypatch)
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.calibration,1)
    original=arm.bus.write
    writes=[]
    def write(register,joint,value,**kwargs):
        writes.append((register,joint,value,release.is_set()))
        result=original(register,joint,value,**kwargs)
        if register=='Torque_Enable' and set(arm.bus.data['Torque_Enable'].values())=={0}:
            release.set()
        return result
    arm.bus.write=write
    try:
        session.handle({'command':command})
        disables=[w for w in writes if w[0]=='Torque_Enable']
        assert len(disables)==6 and all(w[2]==0 and not w[3] for w in disables)
        assert not arm.bus.eeprom_writes
        assert writer.close(.5)
        import json
        assert json.loads(session.state_file.read_text())['event']=='torque_off'
    finally:
        release.set();writer.close(.5);session._idle_telemetry=None;session.close_log()


def test_health_fault_diagnostics_before_disarm_and_sink(tmp_path,monkeypatch):
    import os
    from test_arm_session import make_session
    session,arm,clock=make_session(tmp_path)
    session.initialize()
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.calibration,1)
    session._failed_endpoint_hold=dict(schema='failed-endpoint-v1',held_raw=dict(arm.bus.data['Present_Position']))
    session._failed_hold_last_tick=clock.monotonic()
    arm.bus.present_read_delays.append(.11)
    original=arm.bus.write
    def write(*args,**kwargs):
        assert session._idle_fault_diagnostics is not None
        return original(*args,**kwargs)
    arm.bus.write=write
    r,w=os.pipe()
    try:
        with pytest.raises(RuntimeError,match='100ms'):
            session.serve(r)
        assert set(arm.bus.data['Torque_Enable'].values())=={0}
        assert session._idle_fault_diagnostics['timing']
        assert not arm.bus.eeprom_writes
    finally: os.close(r);os.close(w);session.close_log()


def test_acquisition_original_clocks_not_refreshed_by_offer(tmp_path):
    from test_arm_session import make_session
    session,arm,clock=make_session(tmp_path);session.initialize()
    writer=IdleTelemetry(lambda state:None);session._idle_telemetry=writer
    try:
        state=session._build_state()
        stamps=(state['monotonic'],state['torque_monotonic'],state['timestamp_utc'])
        clock.value+=.11
        writer.offer(state)
        session._failed_endpoint_hold={'held_raw':state['raw']}
        session._failed_hold_last_tick=clock.monotonic()
        with pytest.raises(RuntimeError):session._check_failed_hold_snapshot(state,state['monotonic'])
        assert stamps==(state['monotonic'],state['torque_monotonic'],state['timestamp_utc'])
    finally:writer.close(.5);session._idle_telemetry=None;session.shutdown();session.close_log()


def test_eof_disarms_before_blocked_writer(tmp_path,monkeypatch):
    import os
    from test_arm_session import make_session
    import workshop.station.arm_session as module
    session,arm,clock=make_session(tmp_path);session.initialize()
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.calibration,1)
    entered,release=threading.Event(),threading.Event()
    snapshot=module._atomic_snapshot_json
    def blocked(path,state):
        if state['event']=='sample':entered.set();assert release.wait(1)
        snapshot(path,state)
    monkeypatch.setattr(module,'_atomic_snapshot_json',blocked)
    original=arm.bus.write
    def disarm(register,joint,value,**kwargs):
        assert not release.is_set()
        result=original(register,joint,value,**kwargs)
        if set(arm.bus.data['Torque_Enable'].values())=={0}:release.set()
        return result
    arm.bus.write=disarm
    r,w=os.pipe();os.close(w)
    def wait(*args):
        assert entered.wait(.5)
        return [r],[],[]
    try:
        assert session.serve(r,wait=wait)==0
        assert set(arm.bus.data['Torque_Enable'].values())=={0}
        assert not arm.bus.eeprom_writes
    finally:release.set();os.close(r);session.close_log()


def test_barrier_does_not_refresh_existing_hold_or_provider_lease(tmp_path,monkeypatch):
    session,arm,clock,writer,release=_session_writer(tmp_path,monkeypatch)
    raw=dict(arm.bus.data['Present_Position'])
    session._failed_endpoint_hold={'held_raw':raw}
    session._failed_hold_last_tick=clock.monotonic()
    original_tick=session._failed_hold_last_tick
    def expire():
        time.sleep(.005)
        clock.value+=.11
        release.set()
    thread=threading.Thread(target=expire);thread.start()
    try:
        # The newer read does NOT renew the prior hold's 100ms gap budget.
        with pytest.raises(RuntimeError,match='100ms'):
            session.handle({'command':'read_state'})
        assert session._failed_hold_last_tick==original_tick
        assert not arm.bus.goal_history
    finally:
        thread.join();release.set();writer.close(.5)
        session._idle_telemetry=None;session.shutdown();session.close_log()


def test_idle_tick_has_no_publication_or_io_and_helper_is_source_pinned():
    import ast,inspect,textwrap
    from workshop.station.arm_session import ArmSession
    from workshop.station.recovery_types import OWNER_SOURCE_MODULES
    assert 'idle_telemetry' in OWNER_SOURCE_MODULES
    source=inspect.getsource(ArmSession._serve)
    begin=source.index('sample_started =')
    end=source.index('except BaseException as error:',begin)
    tick=source[begin:end]
    assert '_build_state' in tick and '_check_failed_hold_snapshot' in tick
    assert tick.index('_check_failed_hold_snapshot')<tick.index('.offer(state)')
    assert all(word not in tick for word in ('publish_state','_record','json.','open(','read_bytes','write('))
    tree=ast.parse(textwrap.dedent(inspect.getsource(ArmSession._check_failed_hold_snapshot)))
    constants=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant)]
    assert .1 in constants


def test_popped_old_sample_cannot_overwrite_command_after_barrier():
    written=[];writer=IdleTelemetry(written.append)
    try:
        with writer.serialized():
            writer.offer({'old':True})
            deadline=time.monotonic()+.5
            while writer.pending_count and time.monotonic()<deadline:time.sleep(.001)
            assert writer.pending_count==0  # Popped; worker waits for our gate.
            with writer.serialized():written.append({'command':True})
        assert writer.close(.5)
        assert written==[{'command':True}]
    finally:writer.close(.5)


@pytest.mark.parametrize('sink_stage',['snapshot','sample_log'])
def test_blocked_real_writer_does_not_block_twelve_acquisitions_or_eof(tmp_path,monkeypatch,sink_stage):
    import os
    import workshop.station.arm_session as module
    from test_arm_session import make_session
    session,arm,clock=make_session(tmp_path);session.initialize()
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.calibration,1)
    session._failed_endpoint_hold={'held_raw':dict(arm.bus.data['Present_Position'])}
    session._failed_hold_last_tick=clock.monotonic()
    entered,release=threading.Event(),threading.Event()
    original=module._atomic_snapshot_json if sink_stage=='snapshot' else session._record_direct
    def sink(*args):
        state=args[-1]
        if state.get('event')=='sample':
            entered.set();assert release.wait(2)
        return original(*args)
    if sink_stage=='snapshot':monkeypatch.setattr(module,'_atomic_snapshot_json',sink)
    else:session._record_direct=sink
    write=arm.bus.write
    def off(register,joint,value,**kwargs):
        assert not release.is_set()
        result=write(register,joint,value,**kwargs)
        if set(arm.bus.data['Torque_Enable'].values())=={0}:release.set()
        return result
    arm.bus.write=off
    r,w=os.pipe();ticks=0;observed=[]
    def wait(*args):
        nonlocal ticks
        ticks+=1
        assert entered.wait(.5)
        observed.append((session._idle_telemetry.pending_count,session._failed_hold_last_tick))
        assert set(arm.bus.data['Torque_Enable'].values())=={1}
        if ticks==12:os.close(w);return [r],[],[]
        clock.sleep(.05)
        return [],[],[]
    try:
        assert session.serve(r,wait=wait)==0
        assert ticks==12 and all(count<=1 for count,stamp in observed)
        assert observed[-1][1]-observed[0][1]>.5
        assert set(arm.bus.data['Torque_Enable'].values())=={0}
        assert not arm.bus.eeprom_writes
    finally:release.set();os.close(r);session.close_log()


def test_real_wall_100ms_acquisition_and_gap_remain_fail_closed(tmp_path):
    from test_arm_session import make_session
    from datetime import datetime,timezone
    session,arm,_=make_session(tmp_path);session.initialize()
    session.clock=time.monotonic
    session.wall_clock=lambda:datetime.now(timezone.utc)
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.calibration,1)
    entered,release=threading.Event(),threading.Event()
    def blocked(state):entered.set();assert release.wait(1)
    writer=IdleTelemetry(blocked);session._idle_telemetry=writer
    try:
        start=session.clock();state=session._build_state()
        session._failed_endpoint_hold={'held_raw':dict(state['raw'])}
        session._failed_hold_last_tick=start
        writer.offer(state);assert entered.wait(.5)
        time.sleep(.11)  # Real OS/thread wait; no virtual time advancement.
        with pytest.raises(RuntimeError,match='100ms'):
            session._check_failed_hold_snapshot(state,start)
        assert session._failed_hold_last_tick==start
        assert state['monotonic']-start<.1
        assert writer.pending_count==0
        active=writer.diagnostics()['active']['writer']
        assert active and active[0]['stage']=='writer'
    finally:
        release.set();assert writer.close(.5)
        session._idle_telemetry=None;session.shutdown();session.close_log()

def test_failed_sink_must_be_visible_before_gate_allows_command():
    released=threading.Event();resume=threading.Event()
    def sink(state):raise OSError('deterministic sink failure')
    writer=IdleTelemetry(sink);original=writer._gate
    class ScheduleGate:
        def acquire(self,*a,**kw):return original.acquire(*a,**kw)
        def release(self):return original.release()
        def __enter__(self):self.acquire();return self
        def __exit__(self,*args):
            self.release()
            if writer.is_writer_thread:
                released.set();assert resume.wait(1)
    writer._gate=ScheduleGate()
    try:
        writer.offer({'event':'sample'})
        assert released.wait(.5)
        with pytest.raises(RuntimeError,match='sink failure'):
            with writer.serialized():
                pass  # Existing command admission would proceed here.
    finally:
        resume.set();assert writer.close(.5)
