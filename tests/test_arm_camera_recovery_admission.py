"""A rejected recovery must not energize hardware via a purported hold."""
import pytest
from test_arm_camera_recovery import recovery_session
from workshop.station.arm_session import ArmPlanPreflightError,ArmSessionCancelled


def energizing_bus(arm):
    original=arm.bus.sync_write
    def write(register,values,**kwargs):
        original(register,values,**kwargs)
        if register=='Goal_Position':arm.bus.data['Torque_Enable'].update(dict.fromkeys(values,1))
    arm.bus.sync_write=write


@pytest.mark.parametrize('fault',['changed_raw','profile_write','cancel_hold','disarm_failure'])
def test_pre_dispatch_failures_disarm_without_goal_or_accidental_enable(tmp_path,fault):
    def provider(scene,index,session,arm,clock):
        if index==1 and fault in {'changed_raw','disarm_failure'}:
            arm.bus.data['Present_Position']['wrist_roll']+=1
        return scene
    session,arm,_,_=recovery_session(tmp_path,provider=provider)
    energizing_bus(arm)
    original=arm.bus.write
    fired=False
    def write(reg,joint,value,**kwargs):
        nonlocal fired
        original(reg,joint,value,**kwargs)
        if fault in {'profile_write','cancel_hold'} and reg=='Acceleration' and value==8 and not fired:
            fired=True
            if fault=='profile_write':raise ConnectionError('partial profile write')
            session._cancel('trajectory_cancel_hold')
        if fault=='disarm_failure' and reg=='Torque_Enable' and value==0:
            arm.bus.data[reg][joint]=1
            raise ConnectionError('cannot disarm')
    arm.bus.write=write
    expected=RuntimeError if fault=='disarm_failure' else ArmSessionCancelled if fault=='cancel_hold' else ArmPlanPreflightError
    with pytest.raises(expected):session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[]
    if fault!='disarm_failure':
        assert not any(arm.bus.data['Torque_Enable'].values())
        assert session.saved_ram is None
        assert arm.bus.data['Acceleration']==dict.fromkeys(arm.bus.motors,2)
    session.close_log()


def test_first_goal_write_uncertain_receipt_is_post_dispatch_not_preflight(tmp_path):
    session,arm,_,_=recovery_session(tmp_path)
    energizing_bus(arm)
    original=arm.bus.sync_write
    fired=False
    def write(reg,values,**kwargs):
        nonlocal fired
        original(reg,values,**kwargs)
        if reg=='Goal_Position' and not fired:
            fired=True
            raise ConnectionError('goal applied but reply lost')
    arm.bus.sync_write=write
    result=session.handle({'command':'recover_camera'})
    assert result['ok'] is False and result['recovery']['status']=='held'
    assert len(arm.bus.goal_history)==2
    assert all(value==1 for value in arm.bus.data['Torque_Enable'].values())
    session.shutdown();session.close_log()


def test_preflight_abort_does_not_reuse_earlier_shutdown_ack(tmp_path):
    def provider(scene,*_):scene.clearance=None;return scene
    session,arm,_,_=recovery_session(tmp_path,provider=provider)
    session.shutdown()
    arm.bus.writes.clear()
    # A previous stop is not a fresh readback for this new explicit operation.
    arm.bus.data['Torque_Enable']['wrist_roll']=1
    with pytest.raises(ArmPlanPreflightError):session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[]
    assert not any(arm.bus.data['Torque_Enable'].values())
    assert any(reg=='Torque_Enable' and value==0 for reg,_,value in arm.bus.writes)
    session.close_log()


@pytest.mark.parametrize('fault',['slow_read_state','changed_raw','cancel'])
def test_first_goal_final_input_pump_cannot_invalidate_proof(tmp_path,fault,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    energizing_bus(arm)
    original_enable=session._enable_current_goal
    original_poll=session._poll_busy_input
    in_enable=False
    fired=False
    def enable(*args,**kwargs):
        nonlocal in_enable
        in_enable=True
        return original_enable(*args,**kwargs)
    def poll():
        nonlocal fired
        original_poll()
        if in_enable and not fired:
            fired=True
            session.read_state()  # the real queued-status I/O path
            if fault=='slow_read_state':clock.value+=.251
            if fault=='changed_raw':arm.bus.data['Present_Position']['wrist_roll']+=1
            if fault=='cancel':session._cancel('trajectory_cancel_hold')
    monkeypatch.setattr(session,'_enable_current_goal',enable)
    monkeypatch.setattr(session,'_poll_busy_input',poll)
    with pytest.raises(ArmSessionCancelled if fault=='cancel' else ArmPlanPreflightError):
        session.handle({'command':'recover_camera'})
    assert fired and arm.bus.goal_history==[]
    assert not any(arm.bus.data['Torque_Enable'].values())
    assert session.saved_ram is None
    session.close_log()


def test_camera_completion_keeps_actual_hold_read_stamps_across_slow_log(tmp_path):
    session,arm,clock,_=recovery_session(tmp_path)
    original_event=session._event
    held=[]
    original_hold=session.cancel_hold
    def hold():
        state=original_hold();held.append(state);return state
    session.cancel_hold=hold
    def event(name,**fields):
        original_event(name,**fields)
        if name=='trajectory_cancel_hold':clock.sleep(.200)
    session._event=event
    result=session.handle({'command':'recover_camera'})
    state=result['state']
    assert result['ok']
    for field in ('monotonic','torque_monotonic','timestamp_utc','torque_timestamp_utc'):
        assert state[field]==held[0][field]
    assert clock.monotonic()-state['monotonic'] >= .200
    assert clock.monotonic()-state['torque_monotonic'] >= .200
    assert len(arm.bus.goal_history)==12  # original recovery/terminal hold only
    session.shutdown();session.close_log()


def test_cancel_hold_positions_not_restamped_after_goal_readback(tmp_path):
    session,arm,clock,_=recovery_session(tmp_path)
    read=session._read
    stamps={}
    def delayed(register):
        if register=='Goal_Position':clock.sleep(.050)
        value=read(register)
        if register in {'Present_Position','Torque_Enable'}:stamps[register]=clock.monotonic()
        return value
    session._read=delayed
    state=session.cancel_hold()
    assert state['monotonic']==stamps['Present_Position']
    assert state['torque_monotonic']==stamps['Torque_Enable']
    assert state['torque_monotonic']-state['monotonic'] >= .050
    assert len(arm.bus.goal_history)==1
    session.shutdown();session.close_log()


def test_final_input_pump_cannot_outlive_trusted_identity_deadline(tmp_path):
    session,arm,clock,_=recovery_session(tmp_path)
    original=session._camera_recovery_check
    def proof(*args,**kwargs):
        plan=original(*args,**kwargs)
        session._camera_recovery_valid_until_monotonic=clock.monotonic()+.010
        return plan
    session._camera_recovery_check=proof
    original_enable=session._enable_current_goal
    def enable(*args,**kwargs):
        clock.sleep(.020)
        return original_enable(*args,**kwargs)
    session._enable_current_goal=enable
    with pytest.raises(ArmPlanPreflightError,match='expired'):
        session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[] and not any(arm.bus.data['Torque_Enable'].values())
    session.close_log()


def test_recovery_observation_precedes_rejected_proof_with_actual_clocks(tmp_path):
    import json
    from datetime import datetime
    observations=[]
    def provider(scene,index,session,arm,clock):
        state=json.loads(session.state_file.read_text())
        assert state['event']=='camera_recovery_observation'
        assert state['monotonic']==scene.arm_stamp_monotonic
        utc=datetime.fromisoformat(state['timestamp_utc'])
        assert utc==clock.utc()
        observations.append(state)
        clock.value+=.060
        scene.clearance=None
        return scene
    session,arm,clock,_=recovery_session(tmp_path,provider=provider)
    cached=session._last_published_torque
    with pytest.raises(ArmPlanPreflightError):session.handle({'command':'recover_camera'})
    assert observations[0]['torque_monotonic']==cached[1]
    assert observations[0]['torque_timestamp_utc']==cached[2].isoformat()
    assert arm.bus.goal_history==[]
    session.close_log()


def test_observation_reuses_samples_without_bus_operations_and_never_restamps(tmp_path,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    raw=dict(arm.bus.data['Present_Position']);stamp=clock.monotonic();utc=clock.utc()
    torque=dict(arm.bus.data['Torque_Enable']);tstamp=stamp;tutc=utc
    monkeypatch.setattr(session,'_read',lambda *_:pytest.fail('observation added a bus read'))
    monkeypatch.setattr(session,'_record',lambda *_:setattr(clock,'value',clock.value+.080))
    state=session._publish_camera_observation(raw,stamp,utc,torque_observation=(torque,tstamp,tutc))
    assert state['monotonic']==stamp and state['torque_monotonic']==tstamp
    assert state['timestamp_utc']==utc.isoformat() and state['torque_timestamp_utc']==tutc.isoformat()
    assert arm.bus.goal_history==[]
    session.close_log()


def test_delayed_observation_cannot_enable_first_goal(tmp_path,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    original=session._publish_camera_observation
    def publish(*args,**kwargs):
        result=original(*args,**kwargs);clock.value+=.260;return result
    monkeypatch.setattr(session,'_publish_camera_observation',publish)
    with pytest.raises(ArmPlanPreflightError):session.handle({'command':'recover_camera'})
    assert arm.bus.goal_history==[]
    assert not any(arm.bus.data['Torque_Enable'].values())
    session.close_log()


def test_missing_original_torque_clock_cannot_fall_back_to_new_read(tmp_path,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    monkeypatch.setattr(session,'_read',lambda *_:pytest.fail('hidden bus read'))
    with pytest.raises(ValueError,match='timestamps'):
        session._publish_camera_observation(dict(arm.bus.data['Present_Position']),clock.monotonic(),clock.utc(),
            torque_observation=(dict.fromkeys(arm.bus.motors,0),None,None))
    session.close_log()


def test_recovery_observations_preserve_ordered_bus_trace(tmp_path,monkeypatch):
    traces=[]
    for enabled in (False,True):
        (tmp_path/str(enabled)).mkdir()
        session,arm,_,_=recovery_session(tmp_path/str(enabled))
        trace=[]
        original_read=session._read
        def read(register):
            trace.append(('read',register));return original_read(register)
        monkeypatch.setattr(session,'_read',read)
        original_sync=arm.bus.sync_write
        def sync(register,values,**kwargs):
            trace.append(('sync_write',register,tuple(sorted(values.items()))))
            return original_sync(register,values,**kwargs)
        monkeypatch.setattr(arm.bus,'sync_write',sync)
        original_write=arm.bus.write
        def write(register,joint,value,**kwargs):
            trace.append(('write',register,joint,value));return original_write(register,joint,value,**kwargs)
        monkeypatch.setattr(arm.bus,'write',write)
        if not enabled:monkeypatch.setattr(session,'_publish_camera_observation',lambda *a,**k:None)
        assert session.handle({'command':'recover_camera'})['ok']
        traces.append(trace)
        session.close_log()
    assert traces[0]==traces[1]


@pytest.mark.parametrize('fault',[None,'health','timeout','drift','torque','stop','oldcohort','generation','stale'])
def test_stationary_wait_has_zero_goal_enable_profile_writes(tmp_path,monkeypatch,fault):
    from types import SimpleNamespace as NS
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    held=dict(arm.bus.data['Present_Position'])
    arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    session.camera_scene_provider=provider
    monkeypatch.setattr(provider,'recapture_health',lambda generation:None if fault=='health' else {'state':'ready'})
    def candidate(self,raw,stamp):
        if fault in {'timeout','drift','torque','stop','stale'} or clock.value<.100:return None
        return NS(stamp_monotonic=0. if fault=='oldcohort' else clock.value,
            acquisition_arm_raw=held,acquisition=NS(acquisition_clock_provenance={
                'declared_depth_stamp_monotonic':0. if fault=='oldcohort' else clock.value,
                'camera_identity':{'generation':'b'*64 if fault=='generation' else 'a'*64}}))
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    original_sleep=session.sleep
    def sleep(delay):
        original_sleep(delay)
        if fault=='drift':
            arm.bus.drift_after_goal=('wrist_roll',1);arm.bus._drift_pending=True
        if fault=='torque':arm.bus.data['Torque_Enable']['wrist_roll']=0
        if fault=='stop':session._cancel('stop')
    session.sleep=sleep
    arm.bus.writes.clear();arm.bus.goal_history.clear()
    initial=(held,clock.value,clock.utc(),dict.fromkeys(held,1),clock.value-(.051 if fault=='stale' else 0),clock.utc())
    if fault is None:
        result=session._wait_camera_recapture(held,0.,initial,0.,'a'*64)
        assert result[0]==held and clock.value>=.1
    else:
        with pytest.raises((ValueError,RuntimeError,TimeoutError)) as caught:
            session._wait_camera_recapture(held,0.,initial,0.,'a'*64)
        if fault in {'health','timeout','oldcohort','generation'}:
            result=session._camera_recovery_hold(caught.value)
            assert result['ok'] is False and result['recovery']['status']=='held'
        else:
            with pytest.raises(ArmSessionCancelled):session._camera_recovery_hold(caught.value)
            assert not any(arm.bus.data['Torque_Enable'].values())
    assert arm.bus.goal_history==[]
    assert not any(reg in {'Acceleration','Goal_Velocity','Torque_Limit','Goal_Position'} or
                   reg=='Torque_Enable' and value==1 for reg,_,value in arm.bus.writes)
    session.close_log()


@pytest.mark.parametrize('read_delay',[.004,.060])
def test_stationary_wait_real_clock_read_reserve_and_hard_deadline(tmp_path,monkeypatch,read_delay):
    import time
    from types import SimpleNamespace as NS
    from datetime import datetime,timezone
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,_,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    session.clock=time.monotonic;session.wall_clock=lambda:datetime.now(timezone.utc);session.sleep=time.sleep
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    starts=[];original=session._read
    def read(register):
        if register=='Present_Position':time.sleep(read_delay)
        if register=='Torque_Enable':time.sleep(.002)
        result=original(register)
        if register=='Present_Position':starts.append(time.monotonic())
        return result
    monkeypatch.setattr(session,'_read',read)
    monkeypatch.setattr(provider,'recapture_health',lambda _: {'state':'ready'})
    entered=time.monotonic()
    def provide(self,raw,stamp):
        if time.monotonic()-entered<.100:return None
        return NS(stamp_monotonic=time.monotonic(),acquisition_arm_raw=held,
            acquisition=NS(acquisition_clock_provenance={'declared_depth_stamp_monotonic':time.monotonic(),
                'camera_identity':{'generation':'a'*64}}))
    monkeypatch.setattr(RecoveryRuntime,'__call__',provide)
    utc=session.wall_clock();initial=(held,entered,utc,dict.fromkeys(held,1),entered,utc)
    if read_delay<.050:
        result=session._wait_camera_recapture(held,entered,initial,entered,'a'*64)
        assert result[0]==held and len(starts)>=2
        assert max(b-a for a,b in zip([entered]+starts,starts))<=.051
    else:
        with pytest.raises(ArmSessionCancelled):session._wait_camera_recapture(held,entered,initial,entered,'a'*64)
        assert not any(arm.bus.data['Torque_Enable'].values())
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('delay_at',['goal_read','publication'])
def test_failed_wait_hold_cleanup_expiry_disarms_without_goal(tmp_path,monkeypatch,delay_at):
    session,arm,clock,_=recovery_session(tmp_path)
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    session._camera_recovery_stationary_wait=held
    if delay_at=='goal_read':
        original=session._read
        def read(register):
            result=original(register)
            if register=='Goal_Position':clock.value+=.060
            return result
        monkeypatch.setattr(session,'_read',read)
    else:
        original=session._publish_camera_observation
        def publish(*args,**kwargs):
            result=original(*args,**kwargs);clock.value+=.060;return result
        monkeypatch.setattr(session,'_publish_camera_observation',publish)
    with pytest.raises(RuntimeError,match='expired'):
        session._camera_recovery_hold(ValueError('camera unavailable'))
    assert not any(arm.bus.data['Torque_Enable'].values()) and arm.bus.goal_history==[]
    session.close_log()


def test_stationary_wait_50_001ms_interval_disarms(tmp_path,monkeypatch):
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    monkeypatch.setattr(provider,'recapture_health',lambda _: {'state':'ready'})
    monkeypatch.setattr(RecoveryRuntime,'__call__',lambda *_:None)
    session.sleep=lambda _:setattr(clock,'value',.050001)
    initial=(held,clock.value,clock.utc(),dict.fromkeys(held,1),clock.value,clock.utc())
    with pytest.raises(ArmSessionCancelled,match='cadence'):
        session._wait_camera_recapture(held,0.,initial,0.,'a'*64)
    assert not any(arm.bus.data['Torque_Enable'].values()) and arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('boundary',['wait500','overall6'])
def test_stationary_wait_provider_access_cannot_cross_hard_deadline(tmp_path,monkeypatch,boundary):
    from types import SimpleNamespace as NS
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    monkeypatch.setattr(provider,'recapture_health',lambda _: {'state':'ready'})
    def late(self,raw,stamp):
        clock.value=.500001 if boundary=='wait500' else .100001
        return NS(stamp_monotonic=clock.value)
    monkeypatch.setattr(RecoveryRuntime,'__call__',late)
    initial=(held,clock.value,clock.utc(),dict.fromkeys(held,1),clock.value,clock.utc())
    with pytest.raises(TimeoutError,match='during provider'):
        session._wait_camera_recapture(held,0.,initial,0. if boundary=='wait500' else -5.9,'a'*64)
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('fault',[None,'start','inactive_plus','inactive_minus','overshoot','outward'])
def test_staged_waypoint_exact_interval_disarms_escape_without_new_goal(tmp_path,fault):
    session,arm,_,_=recovery_session(tmp_path)
    start=dict(arm.bus.data['Present_Position']);command=dict(start,elbow_flex=start['elbow_flex']-2)
    actual=dict(start if fault=='start' else command)
    if fault=='inactive_plus':actual['wrist_roll']+=1
    if fault=='inactive_minus':actual['wrist_roll']-=1
    if fault=='overshoot':actual['elbow_flex']-=1
    if fault=='outward':actual['elbow_flex']=start['elbow_flex']+1
    arm.bus.data['Torque_Enable']=dict.fromkeys(start,1)
    arm.bus.data['Present_Position']=dict(actual)
    if fault in {None,'start'}:
        session._check_camera_waypoint(actual,start,command)
        assert all(arm.bus.data['Torque_Enable'].values())
    else:
        with pytest.raises(ArmSessionCancelled):session._check_camera_waypoint(actual,start,command)
        assert not any(arm.bus.data['Torque_Enable'].values())
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('refresh_after_candidate',[False,True])
def test_wait_refresh_transition_never_returns_old_candidate_or_resets_deadline(tmp_path,monkeypatch,refresh_after_candidate):
    from types import SimpleNamespace as NS
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    clock.value=.010
    calls=[];refresh=[not refresh_after_candidate]
    def health(_):
        return {'state':'awaiting_post_confirmation_exposure' if refresh[0] and clock.value<.100 else 'ready'}
    monkeypatch.setattr(provider,'recapture_health',health)
    def candidate(self,raw,stamp):
        calls.append(clock.value)
        if refresh_after_candidate and len(calls)==1:refresh[0]=True
        return NS(stamp_monotonic=clock.value,acquisition_arm_raw=held,
            acquisition=NS(acquisition_clock_provenance={'declared_depth_stamp_monotonic':clock.value,
                'camera_identity':{'generation':'a'*64}}))
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    initial=(held,clock.value,clock.utc(),dict.fromkeys(held,1),clock.value,clock.utc())
    result=session._wait_camera_recapture(held,0.,initial,0.,'a'*64)
    assert result[1]>=.100 and session._camera_recovery_wait_deadline==.500
    assert (calls[0]==.010) if refresh_after_candidate else (calls[0]>=.100)
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('fault',[None,'geometry','slow','deadline','stop','terminal_log'])
def test_retained_hold_refresh_control_path_keeps_original_budget_and_no_terminal_goal(tmp_path,monkeypatch,fault):
    """Control-state regression; oracle correctness has separate real-model tests."""
    from types import SimpleNamespace as NS
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime,BoundRecoveryScene
    from workshop.station.arm_session import _CameraRefreshWait
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    phase={'refresh':False,'triggered':False,'until':0.}; waits=[]
    generation='a'*64
    def health(_):
        return {'state':'awaiting_post_confirmation_exposure' if phase['refresh'] and clock.value<phase['until'] else 'ready'}
    monkeypatch.setattr(provider,'recapture_health',health)
    def candidate(self,raw,stamp):
        return NS(stamp_monotonic=clock.value,acquisition_arm_raw=raw,
            acquisition=NS(acquisition_clock_provenance={'declared_depth_stamp_monotonic':clock.value,
                'camera_identity':{'generation':generation}}))
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    original_check=session._camera_recovery_check
    def check(raw,stamp,target=None):
        if session._camera_refresh_pending():raise _CameraRefreshWait('typed refresh',stamp)
        context=getattr(session,'_camera_recovery_hold_context',None)
        # Refresh during final settling, after an actual completed wait.
        if context is not None and raw['elbow_flex']==2973 and not phase['triggered']:
            phase.update(triggered=True,refresh=True,until=clock.value+.050)
            if fault=='geometry':raise ValueError('real geometry failure concurrent with refresh')
            if fault=='slow':clock.sleep(.050001)
            if fault=='deadline':clock.value=context['deadline']+.000001
            if fault=='stop':session._cancel('stop')
            raise _CameraRefreshWait('typed refresh',stamp)
        acquisition=NS(acquisition_clock_provenance={'camera_identity':{'generation':generation}})
        session._camera_recovery_scene=BoundRecoveryScene(acquisition,raw,stamp,clock.value)
        session._camera_recovery_valid_until_monotonic=clock.value+.250
        return NS(target_raw=dict(raw,elbow_flex=2973),scene_stamp_monotonic=stamp,path_sha256='a'*64)
    monkeypatch.setattr(session,'_camera_recovery_check',check)
    original_wait=session._wait_camera_recapture
    def wait(*args):
        before=len(arm.bus.goal_history)
        result=original_wait(*args)
        context=session._camera_recovery_hold_context
        waits.append((context['entered'],context['deadline'],before,len(arm.bus.goal_history)))
        return result
    monkeypatch.setattr(session,'_wait_camera_recapture',wait)
    original_event=session._event
    def event(name,**kwargs):
        original_event(name,**kwargs)
        if name=='camera_recovery_complete' and fault=='terminal_log':clock.sleep(.050001)
    monkeypatch.setattr(session,'_event',event)
    result=None
    try:result=session.handle({'command':'recover_camera'})
    except (ArmSessionCancelled,RuntimeError):pass
    assert phase['triggered']
    assert all(before==after for _,_,before,after in waits)
    assert len(arm.bus.goal_history)==11  # seed + ten certified waypoints, never terminal Goal
    if fault is None:
        assert result['ok'] and result['state']['raw']['elbow_flex']==2973
        assert waits[-1][:2]==waits[-2][:2]  # same entry/deadline across refresh
    else:
        assert result is None or not result['ok']
        if fault in {'slow','stop','terminal_log'}:assert not any(arm.bus.data['Torque_Enable'].values())
    session.close_log()


@pytest.mark.parametrize('health_state',['ready','pending_future_bracket','nonstationary_bracket'])
def test_final_match_ready_new_confirmation_cannot_use_old_proof_scene(tmp_path,monkeypatch,health_state):
    from types import SimpleNamespace as NS,MappingProxyType
    from test_recovery_runtime import runtime
    from workshop.station.arm_session import _CameraRefreshWait
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider
    raw=dict(arm.bus.data['Present_Position']);clock.value=.020
    session._camera_recovery_command_generation='a'*64
    session._camera_recovery_hold_context=MappingProxyType(dict(raw=raw,entered=0.,deadline=.500,generation='a'*64))
    session._camera_recovery_scene=NS(stamp_monotonic=.005,
        acquisition=NS(acquisition_clock_provenance={'declared_depth_stamp_monotonic':.005}))
    session._camera_recovery_valid_until_monotonic=.250
    monkeypatch.setattr(provider,'recapture_health',lambda _:dict(state=health_state,
        identity_confirmed_monotonic=.010,identity_request_started_monotonic=.008,
        same_generation_refresh_request_started_monotonic=.008))
    with pytest.raises(_CameraRefreshWait):
        session._enable_current_goal(raw,0.,'camera_reentry',on_goal_dispatch=lambda:session._camera_recovery_match(raw,.020,.005,0.))
    assert arm.bus.goal_history==[]
    session.close_log()


def test_refresh_marker_wait_yields_without_repeated_old_candidate_proofs(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS,MappingProxyType
    from test_recovery_runtime import runtime
    from workshop.station.recovery_runtime import RecoveryRuntime
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider;clock.value=.010
    monkeypatch.setattr('workshop.station.recovery_runtime.time.monotonic',clock.monotonic)
    held=dict(arm.bus.data['Present_Position']);arm.bus.data['Torque_Enable']=dict.fromkeys(held,1)
    provider._runtime_preparation_fault=False
    provider._health=MappingProxyType(dict(camera_generation='a'*64,state='pending_future_bracket',
        valid_until_monotonic=.400,identity_confirmed_monotonic=.005,
        identity_request_started_monotonic=.003,same_generation_refresh_request_started_monotonic=.003))
    provider._latest=NS(stamp_monotonic=.001,acquisition_clock_provenance={'declared_depth_stamp_monotonic':.001})
    calls=[];sleeps=[]
    def candidate(self,raw,stamp):
        calls.append(clock.value)
        return NS(stamp_monotonic=clock.value,acquisition_arm_raw=held,
            acquisition=NS(acquisition_clock_provenance={'declared_depth_stamp_monotonic':clock.value,
                'camera_identity':{'generation':'a'*64}}))
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    original_sleep=session.sleep
    def sleep(delay):
        sleeps.append(delay);original_sleep(delay)
        if clock.value>=.100:provider._latest=NS(stamp_monotonic=clock.value,
            acquisition_clock_provenance={'declared_depth_stamp_monotonic':clock.value})
    session.sleep=sleep
    session._wait_camera_recapture(held,0.,(held,clock.value,clock.utc(),dict.fromkeys(held,1),clock.value,clock.utc()),0.,'a'*64)
    assert calls and min(calls)>=.100
    assert len(sleeps)>=3 and min(sleeps)>.020
    assert arm.bus.goal_history==[]
    session.close_log()
