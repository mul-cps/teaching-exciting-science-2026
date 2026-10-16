"""Passive renewal control phase; real oracle contracts are tested separately."""
from types import MappingProxyType,SimpleNamespace as NS
import pytest
from test_arm_camera_recovery import recovery_session
from test_recovery_runtime import runtime
from workshop.station.recovery_runtime import RecoveryRuntime,BoundRecoveryScene
from workshop.station.recovery_depth_scene import DepthRecoveryScene
from workshop.station.arm_session import ArmSessionCancelled


def setup(tmp_path,monkeypatch):
    session,arm,clock,_=recovery_session(tmp_path)
    folder=tmp_path/'runtime';folder.mkdir();provider,_,_=runtime(folder)
    session.camera_scene_provider=provider;clock.value=.010
    raw=dict(arm.bus.data['Present_Position'])
    session._camera_recovery_predispatch_baseline=raw
    session._camera_recovery_command_generation='a'*64
    session._camera_recovery_dispatch_attempted=False
    phase={'ready_at':.075,'state':'ready'}
    def health(_):
        return None if phase['state']=='invalid' else dict(camera_generation='a'*64,state='awaiting_post_confirmation_exposure' if clock.value<phase['ready_at'] else phase['state'],
            identity_confirmed_monotonic=.005,identity_request_started_monotonic=.003,
            same_generation_refresh_request_started_monotonic=.003)
    monkeypatch.setattr(provider,'recapture_health',health)
    def candidate(self,raw,stamp):
        acquisition=object.__new__(DepthRecoveryScene)
        acquisition.stamp_monotonic=clock.value
        acquisition.arm_stamp_monotonic=clock.value
        acquisition.arm_raw=MappingProxyType(dict(raw))
        acquisition.acquisition_clock_provenance={'declared_depth_stamp_monotonic':clock.value,
            'camera_identity':{'generation':'a'*64,'request_started_monotonic':.003}}
        return BoundRecoveryScene(acquisition,dict(raw),stamp,clock.value)
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    return session,arm,clock,phase


@pytest.mark.parametrize('fault',[None,'drift','torque','health','timeout','stop','slow_read','slow_log'])
def test_passive_acquisition_is_bounded_all_off_unchanged_and_write_free(tmp_path,monkeypatch,fault):
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    if fault=='health':phase['state']='invalid';phase['ready_at']=0.
    if fault=='timeout':phase['ready_at']=10.
    original_sleep=session.sleep
    def sleep(delay):
        original_sleep(delay)
        if fault=='drift':arm.bus.data['Present_Position']['wrist_roll']+=1
        if fault=='torque':arm.bus.data['Torque_Enable']['gripper']=1
        if fault=='stop':session._cancel('stop')
    session.sleep=sleep
    original_read=session._read
    def read(register):
        if fault=='slow_read' and register=='Present_Position':clock.sleep(.050001)
        return original_read(register)
    session._read=read
    original_publish=session._publish_camera_observation
    def publish(*args,**kwargs):
        result=original_publish(*args,**kwargs)
        if fault=='slow_log':clock.sleep(.050001)
        return result
    session._publish_camera_observation=publish
    before=list(arm.bus.writes)
    if fault is None:
        session._wait_camera_predispatch_acquisition(0.,clock.value)
        context=session._camera_recovery_predispatch_context
        assert context['entered']==.010 and context['deadline']==.510
        assert clock.value>=.075 and not any(arm.bus.data['Torque_Enable'].values())
    else:
        with pytest.raises((RuntimeError,ValueError,TimeoutError,ArmSessionCancelled)):
            session._wait_camera_predispatch_acquisition(0.,clock.value)
    assert arm.bus.writes==before and arm.bus.goal_history==[]
    session.close_log()


def test_passive_wait_reentry_never_resets_original_deadline(tmp_path,monkeypatch):
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    session._wait_camera_predispatch_acquisition(0.,clock.value)
    context=session._camera_recovery_predispatch_context
    phase['ready_at']=10.
    with pytest.raises(TimeoutError):session._wait_camera_predispatch_acquisition(0.,clock.value)
    assert session._camera_recovery_predispatch_context is context
    assert clock.value<=.535 and arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('fault',[None,'geometry','drift','torque','stop','timeout','health_delay','phase_deadline'])
def test_one_recover_command_renews_passive_admission_without_retry_or_goal(tmp_path,monkeypatch,fault):
    from workshop.station.arm_session import _CameraPredispatchRefresh
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    phase['ready_at']=0.;phase['confirmed']=.005
    def health(_):
        if phase.get('wait_finished') and not phase.get('stalled') and fault in {'health_delay','phase_deadline'}:
            phase['stalled']=True;clock.sleep(.060 if fault=='health_delay' else .501)
        return dict(camera_generation='a'*64,state='awaiting_post_confirmation_exposure' if clock.value<phase['ready_at'] else 'ready',
            identity_confirmed_monotonic=phase['confirmed'],identity_request_started_monotonic=.003,
            same_generation_refresh_request_started_monotonic=.003)
    monkeypatch.setattr(session.camera_scene_provider,'recapture_health',health)
    calls=[]
    def proof(raw,stamp,target=None):
        if session._camera_predispatch_refresh_pending():raise _CameraPredispatchRefresh(stamp)
        calls.append(stamp)
        if len(calls)==3 and fault=='geometry':raise ValueError('true geometry rejection after passive wait')
        session._camera_recovery_scene=session.camera_scene_provider(raw,stamp)
        session._camera_recovery_valid_until_monotonic=clock.value+.250
        plan=NS(target_raw=dict(raw,elbow_flex=2973),scene_stamp_monotonic=clock.value,path_sha256='a'*64)
        if len(calls)==2:
            clock.sleep(.005);phase['confirmed']=clock.value
            phase['ready_at']=clock.value+(.075 if fault!='timeout' else 10.)
        return plan
    monkeypatch.setattr(session,'_camera_recovery_check',proof)
    original_wait=session._wait_camera_predispatch_acquisition;waits=[]
    def wait(*args):
        before=list(arm.bus.writes)
        try:
            result=original_wait(*args);phase['wait_finished']=True;return result
        finally:waits.append((before,list(arm.bus.writes),session._camera_recovery_predispatch_context))
    session._wait_camera_predispatch_acquisition=wait
    original_sleep=session.sleep
    def sleep(delay):
        original_sleep(delay)
        if session._camera_recovery_predispatch_context is not None:
            if fault=='drift':arm.bus.data['Present_Position']['wrist_roll']+=1
            if fault=='torque':arm.bus.data['Torque_Enable']['gripper']=1
            if fault=='stop':session._cancel('stop')
    session.sleep=sleep
    original_goal=arm.bus.sync_write
    def goal(register,values,**kwargs):
        if register=='Goal_Position':
            assert session._camera_recovery_dispatch_attempted
            assert session._camera_recovery_predispatch_context is None
        return original_goal(register,values,**kwargs)
    arm.bus.sync_write=goal
    result=None
    try:result=session.handle({'command':'recover_camera'})
    except (RuntimeError,ValueError):pass
    assert len(waits)==1 and waits[0][0]==waits[0][1]
    assert waits[0][2]['entered']==.015 and waits[0][2]['deadline']==.515
    if fault is None:
        assert result['ok'] and result['state']['raw']['elbow_flex']==2973
        assert len(arm.bus.goal_history)==11
    else:
        assert not result or not result['ok']
        assert arm.bus.goal_history==[] and not any(arm.bus.data['Torque_Enable'].values())
        assert session.saved_ram is None
    # Initial native profile is installed once, never replayed by continuation.
    assert sum(write[0:3]==('Acceleration','shoulder_pan',8) for write in arm.bus.writes)==1
    session.close_log()


def test_passive_wait_static_guard_has_no_write_profile_or_filesystem_calls():
    import ast,inspect,textwrap
    from workshop.station.arm_session import ArmSession
    tree=ast.parse(textwrap.dedent(inspect.getsource(ArmSession._wait_camera_predispatch_acquisition)))
    calls={node.func.attr if isinstance(node.func,ast.Attribute) else node.func.id
        for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,(ast.Attribute,ast.Name))}
    assert not calls&{'write','sync_write','_write_each','_enable_current_goal','_ensure_profile',
        'read_bytes','read_text','open','plan_bound_reentry','owner_source_sha256'}


@pytest.mark.parametrize('missing',['baseline','generation','marker','different_marker','dispatch_started'])
def test_initial_or_unknown_refresh_never_enters_passive_continuation(tmp_path,monkeypatch,missing):
    session,arm,_,phase=setup(tmp_path,monkeypatch)
    if missing=='baseline':session._camera_recovery_predispatch_baseline=None
    if missing=='generation':session._camera_recovery_command_generation=None
    if missing=='dispatch_started':session._camera_recovery_dispatch_attempted=True
    if missing in {'marker','different_marker'}:
        monkeypatch.setattr(session.camera_scene_provider,'recapture_health',lambda _:dict(
            state='awaiting_post_confirmation_exposure',identity_request_started_monotonic=.003,
            same_generation_refresh_request_started_monotonic=None if missing=='marker' else .004))
    assert not session._camera_predispatch_refresh_pending()
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('deadline',['phase500','operation6'])
def test_passive_candidate_accessor_cannot_outlive_absolute_deadline(tmp_path,monkeypatch,deadline):
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    phase['ready_at']=0.
    original=RecoveryRuntime.__call__
    def candidate(self,raw,stamp):
        clock.sleep(.030)
        return original(self,raw,stamp)
    monkeypatch.setattr(RecoveryRuntime,'__call__',candidate)
    if deadline=='phase500':
        session._camera_recovery_predispatch_context=MappingProxyType(dict(raw=MappingProxyType(dict(arm.bus.data['Present_Position'])),
            entered=-.470,deadline=.030,generation='a'*64))
        started=0.
    else:started=-5.980
    with pytest.raises(TimeoutError):session._wait_camera_predispatch_acquisition(started,clock.value)
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('fault',['feedback50','phase500'])
def test_final_passive_health_access_cannot_outlive_feedback_or_phase(tmp_path,monkeypatch,fault):
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    raw=dict(arm.bus.data['Present_Position']);phase['ready_at']=0.
    session._camera_recovery_predispatch_context=MappingProxyType(dict(raw=MappingProxyType(raw),
        entered=0.,deadline=.030 if fault=='phase500' else .500,generation='a'*64))
    session._camera_recovery_scene=session.camera_scene_provider(raw,clock.value)
    session._camera_recovery_valid_until_monotonic=.250
    def health(_):
        clock.sleep(.021 if fault=='phase500' else .060)
        return dict(camera_generation='a'*64,state='ready',identity_confirmed_monotonic=.005,
            identity_request_started_monotonic=.003,same_generation_refresh_request_started_monotonic=.003)
    monkeypatch.setattr(session.camera_scene_provider,'recapture_health',health)
    before=list(arm.bus.writes)
    with pytest.raises((RuntimeError,TimeoutError)):
        session._enable_current_goal(raw,0.,'camera_reentry',on_goal_dispatch=lambda:
            session._camera_recovery_match(raw,.010,.010,0.))
    assert arm.bus.writes==before and arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('case',['renewal','generic_ready','wrong_marker','wrong_generation','not_older'])
def test_final_ready_health_old_proof_only_allows_genuine_passive_transition(tmp_path,monkeypatch,case):
    from workshop.station.arm_session import _CameraPredispatchRefresh
    session,arm,clock,phase=setup(tmp_path,monkeypatch)
    clock.value=.020;raw=dict(arm.bus.data['Present_Position'])
    acquisition=object.__new__(DepthRecoveryScene)
    acquisition.stamp_monotonic=.015 if case=='not_older' else .005
    acquisition.acquisition_clock_provenance={'declared_depth_stamp_monotonic':acquisition.stamp_monotonic}
    session._camera_recovery_scene=BoundRecoveryScene(acquisition,raw,.020,.020)
    session._camera_recovery_valid_until_monotonic=.250
    health=dict(camera_generation='b'*64 if case=='wrong_generation' else 'a'*64,state='ready',
        identity_confirmed_monotonic=.010,identity_request_started_monotonic=.008,
        same_generation_refresh_request_started_monotonic=None if case=='generic_ready' else .009 if case=='wrong_marker' else .008)
    monkeypatch.setattr(session.camera_scene_provider,'recapture_health',lambda _:health)
    assert not session._camera_predispatch_refresh_pending()  # ready alone never starts a phase
    if case=='renewal':
        with pytest.raises(_CameraPredispatchRefresh):
            session._enable_current_goal(raw,0.,'camera_reentry',on_goal_dispatch=lambda:
                session._camera_recovery_match(raw,.020,acquisition.stamp_monotonic,0.))
    elif case=='not_older':session._camera_recovery_match(raw,.020,acquisition.stamp_monotonic,0.)
    else:
        with pytest.raises(RuntimeError) as error:session._camera_recovery_match(raw,.020,acquisition.stamp_monotonic,0.)
        assert not isinstance(error.value,_CameraPredispatchRefresh)
    assert arm.bus.goal_history==[]
    session.close_log()
