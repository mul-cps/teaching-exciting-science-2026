"""Resident capture uses only fake existing bus; no hardware or actuation."""
import ast
import hashlib
import inspect
import json
from pathlib import Path
import textwrap
import pytest
from test_arm_session import make_session
from workshop.station import arm as arm_module


def passive(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize();session.handle({'command':'stop'})
    arm.bus.writes.clear();arm.bus.goal_history.clear()
    return session,arm,clock


def capture(session):return session.handle({'command':'capture_home','command_id':'c'*32,
    'expected_start_raw':dict(session.bus.data['Present_Position']), 'owner_session_id':session._owner_session_id})


def test_capture_after_stop_preserves_caches_exact_prior_and_never_opens_bus(tmp_path,monkeypatch):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    pose,raw=dict(s.home_pose),dict(s.home_raw)
    a.bus.data['Present_Position']['shoulder_pan']+=3
    monkeypatch.setattr(a,'connect',lambda **kwargs:pytest.fail('no second connection'))
    r=capture(s);assert r['ok'] and r['command_id']=='c'*32
    v=json.loads(path.read_text());assert v['kind']=='measured-home' and v['raw_position']==a.bus.data['Present_Position']
    assert v['geometry_commissioned'] is v['grasp_commissioned'] is v['qualified'] is False
    assert v['previous_sha256']==hashlib.sha256(old).hexdigest()
    assert Path(v['previous_artifact']).read_bytes()==old
    assert v['owner_session_id']==s._owner_session_id and v['calibration_sha256']==s.calibration_digest
    assert v['torque_at_capture']==dict.fromkeys(raw,0)
    assert s.home_pose==pose and s.home_raw==raw
    assert not a.bus.writes and not a.bus.goal_history


@pytest.mark.parametrize('fault',['torque','failed_hold','teleop','component','restore','recovery','observe','uncalibrated','bounds','identity','source','path'])
def test_unsafe_capture_never_replaces_or_writes(tmp_path,fault):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    if fault=='torque':a.bus.data['Torque_Enable']['elbow_flex']=1
    if fault=='failed_hold':s._failed_endpoint_hold={'held_raw':dict(a.bus.data['Present_Position'])}
    if fault=='teleop':s._teleop={'busy':True}
    if fault=='component':s._component_scope=True
    if fault=='restore':s.saved_ram={'unrestored':True}
    if fault=='recovery':s._camera_recovery_wait_active=True
    if fault=='observe':s._observe_only=True
    if fault=='uncalibrated':a.bus.is_calibrated=False
    if fault=='bounds':a.bus.data['Present_Position']['elbow_flex']=4095
    if fault=='identity':a.settings['id']='other'
    if fault=='source':s._owner_source_sha256={'changed':'source'}
    if fault=='path':a.settings['home_file']=str(tmp_path/'another.json')
    r=capture(s);assert not r['ok']
    assert path.read_bytes()==old and not a.bus.writes and not a.bus.goal_history


@pytest.mark.parametrize('fault',['drift','late','read','calibration','write','replace'])
def test_failure_preserves_prior_artifact(tmp_path,monkeypatch,fault):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes();read=a.bus.sync_read
    count=0
    def changed(register,**kwargs):
        nonlocal count
        value=read(register,**kwargs)
        if register=='Present_Position':
            count+=1
            if fault=='drift' and count>=2:value['elbow_flex']+=1
            if fault=='late':c.value+=.11
            if fault=='read':raise OSError('read fixture')
            if fault=='calibration':Path(a.settings['calibration']['path']).write_text('{}')
        return value
    a.bus.sync_read=changed
    if fault=='write':monkeypatch.setattr(arm_module.os,'fsync',lambda fd:(_ for _ in ()).throw(OSError('write fixture')))
    if fault=='replace':
        replace=Path.replace
        def fail(self,target):
            if Path(target)==path:raise OSError('replace fixture')
            return replace(self,target)
        monkeypatch.setattr(Path,'replace',fail)
    r=capture(s);assert not r['ok']
    assert path.read_bytes()==old and not a.bus.writes and not a.bus.goal_history


def test_capture_has_fixed_fields_and_no_register_write_connect_or_promotion(tmp_path):
    s,a,c=passive(tmp_path)
    for request in ({'command':'capture_home'}, {'command':'capture_home','command_id':'x','path':'/tmp/arbitrary'}):
        with pytest.raises(ValueError):s.handle(request)
    tree=ast.parse(textwrap.dedent(inspect.getsource(type(s).capture_home)))
    calls=[ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)]
    assert not any(x.endswith(('.connect','.write','.sync_write','.shutdown','.home','.initialize')) for x in calls)


def test_failed_hold_then_explicit_stop_can_capture_same_owner(tmp_path):
    from test_endpoint_failure_hold import plateau
    s,a,c,command=plateau(tmp_path)
    assert s.handle(command)['status']=='failed_endpoint_held'
    assert not capture(s)['ok']
    s.handle({'command':'stop'})
    a.bus.on_present_read=None;a.bus.writes.clear();a.bus.goal_history.clear()
    response=capture(s);assert response['ok'],response
    assert not a.bus.writes and not a.bus.goal_history


def test_final_staging_read_failure_preserves_prior_home(tmp_path,monkeypatch):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    replace=arm_module.replace_measured_home
    def staged(path,data,guard):
        a.bus.sync_read_failures['Present_Position']=3
        return replace(path,data,guard)
    monkeypatch.setattr(arm_module,'replace_measured_home',staged)
    response=capture(s);assert not response['ok']
    assert path.read_bytes()==old and not a.bus.writes and not a.bus.goal_history


def test_cancel_before_commit_preserves_home_no_register_writes(tmp_path,monkeypatch):
    from workshop.station.arm_session import ArmSessionCancelled
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    replace=arm_module.replace_measured_home
    def staged(path,data,guard):
        s._cancel_requested=True;s._cancel_reason='busy_stop'
        return replace(path,data,guard)
    monkeypatch.setattr(arm_module,'replace_measured_home',staged)
    with pytest.raises(ArmSessionCancelled):capture(s)
    assert path.read_bytes()==old and not a.bus.writes and not a.bus.goal_history


def test_write_failure_after_backup_before_final_guard_leaves_exact_previous(tmp_path,monkeypatch):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    calls=0;sync=arm_module.os.fsync
    def fail(fd):
        nonlocal calls
        calls+=1
        if calls==3:raise OSError('directory sync fixture')
        return sync(fd)
    monkeypatch.setattr(arm_module.os,'fsync',fail)
    assert not capture(s)['ok'] and path.read_bytes()==old
    assert not a.bus.writes and not a.bus.goal_history


@pytest.mark.parametrize('fault',['source','identity','delay','torque','drift'])
def test_final_commit_guard_rejects_changed_or_late_measurement(tmp_path,monkeypatch,fault):
    s,a,c=passive(tmp_path);path=Path(a.settings['home_file']);old=path.read_bytes()
    replace=arm_module.replace_measured_home
    def staged(path,data,guard):
        def mutated_guard():
            if fault=='source':s._owner_source_sha256={'changed':'source'}
            if fault=='identity':a.settings['id']='other'
            if fault=='delay':c.value+=2.
            if fault=='torque':a.bus.data['Torque_Enable']['elbow_flex']=1
            if fault=='drift':a.bus.data['Present_Position']['elbow_flex']+=1
            return guard()
        return replace(path,data,mutated_guard)
    monkeypatch.setattr(arm_module,'replace_measured_home',staged)
    response=capture(s);assert not response['ok'] and path.read_bytes()==old
    assert not a.bus.writes and not a.bus.goal_history


def test_capture_cold_only_and_writer_commit_has_no_postreplace_filesystem_calls():
    import inspect
    from workshop.station.arm_session import ArmSession
    for fn in (ArmSession.publish_state,ArmSession._arm_bounds_observation):
        source=inspect.getsource(fn)
        assert 'capture_home(' not in source and 'replace_measured_home(' not in source
    source=inspect.getsource(arm_module.replace_measured_home)
    assert 'if not committed:' in source and 'final_guard()' in source


@pytest.mark.parametrize('fault',['malformed_fields','malformed_id','torque_on','filesystem','precommit'])
def test_served_capture_rejection_never_shutdown_or_disarms(tmp_path,monkeypatch,fault):
    import os
    import select
    from workshop.station.arm_session import ArmSession
    s,a,c=passive(tmp_path)
    # Do not rely on shutdown's no-op cache: a generic served failure WOULD write.
    s._shutdown_complete=False
    path=Path(a.settings['home_file']);old=path.read_bytes()
    request=dict(command='capture_home',command_id='d'*32,
        expected_start_raw=dict(a.bus.data['Present_Position']),owner_session_id=s._owner_session_id)
    if fault=='malformed_fields':request['path']='/tmp/forbidden'
    if fault=='malformed_id':request['command_id']='wrong'
    if fault=='torque_on':a.bus.data['Torque_Enable']=dict.fromkeys(request['expected_start_raw'],1)
    if fault=='filesystem':monkeypatch.setattr(arm_module.os,'fsync',lambda fd:(_ for _ in ()).throw(OSError('FS fixture')))
    if fault=='precommit':
        replace=arm_module.replace_measured_home
        def staged(path,data,guard):
            def expired():c.value+=2.;return guard()
            return replace(path,data,expired)
        monkeypatch.setattr(arm_module,'replace_measured_home',staged)
    shutdown=[];original=s.shutdown
    def tracked(**kwargs):shutdown.append(kwargs);return original(**kwargs)
    monkeypatch.setattr(s,'shutdown',tracked)
    class EndFixture(Exception):pass
    read_fd,write_fd=os.pipe();os.write(write_fd,(json.dumps(request)+'\n').encode());replies=[]
    def wait(readers,writers,errors,timeout):
        if replies:raise EndFixture()
        return select.select(readers,writers,errors,0)
    try:
        with pytest.raises(EndFixture):s.serve(fd=read_fd,emit=replies.append,wait=wait)
    finally:os.close(read_fd);os.close(write_fd);s.close_log()
    assert replies and replies[0]['ok'] is False
    assert not shutdown and not a.bus.writes and not a.bus.goal_history
    assert path.read_bytes()==old
    if fault=='torque_on':assert set(a.bus.data['Torque_Enable'].values())=={1}
