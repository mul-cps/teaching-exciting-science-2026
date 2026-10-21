"""Disabled native unit rendering and lifecycle proof, without systemd/ROS/devices."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S
import pytest
from test_station_process import station
ROOT=Path(__file__).resolve().parents[1]


def units():
    path=ROOT/'tools/install_station_user_units.py'
    assert path.exists(),'unit installer implementation missing'
    spec=importlib.util.spec_from_file_location('station_units_test',path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def args(tmp_path):
    return S(profile='fwmini4',wrapper=None,wrapper_command=None,domain=177,
             config=tmp_path/'station.json',python=Path.home()/'.local/share/exciting-science/station/venv/bin/python',
             contract=tmp_path/'limits.json',state_file=tmp_path/'state.json',station_lock=tmp_path/'lock',
             log_dir=tmp_path/'logs',calibration_dir=tmp_path/'calibration',camera_sdk=Path.home()/'miniforge3/envs/so101-jazzy',
             color_mask_backend='vulkan',existing_tunnel=True,bootstrap_owner=False,background=True)


def manifest(tmp_path):
    return units().build_manifest(args(tmp_path))


def test_render_has_fixed_roles_shared_arguments_disabled_units_and_exact_allowlists(tmp_path):
    u=units();m=u.build_manifest(args(tmp_path));u.validate_manifest(m)
    commands={role:u.component_command(m,role) for role in u.ROLES}
    for role,command in commands.items():
        assert command[:5]==[str(Path.home()/'.local/bin/station-fwmini4'),'ros-vulkan','ros2','launch','so101_station_bringup']
        assert command[5]=='station_live.launch.py'
        assert f'component:={role}' in command
        assert command[6:-1]==commands['owner'][6:-1]
        text=u.unit_text(m,role)
        assert 'Restart=no' in text and '[Install]' not in text and 'WantedBy=' not in text
        assert 'ROS_DOMAIN_ID=177' in text and 'ROS_LOCALHOST_ONLY=1' in text
        assert 'KillMode=control-group' in text
    p=m['parameters']
    assert p['operator_camera_unit']==p['operator_depth_unit']==u.UNITS['camera']
    assert p['operator_recovery_profile']=='native_ros'
    assert p['operator_native_mask_backend']=='vulkan'
    assert p['color_profile']==p['depth_profile']=='640,480,30'
    assert not set((u.UNITS['owner'],u.UNITS['apis'])) & {v for k,v in p.items() if k.endswith('_unit')}
    with pytest.raises(ValueError):u.component_command(m,'arbitrary-command')


@pytest.mark.parametrize('change',[lambda m:m.update(wrapper='/bin/sh'),lambda m:m.update(domain=76),
    lambda m:m['parameters'].update(component='owner'),lambda m:m['parameters'].update(station_python='/bin/sh'),
    lambda m:m['parameters'].update(operator_scene_unit='so101-fwmini4-owner.service'),
    lambda m:m['parameters'].update(operator_recovery_profile='derived_rocm'),lambda m:m.update(extra='unknown'),
    lambda m:m['sources'].update({'../other':'a'*64}),lambda m:m['parameters'].update(color_mask_backend='cpu')])
def test_malformed_or_unsafe_manifests_are_rejected(tmp_path,change):
    u=units();m=u.build_manifest(args(tmp_path));change(m)
    with pytest.raises((ValueError,RuntimeError)):u.validate_manifest(m)


def test_renderer_is_deterministic_and_never_overwrites_unowned_files(tmp_path):
    u=units();m=u.build_manifest(args(tmp_path));out=tmp_path/'rendered'
    u.write_installation(m,out,out/'manifest.json')
    before={p.name:p.read_bytes() for p in out.iterdir()}
    u.write_installation(m,out,out/'manifest.json')
    assert before=={p.name:p.read_bytes() for p in out.iterdir()}
    (out/u.UNITS['owner']).write_text('unowned')
    with pytest.raises(RuntimeError):u.write_installation(m,out,out/'manifest.json')
    assert (out/u.UNITS['owner']).read_text()=='unowned'


def test_unit_quoting_preserves_spaces_percent_and_json_arguments(tmp_path):
    u=units();m=u.build_manifest(args(tmp_path));m['parameters']['station_config']=str(tmp_path/'a b%name.json')
    u.validate_manifest(m)
    text=u.unit_text(m,'owner')
    assert 'ExecStart=:' in text and 'a b%%name.json' in text
    assert '\\"rgb_image\\"' in text


def test_unit_manager_identity_is_required_before_dispatch():
    assert callable(getattr(station,'manager_identity',None)),'missing exact user-manager identity guard'


def test_unit_mode_preserves_prerequisites_and_passive_bootstrap(tmp_path,monkeypatch):
    a=args(tmp_path);a.deployment='user-units';a.unit_manifest=tmp_path/'manifest.json'
    calls=[]
    monkeypatch.setattr(station,'duplicate_owners',lambda:[])
    monkeypatch.setattr(station,'prepare_unit_start',lambda args,ros:calls.append('units') or {},raising=False)
    monkeypatch.setattr(station,'prerequisites',lambda *x:(_ for _ in ()).throw(RuntimeError('missing device')))
    monkeypatch.setattr(station,'start_units',lambda *x:pytest.fail('must not start units'),raising=False)
    with pytest.raises(RuntimeError,match='missing device'):station.start(a,tmp_path/'record.json')
    assert calls==['units']


def test_unit_owner_proof_requires_identity_source_six_off_and_freshness(tmp_path):
    assert hasattr(station,'validate_unit_owner'),'missing fresh owner proof'
    now=10.
    expected=dict(arm_id='fake',calibration_sha256='a'*64,owner_source_sha256={'arm_session':'b'*64})
    record={'expected_owner':expected,'owner_identity':None,'owner_requested_monotonic':8.}
    state=dict(expected,kind='follower_status',fresh=True,connected=True,owner_faulted=False,
               owner_started_monotonic=9.,sampled_monotonic=now,position_monotonic=now,torque_monotonic=now,
               torque=dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0),
               external_job_pending=False)
    identity=station.validate_unit_owner(record,state,now,all_off=True)
    for update in ({'fresh':False},{'position_monotonic':8.},{'owner_source_sha256':{}},
                   {'external_job_pending':True},{'owner_started_monotonic':7.},
                   {'torque':dict.fromkeys(state['torque'],1)}):
        with pytest.raises(RuntimeError):station.validate_unit_owner(record,dict(state,**update),now,all_off=True)
    record['owner_identity']=identity
    with pytest.raises(RuntimeError):station.validate_unit_owner(record,dict(state,owner_started_monotonic=9.5),now,all_off=True)


def test_manifest_round_trip_preserves_exact_unit_execstart_bytes(tmp_path):
    u=units();m=u.build_manifest(args(tmp_path))
    restored=json.loads(json.dumps(m,sort_keys=True))
    assert u.unit_text(m,'owner')==u.unit_text(restored,'owner')


@pytest.fixture
def fake_manager(tmp_path,monkeypatch):
    import os,time
    u=units();a=args(tmp_path);a.deployment='user-units';a.unit_manifest=tmp_path/'manifest.json'
    native=u.native_parameters
    monkeypatch.setattr(u,'native_parameters',lambda:dict(native(),operator_maintenance_journal=str(tmp_path/'pending.json')))
    m=u.build_manifest(a);a.unit_manifest.write_text(json.dumps(m))
    expected=dict(arm_id='fake',calibration_sha256='a'*64,owner_source_sha256={'arm_session':'b'*64})
    a.contract.write_text(json.dumps(expected))
    manager={'uid':os.getuid(),'boot_id':'00000000-0000-0000-0000-000000000001','dbus_owner':':1.3'}
    rows={role:dict(Id=u.UNITS[role],LoadState='loaded',ActiveState='inactive',SubState='dead',Job='0',
                   InvocationID='',StateChangeTimestampMonotonic='1',MainPID='0',ControlGroup='/fixture/'+u.UNITS[role]) for role in u.ROLES}
    model=S(u=u,args=a,manifest=m,manager=manager,rows=rows,calls=[],generation=None,stop_error=None,start_error=None,stop_unit_error=None)
    monkeypatch.setenv('ROS_DOMAIN_ID','177');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setattr(station,'unit_tools',lambda:u)
    monkeypatch.setattr(station,'manager_identity',lambda:dict(model.manager))
    monkeypatch.setattr(station,'read_managed_unit',lambda manifest,role:dict(rows[role]))
    def binding(row):
        if row['ActiveState']!='active' or row['SubState']!='running' or row['Job']!='0' or row['MainPID']=='0':raise RuntimeError('unit not active')
        return dict(invocation=row['InvocationID'],process={'pid':int(row['MainPID'])},control_group=row['ControlGroup'])
    monkeypatch.setattr(station,'unit_binding',binding)
    def run(command,**kwargs):
        model.calls.append(tuple(command))
        verb=command[2];role=next(k for k,v in u.UNITS.items() if v==command[-1])
        if verb=='start':
            rows[role].update(ActiveState='active',SubState='running',InvocationID=f'{len(model.calls):032x}',
                              MainPID=str(100+u.ROLES.index(role)),StateChangeTimestampMonotonic=str(time.monotonic_ns()//1000))
            if role=='owner':model.generation=time.monotonic()
            if role==getattr(model,'failed_after_start',None):rows[role].update(ActiveState='failed',SubState='failed',MainPID='0')
            if role==model.start_error:raise RuntimeError('simulated start acknowledgement unknown')
        elif verb=='stop':
            if role==model.stop_unit_error:raise RuntimeError('simulated stop outcome unknown')
            rows[role].update(ActiveState='inactive',SubState='dead',MainPID='0',StateChangeTimestampMonotonic=str(time.monotonic_ns()//1000))
        else:pytest.fail('unexpected command '+str(command))
        return S(stdout='')
    def rpc(record,endpoint,**kwargs):
        model.calls.append(('rpc',endpoint))
        if endpoint==model.stop_error:raise RuntimeError('simulated STOP unknown')
        if endpoint.startswith('/station/') and rows['apis']['ActiveState']!='active':raise RuntimeError('workflow API absent')
        if endpoint=='/station/stop':return {'request_id':'00000000-0000-0000-0000-000000000002'}
        if endpoint=='/station/stop_status':return dict(request_id='00000000-0000-0000-0000-000000000002',pending=False,ev3_stopped=True,arm_disarmed=True,**{}) | getattr(model,'station_status_update',{})
        if endpoint=='/follower/status':
            now=time.monotonic()
            return dict(expected,kind='follower_status',fresh=True,connected=True,owner_faulted=False,
                        owner_started_monotonic=model.generation,sampled_monotonic=now,position_monotonic=now,torque_monotonic=now,
                        torque=dict.fromkeys(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'),0),external_job_pending=False) | getattr(model,'owner_flags',{})
    monkeypatch.setattr(station,'run',run);monkeypatch.setattr(station,'native_rpc',rpc)
    model.path=tmp_path/'record.json';model.record=station.prepare_unit_start(a,[m['wrapper'],'ros-vulkan'])
    return model


def test_fake_unit_start_stop_owns_one_group_and_never_requests_motion(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record)
    assert f.path.exists();assert f.record['phase']=='running all OFF'
    starts=[c[-1] for c in f.calls if c[:3]==('systemctl','--user','start')]
    assert starts==list(f.u.UNITS.values())
    assert sum(c==('rpc','/station/ev3/connect') for c in f.calls)==1
    station.stop_record(f.path,f.record)
    assert not f.path.exists()
    assert len([c for c in f.calls if c[:3]==('systemctl','--user','stop')])==8
    assert not any('/home' in c or '/rearm_motion_profile' in c for c in f.calls)


def test_start_failure_cleans_only_proven_units_after_stop(fake_manager):
    f=fake_manager;f.start_error='apis'
    with pytest.raises(RuntimeError,match='start acknowledgement'):station.start_units(f.args,f.path,f.record)
    assert not f.path.exists()
    starts={c[-1] for c in f.calls if c[:3]==('systemctl','--user','start')}
    stops={c[-1] for c in f.calls if c[:3]==('systemctl','--user','stop')}
    assert starts==stops
    assert ('rpc','/follower/stop') in f.calls


def test_unknown_stop_ack_attempts_independent_follower_but_sends_no_unit_stop(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear();f.stop_error='/station/stop'
    with pytest.raises(RuntimeError,match='STOP acknowledgement'):station.stop_record(f.path,f.record)
    assert ('rpc','/follower/stop') in f.calls
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)
    assert f.path.exists() and json.loads(f.path.read_text())['stop_errors']


def test_changed_manager_or_unproved_invocation_sends_no_rpc_or_stop(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear()
    f.manager['dbus_owner']=':1.4'
    with pytest.raises(RuntimeError,match='manager identity'):station.stop_record(f.path,f.record)
    assert f.calls==[] and f.path.exists()
    f.manager['dbus_owner']=':1.3';f.rows['camera']['InvocationID']='f'*32
    with pytest.raises(RuntimeError,match='receipt'):station.stop_record(f.path,f.record)
    assert ('rpc','/follower/stop') in f.calls and f.path.exists()
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)


def test_unknown_manager_stop_retains_exact_record_and_never_stops_owner(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear();f.stop_unit_error='scene'
    with pytest.raises(RuntimeError,match='stop outcome'):station.stop_record(f.path,f.record)
    assert f.path.exists() and json.loads(f.path.read_text())['stopping']=='scene'
    assert not any(c[:3]==('systemctl','--user','stop') and c[-1]==f.u.UNITS['owner'] for c in f.calls)


def test_legitimate_existing_manager_receipt_allows_restarted_component_cleanup(fake_manager):
    import time
    f=fake_manager;station.start_units(f.args,f.path,f.record)
    role='camera';before=dict(f.rows[role]);requested=time.monotonic_ns()//1000
    f.rows[role].update(InvocationID='f'*32,StateChangeTimestampMonotonic=str(requested))
    props=('Id','LoadState','ActiveState','SubState','Job','InvocationID','StateChangeTimestampMonotonic')
    pending=dict(schema='operator-restart-pending-v1',job_id='e'*32,token='d'*32,
                 owner_generation=f.record['owner_identity']['owner_started_monotonic'],
                 **f.record['expected_owner'],restart_source_sha256={name:f.manifest['sources']['ros2/src/so101_station_control/so101_station_control/'+name+'.py'] for name in ('operator_restart','trajectory_server')},
                 unit=f.u.UNITS[role],requested_monotonic_us=requested,boot_id=f.manager['boot_id'],before={k:before[k] for k in props})
    proof=dict(job_id=pending['job_id'],unit=pending['unit'],owner_generation=pending['owner_generation'],
               kind='restart_unit_terminal',success=True,after={k:f.rows[role][k] for k in props})
    journal=Path(f.manifest['parameters']['operator_maintenance_journal'])
    journal.with_name(journal.name+'.resolved-'+pending['job_id']).write_text(json.dumps(dict(pending=pending,proof=proof)))
    station.stop_record(f.path,f.record)
    assert not f.path.exists()


def test_user_unit_parser_preserves_venv_python_symlink(tmp_path):
    target=tmp_path/'python-real';target.write_text('fixture')
    link=tmp_path/'venv/bin/python';link.parent.mkdir(parents=True);link.symlink_to(target)
    a=station.parse_arguments(['start','--profile','fwmini4','--deployment','user-units','--python',str(link)])
    assert a.python==link.absolute()


@pytest.mark.parametrize('update',[{'pending':True},{'request_id':'wrong'},{'request_id':None},{'ev3_stopped':False},{'arm_disarmed':False}])
def test_station_stop_requires_exact_terminal_correlated_status(fake_manager,monkeypatch,update):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear()
    f.station_status_update=update
    monkeypatch.setattr(station,'STATION_STOP_TIMEOUT',.05,raising=False)
    with pytest.raises(RuntimeError):station.stop_record(f.path,f.record)
    assert ('rpc','/follower/stop') in f.calls
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)
    assert f.path.exists()


def test_faulted_pending_owner_still_gets_both_stops_before_cleanup_barrier(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear()
    f.owner_flags={'owner_faulted':True,'external_job_pending':True}
    with pytest.raises(RuntimeError):station.stop_record(f.path,f.record)
    assert ('rpc','/station/stop') in f.calls and ('rpc','/follower/stop') in f.calls
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)
    assert f.path.exists()


def test_failure_before_api_stops_follower_and_retains_prefix_for_inspection(fake_manager):
    f=fake_manager;f.start_error='objects'
    with pytest.raises(RuntimeError):station.start_units(f.args,f.path,f.record)
    data=json.loads(f.path.read_text())
    assert ('rpc','/follower/stop') in f.calls
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)
    assert 'workflow API absent' in data['cleanup_error'] and data['pending']['role']=='objects'


def test_new_failed_owner_invocation_never_loses_ownership_record(fake_manager):
    f=fake_manager;f.failed_after_start='owner'
    with pytest.raises(RuntimeError):station.start_units(f.args,f.path,f.record)
    data=json.loads(f.path.read_text())
    assert data['pending']['role']=='owner'
    assert data['pending']['before']['InvocationID']!=f.rows['owner']['InvocationID']
    assert 'unproved hardware outcome' in data['cleanup_error']
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)


def test_faulted_but_known_disarmed_owner_can_be_explicitly_stopped(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record)
    f.owner_flags={'owner_faulted':True}
    station.stop_record(f.path,f.record)
    assert not f.path.exists()


def test_manager_identity_pins_bus_id_as_well_as_unique_name(monkeypatch):
    calls=[]
    def run(command,**kwargs):
        calls.append(command)
        return S(stdout='s "'+('0123456789abcdef'*2 if command[-1]=='GetId' else ':1.3')+'"\n')
    monkeypatch.setattr(station,'run',run)
    identity=station.manager_identity()
    assert identity['dbus_id']=='0123456789abcdef'*2
    assert len(calls)==2


def test_installed_component_overlay_must_match_source_before_devices_start(tmp_path,monkeypatch):
    assert hasattr(station,'verify_component_overlay'),'missing installed component parity guard'
    expected={
        'ros2/src/so101_station_bringup/launch/station_live.launch.py':'',
        'ros2/src/so101_station_control/launch/moveit_resident.launch.py':'',
        'ros2/src/so101_station_perception/launch/perception.launch.py':'',
        'ros2/src/so101_station_operator/launch/operator.launch.py':'',
        'ros2/src/so101_station_workflow/scripts/station_api':'',
        'ros2/src/so101_station_operator/scripts/operator_api':'',
        'ros2/src/so101_station_control/so101_station_control/operator_restart.py':'',
        'ros2/src/so101_station_control/so101_station_control/trajectory_server.py':''}
    mapping={path:str(ROOT/path) for path in expected}
    monkeypatch.setattr(station,'run',lambda *a,**kw:S(stdout=json.dumps(mapping)))
    station.verify_component_overlay(['/fixture/wrapper','ros-vulkan'])
    stale=tmp_path/'stale.launch.py';stale.write_text('stale')
    mapping[next(iter(expected))]=str(stale)
    with pytest.raises(RuntimeError,match='stale'):station.verify_component_overlay(['/fixture/wrapper','ros-vulkan'])


def test_native_stop_rpc_uses_compact_status_and_preserves_diagnostic_bound(monkeypatch,capsys):
    from types import ModuleType
    from tools.ros_station_api import station_stop_snapshot
    state = dict(request_id='00000000-0000-0000-0000-000000000002',pending=False,
                 ev3_stopped=True,arm_disarmed=True)
    diagnostics = dict(stop=state,workflow=dict(events=[dict(message='x'*1000)]*100),
                       ev3=dict(status='x'*70000))
    messages = {'/station/status':json.dumps(diagnostics),
                '/station/stop_status':json.dumps(station_stop_snapshot(state))}
    endpoints=[]
    class Node:
        def create_client(self,kind,endpoint):
            endpoints.append(endpoint)
            return S(wait_for_service=lambda **kw:True,call_async=lambda request:S(
                done=lambda:True,result=lambda:S(success=True,message=messages[endpoint])))
        def destroy_node(self):pass
    ros=ModuleType('rclpy')
    ros.init=lambda:None;ros.shutdown=lambda:None;ros.create_node=lambda name:Node()
    ros.spin_until_future_complete=lambda *a,**kw:None
    services=ModuleType('std_srvs.srv');services.Trigger=S(Request=lambda:None)
    monkeypatch.setitem(sys.modules,'rclpy',ros)
    monkeypatch.setitem(sys.modules,'std_srvs',ModuleType('std_srvs'))
    monkeypatch.setitem(sys.modules,'std_srvs.srv',services)
    monkeypatch.setattr(sys,'argv',['rpc','/station/status','8'])
    with pytest.raises(RuntimeError,match='oversize workflow result'):exec(station.NATIVE_RPC,{})
    monkeypatch.setattr(sys,'argv',['rpc','/station/stop_status','8'])
    exec(station.NATIVE_RPC,{})
    assert json.loads(capsys.readouterr().out)==dict(success=True,data=state)
    assert endpoints==['/station/status','/station/stop_status']


def test_missing_compact_stop_endpoint_retains_record_and_dispatches_no_unit_stops(fake_manager):
    f=fake_manager;station.start_units(f.args,f.path,f.record);f.calls.clear()
    f.stop_error='/station/stop_status'
    with pytest.raises(RuntimeError,match='STOP acknowledgement'):station.stop_record(f.path,f.record)
    assert ('rpc','/station/status') not in f.calls
    assert ('rpc','/follower/stop') in f.calls
    assert not any(c[:3]==('systemctl','--user','stop') for c in f.calls)
    assert f.path.exists() and json.loads(f.path.read_text())['stop_errors']


def test_manifest_pins_api_delegate_and_implementation(tmp_path):
    u=units();m=u.build_manifest(args(tmp_path))
    for source in ('tools/ros_station_api.py','ros2/src/so101_station_workflow/scripts/station_api'):
        assert source in m['sources']
        m['sources'][source]='0'*64
        with pytest.raises(ValueError,match='source cohort'):u.validate_manifest(m)
        m=u.build_manifest(args(tmp_path))
