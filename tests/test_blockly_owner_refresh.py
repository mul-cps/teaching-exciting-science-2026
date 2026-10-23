"""Refresh owner snapshots often enough without relaxing cached sample admission."""
import ast
from pathlib import Path
import threading
import pytest
from workshop.blockly_ros import ROSBackend

SOURCE=Path(__file__).resolve().parents[1]/'workshop/blockly_ros.py'


def test_owner_refresh_period_is_50ms_and_single_timer():
    tree=ast.parse(SOURCE.read_text())
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
           and isinstance(node.func,ast.Attribute) and node.func.attr=='create_timer'
           and len(node.args)>1 and ast.unparse(node.args[1])=='self._refresh_owner']
    assert len(calls)==1
    assert ast.literal_eval(calls[0].args[0])==.05


def owner(now,measurement_age=.2):
    return {'schema':1,'kind':'follower_status','sampled_monotonic':now,
            'fresh':True,'position_age_s':measurement_age,'torque_age_s':measurement_age,
            'arm_id':'fake','calibration_sha256':'a'*64,'owner_started_monotonic':1.,
            'owner_source_sha256':{'owner':'b'*64},'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}


def test_refresh_period_leaves_measurement_freshness_margin(monkeypatch):
    clock=[100.];monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:clock[0])
    backend=ROSBackend.__new__(ROSBackend);backend.lock=threading.Lock()
    backend.owner_cache=(100.,owner(100.))
    backend.latest={'positions':[0.]*5,'jaw':0.,'stamp':100.,'received':100.}
    backend.trigger=lambda *args,**kwargs:pytest.fail('sample must remain cache-only')
    clock[0]=100.05
    assert backend.sample()['identity']['arm_id']=='fake'
    clock[0]=100.11
    with pytest.raises(RuntimeError,match='owner measurements are stale'):backend.sample()


@pytest.mark.parametrize('field',['position_age_s','torque_age_s'])
def test_measurement_300ms_bound_is_not_relaxed(monkeypatch,field):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status=owner(100.,.01);status[field]=.301
    with pytest.raises(RuntimeError,match='owner measurements are stale'):ROSBackend._identity(status)


def test_snapshot_300ms_bound_is_not_relaxed(monkeypatch):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.301)
    with pytest.raises(RuntimeError,match='source snapshot is stale'):ROSBackend._identity(owner(100.,0.))


@pytest.mark.parametrize('field',['jaw_calibration_sha256','joint_calibration_sha256'])
@pytest.mark.parametrize('value',['','unknown','A'*64,'g'*64,False])
def test_identity_rejects_unknown_or_malformed_loaded_calibration(monkeypatch,field,value):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status={**owner(100.,0.),'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64,field:value}
    with pytest.raises(RuntimeError,match='calibration'):ROSBackend._identity(status)


@pytest.mark.parametrize('field',['jaw_calibration_sha256','joint_calibration_sha256'])
def test_same_arm_and_source_restart_with_changed_loaded_calibration_rejects_stroke(monkeypatch,field):
    from workshop.blockly_recording import Recorder,replay_points
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    before={**owner(100.,0.),'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}
    recorder=Recorder('stroke',ROSBackend._identity(before),origin='physical')
    recorder.add([0.]*5,0.,0.);recorder.add([.01]*5,0.,.1)
    after={**before,'owner_started_monotonic':2.,field:'e'*64}
    with pytest.raises(ValueError):
        replay_points(recorder.finish(),ROSBackend._identity(after),physical=True)


def test_owner_identity_remains_available_for_safe_recovery_without_jaw_calibration(monkeypatch):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status=owner(100.,0.)
    for field in ('jaw_calibration_sha256','joint_calibration_sha256'):status.pop(field)
    identity=ROSBackend._identity(status)
    assert identity['arm_id']=='fake'
    assert identity['jaw_calibration_sha256'] is None and identity['joint_calibration_sha256'] is None


@pytest.mark.parametrize('fault',[None,'torque','stop','owner_fault','stale'])
def test_uncalibrated_recovery_retains_existing_arm_and_stop_gates(monkeypatch,fault):
    from workshop.blockly_recording import JOINTS
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status={**owner(100.,0.),'jaw_calibration_sha256':None,'joint_calibration_sha256':None,
            'torque':dict.fromkeys(JOINTS+('gripper',),0),'owner_faulted':False,
            'stop_latched':False,'active_operation':None,'teleop_state':'disabled',
            'operator_external_job':None,'operator_maintenance_token':None}
    station={'workflow':{'state':'idle','fault':None},'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True}}
    if fault=='torque':status['torque']['gripper']=1
    if fault=='stop':station['stop']['pending']=True
    if fault=='owner_fault':status['owner_faulted']=True
    if fault=='stale':status['position_age_s']=.4
    backend=ROSBackend.__new__(ROSBackend)
    backend.action_lock=threading.Lock();backend.lock=threading.Lock()
    backend.active_handle=None;backend.pending_mutations=set();backend.stop=lambda:None
    backend.trigger=lambda endpoint,**kwargs:status if endpoint=='/follower/status' else station
    if fault is None:assert backend.recover()=={'recovered':True}
    else:
        with pytest.raises(RuntimeError):backend.recover()


def test_cached_calibration_identity_microbenchmark(monkeypatch,capsys):
    import statistics,time
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status=owner(100.,0.);samples=[]
    for _ in range(1000):
        start=time.perf_counter_ns();ROSBackend._identity(status);samples.append(time.perf_counter_ns()-start)
    p95=sorted(samples)[949]/1000
    print(f'cached_owner_calibration_identity median_us={statistics.median(samples)/1000:.2f} p95_us={p95:.2f}')
    assert p95<10000
