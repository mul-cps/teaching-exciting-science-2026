"""Trusted runtime factory and actual acquisition-oracle bindings, no hardware."""
import hashlib
import io
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import pytest

from test_arm_session import FakeArm, target
from test_recovery_depth_scene import urdf, calibration, model
from workshop.station.recovery_runtime import create_recovery_runtime, BoundRecoveryScene



class FactoryFixtureClock:
    """Logical sensor time only; host threads, waits and CPU timing stay real."""
    def __init__(self):
        self.host=time;self.value=1000.;self.origin=time.time_ns()
    def monotonic(self):return self.value
    def time_ns(self):return self.origin+round((self.value-1000.)*1e9)
    def advance(self,seconds):self.value+=seconds
    def utc(self):
        from datetime import datetime,timezone
        return datetime.fromtimestamp(self.time_ns()/1e9,tz=timezone.utc)
    def __getattr__(self,key):return getattr(self.host,key)


def factory_fixture_clock():
    """Test-only context; never patch attributes of the global time module."""
    from contextlib import contextmanager
    import importlib
    import sys
    @contextmanager
    def scoped():
        modules=[sys.modules[__name__]]+[importlib.import_module('workshop.station.'+name)
            for name in ('recovery_runtime','camera_recovery','recovery_depth_scene','recovery_collision_model')]
        previous=[module.time for module in modules];clock=FactoryFixtureClock()
        try:
            for module in modules:module.time=clock
            yield clock
        finally:
            for module,original in zip(modules,previous):module.time=original
    return scoped()


def prepare_factory_fixture_ticks(session,clock):
    """Keep real factory/model/recovery but schedule fake acquisitions explicitly.

    Async cache-worker tests exercise the real thread separately. Factory wiring
    tests do not measure whether the host can prepare scenes within50ms.
    """
    provider=session.camera_scene_provider
    provider.close();provider.prewarm_model(dict(session.arm.bus.data['Present_Position']))
    def source_tick():
        clock.advance(.001)  # a genuinely new post-prewarm ORIGINAL exposure
        write_arm_frame(provider,session.arm)
        metadata=json.loads((provider.cache_root/'latest.json').read_text())
        provider.prepare(metadata,np.ones((100,100),np.float32));provider._read_health()
    def sensor_sleep(seconds):clock.advance(seconds);source_tick()
    session.sleep=sensor_sleep;source_tick()
    return provider

def fixture(tmp_path):
    arm=FakeArm(tmp_path, initial=target(elbow_flex=2993))
    arm.bus._recovery_capture_lock=threading.Lock()
    original_read=arm.bus.sync_read
    def synchronized_read(*args,**kwargs):
        with arm.bus._recovery_capture_lock:return original_read(*args,**kwargs)
    arm.bus.sync_read=synchronized_read
    arm.settings['limits']['elbow_flex']['max']=(2975.1-2027.5)*360/4095
    config={'arm':arm.settings}
    mapping=calibration(elbow_flex={'urdf_joint':'elbow_flex_joint','raw_reference':2993,
        'position_reference':0.,'units_per_count':.001,'unit':'radians'})
    from workshop.station.recovery_collision_model import URDFCollisionModel
    collision=URDFCollisionModel(urdf(),joint_calibration=mapping,calibration_qualified=True,base_frame='base_link')
    def pin(name,value):
        path=tmp_path/name
        data=value.encode() if isinstance(value,str) else json.dumps(value).encode()
        path.write_bytes(data)
        return {'path':str(path),'sha256':hashlib.sha256(data).hexdigest()}
    mapping_pin=pin('mapping.json',dict(schema='recovery-joint-mapping-v1',qualified=True,arm_id='rig',
        arm_calibration_sha256=arm.settings['calibration']['sha256'],joints=mapping))
    registration=dict(schema='camera-arm-calibration-v1',qualified=True,arm_calibration_sha256=arm.settings['calibration']['sha256'],
        model_sha256=collision.model_sha256,base_frame='base_link',camera_serial='synthetic-qualified',
        camera_frame='camera_optical_frame',base_from_camera=np.eye(4).tolist(),
        image_width=100,image_height=100,intrinsics=dict(fx=100.,fy=100.,ppx=49.5,ppy=49.5),
        distortion=[0.]*5,distortion_model='plumb_bob')
    manifest=dict(schema='so101-camera-recovery-runtime-v1',qualified=True,arm_id='rig',arm_calibration_sha256=arm.settings['calibration']['sha256'],
        joint_mapping=mapping_pin,effective_urdf=pin('effective.urdf',urdf()),registration=pin('registration.json',registration),
        model_sha256=collision.model_sha256,base_frame='base_link',allowed_self_contacts=[],support_planes=[],
        depth_uncertainty_m=.002,geometry_uncertainty_m=.001)
    config['arm']['camera_recovery']={'enabled':True,'manifest':pin('runtime.json',manifest)}
    return config,arm,manifest,registration


def record(runtime,raw=None,age=0.):
    now=time.monotonic()-age
    return dict(schema='recovery-depth-acquisition-v1',camera_serial='synthetic-qualified',camera_frame='camera_optical_frame',
        arm_id='rig',arm_calibration_sha256=runtime.policy.arm_calibration_sha256,source_clock='system',
        depth_stamp_ns=time.time_ns()-int(age*1e9),
        depth_stamp_monotonic=now,
        camera_identity=dict(schema='realsense-camera-identity-v1',serial_number='synthetic-qualified',
            serial_no_parameter='synthetic-qualified',generation='a'*64,
            request_started_monotonic=runtime.runtime_started_monotonic-.002,
            confirmed_monotonic=runtime.runtime_started_monotonic-.001),
        acquisition_owner_source_sha256=runtime.settings['owner_source_sha256'],
        acquisition_owner_started_monotonic=runtime.runtime_started_monotonic-.001,
        acquisition_arm_stamp_monotonic=now,acquisition_arm_raw=raw or target(elbow_flex=2993),
        intrinsics=dict(fx=100.,fy=100.,ppx=49.5,ppy=49.5),distortion=[0.]*5,distortion_model='plumb_bob')


def runtime(tmp_path):
    config,arm,_,_=fixture(tmp_path)
    value=create_recovery_runtime(config,tmp_path/'state.json')
    value.close()  # deterministic preparation in these unit tests
    return value,config,arm


def write_arm_frame(provider,arm):
    # Snapshot and original clock atomically against fake-bus position updates.
    # Serialization happens after releasing the fake transport lock.
    with arm.bus._recovery_capture_lock:
        acquired=(dict(arm.bus.data['Present_Position']),time.monotonic(),time.time_ns())
    write_frame(provider,acquired[0],acquisition=acquired[1:])


def write_frame(provider,raw,*,acquisition=None):
    provider.cache_root.mkdir(parents=True,exist_ok=True)
    generation=uuid.uuid4().hex
    buffer=io.BytesIO();np.savez(buffer,depth_m=np.ones((100,100),np.float32))
    payload=buffer.getvalue()
    (provider.cache_root/(generation+'.npz')).write_bytes(payload)
    metadata=record(provider,raw)
    now=time.monotonic()
    if acquisition is not None:
        original_mono,original_ns=acquisition
        metadata.update(depth_stamp_monotonic=original_mono,depth_stamp_ns=original_ns,
                        acquisition_arm_stamp_monotonic=original_mono)
    else:original_mono=metadata['depth_stamp_monotonic']
    # Model an actual query lease, not a new confirmation for every frame.
    identity=getattr(provider,'_fixture_identity',None)
    refreshed=False
    if identity is None:
        identity=dict(metadata['camera_identity'])
        provider._fixture_identity=identity
    elif original_mono-identity['request_started_monotonic']>=.500:
        identity=dict(metadata['camera_identity'],request_started_monotonic=original_mono-.003,
                      confirmed_monotonic=original_mono-.002)
        provider._fixture_identity=identity
        provider._fixture_refresh_request=identity['request_started_monotonic']
    metadata['camera_identity']=dict(identity)
    health=dict(schema='recovery-depth-health-v1',cache_generation=provider.cache_generation,
        runtime_started_monotonic=provider.runtime_started_monotonic,
        owner_started_monotonic=provider.runtime_started_monotonic-.001,
        owner_source_sha256=provider.settings['owner_source_sha256'],arm_id='rig',
        arm_calibration_sha256=provider.policy.arm_calibration_sha256,
        camera_identity=metadata['camera_identity'],observed_monotonic=now,state='ready',
        streams={k:dict(header_ns=metadata['depth_stamp_ns'],received_monotonic=now) for k in ('depth','info')},
        same_generation_refresh_request_started_monotonic=getattr(provider,'_fixture_refresh_request',None))
    healthtmp=provider.cache_root/'health.tmp';healthtmp.write_text(json.dumps(health))
    healthtmp.replace(provider.cache_root/'health.json')
    metadata.update(generation=generation,depth_sha256=hashlib.sha256(payload).hexdigest())
    temporary=provider.cache_root/'latest.tmp'
    temporary.write_text(json.dumps(metadata))
    temporary.replace(provider.cache_root/'latest.json')


def test_binding_changed_current_raw_preserves_actual_acquisition_and_carving(tmp_path):
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    original=(scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw),scene._current_interior.copy())
    current=target(elbow_flex=2989)
    stamp=time.monotonic()
    bound=value(current,stamp)
    assert isinstance(bound,BoundRecoveryScene) and bound.arm_raw==current
    assert bound.arm_stamp_monotonic==stamp
    assert bound.acquisition_arm_raw==target(elbow_flex=2993)
    assert bound.minimum_clearance_m(current)==.01
    assert bound.minimum_segment_clearance_m(current,target(elbow_flex=2987))==.01
    assert (scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw))==original[:3]
    assert np.array_equal(scene._current_interior,original[3])
    # A materially different candidate is still evaluated, not carved as self.
    assert bound.minimum_clearance_m(target(elbow_flex=3990)) is None


@pytest.mark.parametrize('fault',['stale','camera','arm','pose','pair_stamp','occluded'])
def test_bad_acquisition_cannot_produce_free_proof(tmp_path,fault):
    value,_,_=runtime(tmp_path)
    frame=record(value,age=.3 if fault=='stale' else 0.)
    if fault=='camera':frame['camera_serial']='other'
    if fault=='arm':frame['arm_calibration_sha256']='b'*64
    if fault=='pose':frame['acquisition_arm_raw']={'gripper':2000}
    if fault=='pair_stamp':frame['acquisition_arm_stamp_monotonic']-=.2
    depth=np.ones((100,100),np.float32)
    if fault=='occluded':depth[40:60,40:60]=0
    if fault!='occluded':
        with pytest.raises(ValueError): value.prepare(frame,depth)
        assert value(target(elbow_flex=2993),time.monotonic()) is None
    else:
        value.prepare(frame,depth)
        bound=value(target(elbow_flex=2993),time.monotonic())
        assert bound.minimum_clearance_m(dict(bound.arm_raw)) is None


def test_factory_default_disabled_and_unqualified_rejected(tmp_path):
    assert create_recovery_runtime({'arm':{}},tmp_path/'state') is None
    config,_,manifest,_=fixture(tmp_path)
    manifest['qualified']=False
    data=json.dumps(manifest).encode();path=tmp_path/'runtime.json';path.write_bytes(data)
    config['arm']['camera_recovery']['manifest']['sha256']=hashlib.sha256(data).hexdigest()
    with pytest.raises(ValueError,match='qualified'): create_recovery_runtime(config,tmp_path/'state')


def test_rebinding_does_not_refresh_original_depth_or_pair_age(tmp_path):
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    source=(scene.stamp_monotonic,scene.arm_stamp_monotonic)
    bound=value(target(elbow_flex=2991),time.monotonic())
    time.sleep(.26)
    assert value(target(elbow_flex=2989),time.monotonic()) is None
    assert source==(scene.stamp_monotonic,scene.arm_stamp_monotonic)
    # The old binding also expires its independent250ms proof deadline.
    assert bound.minimum_clearance_m(dict(bound.arm_raw)) is None


def test_run_locked_session_factory_to_prepared_provider_to_real_proof(tmp_path):
    from contextlib import nullcontext
    from workshop.station.arm_session import ArmSession,run_locked_session
    with factory_fixture_clock() as clock:
        config,arm,_,_=fixture(tmp_path)
        result=[]
        class Session(ArmSession):
            def __init__(self,*args,**kwargs):
                kwargs.update(clock=clock.monotonic,wall_clock=clock.utc)
                super().__init__(*args,**kwargs)
            def serve(self,**kwargs):
                assert self.camera_scene_provider is not None
                prepare_factory_fixture_ticks(self,clock)
                result.append(self.handle({'command':'recover_camera'}))
                return 0
        assert run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
            lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm,session_factory=Session)==0
        assert result[0]['ok'] and result[0]['state']['raw']['elbow_flex']==2973,result[0]
        assert arm.bus.goal_history and arm.bus.eeprom_writes==[]


@pytest.mark.parametrize('fault',['missing','stale','occluded','provider_slow','proof_slow','binding_mismatch'])
def test_full_factory_owner_rejections_have_zero_goal_writes(tmp_path,monkeypatch,fault):
    from contextlib import nullcontext
    from dataclasses import replace
    from workshop.station.arm_session import ArmSession,ArmPlanPreflightError,run_locked_session
    from workshop.station.recovery_runtime import RecoveryRuntime
    config,arm,_,_=fixture(tmp_path)
    original_call=RecoveryRuntime.__call__
    def acquire(self,raw,stamp):
        if fault=='provider_slow':time.sleep(.055)
        bound=original_call(self,raw,stamp)
        if fault=='binding_mismatch' and bound is not None:
            return replace(bound,arm_raw=target(elbow_flex=2992))
        return bound
    monkeypatch.setattr(RecoveryRuntime,'__call__',acquire)
    if fault=='proof_slow':
        original=BoundRecoveryScene.minimum_clearance_m
        def slow(self,raw):time.sleep(.26);return original(self,raw)
        monkeypatch.setattr(BoundRecoveryScene,'minimum_clearance_m',slow)
    class Session(ArmSession):
        def serve(self,**kwargs):
            provider=self.camera_scene_provider
            provider.close()
            if fault!='missing':
                depth=np.ones((100,100),np.float32)
                if fault=='occluded':depth[40:60,40:60]=0
                provider.prepare(record(provider),depth)
                if fault=='stale':time.sleep(.26)
            with pytest.raises(ArmPlanPreflightError):self.handle({'command':'recover_camera'})
            assert not arm.bus.goal_history
            return 0
    assert run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
        lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm,session_factory=Session)==0
    assert not arm.bus.goal_history and not arm.bus.eeprom_writes


@pytest.mark.parametrize('fault',['unqualified','mismatched','hash','uncertainty','mapping'])
def test_factory_rejects_bad_authority_before_owner_connect(tmp_path,fault):
    from contextlib import nullcontext
    from workshop.station.arm_session import run_locked_session
    config,arm,manifest,_=fixture(tmp_path)
    if fault=='unqualified':manifest['qualified']=False
    if fault=='mismatched':manifest['arm_calibration_sha256']='b'*64
    if fault=='uncertainty':manifest['geometry_uncertainty_m']=0
    if fault=='mapping':manifest['joint_mapping']['sha256']='b'*64
    payload=json.dumps(manifest).encode();(tmp_path/'runtime.json').write_bytes(payload)
    config['arm']['camera_recovery']['manifest']['sha256']=hashlib.sha256(payload).hexdigest()
    if fault=='hash':config['arm']['camera_recovery']['manifest']['sha256']='c'*64
    with pytest.raises(ValueError):
        run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
            lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm)
    assert arm.connect_calls==0 and not arm.bus.writes and not arm.bus.goal_history


def test_provider_lookup_has_no_file_or_model_work_on_owner_tick(tmp_path,monkeypatch):
    import ast,inspect,textwrap
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    def forbidden(*args,**kwargs):raise AssertionError('expensive preparation on provider tick')
    monkeypatch.setattr(Path,'read_bytes',forbidden)
    monkeypatch.setattr(type(value.model),'voxel_volume',forbidden)
    tree=ast.parse(textwrap.dedent(inspect.getsource(type(value).__call__)))
    assert not any(isinstance(n,ast.Attribute) and n.attr in {'read_bytes','read_text','load','prepare','voxel_volume','sha256'} for n in ast.walk(tree))
    for index in range(100):
        bound=value(target(elbow_flex=2993-index%4),time.monotonic())
        assert bound is not None and bound.acquisition is scene


def test_prepared_acquisitions_isolate_mutable_geometry_caches(tmp_path):
    value,_,_=runtime(tmp_path)
    first=value.prepare(record(value),np.ones((100,100),np.float32))
    second=value.prepare(record(value,target(elbow_flex=2991)),np.ones((100,100),np.float32))
    assert first.model is not value.model and second.model is not value.model
    assert first.model._volume_cache is not second.model._volume_cache
    assert first.model._volume_cache is not value.model._volume_cache
    assert first.model.shapes is value.model.shapes  # immutable material reused
    assert first.model.model_sha256==second.model.model_sha256==value.model.model_sha256


def test_bad_new_acquisition_revokes_previous_free_scene(tmp_path):
    value,_,_=runtime(tmp_path)
    value.prepare(record(value),np.ones((100,100),np.float32))
    changed=record(value);changed['camera_serial']='wrong'
    with pytest.raises(ValueError):value.prepare(changed,np.ones((100,100),np.float32))
    assert value(target(elbow_flex=2993),time.monotonic()) is None


@pytest.mark.parametrize('fault', ['ancient', 'future', 'inconsistent', 'unsupported_clock'])
def test_invalid_depth_header_full_owner_zero_goals(tmp_path, fault):
    from contextlib import nullcontext
    from workshop.station.arm_session import ArmSession, ArmPlanPreflightError, run_locked_session
    config, arm, _, _ = fixture(tmp_path)
    class Session(ArmSession):
        def serve(self, **kwargs):
            provider = self.camera_scene_provider
            provider.close()
            frame = record(provider)
            if fault == 'ancient': frame['depth_stamp_ns'] = 1
            if fault == 'future': frame['depth_stamp_ns'] += 1_000_000_000
            if fault == 'inconsistent': frame['depth_stamp_monotonic'] -= .100
            if fault == 'unsupported_clock': frame['source_clock'] = 'sim'
            with pytest.raises(ValueError, match='SYSTEM depth header'):
                provider.prepare(frame, np.ones((100,100),np.float32))
            with pytest.raises(ArmPlanPreflightError):
                self.handle({'command':'recover_camera'})
            assert not arm.bus.goal_history
            return 0
    assert run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
        lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm,session_factory=Session) == 0
    assert not arm.bus.goal_history


def test_passive_bad_optional_recovery_keeps_explicit_stop_available(tmp_path):
    from contextlib import nullcontext
    from workshop.station.arm_session import ArmSession, run_locked_session
    config, arm, _, _ = fixture(tmp_path)
    (tmp_path/'runtime.json').unlink()
    arm.bus.data['Torque_Enable']['elbow_flex'] = 1
    arm.connect = lambda **kwargs: setattr(arm.bus, 'is_connected', True)
    class Session(ArmSession):
        def serve(self, **kwargs):
            assert self._observe_only and self.camera_scene_provider is None
            assert not arm.bus.writes
            assert self.read_state()['torque']['elbow_flex'] == 1
            result = self.handle({'command':'stop'})
            assert result['stopped'] and not any(result['state']['torque'].values())
            assert all(reg=='Torque_Enable' and value==0 for reg,_,value in arm.bus.writes)
            return 0
    assert run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
        observe_only=True,lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm,
        session_factory=Session) == 0
    assert 'camera_recovery_disabled' in ''.join(p.read_text() for p in (tmp_path/'logs').glob('*.jsonl'))


def test_prewarm_drops_triggering_cohort_and_requires_new_header(tmp_path):
    value, _, _ = runtime(tmp_path)
    old = record(value)
    value.prewarm_model(old['acquisition_arm_raw'])
    assert value._latest is None and value.prewarm_duration_s < 2
    # Exercise the production worker with a pre-prewarm header in a NEW cohort.
    write_frame(value,old['acquisition_arm_raw'])
    path=value.cache_root/'latest.json'
    metadata=json.loads(path.read_text());metadata['depth_stamp_ns']=old['depth_stamp_ns']
    path.write_text(json.dumps(metadata))
    value._stop.clear()
    worker=threading.Thread(target=value._collect);worker.start()
    try:
        time.sleep(.04)
        assert value._latest is None and 'post-prewarm' in value.last_error
        write_frame(value,old['acquisition_arm_raw'])
        until=time.monotonic()+.2
        while value._latest is None and time.monotonic()<until:time.sleep(.005)
        assert value._latest is not None
        assert value._latest.acquisition_clock_provenance['depth_stamp_ns'] > value._prewarm_finished_header_ns
    finally:value._stop.set();worker.join(1)


@pytest.mark.parametrize('host_pause_s',[0.,.150])
def test_actual_module_entrypoint_factory_uses_shared_policy(tmp_path,host_pause_s):
    """Real -m factory/recovery; deterministic fake sensor and scheduler ticks.

    The asynchronous cache reader has separate real-thread coverage above. This
    test checks factory/policy wiring and actual recovery, not50ms host latency.
    """
    import os
    import subprocess
    import sys
    hook=tmp_path/'sitecustomize.py'
    hook.write_text('''
import os,sys,time as host_time
from pathlib import Path
from test_recovery_runtime import fixture,factory_fixture_clock,prepare_factory_fixture_ticks
clock_scope=factory_fixture_clock();clock=clock_scope.__enter__()
# This child owns its test-only module bindings until process teardown.
import atexit
atexit.register(clock_scope.__exit__,None,None,None)
folder=Path(os.environ['RECOVERY_FIXTURE'])
config,arm,_,_=fixture(folder)
def trace(frame,event,arg):
    if event=='call' and frame.f_code.co_name=='main' and frame.f_globals.get('__name__')=='__main__':
        sys.setprofile(None)
        g=frame.f_globals
        g['load_config']=lambda *a,**k:config
        g['validate_arm_session_config']=lambda c:None
        original=g['run_locked_session']
        class Session(g['ArmSession']):
            def __init__(self,*args,**kwargs):
                kwargs.update(clock=clock.monotonic,wall_clock=clock.utc)
                super().__init__(*args,**kwargs)
            def serve(self,**kwargs):
                provider=self.camera_scene_provider
                assert isinstance(provider.policy,g['CameraRecoveryPolicy'])
                from workshop.station.recovery_types import owner_source_sha256
                assert self._owner_source_sha256 == owner_source_sha256(Path(g['__file__']).parent)
                assert set(self._owner_source_sha256) == {'arm','arm_session','recovery_runtime','recovery_types','recovery_runtime_config','camera_recovery','idle_telemetry'}
                prepare_factory_fixture_ticks(self,clock)
                original_stamp=provider._latest.stamp_monotonic
                host_time.sleep(float(os.environ['RECOVERY_HOST_PAUSE']))
                assert provider._latest.stamp_monotonic==original_stamp
                result=self.handle({'command':'recover_camera'})
                assert result['ok'] and arm.bus.goal_history,result
                # Explicit logical expiry still revokes the ORIGINAL proof.
                scene=provider._latest;generation=provider._health['camera_generation']
                clock.advance(.051)
                assert provider.recapture_health(generation) is None
                assert provider._latest is scene
                print('REAL_MODULE_FACTORY_PROOF_OK',flush=True)
                return 0
        g['run_locked_session']=lambda *a,**k:original(*a,**k,arm_factory=lambda c:arm,session_factory=Session)
sys.setprofile(trace)
''')
    env=dict(os.environ,RECOVERY_FIXTURE=str(tmp_path),RECOVERY_HOST_PAUSE=str(host_pause_s),
        PYTHONPATH=os.pathsep.join((str(tmp_path),str(Path.cwd()),str(Path.cwd()/'tests'))))
    result=subprocess.run([sys.executable,'-m','workshop.station.arm_session','--config',str(tmp_path/'unused'),
        '--state-file',str(tmp_path/'state'),'--logdir',str(tmp_path/'logs'),'--lock',str(tmp_path/'lock')],
        env=env,text=True,capture_output=True,timeout=5)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'REAL_MODULE_FACTORY_PROOF_OK' in result.stdout


def test_clock_bracket_uncertainty_cannot_extend_depth_lifetime(tmp_path, monkeypatch):
    value, _, _ = runtime(tmp_path)
    frame=record(value)
    wall=10_000_000_000
    frame['depth_stamp_ns']=wall-247_000_000
    frame['depth_stamp_monotonic']=99.753
    frame['acquisition_arm_stamp_monotonic']=99.800
    value.runtime_started_monotonic=99.700
    frame['acquisition_owner_started_monotonic']=99.600
    frame['camera_identity']['request_started_monotonic']=99.600
    frame['camera_identity']['confirmed_monotonic']=99.700
    clocks=iter([100.,100.,100.004])
    monkeypatch.setattr(time,'monotonic',lambda:next(clocks,100.004))
    monkeypatch.setattr(time,'time_ns',lambda:wall)
    with pytest.raises(ValueError,match='expired'):
        value.prepare(frame,np.ones((100,100),np.float32))
    assert value._latest is None


@pytest.mark.parametrize('fault',['source','old_owner','future_owner','nan_owner','prior_runtime_read'])
def test_acquisition_owner_generation_provenance_rejects(tmp_path,fault):
    value,_,_=runtime(tmp_path)
    frame=record(value)
    if fault=='source':frame['acquisition_owner_source_sha256']={}
    if fault=='old_owner':frame['acquisition_owner_started_monotonic']=value.runtime_started_monotonic+.000001
    if fault=='future_owner':frame['acquisition_owner_started_monotonic']=time.monotonic()+1
    if fault=='nan_owner':frame['acquisition_owner_started_monotonic']=float('nan')
    if fault=='prior_runtime_read':frame['acquisition_arm_stamp_monotonic']=value.runtime_started_monotonic-.001
    with pytest.raises(ValueError,match='identity-matched'):
        value.prepare(frame,np.ones((100,100),np.float32))
    assert value._latest is None


def test_new_runtime_synchronously_revokes_previous_generation_pointer(tmp_path):
    config,_,_,_=fixture(tmp_path)
    directory=tmp_path/'camera-recovery'/('f'*32);directory.mkdir(parents=True)
    pointer=directory/'latest.json';pointer.write_text('{"generation":"old"}')
    value=create_recovery_runtime(config,tmp_path/'state')
    try:
        assert value.cache_root!=directory and not (value.cache_root/'latest.json').exists()
        assert pointer.exists()  # stale isolated data grants no authority
    finally:value.close()


@pytest.mark.parametrize('fault',['missing','actual_serial','parameter_serial','stale','future','nan','unconfirmed_exposure','generation'])
def test_runtime_requires_actual_camera_identity_provenance(tmp_path,fault):
    value,_,_=runtime(tmp_path);frame=record(value)
    identity=frame['camera_identity']
    if fault=='missing':del frame['camera_identity']
    if fault=='actual_serial':identity['serial_number']='other'
    if fault=='parameter_serial':identity['serial_no_parameter']='other'
    if fault=='stale':identity['request_started_monotonic']=time.monotonic()-1.01
    if fault=='future':identity['confirmed_monotonic']=time.monotonic()+1
    if fault=='nan':identity['request_started_monotonic']=float('nan')
    if fault=='unconfirmed_exposure':identity['confirmed_monotonic']=time.monotonic()
    if fault=='generation':identity['generation']='bad'
    with pytest.raises(ValueError,match='actually observed camera'):
        value.prepare(frame,np.ones((100,100),np.float32))
    assert value._latest is None


def test_identity_lease_expiry_after_prepare_cannot_rebind(tmp_path,monkeypatch):
    value,_,_=runtime(tmp_path);frame=record(value)
    frame['camera_identity']['request_started_monotonic']=time.monotonic()-.900
    value.prepare(frame,np.ones((100,100),np.float32))
    monkeypatch.setattr(time,'monotonic',lambda:frame['camera_identity']['request_started_monotonic']+1.001)
    assert value(target(elbow_flex=2993),time.monotonic()) is None


@pytest.mark.parametrize('expiry',['acquisition','identity'])
def test_bound_proof_cannot_cross_original_expiry(tmp_path,monkeypatch,expiry):
    value,_,_=runtime(tmp_path);frame=record(value)
    if expiry=='identity':frame['camera_identity']['request_started_monotonic']=time.monotonic()-.900
    scene=value.prepare(frame,np.ones((100,100),np.float32))
    bound=value(dict(scene.arm_raw),time.monotonic())
    assert bound is not None
    from workshop.station.recovery_depth_scene import DepthRecoveryScene
    def slow(self,raw):
        monkeypatch.setattr(time,'monotonic',lambda:bound.valid_until_monotonic+.001)
        return .010
    monkeypatch.setattr(DepthRecoveryScene,'minimum_clearance_m',slow)
    assert bound.minimum_clearance_m(dict(scene.arm_raw)) is None


def test_retained_acquisition_binding_expires_without_timestamp_refresh(tmp_path,monkeypatch):
    value,_,_=runtime(tmp_path);frame=record(value)
    scene=value.prepare(frame,np.ones((100,100),np.float32))
    original=(scene.stamp_monotonic,scene.arm_stamp_monotonic)
    deadline=min(original)+.250
    monkeypatch.setattr(time,'monotonic',lambda:deadline-.000001)
    assert value(dict(scene.arm_raw),time.monotonic()) is not None
    monkeypatch.setattr(time,'monotonic',lambda:deadline+.000001)
    assert value(dict(scene.arm_raw),time.monotonic()) is None
    assert (scene.stamp_monotonic,scene.arm_stamp_monotonic)==original


@pytest.mark.parametrize('fault',[None,'pair100','current50','depth250','raw'])
def test_trusted_binding_separates_acquisition_sync_from_current_read(tmp_path,monkeypatch,fault):
    from workshop.station.camera_recovery import plan_bound_reentry
    from workshop.station.recovery_runtime import BoundRecoveryScene
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    now=scene.stamp_monotonic+.120  #80ms receipt +two20ms proof/setup stages
    current=dict(scene.arm_raw)
    stamp=now-(.050001 if fault=='current50' else 0.)
    if fault=='pair100':object.__setattr__(scene,'arm_stamp_monotonic',scene.stamp_monotonic-.100001)  #corruptedfixture
    if fault=='depth250':now=scene.stamp_monotonic+.250001;stamp=now
    if fault=='raw':current['wrist_roll']+=1
    bound=BoundRecoveryScene(scene,dict(scene.arm_raw),stamp,now)
    original=(scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw))
    monkeypatch.setattr(time,'monotonic',lambda:now)
    bounds={j:(1000,3000) for j in current};bounds['elbow_flex']=(1000,2975)
    firmware=dict.fromkeys(current,(500,3500))
    call=lambda:plan_bound_reentry(current,bounds,firmware,scene=bound,now=now,
        expected_model_sha256=value.model.model_sha256,
        expected_calibration_sha256=value.model.calibration_sha256,
        expected_camera_serial=scene.camera_serial)
    if fault:
        with pytest.raises(ValueError):call()
    else:assert call().target_raw['elbow_flex']<current['elbow_flex']
    assert original==(scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw))


def test_actual_owner_initial_and_setup_proofs_after_80ms_receipt(tmp_path,monkeypatch):
    from workshop.station.arm_session import ArmSession,ArmPlanPreflightError
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    original=(scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw),scene._current_interior.copy())
    config,arm,_,_=fixture(tmp_path)
    # Same fixture identity/model/calibration; fake clock controls receipt plus
    # two actual owner proof stages, not a single planner call with a comment.
    now=[scene.stamp_monotonic+.080]
    monkeypatch.setattr(time,'monotonic',lambda:now[0])
    checks=[]
    class Session(ArmSession):
        def _camera_recovery_check(self,raw,stamp,target=None):
            result=super()._camera_recovery_check(raw,stamp,target)
            checks.append((stamp,result.scene_stamp_monotonic))
            now[0]+=.020
            return result
    session=Session(arm,tmp_path/'owner.json',tmp_path/'ownerlogs',
        clock=lambda:now[0],camera_scene_provider=value,camera_recovery_policy=value.policy)
    session.initialize()
    monkeypatch.setattr(value,'recapture_health',lambda _: {'state':'ready'})
    def admission_reached(*args,**kwargs):
        assert len(checks)==2
        assert checks[1][0]-scene.stamp_monotonic==pytest.approx(.100)
        raise ValueError('test stops before authorized goal')
    monkeypatch.setattr(session,'_enable_current_goal',admission_reached)
    with pytest.raises(ArmPlanPreflightError,match='test stops'):
        session.handle({'command':'recover_camera'})
    assert len(checks)==2 and arm.bus.goal_history==[]
    assert original[:3]==(scene.stamp_monotonic,scene.arm_stamp_monotonic,dict(scene.arm_raw))
    assert np.array_equal(original[3],scene._current_interior)
    session.close_log()


@pytest.mark.parametrize('shape',[[],{'camera_identity':'x'},{'streams':[]},{'streams':{'depth':'x','info':{}}}])
def test_malformed_health_is_typed_unavailable_then_valid_can_recover(tmp_path,shape):
    value,_,_=runtime(tmp_path);value.prepare(record(value),np.ones((100,100),np.float32));write_frame(value,target(elbow_flex=2993))
    path=value.cache_root/'health.json';valid=path.read_bytes()
    path.write_text(json.dumps(shape))
    with pytest.raises(ValueError):value._read_health()
    path.write_bytes(valid);value._read_health()
    assert value.recapture_health('a'*64) is not None


@pytest.mark.parametrize('state',['ready','awaiting_post_confirmation_exposure'])
def test_runtime_geometry_fault_overrides_ready_health_until_new_success(tmp_path,state):
    value,_,_=runtime(tmp_path);value.prepare(record(value),np.ones((100,100),np.float32));write_frame(value,target(elbow_flex=2993));value._read_health()
    healthpath=value.cache_root/'health.json';health=json.loads(healthpath.read_text())
    health.update(state=state,same_generation_refresh_request_started_monotonic=health['camera_identity']['request_started_monotonic'])
    healthpath.write_text(json.dumps(health));value._read_health()
    assert value.recapture_health('a'*64) is not None
    bad=record(value);bad['camera_serial']='wrong'
    with pytest.raises(ValueError):value.prepare(bad,np.ones((100,100),np.float32))
    value._read_health()  #producerready cannot override runtimefault
    assert value.recapture_health('a'*64) is None
    value.prepare(record(value),np.ones((100,100),np.float32))
    assert value.recapture_health('a'*64) is not None


def test_recapture_health_accessor_has_no_filesystem_io(tmp_path,monkeypatch):
    value,_,_=runtime(tmp_path);value.prepare(record(value),np.ones((100,100),np.float32));write_frame(value,target(elbow_flex=2993));value._read_health()
    monkeypatch.setattr(Path,'read_bytes',lambda *_:pytest.fail('critical-path file read'))
    assert value.recapture_health('a'*64) is not None
    assert value.recapture_health('b'*64) is None


@pytest.mark.parametrize('fault',['generation','old','declared_old','pose'])
def test_actual_postwait_proof_rejects_scene_substitution(tmp_path,monkeypatch,fault):
    import copy
    from dataclasses import replace
    from workshop.station.arm_session import ArmSession
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    config,arm,_,_=fixture(tmp_path)
    session=ArmSession(arm,tmp_path/'owner.json',tmp_path/'logs',
        camera_scene_provider=value,camera_recovery_policy=value.policy)
    session.initialize();session._camera_recovery_command_generation='a'*64
    arm.bus.data['Torque_Enable']=dict.fromkeys(arm.bus.motors,1)
    hold=min(scene.stamp_monotonic,scene.acquisition_clock_provenance['declared_depth_stamp_monotonic'])-.001
    raw=dict(scene.arm_raw);stamp=time.monotonic();utc=session.wall_clock()
    monkeypatch.setattr(value,'recapture_health',lambda _: {'state':'ready'})
    count=[0]
    def provide(self,raw,stamp):
        count[0]+=1
        acquired=scene
        if count[0]>1:
            acquired=copy.copy(scene)
            provenance=dict(scene.acquisition_clock_provenance)
            if fault=='generation':provenance['camera_identity']=dict(provenance['camera_identity'],generation='b'*64)
            if fault=='old':object.__setattr__(acquired,'stamp_monotonic',hold-.001)
            if fault=='declared_old':provenance['declared_depth_stamp_monotonic']=hold-.001
            if fault=='pose':object.__setattr__(acquired,'arm_raw',dict(raw,wrist_roll=raw['wrist_roll']+1))
            object.__setattr__(acquired,'acquisition_clock_provenance',provenance)
        return BoundRecoveryScene(acquired,dict(raw),stamp,time.monotonic())
    monkeypatch.setattr(type(value),'__call__',provide)
    result=session._wait_camera_recapture(raw,hold,(raw,stamp,utc,dict.fromkeys(raw,1),stamp,utc),stamp,'a'*64)
    with pytest.raises(ValueError):session._camera_recovery_check(result[0],result[1])
    assert count[0]==2 and arm.bus.goal_history==[]
    session.close_log()


def test_delayed_fixture_writer_preserves_original_atomic_pose_and_time(tmp_path,monkeypatch):
    import sys
    value,_,arm=runtime(tmp_path)
    original=write_frame;captured=[]
    def delayed(provider,raw,*,acquisition=None):
        captured.append((dict(raw),acquisition))
        arm.bus.data['Present_Position']['elbow_flex']-=2
        time.sleep(.260)
        return original(provider,raw,acquisition=acquisition)
    monkeypatch.setattr(sys.modules[__name__],'write_frame',delayed)
    write_arm_frame(value,arm)
    data=json.loads((value.cache_root/'latest.json').read_text())
    assert data['acquisition_arm_raw']==captured[0][0]
    assert data['depth_stamp_monotonic']==captured[0][1][0]
    assert data['depth_stamp_ns']==captured[0][1][1]
    assert data['acquisition_arm_raw']!=arm.bus.data['Present_Position']
    with pytest.raises(ValueError):value.prepare(data,np.ones((100,100),np.float32))


@pytest.mark.parametrize('fault',['generation','runtime_preparation','awaiting_confirmation'])
def test_final_goal_match_rechecks_cached_health_without_extra_goal(tmp_path,monkeypatch,fault):
    from types import MappingProxyType
    from workshop.station.arm_session import ArmSession
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    write_frame(value,dict(scene.arm_raw));value._read_health()
    config,arm,_,_=fixture(tmp_path)
    session=ArmSession(arm,tmp_path/'owner.json',tmp_path/'logs',
        camera_scene_provider=value,camera_recovery_policy=value.policy)
    session.initialize();session._camera_recovery_command_generation='a'*64
    stamp=time.monotonic();raw=dict(scene.arm_raw)
    plan=session._camera_recovery_check(raw,stamp)
    if fault=='generation':value._health=MappingProxyType(dict(value._health,camera_generation='b'*64))
    elif fault=='runtime_preparation':value._runtime_preparation_fault=True
    else:value._health=MappingProxyType(dict(value._health,state='awaiting_post_confirmation_exposure'))
    with pytest.raises(RuntimeError,match='health revoked'):
        session._enable_current_goal(raw,time.monotonic(),'camera_reentry',
            on_goal_dispatch=lambda:session._camera_recovery_match(raw,stamp,plan.scene_stamp_monotonic,time.monotonic()))
    assert arm.bus.goal_history==[]
    session.close_log()


def test_real_oracle_full_joint_envelope_covers_asymmetric_clamped_waypoint(tmp_path):
    value,_,_=runtime(tmp_path)
    acquired=value.prepare(record(value),np.ones((100,100),np.float32))
    start=dict(acquired.arm_raw)
    destination=dict(start,elbow_flex=start['elbow_flex']-20,shoulder_pan=start['shoulder_pan']-4)
    waypoint=dict(start,elbow_flex=start['elbow_flex']-2,shoulder_pan=start['shoulder_pan']-2)
    full=value.model.maximum_displacement_m(start,destination)
    step=value.model.maximum_displacement_m(start,waypoint)
    assert 0<step<=full
    # This is the real whole-volume envelope oracle, not a line-only mock.
    assert acquired.minimum_segment_clearance_m(start,destination)==.010
    assert acquired.minimum_clearance_m(waypoint)==.010
    assert acquired.minimum_segment_clearance_m(start,waypoint)==.010


def test_staged_actual_proof_rejects_alternative_oracle_subclass(tmp_path,monkeypatch):
    import copy
    from workshop.station.arm_session import ArmSession
    from workshop.station.recovery_depth_scene import DepthRecoveryScene
    value,_,_=runtime(tmp_path)
    acquired=value.prepare(record(value),np.ones((100,100),np.float32))
    class Alternative(DepthRecoveryScene):pass
    alternate=copy.copy(acquired);object.__setattr__(alternate,'__class__',Alternative)
    config,arm,_,_=fixture(tmp_path)
    session=ArmSession(arm,tmp_path/'owner.json',tmp_path/'logs',
        camera_scene_provider=value,camera_recovery_policy=value.policy)
    session.initialize();session._camera_recovery_command_generation='a'*64
    monkeypatch.setattr(type(value),'__call__',lambda self,raw,stamp:BoundRecoveryScene(alternate,dict(raw),stamp,time.monotonic()))
    with pytest.raises(ValueError,match='full-envelope'):
        session._camera_recovery_check(dict(acquired.arm_raw),time.monotonic())
    assert arm.bus.goal_history==[]
    session.close_log()


@pytest.mark.parametrize('refresh',[False,True])
@pytest.mark.parametrize('missing',[False,True])
def test_ready_health_cannot_promote_old_prepared_cohort_after_confirmation(tmp_path,refresh,missing):
    from types import MappingProxyType
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    now=time.monotonic()
    request=scene.stamp_monotonic-.001
    value._health=MappingProxyType(dict(camera_generation='a'*64,state='ready',
        valid_until_monotonic=now+.050,identity_confirmed_monotonic=scene.stamp_monotonic+.001,
        identity_request_started_monotonic=request,
        same_generation_refresh_request_started_monotonic=request if refresh else None))
    if missing:value._latest=None
    result=value.recapture_health('a'*64)
    assert (result is not None and result['state']=='awaiting_post_confirmation_exposure') if refresh else result is None
    assert value._latest is (None if missing else scene) and scene.stamp_monotonic<now


@pytest.mark.parametrize('state',['pending_future_bracket','nonstationary_bracket','awaiting_post_confirmation_exposure'])
def test_live_hold_only_health_needs_no_prepared_scene_or_refresh_for_bracket(tmp_path,state):
    from types import MappingProxyType
    value,_,_=runtime(tmp_path)
    value._latest=None;value._runtime_preparation_fault=False
    now=time.monotonic()
    value._health=MappingProxyType(dict(camera_generation='a'*64,state=state,
        valid_until_monotonic=now+.050,identity_confirmed_monotonic=now-.100,
        identity_request_started_monotonic=now-.200,
        same_generation_refresh_request_started_monotonic=now-.200 if state=='awaiting_post_confirmation_exposure' else None))
    result=value.recapture_health('a'*64)
    assert dict(result)==dict(value._health) and result['state']==state
    assert value(target(elbow_flex=2993),time.monotonic()) is None
    assert value.recapture_health('b'*64) is None
    value._health=MappingProxyType(dict(value._health,valid_until_monotonic=now-.001))
    assert value.recapture_health('a'*64) is None
    value._health=MappingProxyType(dict(value._health,valid_until_monotonic=now+.050))
    value._runtime_preparation_fault=True
    assert value.recapture_health('a'*64) is None


@pytest.mark.parametrize('case',['nonstationary_bracket','awaiting_post_confirmation_exposure','ready','expired','wrong_generation','prior_fault','missing_payload'])
def test_missing_pointer_rereads_committed_transition_health_without_minting_authority(tmp_path,monkeypatch,case):
    value,_,_=runtime(tmp_path)
    value.prepare(record(value),np.ones((100,100),np.float32))
    write_frame(value,target(elbow_flex=2993));value._read_health()
    assert value._health['state']=='ready'
    value._runtime_preparation_fault=case=='prior_fault'
    healthpath=value.cache_root/'health.json';pointer=value.cache_root/'latest.json'
    original_stat=Path.stat;interleaved=[False]
    def stat(path,*args,**kwargs):
        if path==pointer and not interleaved[0]:
            interleaved[0]=True
            health=json.loads(healthpath.read_text())
            health['state']=case if case in {'ready','nonstationary_bracket','awaiting_post_confirmation_exposure'} else 'nonstationary_bracket'
            if case=='awaiting_post_confirmation_exposure':health['same_generation_refresh_request_started_monotonic']=health['camera_identity']['request_started_monotonic']
            if case=='expired':health['observed_monotonic']-=1.
            if case=='wrong_generation':health['cache_generation']='f'*32
            healthpath.write_text(json.dumps(health))  # typed commit FIRST
            if case=='missing_payload':
                metadata=json.loads(pointer.read_text())
                (value.cache_root/(metadata['generation']+'.npz')).unlink()
                value._prewarm_attempted=True;value._prewarm_finished_header_ns=0
            else:pointer.unlink()  # old ready snapshot now sees ENOENT
        return original_stat(path,*args,**kwargs)
    monkeypatch.setattr(Path,'stat',stat)
    value._stop.clear()
    monkeypatch.setattr(value._stop,'wait',lambda *_:value._stop.set())
    value._collect()
    assert interleaved[0] and value._latest is None
    allowed=case in {'nonstationary_bracket','awaiting_post_confirmation_exposure'}
    assert value._runtime_preparation_fault is (not allowed)
    assert (value.recapture_health('a'*64) is not None) is allowed
    assert value(target(elbow_flex=2993),time.monotonic()) is None


@pytest.mark.parametrize('marker',[False,True])
@pytest.mark.parametrize('state',['ready','pending_future_bracket','nonstationary_bracket'])
@pytest.mark.parametrize('cohort',['old','missing','fresh'])
def test_refresh_marker_precedes_hold_state_without_promoting_scene_authority(tmp_path,marker,state,cohort):
    from types import MappingProxyType
    value,_,_=runtime(tmp_path)
    scene=value.prepare(record(value),np.ones((100,100),np.float32))
    now=time.monotonic();request=scene.stamp_monotonic-.002
    confirmed=scene.stamp_monotonic+(.001 if cohort in {'old','missing'} else -.001)
    value._health=MappingProxyType(dict(camera_generation='a'*64,state=state,
        valid_until_monotonic=now+.050,identity_confirmed_monotonic=confirmed,
        identity_request_started_monotonic=request,
        same_generation_refresh_request_started_monotonic=request if marker else None))
    if cohort=='missing':value._latest=None
    result=value.recapture_health('a'*64)
    if cohort!='fresh' and marker:
        assert result['state']=='awaiting_post_confirmation_exposure'
    elif cohort!='fresh' and state=='ready':assert result is None
    else:assert result['state']==state
    assert value._latest is (None if cohort=='missing' else scene)
    if cohort=='missing':assert value(target(elbow_flex=2993),time.monotonic()) is None


def test_factory_clock_scope_does_not_patch_global_time_and_restores_modules():
    import workshop.station.recovery_runtime as module
    real_time=time;real_monotonic=real_time.monotonic;prior_binding=module.time
    with factory_fixture_clock() as clock:
        assert module.time is clock and real_time.monotonic is real_monotonic
        before=clock.monotonic();real_time.sleep(.015)
        assert clock.monotonic()==before
        clock.advance(.101)
        assert clock.monotonic()==pytest.approx(before+.101)
    assert module.time is prior_binding and time is real_time
    assert real_time.monotonic is real_monotonic
