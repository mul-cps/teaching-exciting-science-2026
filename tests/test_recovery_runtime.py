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


def fixture(tmp_path):
    arm=FakeArm(tmp_path, initial=target(elbow_flex=2993))
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
        depth_stamp_monotonic=now,acquisition_arm_stamp_monotonic=now,acquisition_arm_raw=raw or target(elbow_flex=2993),
        intrinsics=dict(fx=100.,fy=100.,ppx=49.5,ppy=49.5),distortion=[0.]*5,distortion_model='plumb_bob')


def runtime(tmp_path):
    config,arm,_,_=fixture(tmp_path)
    value=create_recovery_runtime(config,tmp_path/'state.json')
    value.close()  # deterministic preparation in these unit tests
    return value,config,arm


def write_frame(provider,raw):
    provider.cache_root.mkdir(exist_ok=True)
    generation=uuid.uuid4().hex
    buffer=io.BytesIO();np.savez(buffer,depth_m=np.ones((100,100),np.float32))
    payload=buffer.getvalue()
    (provider.cache_root/(generation+'.npz')).write_bytes(payload)
    metadata=record(provider,raw)
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
    config,arm,_,_=fixture(tmp_path)
    result=[]
    class Session(ArmSession):
        def serve(self,**kwargs):
            assert self.camera_scene_provider is not None
            provider=self.camera_scene_provider
            stop=threading.Event()
            def camera():
                while not stop.is_set():
                    write_frame(provider,dict(arm.bus.data['Present_Position']))
                    stop.wait(.025)
            worker=threading.Thread(target=camera)
            worker.start()
            until=time.monotonic()+1
            while provider._latest is None and time.monotonic()<until:time.sleep(.005)
            try:
                result.append(self.handle({'command':'recover_camera'}))
                return 0
            finally:stop.set();worker.join(1)
    assert run_locked_session(config,tmp_path/'state',tmp_path/'logs',lock_path=tmp_path/'lock',
        lock_factory=lambda _:nullcontext(),arm_factory=lambda _:arm,session_factory=Session)==0
    assert result[0]['ok'] and result[0]['state']['raw']['elbow_flex']==2973
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


def test_actual_module_entrypoint_factory_uses_shared_policy(tmp_path):
    """Execute exactly python -m owner; replace only device/config boundaries."""
    import os
    import subprocess
    import sys
    hook=tmp_path/'sitecustomize.py'
    hook.write_text('''
import os,sys,threading,time
from pathlib import Path
from test_recovery_runtime import fixture,write_frame
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
            def serve(self,**kwargs):
                provider=self.camera_scene_provider
                assert isinstance(provider.policy,g['CameraRecoveryPolicy'])
                from workshop.station.recovery_types import owner_source_sha256
                assert self._owner_source_sha256 == owner_source_sha256(Path(g['__file__']).parent)
                assert set(self._owner_source_sha256) == {'arm','arm_session','recovery_runtime','recovery_types'}
                stop=threading.Event()
                def collect():
                    while not stop.is_set():
                        write_frame(provider,dict(arm.bus.data['Present_Position']))
                        stop.wait(.025)
                worker=threading.Thread(target=collect);worker.start()
                try:
                    until=time.monotonic()+1
                    while provider._latest is None and time.monotonic()<until:time.sleep(.005)
                    result=self.handle({'command':'recover_camera'})
                    assert result['ok'] and arm.bus.goal_history
                    print('REAL_MODULE_FACTORY_PROOF_OK',flush=True)
                    return 0
                finally:stop.set();worker.join(1)
        g['run_locked_session']=lambda *a,**k:original(*a,**k,arm_factory=lambda c:arm,session_factory=Session)
sys.setprofile(trace)
''')
    env=dict(os.environ,RECOVERY_FIXTURE=str(tmp_path),
        PYTHONPATH=os.pathsep.join((str(tmp_path),str(Path.cwd()),str(Path.cwd()/'tests'))))
    result=subprocess.run([sys.executable,'-m','workshop.station.arm_session','--config',str(tmp_path/'unused'),
        '--state-file',str(tmp_path/'state'),'--logdir',str(tmp_path/'logs'),'--lock',str(tmp_path/'lock')],
        env=env,text=True,capture_output=True,timeout=5)
    assert result.returncode==0, result.stdout+result.stderr
    assert 'REAL_MODULE_FACTORY_PROOF_OK' in result.stdout


def test_clock_bracket_uncertainty_cannot_extend_depth_lifetime(tmp_path, monkeypatch):
    value, _, _ = runtime(tmp_path)
    frame=record(value)
    wall=10_000_000_000
    frame['depth_stamp_ns']=wall-247_000_000
    frame['depth_stamp_monotonic']=99.753
    frame['acquisition_arm_stamp_monotonic']=99.800
    clocks=iter([100.,100.,100.004])
    monkeypatch.setattr(time,'monotonic',lambda:next(clocks,100.004))
    monkeypatch.setattr(time,'time_ns',lambda:wall)
    with pytest.raises(ValueError,match='expired'):
        value.prepare(frame,np.ones((100,100),np.float32))
    assert value._latest is None
