"""ROS-message collector core tests without ROS, camera or hardware."""
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_control'))
from so101_station_control.recovery_depth_cache import RecoveryDepthCache,decode_depth
from test_recovery_runtime import fixture
from test_arm_session import target
from workshop.station.recovery_runtime_config import load_recovery_runtime_config
from workshop.station.recovery_types import owner_source_sha256


def collector(tmp_path,start_worker=False,nonce='1'*32):
    config,arm,_,_=fixture(tmp_path)
    trusted=load_recovery_runtime_config(config,tmp_path/'state')
    sources=owner_source_sha256(Path('workshop/station'))
    value=RecoveryDepthCache(None,trusted_config=trusted,expected_owner_sources=sources,
        state_file=tmp_path/'state',cache_generation=nonce,
        runtime_started_monotonic=time.monotonic()-.180,start_worker=start_worker)
    generation=('fake-owner',time.monotonic()-.2)
    value.invalidate_generation(generation)
    now=time.monotonic()
    value.observe_camera_identity(dict(schema='realsense-camera-identity-v1',
        serial_number='synthetic-qualified',serial_no_parameter='synthetic-qualified',
        generation='a'*64,request_started_monotonic=now-.170,confirmed_monotonic=now-.150))
    return value,arm,generation


def message(value,age=.050):
    ns=time.time_ns()-int(age*1e9)
    header=NS(stamp=NS(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000),
        frame_id=value.registration['camera_frame'])
    image=NS(header=header,width=100,height=100,encoding='16UC1',is_bigendian=False,
        step=200,data=np.full((100,100),1000,dtype='<u2').tobytes())
    info=NS(header=header,width=100,height=100,distortion_model='plumb_bob',
        d=[0.]*5,k=[100.,0.,49.5,0.,100.,49.5,0.,0.,1.])
    return image,info,time.monotonic()-age


def observe(value,generation,stamp,raw=None,**overrides):
    now=time.monotonic();wall=time.time_ns()
    state=dict(raw=raw or target(elbow_flex=2993),monotonic=stamp,
        timestamp_utc=datetime.fromtimestamp((wall/1e9)+(stamp-now),timezone.utc).isoformat(),
        arm_id=value.arm['id'],calibration_sha256=value.arm['calibration']['sha256'],
        owner_source_sha256=value.expected_sources)
    state.update(overrides)
    value.observe_owner_sample(state,generation)


def paired(value,generation,stamp,changed=False):
    observe(value,generation,stamp-.020)
    observe(value,generation,stamp+.020,raw=target(elbow_flex=2992 if changed else 2993))


def wait_for(predicate,timeout=.4):
    until=time.monotonic()+timeout
    while not predicate() and time.monotonic()<until:time.sleep(.005)
    assert predicate()


def test_lightweight_loader_no_model_construction_and_disabled(tmp_path,monkeypatch):
    import workshop.station.recovery_collision_model as model
    config,_,_,_=fixture(tmp_path)
    monkeypatch.setattr(model,'URDFCollisionModel',lambda *a,**k:pytest.fail('model constructed'))
    trusted=load_recovery_runtime_config(config,tmp_path/'state')
    assert trusted['registration']['camera_serial']=='synthetic-qualified'
    assert load_recovery_runtime_config({'arm':{}},tmp_path/'state') is None


@pytest.mark.parametrize('encoding,bigendian', [('16UC1',False),('16UC1',True),('32FC1',False),('32FC1',True)])
def test_depth_units_stride_endian_and_unknown_values(encoding,bigendian):
    typ=('>' if bigendian else '<')+('u2' if encoding=='16UC1' else 'f4')
    source=np.array([[1000,0],[2000,3000]],dtype=typ) if encoding=='16UC1' else np.array([[1.,0.],[np.nan,2.]],dtype=typ)
    rows=[row.tobytes()+b'padding!' for row in source]
    image=NS(width=2,height=2,encoding=encoding,is_bigendian=bigendian,
        step=len(rows[0]),data=b''.join(rows))
    depth=decode_depth(image)
    assert depth.dtype==np.float32 and depth[0,0]==1 and depth[0,1]==0
    if encoding=='32FC1':assert np.isnan(depth[1,0])
    else:assert depth[1,0]==2


@pytest.mark.parametrize('fault',['encoding','stride','payload'])
def test_bad_depth_payload_rejected(fault):
    image=NS(width=2,height=2,encoding='16UC1',is_bigendian=False,step=4,data=b'\0'*8)
    if fault=='encoding':image.encoding='mono16'
    if fault=='stride':image.step=3
    if fault=='payload':image.data=b'\0'
    with pytest.raises(ValueError):decode_depth(image)


def test_worker_writes_exact_fixed_cohort_and_preserves_acquisition(tmp_path):
    value,_,generation=collector(tmp_path,start_worker=True)
    try:
        image,info,stamp=message(value)
        paired(value,generation,stamp)
        value.submit_depth(image,info)
        path=value.cache_root/'latest.json'
        wait_for(path.exists)
        record=json.loads(path.read_text())
        assert record['depth_stamp_ns']==image.header.stamp.sec*1_000_000_000+image.header.stamp.nanosec
        assert record['acquisition_arm_raw']==target(elbow_flex=2993)
        assert abs(record['acquisition_arm_stamp_monotonic']-stamp)==pytest.approx(.020)
        with np.load(value.cache_root/(record['generation']+'.npz'),allow_pickle=False) as payload:
            assert payload.files==['depth_m'] and np.all(payload['depth_m']==1.)
        assert value.snapshot()['state']=='published'
    finally:value.close()
    wait_for(lambda:not path.exists())


@pytest.mark.parametrize('fault',['moving','missing_bracket','source','identity','calibration','generation','old','future','frame','intrinsics','utc'])
def test_bad_brackets_or_images_cannot_publish(tmp_path,fault):
    value,_,generation=collector(tmp_path,start_worker=True)
    try:
        image,info,stamp=message(value,age=.3 if fault=='old' else -.01 if fault=='future' else .05)
        if fault=='moving':paired(value,generation,stamp,changed=True)
        elif fault=='missing_bracket':observe(value,generation,stamp-.02)
        else:paired(value,generation,stamp)
        if fault=='source':observe(value,generation,time.monotonic(),owner_source_sha256={})
        if fault=='identity':observe(value,generation,time.monotonic(),arm_id='wrong')
        if fault=='calibration':observe(value,generation,time.monotonic(),calibration_sha256='f'*64)
        if fault=='generation':value.invalidate_generation(('replacement',time.monotonic()))
        if fault=='frame':image.header.frame_id='other'
        if fault=='intrinsics':info.k[0]=101.
        if fault=='utc':
            from dataclasses import replace
            with value._lock:value._history=(replace(value._history[0],utc_ns=1),replace(value._history[1],utc_ns=2))
        value.submit_depth(image,info)
        time.sleep(.04)
        assert not (value.cache_root/'latest.json').exists()
        assert value.snapshot()['state']=='waiting'
    finally:value.close()


def test_generation_change_during_serialization_revokes_without_publication(tmp_path):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value);paired(value,generation,stamp)
    value._revoke=False
    record,depth=value._prepare(image,info,generation,value._epoch,tuple(value._history))
    epoch=value._epoch
    value.invalidate_generation(('replacement',time.monotonic()))
    with pytest.raises(ValueError,match='generation invalidated'):
        value._write(record,depth,generation,epoch)
    assert not (value.cache_root/'latest.json').exists()


def test_callbacks_nonblocking_no_conversion_hash_or_files(tmp_path,monkeypatch):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value)
    monkeypatch.setattr(np,'savez',lambda *a,**k:pytest.fail('callback serialization'))
    monkeypatch.setattr(Path,'write_bytes',lambda *a,**k:pytest.fail('callback file write'))
    with value._lock:
        started=time.monotonic()
        observe(value,generation,stamp)
        value.submit_depth(image,info)
        value._on_depth(image,camera_generation='a'*64);value._on_info(info,camera_generation='a'*64)
        assert time.monotonic()-started<.050
    assert value._pending is None
    for index in range(20):observe(value,generation,stamp-.01+index*.0001)
    assert len(value._history)==8


def test_producer_record_to_real_runtime_oracle_preserves_owner_provenance(tmp_path):
    from workshop.station.recovery_runtime import create_recovery_runtime
    config,_,_,_=fixture(tmp_path)
    runtime=create_recovery_runtime(config,tmp_path/'state');runtime.close()
    value,_,generation=collector(tmp_path)
    value.cache_generation=runtime.cache_generation
    value.cache_root=runtime.cache_root
    value.runtime_started_monotonic=runtime.runtime_started_monotonic
    time.sleep(.035)
    image,info,stamp=message(value,age=.010)
    observe(value,generation,stamp-.005)
    observe(value,generation,stamp+.005)
    record,depth=value._prepare(image,info,generation,value._epoch,tuple(value._history))
    scene=runtime.prepare(record,depth)
    bound=runtime(dict(scene.arm_raw),time.monotonic())
    assert bound.minimum_clearance_m(dict(scene.arm_raw))==.010
    provenance=scene.acquisition_clock_provenance
    assert provenance['acquisition_owner_source_sha256']==value.expected_sources
    assert provenance['acquisition_owner_started_monotonic']==generation[1]
    assert provenance['runtime_started_monotonic']==runtime.runtime_started_monotonic
    assert generation[1]<runtime.runtime_started_monotonic<scene.arm_stamp_monotonic


def test_duplicate_callback_keeps_cohort_but_invalid_new_frame_revokes(tmp_path):
    value,_,generation=collector(tmp_path,start_worker=True)
    try:
        image,info,stamp=message(value);paired(value,generation,stamp)
        value.submit_depth(image,info)
        path=value.cache_root/'latest.json';wait_for(path.exists)
        original=path.read_bytes()
        value.submit_depth(image,info);time.sleep(.025)
        assert path.read_bytes()==original
        bad,badinfo,newstamp=message(value,age=.01);bad.header.frame_id='wrong'
        value.submit_depth(bad,badinfo)
        wait_for(lambda:not path.exists())
    finally:value.close()


def test_invalid_owner_even_during_callback_lock_contention_revokes_epoch(tmp_path):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value);paired(value,generation,stamp)
    value._revoke=False
    epoch=value._epoch
    record,depth=value._prepare(image,info,generation,epoch,tuple(value._history))
    with value._lock:observe(value,generation,time.monotonic(),owner_source_sha256={})
    assert value._epoch!=epoch
    with pytest.raises(ValueError,match='generation invalidated'):
        value._write(record,depth,generation,epoch)
    assert not (value.cache_root/'latest.json').exists()


def test_repeated_frames_bound_disk_retention_and_latest_readable(tmp_path):
    value,_,generation=collector(tmp_path,start_worker=True)
    try:
        value.cache_root.mkdir(parents=True,exist_ok=True)
        unrelated=value.cache_root/'keep.txt';unrelated.write_text('preserve')
        # Leftover validated old process cohorts are bounded on first publish.
        for index in range(5):(value.cache_root/(f'{index:032x}'+'.npz')).write_bytes(b'old')
        for index in range(8):
            image,info,stamp=message(value)
            paired(value,generation,stamp)
            value.submit_depth(image,info)
            wait_for(lambda:value.snapshot()['published']>=index+1)
            record=json.loads((value.cache_root/'latest.json').read_text())
            with np.load(value.cache_root/(record['generation']+'.npz')) as payload:
                assert np.all(payload['depth_m']==1.)
            assert len(list(value.cache_root.glob('*.npz')))<=3
            assert value._pending is None and len(value._history)<=8
            time.sleep(.025)
        assert unrelated.read_text()=='preserve'
    finally:value.close()


def test_blocked_old_writer_close_is_nonblocking_and_cannot_touch_new_nonce(tmp_path,monkeypatch):
    import threading
    old,_,oldgen=collector(tmp_path,start_worker=True)
    entered=threading.Event();release=threading.Event()
    original=Path.replace
    def blocked(path,destination):
        if path.parent==old.cache_root:
            entered.set();release.wait(2)
        return original(path,destination)
    monkeypatch.setattr(Path,'replace',blocked)
    new=None;third=None
    try:
        image,info,stamp=message(old);paired(old,oldgen,stamp);old.submit_depth(image,info)
        assert entered.wait(.5)
        with old._lock:
            started=time.monotonic();old.invalidate_generation(None);old.close()
            assert time.monotonic()-started<.020
        new,_,newgen=collector(tmp_path,start_worker=True,nonce='2'*32)
        image,info,stamp=message(new);paired(new,newgen,stamp);new.submit_depth(image,info)
        newpointer=new.cache_root/'latest.json';wait_for(newpointer.exists)
        record=newpointer.read_bytes()
        third,_,_=collector(tmp_path,start_worker=True,nonce='3'*32)
        assert third._worker is None and third.snapshot()['state']=='unavailable'
        release.set();old._worker.join(.5)
        assert not old._worker.is_alive()
        assert newpointer.read_bytes()==record
        data=json.loads(record)
        with np.load(new.cache_root/(data['generation']+'.npz')) as payload:assert np.all(payload['depth_m']==1.)
    finally:
        release.set();old.close()
        if new:new.close()
        if third:third.close()
        for value in (old,new):
            if value and value._worker:value._worker.join(.5)


def test_failed_pointer_serialization_does_not_accumulate_orphan_cohorts(tmp_path,monkeypatch):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value);paired(value,generation,stamp)
    value._revoke=False
    record,depth=value._prepare(image,info,generation,value._epoch,tuple(value._history))
    original=Path.write_text
    def fail(path,*args,**kwargs):
        if path.suffix=='.tmp':raise OSError('injected metadata write failure')
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'write_text',fail)
    for _ in range(5):
        with pytest.raises(OSError,match='metadata'):
            value._write(record,depth,generation,value._epoch)
        assert not list(value.cache_root.glob('*.npz'))
        assert not list(value.cache_root.glob('*.tmp'))
    assert not (value.cache_root/'latest.json').exists()


def test_closed_callbacks_are_inert_and_release_retained_messages(tmp_path):
    value,_,generation=collector(tmp_path)
    image,info,_=message(value)
    value._on_depth(image,camera_generation='a'*64);value._on_info(info,camera_generation='a'*64)
    assert value._image is image and value._info is info
    value.close()
    value._on_depth(image,camera_generation='a'*64);value._on_info(info,camera_generation='a'*64);value.submit_depth(image,info)
    assert value._image is None and value._info is None and value._pending is None


@pytest.mark.parametrize('fault',['missing','actual_serial','parameter_serial','stale','future','nan','generation'])
def test_actual_camera_identity_cannot_be_asserted_or_refreshed_by_receipt(tmp_path,fault):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value);paired(value,generation,stamp)
    identity=dict(value._camera_identity)
    if fault=='missing':identity={}
    if fault=='actual_serial':identity['serial_number']='other'
    if fault=='parameter_serial':identity['serial_no_parameter']='other'
    if fault=='stale':identity['request_started_monotonic']=time.monotonic()-1.01
    if fault=='future':identity['confirmed_monotonic']=time.monotonic()+1
    if fault=='nan':identity['request_started_monotonic']=float('nan')
    if fault=='generation':identity['generation']='bad'
    value.observe_camera_identity(identity)
    assert value._camera_identity is None and not value._history
    with pytest.raises(ValueError,match='actual camera identity'):
        value._prepare(image,info,generation,value._epoch,())


@pytest.mark.parametrize('fault',['gid_change','rollback','stream_gap'])
def test_stream_generation_and_liveness_revoke_identity(tmp_path,fault):
    value,_,_=collector(tmp_path)
    image,info,_=message(value)
    value._on_depth(image,camera_generation='a'*64)
    if fault=='gid_change':value._on_info(info,camera_generation='b'*64)
    if fault=='rollback':
        older,_,_=message(value,age=.08)
        value._on_depth(older,camera_generation='a'*64)
    if fault=='stream_gap':
        value._stream_state['depth']=(0,time.monotonic()-.251)
        value._on_depth(image,camera_generation='a'*64)
    assert value._camera_identity is None and value._pending is None


def test_identity_confirmation_requires_new_exposure_without_relabeling_old_frame(tmp_path):
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value);paired(value,generation,stamp)
    old=dict(value._camera_identity)
    current=dict(old,request_started_monotonic=time.monotonic()-.002,confirmed_monotonic=time.monotonic()-.001)
    value.observe_camera_identity(current)
    with pytest.raises(ValueError,match='post-confirmation'):
        value._prepare(image,info,generation,value._epoch,tuple(value._history))
    assert old['confirmed_monotonic']!=value._camera_identity['confirmed_monotonic']


def test_actual_pose_bracket_covers_whole_clock_sampling_interval(tmp_path,monkeypatch):
    import so101_station_control.recovery_depth_cache as module
    value,_,generation=collector(tmp_path)
    image,info,stamp=message(value)
    observe(value,generation,stamp-.020)
    observe(value,generation,stamp+.002)
    monkeypatch.setattr(module,'exposure_monotonic',lambda header:(stamp,stamp+.004))
    with pytest.raises(ValueError,match='bracketing'):
        value._prepare(image,info,generation,value._epoch,value._history)
    observe(value,generation,stamp+.005)
    record,_=value._prepare(image,info,generation,value._epoch,value._history)
    assert record['depth_stamp_monotonic']==stamp
    assert record['depth_clock_uncertainty_s']==pytest.approx(.004)


def test_pending_future_bracket_preserves_immutable_prior_and_then_replaces(tmp_path):
    value,_,generation=collector(tmp_path,start_worker=True)
    try:
        image,info,stamp=message(value,age=.080);paired(value,generation,stamp)
        value.submit_depth(image,info);path=value.cache_root/'latest.json'
        wait_for(path.exists);original=path.read_bytes()
        image2,info2,stamp2=message(value,age=.015)
        observe(value,generation,stamp2-.005)
        value.submit_depth(image2,info2)
        wait_for(lambda:'future bracketing' in (value.snapshot().get('reason') or ''))
        assert path.read_bytes()==original
        observe(value,generation,stamp2+.005)
        wait_for(lambda:path.exists() and path.read_bytes()!=original)
        replacement=json.loads(path.read_text())
        assert replacement['depth_stamp_ns']>json.loads(original)['depth_stamp_ns']
        assert replacement['acquisition_arm_raw']==json.loads(original)['acquisition_arm_raw']
    finally:value.close()


@pytest.mark.parametrize('fault',['epoch','old'])
def test_invalid_prior_sample_is_not_pending_future_bracket(tmp_path,fault):
    from dataclasses import replace
    from so101_station_control.recovery_depth_cache import PendingFutureBracket
    value,_,generation=collector(tmp_path)
    try:
        image,info,stamp=message(value,age=.015)
        observe(value,generation,stamp-.005)
        sample=value._history[-1]
        sample=replace(sample,epoch=sample.epoch+1) if fault=='epoch' else replace(sample,stamp=stamp-.110)
        with pytest.raises(ValueError) as caught:
            value._prepare(image,info,generation,value._epoch,(sample,))
        assert not isinstance(caught.value,PendingFutureBracket)
    finally:value.close()


def test_health_snapshots_stream_map_before_iterating_callback_mutation(tmp_path):
    value,_,_=collector(tmp_path)
    class ConcurrentInsertion(dict):
        def items(self):raise RuntimeError('dict changed size during iteration')
    now=time.monotonic();header=time.time_ns()
    value._stream_state=ConcurrentInsertion(depth=(header,now),info=(header,now))
    value._write_health()
    saved=json.loads((value.cache_root/'health.json').read_text())
    assert set(saved['streams'])=={'depth','info'}
    value.close()


@pytest.mark.parametrize('fault',[None,'initial','expired_prior','gid','serial','frame','intrinsics','encoding','old_header','future_header'])
def test_same_context_refresh_transition_is_wait_only_and_narrow(tmp_path,fault):
    from types import MappingProxyType
    from so101_station_control.recovery_depth_cache import PendingPostConfirmationExposure
    value,_,generation=collector(tmp_path)
    try:
        now=time.monotonic()
        if fault=='initial':value.invalidate_camera_identity()
        if fault=='expired_prior':
            value._camera_identity=MappingProxyType(dict(value._camera_identity,
                request_started_monotonic=now-1.001,confirmed_monotonic=now-1.))
        identity=dict(schema='realsense-camera-identity-v1',serial_number='synthetic-qualified',
            serial_no_parameter='synthetic-qualified',generation='b'*64 if fault=='gid' else 'a'*64,
            request_started_monotonic=now-.060,confirmed_monotonic=now-.050)
        if fault=='serial':identity['serial_no_parameter']='wrong'
        value.observe_camera_identity(identity)
        image,info,stamp=message(value,age=.080)
        paired(value,generation,stamp)
        if fault=='frame':image.header.frame_id='wrong'
        if fault=='intrinsics':info.k[0]+=1
        if fault=='encoding':image.encoding='unsupported'
        if fault in {'old_header','future_header'}:
            header=time.time_ns()+(-300_000_000 if fault=='old_header' else 10_000_000)
            image.header.stamp=NS(sec=header//1_000_000_000,nanosec=header%1_000_000_000)
        with pytest.raises(ValueError) as caught:
            value._prepare(image,info,generation,value._epoch,tuple(value._history))
        assert isinstance(caught.value,PendingPostConfirmationExposure)==(fault is None)
        assert not (value.cache_root/'latest.json').exists()  # never scene authority
    finally:value.close()


def test_refresh_transition_marker_clears_on_all_context_invalidation(tmp_path):
    value,_,generation=collector(tmp_path)
    now=time.monotonic()
    identity=dict(value._camera_identity,request_started_monotonic=now-.060,confirmed_monotonic=now-.050)
    value.observe_camera_identity(identity)
    assert value._refresh_request_started==identity['request_started_monotonic']
    value.invalidate_generation(None);assert value._refresh_request_started is None
    value.observe_camera_identity(dict(identity,request_started_monotonic=now-.040,confirmed_monotonic=now-.030))
    value.invalidate_camera_identity();assert value._refresh_request_started is None
    value.close()


@pytest.mark.parametrize('transition',['nonstationary','refresh'])
@pytest.mark.parametrize('health_failure',[False,True])
def test_transition_health_commits_before_pointer_revoke_even_if_health_write_fails(tmp_path,monkeypatch,transition,health_failure):
    from so101_station_control.recovery_depth_cache import NonstationaryBracket,PendingPostConfirmationExposure
    value,_,_=collector(tmp_path)
    pointer=value.cache_root/'latest.json';pointer.parent.mkdir(parents=True,exist_ok=True);pointer.write_text('{}')
    value._revoke=False;value._pending=(None,None,None,None,None)
    error=NonstationaryBracket if transition=='nonstationary' else PendingPostConfirmationExposure
    def prepare(*_):raise error('controlled transition')
    monkeypatch.setattr(value,'_prepare',prepare)
    calls=[]
    def health():
        calls.append(pointer.exists())
        value._closed=True
        if health_failure:raise OSError('controlled health serialization failure')
    monkeypatch.setattr(value,'_write_health',health)
    value._event.set();value._run()
    assert calls[0] is True and not pointer.exists()
    assert value._health_code==('nonstationary_bracket' if transition=='nonstationary' else 'awaiting_post_confirmation_exposure')
