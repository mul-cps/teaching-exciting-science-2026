"""Trusted ROS depth collector; serialization/geometry never run in callbacks.

Only identical measured encoder samples bracketing the exposure can pair a
frame. This deliberately cannot certify moving acquisition geometry.
"""
from collections import deque
from dataclasses import dataclass
from datetime import datetime
import hashlib
import io
import json
import math
import re
from pathlib import Path
import threading
import time
from types import MappingProxyType
import uuid

import numpy as np

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
MAX_AGE_S=.250
MAX_PAIR_S=.100
_WORKER_SLOTS=threading.BoundedSemaphore(2)


class PendingPostConfirmationExposure(ValueError):
    """Same-context identity refresh; old exposure never gains authority."""


class NonstationaryBracket(ValueError):
    """Identity-valid acquisition brackets differ; no scene authority."""


class PendingFutureBracket(ValueError):
    """Valid new frame awaits a later real owner sample; no contradiction."""


def stamp_ns(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


def exposure_monotonic(header_ns):
    before=time.monotonic();wall=time.time_ns();after=time.monotonic()
    age=(wall-header_ns)/1e9
    if after-before>.005 or not 0<=age<=MAX_AGE_S:
        raise ValueError('depth SYSTEM header is future/stale or clock bracket too slow')
    return before-age,after-age


def decode_depth(image):
    """RealSense ROS4.58.4 normalizes16UC1 to millimeters before publication."""
    if image.encoding not in ('16UC1','32FC1'):
        raise ValueError('unsupported depth encoding')
    size=2 if image.encoding=='16UC1' else 4
    if (type(image.width) is not int or type(image.height) is not int or
            not 0<image.width<=4096 or not 0<image.height<=4096 or
            image.width*image.height>4_194_304 or image.step<image.width*size or
            len(image.data)!=image.step*image.height or len(image.data)>32*1024*1024):
        raise ValueError('invalid bounded depth dimensions/stride/payload')
    dtype=np.dtype(('>' if image.is_bigendian else '<')+('u2' if size==2 else 'f4'))
    data=np.ndarray((image.height,image.width),dtype=dtype,buffer=image.data,
                    strides=(image.step,size)).astype(np.float32)
    if size==2:data*=.001
    # Zero/NaN/inf/negative values remain unknown in the existing depth oracle.
    return data


@dataclass(frozen=True)
class OwnerSample:
    raw: object
    stamp: float
    utc_ns: int
    generation: object
    epoch: int


class RecoveryDepthCache:
    def __init__(self,node,*,trusted_config,expected_owner_sources,state_file,cache_generation,runtime_started_monotonic,
                 depth_topic='/camera/aligned_depth_to_color/image_raw',
                 camera_info_topic='/camera/color/camera_info',start_worker=True):
        self.registration=trusted_config['registration']
        self.arm=trusted_config['arm']
        self.expected_sources=dict(expected_owner_sources)
        if (not isinstance(cache_generation,str) or not re.fullmatch('[a-f0-9]{32}',cache_generation) or
                type(runtime_started_monotonic) not in (int,float) or
                not math.isfinite(runtime_started_monotonic) or
                not 0<=runtime_started_monotonic<=time.monotonic()):
            raise ValueError('trusted runtime cache generation/start required')
        self.cache_generation=cache_generation
        self.runtime_started_monotonic=runtime_started_monotonic
        self.cache_root=Path(state_file).resolve().parent/'camera-recovery'/cache_generation
        self._lock=threading.Lock();self._history=()
        self._generation=None;self._camera_identity=None;self._stream_state={};self._epoch=0;self._pending=None;self._image=None;self._info=None
        self._revoke=True;self._closed=False;self._event=threading.Event()
        self._worker=None;self._subscriptions=[];self._node=node
        self._diagnostics=dict(state='waiting',reason='no paired acquisition',published=0)
        self._health_code='unavailable'
        self._refresh_request_started=None
        self._published_files=deque();self._last_header=0
        if node is not None:
            raise ValueError('use the bridge-owned permanent subscription pair; node must be None')
        if start_worker and _WORKER_SLOTS.acquire(blocking=False):
            self._worker=threading.Thread(target=self._worker_main,name='recovery-depth-writer',daemon=True)
            self._worker.start()
        elif start_worker:
            self._diagnostics.update(state='unavailable',reason='bounded depth writer slots exhausted')

    def invalidate_generation(self,generation):
        # No I/O, lock wait or join on the stop/telemetry paths.
        self._health_code="invalid";self._refresh_request_started=None
        self._epoch+=1;self._generation=generation
        self._history=();self._pending=None;self._image=None;self._info=None
        self._revoke=True;self._event.set()

    def invalidate_camera_identity(self):
        self._health_code="invalid";self._refresh_request_started=None
        self._camera_identity=None;self._stream_state={}
        self._epoch+=1;self._history=();self._pending=None;self._image=None;self._info=None
        self._revoke=True;self._event.set()

    def observe_camera_identity(self,identity):
        now=time.monotonic()
        try:
            value=dict(identity)
            if (value.get('schema')!='realsense-camera-identity-v1' or
                    value.get('serial_number')!=self.registration['camera_serial'] or
                    value.get('serial_no_parameter')!=self.registration['camera_serial'] or
                    not isinstance(value.get('generation'),str) or
                    not re.fullmatch('[a-f0-9]{64}',value['generation']) or
                    any(type(value.get(k)) not in (int,float) or not math.isfinite(value[k])
                        for k in ('request_started_monotonic','confirmed_monotonic')) or
                    not 0<=now-value['request_started_monotonic']<=1. or
                    not value['request_started_monotonic']<=value['confirmed_monotonic']<=now):
                raise ValueError('actual camera identity missing/stale/mismatched')
            previous=self._camera_identity
            if previous is None or previous['generation']!=value['generation']:
                self.invalidate_camera_identity()
            elif value['request_started_monotonic']<previous['request_started_monotonic']:
                raise ValueError('camera identity request clock rolled back')
            elif value['request_started_monotonic']>previous['request_started_monotonic']:
                self._refresh_request_started=(value['request_started_monotonic']
                    if 0<=now-previous['request_started_monotonic']<=1. else None)
            self._camera_identity=MappingProxyType(value)
        except (ValueError,TypeError,KeyError):
            self.invalidate_camera_identity()

    def _stream_valid(self,image,camera_generation,kind):
        identity=self._camera_identity;now=time.monotonic()
        if (identity is None or camera_generation!=identity['generation'] or
                not 0<=now-identity['request_started_monotonic']<=1.):
            self.invalidate_camera_identity();return False
        header=stamp_ns(image)
        last_header,last_received=self._stream_state.get(kind,(0,None))
        if (header<last_header or last_received is not None and now-last_received>MAX_AGE_S):
            self.invalidate_camera_identity();return False
        self._stream_state[kind]=(header,now)
        return True

    def observe_owner_sample(self,state,generation):
        """Small immutable cache copy; caller already admitted process generation."""
        if self._closed:return
        now=time.monotonic()
        try:
            raw=state['raw'];stamp=state['monotonic']
            utc_ns=int(datetime.fromisoformat(state['timestamp_utc']).timestamp()*1e9)
            if (state.get('arm_id')!=self.arm['id'] or
                    state.get('calibration_sha256')!=self.arm['calibration']['sha256'] or
                    state.get('owner_source_sha256')!=self.expected_sources or
                    set(raw)!=set(JOINTS) or any(type(v) is not int or not 0<=v<=4095 for v in raw.values()) or
                    type(stamp) not in (int,float) or not math.isfinite(stamp) or not 0<=now-stamp<=MAX_AGE_S or
                    not isinstance(generation,tuple) or len(generation)!=2 or
                    type(generation[1]) not in (int,float) or not math.isfinite(generation[1]) or
                    not 0<=generation[1]<=self.runtime_started_monotonic<=stamp):
                raise ValueError('owner acquisition identity/source/time mismatch')
            sample=OwnerSample(MappingProxyType(dict(raw)),stamp,utc_ns,generation,self._epoch)
            if not self._lock.acquire(blocking=False):return
            try:
                if self._closed:return
                if self._generation is None:self._generation=generation
                if generation!=self._generation:return
                history=self._history
                if history and stamp<=history[-1].stamp:return
                self._history=tuple(s for s in history if now-s.stamp<=.400)[-7:]+(sample,)
            finally:self._lock.release()
        except (ValueError,KeyError,TypeError,OverflowError):
            self._epoch+=1;self._revoke=True
            if self._lock.acquire(blocking=False):
                try:self._history=();self._pending=None
                finally:self._lock.release()
        self._event.set()

    def _on_depth(self,image,*,camera_generation=None):
        if self._closed or not self._stream_valid(image,camera_generation,'depth'):return
        if not self._lock.acquire(blocking=False):return
        try:
            if self._closed:return
            self._image=image;info=self._info
        finally:self._lock.release()
        if info is not None:self.submit_depth(image,info)

    def _on_info(self,info,*,camera_generation=None):
        if self._closed or not self._stream_valid(info,camera_generation,'info'):return
        if not self._lock.acquire(blocking=False):return
        try:
            if self._closed:return
            self._info=info;image=self._image
        finally:self._lock.release()
        if image is not None:self.submit_depth(image,info)

    def submit_depth(self,image,info):
        if not self._lock.acquire(blocking=False):return
        try:
            if not self._closed and stamp_ns(image)>self._last_header:
                self._pending=(image,info,self._generation,self._epoch,self._camera_identity)
        finally:self._lock.release()
        self._event.set()

    def snapshot(self):
        return dict(self._diagnostics)  # atomically replaced small immutable dict

    def _prepare(self,image,info,generation,epoch,history,camera_identity=None):
        header=stamp_ns(image);acquired,exposure_upper=exposure_monotonic(header);r=self.registration
        identity=camera_identity if camera_identity is not None else self._camera_identity
        if (identity is None or self._camera_identity is None or
                identity['generation']!=self._camera_identity['generation'] or
                not 0<=time.monotonic()-identity['request_started_monotonic']<=1.):
            raise ValueError('fresh actual camera identity and post-confirmation exposure required')
        if header<=self._last_header:raise ValueError('depth cohort is not strictly newer')
        if (image.header.frame_id!=r['camera_frame'] or info.header.frame_id!=r['camera_frame'] or
                abs(stamp_ns(info)-header)>40_000_000 or image.width!=r['image_width'] or
                image.height!=r['image_height'] or info.width!=image.width or info.height!=image.height or
                info.distortion_model!=r['distortion_model'] or list(info.d)!=r['distortion'] or
                dict(fx=info.k[0],fy=info.k[4],ppx=info.k[2],ppy=info.k[5])!=r['intrinsics']):
            raise ValueError('depth/CameraInfo do not match trusted registration')
        depth=decode_depth(image)
        if acquired<=identity['confirmed_monotonic']:
            if (identity['request_started_monotonic']==self._refresh_request_started and
                    identity['request_started_monotonic']==self._camera_identity['request_started_monotonic']):
                raise PendingPostConfirmationExposure('awaiting genuinely post-confirmation exposure after same-context refresh')
            raise ValueError('post-confirmation exposure required')
        before=[s for s in history if s.stamp<=acquired]
        after=[s for s in history if s.stamp>=exposure_upper]
        if not before:raise ValueError('missing prior bracketing owner acquisition')
        if (before[-1].generation!=generation or before[-1].epoch!=epoch or
                acquired-before[-1].stamp>MAX_PAIR_S):
            raise ValueError('valid prior bracketing owner acquisition required')
        if not after:raise PendingFutureBracket('waiting for future bracketing owner acquisition')
        low,high=before[-1],after[0]
        if (low.generation!=generation or high.generation!=generation or
                low.epoch!=epoch or high.epoch!=epoch or
                acquired-low.stamp>MAX_PAIR_S or high.stamp-acquired>MAX_PAIR_S):
            raise ValueError('identity-bound bracketing owner timing required')
        # Both original clock domains must corroborate the bracket. No receipt
        # timestamp or interpolation may replace an actual encoder acquisition.
        if not low.utc_ns<=header<=high.utc_ns:
            raise ValueError('owner UTC and monotonic exposure brackets disagree')
        if low.raw!=high.raw:raise NonstationaryBracket('identical identity-bound bracketing owner raw required')
        sample=min((low,high),key=lambda s:abs(s.stamp-acquired))
        if not 0<=time.monotonic()-sample.stamp<=MAX_AGE_S:raise ValueError('paired owner acquisition stale')
        record=dict(schema='recovery-depth-acquisition-v1',source_clock='system',depth_stamp_ns=header,
            camera_identity=dict(identity),
            acquisition_owner_source_sha256=self.expected_sources,
            acquisition_owner_started_monotonic=generation[1],
            depth_stamp_monotonic=acquired,depth_clock_uncertainty_s=exposure_upper-acquired,
            acquisition_arm_stamp_monotonic=sample.stamp,
            acquisition_arm_raw=dict(sample.raw),arm_id=self.arm['id'],
            arm_calibration_sha256=self.arm['calibration']['sha256'],camera_serial=r['camera_serial'],
            camera_frame=r['camera_frame'],intrinsics=r['intrinsics'],distortion=r['distortion'],
            distortion_model=r['distortion_model'])
        return record,depth

    def _publication_valid(self,record,generation,epoch):
        identity=self._camera_identity;captured=record.get('camera_identity');now=time.monotonic()
        return (not self._closed and generation==self._generation and epoch==self._epoch and
                not self._revoke and identity is not None and captured is not None and
                captured['generation']==identity['generation'] and
                0<=now-captured['request_started_monotonic']<=1. and
                0<=now-record['depth_stamp_monotonic']<=MAX_AGE_S)

    def _write(self,record,depth,generation,epoch):
        buffer=io.BytesIO();np.savez(buffer,depth_m=depth);payload=buffer.getvalue()
        record=dict(record,generation=uuid.uuid4().hex,depth_sha256=hashlib.sha256(payload).hexdigest())
        if not 0<=time.monotonic()-record['depth_stamp_monotonic']<=MAX_AGE_S:
            raise ValueError('exposure expired during serialization')
        if not self._publication_valid(record,generation,epoch):
            raise ValueError('owner generation invalidated during serialization')
        self.cache_root.mkdir(parents=True,exist_ok=True)
        frame=self.cache_root/(record['generation']+'.npz')
        temporary=self.cache_root/(uuid.uuid4().hex+'.tmp')
        published=False
        try:
            frame.write_bytes(payload)
            temporary.write_text(json.dumps(record,allow_nan=False))
            if not self._publication_valid(record,generation,epoch):
                raise ValueError('owner generation invalidated before publication')
            # This may block, but only this worker's immutable runtime directory
            # is affected. Stop and replacement runtimes never wait for it.
            temporary.replace(self.cache_root/'latest.json')
            published=True
            if not self._publication_valid(record,generation,epoch):
                (self.cache_root/'latest.json').unlink(missing_ok=True)
                raise ValueError('owner invalidated during pointer publication')
            self._last_header=record['depth_stamp_ns']
            self._published_files.append(frame)
            while len(self._published_files)>3:self._published_files.popleft()
            retained=set(self._published_files)
            for old in self.cache_root.iterdir():
                if re.fullmatch(r'[a-f0-9]{32}\.npz',old.name) and old not in retained:
                    old.unlink(missing_ok=True)
            self._diagnostics=dict(state='published',reason=None,published=self._diagnostics['published']+1)
        finally:
            temporary.unlink(missing_ok=True)
            if not published:frame.unlink(missing_ok=True)

    def _prune_old_runtime_directories(self):
        """Best effort, worker only; stale late recreation is non-authoritative."""
        parent=self.cache_root.parent
        if not parent.exists():return
        candidates=[]
        for path in parent.iterdir():
            if len(candidates)>=64:break
            if (path!=self.cache_root and re.fullmatch('[a-f0-9]{32}',path.name)
                    and not path.is_symlink() and path.is_dir()):candidates.append(path)
        for directory in candidates[:-2]:
            try:
                for child in directory.iterdir():
                    if (child.name in {'latest.json','health.json'} or
                            re.fullmatch(r'[a-f0-9]{32}\.(npz|tmp|health\.tmp)',child.name)):
                        child.unlink(missing_ok=True)
                directory.rmdir()
            except OSError:pass  # no authority; retry at a later runtime start

    def _worker_main(self):
        try:
            self._prune_old_runtime_directories()
            self._run()
        finally:
            try:(self.cache_root/'latest.json').unlink(missing_ok=True)
            finally:_WORKER_SLOTS.release()

    def _write_health(self):
        # Background only. Original stream/request clocks never become this
        # serialization clock; nonce confines all late writes to this runtime.
        identity=self._camera_identity
        generation=self._generation
        record=dict(schema='recovery-depth-health-v1',cache_generation=self.cache_generation,
            runtime_started_monotonic=self.runtime_started_monotonic,
            owner_started_monotonic=generation[1] if generation is not None else None,
            owner_source_sha256=self.expected_sources,arm_id=self.arm['id'],
            arm_calibration_sha256=self.arm['calibration']['sha256'],
            camera_identity=dict(identity) if identity is not None else None,
            streams={k:dict(header_ns=v[0],received_monotonic=v[1])
                     for k,v in self._stream_state.copy().items()},
            observed_monotonic=time.monotonic(),state=self._health_code,
            same_generation_refresh_request_started_monotonic=self._refresh_request_started)
        self.cache_root.mkdir(parents=True,exist_ok=True)
        temporary=self.cache_root/(uuid.uuid4().hex+'.health.tmp')
        try:
            temporary.write_text(json.dumps(record));temporary.replace(self.cache_root/'health.json')
        finally:temporary.unlink(missing_ok=True)

    def _run(self):
        while not self._closed:
            self._event.wait(.010);self._event.clear()
            with self._lock:
                pending=self._pending;history=tuple(self._history);revoke=self._revoke
                self._revoke=False
            try:
                if revoke:(self.cache_root/'latest.json').unlink(missing_ok=True)
                if pending is not None:
                    record,depth=self._prepare(*pending[:4],history,pending[4])
                    self._write(record,depth,pending[2],pending[3])
                    self._health_code='ready'
                    with self._lock:
                        if self._pending is pending:self._pending=None
            except PendingFutureBracket as error:
                self._health_code='pending_future_bracket'
                with self._lock:self._diagnostics=dict(self._diagnostics,state='waiting',reason=str(error))
            except PendingPostConfirmationExposure as error:
                self._health_code='awaiting_post_confirmation_exposure'
                # Publish the typed hold-only reason before revoking scene
                # authority. A reader with an old health snapshot rereads it
                # after observing the missing pointer.
                try:self._write_health()
                except (OSError,ValueError,TypeError):pass
                finally:(self.cache_root/'latest.json').unlink(missing_ok=True)
                with self._lock:self._diagnostics=dict(self._diagnostics,state='waiting',reason=str(error))
            except NonstationaryBracket as error:
                self._health_code='nonstationary_bracket'
                # Publish the typed hold-only reason before revoking scene
                # authority. A reader with an old health snapshot rereads it
                # after observing the missing pointer.
                try:self._write_health()
                except (OSError,ValueError,TypeError):pass
                finally:(self.cache_root/'latest.json').unlink(missing_ok=True)
                with self._lock:self._diagnostics=dict(self._diagnostics,state='waiting',reason=str(error))
            except (ValueError,KeyError,TypeError,OSError,OverflowError) as error:
                self._health_code='invalid'
                (self.cache_root/'latest.json').unlink(missing_ok=True)
                with self._lock:self._diagnostics=dict(self._diagnostics,state='waiting',reason=str(error))
            try:self._write_health()
            except (OSError,ValueError,TypeError):pass  # stale/missing health fails closed

    def close(self):
        # Deliberately zero filesystem I/O, lock acquisition and joins. A stuck
        # worker remains isolated in its old runtime directory and consumes one
        # of the two process-wide slots; further acquisition then fails closed.
        self._closed=True;self._refresh_request_started=None;self._epoch+=1;self._pending=None;self._history=()
        self._image=None;self._info=None;self._camera_identity=None
        self._event.set()
