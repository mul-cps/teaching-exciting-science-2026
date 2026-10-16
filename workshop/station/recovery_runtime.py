"""Trusted recovery runtime: prepared acquisition scenes, fresh owner bindings.

Camera acquisition time/pose never change. A binding adds a CURRENT encoder
observation, then evaluates candidate/swept volumes against the original scene:
only material physically certified at acquisition is excluded from camera tests.
No candidate-pose geometry is used to carve the acquired image.
"""
from dataclasses import dataclass
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import threading
import zipfile
import time
from types import MappingProxyType

import numpy as np

from .config import JOINTS
from .recovery_collision_model import URDFCollisionModel, validate_transform
from .recovery_depth_scene import DepthRecoveryScene

PROVIDER_BUDGET_S = .050
PROOF_BUDGET_S = .250
ACQUISITION_MAX_AGE_S = .250


def _raw(value):
    if (not isinstance(value, dict) or set(value) != set(JOINTS) or
            any(type(v) is not int or not 0 <= v <= 4095 for v in value.values())):
        raise ValueError('complete hardware-range integer encoder observation required')
    return dict(value)


def _fresh(stamp, now, maximum):
    return type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp <= maximum


def _pinned(entry, directory, maximum=16*1024*1024):
    if not isinstance(entry, dict) or set(entry) != {'path', 'sha256'}:
        raise ValueError('server artifact requires exact path/SHA256 binding')
    path = Path(entry['path'])
    if not path.is_absolute():
        path = directory/path
    if path.stat().st_size > maximum:
        raise ValueError("recovery artifact exceeds bounded size")
    with path.open("rb") as stream:
        data = stream.read(maximum+1)
    if len(data) > maximum or hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise ValueError('recovery artifact size/hash mismatch')
    return data, path.resolve()


@dataclass(frozen=True)
class BoundRecoveryScene:
    """Fresh current read paired to an UNMODIFIED acquired collision oracle.

    ``stamp_monotonic`` is the original depth exposure. ``arm_stamp_monotonic``
    is the actual CURRENT owner read. ``acquisition_arm_*`` retain the original
    historical camera-pose observation. New bindings do not refresh either
    acquisition timestamp or rebuild the self-interior at the current pose.
    """
    acquisition: DepthRecoveryScene
    arm_raw: object
    arm_stamp_monotonic: float
    proof_started_monotonic: float

    @property
    def acquisition_arm_raw(self): return self.acquisition.arm_raw
    @property
    def acquisition_arm_stamp_monotonic(self): return self.acquisition.arm_stamp_monotonic
    @property
    def stamp_monotonic(self): return self.acquisition.stamp_monotonic
    @property
    def model_sha256(self): return self.acquisition.model_sha256
    @property
    def calibration_sha256(self): return self.acquisition.calibration_sha256
    @property
    def registration_qualified(self): return self.acquisition.registration_qualified
    @property
    def camera_serial(self): return self.acquisition.camera_serial

    def _check(self, method, *args):
        # Original acquisition expiry is also independently checked by the
        # delegated oracle, including cached successful proofs.
        if time.monotonic()-self.proof_started_monotonic > PROOF_BUDGET_S:
            return None
        result = method(*args)
        return result if time.monotonic()-self.proof_started_monotonic <= PROOF_BUDGET_S else None

    def minimum_clearance_m(self, raw):
        return self._check(self.acquisition.minimum_clearance_m, raw)

    def minimum_segment_clearance_m(self, start, end):
        return self._check(self.acquisition.minimum_segment_clearance_m, start, end)


class RecoveryRuntime:
    """One prepared-model owner and background acquisition preparer.

    Camera transport is a fixed server-owned RAM cache, not a command argument.
    The future ROS collector writes immutable <generation>.npz then atomically
    publishes latest.json. No camera SDK is opened here. Missing cache is a gate.
    """
    def __init__(self, *, model, registration, policy, settings, cache_root, start_worker=True):
        self.model, self.registration, self.policy = model, registration, policy
        self.settings, self.cache_root = settings, Path(cache_root)
        self._lock = threading.Lock()
        self._prepare_lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._worker = None
        self.last_error = 'no prepared acquisition'
        self.prepare_duration_s = None
        self.prewarm_duration_s = None
        self._prewarm_attempted = False
        self._prewarm_finished_header_ns = None
        if start_worker:
            self._worker = threading.Thread(target=self._collect, name='recovery-scene-cache', daemon=True)
            self._worker.start()

    def prepare(self, record, depth_m):
        """Background preparation. A known bad new acquisition revokes cache."""
        if not self._prepare_lock.acquire(blocking=False):
            raise ValueError("acquisition preparation already active")
        try:
            return self._prepare(record, depth_m)
        except (ValueError, KeyError, TypeError, ArithmeticError):
            with self._lock:
                self._latest = None
            raise
        finally:
            self._prepare_lock.release()

    def _prepare(self, record, depth_m):
        """Expensive work: background only, never invoked by __call__/owner tick."""
        started = time.monotonic()
        now = started
        # This producer contract accepts only epoch SYSTEM-clock headers. Pair
        # the independent host wall clock with monotonic now; receipt time alone
        # cannot turn an old exposure into a fresh acquisition.
        clock_before = time.monotonic()
        wall_ns = time.time_ns()
        clock_after = time.monotonic()
        clock_uncertainty = clock_after-clock_before
        header_ns = record.get('depth_stamp_ns')
        header_age = ((wall_ns-header_ns)/1e9
                      if type(header_ns) is int else float('inf'))
        claimed_stamp = record.get('depth_stamp_monotonic')
        derived_stamp = clock_before-header_age
        if (record.get('source_clock') != 'system' or clock_uncertainty > .005 or
                not 0 <= header_age <= ACQUISITION_MAX_AGE_S or
                not isinstance(claimed_stamp, (int, float)) or
                not math.isfinite(claimed_stamp) or
                abs(derived_stamp-claimed_stamp) > .020):
            raise ValueError('actual SYSTEM depth header age/monotonic mapping invalid')
        r = self.registration
        if (record.get('schema') != 'recovery-depth-acquisition-v1' or
                record.get('camera_serial') != self.policy.camera_serial or
                record.get('camera_frame') != r['camera_frame'] or
                record.get('arm_id') != self.settings['arm_id'] or
                record.get('arm_calibration_sha256') != self.policy.arm_calibration_sha256 or
                type(record.get('depth_stamp_ns')) is not int or record['depth_stamp_ns'] <= 0 or
                not _fresh(record.get('depth_stamp_monotonic'), now, ACQUISITION_MAX_AGE_S) or
                not _fresh(record.get('acquisition_arm_stamp_monotonic'), now, ACQUISITION_MAX_AGE_S)):
            raise ValueError('fresh identity-matched actual depth/arm acquisition required')
        if (record.get('intrinsics') != r['intrinsics'] or
                record.get('distortion') != r['distortion'] or
                record.get('distortion_model') != r['distortion_model'] or
                np.shape(depth_m) != (r['image_height'], r['image_width'])):
            raise ValueError('acquisition intrinsics/distortion/resolution do not match qualified registration')
        if (self._prewarm_finished_header_ns is not None and
                header_ns <= self._prewarm_finished_header_ns):
            raise ValueError('new post-prewarm depth acquisition required')
        raw = _raw(record.get('acquisition_arm_raw'))
        # The preparer exclusively warms its base-model LRU. Each acquisition
        # gets a private LRU sharing only read-only material/shape arrays, so
        # owner proofs cannot race background cache eviction.
        self.model.voxel_volume(raw, voxel_size_m=.005, padding_m=0.,
            interior_margin_m=self.settings['geometry_uncertainty_m'],
            deadline_monotonic=started+PROOF_BUDGET_S)
        if time.monotonic()-started > PROOF_BUDGET_S:
            raise ValueError('acquisition preparation exceeded 250 ms')
        scene_model = copy.copy(self.model)
        scene_model._volume_cache = self.model._volume_cache.copy()
        scene_model._volume_cache_cell_count = self.model._volume_cache_cell_count
        scene = DepthRecoveryScene(depth_m=depth_m, intrinsics=record['intrinsics'],
            distortion=record['distortion'], distortion_model=record['distortion_model'],
            base_from_camera=r['base_from_camera'], base_frame=r['base_frame'],
            camera_frame=r['camera_frame'], camera_serial=self.policy.camera_serial,
            model_sha256=self.model.model_sha256, calibration_sha256=self.model.calibration_sha256,
            registration_qualified=True, stamp_monotonic=derived_stamp,
            arm_raw=raw, arm_stamp_monotonic=record['acquisition_arm_stamp_monotonic'], model=scene_model,
            clearance_m=.010, voxel_size_m=.005,
            depth_uncertainty_m=self.settings['depth_uncertainty_m'],
            geometry_uncertainty_m=self.settings['geometry_uncertainty_m'],
            support_planes=self.settings['support_planes'], max_check_time_s=PROOF_BUDGET_S,
            acquisition_clock_provenance=dict(source_clock='system', depth_stamp_ns=header_ns,
                declared_depth_stamp_monotonic=claimed_stamp,
                source_clock_uncertainty_s=clock_uncertainty))
        duration = time.monotonic()-started
        if duration > PROOF_BUDGET_S:
            raise ValueError('acquisition preparation exceeded 250 ms')
        if not all(_fresh(stamp, time.monotonic(), ACQUISITION_MAX_AGE_S) for stamp in
                   (scene.stamp_monotonic, scene.arm_stamp_monotonic)):
            raise ValueError('acquisition expired during preparation')
        with self._lock:
            self._latest = scene
        self.prepare_duration_s, self.last_error = duration, None
        return scene

    def __call__(self, raw, stamp):
        started = time.monotonic()
        raw = _raw(raw)
        if not _fresh(stamp, started, PROVIDER_BUDGET_S) or not self._lock.acquire(blocking=False):
            return None
        try:
            scene = self._latest
        finally:
            self._lock.release()
        if (scene is None or not all(_fresh(s, time.monotonic(), ACQUISITION_MAX_AGE_S)
                for s in (scene.stamp_monotonic, scene.arm_stamp_monotonic))):
            return None
        binding = BoundRecoveryScene(scene, MappingProxyType(raw), stamp, started)
        return binding if time.monotonic()-started <= PROVIDER_BUDGET_S else None

    def prewarm_model(self, raw):
        """Once-only worker preparation; no scene/proof authority or depth reuse."""
        if self._prewarm_attempted:
            return
        self._prewarm_attempted = True
        started = time.monotonic()
        try:
            self.model.voxel_volume(_raw(raw), voxel_size_m=.005, padding_m=0.,
                interior_margin_m=self.settings['geometry_uncertainty_m'],
                deadline_monotonic=started+2.)
        finally:
            self.prewarm_duration_s = time.monotonic()-started
            self._prewarm_finished_header_ns = time.time_ns()
            with self._lock:
                self._latest = None

    def _collect(self):
        generation = None
        while not self._stop.is_set():
            try:
                path = self.cache_root/'latest.json'
                if path.stat().st_size > 65536:
                    raise ValueError('recovery cache manifest too large')
                record = json.loads(path.read_bytes())
                if not isinstance(record, dict):
                    raise ValueError('recovery cache manifest must be an object')
                current = record.get('generation')
                if not isinstance(current, str) or not re.fullmatch('[a-f0-9]{32}', current):
                    raise ValueError('invalid immutable recovery cache generation')
                if current != generation:
                    if not self._prewarm_attempted:
                        generation = current  # triggering cohort is permanently dropped
                        self.prewarm_model(record.get('acquisition_arm_raw'))
                        continue
                    if (type(record.get('depth_stamp_ns')) is not int or
                            record['depth_stamp_ns'] <= self._prewarm_finished_header_ns):
                        raise ValueError('new post-prewarm depth acquisition required')
                    frame = self.cache_root/(current+'.npz')
                    if frame.stat().st_size > 32*1024*1024:
                        raise ValueError('recovery depth cache payload too large')
                    payload = frame.read_bytes()
                    if hashlib.sha256(payload).hexdigest() != record['depth_sha256']:
                        raise ValueError('recovery depth cache hash mismatch')
                    import io
                    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
                        members = archive.infolist()
                        if (len(members) != 1 or members[0].filename != 'depth_m.npy' or
                                members[0].file_size > 32*1024*1024+4096):
                            raise ValueError('uncompressed recovery cache exceeds bounded depth payload')
                    with np.load(io.BytesIO(payload), allow_pickle=False) as arrays:
                        if set(arrays.files) != {'depth_m'}:
                            raise ValueError('recovery cache requires only metric depth')
                        self.prepare(record, arrays['depth_m'])
                    generation = current
            except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile, EOFError) as error:
                self.last_error = str(error)
                with self._lock:
                    self._latest = None
            self._stop.wait(.020)

    def close(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.1)


def create_recovery_runtime(config, state_file):
    """Server-configured authority only. Missing/disabled stays unqualified."""
    option = config.get('arm', {}).get('camera_recovery')
    if not option or option.get('enabled') is not True:
        return None
    manifest_bytes, manifest_path = _pinned(option['manifest'], Path(state_file).parent)
    manifest = json.loads(manifest_bytes)
    arm = config['arm']
    if (manifest.get('schema') != 'so101-camera-recovery-runtime-v1' or
            manifest.get('qualified') is not True or manifest.get('arm_id') != arm['id'] or
            manifest.get('arm_calibration_sha256') != arm['calibration']['sha256']):
        raise ValueError('qualified identity-matched recovery runtime manifest required')
    mapping_bytes, _ = _pinned(manifest['joint_mapping'], manifest_path.parent)
    mapping = json.loads(mapping_bytes)
    if (mapping.get('schema') != 'recovery-joint-mapping-v1' or
            mapping.get('qualified') is not True or mapping.get('arm_id') != arm['id'] or
            mapping.get('arm_calibration_sha256') != arm['calibration']['sha256']):
        raise ValueError('qualified physical-arm-matched joint mapping required')
    urdf, _ = _pinned(manifest['effective_urdf'], manifest_path.parent)
    model = URDFCollisionModel(urdf.decode(), joint_calibration=mapping['joints'],
        calibration_qualified=True, base_frame=manifest['base_frame'],
        package_paths=manifest.get('package_paths', {}),
        allowed_self_contacts=manifest['allowed_self_contacts'])
    if model.model_sha256 != manifest['model_sha256']:
        raise ValueError('effective collision model hash mismatch')
    registration_bytes, _ = _pinned(manifest['registration'], manifest_path.parent)
    registration = json.loads(registration_bytes)
    if (registration.get('schema') != 'camera-arm-calibration-v1' or
            registration.get('qualified') is not True or
            registration.get('arm_calibration_sha256') != arm['calibration']['sha256'] or
            registration.get('model_sha256') != model.model_sha256 or
            registration.get('base_frame') != model.base_frame or
            not isinstance(registration.get('camera_serial'), str) or not registration['camera_serial'] or
            not isinstance(registration.get('camera_frame'), str) or not registration['camera_frame']):
        raise ValueError('qualified model/arm/camera-matched base registration required')
    registration['base_from_camera'] = validate_transform(registration['base_from_camera'])
    for dimension in ('image_width', 'image_height'):
        if type(registration.get(dimension)) is not int or not 0 < registration[dimension] <= 4096:
            raise ValueError('qualified image dimensions required')
    if registration['image_width']*registration['image_height'] > 4_194_304:
        raise ValueError('qualified image exceeds bounded resolution')
    intrinsics = registration.get('intrinsics')
    if (not isinstance(intrinsics, dict) or set(intrinsics) != {'fx','fy','ppx','ppy'} or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in intrinsics.values()) or
            min(intrinsics['fx'],intrinsics['fy']) <= 0):
        raise ValueError('qualified finite pinhole intrinsics required')
    distortion = registration.get('distortion')
    if (registration.get('distortion_model') != 'plumb_bob' or
            not isinstance(distortion,list) or len(distortion) not in (4,5) or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in distortion)):
        raise ValueError('qualified Brown-Conrady distortion required')
    for name in ('depth_uncertainty_m', 'geometry_uncertainty_m'):
        value = manifest.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('positive commissioned uncertainty required: '+name)
    from .recovery_types import CameraRecoveryPolicy
    policy = CameraRecoveryPolicy(model.model_sha256, model.calibration_sha256,
        registration['camera_serial'], arm['calibration']['sha256'])
    settings = dict(arm_id=arm['id'], depth_uncertainty_m=manifest['depth_uncertainty_m'],
        geometry_uncertainty_m=manifest['geometry_uncertainty_m'], support_planes=manifest['support_planes'])
    return RecoveryRuntime(model=model, registration=registration, policy=policy,
        settings=settings, cache_root=Path(state_file).parent/'camera-recovery')
