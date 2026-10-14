"""Read-only bridge from saved calibration and phase evidence into the twin."""
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation


BOARD_FROM_WORLD = np.diag([1., -1., -1., 1.])
FRESHNESS_LIMIT_S = .5
POSE_PAIR_LIMIT_S = .16


def _rigid_transform(value, name):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'{name} must be a finite 4x4 transform')
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f'{name} must have a homogeneous final row')
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-2)
            or not np.isclose(np.linalg.det(rotation), 1., atol=2e-2)):
        raise ValueError(f'{name} must contain a proper rigid rotation')
    return transform


def compose_camera_arm_registration(camera_from_target, base_from_camera):
    """Compose the board-up world frame and provisional camera-to-arm evidence.

    The ChArUco target frame has image-down +Y and its +Z points away from the
    camera. BOARD_FROM_WORLD changes those axes so the digital-twin world has
    +Z up. The resulting camera/world and world/base transforms use full SE(3).
    """
    camera_from_target = _rigid_transform(camera_from_target, 'camera_from_target')
    base_from_camera = _rigid_transform(base_from_camera, 'base_from_camera')
    camera_from_world = camera_from_target @ BOARD_FROM_WORLD
    world_from_camera = np.linalg.inv(camera_from_world)
    world_from_base = world_from_camera @ np.linalg.inv(base_from_camera)
    return {
        'schema': 'provisional-camera-arm-registration-v1',
        'camera_from_world': camera_from_world.tolist(),
        'world_from_camera': world_from_camera.tolist(),
        'base_from_camera': base_from_camera.tolist(),
        'world_from_base': world_from_base.tolist(),
        'world_frame': 'ChArUco board origin; board +Z flipped to world +Z up',
        'composition': 'world_from_camera @ inverse(base_from_camera)',
        'qualified': False,
        'status': 'PROVISIONAL_UNQUALIFIED',
    }


def load_camera_arm_registration(camera_arm_path, camera_target_path):
    camera_arm_path, camera_target_path = Path(camera_arm_path), Path(camera_target_path)
    arm_report = json.loads(camera_arm_path.read_text())
    target_report = json.loads(camera_target_path.read_text())
    if 'base_from_camera' not in arm_report:
        raise ValueError('camera-arm evidence has no base_from_camera transform')
    if target_report.get('transform_direction') != 'camera_from_target':
        raise ValueError('ChArUco evidence must declare camera_from_target direction')
    if 'camera_from_target' not in target_report:
        raise ValueError('ChArUco evidence has no camera_from_target transform')
    result = compose_camera_arm_registration(target_report['camera_from_target'],
                                             arm_report['base_from_camera'])
    result['qualified'] = False
    result['status'] = 'PROVISIONAL_UNQUALIFIED'
    result['component_status'] = {
        'camera_arm_qualified': bool(arm_report.get('qualified', False)),
        'camera_target_qualified': bool(target_report.get('qualified', False)),
        'camera_arm_reason': arm_report.get('reason', 'source has no qualification reason'),
        'camera_arm_heldout': arm_report.get('heldout_candidate_closure', []),
        'camera_target_note': target_report.get('qualification_note', 'source has no qualification note'),
    }
    result['sources'] = {
        'camera_arm': str(camera_arm_path),
        'camera_arm_sha256': file_sha256(camera_arm_path),
        'camera_target': str(camera_target_path),
        'camera_target_sha256': file_sha256(camera_target_path),
    }
    return result


def scene_from_camera_world_delta(scene_world_from_camera, board_world_from_camera):
    """Transform old table-fit world coordinates into the board-up world frame."""
    scene_world_from_camera = _rigid_transform(scene_world_from_camera, 'scene_world_from_camera')
    board_world_from_camera = _rigid_transform(board_world_from_camera, 'board_world_from_camera')
    return board_world_from_camera @ np.linalg.inv(scene_world_from_camera)


def apply_world_registration(model, data, registration, scene_world_from_camera):
    """Re-express the rendered scene in board coordinates and place the arm by SE(3)."""
    import mujoco

    scene_world_from_camera = _rigid_transform(scene_world_from_camera, 'scene_world_from_camera')
    board_world_from_camera = _rigid_transform(registration['world_from_camera'], 'world_from_camera')
    world_from_scene = scene_from_camera_world_delta(scene_world_from_camera, board_world_from_camera)
    rotation = Rotation.from_matrix(world_from_scene[:3, :3])
    arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    for body in range(1, model.nbody):
        if model.body_parentid[body] != 0:
            continue
        if body == arm:
            world_from_base = _rigid_transform(registration['world_from_base'], 'world_from_base')
            model.body_pos[body] = world_from_base[:3, 3]
            model.body_quat[body] = Rotation.from_matrix(world_from_base[:3, :3]).as_quat()[[3, 0, 1, 2]]
            continue
        model.body_pos[body] = world_from_scene[:3, :3] @ model.body_pos[body] + world_from_scene[:3, 3]
        old_rotation = Rotation.from_quat(model.body_quat[body][[1, 2, 3, 0]])
        model.body_quat[body] = (rotation * old_rotation).as_quat()[[3, 0, 1, 2]]
    for geom in range(model.ngeom):
        if model.geom_bodyid[geom] != 0:
            continue
        model.geom_pos[geom] = world_from_scene[:3, :3] @ model.geom_pos[geom] + world_from_scene[:3, 3]
        old_rotation = Rotation.from_quat(model.geom_quat[geom][[1, 2, 3, 0]])
        model.geom_quat[geom] = (rotation * old_rotation).as_quat()[[3, 0, 1, 2]]
    for camera in range(model.ncam):
        if model.cam_bodyid[camera] != 0:
            continue
        model.cam_pos[camera] = world_from_scene[:3, :3] @ model.cam_pos[camera] + world_from_scene[:3, 3]
        old_rotation = Rotation.from_quat(model.cam_quat[camera][[1, 2, 3, 0]])
        model.cam_quat[camera] = (rotation * old_rotation).as_quat()[[3, 0, 1, 2]]
    mujoco.mj_forward(model, data)
    return world_from_scene


def transform_points(points, transform):
    points = np.asarray(points, dtype=float)
    transform = _rigid_transform(transform, 'world_from_scene')
    return points @ transform[:3, :3].T + transform[:3, 3]


def read_phase_evidence(path, now_monotonic=None, max_age_s=FRESHNESS_LIMIT_S):
    """Read phase or resident state evidence in the common raw-pose shape."""
    if now_monotonic is None:
        now_monotonic = time.monotonic()
    try:
        evidence = json.loads(Path(path).read_text())
        event_time = float(evidence['monotonic'])
        pose = evidence.get('raw', evidence.get('pose_raw'))
        if not isinstance(pose, dict) or not pose:
            raise ValueError('raw resident/phase pose is empty')
        pose = {str(name): int(value) for name, value in pose.items()}
        age = float(now_monotonic) - event_time
        if age < -.05:
            return {'freshness': 'invalid', 'reason': 'phase event timestamp is in the future',
                    'pose_raw': pose, 'phase': evidence.get('phase'), 'age_seconds': age}
        return {'freshness': 'fresh' if age <= max_age_s else 'stale',
                'reason': None if age <= max_age_s else 'last pose sample is older than freshness limit',
                'phase': str(evidence.get('phase') or evidence.get('label') or evidence.get('event') or 'resident sample'),
                'pose_raw': pose,
                'event_monotonic': event_time, 'event_id': evidence.get('event_id'),
                'timestamp_utc': evidence.get('timestamp_utc'),
                'schema': evidence.get('schema'),
                'age_seconds': max(age, 0.)}
    except FileNotFoundError:
        return {'freshness': 'missing', 'reason': 'phase file not found', 'pose_raw': None}
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        return {'freshness': 'invalid', 'reason': str(error), 'pose_raw': None}


def select_latest_rgbd_pair(snapshot, now_wall=None, now_monotonic=None):
    """Select the newest saved paired RGB/depth files by their file timestamps."""
    snapshot = Path(snapshot)
    live_pair = select_live_rgbd_pair(snapshot, now_wall=now_wall,
                                      now_monotonic=now_monotonic)
    if live_pair is not None:
        return live_pair
    # If this folder advertises a rolling latest pair, malformed, stale, or
    # unsynchronized metadata is a hard failure. Never silently substitute an
    # unrelated historical frame from that same live recorder folder.
    if (snapshot / 'latest.json').is_file() or (snapshot / 'timestamps.jsonl').is_file():
        return None
    candidates = [
        (snapshot / 'home-recheck.png', snapshot / 'home-recheck-depth.npy'),
        (snapshot / 'color.png', snapshot / 'aligned-depth.npy'),
    ]
    frames = snapshot / 'initial-align-retry' / 'frames'
    if frames.is_dir():
        candidates.extend((image, image.with_suffix('.npy')) for image in frames.glob('*.png'))
    valid = []
    for image, depth in candidates:
        if image.is_file() and depth.is_file():
            try:
                timestamp = min(image.stat().st_mtime, depth.stat().st_mtime)
            except OSError:
                continue
            valid.append((timestamp, image, depth))
    if not valid:
        return None
    timestamp, image, depth = max(valid, key=lambda value: value[0])
    return {'image': image, 'depth': depth, 'timestamp_wall': float(timestamp)}


def _last_jsonl_record(path):
    """Read only the final JSONL record, keeping the 5 Hz live path bounded."""
    with Path(path).open('rb') as stream:
        stream.seek(0, 2)
        end = stream.tell()
        stream.seek(max(0, end - 8192))
        lines = stream.read().decode('utf-8').splitlines()
    for line in reversed(lines):
        if line.strip():
            return json.loads(line)
    raise ValueError('camera timestamp log is empty')


def select_live_rgbd_pair(folder, now_wall=None, now_monotonic=None,
                          max_age_s=FRESHNESS_LIMIT_S):
    """Select saved latest RGB-D files only while their metadata and bytes are fresh."""
    folder = Path(folder)
    image, depth = folder / 'latest.png', folder / 'latest-depth.npy'
    latest_record = folder / 'latest.json'
    timestamps = folder / 'timestamps.jsonl'
    if not all(path.is_file() for path in (image, depth)) or not (latest_record.is_file() or timestamps.is_file()):
        return None
    if now_wall is None:
        now_wall = time.time()
    if now_monotonic is None:
        now_monotonic = time.monotonic()
    try:
        if latest_record.is_file():
            if latest_record.stat().st_size > 8192:
                return None
            record = json.loads(latest_record.read_bytes())
            if record.get('paths', {}).get('latest_color') != image.name or record.get('paths', {}).get('latest_depth') != depth.name:
                return None
            arm = record.get('arm') or {}
            if not arm.get('qualified_sync'):
                return None
            pose_raw = arm.get('raw')
            pose_monotonic = float(arm['monotonic'])
            pose_delta = float(record['monotonic']) - pose_monotonic
            declared_delta = float(arm['delta_s'])
            if (not isinstance(pose_raw, dict) or not pose_raw or abs(pose_delta) > POSE_PAIR_LIMIT_S
                    or abs(declared_delta - pose_delta) > .005):
                return None
            timestamp_record = record
        else:
            timestamp_record = _last_jsonl_record(timestamps)
            arm = {}
            pose_raw = timestamp_record.get('pose_raw', timestamp_record.get('raw'))
            pose_monotonic = timestamp_record.get('pose_monotonic')
            pose_delta = timestamp_record.get('pose_delta_s')
        timestamp_wall = datetime.fromisoformat(timestamp_record.get('utc', timestamp_record.get('timestamp_utc'))).timestamp()
        timestamp_monotonic = float(timestamp_record['monotonic'])
        image_mtime, depth_mtime = image.stat().st_mtime, depth.stat().st_mtime
        age = max(float(now_wall) - timestamp_wall,
                  float(now_monotonic) - timestamp_monotonic,
                  float(now_wall) - min(image_mtime, depth_mtime))
        if (not math.isfinite(age) or age < -.05 or age > max_age_s
                or abs(image_mtime - depth_mtime) > max_age_s
                or abs(min(image_mtime, depth_mtime) - timestamp_wall) > max_age_s):
            return None
        return {'image': image, 'depth': depth, 'timestamp_wall': timestamp_wall,
                'timestamp_monotonic': timestamp_monotonic,
                'timestamp_source': 'camera latest.json' if latest_record.is_file() else 'camera timestamps.jsonl',
                'camera_frame_index': timestamp_record.get('index'),
                'pose_raw': pose_raw,
                'pose_monotonic': pose_monotonic,
                'pose_sample_delta_seconds': pose_delta,
                'pose_qualified_sync': arm.get('qualified_sync'),
                'freshness': 'fresh'}
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def select_latest_keyframe_pair(keyframes, max_pose_delta_s=.16):
    """Select the newest manifest-paired keyframe with synchronized pose evidence."""
    keyframes = Path(keyframes)
    manifest = keyframes / 'manifest.jsonl'
    if not manifest.is_file():
        return None
    candidates = []
    for line in manifest.read_text().splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
            delta = float(record['phase_event_delta_s'])
            timestamp_mono = float(record['monotonic'])
            raw_pose = record.get('pose_raw')
            image = keyframes / record['color']
            depth = keyframes / record['depth']
            if (not record.get('pose_synchronized') or raw_pose is None
                    or abs(delta) > max_pose_delta_s or not image.is_file() or not depth.is_file()):
                continue
            timestamp_wall = datetime.fromisoformat(record['utc']).timestamp()
            candidates.append((timestamp_mono, {'image': image, 'depth': depth,
                'timestamp_wall': timestamp_wall, 'timestamp_monotonic': timestamp_mono,
                'phase_event_monotonic': float(record['phase_event_monotonic']),
                'phase_event_delta_s': delta, 'pose_raw': raw_pose,
                'pose_synchronized': True, 'phase': record.get('phase'),
                'keyframe_index': int(record['index']), 'keyframe_record': record,
                'manifest': manifest}))
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
            continue
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def _tail_jsonl_records(path, max_bytes=8192, max_records=128):
    """Read a fixed-size JSONL tail, dropping any row cut by the byte bound."""
    with Path(path).open('rb') as stream:
        stream.seek(0, 2)
        end = stream.tell()
        start = max(0, end - max_bytes)
        stream.seek(start)
        payload = stream.read(max_bytes)
    if start:
        first_newline = payload.find(b'\n')
        if first_newline < 0:
            return []
        payload = payload[first_newline + 1:]
    lines = payload.splitlines()
    if payload and not payload.endswith(b'\n'):
        lines = lines[:-1]
    records = []
    for line in lines[-max_records:]:
        if not line.strip():
            continue
        try:
            records.append(json.loads(line))
        except (ValueError, TypeError, json.JSONDecodeError):
            continue
    return records


def select_latest_keyframe_pair_tail(keyframes, max_pose_delta_s=.16,
                                     max_bytes=8192, max_records=128,
                                     max_age_s=FRESHNESS_LIMIT_S,
                                     now_wall=None, now_monotonic=None):
    """Select the newest synchronized keyframe from only the bounded manifest tail.

    This is for the viewer's repeated polling path. Startup/batch selection keeps
    using ``select_latest_keyframe_pair`` to search the complete manifest once.
    """
    keyframes = Path(keyframes)
    manifest = keyframes / 'manifest.jsonl'
    if not manifest.is_file():
        return None
    if now_wall is None:
        now_wall = time.time()
    if now_monotonic is None:
        now_monotonic = time.monotonic()
    candidates = []
    for record in _tail_jsonl_records(manifest, max_bytes=max_bytes,
                                      max_records=max_records):
        try:
            delta = float(record['phase_event_delta_s'])
            timestamp_mono = float(record['monotonic'])
            raw_pose = record.get('pose_raw')
            image = keyframes / record['color']
            depth = keyframes / record['depth']
            if (not record.get('pose_synchronized') or raw_pose is None
                    or abs(delta) > max_pose_delta_s or not image.is_file() or not depth.is_file()):
                continue
            timestamp_wall = datetime.fromisoformat(record['utc']).timestamp()
            age = max(float(now_wall) - timestamp_wall,
                      float(now_monotonic) - timestamp_mono)
            if not math.isfinite(age) or age < -.05 or age > max_age_s:
                continue
            candidates.append((timestamp_mono, {'image': image, 'depth': depth,
                'timestamp_wall': timestamp_wall, 'timestamp_monotonic': timestamp_mono,
                'phase_event_monotonic': float(record['phase_event_monotonic']),
                'phase_event_delta_s': delta, 'pose_raw': raw_pose,
                'pose_synchronized': True, 'phase': record.get('phase'),
                'keyframe_index': int(record['index']), 'keyframe_record': record,
                'manifest': manifest}))
        except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError):
            continue
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def raw_pose_to_joint_degrees(raw_pose, calibration):
    """Map saved encoder samples through the checked calibration used by prepare."""
    result = {}
    for joint, raw_value in raw_pose.items():
        bounds = calibration[joint]
        low, high = float(bounds['range_min']), float(bounds['range_max'])
        if not math.isfinite(low + high) or high <= low:
            raise ValueError(f'invalid calibration range for {joint}')
        raw_value = float(raw_value)
        result[joint] = ((raw_value - low) * 100. / (high - low) if joint == 'gripper'
                         else (raw_value - (low + high) / 2.) * 360. / 4095.)
    return result


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _json_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def write_pointcloud_provenance(report, output):
    """Bind reusable 3D caches to the RGB/depth bytes and capture timestamp."""
    output = Path(output)
    image_path, depth_path = Path(report['image']), Path(report['depth'])
    record = {
        'schema': 'digital-twin-pointcloud-provenance-v1',
        'image': str(image_path), 'image_sha256': file_sha256(image_path),
        'depth': str(depth_path), 'depth_sha256': file_sha256(depth_path),
        'capture_timestamp': float(report.get('camera_sample_timestamp_wall', report['camera']['timestamp'])),
        'scene_world_from_camera_sha256': _json_sha256(np.asarray(
            report.get('scene_world_from_camera', report.get('world_from_camera', np.eye(4))),
            dtype=float).tolist()),
        'camera_parameters_sha256': _json_sha256({'intrinsics': report['camera']['intrinsics'],
                                                  'depth_scale_m': report['camera']['depth_scale_m']}),
        'pointcloud_sha256': file_sha256(output / 'pointcloud.npz'),
        'measured_surface_sha256': file_sha256(output / 'measured-surface.npz'),
    }
    (output / 'pointcloud-provenance.json').write_text(json.dumps(record, indent=2) + '\n')
    return record


def pointcloud_cache_status(report, output):
    """Refuse cache reuse when source bytes, timestamp, or generated archives drift."""
    path = Path(output) / 'pointcloud-provenance.json'
    try:
        record = json.loads(path.read_text())
        valid = (record.get('schema') == 'digital-twin-pointcloud-provenance-v1'
                 and record['image'] == str(Path(report['image']))
                 and record['depth'] == str(Path(report['depth']))
                 and record['image_sha256'] == file_sha256(report['image'])
                 and record['depth_sha256'] == file_sha256(report['depth'])
                 and record['pointcloud_sha256'] == file_sha256(Path(output) / 'pointcloud.npz')
                 and record['measured_surface_sha256'] == file_sha256(Path(output) / 'measured-surface.npz')
                 and record['scene_world_from_camera_sha256'] == _json_sha256(np.asarray(
                     report.get('scene_world_from_camera', report.get('world_from_camera', np.eye(4))),
                     dtype=float).tolist())
                 and record['camera_parameters_sha256'] == _json_sha256({'intrinsics': report['camera']['intrinsics'],
                                                                          'depth_scale_m': report['camera']['depth_scale_m']})
                 and record['capture_timestamp'] == float(report.get('camera_sample_timestamp_wall', report['camera']['timestamp'])))
        return {'reusable': bool(valid), 'reason': None if valid else 'cache provenance no longer matches camera/depth inputs',
                'capture_timestamp': record.get('capture_timestamp')}
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return {'reusable': False, 'reason': 'pointcloud cache provenance missing or invalid',
                'capture_timestamp': None}


def camera_sample_status(report, phase, now_wall=None, now_monotonic=None, cache_reusable=False):
    if now_wall is None:
        now_wall = time.time()
    if now_monotonic is None:
        now_monotonic = time.monotonic()
    timestamp = report.get('camera_sample_timestamp_wall', report.get('camera', {}).get('timestamp'))
    capture_monotonic = report.get('camera_sample_timestamp_monotonic')
    try:
        timestamp = float(timestamp)
        age = (max(0., float(now_monotonic) - float(capture_monotonic))
               if capture_monotonic is not None else max(0., float(now_wall) - timestamp))
    except (TypeError, ValueError):
        return {'freshness': 'invalid', 'reason': 'camera capture timestamp missing or invalid',
                'pointcloud_cache_reusable': False}
    if not math.isfinite(timestamp) or timestamp > now_wall + .05:
        return {'freshness': 'invalid', 'reason': 'camera capture timestamp is invalid or in the future',
                'age_seconds': age, 'pointcloud_cache_reusable': False}
    pose_delta = report.get('camera_sample_pose_delta_seconds')
    if phase.get('event_monotonic') is not None:
        if pose_delta is None:
            if capture_monotonic is not None:
                pose_delta = abs(float(capture_monotonic) - float(phase['event_monotonic']))
            else:
                monotonic_wall_offset = float(now_wall) - float(now_monotonic)
                phase_wall = float(phase['event_monotonic']) + monotonic_wall_offset
                pose_delta = abs(timestamp - phase_wall)
    live_sample = report.get('camera_sample_timestamp_source') in (
        'camera latest.json', 'camera timestamps.jsonl')
    captured_pose = report.get('camera_sample_pose_raw')
    resident_pose = phase.get('pose_raw') if phase else None
    raw_pose_match = None
    if captured_pose is not None and resident_pose is not None:
        try:
            raw_pose_match = ({str(name): int(value) for name, value in captured_pose.items()}
                              == {str(name): int(value) for name, value in resident_pose.items()})
        except (AttributeError, TypeError, ValueError):
            raw_pose_match = False
    pose_is_fresh = phase.get('freshness') not in ('stale', 'invalid', 'missing')
    pairing_required = (live_sample or capture_monotonic is not None or captured_pose is not None
                        or phase.get('event_monotonic') is not None)
    qualified_sync = report.get('camera_sample_pose_qualified_sync')
    paired = (not pairing_required or
              (pose_delta is not None and abs(float(pose_delta)) <= POSE_PAIR_LIMIT_S
               and (not live_sample or pose_is_fresh)
               and qualified_sync is not False
               and raw_pose_match is not False))
    fresh = age <= FRESHNESS_LIMIT_S and paired
    if fresh:
        reason = None
    elif age > FRESHNESS_LIMIT_S:
        reason = 'camera sample is older than freshness limit'
    elif not pose_is_fresh and live_sample:
        reason = 'resident pose publication is stale or invalid; camera sample is unpaired'
    elif raw_pose_match is False:
        reason = 'camera sample raw pose does not match resident pose; sample is unpaired'
    else:
        reason = 'camera and resident pose timestamps differ by more than 160 ms'
    return {'freshness': 'fresh' if fresh else 'stale',
            'pairing_status': 'paired' if paired else 'unpaired',
            'age_seconds': age,
            'pose_sample_delta_seconds': pose_delta,
            'raw_pose_match': raw_pose_match,
            'pointcloud_cache_reusable': bool(cache_reusable),
            'reason': reason}
