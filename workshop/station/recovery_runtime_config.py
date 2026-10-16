"""Pinned recovery configuration parsing without collision geometry construction.

Used only during trusted server startup, never from a ROS caller path or tick.
The owner factory separately verifies the constructed collision-model digest.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def validate_registration_transform(value):
    value = np.asarray(value, dtype=np.float64)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1., atol=1e-8)):
        raise ValueError('a finite proper rigid frame transform is required')
    return value.copy()


def pinned_artifact(entry, directory, maximum=16*1024*1024):
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


def load_recovery_runtime_config(config, state_file):
    """Return pinned authority inputs, or None when explicitly disabled."""
    option = config.get('arm', {}).get('camera_recovery')
    if not option or option.get('enabled') is not True:
        return None
    manifest_bytes, manifest_path = pinned_artifact(option['manifest'], Path(state_file).parent)
    manifest = json.loads(manifest_bytes)
    arm = config['arm']
    if (manifest.get('schema') != 'so101-camera-recovery-runtime-v1' or
            manifest.get('qualified') is not True or manifest.get('arm_id') != arm['id'] or
            manifest.get('arm_calibration_sha256') != arm['calibration']['sha256']):
        raise ValueError('qualified identity-matched recovery runtime manifest required')
    mapping_bytes, _ = pinned_artifact(manifest['joint_mapping'], manifest_path.parent)
    mapping = json.loads(mapping_bytes)
    if (mapping.get('schema') != 'recovery-joint-mapping-v1' or
            mapping.get('qualified') is not True or mapping.get('arm_id') != arm['id'] or
            mapping.get('arm_calibration_sha256') != arm['calibration']['sha256']):
        raise ValueError('qualified physical-arm-matched joint mapping required')
    urdf, _ = pinned_artifact(manifest['effective_urdf'], manifest_path.parent)
    registration_bytes, _ = pinned_artifact(manifest['registration'], manifest_path.parent)
    registration = json.loads(registration_bytes)
    if (registration.get('schema') != 'camera-arm-calibration-v1' or
            registration.get('qualified') is not True or
            registration.get('arm_calibration_sha256') != arm['calibration']['sha256'] or
            registration.get('model_sha256') != manifest['model_sha256'] or
            registration.get('base_frame') != manifest['base_frame'] or
            not isinstance(registration.get('camera_serial'), str) or not registration['camera_serial'] or
            not isinstance(registration.get('camera_frame'), str) or not registration['camera_frame']):
        raise ValueError('qualified model/arm/camera-matched base registration required')
    registration['base_from_camera'] = validate_registration_transform(registration['base_from_camera'])
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
    return dict(manifest=manifest, mapping=mapping, urdf=urdf, registration=registration, arm=arm)
