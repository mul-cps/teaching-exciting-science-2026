"""Offline CPU benchmark of RGB-D projection, startup registration and virtual FK.

Optional --snapshot reads saved capture.json and depth; no devices are opened.
Registration uses saved RGB-D/readback pairs and runs once without warmup.
Saved-series/global-tracker timing includes its file IO and diagnostic PNGs;
projection/FK timings exclude file loading, upload, compression and GPU rendering.
No timing threshold is used for correctness tests on a shared workstation.
"""
import argparse
import hashlib
import itertools
import json
import platform
from pathlib import Path
import statistics
import time

import numpy as np

from workshop.digital_twin.scene import fit_table, make_model, unproject


def measure(operation, repetitions, warmup=True):
    if warmup: operation()  # Warm reusable imports/allocators outside the samples.
    samples = []
    for _ in range(repetitions):
        start = time.perf_counter_ns()
        result = operation()
        samples.append((time.perf_counter_ns() - start) / 1e6)
    return result, {'samples': len(samples), 'median_ms': statistics.median(samples),
                    'p95_ms': float(np.percentile(samples, 95)), 'max_ms': max(samples)}


def benchmark(snapshot=None, repetitions=5, fk_repetitions=100, registration=False):
    if any(type(value) is not int or not 1 <= value <= 10000 for value in (repetitions, fk_repetitions)):
        raise ValueError('integer repetitions in 1..10000 required')
    provenance = {'kind': 'synthetic planar 640x480 depth fixture'}
    if snapshot is None:
        depth = np.full((480, 640), 800, dtype=np.uint16)
        metadata = {'intrinsics': dict(fx=605., fy=605., ppx=320., ppy=240., coeffs=[0.] * 5),
                    'depth_scale_m': .001}
    else:
        snapshot = Path(snapshot)
        capture = snapshot / 'capture.json'
        metadata = json.loads(capture.read_text())
        path = snapshot / 'home-recheck-depth.npy'
        if not path.is_file(): path = snapshot / 'aligned-depth.npy'
        depth = np.load(path, allow_pickle=False)
        provenance = {'kind': 'saved RGB-D capture', 'depth': str(path),
                      'capture_sha256': hashlib.sha256(capture.read_bytes()).hexdigest(),
                      'depth_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    (points, valid), projection = measure(
        lambda: unproject(depth, metadata['intrinsics'], metadata['depth_scale_m']), repetitions)
    assert points.shape == (*depth.shape, 3) and valid.shape == depth.shape
    roi = np.zeros(depth.shape, dtype=bool); roi[340:470, 180:580] = True
    selected = points[roi & valid][::3]
    (center, basis, evidence), table = measure(lambda: fit_table(selected), repetitions)
    assert np.isfinite(center).all() and np.linalg.det(basis) > .999
    assert evidence['sample_count'] == len(selected)
    world, transform = measure(lambda: (points - center) @ basis, repetitions)
    assert world.shape == points.shape and np.isfinite(world[valid]).all()
    import mujoco
    from workshop.digital_twin.kinematics import export_fk, set_pose
    model = mujoco.MjModel.from_xml_string(make_model([.2, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    pose = dict(shoulder_pan=-10, shoulder_lift=-110, elbow_flex=97,
                wrist_flex=76, wrist_roll=2, gripper=4)
    pose_sequence = [pose]
    pose_log = None if snapshot is None else snapshot / 'scene-calibration-recorded-result.json'
    if pose_log is not None and pose_log.is_file():
        pose_sequence = [record['dwell_end_position'] for record in json.loads(pose_log.read_text())['records']]
        if not pose_sequence: raise ValueError('saved pose recording is empty')
        provenance['pose_kind'] = 'actual saved dwell-end joint readbacks, cycled through interactive FK'
        provenance['pose_log_sha256'] = hashlib.sha256(pose_log.read_bytes()).hexdigest()
    else:
        provenance['pose_kind'] = 'synthetic fixed virtual pose'
    poses = itertools.cycle(pose_sequence)
    _, fk = measure(lambda: set_pose(model, data, next(poses)), fk_repetitions)
    assert np.isfinite(data.geom_xpos).all() and model.nu == 6
    exported = export_fk(model, data)
    assert len(exported['joints']) == 6
    registration_result = None
    if registration:
        registration_result = benchmark_registration(snapshot, metadata, center, basis, model)
    return {'python': platform.python_version(), 'platform': platform.platform(),
            'numpy': np.__version__, 'mujoco': mujoco.__version__, 'provenance': provenance,
            'image_shape': list(depth.shape), 'valid_pixels': int(valid.sum()),
            'table_samples': len(selected), 'geometries': model.ngeom,
            'repetitions': repetitions, 'fk_repetitions': fk_repetitions, 'distinct_pose_inputs': len(pose_sequence),
            'projection': projection, 'table_fit': table, 'world_transform': transform,
            'virtual_set_pose': fk, 'table_evidence': evidence, 'registration': registration_result,
            'scope': 'actual CPU functions; registration and table fit are startup work; saved-series loading includes file IO; other timings exclude file IO, upload, compression, GPU rendering and hardware'}


def benchmark_registration(snapshot, metadata, center, basis, model):
    """Startup fits run once without warmup; no render callbacks or devices."""
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.registration import (
        cad_surface, fit_registration, fit_pose_series, load_pose_series,
    )
    from workshop.digital_twin.kinematics import set_pose
    import mujoco
    result = {'status': 'UNQUALIFIED startup fit; no global IK or sim-to-real acceptance',
              'model_convention': 'new', 'hardware_access': False}
    if snapshot is None:
        rng = np.random.default_rng(19)
        cad = rng.normal(size=(100, 3)) * [.03, .015, .06]
        observed = cad @ Rotation.from_euler('z', 135, degrees=True).as_matrix().T + [.15, .13, .01]
        fit, timing = measure(lambda: fit_registration(cad, observed, yaw_seeds=(120, 140)), 1, warmup=False)
        result.update(data_kind='synthetic asymmetric surface', single_pose={'timing': timing, 'evidence': fit})
        return result
    snapshot = Path(snapshot)
    result_name, folder = 'scene-calibration-recorded-result.json', 'scene-calibration-recorded'
    if not (snapshot / result_name).is_file():
        result_name, folder = 'visible-pan-result.json', 'visible-pan-retry'
    if not (snapshot / result_name).is_file():
        raise ValueError('saved registration benchmark requires an actual paired pose-series recording')
    frames, preparation = measure(lambda: load_pose_series(
        snapshot, center, basis, metadata, result_name=result_name, folder=folder), 1, warmup=False)
    if len(frames) < 3:
        raise ValueError('saved registration benchmark requires at least three admitted paired frames')
    source_files = [snapshot / result_name, snapshot / folder / 'timestamps.jsonl']
    source_files += [path for frame in frames for path in (Path(frame['image']), Path(frame['image']).with_suffix('.npy'))]
    result.update(data_kind='saved RGB-D frames and actual dwell-end readbacks',
                  frame_count=len(frames), labels=[frame['label'] for frame in frames],
                  maximum_pairing_difference_s=max(frame['timestamp_difference_s'] for frame in frames),
                  hashes={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_files},
                  saved_series_preparation={'timing': preparation, 'includes': 'saved file IO, projection, segmentation and vectorized patch tracking'})
    data = mujoco.MjData(model)
    set_pose(model, data, frames[0]['degrees'])
    cad = cad_surface(model, data, count=3500)
    fit, timing = measure(lambda: fit_registration(cad, frames[0]['observed']), 1, warmup=False)
    result['single_pose'] = {'timing': timing, 'evidence': fit}
    fit, timing = measure(lambda: fit_pose_series(model, frames), 1, warmup=False)
    result['multi_pose'] = {'timing': timing, 'evidence': fit}
    from tempfile import TemporaryDirectory
    from workshop.digital_twin.tracking import fit_tracked_tcp
    world_from_camera = np.eye(4)
    world_from_camera[:3, :3] = basis.T
    world_from_camera[:3, 3] = -center @ basis
    if result_name == 'scene-calibration-recorded-result.json':
        # This public startup path also writes annotation PNGs. Keep its IO in
        # a private temporary directory and report that boundary explicitly.
        with TemporaryDirectory(prefix='twin-registration-benchmark-') as output:
            fit, timing = measure(lambda: fit_tracked_tcp(
                snapshot, metadata, output, world_from_camera, convention='new'), 1, warmup=False)
        result['tracked_feature_global'] = {'timing': timing, 'evidence': fit,
            'includes': 'saved RGB-D file IO, feature tracking, candidate fits and diagnostic PNG writing'}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--registration', action='store_true', help='Run bounded startup registration once, separately from interactive FK')
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--fk-repetitions', type=int, default=100)
    args = parser.parse_args()
    print(json.dumps(benchmark(args.snapshot, args.repetitions, args.fk_repetitions, args.registration), indent=2))


if __name__ == '__main__': main()
