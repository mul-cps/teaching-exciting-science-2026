"""Saved-frame exact-output/latency comparison for local component buffers.

No ROS, camera SDK or device calls. Baseline code is read from a Git revision;
component support and all admission flags must match, numeric outputs use only
1e-9 absolute tolerance for equivalent cropped-intrinsics arithmetic.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import types

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
BASELINE_REVISION = '522a6584106a9650811e9a238a540d6c0068a842'
sys.path[:0] = [str(ROOT), str(ROOT/'ros2/src/so101_station_objects')]
from so101_station_objects import objects


def baseline_analyzer(revision=BASELINE_REVISION):
    """Load unchanged baseline helpers as distinct modules, never edit a checkout."""
    prefix = 'ros2/src/so101_station_objects/so101_station_objects/'
    modules = []
    for name in ('colored_object_validation', 'objects'):
        path = prefix+name+'.py'
        source = subprocess.check_output(['git', 'show', revision+':'+path], cwd=ROOT, text=True)
        module = types.ModuleType('_locality_baseline_'+name)
        module.__file__ = str(ROOT/path)
        sys.modules[module.__name__] = module
        exec(compile(source, '<baseline '+path+'>', 'exec'), module.__dict__)
        modules.append(module)
    for name in ('split_depth_coherent_components', 'validate_colored_component'):
        setattr(modules[1], name, getattr(modules[0], name))
    return modules[1].analyze_rgbd


def assert_same(before, after, path='result'):
    if isinstance(before, dict):
        assert isinstance(after, dict) and before.keys() == after.keys(), path
        for key in before:
            assert_same(before[key], after[key], path+'.'+str(key))
    elif isinstance(before, (list, tuple)):
        assert isinstance(after, (list, tuple)) and len(before) == len(after), path
        for i, (a, b) in enumerate(zip(before, after)):
            assert_same(a, b, path+'['+str(i)+']')
    elif isinstance(before, (str, bool, type(None), int, np.integer)):
        assert before == after, (path, before, after)
    else:
        np.testing.assert_allclose(after, before, rtol=0, atol=1e-9, err_msg=path)


def load_capture(directory):
    directory = Path(directory)
    metadata_bytes = (directory/'capture.json').read_bytes()
    metadata = json.loads(metadata_bytes)
    color_bytes = (directory/'color.png').read_bytes()
    rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(color_bytes, np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    path = directory/('depth.npy' if (directory/'depth.npy').is_file() else 'aligned-depth-raw.npy')
    depth_bytes = path.read_bytes()
    depth = np.load(path, allow_pickle=False)
    camera = metadata.get('camera', metadata.get('camera_info'))
    k = np.asarray(camera['k']).reshape(3, 3)
    intrinsics = dict(fx=k[0, 0], fy=k[1, 1], ppx=k[0, 2], ppy=k[1, 2])
    if 'world_from_camera' in metadata:
        world_from_camera = np.asarray(metadata['world_from_camera'])
    else:
        t = metadata['transforms']['charuco_board']['transform']
        camera_from_world = np.eye(4)
        camera_from_world[:3, :3] = Rotation.from_quat([t['rotation'][x] for x in 'xyzw']).as_matrix()
        camera_from_world[:3, 3] = [t['translation'][x] for x in 'xyz']
        world_from_camera = np.linalg.inv(camera_from_world)
    arguments = (rgb, depth, intrinsics, camera['d'], metadata.get('depth_scale_m', .001), world_from_camera)
    provenance = dict(capture=str(directory), shape=list(rgb.shape),
        capture_sha256=hashlib.sha256(metadata_bytes).hexdigest(),
        rgb_sha256=hashlib.sha256(color_bytes).hexdigest(),
        depth_sha256=hashlib.sha256(depth_bytes).hexdigest())
    return arguments, provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, action='append', required=True)
    parser.add_argument('--baseline-ref', default=BASELINE_REVISION)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 20 or len(args.capture) > 8:
        parser.error('one to eight captures, one to twenty samples required')
    cv2.setNumThreads(1)
    original = baseline_analyzer(args.baseline_ref)
    records = []
    for directory in args.capture:
        inputs, record = load_capture(directory)
        durations = {'before': [], 'after': []}
        for n in range(args.samples):
            results = {}
            calls = [('before', original), ('after', objects.analyze_rgbd)]
            for name, analyze in calls if n % 2 == 0 else reversed(calls):
                started = time.perf_counter_ns()
                results[name] = analyze(*inputs)
                durations[name].append((time.perf_counter_ns()-started)/1e6)
            assert_same(results['before'], results['after'])
        record.update(parity=True, object_count=len(results['after']['objects']),
            validation_diagnostics=results['after']['validation_diagnostics'],
            timings={name: dict(samples=len(values), mean_ms=statistics.mean(values),
                median_ms=statistics.median(values), min_ms=min(values), max_ms=max(values),
                first_ms=values[0]) for name, values in durations.items()})
        records.append(record)
    report = dict(scope='offline exact frozen inputs; no live freshness or qualification claim',
        geometry_abs_tolerance=1e-9, acceptance_reasons_counts_exact=True,
        opencv=cv2.__version__, numpy=np.__version__, opencv_threads=cv2.getNumThreads(), loadavg=os.getloadavg(),
        baseline_ref=subprocess.check_output(['git', 'rev-parse', args.baseline_ref], cwd=ROOT, text=True).strip(),
        sources={name: hashlib.sha256((ROOT/'ros2/src/so101_station_objects/so101_station_objects'/name).read_bytes()).hexdigest()
                 for name in ('objects.py', 'colored_object_validation.py')}, records=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps([dict(capture=x['capture'], parity=x['parity'], objects=x['object_count'], timings=x['timings']) for x in records], indent=2))


if __name__ == '__main__':
    main()
