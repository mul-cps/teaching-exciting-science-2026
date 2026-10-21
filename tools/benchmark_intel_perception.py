#!/usr/bin/env python3
"""Offline whole-analyzer comparison. Never opens a camera, ROS or a controller."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2/src/so101_station_objects')]
os.environ['SO101_STATION_ROOT'] = str(ROOT)
from so101_station_objects.objects import analyze_rgbd
from so101_station_objects.color_mask_backend import ColorMaskBackend


def counters():
    result = {}
    for path in Path('/proc/self/fdinfo').glob('*'):
        try:
            for line in path.read_text().splitlines():
                match = re.match(r'(drm-engine-[^:]+):\s*(\d+)\s*ns', line)
                if match:
                    result[str(path) + ':' + match[1]] = int(match[2])
        except OSError:
            pass
    return result


def canonical(report):
    report = dict(report)
    report.pop('color_mask_backend', None)
    return json.dumps(report, sort_keys=True, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backend', choices=['vulkan', 'opencl'], default='vulkan')
    parser.add_argument('--repetitions', type=int, default=9)
    args = parser.parse_args()
    if args.repetitions < 3:
        parser.error('at least three repetitions required')
    cv2.setNumThreads(1)
    base = args.inputs
    meta = json.loads((base / 'capture.json').read_text())
    protocol = json.loads((base / 'object-protocol.json').read_text())
    for name, expected in protocol['capture_sha256'].items():
        assert hashlib.sha256((base / name).read_bytes()).hexdigest() == expected, name
    rgb = cv2.cvtColor(cv2.imread(str(base / 'color.png')), cv2.COLOR_BGR2RGB)
    depth = np.load(base / 'depth.npy', allow_pickle=False)
    backend = ColorMaskBackend(args.backend)
    cpu = ColorMaskBackend('cpu')
    cases = []
    for width, height in [(1920, 1080), (960, 540), (640, 480)]:
        image = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)
        z = cv2.resize(depth, (width, height), interpolation=cv2.INTER_NEAREST)
        k = np.array(meta['camera']['k']).reshape(3, 3).copy()
        sx, sy = width / rgb.shape[1], height / rgb.shape[0]
        k[0, :] *= sx; k[1, :] *= sy
        k[0, 2] += .5 * sx - .5; k[1, 2] += .5 * sy - .5
        intrinsics = dict(fx=k[0, 0], fy=k[1, 1], ppx=k[0, 2], ppy=k[1, 2])
        # Deterministic derived frames expose stale buffers and color boundaries.
        # These are synthetic variants of one recorded scene, not independent captures.
        dark = np.rint(image.astype(np.float32) * .9).astype(np.uint8)
        shifted = np.roll(image, 7, axis=1); shifted_z = np.roll(z, 7, axis=1)
        variants = [('recorded', image, z), ('brightness_0.9', dark, z),
                    ('shift_x7', shifted, shifted_z)]
        for name, frame, depths in variants:
            call_args = frame, depths, intrinsics, meta['camera']['d'], meta['depth_scale_m'], np.array(meta['world_from_camera'])
            def run(selected):
                return analyze_rgbd(*call_args, **protocol['kwargs'], color_mask_backend=selected)
            for _ in range(2):
                assert canonical(run(cpu)) == canonical(run(backend))
            assert backend.status()['selected'] == args.backend, backend.status()
            before = counters()
            times = {'cpu': [], args.backend: []}; cpu_times = {key: [] for key in times}
            for iteration in range(args.repetitions):
                results = {}
                order = [('cpu', cpu), (args.backend, backend)]
                for label, selected in (order if iteration % 2 == 0 else order[::-1]):
                    start_cpu = time.process_time(); start = time.perf_counter()
                    results[label] = run(selected)
                    times[label].append((time.perf_counter() - start) * 1000)
                    cpu_times[label].append((time.process_time() - start_cpu) * 1000)
                assert canonical(results['cpu']) == canonical(results[args.backend]), (width, height, name)
                assert backend.status()['selected'] == args.backend, backend.status()
            after = counters()
            delta = {key: after[key] - value for key, value in before.items() if key in after}
            assert any(value > 0 for value in delta.values()), 'No own DRM engine activity observed'
            stats = {label: {'all_wall_ms': values, 'p50_ms': float(np.median(values)),
                            'p95_ms': float(np.percentile(values, 95)),
                            'all_cpu_ms': cpu_times[label], 'cpu_p50_ms': float(np.median(cpu_times[label]))}
                     for label, values in times.items()}
            case = {'size': [width, height], 'variant': name, 'exact_output_parity': True,
                    'rgb_sha256': hashlib.sha256(frame.tobytes()).hexdigest(),
                    'depth_sha256': hashlib.sha256(depths.tobytes()).hexdigest(),
                    'objects': len(results['cpu']['objects']), 'backend': backend.status(),
                    'drm_engine_delta_ns': delta, 'timings': stats}
            cases.append(case)
            print(json.dumps(case), flush=True)
    sources = [ROOT / 'tools/benchmark_intel_perception.py', ROOT / 'tools/propose_rgbd_grasp.py']
    sources += list((ROOT / 'ros2/src/so101_station_objects/so101_station_objects').glob('*.py'))
    sources += list((ROOT / 'ros2/src/so101_station_objects/so101_station_objects').glob('*.slang'))
    report = {'schema': 'intel-perception-offline-v1', 'qualified': False,
              'scope': 'Saved RGB-D plus deterministic synthetic variants of one capture; no live-rate or physical qualification',
              'source_commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
              'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              'input_sha256': protocol['capture_sha256'], 'kwargs': protocol['kwargs'],
              'python': sys.version, 'opencv': cv2.__version__, 'numpy': np.__version__,
              'environment': {key: os.environ.get(key) for key in ('VK_DRIVER_FILES', 'VK_ICD_FILENAMES', 'OCL_ICD_VENDORS', 'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')},
              'opencv_threads': cv2.getNumThreads(), 'warmups': 2, 'repetitions': args.repetitions,
              'cases': cases}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
