#!/usr/bin/env python3
"""Reproduce frozen procedural RGB-D cases and run an explicit offline matrix.

No camera access. Recorded data is included only with --recorded-fixture.
Remote execution requires explicit --ssh-host/--remote-directory; only that
specified directory receives generated inputs and results.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time

import numpy as np
from benchmark_resident_rgbd import qualify_dump


def procedural_cases():
    width, height = 128, 96
    yy, xx = np.indices((height, width))
    raw = np.full((height, width), 500, dtype=np.uint16)
    rgb = np.zeros((height, width, 3), np.uint8)
    rgb[4:-4, 4:-4, 0] = 255
    rgb[12::7, 12::9] = 0
    yield 'holes', rgb.copy(), raw.copy(), {}
    rgb[:] = 0
    distance = np.minimum.reduce([xx, yy, width-1-xx, height-1-yy])
    rgb[distance % 4 == 0, 0] = 255
    yield 'rings', rgb.copy(), raw.copy(), {}
    rgb[:] = 0
    rgb[np.arange(96), np.arange(96), 2] = 255
    yield 'diagonal', rgb.copy(), raw.copy(), {}
    rgb[:] = 0
    rgb[:, :, 1] = 255
    split = raw.copy()
    split[:, width//2:] = 520
    yield 'depthsplit', rgb.copy(), split, {}
    rgb[:] = 0
    rgb[(xx+yy) % 2 == 0, 0] = 255
    rgb[(xx+yy) % 2 == 1, 1] = 255
    yield 'overlap_checker', rgb.copy(), raw.copy(), {}
    rgb[:] = 0
    for row in range(0, height, 2):
        rgb[row, :, 0] = 255
        if row+1 < height:
            rgb[row+1, width-1 if row//2 % 2 == 0 else 0, 0] = 255
    yield 'long_snake', rgb.copy(), raw.copy(), {}
    palette = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]], np.uint8)
    yy, xx = np.indices((192, 640))
    rgb = palette[(xx//4) % 2 + 2*((yy//6) % 2)]
    yield 'capacity5120', rgb, np.full((192, 640), 500, np.uint16), {'expected_fault': 2}
    # Exact native regression: 4096small roots followed by one5120pixel root.
    yy, xx = np.indices((224, 512))
    rgb = np.zeros((224, 512, 3), np.uint8)
    rgb[:192] = palette[(xx[:192]//4) % 2 + 2*((yy[:192]//6) % 2)]
    rgb[194:204, :, 0] = 255
    yield 'overflow_large', rgb, np.full((224, 512), 500, np.uint16), {'expected_fault': 2}


def source_hashes(rgb, depth):
    return {'rgb.bin': hashlib.sha256(rgb.tobytes()).hexdigest(),
            'depth.bin': hashlib.sha256(depth.astype('<u2', copy=False).tobytes()).hexdigest()}


def prepare(directory, recorded_fixture=None):
    directory.mkdir(parents=True, exist_ok=True)
    cases = list(procedural_cases())
    if recorded_fixture is not None:
        import cv2
        fixture = Path(recorded_fixture)
        record = json.loads((fixture/'capture.json').read_text())
        info = record['messages']['color_info']['message']
        if any(info['d']) or 'millimeters' not in record['messages']['aligned_depth']['metric_units']:
            raise ValueError('recorded fixture requires zero distortion and explicit millimeter Z16')
        bgr = cv2.imread(str(fixture/'color.png'))
        if bgr is None:
            raise ValueError('recorded color.png missing')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        raw = np.load(fixture/'aligned_depth-raw.npy', allow_pickle=False)
        if raw.dtype != np.uint16 or raw.shape != rgb.shape[:2]:
            raise ValueError('original matching aligned uint16/RGB image sizes required')
        k = info['k']
        cases.append(('recorded', rgb, raw, {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5],
                      'source_sha256': {name: hashlib.sha256((fixture/name).read_bytes()).hexdigest()
                                        for name in ('color.png', 'aligned_depth-raw.npy', 'capture.json')},
                      'source_stamp': record['messages']['aligned_depth']['header']}))
    for name, rgb, depth, extra in cases:
        target = directory/name
        target.mkdir(exist_ok=True)
        rgb.tofile(target/'rgb.bin')
        depth.astype('<u2', copy=False).tofile(target/'depth.bin')
        metadata = {'name': name, 'width': depth.shape[1], 'height': depth.shape[0],
                    'fx': 600., 'fy': 600., 'cx': depth.shape[1]/2, 'cy': depth.shape[0]/2,
                    'depth_scale': .001, **extra, 'input_sha256': source_hashes(rgb, depth)}
        (target/'case.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return [name for name, *_ in cases]


def run(args):
    prefix = json.loads(args.runner_prefix_json)
    if not isinstance(prefix, list) or not all(isinstance(item, str) for item in prefix):
        raise ValueError('--runner-prefix-json must be a JSON array of executable/arguments')
    if bool(args.ssh_host) != bool(args.remote_directory):
        raise ValueError('--ssh-host and --remote-directory must be supplied together')
    if args.ssh_host and not Path(args.remote_directory).is_absolute():
        raise ValueError('--remote-directory must be absolute')
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for path in sorted(args.inputs.iterdir()):
        if not (path/'case.json').exists() or args.case and path.name not in args.case:
            continue
        metadata = json.loads((path/'case.json').read_text())
        # Fail before running if a frozen input has drifted.
        for name, expected in metadata['input_sha256'].items():
            if hashlib.sha256((path/name).read_bytes()).hexdigest() != expected:
                raise ValueError(f'frozen input digest mismatch: {path/name}')
        local_output = args.output/path.name
        local_output.mkdir(exist_ok=True)
        input_path, output_path = str(path.resolve()), str(local_output.resolve())
        if args.ssh_host:
            input_path = str(Path(args.remote_directory)/'inputs'/path.name)
            output_path = str(Path(args.remote_directory)/'outputs'/path.name)
            subprocess.run(['ssh', args.ssh_host, shlex.join(['mkdir', '-p', input_path, output_path])], check=True)
            subprocess.run(['rsync', '-a', str(path)+'/', args.ssh_host+':'+input_path+'/'], check=True)
        native = ['/usr/bin/time', '-v', '-o', output_path+'/time.txt', args.binary,
                  '--backend', args.backend, '--rgb', input_path+'/rgb.bin', '--depth', input_path+'/depth.bin',
                  '--width', str(metadata['width']), '--height', str(metadata['height']),
                  '--frames', str(args.frames), '--warmup', str(args.warmup),
                  '--fx', str(metadata['fx']), '--fy', str(metadata['fy']),
                  '--cx', str(metadata['cx']), '--cy', str(metadata['cy']), '--depth-scale', '.001',
                  '--output', output_path+'/native.json']
        if args.timestamps:
            native += ['--timestamps', '1']
        if args.shaders:
            native += ['--shaders', args.shaders]
        if args.backend == 'compare':
            native += ['--dump', output_path+'/scratch.bin']
        command = prefix+native
        if args.ssh_host:
            command = ['ssh', args.ssh_host, shlex.join(command)]
        start = time.monotonic()
        execution = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if args.ssh_host:
            subprocess.run(['rsync', '-a', args.ssh_host+':'+output_path+'/', str(local_output)+'/'], check=True)
        (local_output/'run.log').write_text(execution.stdout)
        row = {'case': path.name, 'returncode': execution.returncode,
               'wall_seconds_with_runner': time.monotonic()-start,
               'input_sha256': metadata['input_sha256'], 'backend_requested': args.backend,
               'full_station_qualified': False}
        if 'expected_fault' in metadata:
            fault = metadata['expected_fault']
            row.update(expected_fault=fault, fault_refused=execution.returncode != 0 and
                       f'fault: {fault}' in execution.stdout)
            if not row['fault_refused']:
                raise RuntimeError(row)
        elif execution.returncode:
            row['error'] = execution.stdout
            results.append(row)
            (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
            raise RuntimeError(row)
        else:
            row.update(json.loads((local_output/'native.json').read_text()))
            if args.backend == 'compare':
                rgb = np.fromfile(path/'rgb.bin', np.uint8).reshape(metadata['height'], metadata['width'], 3)
                raw = np.fromfile(path/'depth.bin', '<u2').reshape(metadata['height'], metadata['width'])
                row.update(qualify_dump(local_output/'scratch.bin', rgb, raw, .001))
        results.append(row)
        (args.output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(row), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    generate = commands.add_parser('prepare')
    generate.add_argument('--directory', type=Path, required=True)
    generate.add_argument('--recorded-fixture', type=Path)
    execute = commands.add_parser('run')
    execute.add_argument('--inputs', type=Path, required=True)
    execute.add_argument('--output', type=Path, required=True)
    execute.add_argument('--binary', required=True)
    execute.add_argument('--shaders')
    execute.add_argument('--runner-prefix-json', default='[]')
    execute.add_argument('--ssh-host')
    execute.add_argument('--remote-directory')
    execute.add_argument('--backend', choices=['compare', 'vulkan', 'cpu'], default='compare')
    execute.add_argument('--frames', type=int, default=60)
    execute.add_argument('--warmup', type=int, default=10)
    execute.add_argument('--timestamps', action='store_true')
    execute.add_argument('--case', action='append')
    args = parser.parse_args()
    if args.command == 'prepare':
        print(json.dumps(prepare(args.directory, args.recorded_fixture)))
    else:
        run(args)


if __name__ == '__main__':
    main()
