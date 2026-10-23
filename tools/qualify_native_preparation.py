"""Offline fixture preparation and explicitly gated GPU preparation diagnostics.

No camera, ROS service, object admission, or physical station claim is made.
The GPU command is intentionally separate from prepare and compare.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import native_geometry_oracle as oracle

COORDINATE_TOLERANCE = 1e-12
ALIGNMENT_TOLERANCE = 1e-7


def scenes():
    cases = {}
    for name, w, h in [('procedural', 128, 96), ('partial_grid', 137, 103),
                       ('all_valid', 640, 480), ('all_vivid', 137, 103),
                       ('hue_gaps', 180, 19), ('distortion4', 137, 103),
                       ('distortion5', 137, 103), ('distortion8', 137, 103)]:
        a = oracle.procedural_scene(w, h)
        if name == 'all_valid':
            a['rgb'][:] = 128
            a['support_roi'] = (-10., 10., -10., 10., -10., 10.)
        if name == 'all_vivid':
            a['rgb'][:] = [255, 0, 0]
        if name == 'partial_grid':
            a['depth'][::7, ::3] = 0
            a['depth'][::11, ::5] = 99
            a['depth'][::13, ::7] = 1201
        if name == 'hue_gaps':
            hsv = np.zeros((h, w, 3), dtype=np.uint8)
            hsv[..., 0] = np.arange(w, dtype=np.uint8)
            hsv[..., 1:] = 255
            a['rgb'] = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        if name.startswith('distortion'):
            count = int(name.removeprefix('distortion'))
            a['distortion'] = np.array([.04, -.01, .001, -.002, .003, .001, -.002, .0001][:count])
            angle = .17
            rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                                 [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
            a['board_from_camera'][:3, :3] = rotation @ a['board_from_camera'][:3, :3]
            a['board_from_camera'][:3, 3] += [.01, -.02, .003]
        cases[name] = a
    return cases


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(directory):
    directory.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema='native-preparation-diagnostic-v1', physical=False,
                    coordinate_tolerance=COORDINATE_TOLERANCE,
                    alignment_tolerance=ALIGNMENT_TOLERANCE,
                    numpy=np.__version__, opencv=cv2.__version__, cases={}, sources={})
    sources = list(oracle.SOURCES) + ['tools/qualify_native_preparation.py',
        'ros2/src/so101_station_vulkan/test/qualify_geometry.cpp',
        'ros2/src/so101_station_vulkan/src/geometry.cpp',
        'ros2/src/so101_station_vulkan/src/vulkan_context.cpp',
        'ros2/src/so101_station_vulkan/include/so101_station_vulkan/pipeline.hpp',
        'ros2/src/so101_station_vulkan/shaders/geometry.slang',
        'ros2/src/so101_station_vulkan/src/alignment.cpp']
    package = ROOT/'ros2/src/so101_station_vulkan'
    sources += [str(p.relative_to(ROOT)) for folder in ('src', 'include', 'shaders', 'cmake')
                for p in (package/folder).rglob('*') if p.is_file()]
    sources += ['ros2/src/so101_station_vulkan/CMakeLists.txt',
                'ros2/src/so101_station_vulkan/test/alignment_oracle.cpp']
    manifest['sources'] = {p: digest(ROOT/p) for p in sorted(set(sources))}
    for name, a in scenes().items():
        reference = oracle.preparation_reference(a)
        h, w = a['depth'].shape
        roi = a.get('support_roi', oracle.objects.DEFAULT_TABLE_ROI)
        masks = oracle.color_masks(a['rgb'])
        vivid = np.logical_or.reduce(tuple(masks.values())).astype('<u4')
        depth = (a['depth'].astype(np.float32)*a['depth_scale_m']).astype('<f4')
        binary = directory/(name+'.bin')
        with binary.open('wb') as f:
            f.write(struct.pack('<8sIIQdd', b'GEOQv1\0\0', w, h, 17, a['depth_scale_m'], a.get('max_observation_depth_m', 1.2)))
            f.write(np.asarray(a['board_from_camera'][:3], dtype='<f8').tobytes())
            f.write(np.asarray(roi, dtype='<f8').tobytes())
            f.write(reference['rays'].astype('<f8').tobytes())
            f.write(vivid.tobytes()); f.write(depth.tobytes())
        expected = directory/(name+'.npz')
        np.savez(expected, **reference, rgb=a['rgb'], depth_z16=a['depth'],
                 distortion=a['distortion'], transform=a['board_from_camera'], vivid=vivid)
        manifest['cases'][name] = dict(width=w, height=h, generation=17,
                                      fixture_sha256=digest(binary), expected_sha256=digest(expected))
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


def check_coordinates(actual, expected):
    if actual.shape != expected.shape or not np.array_equal(np.isfinite(actual), np.isfinite(expected)):
        raise AssertionError('coordinate shape or finite status differs')
    if not np.array_equal(np.isnan(actual), np.isnan(expected)) or not np.array_equal(np.isposinf(actual), np.isposinf(expected)) or not np.array_equal(np.isneginf(actual), np.isneginf(expected)):
        raise AssertionError('coordinate nonfinite values differ')
    error = float(np.max(np.abs(actual[np.isfinite(expected)]-expected[np.isfinite(expected)]), initial=0.))
    if error > COORDINATE_TOLERANCE:
        raise AssertionError(f'coordinate error {error} exceeds {COORDINATE_TOLERANCE}')
    return error


def compare_case(directory, name, result_prefix):
    manifest = json.loads((directory/'manifest.json').read_text())
    case = manifest['cases'][name]
    if digest(directory/(name+'.bin')) != case['fixture_sha256'] or digest(directory/(name+'.npz')) != case['expected_sha256']:
        raise AssertionError('frozen fixture changed')
    expected = np.load(directory/(name+'.npz'), allow_pickle=False)
    result = json.loads(Path(str(result_prefix)+'.json').read_text())
    if result.get('physical') is not False or result.get('generation') != case['generation'] or result.get('metadata_preserved') is not True:
        raise AssertionError('provenance differs')
    w, h = case['width'], case['height']; n = w*h; g = ((w+7)//8)*((h+7)//8); b = (g+255)//256
    selected_offset = (n*28+g*4+b*8+7)&~7
    arena = Path(str(result_prefix)+'.arena').read_bytes()
    if len(arena) != selected_offset+g*28:
        raise AssertionError('arena byte count differs')
    def array(dtype, count, offset):
        return np.frombuffer(arena, dtype=dtype, count=count, offset=offset)
    flags = array('<u4', n, 24*n).reshape(h, w)
    if not np.array_equal(flags, expected['flags']):
        raise AssertionError('valid/vivid/support ROI flags differ')
    count = len(expected['selected_pixels'])
    if result['support_count'] != count or result['valid_count'] != int(expected['valid_count']):
        raise AssertionError('compact counts differ')
    pixels = array('<u4', count, selected_offset+24*g)
    if not np.array_equal(pixels, expected['selected_pixels']):
        raise AssertionError('support pixel membership/order differs')
    errors = [check_coordinates(array('<f8', 3*n, 0).reshape(h, w, 3), expected['board']),
              check_coordinates(array('<f8', 3*count, selected_offset).reshape(-1, 3), expected['selected_board'])]
    return dict(case=name, max_coordinate_error_m=max(errors), valid_count=result['valid_count'], support_count=count, physical=False)


def alignment_result(output):
    matches = re.findall(r'PASS cases=(\d+) max_xyz_error_m=([\deE+.-]+) gpu=(\d+) physical=false', output)
    if len(matches) != 1:
        raise AssertionError('missing unique alignment result')
    cases, error, gpu = matches[0]
    if int(cases) != 8 or int(gpu) != 1 or not np.isfinite(float(error)) or float(error) > ALIGNMENT_TOLERANCE:
        raise AssertionError('outer alignment threshold/case/GPU gate failed')
    if output.count('cpu_sdk_mismatch=0') != 8:
        raise AssertionError('alignment SDK exact-depth cases missing')
    return dict(cases=8, max_xyz_error_m=float(error), threshold_m=ALIGNMENT_TOLERANCE, exact_depth=True, physical=False)


def run(args):
    if args.gpu_release != 'ROOT_RELEASED_GPU_PREPARATION':
        raise ValueError('explicit root GPU release required')
    directory = args.fixtures.resolve()
    manifest = json.loads((directory/'manifest.json').read_text())
    for path, value in manifest['sources'].items():
        if digest(ROOT/path) != value:
            raise AssertionError(f'source changed since freeze: {path}')
    args.output.mkdir(parents=True, exist_ok=False)
    artifacts = dict(physical=False, geometry_preparation_only=True, libraries={}, sources=manifest['sources'])
    for executable in (args.geometry, args.alignment):
        if executable:
            artifacts['libraries'][str(executable.resolve())] = digest(executable)
            deps = subprocess.run(['ldd', str(executable.resolve())], check=True, text=True, capture_output=True).stdout
            artifacts.setdefault('ldd', {})[str(executable)] = deps
            for library in re.findall(r'=> (/\S+)', deps):
                artifacts['libraries'][library] = digest(library)
    shader_dirs = [args.shaders, args.shaders.parent/'alignment-shaders', args.shaders.parent/'shaders']
    artifacts['shaders'] = {str(p.resolve()): digest(p) for folder in shader_dirs for p in folder.glob('*.spv')}
    artifacts['fixtures_manifest_sha256'] = digest(directory/'manifest.json')
    artifacts['comparator_numpy'] = np.__version__
    artifacts['comparator_opencv'] = cv2.__version__
    (args.output/'provenance.json').write_text(json.dumps(artifacts, indent=2)+'\n')
    mode = '--expect-fp64-rejection' if args.expect_fp64_rejection else '--vulkan'
    results = []
    for name in manifest['cases']:
        prefix = args.output/name
        command = [str(args.geometry.resolve()), mode, str(directory/(name+'.bin')), str(args.shaders.resolve()), str(prefix)]
        completed = subprocess.run(command, text=True, capture_output=True)
        (args.output/(name+'.log')).write_text(completed.stdout+completed.stderr)
        completed.check_returncode()
        if args.expect_fp64_rejection:
            rejection = json.loads(Path(str(prefix)+'.json').read_text())
            if rejection.get('fp64_rejection') is not True or rejection.get('physical') is not False:
                raise AssertionError('missing physical-feature rejection proof')
            results.append(rejection)
            break
        results.append(compare_case(directory, name, prefix))
    artifacts['geometry'] = results
    if not args.expect_fp64_rejection:
        if not args.alignment:
            raise ValueError('full preparation run requires alignment executable')
        output = subprocess.run([str(args.alignment.resolve()), '--vulkan'], check=True, text=True, capture_output=True)
        (args.output/'alignment.log').write_text(output.stdout+output.stderr)
        artifacts['alignment'] = alignment_result(output.stdout)
    artifacts['fixtures_manifest_sha256'] = digest(directory/'manifest.json')
    artifacts['comparator_numpy'] = np.__version__
    artifacts['comparator_opencv'] = cv2.__version__
    (args.output/'result.json').write_text(json.dumps(artifacts, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare'); p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('compare'); p.add_argument('--fixtures', type=Path, required=True); p.add_argument('--case', required=True); p.add_argument('--result-prefix', type=Path, required=True)
    p = sub.add_parser('run'); p.add_argument('--fixtures', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--geometry', type=Path, required=True); p.add_argument('--alignment', type=Path); p.add_argument('--shaders', type=Path, required=True)
    p.add_argument('--expect-fp64-rejection', action='store_true'); p.add_argument('--gpu-release', required=True)
    args = parser.parse_args()
    if args.command == 'prepare':
        print(json.dumps(prepare(args.output)))
    elif args.command == 'compare':
        print(json.dumps(compare_case(args.fixtures, args.case, args.result_prefix)))
    else:
        run(args)


if __name__ == '__main__':
    main()
