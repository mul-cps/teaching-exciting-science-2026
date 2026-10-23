"""Freeze two CPU-oracle GEOQv1 fixtures. Never runs a GPU or software math helper."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import struct
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools import qualify_native_preparation as retained


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    ordinary = retained.scenes()['all_valid']
    dense = copy.deepcopy(ordinary)
    x, y, z = .31, -.27, .19
    rx = np.array([[1., 0., 0.], [0., np.cos(x), -np.sin(x)],
                   [0., np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0., np.sin(y)], [0., 1., 0.],
                   [-np.sin(y), 0., np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0.],
                   [np.sin(z), np.cos(z), 0.], [0., 0., 1.]])
    rotation = rz @ ry @ rx
    assert np.all(rotation != 0) and np.all(np.abs(rotation) != 1)
    assert np.allclose(rotation @ rotation.T, np.eye(3), rtol=0, atol=1e-15)
    assert abs(np.linalg.det(rotation) - 1) < 1e-15
    dense['board_from_camera'][:3, :3] = rotation
    dense['board_from_camera'][:3, 3] = [.017, -.023, .041]
    sources = list(retained.oracle.SOURCES) + [
        'tools/qualify_native_preparation.py',
        'tools/performance/prepare_geometry_benchmark.py',
        'ros2/src/so101_station_vulkan/test/qualify_geometry.cpp',
        'ros2/src/so101_station_vulkan/src/geometry.cpp',
        'ros2/src/so101_station_vulkan/src/vulkan_context.cpp',
        'ros2/src/so101_station_vulkan/include/so101_station_vulkan/geometry.hpp',
        'ros2/src/so101_station_vulkan/include/so101_station_vulkan/pipeline.hpp',
        'ros2/src/so101_station_vulkan/shaders/geometry.slang',
        'ros2/src/so101_station_vulkan/shaders/geometry_soft.slang',
        'ros2/src/so101_station_vulkan/shaders/soft_binary64.slang',
    ]
    manifest = dict(schema='native-preparation-diagnostic-v1',
                    benchmark_scope='repeated-single-resident-source-preparation-only',
                    physical=False, coordinate_tolerance=1e-12,
                    alignment_tolerance=retained.ALIGNMENT_TOLERANCE,
                    numpy=np.__version__, opencv=cv2.__version__, cases={},
                    sources={p: digest(ROOT/p) for p in sorted(set(sources))},
                    qualification_required_before_timing=True)
    for name, arguments in [('all_valid', ordinary), ('full_xyz_rotation', dense)]:
        reference = retained.oracle.preparation_reference(arguments)
        h, w = arguments['depth'].shape
        assert (w, h) == (640, 480)
        masks = retained.oracle.color_masks(arguments['rgb'])
        vivid = np.logical_or.reduce(tuple(masks.values())).astype('<u4')
        depth = (arguments['depth'].astype(np.float32) *
                 arguments['depth_scale_m']).astype('<f4')
        binary = directory/(name+'.bin')
        with binary.open('xb') as stream:
            stream.write(struct.pack('<8sIIQdd', b'GEOQv1\0\0', w, h, 17,
                                     arguments['depth_scale_m'],
                                     arguments.get('max_observation_depth_m', 1.2)))
            stream.write(np.asarray(arguments['board_from_camera'][:3], dtype='<f8').tobytes())
            stream.write(np.asarray(arguments['support_roi'], dtype='<f8').tobytes())
            stream.write(reference['rays'].astype('<f8').tobytes())
            stream.write(vivid.tobytes())
            stream.write(depth.tobytes())
        expected = directory/(name+'.npz')
        np.savez(expected, **reference, rgb=arguments['rgb'],
                 depth_z16=arguments['depth'], distortion=arguments['distortion'],
                 transform=arguments['board_from_camera'], vivid=vivid)
        valid, support = int(reference['valid_count']), len(reference['selected_pixels'])
        manifest['cases'][name] = dict(
            width=w, height=h, generation=17, fixture_sha256=digest(binary),
            expected_sha256=digest(expected), expected_valid=valid,
            expected_support=support,
            benchmark_count_arguments=['--expected-valid', str(valid),
                                       '--expected-support', str(support)])
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(json.dumps(prepare(parser.parse_args().output)))
