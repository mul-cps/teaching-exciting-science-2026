"""Validate frozen CPU child crop/order/support evidence. Offline, never a callback.

The fixture is a literal reference, with no regeneration CLI. Default OpenCV
8-connected CV_32S CCL supplies the complete candidate map; the actual retained
splitter supplies accepted order, regions and support. No native graph or GPU
supplies expected values. Row runs are lossless typed array storage, not hashes
standing in for membership. Depth symbols name literal IEEE binary32 bits.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2/src/so101_station_objects'))
from so101_station_objects.colored_object_validation import split_depth_coherent_components

FIXTURE = ROOT/'tests/fixtures/native_geometry/child-evidence-v1.json'
SOURCES = (
    'tools/native_geometry_child_evidence.py',
    'tests/test_native_geometry_child_evidence.py',
    'ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py',
    'ros2/src/so101_station_objects/so101_station_objects/objects.py',
)
DTYPES = {'bool': np.dtype('bool'), 'int32': np.dtype('<i4'), 'uint32': np.dtype('<u4')}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pack_array(array):
    """Canonical lossless representation for comparing actual arrays to literals."""
    array = np.asarray(array)
    if array.ndim != 2 or array.dtype.name not in DTYPES:
        raise ValueError('two dimensional bool/int32/uint32 array required')
    rows = []
    for row in array.tolist():
        if rows and rows[-1][1] == row:
            rows[-1][0] += 1
        else:
            rows.append([1, row])
    return {'dtype': array.dtype.name, 'shape': list(array.shape), 'row_runs': rows}


def unpack_array(value):
    dtype = DTYPES[value['dtype']]
    height, width = value['shape']
    assert type(height) is int and type(width) is int
    assert 1 <= height <= 2048 and 1 <= width <= 64
    rows = []
    scalar = bool if value['dtype'] == 'bool' else int
    for count, row in value['row_runs']:
        assert type(count) is int and 1 <= count <= height
        assert len(row) == width and all(type(v) is scalar for v in row)
        if scalar is int:
            limits = np.iinfo(dtype)
            assert all(limits.min <= v <= limits.max for v in row)
        assert len(rows)+count <= height
        rows.extend([row]*count)
    assert len(rows) == height
    return np.array(rows, dtype=dtype)


def assert_exact(expected, actual, path='evidence'):
    """No integer/boolean/float coercion or tolerance in this ABI."""
    assert type(expected) is type(actual), f'{path}: type {type(expected)} != {type(actual)}'
    if isinstance(expected, dict):
        assert expected.keys() == actual.keys(), f'{path}: fields differ'
        for key in expected:
            assert_exact(expected[key], actual[key], f'{path}.{key}')
    elif isinstance(expected, list):
        assert len(expected) == len(actual), f'{path}: length differs'
        for index, (left, right) in enumerate(zip(expected, actual)):
            assert_exact(left, right, f'{path}[{index}]')
    else:
        assert expected == actual, f'{path}: {expected!r} != {actual!r}'


def load_fixture():
    fixture = json.loads(FIXTURE.read_text())
    assert fixture['schema'] == 'so101-retained-child-evidence-v1'
    assert 1 <= len(fixture['cases']) <= 32
    assert len({c['id'] for c in fixture['cases']}) == len(fixture['cases'])
    return fixture


def decode_input(case):
    value = case['input']
    parent = unpack_array(value['original_parent'])
    assert parent.dtype == np.bool_
    symbols = value['depth_symbols_u32']
    bits = []
    for count, row in value['depth_symbol_row_runs']:
        assert type(count) is int and 1 <= count <= parent.shape[0]
        assert len(row) == parent.shape[1]
        assert len(bits)+count <= parent.shape[0]
        bits.extend([[symbols[s] for s in row]]*count)
    assert len(bits) == parent.shape[0]
    assert all(type(v) is int and 0 <= v <= 0xffffffff for row in bits for v in row)
    depth = np.array(bits, dtype='<u4').view('<f4')
    origin = value['pixel_origin_xy']
    assert len(origin) == 2 and all(type(v) is int and v >= 0 for v in origin)
    initial = value['initial_input_crop_xywh']
    assert_exact([*origin, parent.shape[1], parent.shape[0]], initial, 'initial input crop')
    minimum = value['min_pixels']
    assert type(minimum) is int and minimum >= 8
    if value['caller_parent_min_pixels'] is not None:
        assert minimum == max(24, value['caller_parent_min_pixels'])
    return parent, depth


def trace_case(case):
    """Trace independent candidate predicates, actual CCL and actual splitter."""
    value = case['input']
    original, original_depth = decode_input(case)
    x0, y0, width, height = map(int, cv2.boundingRect(original.astype(np.uint8)))
    assert width and height, 'fixture requires nonempty original parent'
    parent = original[y0:y0+height, x0:x0+width]
    depth = original_depth[y0:y0+height, x0:x0+width]
    ox, oy = value['pixel_origin_xy']
    valid = np.isfinite(depth) & (depth >= .10) & (depth <= value['max_observation_depth_m'])
    before_cut = parent & valid
    cut = np.zeros_like(parent)
    # Scalar edge walk, independently expressed from the splitter's vector slices.
    for y, x in np.argwhere(before_cut):
        for yy, xx in ((y+1, x), (y, x+1)):
            if yy < height and xx < width and before_cut[yy, xx]:
                delta = np.abs(depth[y, x]-depth[yy, xx])
                if delta > np.float32(value['max_neighbor_jump_m']):
                    cut[y, x] = cut[yy, xx] = True
    candidate = before_cut & ~cut
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate.astype(np.uint8), connectivity=8)
    assert labels.dtype == np.int32 and stats.dtype == np.int32
    components = []
    for label in range(1, count):
        ys, xs = np.where(labels == label)
        flat = int((ys*width+xs).min())
        components.append({
            'label_id': label, 'area': int(stats[label, cv2.CC_STAT_AREA]),
            'bounds_xywh': [int(v) for v in stats[label, :4]],
            'min_crop_block_key': int(((ys//2)*((width+1)//2)+xs//2).min()),
            'min_flat_pixel': flat,
            'minpixel_transformed_key': (flat//width//2)*((width+1)//2)+(flat % width)//2,
            # Diagnostic wrong-domain witness only, NEVER used for ordering.
            'min_global_block_key': int((((ys+oy+y0)//2)*((ox+x0+width+1)//2)+(xs+ox+x0)//2).min()),
        })
    pieces = split_depth_coherent_components(
        original, original_depth, min_pixels=value['min_pixels'],
        max_neighbor_jump_m=value['max_neighbor_jump_m'],
        max_observation_depth_m=value['max_observation_depth_m'],
        local_output=True, pixel_origin=(ox, oy))
    accepted = []
    for piece in pieces:
        assert piece.mask.dtype == np.bool_ and piece.support_mask.dtype == np.bool_
        assert not piece.mask.flags.writeable and not piece.support_mask.flags.writeable
        left = piece.origin_xy[0]-ox-x0
        top = piece.origin_xy[1]-oy-y0
        rh, rw = piece.mask.shape
        assert 0 <= left < width and 0 <= top < height
        assert left+rw <= width and top+rh <= height
        local_labels = labels[top:top+rh, left:left+rw]
        matching = np.unique(local_labels[piece.mask])
        assert len(matching) == 1 and matching[0] > 0
        label = int(matching[0])
        assert np.array_equal(piece.mask, local_labels == label)
        assert int(piece.mask.sum()) == components[label-1]['area']
        accepted.append({
            'label_id': label, 'origin_xy': list(piece.origin_xy),
            'local_bounds_xywh': [left, top, rw, rh],
            'region': pack_array(piece.mask), 'support': pack_array(piece.support_mask),
            'depth_cut_pixels': piece.depth_cut_pixels, 'parent_pixels': piece.parent_pixels,
        })
    return {
        'tight_bbox_in_input_xywh': [x0, y0, width, height],
        'tight_global_bbox_xywh': [ox+x0, oy+y0, width, height],
        'effective_min_pixels': value['min_pixels'], 'parent_pixels': int(parent.sum()),
        'parent': pack_array(parent), 'depth_bits': pack_array(depth.view('<u4')),
        'valid': pack_array(valid), 'candidate_before_cut': pack_array(before_cut),
        'cut': pack_array(cut), 'candidate': pack_array(candidate),
        'labels': pack_array(labels), 'stats': pack_array(stats),
        'components': components, 'accepted': accepted,
    }


def runtime_identity():
    module = Path(cv2.__file__).resolve()
    # Conda keeps the extension in python-3/, wheels put it beside __init__.py.
    # Identify the loaded native module, rather than guessing the package layout.
    native_module = getattr(cv2, '_native', cv2)
    binary = Path(native_module.__file__).resolve()
    assert binary.suffix in ('.so', '.pyd'), f'cannot identify loaded cv2 binary: {binary}'
    build = cv2.getBuildInformation()
    backend = next((line.split(':', 1)[1].strip() for line in build.splitlines()
                    if 'Parallel framework:' in line), 'unknown')
    return {'opencv_version': cv2.__version__, 'numpy_version': np.__version__,
            'python_version': sys.version, 'module_path': str(module),
            'binary_path': str(binary), 'binary_sha256': digest(binary),
            'build_information_sha256': hashlib.sha256(build.encode()).hexdigest(),
            'parallel_backend': backend}


def validate(fixture=None):
    """Compare fixed literals under default/1/2/4, restoring process thread state."""
    fixture = load_fixture() if fixture is None else fixture
    saved_threads = cv2.getNumThreads()
    profiles = []
    try:
        runtime = runtime_identity()
        for requested in ('default', 1, 2, 4):
            if requested != 'default':
                cv2.setNumThreads(requested)
            configured = cv2.getNumThreads()
            for case in fixture['cases']:
                assert_exact(case['expected'], trace_case(case), case['id'])
            tall = next(c for c in fixture['cases'] if c['id'] == 'tall-1025x7')
            height = tall['expected']['labels']['shape'][0]
            profiles.append({
                'requested_threads': requested, 'configured_threads': configured,
                'cases': len(fixture['cases']),
                'parallel_dispatch_eligible_tall': runtime['parallel_backend'] not in ('none', 'unknown')
                    and configured > 1 and height//configured >= 2,
                'actual_worker_concurrency_proven': False,
            })
    finally:
        cv2.setNumThreads(saved_threads)
    return {'schema': fixture['schema'], 'scope': 'CPU retained child evidence only',
            'gpu_executed': False, 'runtime': runtime, 'profiles': profiles,
            'fixture_sha256': digest(FIXTURE),
            'sources_sha256': {name: digest(ROOT/name) for name in SOURCES},
            'opencv_threads_restored': cv2.getNumThreads() == saved_threads}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs='?', choices=['validate'], default='validate')
    parser.parse_args()
    print(json.dumps(validate(), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
