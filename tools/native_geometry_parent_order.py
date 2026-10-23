"""Validate frozen original-color GRANA parents before any resident port.

This CPU reference deliberately calls CCL on the full color mask, never on a
valid-depth mask or an area-filtered population. It writes no goldens and is not
imported by the production callback. GRANA birth order is implementation-specific;
keep the frozen witnesses when upgrading OpenCV rather than regenerating them.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform

import cv2
import numpy as np

SCHEMA = 'so101-original-color-parent-order-v1'
ALGORITHM = {'connectivity': 8, 'label_type': 'CV_32S', 'ccl': 'CCL_GRANA'}
DEFAULT_FIXTURE = Path(__file__).resolve().parents[1]/'tests/fixtures/native_geometry/parent-order-v1.json'


def mask_from_rows(rows):
    """Read the small, public literal binary masks used by this fixture only."""
    if (not isinstance(rows, list) or not rows or not isinstance(rows[0], str)
            or not rows[0] or any(not isinstance(row, str) or len(row) != len(rows[0])
                                 or set(row) - {'0', '1'} for row in rows)):
        raise ValueError('expected nonempty rectangular binary mask rows')
    return np.array([[int(pixel) for pixel in row] for row in rows], dtype=np.uint8)


def trace_parent_mask(mask, min_component_pixels=30):
    """Trace all foreground IDs first; preserve those IDs across the area gate."""
    mask = np.asarray(mask)
    if (mask.ndim != 2 or not mask.size or not np.all((mask == 0) | (mask == 1))):
        raise ValueError('expected nonempty binary color mask')
    if (isinstance(min_component_pixels, bool) or not isinstance(min_component_pixels, int)
            or min_component_pixels < 1):
        raise ValueError('minimum component pixels must be a positive integer')
    mask = np.ascontiguousarray(mask, dtype=np.uint8)
    # Match objects.analyze_rgbd: full original color domain, explicit algorithm.
    count, labels, stats, _ = cv2.connectedComponentsWithStatsWithAlgorithm(mask, 8, cv2.CV_32S, cv2.CCL_GRANA)
    width = mask.shape[1]
    block_width = (width+1)//2
    components = []
    for label in range(1, count):
        yy, xx = np.nonzero(labels == label)
        components.append({
            'label': label,
            'area': int(stats[label, cv2.CC_STAT_AREA]),
            'bounds_xywh': [int(stats[label, field]) for field in (
                cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT)],
            'min_flat_pixel': int(np.min(yy*width+xx)),
            # Reduce over complete membership; transforming only minpixel fails.
            'min_block_key': int(np.min((yy//2)*block_width+xx//2)),
        })
    return {'labels': labels.tolist(), 'components': components,
            'kept_label_ids': [c['label'] for c in components
                               if c['area'] >= min_component_pixels]}


def trace_case(case):
    """Depth support is separate evidence and never changes a color parent."""
    mask = mask_from_rows(case['mask_rows'])
    trace = trace_parent_mask(mask, case['min_component_pixels'])
    if 'valid_depth_rows' in case:
        valid = mask_from_rows(case['valid_depth_rows'])
        if valid.shape != mask.shape:
            raise ValueError('valid-depth mask must match color dimensions')
        labels = np.asarray(trace['labels'], dtype=np.int32)
        trace['valid_depth_pixels_by_label'] = [
            {'label': c['label'], 'pixels': int(np.count_nonzero((labels == c['label']) & (valid != 0)))}
            for c in trace['components']]
    return trace


def validate_fixture(fixture):
    """Compare every frozen array/stat/ID exactly; no native expected values."""
    if fixture.get('schema') != SCHEMA or fixture.get('algorithm') != ALGORITHM:
        raise ValueError('unsupported parent-order fixture schema or algorithm')
    cases = fixture.get('cases')
    if not isinstance(cases, list) or not cases:
        raise ValueError('fixture must contain parent cases')
    ids = []
    for case in cases:
        name = case['id']
        if not isinstance(name, str) or not name or name in ids:
            raise ValueError('parent case IDs must be nonempty and unique')
        actual = trace_case(case)
        # JSON identity preserves integer/boolean/float distinctions as well as
        # list order, dimensions, labels and exact statistic/filter values.
        if json.dumps(actual, sort_keys=True, allow_nan=False) != json.dumps(
                case['expected'], sort_keys=True, allow_nan=False):
            raise ValueError(f'parent trace differs for {name}')
        ids.append(name)
    return ids


def validate_thread_profiles(fixture, profiles=(None, 1, 2, 4)):
    """Exercise dispatch profiles and restore process-global OpenCV state."""
    original_threads = cv2.getNumThreads()
    records = []
    try:
        for threads in profiles:
            if threads is not None and (isinstance(threads, bool) or not isinstance(threads, int) or threads < 1):
                raise ValueError('thread profile must be default or a positive integer')
            cv2.setNumThreads(original_threads if threads is None else threads)
            ids = validate_fixture(fixture)
            records.append({'requested_threads': 'default' if threads is None else threads,
                            'actual_threads': cv2.getNumThreads(), 'cases': len(ids)})
    finally:
        cv2.setNumThreads(original_threads)
    return {'schema': SCHEMA, 'cpu_reference_validated': True,
            'resident_parent_qualified': False, 'goldens_regenerated': False,
            'python': platform.python_version(), 'numpy': np.__version__,
            'opencv': cv2.__version__, 'original_threads': original_threads,
            'profiles': records}


def thread_profile(value):
    if value == 'default':
        return None
    try:
        threads = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('expected default or positive integer') from error
    if threads < 1:
        raise argparse.ArgumentTypeError('expected default or positive integer')
    return threads


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument('--threads', nargs='+', type=thread_profile, default=[None, 1, 2, 4])
    args = parser.parse_args(argv)
    raw = args.fixture.read_bytes()
    report = validate_thread_profiles(json.loads(raw), profiles=args.threads)
    report['fixture_sha256'] = hashlib.sha256(raw).hexdigest()
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
