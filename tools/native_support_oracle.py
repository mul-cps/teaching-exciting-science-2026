"""Offline actual retained CPU support trace; never a native golden generator.

Instrumentation is process-global and deliberately single-threaded. Point cases
inject only deprojection inputs; image cases execute the ordinary analyze_rgbd
entry. All fitting, proposal scoring and admission math is retained source.
"""
import argparse
from contextlib import contextmanager
import hashlib
import inspect
import importlib.metadata
import json
from pathlib import Path
import platform
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import native_geometry_oracle as old
from tools import native_geometry_matrix as matrix
from workshop.digital_twin import grasp_geometry as geometry
objects = old.objects


def _copy(value):
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, dict):
        return {k: _copy(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_copy(v) for v in value]
    return value


def support_cases():
    """Deterministic inputs; boundary claims are recorded from achieved traces."""
    result = []
    def plane(count=100, nz=1., d=0., noise=0.):
        i = np.arange(count)
        xy = np.column_stack(((i % 17 - 8)*.03, (i // 17 - 3)*.03))
        n = np.array([np.sqrt(1-nz*nz), 0., nz])
        z = (-d-xy[:, 0]*n[0])/nz + noise*np.sin(i*1.7)
        return np.column_stack((xy, z))
    def add(name, points, **metadata):
        result.append(dict(name=name, kind='points', points=points, intent=metadata))
    add('flat-equal-score', plane())
    add('oblique', plane(nz=.94, d=-.04))
    add('noisy', plane(nz=.98, d=-.03, noise=.001))
    p = plane(150, nz=.97, d=-.03, noise=.0005); p[100:, 2] += .03*np.cos(np.arange(50))
    add('outliers', p)
    for count in (29, 30, 2000, 2001, 4800):
        add(f'population-{count}', plane(count), requested_population=count)
    add('no-proposal', np.column_stack((np.arange(30), np.zeros((30, 2)))))
    # Cross products survive 1e-12 but the second centered singular is <1e-9.
    p = plane(30); p[:, 0] *= 100; p[:, 1] *= 1e-9
    add('near-rank-below', p)
    p = plane(30); p[:, 0] *= 100; p[:, 1] *= 1e-7
    add('near-rank-above', p)
    for label, nz in [('below', np.nextafter(.85, 0)), ('at', .85), ('above', np.nextafter(.85, 1))]:
        add('tabletop-'+label, plane(nz=nz, d=-.04), requested_nz=float(nz))
    cosine = np.cos(np.deg2rad(5.))
    for label, nz in [('below', np.nextafter(cosine, 0)), ('at', cosine), ('above', np.nextafter(cosine, 1))]:
        add('snap-angle-'+label, plane(nz=nz), requested_nz=float(nz))
    for label, d in [('inside', .014), ('at', .015), ('outside', np.nextafter(.015, 1))]:
        add('snap-offset-'+label, plane(d=d), requested_abs_d=float(d))
    add('snap-offset-clear-outside', plane(d=.015001), requested_abs_d=.015001)
    for label, distance in [('below', np.nextafter(.003, 0)), ('at', .003), ('above', np.nextafter(.003, 1))]:
        p = plane(); p[-1, 2] = distance
        add('threshold-'+label, p, requested_residual=float(distance))
    # Fixed nonplanar background; achieved final inliers/fraction are explicit.
    background = np.random.default_rng(73).uniform(-1, 1, (100, 3))
    background[:, 2] += 3
    for count, total in ((29, 100), (30, 100), (35, 100), (35, 101), (36, 100)):
        p = background[:total-count].copy()
        if len(p) < total-count:
            raise AssertionError('fixture population')
        add(f'dominance-{count}-{total}', np.vstack((plane(count, d=-.04), p)), requested_inliers=count, requested_fraction=count/total)
    add('non-tabletop-low-dominance', np.vstack((plane(29, nz=.7, d=-.04), background[:71])))
    add('invalid-nonfinite', np.full((30, 3), np.nan))
    for name, arguments in matrix.cases():
        result.append(dict(name='image-'+name, kind='image', arguments=arguments))
    # Preserve the original 44-case order; independently reject the angle conjunct.
    add('snap-angle-clear-below', plane(nz=.99), requested_nz=.99)
    return result


@contextmanager
def _point_injection(case):
    original = objects.camera_points
    try:
        if case['kind'] == 'points':
            objects.camera_points = lambda *args, **kwargs: case['points'].copy()
        yield
    finally:
        objects.camera_points = original


def _invoke(case):
    with _point_injection(case):
        if case['kind'] == 'image':
            return old.analyze_reference(case['arguments'])
        count = len(case['points'])
        return objects._fit_world_support(
            np.ones((1, count)), np.ones((1, count), dtype=bool),
            np.full((1, count, 3), 128, dtype=np.uint8),
            dict(fx=1., fy=1., ppx=0., ppy=0.), np.zeros(5), np.eye(4),
            [-np.inf, np.inf]*3, stride=1)


def call_support_case(case):
    """Ordinary retained return or exact ordinary exception, without tracing."""
    try:
        return dict(result=_invoke(case), failure=None)
    except Exception as error:
        return dict(result=None, failure=dict(type=type(error).__name__, message=str(error)))


def instrumented_attributes():
    return (np.random.default_rng, np.linalg.svd, geometry._weighted_plane,
            objects.fit_support_plane, objects._orient_plane_up, objects.camera_points)


def trace_support_case(case):
    """Observe actual RNG/SVD/functions; restore every hook even on exceptions."""
    trace = dict(name=case['name'], kind=case['kind'], intent=case.get('intent', {}),
                 rng=[], proposals=[], fits=[], winner=None)
    original_rng, original_svd, original_weighted, original_fit, original_orient, _ = instrumented_attributes()
    previous_trace = sys.gettrace()
    # Source line hooks observe locals after the actual statements execute.
    source, start = inspect.getsourcelines(geometry.fit_support_plane)
    norm_line = start + next(i for i, s in enumerate(source) if 'if norm < 1e-12:' in s)
    score_line = start + next(i for i, s in enumerate(source) if 'if score > best_score:' in s)
    world_source, world_start = inspect.getsourcelines(objects._fit_world_support)
    population_line = world_start + next(i for i, s in enumerate(world_source) if 'if len(board) > max_points:' in s)
    active_fit = []
    proposal_geometry = {}
    class ObservedRng:
        def __init__(self, generator, record):
            self.generator, self.record = generator, record
        def choice(self, *args, **kwargs):
            value = self.generator.choice(*args, **kwargs)
            self.record['choices'].append(value.copy())
            self.record['calls'].append(dict(population=_copy(args[0]), size=args[1] if len(args)>1 else kwargs.get('size'), replace=kwargs.get('replace', True)))
            return value
    def rng(seed=None, *args, **kwargs):
        record = dict(seed=seed, choices=[], calls=[])
        trace['rng'].append(record)
        return ObservedRng(original_rng(seed, *args, **kwargs), record)
    def svd(*args, **kwargs):
        value = original_svd(*args, **kwargs)
        if active_fit:
            active_fit[-1]['svd_input'] = args[0].copy()
            active_fit[-1]['singular'] = value[1].copy()
            active_fit[-1]['vt'] = value[2].copy()
        return value
    def weighted(points, weights):
        record = dict(points=points.copy(), weights=weights.copy())
        trace['fits'].append(record); active_fit.append(record)
        try:
            n, d = original_weighted(points, weights)
            record.update(normal=n.copy(), d=d)
            return n, d
        finally:
            active_fit.pop()
    def fit(points, *args, **kwargs):
        trace['selected_points'] = points.copy()
        if 'selected_indices' not in trace:
            selection = trace['rng'][0]['choices'][0] if trace['rng'] else np.arange(len(points))
            trace['selected_indices'] = selection.copy()
        value = original_fit(points, *args, **kwargs)
        trace['pre_orientation'] = _copy(value)
        return value
    def orient(plane):
        value = original_orient(plane)
        trace['post_orientation'] = _copy(value)
        return value
    def observe(frame, event, arg):
        if event == 'line':
            if frame.f_code is geometry.fit_support_plane.__code__ and frame.f_lineno == norm_line:
                l = frame.f_locals
                proposal_geometry[l['_']] = dict(sample=l['sample'].copy(), cross=l['n'].copy(), norm=float(l['norm']))
            elif frame.f_code is geometry.fit_support_plane.__code__ and frame.f_lineno == score_line:
                l = frame.f_locals
                trace['proposals'].append(dict(iteration=l['_'], normal=l['n'].copy(), mask=l['mask'].copy(), score=float(l['score'])))
                if l['score'] > l['best_score']:
                    trace['winner'] = l['_']
            elif frame.f_code is objects._fit_world_support.__code__ and frame.f_lineno == population_line:
                trace['population_points'] = frame.f_locals['board'].copy()
                trace['sampled_pixels'] = frame.f_locals['pixels'].copy()
                trace['camera_points'] = frame.f_locals['camera'].copy()
            elif frame.f_code is original_weighted.__code__ and 'center' in frame.f_locals and active_fit:
                active_fit[-1]['center'] = frame.f_locals['center'].copy()
        if event == 'return' and frame.f_code is geometry.fit_support_plane.__code__:
            if 'q' in frame.f_locals:
                trace['winning_points'] = frame.f_locals['q'].copy()
                trace['winning_mask'] = frame.f_locals['best'].copy()
        return observe
    try:
        np.random.default_rng = rng; np.linalg.svd = svd
        geometry._weighted_plane = weighted; objects.fit_support_plane = fit
        objects._orient_plane_up = orient
        sys.settrace(observe)
        trace.update(call_support_case(case))
    finally:
        sys.settrace(previous_trace)
        np.random.default_rng = original_rng; np.linalg.svd = original_svd
        geometry._weighted_plane = original_weighted; objects.fit_support_plane = original_fit
        objects._orient_plane_up = original_orient
    if 'selected_indices' not in trace:
        trace['selected_indices'] = np.arange(min(len(trace.get('population_points', [])), 2000))
    # Include skipped proposals, preserving their actual iteration positions.
    observed = {p['iteration']: p for p in trace['proposals']}
    proposal_rng = next((r for r in reversed(trace['rng']) if len(r['choices']) == 120), None)
    if proposal_rng:
        trace['proposals'] = [dict(observed.get(i, dict(iteration=i, degenerate=True)), **proposal_geometry[i], triplet=c.copy()) for i, c in enumerate(proposal_rng['choices'])]
    pre = trace.get('pre_orientation')
    if pre is not None:
        nz = float(abs(pre['normal'][2])); count = int(pre['inlier_mask'].sum())
        trace['achieved'] = dict(inliers=count, fraction=count/len(pre['inlier_mask']),
            oriented_nz=nz, tabletop_margin=nz-.85, snap_angle_margin=nz-np.cos(np.deg2rad(5.)),
            snap_offset_margin=.015-abs(pre['d']),
            residual_threshold_margin=pre['residuals_m']-.003,
            snapped=trace.get('post_orientation', {}).get('plane_source') == 'measured_table_consistent_with_board_z0')
    trace['rank_margins'] = [float(f['singular'][1]-1e-9) for f in trace['fits'] if 'singular' in f]
    return trace


def source_hashes():
    paths = old.SOURCES + ['tools/native_geometry_matrix.py', 'tools/native_support_oracle.py', 'tests/test_native_support_oracle.py']
    return {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}


def compact_trace(trace):
    """Small portable semantics. Dense exact arrays live in the full snapshot."""
    pre = trace.get('pre_orientation'); post = trace.get('post_orientation')
    return old.plain_values(dict(name=trace['name'], kind=trace['kind'], failure=trace['failure'],
        population=len(trace.get('population_points', [])),
        achieved={k: v for k, v in trace.get('achieved', {}).items() if k != 'residual_threshold_margin'},
        rank_margins=trace['rank_margins'], selected=len(trace['selected_indices']),
        winner=trace['winner'],
        proposal_memberships=[None if 'mask' not in p else np.packbits(p['mask'], bitorder='little').tobytes().hex() for p in trace['proposals']],
        proposal_scores=[p.get('score') for p in trace['proposals']],
        winning_indices=np.flatnonzero(trace.get('winning_mask', [])),
        # Exact integer RNG order is compact enough to retain in the reference.
        rng=[dict(seed=r['seed'], choices=r['choices']) for r in trace['rng']],
        fits=[{k: f[k] for k in ('center', 'singular', 'normal', 'd') if k in f} for f in trace['fits']],
        pre_orientation=None if pre is None else {k: v for k, v in pre.items() if k not in ('inlier_mask', 'residuals_m')},
        inlier_indices=None if pre is None else np.flatnonzero(pre['inlier_mask']),
        post_orientation=None if post is None else {k: v for k, v in post.items() if k not in ('inlier_mask', 'residuals_m')}))


def assert_semantic_equal(expected, actual):
    """Discrete order exact; binary64 values abs 1e-9 (no floating hash gate)."""
    expected, actual = old.plain_values(expected), old.plain_values(actual)
    def same_types(left, right, path='report'):
        assert type(left) is type(right), (path, type(left).__name__, type(right).__name__)
        if isinstance(left, dict):
            assert left.keys() == right.keys(), path
            for key in left:
                same_types(left[key], right[key], path+'.'+str(key))
        elif isinstance(left, list):
            assert len(left) == len(right), path
            for index, (a, b) in enumerate(zip(left, right)):
                same_types(a, b, path+f'[{index}]')
    same_types(expected, actual)
    old.assert_report_equal(expected, actual, tolerance=1e-9)


def write_snapshot(directory, cases):
    """Claim new path, retain lossless arrays, atomically publish manifest."""
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=False)
    arrays = {}
    def external(value):
        if isinstance(value, np.ndarray):
            name = f'array-{len(arrays):06d}.npy'; path = directory/name
            np.save(path, value, allow_pickle=False)
            arrays[name] = dict(shape=list(value.shape), dtype=str(value.dtype), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            return {'array': name}
        if isinstance(value, dict):
            return {k: external(v) for k, v in value.items()}
        if isinstance(value, (tuple, list)):
            return [external(v) for v in value]
        return old.plain_values(value)
    manifest = dict(schema='so101-native-support-oracle-v1', state='building', sources=source_hashes(),
        runtime=dict(python=platform.python_version(), numpy=np.__version__, opencv=old.cv2.__version__, scipy=importlib.metadata.version('scipy')),
        numeric_absolute_tolerance=1e-9, local_ray_absolute_tolerance=1e-12,
        scope='offline retained CPU only; point injection separately labeled',
        reference_sha256=hashlib.sha256((ROOT/'tests/fixtures/native_geometry/support-v1.json').read_bytes()).hexdigest(),
        arrays=arrays, cases=[])
    try:
        for case in cases:
            manifest['cases'].append(dict(input=external(case), trace=external(trace_support_case(case))))
        manifest['state'] = 'complete'
    except Exception:
        (directory/'status.json').write_text(json.dumps(dict(state='failed'))+'\n')
        raise
    old_json = json.dumps(manifest, indent=2, allow_nan=False)
    (directory/'manifest.json.pending').write_text(old_json+'\n')
    (directory/'manifest.json.pending').rename(directory/'manifest.json')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    manifest = write_snapshot(args.output, support_cases())
    print(json.dumps(dict(state=manifest['state'], cases=len(manifest['cases']), arrays=len(manifest['arrays']))))


if __name__ == '__main__':
    main()
