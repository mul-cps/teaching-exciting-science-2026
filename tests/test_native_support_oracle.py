"""Offline executable support trace; no native/device dependency."""
import importlib
from pathlib import Path

import numpy as np


def api():
    from tools import native_support_oracle
    return native_support_oracle


def test_trace_api_exists():
    assert importlib.util.find_spec('tools.native_support_oracle') is not None
    assert callable(api().trace_support_case)
    assert callable(api().support_cases)
    assert callable(api().write_snapshot)


def test_exact_rng_streams_and_nine_real_fits():
    o = api()
    c = next(c for c in o.support_cases() if c['name'] == 'population-2001')
    t = o.trace_support_case(c)
    assert t['failure'] is None
    assert [r['seed'] for r in t['rng']] == [17, 17]
    assert [len(r['choices']) for r in t['rng']] == [1, 120]
    np.testing.assert_array_equal(t['selected_indices'], np.random.default_rng(17).choice(2001, 2000, replace=False))
    r = np.random.default_rng(17)
    for observed in t['rng'][1]['choices']:
        np.testing.assert_array_equal(observed, r.choice(np.arange(2000), 3, replace=False))
    assert len(t['fits']) == 9
    assert len(t['proposals']) == 120
    winner = t['winner']
    assert winner == next(i for i, p in enumerate(t['proposals']) if p.get('score') == max(p.get('score', -1) for p in t['proposals']))
    for fit in t['fits']:
        np.testing.assert_array_equal(fit['points'], t['winning_points'])
        assert fit['singular'].shape == (3,)
        np.testing.assert_allclose(fit['center'], np.average(fit['points'], axis=0, weights=fit['weights']), rtol=0, atol=1e-9)
        np.testing.assert_allclose(fit['svd_input'], (fit['points']-fit['center'])*np.sqrt(fit['weights'][:, None]), rtol=0, atol=1e-9)
    for before, after in zip(t['fits'], t['fits'][1:]):
        residual = np.abs(t['winning_points'] @ before['normal']+before['d'])
        np.testing.assert_allclose(after['weights'], np.minimum(1., .003/np.maximum(residual, 1e-12)), rtol=0, atol=1e-9)


def test_snap_preserves_diagnostics_and_failure_precedence():
    o = api(); cases = {c['name']: c for c in o.support_cases()}
    t = o.trace_support_case(cases['snap-offset-inside'])
    assert t['result']['d'] == 0
    assert abs(t['pre_orientation']['d']) > .01
    np.testing.assert_array_equal(t['pre_orientation']['residuals_m'], t['result']['residuals_m'])
    assert o.trace_support_case(cases['population-29'])['failure']['type'] == 'SupportPlaneUnavailable'
    assert o.trace_support_case(cases['no-proposal'])['failure'] == {'type': 'ValueError', 'message': 'No nondegenerate support plane'}
    t = o.trace_support_case(cases['non-tabletop-low-dominance'])
    assert t['failure']['type'] == 'ValueError'
    assert 'not tabletop-like' in t['failure']['message']


def test_instrumentation_return_exception_transparency_and_restoration():
    o = api()
    import sys
    before = o.instrumented_attributes()
    for c in o.support_cases():
        plain = o.call_support_case(c)
        traced = o.trace_support_case(c)
        o.assert_semantic_equal(plain, {k: traced[k] for k in ('result', 'failure')})
        assert o.instrumented_attributes() == before
    old_trace = sys.gettrace()
    def sentinel(frame, event, arg):
        return sentinel
    sys.settrace(sentinel)
    try:
        o.trace_support_case({'name': 'bad-input', 'kind': 'points', 'points': np.full((30, 3), np.nan)})
        assert sys.gettrace() is sentinel
        assert o.instrumented_attributes() == before
    finally:
        sys.settrace(old_trace)


def test_portable_reference_and_lossless_snapshot(tmp_path):
    import json
    o = api()
    reference = json.loads((Path(__file__).parent/'fixtures/native_geometry/support-v1.json').read_text())
    assert reference['sources'] == o.source_hashes()
    actual = [o.compact_trace(o.trace_support_case(c)) for c in o.support_cases()]
    o.assert_semantic_equal(reference['cases'], actual)
    cases = o.support_cases()[:2]
    manifest = o.write_snapshot(tmp_path/'snapshot', cases)
    assert manifest['state'] == 'complete'
    assert manifest['sources'] == o.source_hashes()
    assert manifest['arrays']
    for name, meta in manifest['arrays'].items():
        import hashlib
        path = tmp_path/'snapshot'/name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == meta['sha256']
        a = np.load(path, allow_pickle=False)
        assert str(a.dtype) == meta['dtype'] and list(a.shape) == meta['shape']
    def restore(value):
        if isinstance(value, dict):
            if set(value) == {'array'}:
                return np.load(tmp_path/'snapshot'/value['array'], allow_pickle=False)
            return {k: restore(v) for k, v in value.items()}
        if isinstance(value, list):
            return [restore(v) for v in value]
        return value
    for case, saved in zip(cases, manifest['cases']):
        o.assert_semantic_equal(case, restore(saved['input']))
        o.assert_semantic_equal(o.trace_support_case(case), restore(saved['trace']))
    import pytest
    with pytest.raises(FileExistsError):
        o.write_snapshot(tmp_path/'snapshot', cases)


def test_actual_boundary_outcomes_and_input_identity():
    o = api(); cases = {c['name']: c for c in o.support_cases()}
    assert sum(c['kind'] == 'image' for c in cases.values()) == 12
    for name, expected in [('dominance-29-100', 29), ('dominance-30-100', 30), ('dominance-35-100', 35), ('dominance-35-101', 35)]:
        t = o.trace_support_case(cases[name])
        assert t['achieved']['inliers'] == expected
        assert (t['failure'] is None) == (name == 'dominance-35-100')
    low = o.trace_support_case(cases['near-rank-below'])
    assert low['failure'] == {'type': 'ValueError', 'message': 'Degenerate plane points'}
    assert low['rank_margins'][0] < 0
    assert o.trace_support_case(cases['near-rank-above'])['rank_margins'][0] > 0
    assert o.trace_support_case(cases['snap-offset-clear-outside'])['achieved']['snap_offset_margin'] < 0
    for c in cases.values():
        before = o._copy(c)
        o.trace_support_case(c)
        o.assert_semantic_equal(before, c)


def test_svd_exception_restores_hooks(monkeypatch):
    o = api()
    def fail(*args, **kwargs):
        raise np.linalg.LinAlgError('injected SVD failure')
    monkeypatch.setattr(np.linalg, 'svd', fail)
    before = o.instrumented_attributes()
    c = o.support_cases()[0]
    expected = o.call_support_case(c)
    observed = o.trace_support_case(c)
    assert observed['failure'] == expected['failure'] == {'type': 'LinAlgError', 'message': 'injected SVD failure'}
    assert o.instrumented_attributes() == before


def test_noisy_winning_subset_really_receives_huber_weights():
    o = api()
    case = next(c for c in o.support_cases() if c['name'] == 'outliers')
    t = o.trace_support_case(case)
    assert len(t['fits']) == 9
    assert any(np.any(f['weights'] < 1) for f in t['fits'][1:])
    for before, after in zip(t['fits'], t['fits'][1:]):
        residual = np.abs(t['winning_points'] @ before['normal']+before['d'])
        np.testing.assert_allclose(after['weights'], np.minimum(1., .003/np.maximum(residual, 1e-12)), rtol=0, atol=1e-9)


def test_angular_snap_rejection_with_offset_inside():
    o = api()
    cases = {c['name']: c for c in o.support_cases()}
    assert 'snap-angle-clear-below' in cases
    t = o.trace_support_case(cases['snap-angle-clear-below'])
    assert t['failure'] is None
    assert t['achieved']['tabletop_margin'] >= 0
    assert t['achieved']['snap_angle_margin'] < 0
    assert t['achieved']['snap_offset_margin'] >= 0
    assert t['achieved']['snapped'] is False
    assert t['result']['plane_source'] == 'measured_table_plane'


import pytest


@pytest.mark.parametrize('expected,actual', [
    (0., 0), (0., False),
    ({'plane': {'d': 0.}}, {'plane': {'d': 0}}),
    ({'plane': {'d': 0.}}, {'plane': {'d': False}}),
    ({'plane': {'normal': [0., 0., 1.]}}, {'plane': {'normal': [0., 0, 1.]}}),
    ({'plane': {'normal': [0., 0., 1.]}}, {'plane': {'normal': [0., False, 1.]}}),
])
def test_semantic_scalar_type_guard(expected, actual):
    with pytest.raises(AssertionError):
        api().assert_semantic_equal(expected, actual)


def test_semantic_float_tolerance_is_absolute_and_preserved():
    o = api()
    o.assert_semantic_equal({'d': np.float64(0.)}, {'d': 5e-10})
    with pytest.raises(AssertionError):
        o.assert_semantic_equal({'d': 0.}, {'d': 2e-9})
    with pytest.raises(AssertionError):
        o.assert_semantic_equal({'d': 1e9}, {'d': 1e9+.01})
