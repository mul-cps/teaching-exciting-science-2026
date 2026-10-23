"""Frozen retained child evidence; these tests never update the fixture."""
import copy
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('child_evidence', ROOT/'tools/native_geometry_child_evidence.py')
evidence = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evidence)


def case(name):
    return next(c for c in evidence.load_fixture()['cases'] if c['id'] == name)


def test_frozen_all_cases_and_thread_profiles():
    original = cv2.getNumThreads()
    report = evidence.validate()
    assert cv2.getNumThreads() == original
    assert [p['requested_threads'] for p in report['profiles']] == ['default', 1, 2, 4]
    assert all(p['cases'] == len(evidence.load_fixture()['cases']) for p in report['profiles'])
    assert report['gpu_executed'] is False
    assert report['runtime']['opencv_version'] == cv2.__version__
    assert len(report['runtime']['binary_sha256']) == 64
    assert len(report['runtime']['build_information_sha256']) == 64


def test_crop_local_keys_and_full_membership_counterexamples():
    c = case('u-membership')
    components = c['expected']['components']
    assert [x['min_crop_block_key'] for x in components] == [0, 1]
    assert [x['min_flat_pixel'] for x in components] == [6, 3]
    assert [x['minpixel_transformed_key'] for x in components] == [3, 1]
    assert [x['label_id'] for x in c['expected']['accepted']] == [1]
    for x, y in [(0, 0), (0, 1), (1, 0), (1, 1)]:
        p = case(f'origin-parity-{x}-{y}')
        assert p['input']['pixel_origin_xy'] == [x, y]
        assert [v['min_crop_block_key'] for v in p['expected']['components']] == [0, 1]
    odd = case('origin-parity-0-1')['expected']['components']
    assert [x['min_global_block_key'] for x in odd] == [2, 1]


def test_filter_rank_gaps_and_explicit_parent_minimum():
    for name, minimum in [('tiny-before-kept-8', 8), ('tiny-before-kept-24', 24),
                          ('configured-parent-min-8', 24), ('configured-parent-min-32', 32)]:
        c = case(name)
        assert c['expected']['effective_min_pixels'] == minimum
        assert c['expected']['components'][0]['area'] == 1
        assert [v['label_id'] for v in c['expected']['accepted']] == [2]


def test_hole_contour_and_halo_are_literal_retained_evidence():
    c = case('invalid-interior-hole')
    piece = c['expected']['accepted'][0]
    region = evidence.unpack_array(piece['region'])
    support = evidence.unpack_array(piece['support'])
    assert region.shape == (13, 13)
    assert not region[6, 6] and support[6, 6]
    assert not region[4:9, 4:9].any()  # center is beyond radius two
    boundary = case('boundary-and-clipped-halo')['expected']['accepted']
    assert boundary[0]['origin_xy'] == [11, 16]
    assert boundary[0]['local_bounds_xywh'] == [0, 0, 7, 9]


def test_depth_edges_and_both_endpoint_cut_are_frozen():
    c = case('depth-endpoints')
    valid = evidence.unpack_array(c['expected']['valid'])
    assert valid[0].tolist() == [True, True, False, False, False, False, False, False]
    all_cut = case('all-cut-empty')
    assert evidence.unpack_array(all_cut['expected']['cut']).all()
    assert not all_cut['expected']['accepted']
    edge = case('strict-jump-f32-edges')
    cut = evidence.unpack_array(edge['expected']['cut'])
    assert cut[0, :2].tolist() == [False, False]
    assert cut[2, :2].tolist() == [True, True]
    assert cut[4, :2].tolist() == [True, True]
    equality = evidence.unpack_array(case('configured-strict-equality')['expected']['cut'])
    assert equality[0].tolist() == [False, False]
    assert equality[2].tolist() == [True, True]
    accepted = case('cut-nearby-halo')['expected']['accepted']
    assert [p['depth_cut_pixels'] for p in accepted] == [14, 14]
    assert accepted[0]['local_bounds_xywh'] == [0, 0, 9, 7]
    assert accepted[1]['local_bounds_xywh'] == [7, 0, 10, 7]


def test_configured_limits_change_acceptance():
    assert case('configured-min-reject24')['expected']['components'][0]['area'] == 24
    assert not case('configured-min-reject24')['expected']['accepted']
    limited = case('configured-max-depth')['expected']
    assert evidence.unpack_array(limited['valid'])[0].tolist() == [True]*8+[False]*9
    assert [p['label_id'] for p in limited['accepted']] == [1]
    assert len(case('depth-inclusive-kept')['expected']['accepted']) == 2


@pytest.mark.parametrize('mutation', ['global_key', 'minpixel', 'order', 'filter_rank',
                                    'one_endpoint', 'invalid_hole', 'halo', 'origin',
                                    'depth_bits', 'bool_type'])
def test_deepcopy_expected_mutations_are_rejected(mutation):
    fixture = copy.deepcopy(evidence.load_fixture())
    def pick(name):
        return next(c for c in fixture['cases'] if c['id'] == name)['expected']
    if mutation == 'global_key':
        e = pick('origin-parity-0-1')
        e['components'][0]['min_crop_block_key'] = e['components'][0]['min_global_block_key']
    elif mutation == 'minpixel':
        e = pick('u-membership')
        e['components'][0]['min_crop_block_key'] = e['components'][0]['minpixel_transformed_key']
    elif mutation == 'order':
        pick('two-kept')['accepted'].reverse()
    elif mutation == 'filter_rank':
        pick('tiny-before-kept-24')['accepted'][0]['label_id'] = 1
    elif mutation == 'one_endpoint':
        a = pick('all-cut-empty')['cut']['row_runs'][0][1]
        a[0] = False
    elif mutation == 'invalid_hole':
        a = pick('invalid-interior-hole')['accepted'][0]['support']
        values = evidence.unpack_array(a).copy(); values[6, 6] = False
        a.update(evidence.pack_array(values))
    elif mutation == 'halo':
        pick('boundary-and-clipped-halo')['accepted'][0]['local_bounds_xywh'][2] -= 1
    elif mutation == 'origin':
        pick('boundary-and-clipped-halo')['accepted'][0]['origin_xy'][0] += 1
    elif mutation == 'depth_bits':
        pick('strict-jump-f32-edges')['depth_bits']['row_runs'][0][1][0] ^= 1
    else:
        pick('tiny-before-kept-24')['parent_pixels'] = True
    with pytest.raises(AssertionError):
        evidence.validate(fixture)


def test_threads_restored_on_failure(monkeypatch):
    original = cv2.getNumThreads()
    real_trace = evidence.trace_case
    calls = 0
    case_count = len(evidence.load_fixture()['cases'])
    def fail(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 2*case_count+1:
            assert cv2.getNumThreads() == 2
            raise RuntimeError('deliberate splitter failure')
        return real_trace(*args, **kwargs)
    monkeypatch.setattr(evidence, 'trace_case', fail)
    with pytest.raises(RuntimeError, match='deliberate'):
        evidence.validate()
    assert cv2.getNumThreads() == original


def test_exact_comparator_does_not_coerce_discrete_types():
    for expected, actual in [(True, 1), (1, 1.0), ([1, 2], [2, 1])]:
        with pytest.raises(AssertionError):
            evidence.assert_exact(expected, actual)


def test_tall_fixture_crosses_configured_dispatch_thresholds():
    c = case('tall-1025x7')
    h = c['expected']['labels']['shape'][0]
    assert h == 1025
    assert all(h//threads >= 2 for threads in (1, 2, 4))
    # The report records eligibility only; thread settings do not prove concurrency.
    report = evidence.validate()
    for profile in report['profiles']:
        assert 'parallel_dispatch_eligible_tall' in profile
        assert profile['actual_worker_concurrency_proven'] is False
