"""Frozen original-color GRANA parent IDs must precede all area/depth filters."""
from copy import deepcopy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from tools import native_geometry_parent_order as parents

FIXTURE = Path(__file__).parent/'fixtures/native_geometry/parent-order-v1.json'


def fixture():
    return json.loads(FIXTURE.read_text())


def case(name):
    return next(c for c in fixture()['cases'] if c['id'] == name)


def test_literal_4x6_label_array_is_not_minpixel_order():
    value = case('minpixel-counterexample-4x6')
    actual = parents.trace_case(value)
    assert actual['labels'] == [[0, 0, 2, 0, 0, 0], [1, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
    assert actual['components'] == [
        {'label': 1, 'area': 1, 'bounds_xywh': [0, 1, 1, 1], 'min_flat_pixel': 6, 'min_block_key': 0},
        {'label': 2, 'area': 1, 'bounds_xywh': [2, 0, 1, 1], 'min_flat_pixel': 2, 'min_block_key': 1},
    ]
    assert actual['kept_label_ids'] == [1, 2]


def test_literal_u_membership_requires_full_component_block_reduction():
    value = case('transformed-minpixel-counterexample-u-4x8')
    actual = parents.trace_case(value)
    assert actual['labels'] == [[0, 0, 0, 2, 0, 0, 1, 0], [1, 0, 0, 0, 0, 0, 1, 0],
                                [1, 0, 0, 0, 0, 0, 1, 0], [1, 1, 1, 1, 1, 1, 1, 0]]
    assert actual['components'] == [
        {'label': 1, 'area': 12, 'bounds_xywh': [0, 0, 7, 4], 'min_flat_pixel': 6, 'min_block_key': 0},
        {'label': 2, 'area': 1, 'bounds_xywh': [3, 0, 1, 1], 'min_flat_pixel': 3, 'min_block_key': 1},
    ]
    transformed = [(c['min_flat_pixel']//8//2)*4 + (c['min_flat_pixel']%8//2)
                   for c in actual['components']]
    assert transformed == [3, 1]


def test_area_filter_keeps_label_two_without_relabeling():
    value = case('filter-keeps-original-label-12x16')
    actual = parents.trace_case(value)
    expected = [[0]*16 for _ in range(12)]
    for row in range(5):
        expected[row][4:12] = [2]*8
    expected[1][0] = 1
    assert actual['labels'] == expected
    assert actual['components'] == [
        {'label': 1, 'area': 1, 'bounds_xywh': [0, 1, 1, 1], 'min_flat_pixel': 16, 'min_block_key': 0},
        {'label': 2, 'area': 40, 'bounds_xywh': [4, 0, 8, 5], 'min_flat_pixel': 4, 'min_block_key': 2},
    ]
    assert actual['kept_label_ids'] == [2]


def test_invalid_depth_hole_does_not_change_parent_membership_or_area(monkeypatch):
    value = case('invalid-depth-hole-7x9')
    mask = parents.mask_from_rows(value['mask_rows'])
    valid = parents.mask_from_rows(value['valid_depth_rows'])
    original = cv2.connectedComponentsWithStatsWithAlgorithm
    calls = []
    def observed(supplied, connectivity, label_type, algorithm):
        calls.append((supplied.copy(), connectivity, label_type, algorithm))
        return original(supplied, connectivity, label_type, algorithm)
    monkeypatch.setattr(cv2, 'connectedComponentsWithStatsWithAlgorithm', observed)
    actual = parents.trace_case(value)
    assert len(calls) == 1
    np.testing.assert_array_equal(calls[0][0], mask)
    assert calls[0][1:] == (8, cv2.CV_32S, cv2.CCL_GRANA)
    assert actual['labels'] == [[0, 1, 1, 1, 1, 1, 1, 1, 0]]*5 + [[0]*9]*2
    assert actual['components'] == [
        {'label': 1, 'area': 35, 'bounds_xywh': [1, 0, 7, 5], 'min_flat_pixel': 1, 'min_block_key': 0},
    ]
    assert actual['kept_label_ids'] == [1]
    assert actual['valid_depth_pixels_by_label'] == [{'label': 1, 'pixels': 26}]
    assert actual['labels'][2][4] == 1 and valid[2, 4] == 0
    # A depth-masked CCL would instead have area 26 and fail min-area 30.
    count, _, stats, _ = original(mask & valid, 8, cv2.CV_32S, cv2.CCL_GRANA)
    assert count == 2 and int(stats[1, cv2.CC_STAT_AREA]) == 26


def test_complete_frozen_fixture_matches_actual_retained_cpu_call():
    value = fixture()
    assert parents.validate_fixture(value) == [c['id'] for c in value['cases']]
    for c in value['cases']:
        trace = parents.trace_case(c)
        assert trace == c['expected']  # every ordered array/stat/filter field
        keys = [component['min_block_key'] for component in trace['components']]
        assert keys == sorted(set(keys))


def test_default_one_two_four_threads_and_odd_shapes_restore_global_threads():
    previous = cv2.getNumThreads()
    report = parents.validate_thread_profiles(fixture())
    assert [p['requested_threads'] for p in report['profiles']] == ['default', 1, 2, 4]
    assert all(p['cases'] == 8 for p in report['profiles'])
    assert cv2.getNumThreads() == previous
    for name, shape in [('odd-partial-blocks-5x7', (5, 7)), ('odd-row-1x5', (1, 5)),
                        ('odd-column-5x1', (5, 1)), ('odd-stripe-65x17', (65, 17))]:
        trace = parents.trace_case(case(name))
        assert np.asarray(trace['labels']).shape == shape
        assert [c['label'] for c in trace['components']] == [1, 2, 3]


def test_thread_state_is_restored_when_fixture_validation_fails():
    previous = cv2.getNumThreads()
    value = fixture()
    value['cases'][0]['expected']['kept_label_ids'] = [2, 1]
    with pytest.raises(ValueError, match='minpixel-counterexample-4x6'):
        parents.validate_thread_profiles(value, profiles=(1, 2))
    assert cv2.getNumThreads() == previous


def relabel(trace, order):
    """An intentionally wrong compact-order candidate, never an expected oracle."""
    replacement = {old: new for new, old in enumerate(order, 1)}
    old = np.asarray(trace['labels'])
    trace['labels'] = np.asarray([[replacement.get(int(x), 0) for x in row] for row in old]).tolist()
    lookup = {c['label']: c for c in trace['components']}
    trace['components'] = [{**lookup[label], 'label': replacement[label]} for label in order]
    trace['kept_label_ids'] = sorted(replacement[label] for label in trace['kept_label_ids'])


@pytest.mark.parametrize('alternative', ['label_swap', 'minpixel_sort', 'transformed_root_sort',
                                         'transformed_root_key', 'filtered_relabel'])
def test_validator_rejects_wrong_label_and_root_order_alternatives(alternative):
    value = deepcopy(fixture())
    by_name = {c['id']: c for c in value['cases']}
    if alternative == 'filtered_relabel':
        by_name['filter-keeps-original-label-12x16']['expected']['kept_label_ids'] = [1]
    else:
        name = ('transformed-minpixel-counterexample-u-4x8' if alternative.startswith('transformed')
                else 'minpixel-counterexample-4x6')
        trace = by_name[name]['expected']
        if alternative == 'label_swap':
            relabel(trace, [2, 1])
        elif alternative == 'minpixel_sort':
            relabel(trace, [c['label'] for c in sorted(trace['components'], key=lambda c: c['min_flat_pixel'])])
        elif alternative == 'transformed_root_sort':
            relabel(trace, [c['label'] for c in sorted(trace['components'], key=lambda c: (c['min_flat_pixel']//8//2)*4 + c['min_flat_pixel']%8//2)])
        else:
            for component in trace['components']:
                pixel = component['min_flat_pixel']
                component['min_block_key'] = (pixel//8//2)*4 + pixel%8//2
    with pytest.raises(ValueError, match='parent trace differs'):
        parents.validate_fixture(value)
