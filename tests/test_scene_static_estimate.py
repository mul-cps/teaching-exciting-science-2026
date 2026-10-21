"""Focused contracts for the provisional scene estimate, independent of ROS."""
import itertools
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_scene'))
from so101_station_scene.scene_geometry import (
    LiveObjectRegistry, _CUBOID_SYMMETRIES, matrix_quaternion_xyzw,
    quaternion_matrix_xyzw,
)


def registry(**kwargs):
    return LiveObjectRegistry(ttl_s=.75, min_extent_m=.002, max_extent_m=.3,
                              padding_m=.002, stabilize_static_objects=True, **kwargs)


def observe(r, p=(0, 0, .1), size=(.04, .03, .02), q=(0, 0, 0, 1), t=1):
    return r.update('brick', p, size, q, now_mono_s=t)


def contains(item, p, size, q):
    signs = np.array(list(itertools.product((-1, 1), repeat=3)))
    corners = signs * (np.asarray(size) / 2 + .002)
    corners = corners @ quaternion_matrix_xyzw(q).T + p
    local = (corners - item.center_m) @ quaternion_matrix_xyzw(item.orientation_xyzw)
    assert np.all(np.abs(local) <= np.asarray(item.size_m) / 2 + 1e-12)


def test_static_pose_contains_raw_padded_box_even_with_near_square_yaw_noise():
    r = registry()
    first = observe(r)
    for i, angle in enumerate((.5, -.4, .2, -.1), 1):
        q = (0, 0, np.sin(angle/2), np.cos(angle/2))
        p = (.001 * (-1)**i, .001, .1)
        item = observe(r, p, q=q, t=1 + i*.1)
        assert item.center_m == first.center_m
        assert item.orientation_xyzw == first.orientation_xyzw
        contains(item, p, (.04, .03, .02), q)


def test_all_24_axis_representations_preserve_dimensions_and_orientation():
    assert len(_CUBOID_SYMMETRIES) == 24
    for rotation in _CUBOID_SYMMETRIES:
        r = registry()
        first = observe(r)
        size = np.abs(rotation).T @ np.array((.04, .03, .02))
        item = observe(r, size=size, q=matrix_quaternion_xyzw(rotation), t=1.1)
        assert item.orientation_xyzw == pytest.approx(first.orientation_xyzw)
        assert item.size_m == pytest.approx(first.size_m)


@pytest.mark.parametrize('p', [(0.01, 0, .1), (0, 0, .11)])
def test_ten_mm_real_motion_is_immediate(p):
    r = registry()
    observe(r)
    assert observe(r, p, t=1.1).center_m == pytest.approx(p)


def test_two_coherent_residuals_release_then_three_consistent_samples_hold():
    r = registry()
    observe(r)
    assert observe(r, (.0035, 0, .1), t=1.1).center_m[0] == 0
    assert observe(r, (.004, 0, .1), t=1.2).center_m[0] == .004
    observe(r, (.0042, 0, .1), t=1.3)
    held = observe(r, (.0041, 0, .1), t=1.4)
    assert observe(r, (.0043, 0, .1), t=1.5).center_m == held.center_m


def test_growth_immediate_shrink_requires_three_fresh_samples():
    r = registry()
    observe(r)
    grown = observe(r, size=(.06, .03, .02), t=1.1)
    for t in (1.2, 1.3):
        assert observe(r, t=t).size_m == grown.size_m
    assert observe(r, t=1.4).size_m[0] == pytest.approx(.044)


def test_ttl_stale_gap_and_attach_reset_state_without_identity_changes():
    r = registry()
    observe(r)
    assert observe(r, (.002, 0, .1), t=2).center_m[0] == .002
    r.lock('brick')
    assert not r._estimates
    r.detach('brick')
    assert observe(r, (.003, 0, .1), t=2.1).center_m[0] == .003
    assert r.expire(now_mono_s=2.849) == []
    assert r.expire(now_mono_s=2.85) == ['colorbrick_brick']
    assert not r._estimates


def test_duplicate_receipt_does_not_advance_three_sample_history():
    r = registry()
    observe(r)
    grown = observe(r, size=(.06, .03, .02), t=1.1)
    for _ in range(5):
        assert observe(r, t=1.1).size_m == grown.size_m
    assert len(r._estimates['colorbrick_brick'].extents) == 2


def test_estimate_state_obeys_registry_capacity_and_expiration():
    r = registry(max_objects=1)
    observe(r)
    with pytest.raises(ValueError, match='limit'):
        r.update('other', (0, 0, .1), (.04, .03, .02), (0, 0, 0, 1), now_mono_s=1.1)
    assert len(r._estimates) == 1
    r.expire(now_mono_s=2)
    assert not r._estimates
