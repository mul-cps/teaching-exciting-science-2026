import numpy as np
import pytest
from workshop.digital_twin.grasp_geometry import (fuse_depth, fit_support_plane, camera_ray_plane, footprint_grasps)


def test_crop_valid_median_and_mad():
    a = np.zeros((4, 5, 6), dtype=np.uint16)
    a[:, 1:3, 2:4] = np.array([0, 100, 102, 900])[:, None, None]
    result = fuse_depth(a, (2, 1, 4, 3))
    assert result['median'].shape == (2, 2)
    assert np.all(result['median'] == 102)
    assert np.all(result['mad'] == 2)
    assert np.all(result['count'] == 3)
    assert result['stable_mask'].all()
    empty = fuse_depth(a, (0, 0, 1, 1))
    assert empty['median'][0, 0] == 0
    assert not empty['stable_mask'][0, 0]


def plane_cloud(seed=3, offset=.5):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-.1, .1, (800, 2))
    z = offset + .2 * xy[:, 0] + .1 * xy[:, 1]
    return np.column_stack((xy, z + rng.normal(0, .0003, len(z))))


def test_oblique_plane_heldout_and_outliers():
    p = plane_cloud()
    p[:80, 2] += .03
    result = fit_support_plane(p)
    heldout = plane_cloud(4)
    residual = abs(heldout @ result['normal'] + result['d'])
    assert np.percentile(residual, 95) < .0007
    assert result['d'] > 0
    assert result['inlier_mask'].sum() >= 710
    other = fit_support_plane(plane_cloud(offset=.55))
    assert abs(other['d'] - result['d']) > .04


def test_degenerate_plane_rejected():
    with pytest.raises(ValueError):
        fit_support_plane(np.column_stack((np.arange(10), np.zeros((10, 2)))))


def test_rectangle_contacts_and_opening():
    intrinsics = dict(fx=500, fy=500, ppx=320, ppy=240)
    plane = dict(normal=np.array([0., 0., -1.]), d=.5)
    pixels = [[310, 235], [330, 235], [330, 245], [310, 245]]
    assert np.allclose(camera_ray_plane([330, 245], intrinsics, plane['normal'], .5), [.01, .005, .5])
    grasps = footprint_grasps(pixels, plane, intrinsics, .01, .021)
    assert len(grasps) == 2
    assert sorted(round(g['opening_m'], 6) for g in grasps) == [.01, .02]
    for g in grasps:
        assert np.allclose(g['center'], [0, 0, .495])
        assert np.linalg.norm(g['contacts'][1] - g['contacts'][0]) == pytest.approx(g['opening_m'])
        assert np.allclose(g['contacts'] @ plane['normal'] + .5, .005)
        assert g['contact_above_support']
        assert not g['finger_clearance_verified']
        assert not g['executable']
        assert g['projection_assumption'] == 'support-plane approximation'
    narrow = footprint_grasps(pixels, plane, intrinsics, .01, .015)
    assert len(narrow) == 1
    assert narrow[0]['opening_m'] <= .015
    assert not footprint_grasps(pixels, plane, intrinsics, .01, .005)


def test_ray_parallel_and_behind():
    intrinsics = dict(fx=500, fy=500, ppx=320, ppy=240)
    with pytest.raises(ValueError):
        camera_ray_plane([320, 240], intrinsics, [1, 0, 0], .5)
    with pytest.raises(ValueError):
        camera_ray_plane([320, 240], intrinsics, [0, 0, 1], .5)


def test_oblique_known_contour_height():
    intrinsics = dict(fx=600, fy=600, ppx=320, ppy=240)
    n = np.array([0., -.6, -.8])
    d = .5
    x = np.array([1., 0., 0.])
    y = np.cross(n, x)
    support_center = -d * n
    height = .02
    top = support_center + height * n
    corners = np.array([top + sx * .02 * x + sy * .01 * y for sx, sy in [(-1,-1),(1,-1),(1,1),(-1,1)]])
    pixels = np.column_stack((600 * corners[:,0] / corners[:,2] + 320, 600 * corners[:,1] / corners[:,2] + 240))
    grasps = footprint_grasps(pixels, dict(normal=n, d=d), intrinsics, height, .05, contour_height_m=height)
    assert sorted(round(g['opening_m'], 6) for g in grasps) == [.02, .04]
    for grasp in grasps:
        assert np.allclose(grasp['center'], support_center + height / 2 * n)
        assert not grasp['executable']
    with pytest.raises(ValueError):
        footprint_grasps(pixels, dict(normal=n,d=d), intrinsics, height, .05, contour_height_m=.03)
