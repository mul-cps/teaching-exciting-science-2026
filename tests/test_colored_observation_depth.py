"""Far-depth admission precedes geometry and preserves original color evidence."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2/src/so101_station_objects'))
from so101_station_objects import objects
from so101_station_objects.colored_object_validation import (
    split_depth_coherent_components, validate_colored_component)

INTR = {'fx':500.,'fy':500.,'ppx':320.,'ppy':240.}
TRANSFORM = np.diag([1.,-1.,-1.,1.]); TRANSFORM[2,3] = .8
PLANE = {'normal':np.array([0.,0.,1.]),'d':0.}


def scene():
    rgb = np.full((480,640,3),128,np.uint8)
    depth = np.full((480,640),800,np.uint16)
    return rgb,depth


def test_far_color_parent_is_rejected_before_any_camera_geometry(monkeypatch):
    rgb,depth = scene()
    rgb[200:224,300:324] = [255,0,0]
    depth[200:224,300:324] = 1400
    def unexpected_geometry(*args,**kwargs):
        pytest.fail('far parent reached expensive camera/SVD geometry')
    monkeypatch.setattr(objects,'camera_points',unexpected_geometry)
    monkeypatch.setattr(np.linalg,'svd',unexpected_geometry)
    report = objects.analyze_rgbd(rgb,depth,INTR,np.zeros(5),.001,TRANSFORM,
                                  support_plane=PLANE,max_observation_depth_m=1.2)
    assert report['objects'] == []
    assert report['observation_depth_range_m'] == [.1,1.2]
    assert report['validation_diagnostics']['rejection_reasons'] == {
        'outside_observation_depth_range':1}


def test_near_brick_joined_to_far_color_background_survives_with_original_support():
    rgb,depth = scene()
    rgb[200:232,300:332] = [255,0,0]; depth[200:232,300:332] = 650
    rgb[110:170,260:380] = [255,0,0]; depth[110:170,260:380] = 1400
    rgb[170:200,315:317] = [255,0,0]; depth[170:200,315:317] = 1400
    report = objects.analyze_rgbd(rgb,depth,INTR,np.zeros(5),.001,TRANSFORM,
                                  support_plane=PLANE,max_observation_depth_m=1.2)
    assert len(report['objects']) == 1
    item = report['objects'][0]
    assert item['surface_centroid'][2] == pytest.approx(.15,abs=.003)
    assert item['depth_validation']['metrics']['local_support_pixels'] > 1024
    assert item['depth_validation']['metrics']['supported_fraction'] >= .90


def test_far_depth_holes_are_unknown_support_and_cannot_be_cropped_into_a_brick():
    rgb,depth = scene()
    mask = np.zeros(depth.shape,bool); mask[180:240,280:360] = True
    rgb[mask] = [255,0,0]
    metric = depth.astype(np.float32)*.001
    metric[mask] = 1.4
    metric[200:220,300:340] = .65
    pieces = split_depth_coherent_components(mask,metric,max_observation_depth_m=1.2)
    assert len(pieces) == 1
    validation = validate_colored_component(rgb,metric,pieces[0].mask,INTR,
        support_mask=pieces[0].support_mask,max_observation_depth_m=1.2)
    assert not validation.accepted
    assert validation.reason == 'locally_unsupported_color_depth'
    assert validation.metrics['supported_fraction'] < .90


def test_depth_boundary_is_inclusive_and_explicit_larger_range_is_supported():
    rgb,depth = scene()
    mask = np.zeros(depth.shape,bool); mask[200:224,300:324] = True
    rgb[mask] = [255,0,0]
    metric = depth.astype(np.float32)*.001
    metric[mask] = np.float32(1.2)
    assert len(split_depth_coherent_components(mask,metric,max_observation_depth_m=1.2)) == 1
    metric[mask] = np.nextafter(np.float32(1.2),np.float32(np.inf))
    assert split_depth_coherent_components(mask,metric,max_observation_depth_m=1.2) == ()
    metric[mask] = 1.4
    assert split_depth_coherent_components(mask,metric) == ()
    assert len(split_depth_coherent_components(mask,metric,max_observation_depth_m=2.)) == 1
    assert validate_colored_component(rgb,metric,mask,INTR,max_observation_depth_m=2.).accepted


@pytest.mark.parametrize('limit',[0.,.1,-1.,np.nan,np.inf,True])
def test_invalid_depth_limits_fail_even_for_empty_color_components(limit):
    rgb,depth = scene()
    mask = np.zeros(depth.shape,bool)
    metric = depth.astype(np.float32)*.001
    with pytest.raises(ValueError):
        split_depth_coherent_components(mask,metric,max_observation_depth_m=limit)
    with pytest.raises(ValueError):
        validate_colored_component(rgb,metric,mask,INTR,max_observation_depth_m=limit)
    with pytest.raises(ValueError):
        objects.analyze_rgbd(rgb,depth,INTR,np.zeros(5),.001,TRANSFORM,
                            support_plane=PLANE,max_observation_depth_m=limit)


def test_confirmed_reserved_brick_beyond_range_is_unobserved_without_losing_ownership():
    state = objects.StableObjectTracker()
    rgb,depth = scene()
    rgb[200:224,300:324] = [255,0,0]
    depth[200:224,300:324] = 700
    near = objects.analyze_rgbd(rgb,depth,INTR,np.zeros(5),.001,TRANSFORM,support_plane=PLANE)
    key = state.update(near['objects'],1.)[0]['track_id']
    state.ownership_status({'track_id':f'red-{key:04d}','state':'attached'})
    depth[200:224,300:324] = 1400
    far = objects.analyze_rgbd(rgb,depth,INTR,np.zeros(5),.001,TRANSFORM,support_plane=PLANE)
    assert far['objects'] == []
    assert state.update(far['objects'],3.) == []
    assert key in state.owned
    assert key in state.tracks
    assert state.tracks[key]['stamp'] == 1.


def test_local_original_far_hole_is_still_unknown_after_boundary_cuts():
    rgb,depth = scene()
    mask = np.zeros(depth.shape,bool); mask[200:232,300:332] = True
    rgb[mask] = [255,0,0]
    metric = depth.astype(np.float32)*.001
    metric[mask] = .65
    metric[208:224,308:324] = 1.4
    pieces = split_depth_coherent_components(mask,metric)
    assert len(pieces) == 1
    assert pieces[0].support_mask.sum() == 1024
    validation = validate_colored_component(rgb,metric,pieces[0].mask,INTR,
                                           support_mask=pieces[0].support_mask)
    assert not validation.accepted
    assert validation.metrics['supported_fraction'] == .75


def test_native_configured_limit_reaches_normal_and_cached_plane_analysis(monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    rclpy.init(args=['--ros-args','-p','max_observation_depth_m:=0.9'])
    node = objects.ColoredObjectsNode()
    try:
        rgb,depth = scene()
        rgb[200:224,300:324] = [255,0,0]; depth[200:224,300:324] = 1000
        job = {'rgb':rgb,'depth':depth,'intrinsics':INTR,'distortion':np.zeros(5),
               'scale':.001,'board_from_camera':TRANSFORM,'stamp':10.2,
               'support_plane_cache':{**PLANE,'stamp':10.,'plane_source':'measured_table_plane'}}
        assert node.get_parameter('max_observation_depth_m').value == .9
        current = node._analyze_job(job)
        assert current['objects'] == []
        assert current['observation_depth_range_m'] == [.1,.9]
        def unavailable_support(*args,**kwargs):
            raise objects.SupportPlaneUnavailable('exercise measured-plane cache')
        monkeypatch.setattr(objects,'_fit_world_support',unavailable_support)
        cached = node._analyze_job(job)
        assert cached['objects'] == []
        assert cached['observation_depth_range_m'] == [.1,.9]
        assert cached['support_plane']['plane_source'] == 'cached_measured_support_plane'
    finally:
        node.destroy_node()
        rclpy.shutdown()
