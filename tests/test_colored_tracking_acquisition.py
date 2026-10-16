"""New-track probation uses independent acquisition evidence, never an ROI."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects.objects import StableObjectTracker, analyze_rgbd


def observation(x=0., color='red'):
    return {'color': color, 'position': np.array([x, .1, .02])}


def tracker(**kwargs):
    return StableObjectTracker(acquisition_min_frames=3, **kwargs)


def test_singletons_stay_unpublished_and_three_moving_acquisitions_confirm():
    state = tracker()
    assert state.update([observation(.25)], 1.) == []
    assert state.update([observation(.29)], 1.2) == []
    confirmed = state.update([observation(.33)], 1.4)
    assert len(confirmed) == 1
    assert confirmed[0]['acquisition_state'] == 'confirmed'
    assert confirmed[0]['acquisition_observations'] == 3
    # Established tracks move immediately, without another probation period.
    record = state.update([observation(.43)], 1.6)[0]
    assert record['track_id'] == confirmed[0]['track_id']


@pytest.mark.parametrize('interruption', ['missing', 'duplicate', 'backward', 'ambiguous'])
def test_missing_or_uncertain_acquisitions_cannot_accumulate_confirmation(interruption):
    state = tracker()
    state.update([observation()], 1.)
    if interruption == 'missing':
        state.update([], 1.2)
    elif interruption == 'ambiguous':
        state.update([observation(-.01), observation(.01)], 1.2)
    else:
        assert state.update([observation()], 1. if interruption == 'duplicate' else .9) == []
    assert state.update([observation()], 1.4) == []
    if interruption in {'missing', 'ambiguous'}:
        assert state.update([observation()], 1.6) == []
        assert len(state.update([observation()], 1.8)) == 1
    else:
        # Duplicate/backward calls are not acquisition frames and contribute zero.
        assert len(state.update([observation()], 1.6)) == 1


def test_pending_capacity_and_expiry_are_bounded_and_never_emit_expired_world_ids():
    state = tracker(max_pending_tracks=4, max_age_s=.3)
    objects = [observation(index, color=str(index)) for index in range(100)]
    assert state.update(objects, 1.) == []
    assert len(state.tracks) == 4
    assert state.acquisition_diagnostics()['pending_count'] == 4
    assert state.update([], 1.4) == []
    assert state.tracks == {}
    assert state.expired_tracks == []


def test_unconfirmed_tracks_cannot_reserve_identity_and_confirmed_ownership_survives():
    state = tracker(max_age_s=.3)
    state.update([observation()], 1.)
    pending_id = next(iter(state.tracks))
    state.ownership_status({'track_id': f'red-{pending_id:04d}', 'state': 'locked'})
    assert state.owned == set()
    state.update([observation()], 1.1)
    established = state.update([observation()], 1.2)[0]
    key = established['track_id']
    state.ownership_status({'track_id': f'red-{key:04d}', 'state': 'locked'})
    assert key in state.owned
    assert state.update([], 9.) == []
    held = state.update([observation(.7)], 9.2,
                        expected_owned_positions={key: np.array([.7,.1,.02])})
    assert held[0]['track_id'] == key
    assert held[0]['acquisition_state'] == 'confirmed'


@pytest.mark.parametrize('kwargs', [
    {'acquisition_min_frames': 0}, {'acquisition_min_frames': 1.5},
    {'acquisition_min_frames': True}, {'max_pending_tracks': 0},
    {'max_pending_tracks': np.inf},
])
def test_acquisition_parameters_are_positive_integers(kwargs):
    with pytest.raises(ValueError):
        StableObjectTracker(**kwargs)


def test_actual_rgbd_analyzer_acquires_moving_surface_beyond_pickup_bounds():
    state = tracker()
    intr = {'fx':500., 'fy':500., 'ppx':320., 'ppy':240.}
    transform = np.diag([1., -1., -1., 1.])
    transform[2,3] = .8
    for frame in range(6):
        rgb = np.full((480,640,3),128,np.uint8)
        depth = np.full((480,640),800,np.uint16)
        # Above the table and beyond the pickup ROI, moving 2cm per acquisition.
        center_x = .16 + frame * .02
        x = round(320 + 500 * center_x / .65)
        rgb[145:177,x-12:x+12] = [255,0,0]
        depth[145:177,x-12:x+12] = 650
        report = analyze_rgbd(rgb,depth,intr,np.zeros(5),.001,transform,
            support_plane={'normal':np.array([0.,0.,1.]),'d':0.})
        assert len(report['objects']) == 1
        assert report['objects'][0]['pickup_eligible'] is False
        result = state.update(report['objects'], 1. + frame * .2)
        assert len(result) == int(frame >= 2)
        if result:
            assert result[0]['track_id'] == 1


def test_ros_node_exposes_three_frame_default_and_bounded_pending_status():
    rclpy = pytest.importorskip('rclpy')
    from so101_station_objects.objects import ColoredObjectsNode
    rclpy.init()
    node = ColoredObjectsNode()
    try:
        assert node.get_parameter('acquisition_min_frames').value == 3
        assert node.tracker.acquisition_min_frames == 3
        assert node.get_parameter('max_pending_tracks').value == 64
    finally:
        node.analysis_worker.close()
        node.destroy_node()
        rclpy.shutdown()


def test_unknown_frame_breaks_pending_evidence_without_forgetting_confirmed_tracks():
    state = tracker()
    for stamp in (1.,1.2,1.4):
        established = state.update([observation()],stamp)
    key = established[0]['track_id']
    state.update([observation(),observation(.3,'blue')],1.6)
    state.invalidate_pending_acquisition(1.8, timestamp_is_trusted=True)
    assert list(state.tracks) == [key]
    assert state.update([observation(.3,'blue')],1.7) == []
    assert state.update([observation(),observation(.3,'blue')],2.)[0]['track_id'] == key
    # Old failed results cannot reset newer candidate evidence.
    state.invalidate_pending_acquisition(1.7)
    state.update([observation(),observation(.3,'blue')],2.2)
    records = state.update([observation(),observation(.3,'blue')],2.4)
    assert len(records) == 2


def test_late_expanded_ambiguity_resets_even_an_initially_clear_pending_assignment():
    state = tracker()
    state.update([observation(x) for x in (0.,.04,.06)],1.)
    assert state.update([observation(x) for x in (0.,.049,.051)],1.2) == []
    assert state.update([observation()],1.4) == []
    assert state.update([observation()],1.6) == []
    assert len(state.update([observation()],1.8)) == 1


def test_capacity_released_by_confirmation_is_independent_of_object_order():
    state = tracker(max_pending_tracks=1)
    state.update([observation()],1.)
    state.update([observation()],1.2)
    # Blue appears before red in this frame; red confirms and frees its slot.
    assert len(state.update([observation(.5,'blue'),observation()],1.4)) == 1
    assert state.acquisition_diagnostics()['pending_count'] == 1
    assert any(track['color'] == 'blue' for track in state.tracks.values())


def test_future_invalid_stamp_cannot_poison_established_acquisition_highwater():
    state = tracker()
    for stamp in (1.,1.2,1.4):
        record = state.update([observation()],stamp)
    key = record[0]['track_id']
    state.invalidate_pending_acquisition(1e12)
    assert state.update([observation()],1.6)[0]['track_id'] == key


def test_ros_future_unavailable_cannot_starve_confirmed_tracks():
    rclpy = pytest.importorskip('rclpy')
    from builtin_interfaces.msg import Time
    from so101_station_objects.objects import ColoredObjectsNode
    rclpy.init()
    node = ColoredObjectsNode()
    try:
        for stamp in (1.,1.2,1.4):
            records = node.tracker.update([observation()],stamp)
        key = records[0]['track_id']
        node._publish_unavailable(Time(sec=2_000_000_000), 'stale', 'future exposure')
        assert node.tracker.update([observation()],1.6)[0]['track_id'] == key
    finally:
        node.destroy_node()
        rclpy.shutdown()


def render_edge_surface(center_u, **kwargs):
    rgb = np.full((480,640,3),128,np.uint8)
    depth = np.full((480,640),800,np.uint16)
    rgb[220:252,max(0,center_u-16):center_u+16] = [255,0,0]
    depth[220:252,max(0,center_u-16):center_u+16] = 780
    intr = {'fx':500.,'fy':500.,'ppx':320.,'ppy':240.}
    transform = np.diag([1.,-1.,-1.,1.]); transform[2,3] = .8
    return analyze_rgbd(rgb,depth,intr,np.zeros(5),.001,transform,
        support_plane={'normal':np.array([0.,0.,1.]),'d':0.}, **kwargs)


def test_new_truncated_surface_acquires_identity_only_after_three_fully_visible_frames():
    state = tracker()
    for stamp in (1.,1.2,1.4):
        partial = render_edge_surface(10)['objects']
        assert len(partial) == 1
        assert partial[0]['acquisition_eligible'] is False
        assert partial[0]['acquisition_eligibility_reason'] == 'image_boundary_truncated'
        assert partial[0]['object_center_qualified'] is False
        assert partial[0]['pickup_eligible'] is False
        assert state.update(partial,stamp) == []
        assert state.acquisition_diagnostics()['pending_count'] == 0
    for index,stamp in enumerate((1.6,1.8,2.)):
        full = render_edge_surface(20)['objects']
        assert full[0]['acquisition_eligible'] is True
        result = state.update(full,stamp)
        assert len(result) == int(index == 2)
    assert result[0]['track_id'] == 1


def test_confirmed_surface_crossing_sensor_edge_keeps_id_as_visible_only_until_ttl():
    state = tracker()
    for stamp in (1.,1.2,1.4):
        records = state.update(render_edge_surface(20)['objects'],stamp)
    key = records[0]['track_id']
    for stamp,u in ((1.6,14),(1.8,10),(2.,6)):
        partial = render_edge_surface(u)['objects']
        records = state.update(partial,stamp)
        assert records[0]['track_id'] == key
        assert records[0]['acquisition_state'] == 'confirmed'
        assert records[0]['acquisition_eligible'] is False
        assert records[0]['observation_geometry'] == 'visible_surface_centroid'
        assert records[0]['object_center_qualified'] is False
        assert records[0]['grasps'] == []
    assert state.update([],3.2) == []
    assert key not in state.tracks
    assert state.expired_tracks[0]['track_id'] == f'red-{key:04d}'


def test_fully_visible_depth_child_survives_a_border_touching_color_parent():
    rgb = np.full((480,640,3),128,np.uint8)
    depth = np.full((480,640),800,np.uint16)
    rgb[150:220,:50] = [255,0,0]; depth[150:220,:50] = 1200
    rgb[180:182,50:100] = [255,0,0]; depth[180:182,50:100] = 1200
    rgb[165:197,100:132] = [255,0,0]; depth[165:197,100:132] = 650
    intr = {'fx':500.,'fy':500.,'ppx':320.,'ppy':240.}
    transform = np.diag([1.,-1.,-1.,1.]); transform[2,3] = .8
    report = analyze_rgbd(rgb,depth,intr,np.zeros(5),.001,transform,
        support_plane={'normal':np.array([0.,0.,1.]),'d':0.})
    foreground = [item for item in report['objects']if item['surface_centroid'][2] > .1]
    assert len(foreground) == 1
    assert foreground[0]['acquisition_eligible'] is True
    assert foreground[0]['image_boundary_sides'] == []


def test_fully_visible_touching_same_depth_child_has_no_parent_border_veto():
    rgb = np.full((480,640,3),128,np.uint8)
    depth = np.full((480,640),800,np.uint16)
    for ys,xs in [(slice(220,252),slice(0,32)),(slice(220,252),slice(40,72)),
                  (slice(234,238),slice(32,40))]:
        rgb[ys,xs] = [255,0,0]; depth[ys,xs] = 780
    intr = {'fx':500.,'fy':500.,'ppx':320.,'ppy':240.}
    transform = np.diag([1.,-1.,-1.,1.]); transform[2,3] = .8
    report = analyze_rgbd(rgb,depth,intr,np.zeros(5),.001,transform,
        support_plane={'normal':np.array([0.,0.,1.]),'d':0.})
    assert len(report['objects']) == 2
    assert sorted(item['acquisition_eligible'] for item in report['objects']) == [False,True]
    assert sorted(item['image_boundary_sides'] for item in report['objects']) == [[],['left']]


def test_owned_partial_view_stays_confirmed_with_timestamped_expected_position():
    state = tracker()
    for stamp in (1.,1.2,1.4):
        result = state.update(render_edge_surface(20)['objects'],stamp)
    key = result[0]['track_id']
    state.ownership_status({'track_id':f'red-{key:04d}','state':'attached'})
    partial = render_edge_surface(10)['objects']
    result = state.update(partial,1.6,expected_owned_positions={key:partial[0]['position']})
    assert result[0]['track_id'] == key
    assert result[0]['acquisition_eligible'] is False
    assert key in state.owned


def test_pending_truncation_resets_confirmation_before_a_full_new_acquisition():
    state = tracker()
    state.update(render_edge_surface(20)['objects'],1.)
    state.update(render_edge_surface(20)['objects'],1.2)
    assert state.update(render_edge_surface(10)['objects'],1.4) == []
    assert state.acquisition_diagnostics()['pending_count'] == 0
    for index,stamp in enumerate((1.6,1.8,2.)):
        result = state.update(render_edge_surface(20)['objects'],stamp)
        assert len(result) == int(index==2)


def test_recorded_wide_frames_keep_all_eight_real_bricks_and_withhold_clipped_blue_ids():
    """Offline annotations are evaluation labels, never runtime admission bounds."""
    import json
    import cv2
    capture = Path.home()/'.local/share/exciting-science/station/evidence/2026-10-16-sorting/camera-wide848-trial'
    if not capture.is_dir():
        pytest.skip('local synchronized wide-camera development capture unavailable')
    state = tracker()
    usable = clipped_blue = 0
    for directory in sorted(capture.glob('frame-*')):
        metadata = json.loads((directory/'info.json').read_text())
        transform = np.asarray(metadata['board_from_camera'])
        if transform.shape != (4,4):
            continue
        rgb = cv2.cvtColor(cv2.imread(str(directory/'color.png')),cv2.COLOR_BGR2RGB)
        depth = np.load(directory/'depth_aligned.npy')
        k = metadata['k']
        report = analyze_rgbd(rgb,depth,dict(fx=k[0],fy=k[4],ppx=k[2],ppy=k[5]),
                              metadata['d'],.001,transform)
        real = []
        for item in report['objects']:
            camera = np.linalg.inv(transform) @ np.append(item['surface_centroid'],1.)
            pixel = camera[:2]/camera[2]*[k[0],k[4]]+[k[2],k[5]]
            if 454 < pixel[0] < 679 and 290 < pixel[1] < 445:
                real.append(item)
            elif item['color'] == 'blue':
                clipped_blue += 1
                assert item['acquisition_eligible'] is False
                assert item['image_boundary_sides'] == ['left']
        assert len(real) == 8
        assert all(item['acquisition_eligible'] for item in real)
        records = state.update(report['objects'],metadata['stamp'])
        usable += 1
        assert len(records) == (8 if usable >= 3 else 0)
    assert usable == 18
    assert clipped_blue == 3


def test_optional_observation_roi_cannot_hide_sensor_truncation_or_invent_a_full_shape():
    complete_observation = render_edge_surface(10)['objects'][0]
    # Cuts away the sensor-border pixels in world coordinates. This controls
    # admission only; it cannot make the rest a newly complete brick surface.
    roi_observation = render_edge_surface(10,
        observation_roi=(-.48,1.,-1.,1.,-1.,1.))['objects'][0]
    assert roi_observation['acquisition_eligible'] is False
    assert roi_observation['image_boundary_sides'] == ['left']
    assert roi_observation['pixel_area'] == complete_observation['pixel_area'] == 832
    np.testing.assert_allclose(roi_observation['position'],complete_observation['position'])
    np.testing.assert_allclose(roi_observation['dimensions_m'],complete_observation['dimensions_m'])
    assert roi_observation['object_center_qualified'] is False
