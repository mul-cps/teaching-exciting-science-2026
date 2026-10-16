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
