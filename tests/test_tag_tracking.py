import cv2
import numpy as np
import pytest
from workshop.digital_twin.tag_tracking import TagTracker, depth_size, pose_candidates, square_points, black_edge_from_total

K = dict(fx=600., fy=600., ppx=320., ppy=240.)


def detection(x=320., y=240., size=.025, z=.5, code=0):
    h = size*K['fx']/z/2
    return {'code_id': code, 'corners_px': [[x-h,y-h], [x+h,y-h], [x+h,y+h], [x-h,y+h]]}


def test_multiple_repeated_id_and_source_resize_coordinates():
    image = np.full((480, 640), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9), 0, 60)
    for x, y in [(100, 100), (260, 180), (440, 320)]:
        image[y:y+60, x:x+60] = marker
    tracker = TagTracker()
    found = tracker.process(image, K)
    assert len(found) == 3
    assert {d['code_id'] for d in found} == {0}
    assert len({d['track_id'] for d in found}) == 3
    centers = sorted(d['center_px'][0] for d in found)
    np.testing.assert_allclose(centers, [129.5, 289.5, 469.5], atol=1)
    found2 = tracker.process(image, K)
    assert [d['track_id'] for d in found] == [d['track_id'] for d in found2]
    assert all(d['pose'] is None and not d['qualified'] for d in found)


@pytest.mark.parametrize('size', [.010, .025])
def test_depth_edge_size_and_robust_invalid_pixels(size):
    depth = np.full((480, 640), 500, dtype=np.uint16)
    corners = detection(size=size)['corners_px']
    for u, v in corners:
        x, y = np.rint([u, v]).astype(int)
        depth[y, x] = 0
        depth[y-1, x-1] = 2000
    result = depth_size(corners, depth, K, .001, candidates=(.01, .025))
    assert result['size_m'] == size
    assert result['relative_error'] < 1e-10


def test_invalid_depth_spread_and_print_margin_are_unknown():
    corners = detection()['corners_px']
    depth = np.zeros((480, 640), dtype=np.uint16)
    assert depth_size(corners, depth, K, .001)['size_m'] is None
    depth[:] = 500
    x, y = np.rint(corners[0]).astype(int)
    depth[y-1:y+2, x-1:x+2] = [[300, 300, 300], [500, 500, 500], [700, 700, 700]]
    assert depth_size(corners, depth, K, .001)['reason'] == 'corner_depth_spread'
    result = depth_size(detection(size=.014)['corners_px'], np.full_like(depth, 500), K, .001)
    assert result['size_m'] is None


def test_association_occlusion_and_size_conflict():
    tracker = TagTracker(max_missed=2, size_candidates=(.01, .025))
    depth = np.full((480, 640), 500, dtype=np.uint16)
    first = tracker.update([detection(x=100), detection(x=400)], K, depth)
    second = tracker.update([detection(x=405)], K, depth)
    assert second[0]['track_id'] == first[1]['track_id']
    third = tracker.update([detection(x=110), detection(x=410)], K, depth)
    assert third[0]['track_id'] == first[0]['track_id']
    assert third[0]['reassociated_after_missed_frames'] == 1
    conflict = tracker.update([detection(x=115, size=.01)], K, depth)[0]
    assert conflict['size_conflict'] and conflict['pose'] is None
    tracker.set_size_override(conflict['track_id'], .025, 'operator declared outer black edge')
    overridden = tracker.update([detection(x=115, size=.01)], K, depth)[0]
    assert overridden['size_conflict'] and overridden['size_m'] == .025
    assert overridden['pose'] and not overridden['qualified']
    assert overridden['size_source'].startswith('operator')


def test_configurable_black_border_size():
    tracker = TagTracker(size_candidates=(.025*7/9, .01*7/9))
    found = tracker.update([detection(size=.025*7/9)], K, np.full((480,640),500,np.uint16))[0]
    assert found['size_m'] == .025*7/9


def test_ippe_axes_positive_depth_and_corner_order():
    obj = square_points(.025)
    rotation = np.array([2.8, .15, .3])
    translation = np.array([.01, -.02, .5])
    k = np.array([[600.,0,320],[0,600.,240],[0,0,1.]])
    corners = cv2.projectPoints(obj, rotation, translation, k, np.zeros(5))[0].reshape(4,2)
    result = pose_candidates(corners, K, .025)
    assert len(result['candidates']) == 2
    best = result['candidates'][0]
    transform = np.asarray(best['camera_from_tag'])
    np.testing.assert_allclose(transform[:3,3], translation, atol=1e-8)
    np.testing.assert_allclose(transform[:3,:3], cv2.Rodrigues(rotation)[0], atol=1e-8)
    assert best['reprojection_rms_px'] < 1e-6
    assert result['ambiguous']  # no depth measurement supplied
    assert (obj @ transform[:3,:3].T + transform[:3,3])[:,2].min() > 0


def test_frontal_pose_keeps_ambiguity_and_expired_tracks_not_reused():
    result = pose_candidates(detection()['corners_px'], K, .025, corner_depth_m=[.5]*4)
    assert result['ambiguous'] and result['selected_index'] is None
    tracker = TagTracker(max_missed=0)
    first = tracker.update([detection()], K)[0]['track_id']
    tracker.update([], K)
    assert tracker.update([detection()], K)[0]['track_id'] != first


def test_alignment_validation():
    with pytest.raises(ValueError, match='aligned'):
        TagTracker().process(np.zeros((480,640,3),np.uint8), K, np.zeros((20,30)))


def test_user_total_tile_size_conversion():
    assert black_edge_from_total(.025) == pytest.approx(.0194444444444)
    assert black_edge_from_total(.010) == pytest.approx(.00777777777778)
    tracker = TagTracker()
    found = tracker.update([detection(size=black_edge_from_total(.025))], K,
                           np.full((480,640),500,np.uint16))[0]
    assert found['size_m'] == pytest.approx(.0194)


def test_manual_override_enables_candidates_without_depth_not_qualification():
    tracker = TagTracker()
    first = tracker.update([detection()], K)[0]
    tracker.set_size_override(first['track_id'], .0194, 'operator measured black border')
    second = tracker.update([detection()], K)[0]
    assert second['pose']['candidates'] and not second['qualified']
    assert second['pose']['ambiguous']
    with pytest.raises(ValueError):
        tracker.set_size_override(first['track_id'], -.1, 'bad')
    with pytest.raises(ValueError):
        tracker.set_size_override(first['track_id'], .0194, '')


def test_invalid_scale_and_empty_size_configuration():
    with pytest.raises(ValueError):
        depth_size(detection()['corners_px'], np.zeros((480,640)), K, float('nan'))
    with pytest.raises(ValueError):
        TagTracker(size_candidates=())
