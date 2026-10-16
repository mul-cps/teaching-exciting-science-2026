import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workshop.digital_twin.placement_region import (
    PlacementRegionTracker,
    SparseRGBDAnchor,
    deproject_pixels,
    fit_rigid_pose,
    intersect_ray_with_image_plane,
    project_rectangle_to_plane,
    pose_from_rectangle,
)


def test_sparse_anchor_tracks_feeder_translation_and_rotating_goal_area():
    tracker = PlacementRegionTracker(max_age_s=.25, inlier_threshold_m=.012)
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    corners = np.array([[-.08, -.04, 0.], [.08, -.04, 0.],
                        [.08, .04, 0.], [-.08, .04, 0.]])
    assert tracker.set_anchor(1_000_000_000, source, frame_id="camera_color_optical_frame",
                              generation="frame-a", received_monotonic=10.)
    assert tracker.set_goal(1_000_000_000, corners, generation="frame-a",
                            received_monotonic=10.)

    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("y", 12, degrees=True).as_matrix()
    pose[:3, 3] = [.13, -.02, .04]
    current = source @ pose[:3, :3].T + pose[:3, 3]
    result = tracker.update(1_100_000_000, range(len(current)), current,
                            generation="frame-b", received_monotonic=10.1)

    assert result.valid
    assert result.reason == "tracked"
    np.testing.assert_allclose(result.root_from_anchor, pose, atol=1e-8)
    np.testing.assert_allclose(result.goal_corners_root,
                               corners @ pose[:3, :3].T + pose[:3, 3], atol=1e-8)
    assert result.placement_motion_qualified is False


def test_area_follows_repeated_feeder_axis_motion_and_current_camera_stamp():
    tracker = PlacementRegionTracker(max_age_s=.25)
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    corners = np.array([[-.04, -.02, .01], [.04, -.02, .01],
                        [.04, .02, .01], [-.04, .02, .01]])
    tracker.set_anchor(1_000_000_000, source, frame_id="camera_color_optical_frame",
        generation="axis-0", received_monotonic=2.)
    tracker.set_goal(1_000_000_000, corners, generation="axis-0", received_monotonic=2.)
    for index in range(1, 11):
        displacement = np.array([0., .003 * index, .001 * index])
        result = tracker.update(1_000_000_000 + index * 100_000_000,
            range(len(source)), source + displacement, generation=f"axis-{index}",
            received_monotonic=2. + .1 * index)
        assert result.valid
        np.testing.assert_allclose(result.goal_corners_root,
                                   corners + displacement, atol=1e-8)
        assert result.stamp_ns == 1_000_000_000 + index * 100_000_000


def test_occlusion_and_stale_input_invalidate_goal_without_static_fallback():
    tracker = PlacementRegionTracker(max_age_s=.25)
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    corners = np.array([[-.02, -.02, 0.], [.02, -.02, 0.],
                        [.02, .02, 0.], [-.02, .02, 0.]])
    tracker.set_anchor(1_000_000_000, source, frame_id="camera", generation="a",
                       received_monotonic=1.)
    tracker.set_goal(1_000_000_000, corners, generation="a", received_monotonic=1.)
    occluded = tracker.update(1_100_000_000, [], np.empty((0, 3)), generation="b",
                              received_monotonic=1.1)
    assert not occluded.valid
    assert occluded.reason == "insufficient_features"
    assert occluded.goal_corners_root is None
    assert occluded.last_valid_goal_corners_root is not None

    stale = tracker.update(1_200_000_000, range(len(source)), source, generation="c",
                           received_monotonic=1.5, now_monotonic=2.)
    assert not stale.valid
    assert stale.reason == "stale_frame"
    assert stale.goal_corners_root is None


def test_two_competing_rigid_tracks_are_reported_ambiguous_and_can_reacquire():
    tracker = PlacementRegionTracker(max_age_s=.25)
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    corners = np.array([[-.02, -.02, 0.], [.02, -.02, 0.],
                        [.02, .02, 0.], [-.02, .02, 0.]])
    tracker.set_anchor(1_000_000_000, source, frame_id="camera", generation="a",
                       received_monotonic=1.)
    tracker.set_goal(1_000_000_000, corners, generation="a", received_monotonic=1.)
    moved = source + [.1, 0., 0.]
    other = source + [.3, .05, 0.]
    ids = list(range(len(source))) + list(range(len(source)))
    observed = np.concatenate([moved, other])
    ambiguous = tracker.update(1_100_000_000, ids, observed, generation="b",
                              received_monotonic=1.1)
    assert not ambiguous.valid
    assert ambiguous.reason == "ambiguous_motion"
    assert ambiguous.goal_corners_root is None

    reacquired = tracker.update(1_200_000_000, range(len(source)), moved,
                                generation="c", received_monotonic=1.2)
    assert reacquired.valid
    assert reacquired.reason == "tracked"
    np.testing.assert_allclose(reacquired.goal_corners_root, corners + [.1, 0., 0.], atol=1e-8)


def test_duplicate_anchor_ids_and_nonmonotonic_frames_are_rejected():
    tracker = PlacementRegionTracker(minimum_features=4)
    points = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]])
    assert not tracker.set_anchor(1, points[:3], frame_id="camera", generation="g",
                                  received_monotonic=0.)
    assert tracker.reason == "insufficient_features"
    assert tracker.set_anchor(2, points, frame_id="camera", generation="g",
                              received_monotonic=0.)
    assert tracker.set_goal(2, points, generation="g", received_monotonic=0.)
    result = tracker.update(2, [0, 0, 2, 3], points, generation="h",
                            received_monotonic=.1)
    assert not result.valid
    assert result.reason == "nonmonotonic_frame"


def test_fit_rigid_pose_rejects_collinear_and_large_residual_points():
    line = np.array([[float(index), 0., 0.] for index in range(8)])
    pose, status = fit_rigid_pose(line, line + [0., .1, 0.])
    assert pose is None and status.reason == "insufficient_inliers"
    cloud = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.],
                      [1., 1., 0.], [.4, .7, .2], [.2, .1, .4]])
    moved = cloud + [0., 0., .1]
    moved[-1] += [.2, 0., 0.]
    pose, status = fit_rigid_pose(cloud, moved, inlier_threshold_m=.01)
    assert pose is None
    assert status.reason == "insufficient_inliers"


def test_rectified_camera_deprojection_and_plane_projection_are_explicit():
    k = np.array([[600., 0., 424.], [0., 600., 240.], [0., 0., 1.]])
    pixels = np.array([[424., 240.], [484., 240.]])
    points = deproject_pixels(pixels, np.array([1., 1.]), k)
    np.testing.assert_allclose(points, [[0., 0., 1.], [.1, 0., 1.]])
    depth = np.ones((480, 848), dtype=float)
    polygon, metrics = project_rectangle_to_plane((400, 220, 448, 260), depth, k,
                                                   np.eye(4), min_samples=20)
    assert polygon.shape == (4, 3)
    assert metrics["plane_rms_m"] < 1e-10
    np.testing.assert_allclose(polygon[:, 2], 1.)


def test_click_ray_is_bound_to_image_plane_generation_and_pixel_shape():
    # Image plane at z=1, spanning calibrated image extent around the optical axis.
    k = np.array([[600., 0., 424.], [0., 600., 240.], [0., 0., 1.]])
    hit = intersect_ray_with_image_plane((0., 0., 0.), (0., 0., 1.),
        np.eye(4), k, (848, 480), distance_m=1.)
    assert hit == pytest.approx((424., 240.))
    assert intersect_ray_with_image_plane((0., 0., 0.), (1., 0., 0.),
        np.eye(4), k, (848, 480), distance_m=1.) is None


def test_sparse_rgbd_frontend_tracks_motion_and_reacquires_after_short_occlusion():
    rng = np.random.default_rng(4)
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[30:90, 35:125] = rng.integers(30, 220, (60, 90, 3), dtype=np.uint8)
    depth = np.ones((120, 160), dtype=np.float32)
    k = np.array([[300., 0., 80.], [0., 300., 60.], [0., 0., 1.]])
    frontend = SparseRGBDAnchor(PlacementRegionTracker(minimum_features=6))
    assert frontend.select_anchor(image, depth, k, np.eye(4), (32, 27, 128, 94),
        stamp_ns=1_000_000_000, frame_id="camera_color_optical_frame",
        generation="a", received_monotonic=1.)
    corners = np.array([[-.04, -.03, 1.], [.04, -.03, 1.],
                        [.04, .03, 1.], [-.04, .03, 1.]])
    assert frontend.tracker.set_goal(1_000_000_000, corners, generation="a",
                                    received_monotonic=1.)
    shifted = np.zeros_like(image)
    shifted[30:90, 40:130] = image[30:90, 35:125]
    tracked = frontend.update(shifted, depth, k, np.eye(4),
        stamp_ns=1_100_000_000, frame_id="camera_color_optical_frame",
        generation="b", received_monotonic=1.1)
    assert tracked.valid
    assert tracked.root_from_anchor[0, 3] == pytest.approx(5. / 300., abs=.006)

    blank = np.zeros_like(image)
    lost = frontend.update(blank, depth, k, np.eye(4),
        stamp_ns=1_200_000_000, frame_id="camera_color_optical_frame",
        generation="c", received_monotonic=1.2)
    assert not lost.valid and lost.goal_corners_root is None
    assert lost.reason in {"insufficient_features", "insufficient_inliers"}

    returned = np.zeros_like(image)
    returned[30:90, 45:135] = image[30:90, 35:125]
    reacquired = frontend.update(returned, depth, k, np.eye(4),
        stamp_ns=1_300_000_000, frame_id="camera_color_optical_frame",
        generation="d", received_monotonic=1.3)
    assert reacquired.valid
    assert reacquired.root_from_anchor[0, 3] == pytest.approx(10. / 300., abs=.008)


def test_sparse_anchor_advances_reference_before_goal_selection():
    rng = np.random.default_rng(22)
    image = np.zeros((160, 200, 3), dtype=np.uint8)
    image[35:125, 35:165] = rng.integers(10, 245, (90, 130, 3), dtype=np.uint8)
    depth = np.ones((160, 200), dtype=np.float32)
    k = np.array([[300., 0., 100.], [0., 300., 80.], [0., 0., 1.]])
    frontend = SparseRGBDAnchor(PlacementRegionTracker(minimum_features=6))
    assert frontend.select_anchor(image, depth, k, np.eye(4), (30, 30, 170, 130),
        stamp_ns=1_000_000_000, frame_id="camera", generation="a",
        received_monotonic=1.)
    shifted = np.zeros_like(image)
    shifted[35:125, 41:171] = image[35:125, 35:165]
    for index in range(1, 4):
        current = np.zeros_like(image)
        dx = 2 * index
        current[35:125, 35 + dx:165 + dx] = image[35:125, 35:165]
        snapshot = frontend.update(current, depth, k, np.eye(4),
            stamp_ns=1_000_000_000 + index * 100_000_000,
            frame_id="camera", generation=f"g{index}",
            received_monotonic=1. + index * .1)
        assert not snapshot.valid  # no goal has been chosen yet
        assert not frontend.identity_reacquired_after_gap
    # Pixel reference and tracker history must both represent the newest frame.
    assert frontend._reference_stamp_ns == 1_300_000_000
    corners = np.array([[.02, -.02, 1.], [.08, -.02, 1.],
                        [.08, .02, 1.], [.02, .02, 1.]])
    assert frontend.tracker.set_goal(1_300_000_000, corners, generation="g3",
                                     received_monotonic=1.3)
    after_goal = np.zeros_like(image)
    after_goal[35:125, 43:173] = image[35:125, 35:165]
    tracked = frontend.update(after_goal, depth, k, np.eye(4),
        stamp_ns=1_400_000_000, frame_id="camera", generation="g4",
        received_monotonic=1.4)
    assert tracked.valid


def test_sparse_anchor_never_uses_stale_lk_reference_across_failed_frames():
    rng = np.random.default_rng(23)
    image = np.zeros((160, 200, 3), dtype=np.uint8)
    image[35:125, 35:165] = rng.integers(10, 245, (90, 130), dtype=np.uint8)[..., None]
    image[35:125, 35:165] = np.repeat(image[35:125, 35:165, :1], 3, axis=2)
    depth = np.ones((160, 200), dtype=np.float32)
    k = np.array([[300., 0., 100.], [0., 300., 80.], [0., 0., 1.]])
    tracker = PlacementRegionTracker(minimum_features=6, max_frame_gap_s=.25)
    frontend = SparseRGBDAnchor(tracker)
    assert frontend.select_anchor(image, depth, k, np.eye(4), (30, 30, 170, 130),
        stamp_ns=1_000_000_000, frame_id="camera", generation="a",
        received_monotonic=1.)
    corners = np.array([[.02, -.02, 1.], [.08, -.02, 1.],
                        [.08, .02, 1.], [.02, .02, 1.]])
    assert tracker.set_goal(1_000_000_000, corners, generation="a", received_monotonic=1.)
    blank = np.zeros_like(image)
    for index in range(1, 5):
        result = frontend.update(blank, depth, k, np.eye(4),
            stamp_ns=1_000_000_000 + index * 100_000_000,
            frame_id="camera", generation=f"lost{index}",
            received_monotonic=1. + index * .1)
        assert not result.valid
    assert frontend._reference_stamp_ns == 1_000_000_000
    returned = np.zeros_like(image)
    returned[35:125, 45:175] = image[35:125, 35:165]
    reacquired = frontend.update(returned, depth, k, np.eye(4),
        stamp_ns=1_500_000_000, frame_id="camera", generation="return",
        received_monotonic=1.5)
    assert reacquired.valid
    assert frontend.identity_reacquired_after_gap
    assert reacquired.root_from_anchor[0, 3] == pytest.approx(10. / 300., abs=.01)


def test_goal_selection_requires_exact_retained_image_generation_and_freshness():
    tracker = PlacementRegionTracker()
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    tracker.set_anchor(1_000_000_000, source, frame_id="camera", generation="a",
                       received_monotonic=1.)
    assert not tracker.set_goal(1_000_000_001, np.zeros((4, 3)), generation="a",
                                received_monotonic=1.)
    assert tracker.reason == "stale_frame"
    assert not tracker.set_goal(1_000_000_000, np.zeros((4, 3)), generation="missing",
                                received_monotonic=1.)
    assert tracker.reason == "goal_frame_unavailable"


def test_selection_id_persists_and_resolver_rebuilds_fresh_target_from_trusted_config(tmp_path):
    import hashlib
    import json
    from workshop.digital_twin.placement_region import (
        load_qualified_frame_registration,
        resolve_placement_goal,
    )

    tracker = PlacementRegionTracker()
    source = np.array([[x, y, 0.] for x, y in [(-.04, -.02), (-.02, .03),
        (.01, -.04), (.035, .025), (.05, -.005), (-.055, .01), (.015, .045)]])
    corners = np.array([[-.02, -.02, .3], [.02, -.02, .3],
                        [.02, .02, .3], [-.02, .02, .3]])
    assert tracker.set_anchor(1_000_000_000, source, frame_id="camera_color_optical_frame",
        generation="frame-a", received_monotonic=10.)
    selection_id = tracker.selection_id
    assert selection_id
    assert tracker.set_goal(1_000_000_000, corners, generation="frame-a",
        received_monotonic=10.)
    snapshot = tracker.update(1_100_000_000, range(len(source)), source + [.01, 0, 0],
        generation="frame-b", received_monotonic=10.1)
    status = {
        "state": "tracked", "valid": True, "selection_id": selection_id,
        "generation": snapshot.generation, "stamp_ns": snapshot.stamp_ns,
        "frame_id": snapshot.frame_id, "age_s": snapshot.age_s,
        "camera_from_goal_stamp_ns": snapshot.stamp_ns,
        "received_monotonic": 10.1, "fit_rms_m": snapshot.fit_rms_m,
        "camera_from_goal": pose_from_rectangle(snapshot.goal_corners_root)[0].tolist(),
        "goal_geometry": pose_from_rectangle(snapshot.goal_corners_root)[1],
        "plane_metrics": {"plane_rms_m": .002, "depth_samples": 64},
        # These camera/UI-provided flags must have no effect on config qualification.
        "physical_placement_admission": True, "camera_base_registration_qualified": True,
    }
    camera_cal = {
        "schema": "camera-arm-calibration-v1",
        "transform_direction": "base_from_camera",
        "qualified": True, "camera_frame": "camera_color_optical_frame",
        "base_from_camera": np.eye(4).tolist(),
    }
    payload = json.dumps(camera_cal).encode()
    path = tmp_path / "camera-arm.json"
    path.write_bytes(payload)
    registration = load_qualified_frame_registration(path)
    target = resolve_placement_goal(status, expected_selection_id=selection_id,
        registration=registration, received_monotonic=10.1, now_monotonic=10.12,
        now_ros_ns=1_120_000_000, max_age_s=.25)
    assert target.frame_id == "base_link"
    assert target.selection_id == selection_id
    assert target.registration_sha256 == hashlib.sha256(payload).hexdigest()
    assert target.ready_for_planning is True
    np.testing.assert_allclose(target.position_m, [0.01, 0., .3], atol=1e-8)
    assert target.area_width_m == pytest.approx(.04)
    assert target.area_height_m == pytest.approx(.04)
    np.testing.assert_allclose(target.normal_xyz, [0., 0., 1.])
    assert target.plane_rms_m == pytest.approx(.002)
    assert target.fit_uncertainty_m == pytest.approx(np.hypot(snapshot.fit_rms_m, .002))


def test_placement_goal_resolver_rejects_mismatch_staleness_loss_and_unqualified_config(tmp_path):
    import json
    from workshop.digital_twin.placement_region import (
        load_qualified_frame_registration,
        resolve_placement_goal,
    )

    calibration = {"schema": "camera-arm-calibration-v1",
        "transform_direction": "base_from_camera", "qualified": True,
        "camera_frame": "camera_color_optical_frame", "base_from_camera": np.eye(4).tolist()}
    path = tmp_path / "camera-arm.json"
    path.write_text(json.dumps(calibration))
    registration = load_qualified_frame_registration(path)
    status = {"state": "tracked", "valid": True, "selection_id": "sel-1",
        "stamp_ns": 123, "frame_id": "camera_color_optical_frame", "received_monotonic": 4.,
        "camera_from_goal_stamp_ns": 123, "camera_from_goal": np.eye(4).tolist(),
        "fit_rms_m": .001, "goal_geometry": {"width_m": .1, "height_m": .08},
        "plane_metrics": {"plane_rms_m": .002}}
    for update in (
        {"selection_id": "other"},
        {"received_monotonic": 3.},
        {"state": "invalid", "valid": False},
        {"camera_from_goal_stamp_ns": 122},
    ):
        rejected = dict(status, **update)
        with pytest.raises(ValueError):
            resolve_placement_goal(rejected, expected_selection_id="sel-1",
                registration=registration, received_monotonic=4., now_monotonic=4.1,
                now_ros_ns=123_000_000_000)
    with pytest.raises(ValueError, match="qualified"):
        resolve_placement_goal(status, expected_selection_id="sel-1",
            registration=None, received_monotonic=4., now_monotonic=4.1,
            now_ros_ns=123_000_000_000)
    with pytest.raises(ValueError, match="stale"):
        resolve_placement_goal(status, expected_selection_id="sel-1",
            registration=registration, received_monotonic=4., now_monotonic=4.1,
            now_ros_ns=700_000_000)
    calibration["qualified"] = False
    path.write_text(json.dumps(calibration))
    with pytest.raises(ValueError, match="qualified"):
        load_qualified_frame_registration(path)


def test_camera_frame_summary_is_json_safe_and_keeps_exact_calibration_metadata():
    import json
    from workshop.digital_twin.placement_region import placement_camera_frame_summary

    summary = placement_camera_frame_summary({
        "generation": "g1", "stamp_ns": 42, "received_monotonic": 1.2,
        "frame_id": "camera", "root_frame": "world", "image_path": "/tmp/frame.jpg",
        "image_shape": [480, 848], "root_from_camera": np.eye(4),
        "base_from_camera": None,
        "camera_matrix": np.array([[600., 0., 424.], [0., 600., 240.], [0., 0., 1.]])})
    assert json.loads(json.dumps(summary)) == {
        "generation": "g1", "stamp_ns": 42, "received_monotonic": 1.2,
        "frame_id": "camera", "root_frame": "world", "image_path": "/tmp/frame.jpg",
        "image_shape": [480, 848], "root_from_camera": np.eye(4).tolist(),
        "base_from_camera": None,
        "camera_matrix": [[600., 0., 424.], [0., 600., 240.], [0., 0., 1.]]}


def test_waiting_for_anchor_does_not_erase_selection_failure_diagnostics():
    from workshop.digital_twin.placement_region import waiting_for_anchor_status

    rejected = {"state": "invalid", "valid": False,
        "reason": "selection_rejected:insufficient_features", "generation": "pinned"}
    assert waiting_for_anchor_status(rejected, generation="new", stamp_ns=4,
        frame_id="camera") == rejected
    waiting = waiting_for_anchor_status({}, generation="new", stamp_ns=4,
        frame_id="camera")
    assert waiting["state"] == "waiting_for_anchor"
    assert waiting["reason"] == "no_feeder_anchor_selected"
    assert waiting["stamp_ns"] == 4
