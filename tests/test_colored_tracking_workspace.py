"""Synthetic full-camera observation versus pickup-admission contract."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2/src/so101_station_objects"))

from so101_station_objects.objects import (  # noqa: E402
    DEFAULT_TABLE_ROI,
    StableObjectTracker,
    analyze_rgbd,
)


INTRINSICS = {"fx": 500.0, "fy": 500.0, "ppx": 320.0, "ppy": 240.0}
DISTORTION = np.zeros(5)
BOARD_FROM_CAMERA = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 0.8],
    [0.0, 0.0, 0.0, 1.0],
])
SUPPORT_PLANE = {"normal": np.array([0.0, 0.0, 1.0]), "d": 0.0}


def rgbd_with_red_brick(position, *, frame_size=(480, 640), pixel_size=32):
    """Render a top-facing red surface at board XYZ into a synthetic depth image."""
    height, width = frame_size
    rgb = np.full((height, width, 3), 128, dtype=np.uint8)
    depth = np.full((height, width), 800, dtype=np.uint16)
    x, y, z = position
    camera_z = 0.8 - z
    u = int(round(INTRINSICS["fx"] * x / camera_z + INTRINSICS["ppx"]))
    v = int(round(-INTRINSICS["fy"] * y / camera_z + INTRINSICS["ppy"]))
    half = pixel_size // 2
    x0, x1 = max(0, u - half), min(width, u + half)
    y0, y1 = max(0, v - half), min(height, v + half)
    if x1 - x0 < pixel_size // 2 or y1 - y0 < pixel_size // 2:
        return rgb, depth
    rgb[y0:y1, x0:x1] = [255, 0, 0]
    depth[y0:y1, x0:x1] = int(round(camera_z * 1000))
    return rgb, depth


def observe(position, *, pixel_size=32):
    rgb, depth = rgbd_with_red_brick(position, pixel_size=pixel_size)
    return analyze_rgbd(
        rgb, depth, INTRINSICS, DISTORTION, 0.001, BOARD_FROM_CAMERA,
        support_plane=SUPPORT_PLANE,
    )


def test_rgbd_tracks_across_old_xyz_bounds_without_grasp_admission():
    # Cross the former x, y and z crop limits while remaining visible.
    report = observe((0.27, 0.19, 0.12))
    assert len(report["objects"]) == 1
    obj = report["objects"][0]
    assert obj["color"] == "red"
    assert obj["pickup_eligible"] is False
    assert obj["grasps"] == []
    assert obj["observation_geometry"] == "visible_surface_centroid"
    assert obj["object_center_qualified"] is False
    assert obj["dimensions_semantics"] == "visible_surface_extent_only"
    assert obj["surface_centroid_spread_m"].shape == (3,)
    np.testing.assert_allclose(obj["position"], [0.27, 0.19, 0.12], atol=0.015)


def test_full_image_observation_has_no_fixed_board_z_bound_below_table():
    obj = observe((0.10, 0.07, -0.10))["objects"][0]
    assert obj["pickup_eligible"] is False
    assert obj["grasps"] == []
    np.testing.assert_allclose(obj["surface_centroid"], [0.10, 0.07, -0.10], atol=0.015)


def test_optional_commissioned_observation_roi_remains_configurable():
    rgb, depth = rgbd_with_red_brick((0.27, 0.19, 0.12))
    report = analyze_rgbd(
        rgb, depth, INTRINSICS, DISTORTION, 0.001, BOARD_FROM_CAMERA,
        support_plane=SUPPORT_PLANE,
        observation_roi=(-0.1, 0.2, -0.1, 0.2, -0.05, 0.08),
    )
    assert report["objects"] == []


def test_pickup_roi_is_independent_and_restored_when_brick_returns():
    report = observe((0.10, 0.07, 0.02))
    assert len(report["objects"]) == 1
    obj = report["objects"][0]
    assert obj["pickup_eligible"] is True
    assert obj["observation_geometry"] == "support_plane_box_estimate"
    assert obj["object_center_qualified"] is False
    assert obj["dimensions_semantics"] == "support_plane_box_extent_estimate"
    assert obj["grasps"]

    outside = observe((DEFAULT_TABLE_ROI[1] + 0.04, 0.07, 0.02))["objects"][0]
    assert outside["pickup_eligible"] is False
    assert outside["grasps"] == []


def test_component_straddling_pickup_roi_is_observed_but_never_clipped_into_candidate():
    # The component's visible footprint crosses x_max although its centroid is
    # inside. Candidate fitting must reject the whole component, not fit only
    # the in-ROI pixels and report a fabricated smaller brick.
    report = observe((0.205, 0.07, 0.02), pixel_size=38)
    assert len(report["objects"]) == 1
    obj = report["objects"][0]
    assert obj["pickup_eligible"] is False
    assert obj["grasps"] == []
    assert obj["observation_geometry"] == "visible_surface_centroid"
    np.testing.assert_allclose(obj["surface_centroid"], [0.205, 0.07, 0.02], atol=0.015)


def test_motion_crossing_old_crop_retains_id_and_expiry_reacquires_new_id():
    tracker = StableObjectTracker(max_distance_m=0.06, max_age_s=1.0)
    positions = [
        np.array([0.16, 0.08, 0.02]),
        np.array([0.195, 0.10, 0.04]),
        np.array([0.23, 0.12, 0.06]),
        np.array([0.265, 0.14, 0.08]),
        np.array([0.23, 0.12, 0.06]),
        np.array([0.195, 0.10, 0.04]),
        np.array([0.16, 0.08, 0.02]),
    ]
    observed = []
    for index, position in enumerate(positions):
        report = observe(position)
        assert len(report["objects"]) == 1
        detected = report["objects"][0]
        if not (DEFAULT_TABLE_ROI[0] <= position[0] <= DEFAULT_TABLE_ROI[1]
                and DEFAULT_TABLE_ROI[2] <= position[1] <= DEFAULT_TABLE_ROI[3]
                and position[2] <= 0.025):
            assert detected["pickup_eligible"] is False
            assert detected["grasps"] == []
        records = tracker.update([detected], 10.0 + index * 0.2)
        assert len(records) == 1
        observed.append(records[0])
    assert {item["track_id"] for item in observed} == {observed[0]["track_id"]}
    # A supported object regains candidate geometry on return to the pickup area.
    assert observed[-1]["pickup_eligible"] is True

    assert tracker.update([], 12.4) == []
    expired = tracker.expired_tracks
    assert len(expired) == 1
    assert expired[0]["track_id"] == f"red-{observed[0]['track_id']:04d}"
    assert expired[0]["reason"] == "not_observed_past_max_age"

    reacquired = tracker.update([observe(positions[-1])["objects"][0]], 12.6)
    assert len(reacquired) == 1
    assert reacquired[0]["track_id"] != observed[0]["track_id"]


def test_owned_identity_never_expires_or_substitutes_same_color_brick():
    tracker = StableObjectTracker(max_distance_m=0.06, max_age_s=0.2)
    original = tracker.update([{"color": "red", "position": np.array([0.1, 0.1, 0.02])}], 1.0)[0]
    tracker.ownership_status({"track_id": f"red-{original['track_id']:04d}",
                              "state": "locked"})
    for stamp in (2.0, 3.0):
        assert tracker.update([], stamp) == []
        assert tracker.expired_tracks == []
    # A same-color object at another visible location cannot inherit ownership.
    visible = tracker.update([{"color": "red", "position": np.array([0.4, 0.1, 0.02])}], 3.2)
    assert len(visible) == 0
    assert original["track_id"] in tracker.owned


def test_expiry_tolerates_legacy_track_without_surface_centroid():
    tracker = StableObjectTracker(max_age_s=0.2)
    first = tracker.update([{"color": "yellow", "position": np.array([0.1, 0.1, 0.02])}],
                           1.0)[0]
    del tracker.tracks[first["track_id"]]["surface_centroid"]
    assert tracker.update([], 1.3) == []
    assert tracker.expired_tracks[0]["last_surface_centroid_world"] == []


def test_velocity_prediction_retains_identity_for_fast_hand_motion_at_five_hz():
    tracker = StableObjectTracker(max_distance_m=0.06, max_age_s=1.0)
    positions = [np.array([0.08, 0.06, 0.02]),
                 np.array([0.17, 0.06, 0.02]),
                 np.array([0.26, 0.06, 0.02])]
    ids = []
    for index, position in enumerate(positions):
        record = tracker.update([{"color": "green", "position": position}],
                                20.0 + index * 0.2)[0]
        ids.append(record["track_id"])
    assert ids[0] == ids[1] == ids[2]


def test_near_tied_same_color_association_is_withheld_until_unambiguous():
    tracker = StableObjectTracker(max_distance_m=0.06)
    first = tracker.update([
        {"color": "blue", "position": np.array([0.12, 0.1, 0.02])},
        {"color": "blue", "position": np.array([0.18, 0.1, 0.02])},
    ], 1.0)
    original_ids = {record["track_id"] for record in first}
    assert len(original_ids) == 2
    assert tracker.update([
        {"color": "blue", "position": np.array([0.149, 0.1, 0.02])},
        {"color": "blue", "position": np.array([0.151, 0.1, 0.02])},
    ], 1.2) == []
    separated = tracker.update([
        {"color": "blue", "position": np.array([0.10, 0.1, 0.02])},
        {"color": "blue", "position": np.array([0.20, 0.1, 0.02])},
    ], 1.4)
    assert {record["track_id"] for record in separated} == original_ids
