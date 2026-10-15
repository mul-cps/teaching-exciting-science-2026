"""Offline contracts for tracked-object scene updates and grasp validation."""

import sys
from pathlib import Path

import numpy as np
import pytest


REPOSITORY = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = REPOSITORY / "ros2/src/so101_station_scene"
sys.path.insert(0, str(PACKAGE_SOURCE))

from so101_station_scene.scene_geometry import (  # noqa: E402
    LiveObjectRegistry,
    object_collision_id,
    table_patches_around_keepout,
    transform_box_pose,
    validate_box,
)


def test_scene_ids_are_stable_and_confined_to_owned_prefix():
    assert object_collision_id("red-17") == "colorbrick_red-17"
    assert object_collision_id("../other-owner") == "colorbrick_..%2Fother-owner"
    with pytest.raises(ValueError, match="stable track id"):
        object_collision_id("")


@pytest.mark.parametrize("size", [
    (0.0, 0.02, 0.02),
    (0.02, float("nan"), 0.02),
    (0.02, 0.02, 2.0),
    (0.02, 0.02),
])
def test_box_dimensions_reject_invalid_or_implausible_extents(size):
    with pytest.raises(ValueError):
        validate_box(size, min_extent_m=0.002, max_extent_m=0.30)


def test_scene_box_minimum_accepts_thin_detected_parts_at_two_millimeters():
    assert validate_box((0.002, 0.02, 0.03), min_extent_m=0.002,
                        max_extent_m=0.30) == pytest.approx((0.002, 0.02, 0.03))
    with pytest.raises(ValueError):
        validate_box((0.0019, 0.02, 0.03), min_extent_m=0.002,
                     max_extent_m=0.30)


def test_box_pose_transforms_center_and_orientation_with_full_se3():
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [1.0, 2.0, 3.0]
    center, orientation = transform_box_pose(
        transform, (0.5, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))
    assert center == pytest.approx((1.0, 2.5, 3.2))
    assert orientation == pytest.approx((0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)))


def test_box_transform_rejects_nonrigid_and_zero_quaternion_inputs():
    with pytest.raises(ValueError, match=r"proper rigid SE\(3\)"):
        transform_box_pose(np.diag([1.0, 1.0, 1.0, 2.0]), (0, 0, 0), (0, 0, 0, 1))
    with pytest.raises(ValueError, match="zero length"):
        transform_box_pose(np.eye(4), (0, 0, 0), (0, 0, 0, 0))


def test_registry_replaces_track_and_expires_only_its_owned_collision_ids():
    registry = LiveObjectRegistry(ttl_s=0.75, min_extent_m=0.002, max_extent_m=0.30)
    first = registry.update("brick-a", (0, 0, 0.1), (0.04, 0.03, 0.02),
                            (0, 0, 0, 1), now_mono_s=1.0)
    assert first.collision_id == "colorbrick_brick-a"
    moved = registry.update("brick-a", (0.01, 0, 0.1), (0.04, 0.03, 0.02),
                            (0, 0, 0, 1), now_mono_s=1.2)
    assert moved.center_m == pytest.approx((0.01, 0, 0.1))
    assert registry.expire(now_mono_s=1.95) == ["colorbrick_brick-a"]
    assert registry.expire(now_mono_s=2.0) == []


def test_detection_freshness_rejects_zero_and_old_stamps():
    from so101_station_scene.scene_geometry import stamp_is_fresh

    assert stamp_is_fresh(now_s=10.0, stamp_s=9.6, max_age_s=0.5)
    assert not stamp_is_fresh(now_s=10.0, stamp_s=9.4, max_age_s=0.5)
    assert not stamp_is_fresh(now_s=10.0, stamp_s=0.0, max_age_s=0.5)
    assert not stamp_is_fresh(now_s=10.0, stamp_s=10.2, max_age_s=0.5)


def test_table_surface_is_split_around_robot_base_keepout_without_overlap():
    patches = table_patches_around_keepout(
        center_xy_m=(0, 0), size_xy_m=(1, 0.8),
        keepout_center_xy_m=(0.05, -0.02), keepout_size_xy_m=(0.2, 0.18))
    assert len(patches) == 4
    for x, y, width, height in patches:
        assert not (x + width / 2 > -0.05 + 1e-9 and x - width / 2 < 0.15 - 1e-9
                    and y + height / 2 > -0.11 + 1e-9 and y - height / 2 < 0.07 - 1e-9)


def test_grasp_validation_is_plan_only_by_contract():
    source = (PACKAGE_SOURCE / "so101_station_scene" / "grasp_validator.py").read_text()
    assert "GetPositionIK" in source
    assert "GetStateValidity" in source
    assert "ExecuteTrajectory" not in source
    assert "ExecuteTaskSolution" not in source
    assert "ik_validated" in source


def test_scene_updater_uses_incremental_world_diff_and_never_edits_attachments():
    source = (PACKAGE_SOURCE / "so101_station_scene" / "scene_updater.py").read_text()
    assert "request.scene.is_diff = True" in source
    assert "request.scene.robot_state.is_diff = True" in source
    assert "request.scene.world.collision_objects = queued" in source
    assert "CollisionObject.REMOVE" in source
    assert "AttachedCollisionObject" not in source
    assert "registry.expire" in source


def test_moveit_octomap_config_is_conservative_and_sensor_specific():
    import yaml

    config = yaml.safe_load((REPOSITORY / "ros2/config/sensors_3d.yaml").read_text())
    assert config["sensors"] == ["resident_pointcloud"]
    assert config["octomap_frame"] == "base_link"
    assert config["octomap_resolution"] == pytest.approx(0.02)
    sensor = config["resident_pointcloud"]
    assert sensor["point_cloud_topic"] == "/camera/depth/color/points"
    assert sensor["sensor_plugin"] == "occupancy_map_monitor/PointCloudOctomapUpdater"
    assert sensor["max_range"] == pytest.approx(1.2)
    assert sensor["max_update_rate"] <= 5.0
    assert "min_range" not in sensor  # not a PointCloudOctomapUpdater parameter


def test_scene_config_uses_current_z_up_board_and_unqualified_default():
    import yaml

    path = PACKAGE_SOURCE / "config/scene_updater.yaml"
    config = yaml.safe_load(path.read_text())["so101_scene_updater"]["ros__parameters"]
    assert config["board_size_m"] == pytest.approx([0.175, 0.125, 0.004])
    assert config["board_center_m"] == pytest.approx([0.0875, -0.0625, -0.002])
    assert config["frames_qualified"] is False
    assert config["table_collision_enabled"] is False


def test_base_vertical_variant_preserves_transformed_position_and_reports_orientation_tradeoff():
    pytest.importorskip("rclpy")
    from so101_station_scene.grasp_validator import base_vertical_variant

    angle = np.deg2rad(5.0)
    transform = np.eye(4)
    transform[:3, :3] = [[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]]
    transform[:3, 3] = [0.1, -0.2, 0.03]
    position, orientation, approach_error, closing_error = base_vertical_variant(
        transform, (0.2, 0.3, 0.04), (0, 0, 0, 1))
    assert position == pytest.approx((0.2 * np.cos(angle) + 0.04 * np.sin(angle) + 0.1,
                                      0.1,
                                      -0.2 * np.sin(angle) + 0.04 * np.cos(angle) + 0.03))
    rotation = __import__("so101_station_scene.scene_geometry", fromlist=["quaternion_matrix_xyzw"]).quaternion_matrix_xyzw(orientation)
    assert rotation[:, 2] == pytest.approx((0, 0, -1))
    assert approach_error == pytest.approx(175.0, abs=1e-5)
    assert closing_error == pytest.approx(5.0, abs=1e-5)


def test_validator_throttles_batches_not_candidates_within_batch():
    pytest.importorskip("rclpy")
    from so101_station_scene.grasp_validator import should_wait_for_batch

    assert not should_wait_for_batch(active=True, pending=True, now=1.1,
                                     last_batch_start=1.0, min_period=1.0)
    assert should_wait_for_batch(active=False, pending=True, now=1.1,
                                 last_batch_start=1.0, min_period=1.0)
    assert not should_wait_for_batch(active=False, pending=True, now=2.0,
                                     last_batch_start=1.0, min_period=1.0)


def test_async_grasp_results_require_live_request_and_fresh_source_and_joint_state():
    pytest.importorskip("rclpy")
    from so101_station_scene.grasp_validator import (
        async_result_is_current,
        async_request_expired,
    )

    args = dict(token=7, active_token=7, now_mono=10.2, deadline_mono=10.45,
                received_mono=10.0,
                max_age_s=0.5, now_ros=20.2, source_stamp_s=20.0,
                joint_stamp_s=20.0, joint_max_age_s=0.5)
    assert async_result_is_current(**args)
    assert not async_result_is_current(**{**args, "now_mono": 10.46})
    assert not async_result_is_current(**{**args, "now_mono": 10.51})
    assert not async_result_is_current(**{**args, "now_ros": 20.51})
    assert not async_result_is_current(**{**args, "joint_stamp_s": 19.6})
    assert not async_result_is_current(**{**args, "active_token": 8})
    assert not async_request_expired(now=10.0, deadline=10.1)
    assert async_request_expired(now=10.1, deadline=10.1)
