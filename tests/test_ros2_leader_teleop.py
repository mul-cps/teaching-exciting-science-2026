"""Offline contract checks for the calibrated leader reader and ROS representation."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control"))
from so101_station_control.leader_reader import (
    JOINTS, joint_state_positions, matched_pose, normalized_position, validate_pose_packet,
)


def test_leader_calibration_mapping_matches_capture_units():
    calibration = {"range_min": 500, "range_max": 3500}
    assert normalized_position(500, calibration, True) == pytest.approx(0.0)
    assert normalized_position(3500, calibration, True) == pytest.approx(100.0)
    assert normalized_position(2000, calibration, False) == pytest.approx(0.0)
    assert normalized_position(2000 + 4095 / 4, calibration, False) == pytest.approx(90.0)


def test_joint_state_uses_radians_and_visual_only_gripper_proxy():
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["shoulder_pan"] = 90.0
    pose["gripper"] = 50.0
    positions = joint_state_positions(pose)
    assert len(positions) == 6
    assert positions[0] == pytest.approx(math.pi / 2)
    assert positions[-1] == pytest.approx(math.radians(45.0))


@pytest.mark.parametrize("gripper", [-0.1, 100.1, math.nan])
def test_gripper_visual_proxy_rejects_out_of_range_values(gripper):
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["gripper"] = gripper
    with pytest.raises(ValueError):
        joint_state_positions(pose)


def test_normalized_position_rejects_invalid_calibration():
    with pytest.raises(ValueError):
        normalized_position(1, {"range_min": 1, "range_max": 1}, False)


def test_pose_contract_and_matched_pose_mapping():
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["gripper"] = 45.0
    packet = {"schema": 1, "stamp_s": 10.0, "sequence": 2,
              "units": {"body": "degrees", "gripper": "percent"}, "pose": pose}
    validated = validate_pose_packet(packet, 10.05)
    leader_start = dict.fromkeys(JOINTS, 0.0)
    follower_start = dict.fromkeys(JOINTS, 10.0)
    follower_start["gripper"] = 50.0
    mapped = matched_pose(validated, leader_start, follower_start)
    assert mapped["shoulder_pan"] == pytest.approx(10.0)
    assert mapped["gripper"] == pytest.approx(95.0)
    with pytest.raises(ValueError, match="freshness"):
        validate_pose_packet(packet, 10.16)
