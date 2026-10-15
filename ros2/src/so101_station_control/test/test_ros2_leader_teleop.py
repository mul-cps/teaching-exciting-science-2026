"""Offline contract checks for the calibrated leader reader and ROS representation."""
import math

import pytest

from so101_station_control.leader_reader import JOINTS, joint_state_positions, normalized_position


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
