import math
import sys
from types import SimpleNamespace
from pathlib import Path

import pytest

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "tools"))

from ros_plan_current import (  # noqa: E402
    ARM_JOINTS,
    ALL_JOINTS,
    _joint_state_map,
    is_calibration_endpoint_result,
    max_deviation_from_linear_knots,
    parse_joint_offsets,
    prepare_with_retiming,
    scale_points,
)


def test_calibration_endpoint_result_requires_flag_code_and_exact_prefix():
    prefix = "calibration_measured_endpoint_held: measured endpoint held"
    assert is_calibration_endpoint_result(True, -5, prefix)
    assert not is_calibration_endpoint_result(False, -5, prefix)
    assert not is_calibration_endpoint_result(True, 0, prefix)
    assert not is_calibration_endpoint_result(True, -5, "trajectory target reached")


def test_joint_offset_defaults_to_small_pan_only_and_accepts_partial_json():
    assert parse_joint_offsets(None) == {
        "shoulder_pan": 0.05, "shoulder_lift": 0.0, "elbow_flex": 0.0,
        "wrist_flex": 0.0, "wrist_roll": 0.0,
    }
    parsed = parse_joint_offsets('{"elbow_flex":-0.02}')
    assert parsed["elbow_flex"] == -0.02
    assert parsed["shoulder_pan"] == 0.0


@pytest.mark.parametrize("value", ["[]", "{bad", '{"gripper":0.1}',
                                    '{"shoulder_pan":NaN}', '{"wrist_roll":0.3}'])
def test_joint_offset_parser_rejects_malformed_unsafe_values(value):
    with pytest.raises(ValueError):
        parse_joint_offsets(value)


def test_uniform_time_scaling_preserves_positions_and_scales_derivatives():
    point = {"time": 2.0, "positions": [0.1] * 5,
             "velocities": [0.2] * 5, "accelerations": [0.4] * 5}
    scaled = scale_points([point], 2.0)[0]
    assert scaled["time"] == 4.0
    assert scaled["positions"] == point["positions"]
    assert scaled["velocities"] == pytest.approx([0.1] * 5)
    assert scaled["accelerations"] == pytest.approx([0.1] * 5)


def test_measured_start_requires_fresh_complete_five_axis_and_gripper_state():
    message = SimpleNamespace(
        name=list(ALL_JOINTS), position=[0.1] * len(ALL_JOINTS),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)))
    assert _joint_state_map(message, 100.4)["gripper"] == pytest.approx(0.1)
    with pytest.raises(ValueError, match="older"):
        _joint_state_map(message, 100.6)
    message.name.remove("gripper")
    message.position.pop()
    with pytest.raises(ValueError, match="missing joints"):
        _joint_state_map(message, 100.0)


def test_dynamic_limit_failure_is_retimed_without_changing_path_knots():
    points = [
        {"time": 0.0, "positions": [0.0] * 5},
        {"time": 0.1, "positions": [0.05, 0.0, 0.0, 0.0, 0.0]},
    ]
    factor, effective, prepared = prepare_with_retiming(
        ARM_JOINTS, points,
        start_radians={joint: 0.0 for joint in ARM_JOINTS},
        velocity_scale=0.25, acceleration_scale=0.25)
    assert factor > 1.0
    assert [point["positions"] for point in effective] == [point["positions"] for point in points]
    assert effective[-1]["time"] == pytest.approx(points[-1]["time"] * factor)
    assert math.isfinite(max_deviation_from_linear_knots(prepared, points, factor))
