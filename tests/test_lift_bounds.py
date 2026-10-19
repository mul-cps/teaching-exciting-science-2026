"""Evidence-pinned lift envelope; all tests are hardware-free."""
import ast
import math
from types import SimpleNamespace
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from workshop.station.arm import LeRobotArm
from workshop.station.ros_trajectory import (
    ARM_JOINTS, POSITION_LIMITS_RAD, _position_limits, prepare_trajectory,
)

ROOT = Path(__file__).resolve().parents[1]
LIFT = "shoulder_lift"
RAW_MIN, RAW_MAX, MID = 1063, 3240, 2151.5


def radians(raw):
    return (raw - MID) * 2 * math.pi / 4095


def owner_limits():
    limits = dict(POSITION_LIMITS_RAD)
    limits[LIFT] = (radians(RAW_MIN), radians(RAW_MAX))
    return limits


def prepare(values, limits):
    positions = [[0., q, 0., 0., 0.] for q in values]
    return prepare_trajectory(
        ARM_JOINTS,
        [{"time": i * 10., "positions": p} for i, p in enumerate(positions)],
        start_radians=dict(zip(ARM_JOINTS, positions[0])),
        max_velocity=1., max_acceleration=2., max_jerk=2.,
        position_limits=limits,
    )


def test_lift_defaults_match_pinned_official_urdf_and_other_axes_unchanged():
    model = ET.parse(ROOT / "workshop/digital_twin/assets/so101_new_calib.urdf")
    bound = model.find(".//joint[@name='shoulder_lift']/limit")
    assert POSITION_LIMITS_RAD[LIFT] == (float(bound.get("lower")), float(bound.get("upper")))
    config = yaml.safe_load((ROOT / "ros2/config/rig01_moveit_limits.yaml").read_text())["joint_limits"]
    assert (config[LIFT]["min_position"], config[LIFT]["max_position"]) == (-1.74533, 1.74533)
    for joint in ("shoulder_pan", "wrist_roll"):
        assert POSITION_LIMITS_RAD[joint] == (-math.pi / 2, math.pi / 2)
        assert (config[joint]["min_position"], config[joint]["max_position"]) == (-1.57079632679, 1.57079632679)
    assert POSITION_LIMITS_RAD["elbow_flex"] == (-1.69, 1.69)
    assert (config["elbow_flex"]["min_position"], config["elbow_flex"]["max_position"]) == (-1.69, 1.69)
    # Only position limits differ from the inherited speed/acceleration/jerk caps.
    assert {k: v for k, v in config[LIFT].items() if k not in ("min_position", "max_position")} == {
        k: v for k, v in config["shoulder_pan"].items() if k not in ("min_position", "max_position")}


def test_live_raw1095_is_admitted_but_old_ninety_degree_policy_rejects():
    q = radians(1095)
    assert math.degrees(q) == pytest.approx(-92.87912087912088)
    assert prepare([q, q], owner_limits()).samples[0].positions[1] == q
    old = owner_limits()
    old[LIFT] = (-math.pi / 2, math.pi / 2)
    with pytest.raises(ValueError, match="position limit exceeded: shoulder_lift"):
        prepare([q, q], old)


@pytest.mark.parametrize("raw", [1063, 3240])
def test_calibrated_firmware_raw_edges_remain_admitted(raw):
    q = radians(raw)
    assert prepare([q, q], owner_limits()).samples[-1].positions[1] == pytest.approx(q)


@pytest.mark.parametrize("raw", [1062, 3241])
def test_one_count_outside_owner_intersection_rejected_at_knots(raw):
    with pytest.raises(ValueError, match="position limit exceeded: shoulder_lift"):
        prepare([radians(raw), radians(raw)], owner_limits())


def test_owner_intersection_is_not_widened_to_urdf_model_envelope():
    actual = _position_limits(owner_limits())[LIFT]
    assert actual == (radians(1063), radians(3240))
    assert math.degrees(actual[1]) == pytest.approx(95.6923076923077)
    assert actual[1] < POSITION_LIMITS_RAD[LIFT][1]


def test_analytic_extremum_rejects_overshoot_between_admitted_knots():
    high = radians(RAW_MAX)
    # Repeated interior upper-bound knots produce an above-bound extremum.
    with pytest.raises(ValueError, match="interpolated position limit exceeded: shoulder_lift"):
        prepare([high - .1, high, high, high - .1], owner_limits())


def test_admission_and_analytic_check_remain_in_prepare_path():
    # Constants-only correction must retain owner narrowing and analytic extrema.
    tree = ast.parse((ROOT / "workshop/station/ros_trajectory.py").read_text())
    funcs = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
    calls = {n.func.id for n in ast.walk(funcs["prepare_trajectory"])
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert {"_position_limits", "_check_limits"} <= calls
    assert "_poly_roots" in {n.func.id for n in ast.walk(funcs["_check_limits"])
                             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}


@pytest.mark.parametrize("configured_degrees, expected_raw", [(90., (1128, 3175)),
                                                              (math.degrees(1.74533), (1063, 3240))])
def test_existing_owner_calibration_intersection(configured_degrees, expected_raw):
    # Call the production pure bound calculator without constructing a bus/session.
    ranges = {"shoulder_pan": (745, 3470), LIFT: (1063, 3240),
              "elbow_flex": (848, 3055), "wrist_flex": (820, 3171),
              "wrist_roll": (0, 4095), "gripper": (1872, 3366)}
    calibration = {joint: {"range_min": lo, "range_max": hi}
                   for joint, (lo, hi) in ranges.items()}
    settings = {"hz": 100, "limits": {joint: {"min": 0. if joint == "gripper" else -90.,
                                             "max": 100. if joint == "gripper" else 90.,
                                             "speed": 10., "acceleration": 20.}
                                    for joint in ranges}}
    settings["limits"][LIFT].update(min=-configured_degrees, max=configured_degrees)
    owner = SimpleNamespace(calibration=calibration, settings=settings)
    LeRobotArm._bounds(owner)
    low, high = owner.bounds[LIFT]
    # Firmware equals the pinned calibration endpoints; reproduce its final raw
    # intersection after the production calibrated/configured bound calculation.
    raw = (max(RAW_MIN, math.ceil(low * 4095 / 360 + MID)),
           min(RAW_MAX, math.floor(high * 4095 / 360 + MID)))
    assert raw == expected_raw
    assert (raw[0] <= 1095 <= raw[1]) == (configured_degrees > 90.)
    if configured_degrees > 90.:
        assert tuple(map(math.radians, owner.bounds[LIFT])) == pytest.approx(owner_limits()[LIFT])
