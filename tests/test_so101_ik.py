"""Real optional backend tests: no connected motors or camera required."""
import hashlib
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin.ik import ASSETS, ARM_JOINTS, SO101IK

CAL = {name: {"range_min": lo, "range_max": hi} for name, lo, hi in (
    ("shoulder_pan", 745, 3470), ("shoulder_lift", 1063, 3240),
    ("elbow_flex", 848, 3055), ("wrist_flex", 820, 3171),
    ("wrist_roll", 0, 4095), ("gripper", 1872, 3366))}
RAW = {name: int(round((c["range_min"] + c["range_max"]) / 2)) for name, c in CAL.items()}
RAW["gripper"] = 2170


def offset(opening):
    matrix = np.eye(4)
    matrix[:3, 3] = [.005, opening * .0002, -.003]
    return matrix


@pytest.fixture
def ik():
    if not SO101IK.available():
        pytest.skip("optional LeRobot/Placo backend unavailable")
    with SO101IK(CAL, offset) as solver:
        yield solver


@pytest.mark.parametrize("perturb", [25, -40, 80])
def test_reachable_round_trip_and_fixed_grip(ik, perturb):
    goal = dict(RAW)
    goal.update(shoulder_pan=2250, shoulder_lift=2070, elbow_flex=1850, wrist_flex=2020)
    target = ik.forward_raw(goal)
    seed = {name: value + perturb if name in ARM_JOINTS else value for name, value in goal.items()}
    result = ik.solve_pinch(target, seed)
    assert result.success, result
    assert result.raw["gripper"] == goal["gripper"]
    assert result.position_error_m <= .002
    assert np.linalg.norm(ik.forward_raw(result.raw)[:3, 3] - target[:3, 3]) <= .002


def test_unreachable(ik):
    target = ik.forward_raw(RAW)
    target[:3, 3] = [5, 5, 5]
    result = ik.solve_pinch(target, RAW, max_iterations=10)
    assert not result.success
    assert result.raw is None


@pytest.mark.parametrize("bad", [np.full((4, 4), np.nan), np.zeros((4, 4)), np.eye(3), np.diag([2, 1, 1, 1])])
def test_invalid_target(ik, bad):
    with pytest.raises(ValueError):
        ik.solve_pinch(bad, RAW)


def test_calibration_and_urdf_limits(ik):
    invalid = dict(RAW, elbow_flex=3056)
    with pytest.raises(ValueError):
        ik.forward_raw(invalid)
    # Within EEPROM range, but outside the official URDF wrist-roll limit.
    with pytest.raises(ValueError):
        ik.forward_raw(dict(RAW, wrist_roll=4095))
    with pytest.raises(ValueError):
        ik.forward_raw(dict(RAW, shoulder_pan=float("nan")))


def test_opening_cache_and_resource_immutability(ik):
    original = ASSETS / "so101_new_calib.urdf"
    digest = hashlib.sha256(original.read_bytes()).hexdigest()
    first = ik.forward_raw(RAW)
    backend = ik._backend
    folder = Path(ik._temp.name)
    ik.forward_raw(RAW)
    assert ik._backend is backend
    opened = dict(RAW, gripper=2400)
    second = ik.forward_raw(opened)
    assert ik._backend is not backend
    assert not folder.exists()
    assert np.linalg.norm(first[:3, 3] - second[:3, 3]) > .001
    assert hashlib.sha256(original.read_bytes()).hexdigest() == digest
    new_folder = Path(ik._temp.name)
    ik.close()
    assert not new_folder.exists()
    with pytest.raises(RuntimeError):
        ik.forward_raw(RAW)


def test_benchmark(ik):
    result = ik.benchmark(RAW, samples=10)
    assert result["ik_success"]
    assert 0 < result["fk_median_ms"] < 1000
    print(result)


def test_constructor_validation():
    with pytest.raises(ValueError):
        SO101IK(CAL, None)
    bad = dict(CAL, gripper={"range_min": 2, "range_max": 2})
    with pytest.raises(ValueError):
        SO101IK(bad, offset)


def test_reachable_orientation_and_masked_gripper(ik):
    target = ik.forward_raw(RAW)
    seed = dict(RAW, shoulder_pan=RAW["shoulder_pan"] + 40)
    result = ik.solve_pinch(target, seed, orientation_weight=1., orientation_tolerance_rad=.01)
    assert result.success, result
    assert result.orientation_error_rad <= .01
    opening = 100 * (RAW["gripper"] - 1872) / (3366 - 1872)
    assert np.isclose(ik._backend.robot.get_joint("gripper"), np.deg2rad(-10 + 1.1 * opening))


def test_post_quantization_residual_is_required(ik):
    # This reachable fractional-angle target cannot be represented by this seed's
    # rounded raw motors at nanometre precision; never emit nominal-FK success.
    q, opening = ik._raw_degrees(RAW)
    q[0] += .031
    target = np.asarray(ik._prepare(opening).forward_kinematics(q)).copy()
    result = ik.solve_pinch(target, RAW, position_tolerance_m=1e-10, max_iterations=8, orientation_weight=1., orientation_tolerance_rad=1e-8)
    assert not result.success
    assert result.raw is None


def test_fixed_opening_telemetry_cache_and_provenance(ik):
    first = ik.forward_raw(RAW, opening_raw=RAW["gripper"])
    backend = ik._backend
    for delta in (-1, 1, 0):
        measured = dict(RAW, gripper=RAW["gripper"] + delta)
        np.testing.assert_allclose(ik.forward_raw(measured, opening_raw=RAW["gripper"]), first)
        assert ik._backend is backend
        provenance = ik.forward_provenance(measured, opening_raw=RAW["gripper"])
        assert provenance["opening_source"] == "commanded-fixed"
        assert provenance["model_opening_percent"] == ik.forward_provenance(RAW)["actual_opening_percent"]
        assert provenance["actual_opening_percent"] == ik.forward_provenance(measured)["actual_opening_percent"]
    changed = dict(RAW, shoulder_pan=RAW["shoulder_pan"] + 30, gripper=RAW["gripper"] + 1)
    assert not np.allclose(ik.forward_raw(changed, opening_raw=RAW["gripper"]), first)
    assert ik._backend is backend
    ik.forward_raw(changed)
    assert ik._backend is not backend
    provenance = ik.forward_provenance(changed)
    assert provenance["opening_source"] == "measured"
    assert provenance["actual_opening_percent"] == provenance["model_opening_percent"]
    assert len(provenance["calibration_sha256"]) == len(provenance["official_urdf_sha256"]) == 64
    result = ik.solve_pinch(first, RAW)
    assert result.numerical_ik_only and not result.grasp_eligible
    assert result.provenance["opening_source"] == "commanded-fixed"
    with pytest.raises(ValueError):
        ik.forward_raw(RAW, opening_raw=9000)
