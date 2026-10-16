"""Offline unit contracts for the resident gripper action mapping."""
import sys
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control"
sys.path.insert(0, str(PACKAGE))

from so101_station_control.gripper_action import (  # noqa: E402
    gap_to_percent,
    percent_to_gap,
)


CALIBRATION = {"closed_gap_m": 0.004, "open_gap_m": 0.075,
               "closed_percent": 0.0, "open_percent": 100.0}


def test_qualified_jaw_gap_calibration_maps_si_to_native_percent_and_back():
    assert gap_to_percent(0.004, CALIBRATION) == 0
    assert gap_to_percent(0.0395, CALIBRATION) == pytest.approx(50)
    assert percent_to_gap(100, CALIBRATION) == 0.075


def test_jaw_gap_calibration_fails_closed_without_explicit_qualification():
    with pytest.raises(ValueError, match="qualified: true"):
        gap_to_percent(0.02, {**CALIBRATION, "qualified": False})
    with pytest.raises(ValueError, match="outside calibrated endpoints"):
        gap_to_percent(0.08, CALIBRATION)


def test_nonfinite_and_out_of_range_native_units_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        gap_to_percent(float("nan"), CALIBRATION)
    with pytest.raises(ValueError, match="outside calibrated endpoints"):
        percent_to_gap(101, CALIBRATION)
