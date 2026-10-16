"""Qualified SI jaw-gap calibration helpers for the resident gripper action."""
import math


def validate_jaw_calibration(data):
    """Require an explicitly qualified linear gap-to-station-percent mapping."""
    if not isinstance(data, dict) or data.get("schema") != "so101-jaw-gap-calibration-v1":
        raise ValueError("jaw calibration must use so101-jaw-gap-calibration-v1")
    if data.get("qualified") is not True:
        raise ValueError("jaw calibration must declare qualified: true")
    minimum = data.get("closed_gap_m")
    maximum = data.get("open_gap_m")
    low = data.get("closed_percent")
    high = data.get("open_percent")
    values = (minimum, maximum, low, high)
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError("jaw calibration endpoints must be finite numbers")
    if not (0 <= minimum < maximum <= 0.20 and 0 <= low < high <= 100):
        raise ValueError("jaw calibration endpoints are outside physical station units")
    return {"closed_gap_m": float(minimum), "open_gap_m": float(maximum),
            "closed_percent": float(low), "open_percent": float(high)}


def gap_to_percent(gap_m, calibration):
    """Convert an SI jaw gap to the resident owner's normalized 0..100 unit."""
    gap = float(gap_m)
    if not math.isfinite(gap):
        raise ValueError("jaw gap must be finite")
    c = validate_jaw_calibration({"schema": "so101-jaw-gap-calibration-v1",
                                  "qualified": True, **calibration})
    if not c["closed_gap_m"] <= gap <= c["open_gap_m"]:
        raise ValueError("jaw gap lies outside calibrated endpoints")
    fraction = (gap - c["closed_gap_m"]) / (c["open_gap_m"] - c["closed_gap_m"])
    return c["closed_percent"] + fraction * (c["open_percent"] - c["closed_percent"])


def percent_to_gap(percent, calibration):
    """Convert a measured station percent into calibrated SI jaw gap."""
    value = float(percent)
    if not math.isfinite(value):
        raise ValueError("gripper percent must be finite")
    c = validate_jaw_calibration({"schema": "so101-jaw-gap-calibration-v1",
                                  "qualified": True, **calibration})
    if not c["closed_percent"] <= value <= c["open_percent"]:
        raise ValueError("gripper percent lies outside calibrated endpoints")
    fraction = (value - c["closed_percent"]) / (c["open_percent"] - c["closed_percent"])
    return c["closed_gap_m"] + fraction * (c["open_gap_m"] - c["closed_gap_m"])
