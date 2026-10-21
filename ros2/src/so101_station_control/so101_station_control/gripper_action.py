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


def validate_joint_calibration(data, limits):
    """Validate measured revolute-joint endpoints without stretching model bounds."""
    if not isinstance(data, dict) or data.get("schema") != "so101-gripper-joint-calibration-v1":
        raise ValueError("joint-angle calibration must use so101-gripper-joint-calibration-v1")
    if data.get("qualified") is not True:
        raise ValueError("joint-angle calibration must declare qualified: true")
    names = ("closed_joint_rad", "open_joint_rad", "closed_percent", "open_percent")
    if any(type(data.get(k)) not in (int, float) or not math.isfinite(data[k]) for k in names):
        raise ValueError("joint-angle calibration endpoints must be finite numbers")
    low, high = limits
    closed, opened = data["closed_joint_rad"], data["open_joint_rad"]
    if not (low <= closed <= high and low <= opened <= high and closed != opened and
            0 <= data["closed_percent"] < data["open_percent"] <= 100):
        raise ValueError("joint-angle endpoints exceed admitted model bounds or percent range")
    if (not isinstance(data.get("arm_id"), str) or not data["arm_id"] or
            not isinstance(data.get("calibration_sha256"), str) or
            len(data["calibration_sha256"]) != 64 or
            any(c not in "0123456789abcdef" for c in data["calibration_sha256"])):
        raise ValueError("joint-angle calibration requires arm identity and calibration SHA256")
    return {k: data[k] for k in (*names, "arm_id", "calibration_sha256")}


def joint_to_percent(radians, calibration):
    """Map a measured/calibrated revolute joint angle, never an SI jaw gap."""
    value = float(radians)
    low, high = sorted((calibration["closed_joint_rad"], calibration["open_joint_rad"]))
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError("gripper joint angle lies outside calibrated endpoints")
    fraction = ((value-calibration["closed_joint_rad"]) /
                (calibration["open_joint_rad"]-calibration["closed_joint_rad"]))
    return calibration["closed_percent"] + fraction * (calibration["open_percent"]-calibration["closed_percent"])


def percent_to_joint(percent, calibration):
    """Publish the same qualified radians used by MoveIt command admission."""
    value = float(percent)
    if not math.isfinite(value) or not calibration["closed_percent"] <= value <= calibration["open_percent"]:
        raise ValueError("gripper percent lies outside joint-angle calibrated endpoints")
    fraction = ((value-calibration["closed_percent"]) /
                (calibration["open_percent"]-calibration["closed_percent"]))
    return calibration["closed_joint_rad"] + fraction * (calibration["open_joint_rad"]-calibration["closed_joint_rad"])


def joint_target_raw(radians, calibration, raw_limits, admitted_limits):
    """Choose the nearest representable raw point inside both qualified intervals.

    The requested angle is validated first. Only its integer quantization can
    move inward; an out-of-range request is never clamped into admission.
    """
    percent = joint_to_percent(radians, calibration)
    low, high = raw_limits
    span = high-low
    if type(low) is not int or type(high) is not int or span <= 0:
        raise ValueError("invalid gripper raw calibration")
    first = max(low, admitted_limits[0], math.floor(low+calibration["closed_percent"]*span/100.))
    last = min(high, admitted_limits[1], math.ceil(low+calibration["open_percent"]*span/100.))
    # Direct percent comparison handles floating point at an integer endpoint
    # conservatively and guarantees the same strict range used by FCL feedback.
    while first <= last and 100.*(first-low)/span < calibration["closed_percent"]:
        first += 1
    while first <= last and 100.*(last-low)/span > calibration["open_percent"]:
        last -= 1
    if first > last:
        raise ValueError("qualified gripper interval contains no admitted integer encoder position")
    requested = low+percent*span/100.
    target = max(first, min(last, int(round(requested))))
    if abs(target-requested) > 1.+1e-12:
        raise ValueError("requested gripper angle lies outside admitted representable range")
    return target
