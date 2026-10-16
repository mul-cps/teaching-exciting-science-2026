"""Plan a small inward re-entry using an acquisition-time RGB-D collision scene.

This module performs no actuator writes. The resident owner must independently
match the measured start and checked path before execution, and monitor vision
and encoder freshness while moving. Normal operating limits are never widened.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import time


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


@dataclass(frozen=True)
class CameraRecoveryPlan:
    start_raw: dict
    target_raw: dict
    samples_raw: tuple
    minimum_clearance_m: float
    scene_stamp_monotonic: float
    path_sha256: str


def _digest(value):
    return (isinstance(value, str) and len(value) == 64
            and all(character in "0123456789abcdef" for character in value))


def plan_bound_reentry(raw, admitted_bounds, firmware_bounds, *, scene, now,
                       expected_model_sha256, expected_calibration_sha256,
                       expected_camera_serial="243322071783",
                       minimum_clearance_m=.010):
    """Check an inward path for a soft-limit excursion of at most 32 counts.

    ``scene.minimum_clearance_m(raw_pose)`` must evaluate the complete effective
    robot against observed obstacles, support surfaces and unknown/occluded
    space. It must exclude measured self geometry without clearing unseen
    space. Missing/unknown clearance must return None, causing rejection.
    The scene is bound to the qualified camera registration and effective
    model hashes, and paired within 100 ms to the measured encoder sample.
    A scene adapter supplying this contract is required before live recovery.
    """
    if (not _digest(expected_model_sha256) or not _digest(expected_calibration_sha256)
            or scene.model_sha256 != expected_model_sha256
            or scene.calibration_sha256 != expected_calibration_sha256
            or scene.registration_qualified is not True
            or not isinstance(expected_camera_serial, str) or not expected_camera_serial
            or scene.camera_serial != expected_camera_serial):
        raise ValueError("identity-matched qualified camera registration and model required")
    stamps = (now, scene.stamp_monotonic, scene.arm_stamp_monotonic)
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in stamps)
            or not 0 <= now-scene.stamp_monotonic <= .5
            or not 0 <= now-scene.arm_stamp_monotonic <= .5
            or abs(scene.stamp_monotonic-scene.arm_stamp_monotonic) > .100):
        raise ValueError("fresh synchronized RGB-D and encoder observations required")
    if (type(minimum_clearance_m) not in (int, float)
            or not math.isfinite(minimum_clearance_m) or minimum_clearance_m < .010):
        raise ValueError("at least 10 mm checked clearance required for recovery")
    if not isinstance(raw, dict) or set(raw) != set(JOINTS):
        raise ValueError("complete measured six-joint raw pose required")
    target = dict(raw)
    for joint in JOINTS:
        value = raw[joint]
        if type(value) is not int or not 0 <= value <= 4095:
            raise ValueError("measured counts must be integers in 0..4095")
        try:
            low, high = admitted_bounds[joint]
            fw_low, fw_high = firmware_bounds[joint]
        except (TypeError, ValueError, KeyError) as error:
            raise ValueError("complete calibrated and firmware bounds required") from error
        if (any(type(v) is not int for v in (low, high, fw_low, fw_high))
                or not 0 <= fw_low <= low < high <= fw_high <= 4095):
            raise ValueError("admitted bounds must remain inside firmware bounds")
        if not fw_low <= value <= fw_high:
            raise ValueError("camera recovery cannot override firmware limits")
        if not low <= value <= high:
            if joint == "gripper":
                raise ValueError("gripper recovery requires a separate contact-aware procedure")
            if high-low < 4:
                raise ValueError("insufficient admitted range for inward recovery")
            target[joint] = low+2 if value < low else high-2
            if abs(target[joint]-value) > 32:
                raise ValueError("re-entry exceeds the bounded camera recovery envelope")
    if getattr(scene, "arm_raw", None) != raw:
        raise ValueError("camera scene must be paired to the exact measured encoder pose")
    steps = max(1, max(math.ceil(abs(target[j]-raw[j])/2) for j in JOINTS))
    samples = tuple({j: round(raw[j]+(target[j]-raw[j])*step/steps) for j in JOINTS}
                    for step in range(steps+1))
    started = time.monotonic()
    clearance = math.inf
    for sample in samples:
        observed = scene.minimum_clearance_m(dict(sample))
        if (type(observed) not in (int, float) or not math.isfinite(observed)
                or observed < minimum_clearance_m):
            raise ValueError("recovery path clearance is unknown or insufficient")
        clearance = min(clearance, observed)
        if now + time.monotonic()-started - scene.stamp_monotonic > .5:
            raise ValueError("camera scene expired while checking recovery")
    payload = {"model_sha256": expected_model_sha256,
               "calibration_sha256": expected_calibration_sha256,
               "camera_serial": expected_camera_serial,
               "scene_stamp_monotonic": scene.stamp_monotonic,
               "samples_raw": samples}
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                        allow_nan=False).encode()).hexdigest()
    return CameraRecoveryPlan(dict(raw), target, samples, clearance,
                              scene.stamp_monotonic, digest)
