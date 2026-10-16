"""Serialized EV3-to-ROS sorting, with explicit physical completion evidence.

Device adapters must enforce the supplied monotonic deadline and cancellation
event during their blocking operations. This module never opens a second arm
or camera owner and never retries an operation with unknown delivery.
"""
from dataclasses import dataclass
import math
import threading
import time


COLORS = frozenset(("red", "green", "blue", "yellow"))


@dataclass(frozen=True)
class StableTarget:
    track_id: str
    color: str
    position: tuple[float, float, float]
    stamp: float


class StableTargetSelector:
    """Require three different fresh image stamps for one stationary new brick."""

    def __init__(self, *, after_stamp, excluded_ids, max_age_s=.5,
                 max_motion_m=.003, samples=3):
        if (not math.isfinite(after_stamp) or not math.isfinite(max_age_s)
                or not math.isfinite(max_motion_m) or max_age_s <= 0
                or max_motion_m <= 0 or type(samples) is not int or samples < 3):
            raise ValueError("finite observation limits and at least three images required")
        self.after_stamp = after_stamp
        self.excluded_ids = {str(value) for value in excluded_ids}
        self.max_age_s = max_age_s
        self.max_motion_m = max_motion_m
        self.samples = samples
        self._previous = None
        self._anchor = None
        self._count = 0

    def _reset(self):
        self._previous = self._anchor = None
        self._count = 0

    def update(self, status, *, now):
        try:
            stamp = float(status["stamp"])
            if (status.get("state") != "tracking" or not status.get("frame_id")
                    or not math.isfinite(stamp) or not math.isfinite(now)
                    or not self.after_stamp < stamp <= now
                    or now - stamp > self.max_age_s):
                self._reset()
                return None
            candidates = {str(item["track_id"]) for item in status["grasp_candidates"]
                          if item.get("pickup_eligible") is True}
            objects = [item for item in status["objects"]
                       if str(item["track_id"]) not in self.excluded_ids
                       and str(item["track_id"]) in candidates
                       and item.get("pickup_eligible") is True]
            # Multiple new objects are ambiguous; never guess which one exited.
            if len(objects) != 1:
                self._reset()
                return None
            item = objects[0]
            color = item["color"]
            position = tuple(float(value) for value in item["position_world"])
            if (color not in COLORS or len(position) != 3
                    or not all(math.isfinite(value) for value in position)):
                self._reset()
                return None
            number = int(item["track_id"])
            target = StableTarget(f"{color}-{number:04d}", color, position, stamp)
        except (KeyError, TypeError, ValueError, OverflowError):
            self._reset()
            return None
        previous = self._previous
        if previous is not None and stamp <= previous.stamp:
            return None  # Repeated delivery cannot count as another camera image.
        if (previous is None or target.track_id != previous.track_id
                or stamp - previous.stamp > self.max_age_s
                or math.dist(position, self._anchor) > self.max_motion_m):
            self._anchor = position
            self._count = 1
        else:
            self._count += 1
        self._previous = target
        return target if self._count >= self.samples else None


def run_sorting(devices, *, destinations, cycles, log, cancel=None,
                operation_timeout_s=120., clock=time.monotonic):
    """Run finite single-brick cycles through an already-owned ROS station.

    ``devices`` provides require_ready, wait_clear, dispense, detect,
    pick_place, and stop. Only a result with independently verified pickup,
    lift, and release admits another brick. Exceptions propagate after an
    independent stop attempt. No state is silently reset after a fault.
    """
    if type(cycles) is not int or not 1 <= cycles <= 100:
        raise ValueError("cycles must be an integer in 1..100")
    if (isinstance(operation_timeout_s, bool) or not math.isfinite(operation_timeout_s)
            or not 0 < operation_timeout_s <= 600):
        raise ValueError("operation timeout must be in (0, 600] seconds")
    if not isinstance(destinations, dict) or not destinations or set(destinations) - COLORS:
        raise ValueError("known color destinations required")
    for pose in destinations.values():
        if (not isinstance(pose, (list, tuple)) or len(pose) != 7
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in pose)
                or sum(value * value for value in pose[3:]) < 1e-12):
            raise ValueError("destination must contain finite XYZ and a nonzero XYZW quaternion")
    cancel = cancel if cancel is not None else threading.Event()
    completed, colors = 0, {}

    def check(deadline=None):
        if cancel.is_set():
            raise InterruptedError("sorting cancelled")
        if deadline is not None and clock() >= deadline:
            raise TimeoutError("sorting operation exceeded deadline")

    def operation(name, *args):
        check()
        deadline = clock() + operation_timeout_s
        log({"event": "operation_start", "operation": name, "cycle": completed + 1,
             "monotonic": clock(), "deadline": deadline})
        value = getattr(devices, name)(*args, deadline, cancel)
        check(deadline)
        if value is False:
            raise RuntimeError(f"{name} rejected")
        return value

    try:
        check()
        operation("require_ready")
        for _ in range(cycles):
            operation("wait_clear")
            log({"event": "cycle_start", "cycle": completed + 1, "monotonic": clock()})
            receipt = operation("dispense")
            target = operation("detect")
            if not isinstance(target, dict) or target.get("color") not in destinations:
                raise ValueError("detected color has no commissioned destination")
            track_id = target.get("track_id")
            if not isinstance(track_id, str) or not track_id:
                raise ValueError("stable tracked target required")
            result = operation("pick_place", target, destinations[target["color"]])
            if (not isinstance(result, dict) or result.get("state") != "pick_place_complete"
                    or result.get("track_id") != track_id
                    or any(result.get(field) is not True for field in
                           ("pickup_verified", "lift_verified", "release_verified"))):
                raise RuntimeError("pickup, lift and release must all be verified")
            operation("wait_clear")
            completed += 1
            colors[target["color"]] = colors.get(target["color"], 0) + 1
            log({"event": "cycle_complete", "cycle": completed, "monotonic": clock(),
                 "color": target["color"], "target": target, "dispense": receipt,
                 "pick_place": result})
        return {"completed": completed, "colors": colors}
    except BaseException as error:
        try:
            devices.stop()
        except BaseException as cleanup_error:
            error.add_note("sorting stop failed: " + str(cleanup_error))
        try:
            log({"event": "fault", "completed": completed, "monotonic": clock(),
                 "error": str(error), "error_type": type(error).__name__})
        except BaseException:
            pass
        raise
