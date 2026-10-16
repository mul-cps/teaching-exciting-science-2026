"""Resident, explicitly commanded SO-ARM101 raw-count controller.

The session connects torque-disabled and never replays a plan on startup.  A
successful plan leaves torque enabled at its final pose so vision can correct
the next command without letting the gripper fall.  EOF, ``stop``, ``exit``,
idle timeout, or any control fault disables torque, verifies it, then restores
the RAM profile that was present before this session's first motion.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import select
import sys
import time
import uuid

from workshop.safe_home import load_home
from .config import JOINTS, load_config
from .locking import station_lock, station_lock_path
from .camera_recovery import plan_bound_reentry
from .ros_trajectory import (ARM_JOINTS, ROS_MAX_ACCELERATION_RAD_S2,
                             ROS_MAX_JERK_RAD_S3, ROS_MAX_VELOCITY_RAD_S,
                             path_digest, prepare_trajectory)

PERIOD_S = 0.05
ROS_MAX_TRACK_ERROR_RAD = math.radians(10.0)
ROS_GOAL_TOLERANCE_RAD = math.radians(2.0)
# The native servo profile (Goal_Velocity 160 body / 220 gripper) executes
# independently of this Python feedback loop. Tolerate at most four missing
# 50 ms observations per leg (one 250 ms freshness window), then re-anchor the
# next tick to the fresh read. This does not scale the absolute raw-count jump
# budgets: body/gripper remain capped at 80/150 counts per observed frame and
# 180/650 counts per leg, with the existing six-second deadline unchanged.
MAX_MISSED_FEEDBACK_TICKS_PER_LEG = 4
MAX_FEEDBACK_GAP_S = PERIOD_S * (MAX_MISSED_FEEDBACK_TICKS_PER_LEG + 1)
FEEDBACK_GAP_EPSILON_S = 0.001
CALIBRATION_ENDPOINT_WINDOW_S = 0.30
CALIBRATION_ENDPOINT_MIN_SAMPLES = 5
CALIBRATION_ENDPOINT_MAX_SPAN_COUNTS = 4
SYNC_READ_MAX_ATTEMPTS = 3
SYNC_READ_DEADLINE_S = 0.020
BODY_STEP = 180
GRIPPER_STEP = 650
ARRIVAL_COUNTS = 10
QUIET_COUNTS = 4
# Distinguish a blocked grasp from the measured near-empty close offset. The
# follower's empty closure can sit about 9 raw counts above its requested
# position; require a clear 30-count excess before inferring contact.
CONTACT_MIN_EXCESS_COUNTS = 30
FRAME_BODY_COUNTS = 80
FRAME_GRIPPER_COUNTS = 150
LEG_DEADLINE_S = 6.0
DWELL_S = 0.4
PROFILE = {
    "Acceleration": {joint: 8 for joint in JOINTS},
    "Goal_Velocity": {joint: (220 if joint == "gripper" else 160) for joint in JOINTS},
    "Torque_Limit": {joint: (250 if joint == "gripper" else 700) for joint in JOINTS},
}
RAM_REGISTERS = tuple(PROFILE)


def _atomic_snapshot_json(path, data):
    """Atomically replace a live snapshot without forcing storage to flush.

    Readers never observe a partial JSON document. The camera twin needs the
    newest snapshot while the session runs; per-frame fsync of both the file
    and directory adds blocking latency without improving that live contract.
    Durable command logs and calibration/home files keep their own policies.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("x") as stream:
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class ArmSessionCancelled(RuntimeError):
    """Internal signal used to unwind motion into verified torque-off cleanup."""


class ArmPlanPreflightError(ValueError):
    """A move plan rejected before any profile, goal, or torque write."""


@dataclass(frozen=True)
class CameraRecoveryPolicy:
    """Trusted in-process identities; never populated from a motion command."""
    model_sha256: str
    calibration_sha256: str
    camera_serial: str
    arm_calibration_sha256: str


class ArmSession:
    """One serialized owner of a connected follower bus.

    ``arm`` is a :class:`LeRobotArm` or a protocol-compatible fake.  It is
    intentionally injected so the control loop and cleanup can be tested with
    no serial device access.
    """

    def __init__(self, arm, state_file, logdir, *, clock=time.monotonic,
                 wall_clock=lambda: datetime.now(timezone.utc), sleep=time.sleep,
                 idle_timeout=60.0, recover_sag=False, persistent_idle=False,
                 camera_scene_provider=None, camera_recovery_policy=None):
        self.arm = arm
        self.bus = arm.bus
        self.state_file = Path(state_file)
        self.logdir = Path(logdir)
        self.clock = clock
        self.wall_clock = wall_clock
        self.sleep = sleep
        self.idle_timeout = float(idle_timeout)
        self.persistent_idle = bool(persistent_idle)
        self._idle_handled = False
        self._shutdown_complete = False
        self._component_scope = False
        self._component_saved_ram = None
        self._component_profile_readback = {}
        self._component_command = None
        self.recover_sag = bool(recover_sag)
        self._sag_recovery_available = bool(recover_sag)
        self.saved_ram = None
        self.firmware_min = None
        self.firmware_max = None
        self.admitted_min = None
        self.admitted_max = None
        self.calibration_digest = None
        self.profile = "unmodified"
        self._active_profile = None
        self.last_command = self.clock()
        self.log_path = self.logdir / ("arm-session-" + uuid.uuid4().hex + ".jsonl")
        self._log_stream = None
        self._initialized = False
        self._motion_busy = False
        self._cancel_requested = False
        self._cancel_reason = None
        self._cancel_request_id = None
        self._exit_requested = False
        self._input_fd = None
        self._input_buffer = bytearray()
        self._emit = None
        self._active_leg = None
        self._completed_leg_records = []
        self._read_diagnostics = {}
        self._teleop = None
        self._teleop_skip_next_sample = False
        # This callback must be nonblocking latest-observation/cache access.
        # It receives (measured_raw_copy, exact_encoder_stamp_monotonic) and
        # supplies a qualified scene paired to that encoder observation.
        # A Python callback cannot be forcibly time bounded here: elapsed-time
        # guards reject a slow return before any subsequent motion update.
        # Production startup deliberately supplies neither provider nor policy.
        self.camera_scene_provider = camera_scene_provider
        self.camera_recovery_policy = camera_recovery_policy

    @staticmethod
    def _six_ints(values, label):
        if not isinstance(values, dict) or set(values) != set(JOINTS):
            raise ValueError(f"{label} requires exactly six named joints")
        result = {}
        for joint in JOINTS:
            value = values[joint]
            if type(value) is not int:
                raise ValueError(f"{label}/{joint} must be an integer raw count")
            result[joint] = value
        return result

    def _read(self, register):
        started = self.clock()
        error = None
        attempts = 0
        deadline_exceeded = False
        for attempt in range(1, SYNC_READ_MAX_ATTEMPTS + 1):
            attempts = attempt
            try:
                # Each SDK call rebuilds the GroupSyncRead parameter list via
                # clearParam/addParam. Only idempotent reads are retried; writes
                # remain single-shot to avoid replaying partially applied goals.
                values = self.bus.sync_read(register, normalize=False, num_retry=0)
                result = self._six_ints(values, register)
                duration = max(0.0, self.clock() - started)
                retry_deadline_exceeded = (
                    attempts > 1 and duration >= SYNC_READ_DEADLINE_S)
                self._read_diagnostics[register] = {
                    "attempts": attempts, "retries": attempts - 1,
                    "duration_s": duration, "success": not retry_deadline_exceeded,
                    "deadline_exceeded": retry_deadline_exceeded,
                }
                if retry_deadline_exceeded:
                    raise TimeoutError(
                        f"sync read {register} exceeded {SYNC_READ_DEADLINE_S * 1000:.0f} ms retry deadline")
                return result
            except ConnectionError as exc:
                error = exc
                duration = max(0.0, self.clock() - started)
                deadline_exceeded = duration >= SYNC_READ_DEADLINE_S
                if deadline_exceeded or attempt == SYNC_READ_MAX_ATTEMPTS:
                    self._read_diagnostics[register] = {
                        "attempts": attempts, "retries": attempts - 1,
                        "duration_s": duration, "success": False,
                        "deadline_exceeded": deadline_exceeded,
                        "error": str(exc),
                    }
                    break
        diagnostic = self._read_diagnostics[register]
        raise ConnectionError(
            f"sync read {register} failed after {attempts} attempts "
            f"({attempts - 1} retries, {diagnostic['duration_s']:.6f}s): {error}") from error

    def _write_each(self, register, values):
        if self._component_scope:
            raise RuntimeError("component scope prohibits body writes")
        for joint in JOINTS:
            self._poll_busy_input()
            self.bus.write(register, joint, values[joint], normalize=False, num_retry=0)
            self._poll_busy_input()

    def _check_cancel(self):
        if self._cancel_requested:
            raise ArmSessionCancelled(self._cancel_reason or "cancelled")

    def _check_leg_deadline(self, started, label):
        self._poll_busy_input()
        if self.clock() - started >= LEG_DEADLINE_S:
            raise TimeoutError("leg 6 second deadline expired: " + label)

    def _check_feedback_freshness(self, *, label, phase, previous_read_at,
                                  read_started, read_ended, total_missed,
                                  previous_tick_timing):
        gap = max(0.0, read_ended - previous_read_at)
        periods = max(1, int((gap + FEEDBACK_GAP_EPSILON_S) / PERIOD_S))
        missed = periods - 1
        missed_total = total_missed + missed
        details = dict(label=label, phase=phase,
                       previous_position_read_monotonic=previous_read_at,
                       position_read_started_monotonic=read_started,
                       position_read_ended_monotonic=read_ended,
                       feedback_gap_s=gap,
                       position_read_duration_s=max(0.0, read_ended - read_started),
                       missed_ticks=missed, missed_ticks_total=missed_total,
                       max_gap_s=MAX_FEEDBACK_GAP_S,
                       max_missed_ticks_per_leg=MAX_MISSED_FEEDBACK_TICKS_PER_LEG,
                       previous_tick_timing=previous_tick_timing)
        if (gap > MAX_FEEDBACK_GAP_S or
                missed_total > MAX_MISSED_FEEDBACK_TICKS_PER_LEG):
            self._event("feedback_cadence_fault", **details)
            raise RuntimeError("arm feedback freshness exceeded bounded period budget: " + label)
        return gap, missed, missed_total

    @staticmethod
    def _stop_request_id(command):
        if set(command) not in ({"command"}, {"command", "request_id"}):
            raise ValueError("stop accepts only optional request_id")
        request_id = command.get("request_id")
        if "request_id" in command and (not isinstance(request_id, str) or
                len(request_id) != 32 or any(c not in "0123456789abcdef" for c in request_id)):
            raise ValueError("stop request_id must be 32 lowercase hexadecimal characters")
        return request_id

    def _cancel(self, reason, *, exiting=False):
        self._cancel_requested = True
        self._cancel_reason = reason
        self._exit_requested = self._exit_requested or exiting

    def _busy_line(self, line):
        try:
            command = json.loads(line)
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            self._cancel("malformed_stdin_command: " + str(error))
            self._check_cancel()
        if not isinstance(command, dict) or not isinstance(command.get("command"), str):
            self._cancel("malformed_stdin_command")
            self._check_cancel()
        name = command["command"]
        if name == "read_state" and set(command) != {"command"}:
            self._cancel("malformed_busy_command")
            self._check_cancel()
        if name == "read_state":
            result = {"ok": True, "state": self.read_state()}
            if self._emit is not None:
                self._emit(result)
            return
        if name == "cancel_hold":
            if set(command) != {"command"}:
                self._cancel("malformed_busy_command")
                self._check_cancel()
            self._cancel("trajectory_cancel_hold")
            self._check_cancel()
        if name in ("stop", "exit"):
            try:
                request_id = self._stop_request_id(command) if name == "stop" else None
                if name == "exit" and set(command) != {"command"}:
                    raise ValueError("exit accepts no extra fields")
            except ValueError:
                self._cancel("malformed_busy_command")
                self._check_cancel()
            self._cancel_request_id = request_id
            self._cancel("busy_" + name, exiting=name == "exit")
            self._check_cancel()
        if self._emit is not None:
            self._emit({"ok": False, "error": "arm session busy; only read_state, stop, and exit are accepted",
                        "command": name})

    def _poll_busy_input(self):
        """Pump status/cancellation input between bounded bus transactions."""
        if not self._motion_busy or self._input_fd is None:
            self._check_cancel()
            return
        # First consume complete commands already buffered with the motion
        # request, then read any newly-ready stdin bytes without blocking.
        while b"\n" in self._input_buffer:
            line, _, remainder = self._input_buffer.partition(b"\n")
            self._input_buffer[:] = remainder
            if len(line) > 1_048_576:
                self._cancel("oversized_stdin_command")
                self._check_cancel()
            if line.strip():
                self._busy_line(bytes(line))
        if len(self._input_buffer) > 1_048_576:
            self._cancel("oversized_stdin_command")
            self._check_cancel()
        ready, _, _ = select.select([self._input_fd], [], [], 0)
        if ready:
            data = os.read(self._input_fd, 4096)
            if not data:
                self._cancel("stdin_eof", exiting=True)
                self._check_cancel()
            self._input_buffer.extend(data)
            if len(self._input_buffer) > 1_048_576 and b"\n" not in self._input_buffer:
                self._cancel("oversized_stdin_command")
                self._check_cancel()
            while b"\n" in self._input_buffer:
                line, _, remainder = self._input_buffer.partition(b"\n")
                self._input_buffer[:] = remainder
                if len(line) > 1_048_576:
                    self._cancel("oversized_stdin_command")
                    self._check_cancel()
                if line.strip():
                    self._busy_line(bytes(line))
        self._check_cancel()

    def _calibration_guard(self):
        path = Path(self.arm.settings["calibration"]["path"])
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != self.calibration_digest:
            raise RuntimeError("calibration file changed during resident session")

    def initialize(self):
        """Connect/admit read-only state; no goal/profile write occurs here."""
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self.log_path.open("a", buffering=1)
        try:
            if self.bus is None or not self.bus.is_connected:
                self.arm.connect()
                self.bus = self.arm.bus
            self.arm._calibration()
            self.calibration_digest = self.arm.settings["calibration"]["sha256"]
            self._calibration_guard()
            self.arm._bounds()
            if not self.bus.is_calibrated:
                raise RuntimeError("hardware calibration differs from station calibration")
            for servo_id, joint in enumerate(JOINTS, start=1):
                if self.bus.ping(joint, num_retry=0, raise_on_error=True) != 777:
                    raise RuntimeError("expected STS3215 model 777 for " + joint)
                if self.bus.motors[joint].id != servo_id:
                    raise RuntimeError("unexpected motor ID for " + joint)
            self.firmware_min = self._read("Min_Position_Limit")
            self.firmware_max = self._read("Max_Position_Limit")
            for joint in JOINTS:
                if not 0 <= self.firmware_min[joint] < self.firmware_max[joint] <= 4095:
                    raise ValueError("invalid firmware raw limits for " + joint)
            self.admitted_min, self.admitted_max = {}, {}
            for joint in JOINTS:
                entry = self.arm.calibration[joint]
                lo, hi = self.arm.bounds[joint]
                if joint == "gripper":
                    scale = entry["range_max"] - entry["range_min"]
                    config_min = math.ceil(lo * scale / 100 + entry["range_min"])
                    config_max = math.floor(hi * scale / 100 + entry["range_min"])
                else:
                    midpoint = (entry["range_min"] + entry["range_max"]) / 2
                    config_min = math.ceil(lo * 4095 / 360 + midpoint)
                    config_max = math.floor(hi * 4095 / 360 + midpoint)
                self.admitted_min[joint] = max(entry["range_min"], config_min,
                                               self.firmware_min[joint])
                self.admitted_max[joint] = min(entry["range_max"], config_max,
                                               self.firmware_max[joint])
                if self.admitted_min[joint] > self.admitted_max[joint]:
                    raise ValueError("empty firmware/calibration/configured raw intersection: " + joint)
            torque = self._read("Torque_Enable")
            if any(value != 0 for value in torque.values()):
                raise RuntimeError("resident session requires verified torque-off startup")
            # Validate measured-home provenance and its calibrated representation.
            self.home_pose = self.arm._pose(load_home(
                self.arm.settings["home_file"], self.arm.settings["id"],
                self.calibration_digest))
            self.home_raw = self.arm._raw_goal(self.home_pose)
            if any(not self.firmware_min[j] <= self.home_raw[j] <= self.firmware_max[j]
                   for j in JOINTS):
                raise ValueError("measured home is outside firmware limits")
            actual = self._read("Present_Position")
            for joint in JOINTS:
                if not self.firmware_min[joint] <= actual[joint] <= self.firmware_max[joint]:
                    overrun = actual[joint] - self.firmware_max[joint] if actual[joint] > self.firmware_max[joint] else self.firmware_min[joint] - actual[joint]
                    if (not self._sag_recovery_available or joint == "gripper" or overrun > 16):
                        raise ValueError("actual raw position outside firmware limits: " + joint)
            state = self.publish_state(event="connected", torque=torque, raw=actual)
            self._event("connected", state=state)
            self._initialized = True
            self.last_command = self.clock()
            return state
        except BaseException as error:
            try:
                self.shutdown(reason="initialize_error")
            except BaseException as cleanup_error:
                error.add_note("arm session cleanup failed: " + str(cleanup_error))
            raise

    def _record(self, value):
        if self._log_stream is None:
            return
        self._log_stream.write(json.dumps(value, allow_nan=False, sort_keys=True) + "\n")
        self._log_stream.flush()

    def _event(self, event, **fields):
        self._record(dict(event=event, utc=self.wall_clock().isoformat(),
                          monotonic=self.clock(), arm_id=self.arm.settings["id"],
                          calibration_sha256=self.calibration_digest,
                          write_scope="gripper_only" if self._component_scope else "normal",
                          component_profile_registers=dict(self._component_profile_readback),
                          component_command=(dict(self._component_command) if self._component_command else None),
                          component_restore_pending=self._component_saved_ram is not None,
                          profile=self.profile, **fields))

    def publish_state(self, *, event="sample", torque=None, phase=None, label=None, raw=None,
                      position_monotonic=None, position_utc=None,
                      torque_monotonic=None, torque_utc=None, feedback_timing=None,
                      trajectory_goal_id=None, trajectory_time=None,
                      trajectory_desired=None):
        raw = self._read("Present_Position") if raw is None else dict(raw)
        if position_monotonic is None:
            position_monotonic = self.clock()
            position_utc = self.wall_clock()
        # Telemetry remains observable when torque-off sag crosses a configured
        # or firmware bound. Motion admission below stays strict; only snapshots
        # represent the actual six in-register positions without clamping them.
        raw = self._six_ints(raw, "Present_Position")
        calibration = self.arm.calibration
        pose, violations = {}, []
        for joint in JOINTS:
            value, entry = raw[joint], calibration[joint]
            if not 0 <= value <= 4095:
                raise ValueError("actual raw position outside 0..4095: " + joint)
            if not entry["range_min"] <= value <= entry["range_max"]:
                violations.append({"joint": joint, "kind": "calibration_range",
                                   "raw": value, "min": entry["range_min"],
                                   "max": entry["range_max"]})
            pose[joint] = ((value - entry["range_min"]) * 100 /
                           (entry["range_max"] - entry["range_min"]) if joint == "gripper"
                           else (value - (entry["range_min"] + entry["range_max"]) / 2) * 360 / 4095)
            lo, hi = self.arm.bounds[joint]
            if not lo <= pose[joint] <= hi:
                violations.append({"joint": joint, "kind": "configured_pose_range",
                                   "pose": pose[joint], "min": lo, "max": hi})
            if self.firmware_min is not None and not self.firmware_min[joint] <= value <= self.firmware_max[joint]:
                violations.append({"joint": joint, "kind": "firmware_range",
                                   "raw": value, "min": self.firmware_min[joint],
                                   "max": self.firmware_max[joint]})
        torque = self._read("Torque_Enable") if torque is None else torque
        if torque_monotonic is None:
            torque_monotonic = self.clock()
            torque_utc = self.wall_clock()
        if any(value not in (0, 1) for value in torque.values()):
            raise RuntimeError("non-binary torque state")
        state = dict(schema=1, event=event, arm_id=self.arm.settings["id"],
                     calibration_sha256=self.calibration_digest,
                     timestamp_utc=position_utc.isoformat(),
                     monotonic=position_monotonic, raw=raw, pose=pose, torque=torque,
                     torque_timestamp_utc=torque_utc.isoformat(),
                     torque_monotonic=torque_monotonic,
                     violations=violations,
                     write_scope="gripper_only" if self._component_scope else "normal",
                     component_profile_registers=dict(self._component_profile_readback),
                     component_command=(dict(self._component_command) if self._component_command else None),
                     component_restore_pending=self._component_saved_ram is not None,
                     profile=self.profile,
                     profile_registers=(self._active_profile if self.saved_ram is not None and self.profile != "unmodified" else None),
                     phase=phase, label=label, feedback_timing=feedback_timing,
                     trajectory_goal_id=trajectory_goal_id,
                     trajectory_time=trajectory_time,
                     trajectory_desired=trajectory_desired,
                     bus_read_diagnostics=dict(self._read_diagnostics))
        snapshot_started = self.clock()
        _atomic_snapshot_json(self.state_file, state)
        if feedback_timing is not None:
            feedback_timing["snapshot_replace_duration_s"] = max(
                0.0, self.clock() - snapshot_started)
        log_started = self.clock()
        # The latest-state file carries live samples. Trajectory samples stay in
        # memory until completion so JSONL flush latency cannot enter the tick.
        if event not in {"read_state", "trajectory_preflight",
                         "trajectory_feedback", "trajectory_settle"}:
            self._record(state)
        if feedback_timing is not None:
            feedback_timing["sample_log_append_duration_s"] = max(
                0.0, self.clock() - log_started)
        return state

    def read_state(self):
        self._require_initialized()
        state = self.publish_state(event="read_state")
        self.last_command = self.clock()
        return state

    def _require_initialized(self):
        if not self._initialized:
            raise RuntimeError("resident arm session is not initialized")

    def _validate_target(self, target, label):
        target = self._six_ints(target, "target_raw")
        for joint in JOINTS:
            if not 0 <= target[joint] <= 4095:
                raise ValueError(f"target outside 0..4095: {label}/{joint}")
            if not self.admitted_min[joint] <= target[joint] <= self.admitted_max[joint]:
                raise ValueError(f"target outside firmware/calibration/configured limits: {label}/{joint}")
        # _validate_raw also checks calibration and configured physical bounds.
        self.arm._validate_raw(target)
        return target

    @staticmethod
    def _step_budget(joint):
        return GRIPPER_STEP if joint == "gripper" else BODY_STEP

    def _validate_transition(self, start, target, label, *, allow_sag=False):
        recovery = {}
        for joint in JOINTS:
            low, high = self.firmware_min[joint], self.firmware_max[joint]
            value = start[joint]
            if not low <= value <= high:
                is_body = joint != "gripper"
                overrun = value - high if value > high else low - value
                inward = target[joint] < value if value > high else target[joint] > value
                if (not allow_sag or not is_body or overrun > 16 or not inward or
                        not self.admitted_min[joint] <= target[joint] <= self.admitted_max[joint]):
                    raise ValueError("actual raw position outside firmware limits: " + joint)
                recovery[joint] = dict(side="high" if value > high else "low",
                                       start_raw=value, reentered=False)
            if abs(target[joint] - value) > self._step_budget(joint):
                raise ValueError("leg exceeds raw step budget: " + label + "/" + joint)
        return recovery

    def _release_component_scope(self):
        if not self._component_scope:
            return
        # Component cleanup stays latched until a fresh, ordinary all-off,
        # in-bounds admission. An out-of-bounds passive body cannot enable here.
        self._calibration_guard()
        raw = self._read("Present_Position")
        torque = self._read("Torque_Enable")
        self._validate_target(raw, "component_scope_normal_admission")
        if any(torque.values()) or self._component_saved_ram is not None:
            raise RuntimeError("normal motion requires clean component cleanup and all torque off")
        self._component_scope = False

    def _ensure_profile(self, *, teleop=False):
        self._release_component_scope()
        self._poll_busy_input()
        self._calibration_guard()
        self._shutdown_complete = False
        if self.saved_ram is None:
            self.saved_ram = {}
            for register in RAM_REGISTERS:
                self._poll_busy_input()
                self.saved_ram[register] = self._read(register)
                self._poll_busy_input()
        # Set the admitted native profile once; leave it active between plans.
        actual = {}
        for register in RAM_REGISTERS:
            self._poll_busy_input()
            actual[register] = self._read(register)
            self._poll_busy_input()
        profile = dict(PROFILE)
        if teleop:
            # Position mode: zero selects the servo's maximum velocity.
            # Retain acceleration smoothing and the existing torque limits.
            profile["Goal_Velocity"] = dict.fromkeys(JOINTS, 0)
        for register, wanted in profile.items():
            if actual[register] != wanted:
                self._write_each(register, wanted)
                self._poll_busy_input()
                if self._read(register) != wanted:
                    raise RuntimeError("native RAM profile readback mismatch: " + register)
        self._active_profile = {register: dict(values) for register, values in profile.items()}
        self.profile = ("teleop-8-uncapped-velocity-torque-700-250" if teleop else
                        "native-8-160-220-torque-700-250")

    def _enable_current_goal(self, target, started, label):
        self._release_component_scope()
        # Install goals before enabling torque; verify broadcast write/readback.
        self._check_leg_deadline(started, label)
        self.bus.sync_write("Goal_Position", target, normalize=False, num_retry=0)
        self._check_leg_deadline(started, label)
        goals = self._read("Goal_Position")
        self._check_leg_deadline(started, label)
        if any(abs(goals[j] - target[j]) > 1 for j in JOINTS):
            raise RuntimeError("Goal_Position readback mismatch")
        torque = self._read("Torque_Enable")
        self._check_leg_deadline(started, label)
        for joint in JOINTS:
            if torque[joint] == 0:
                self._check_leg_deadline(started, label)
                self.bus.write("Torque_Enable", joint, 1, normalize=False, num_retry=0)
                self._check_leg_deadline(started, label)
        torque = self._read("Torque_Enable")
        if any(value != 1 for value in torque.values()):
            raise RuntimeError("torque-enable readback failed")
        self._check_leg_deadline(started, label)
        return torque

    def _guard_actual(self, actual, previous, start, active, recovery, started):
        now = self.clock()
        for joint in JOINTS:
            low, high = self.firmware_min[joint], self.firmware_max[joint]
            admitted_low = self.admitted_min[joint]
            admitted_high = self.admitted_max[joint]
            state = recovery.get(joint)
            if state and not state["reentered"]:
                if now - started > 1.0 and not admitted_low <= actual[joint] <= admitted_high:
                    raise TimeoutError("sag recovery re-entry deadline expired: " + joint)
                if actual[joint] > high and (state["side"] != "high" or
                                             actual[joint] > state["start_raw"] + 2):
                    raise RuntimeError("sag recovery moved farther out of bounds: " + joint)
                if actual[joint] < low and (state["side"] != "low" or
                                            actual[joint] < state["start_raw"] - 2):
                    raise RuntimeError("sag recovery moved farther out of bounds: " + joint)
                if admitted_low <= actual[joint] <= admitted_high:
                    state["reentered"] = True
            elif not low <= actual[joint] <= high:
                raise RuntimeError("actual raw position outside firmware limits: " + joint)
            frame_limit = FRAME_GRIPPER_COUNTS if joint == "gripper" else FRAME_BODY_COUNTS
            if abs(actual[joint] - previous[joint]) > frame_limit:
                raise RuntimeError("per-frame movement exceeds budget: " + joint)
            if abs(actual[joint] - start[joint]) > self._step_budget(joint) + ARRIVAL_COUNTS:
                raise RuntimeError("actual excursion exceeds per-leg budget: " + joint)
            if joint not in active and abs(actual[joint] - start[joint]) > 40:
                raise RuntimeError("inactive joint drift exceeds 40 counts: " + joint)

    def _run_leg(self, label, target, start, *, allow_sag=False, phase="move_plan",
                 grasp=False, hold_on_contact=False):
        self._poll_busy_input()
        recovery = self._validate_transition(start, target, label, allow_sag=allow_sag)
        active = {joint for joint in JOINTS if target[joint] != start[joint]}
        started = self.clock()
        self._event("leg_start", label=label, start_raw=start, target_raw=target,
                    active_joints=sorted(active), sag_recovery=recovery)
        setup_started = self.clock()
        self._enable_current_goal(target, started, label)
        setup_duration = self.clock() - setup_started
        self._event("command_setup_complete", label=label,
                    setup_duration_s=setup_duration,
                    elapsed_from_leg_start_s=self.clock() - started)
        # The native profile moves the servo between observations. Start the
        # feedback freshness window only after the goal broadcast and its
        # readback have completed, rather than charging command setup against
        # the first 20 Hz sample.
        last_feedback_at = self.clock()
        previous = dict(start)
        frames = []
        next_frame = last_feedback_at
        next_trim = last_feedback_at + 0.6
        commanded = dict(target)
        trim = dict.fromkeys(JOINTS, 0)
        contact_inferred = False
        missed_total = 0
        previous_tick_timing = None

        def poll_control(timing):
            poll_started = self.clock()
            self._poll_busy_input()
            timing["control_input_duration_s"] += max(0.0, self.clock() - poll_started)

        settled = False
        while not settled:
            timing = {"command_setup_duration_s": setup_duration,
                      "control_input_duration_s": 0.0}
            poll_control(timing)
            read_started = self.clock()
            actual = self._read("Present_Position")
            position_monotonic = self.clock()
            position_utc = self.wall_clock()
            now = position_monotonic
            if now - started >= LEG_DEADLINE_S:
                raise TimeoutError("leg 6 second deadline expired: " + label)
            poll_control(timing)
            gap, missed, missed_total = self._check_feedback_freshness(
                label=label, phase=phase, previous_read_at=last_feedback_at,
                read_started=read_started, read_ended=position_monotonic,
                total_missed=missed_total, previous_tick_timing=previous_tick_timing)
            timing.update(feedback_gap_s=gap, missed_ticks=missed,
                          missed_ticks_total=missed_total,
                          position_read_duration_s=max(0.0, position_monotonic - read_started))
            poll_control(timing)
            self._guard_actual(actual, previous, start, active, recovery, started)
            goal_read_started = self.clock()
            goals = self._read("Goal_Position")
            timing["goal_read_duration_s"] = max(0.0, self.clock() - goal_read_started)
            poll_control(timing)
            if any(abs(goals[j] - commanded[j]) > 1 for j in JOINTS):
                raise RuntimeError("goal changed or was clamped: " + label)
            frames.append(actual)
            frames = frames[-4:]
            torque_read_started = self.clock()
            torque = self._read("Torque_Enable")
            torque_monotonic = self.clock()
            torque_utc = self.wall_clock()
            timing["torque_read_duration_s"] = max(0.0, torque_monotonic - torque_read_started)
            poll_control(timing)
            publish_started = self.clock()
            self.publish_state(event="sample", phase=phase, label=label, raw=actual,
                               torque=torque, position_monotonic=position_monotonic,
                               position_utc=position_utc, torque_monotonic=torque_monotonic,
                               torque_utc=torque_utc, feedback_timing=timing)
            timing["sample_publish_duration_s"] = max(0.0, self.clock() - publish_started)
            previous_tick_timing = dict(timing)
            body_quiet = len(frames) >= 3 and all(
                max(frame[j] for frame in frames[-3:]) - min(frame[j] for frame in frames[-3:]) <= QUIET_COUNTS
                for j in JOINTS[:-1])
            gripper_quiet = len(frames) >= 4 and max(frame["gripper"] for frame in frames[-4:]) - min(
                frame["gripper"] for frame in frames[-4:]) <= QUIET_COUNTS
            body_ready = all(abs(actual[j] - target[j]) <= ARRIVAL_COUNTS for j in JOINTS[:-1]) and body_quiet
            normal_ready = body_ready and abs(actual["gripper"] - target["gripper"]) <= ARRIVAL_COUNTS and gripper_quiet
            contact_ready = (grasp and now - started >= 0.6 and
                             actual["gripper"] - target["gripper"] >= CONTACT_MIN_EXCESS_COUNTS and
                             gripper_quiet and body_ready)
            if normal_ready or contact_ready:
                contact_inferred = bool(contact_ready)
                if contact_inferred and hold_on_contact:
                    # A jaw-only command must stop pressing toward the next
                    # chunk as soon as contact is inferred. Generic plans
                    # retain their existing grasp/hold semantics.
                    held = self.cancel_hold()
                    commanded = dict(held["raw"])
                settled = True
            elif now >= next_trim:
                revised = dict(commanded)
                if body_quiet:
                    for joint in JOINTS[:-1]:
                        error = target[joint] - actual[joint]
                        if abs(error) > ARRIVAL_COUNTS:
                            increment = max(-6, min(6, round(error * 0.12)))
                            trim[joint] = max(-112, min(112, trim[joint] + increment))
                            revised[joint] = max(
                                self.admitted_min[joint],
                                min(self.admitted_max[joint], target[joint] + trim[joint]))
                if revised != commanded:
                    self._check_leg_deadline(started, label)
                    self.bus.sync_write("Goal_Position", revised, normalize=False, num_retry=0)
                    self._check_leg_deadline(started, label)
                    commanded = revised
                    self._event("bounded_gravity_trim", label=label,
                                command_raw=commanded, trim_raw=trim)
                next_trim = now + 0.1
            if settled:
                last_feedback_at = position_monotonic
                if missed:
                    self._event("feedback_overrun", **timing,
                                label=label, phase=phase,
                                resynchronized=True,
                                max_gap_s=MAX_FEEDBACK_GAP_S,
                                max_missed_ticks_per_leg=MAX_MISSED_FEEDBACK_TICKS_PER_LEG)
                break
            previous = actual
            last_feedback_at = position_monotonic
            next_frame = position_monotonic + PERIOD_S
            if missed:
                self._event("feedback_overrun", **timing,
                            label=label, phase=phase,
                            resynchronized=True,
                            max_gap_s=MAX_FEEDBACK_GAP_S,
                            max_missed_ticks_per_leg=MAX_MISSED_FEEDBACK_TICKS_PER_LEG)
            self.sleep(max(0.0, next_frame - self.clock()))

        dwell_started = self.clock()
        previous = dict(actual)
        settled_raw = dict(actual)
        while self.clock() - dwell_started < DWELL_S:
            timing = {"command_setup_duration_s": setup_duration,
                      "control_input_duration_s": 0.0}
            poll_control(timing)
            read_started = self.clock()
            actual = self._read("Present_Position")
            position_monotonic = self.clock()
            position_utc = self.wall_clock()
            if position_monotonic - started >= LEG_DEADLINE_S:
                raise TimeoutError("leg 6 second deadline expired during dwell: " + label)
            poll_control(timing)
            gap, missed, missed_total = self._check_feedback_freshness(
                label=label, phase=phase + "_dwell", previous_read_at=last_feedback_at,
                read_started=read_started, read_ended=position_monotonic,
                total_missed=missed_total, previous_tick_timing=previous_tick_timing)
            timing.update(feedback_gap_s=gap, missed_ticks=missed,
                          missed_ticks_total=missed_total,
                          position_read_duration_s=max(0.0, position_monotonic - read_started))
            poll_control(timing)
            self._guard_actual(actual, previous, start, active, recovery, started)
            for joint in JOINTS:
                reference = settled_raw[joint] if joint == "gripper" and contact_inferred else target[joint]
                tolerance = QUIET_COUNTS if joint == "gripper" and contact_inferred else ARRIVAL_COUNTS + QUIET_COUNTS
                if abs(actual[joint] - reference) > tolerance:
                    raise RuntimeError("dwell outside arrival tolerance: " + label + "/" + joint)
            goal_read_started = self.clock()
            goals = self._read("Goal_Position")
            timing["goal_read_duration_s"] = max(0.0, self.clock() - goal_read_started)
            poll_control(timing)
            if any(abs(goals[j] - commanded[j]) > 1 for j in JOINTS):
                raise RuntimeError("goal changed during dwell: " + label)
            torque_read_started = self.clock()
            torque = self._read("Torque_Enable")
            torque_monotonic = self.clock()
            torque_utc = self.wall_clock()
            timing["torque_read_duration_s"] = max(0.0, torque_monotonic - torque_read_started)
            poll_control(timing)
            publish_started = self.clock()
            self.publish_state(event="sample", phase=phase + "_dwell", label=label, raw=actual,
                               torque=torque, position_monotonic=position_monotonic,
                               position_utc=position_utc, torque_monotonic=torque_monotonic,
                               torque_utc=torque_utc, feedback_timing=timing)
            timing["sample_publish_duration_s"] = max(0.0, self.clock() - publish_started)
            previous_tick_timing = dict(timing)
            previous = actual
            last_feedback_at = position_monotonic
            next_frame = position_monotonic + PERIOD_S
            if missed:
                self._event("feedback_overrun", **timing,
                            label=label, phase=phase + "_dwell",
                            resynchronized=True,
                            max_gap_s=MAX_FEEDBACK_GAP_S,
                            max_missed_ticks_per_leg=MAX_MISSED_FEEDBACK_TICKS_PER_LEG)
            self.sleep(max(0.0, next_frame - self.clock()))
            if self.clock() - started >= LEG_DEADLINE_S:
                raise TimeoutError("leg 6 second deadline expired during dwell: " + label)
        self._event("leg_settled", label=label, target_raw=target, settled_raw=actual,
                    elapsed=self.clock() - started, command_raw=commanded,
                    gravity_trim_raw=trim, contact_inferred=contact_inferred,
                    grasp_confirmed=False)
        return dict(label=label, target_raw=target, settled_raw=actual,
                    elapsed=self.clock() - started, command_raw=commanded,
                    gravity_trim_raw=trim, contact_inferred=contact_inferred,
                    grasp_confirmed=False)

    def move_plan(self, legs):
        self._require_initialized()
        try:
            if not isinstance(legs, list) or not legs:
                raise ValueError("move_plan requires a nonempty list of raw pose legs")
            if len(legs) > 100:
                raise ValueError("move_plan may contain at most 100 legs")
        except ValueError as error:
            raise ArmPlanPreflightError(str(error)) from error
        # Parse and validate the complete plan against the current raw state
        # before any profile write or torque enable.
        start = self._read("Present_Position")
        try:
            prepared = []
            previous = start
            for index, leg in enumerate(legs):
                if not isinstance(leg, dict) or set(leg) - {"label", "target_raw", "grasp"}:
                    raise ValueError("each leg accepts only label, target_raw, and optional grasp")
                label = leg.get("label")
                if not isinstance(label, str) or not label.strip():
                    raise ValueError("each leg requires a nonempty label")
                grasp = leg.get("grasp", False)
                if type(grasp) is not bool:
                    raise ValueError("grasp must be a boolean")
                target = self._validate_target(leg.get("target_raw"), label)
                self._validate_transition(previous, target, label,
                                          allow_sag=self._sag_recovery_available and index == 0)
                prepared.append((label, target, grasp))
                previous = target
        except ValueError as error:
            if isinstance(error, ArmPlanPreflightError):
                raise
            raise ArmPlanPreflightError(str(error)) from error
        deadline = self.clock() + len(prepared) * LEG_DEADLINE_S + len(prepared) * DWELL_S + 1.0
        self._event("plan_start", legs=len(prepared))
        actual = start
        try:
            self._ensure_profile()
            records = []
            self._completed_leg_records = records
            for index, (label, target, grasp) in enumerate(prepared):
                if self.clock() >= deadline:
                    raise TimeoutError("move_plan deadline exceeded")
                self._poll_busy_input()
                self._active_leg = label
                record = self._run_leg(label, target, actual,
                                       allow_sag=self._sag_recovery_available and index == 0,
                                       grasp=grasp)
                records.append(record)
                self._active_leg = None
                actual = record["settled_raw"]
            self.last_command = self.clock()
            self._sag_recovery_available = False
            self._event("plan_complete", legs=len(prepared), final_raw=actual,
                        torque=self._read("Torque_Enable"))
            state = self.publish_state(event="plan_complete", phase="move_plan",
                                       label=prepared[-1][0])
            state["legs"] = records
            state["completed_legs"] = [label for label, _target, _grasp in prepared]
            return state
        except BaseException as error:
            if isinstance(error, ArmSessionCancelled) and self._cancel_reason == "trajectory_cancel_hold":
                state = self.cancel_hold()
                self._cancel_requested = False
                self._active_leg = None
                return {**state, "cancelled": True}
            self._event("plan_error", error=str(error), error_type=type(error).__name__)
            try:
                self.shutdown(reason="control_error")
            except BaseException as cleanup_error:
                error.add_note("arm session cleanup failed: " + str(cleanup_error))
            raise

    def move_gripper(self, target_raw):
        """Move the full jaw range in bounded legs under one cancellable command."""
        self._require_initialized()
        start = self._read("Present_Position")
        try:
            if type(target_raw) is not int:
                raise ValueError("move_gripper target_raw must be an integer")
            final = self._validate_target({**start, "gripper": target_raw}, "gripper_command")
            # Validate the held body/start without treating the complete jaw
            # span as one leg. Every emitted chunk is checked by _run_leg.
            self._validate_transition(start, start, "gripper_command")
        except ValueError as error:
            raise ArmPlanPreflightError(str(error)) from error
        chunks = max(1, math.ceil(abs(target_raw - start["gripper"]) /
                                  (GRIPPER_STEP - ARRIVAL_COUNTS - QUIET_COUNTS)))
        deadline = self.clock() + chunks * (LEG_DEADLINE_S + DWELL_S) + 1.0
        actual = start
        records = []
        self._completed_leg_records = records
        self._event("plan_start", phase="move_gripper", max_legs=chunks)
        try:
            self._ensure_profile()
            for _ in range(chunks):
                self._poll_busy_input()
                if self.clock() >= deadline:
                    raise TimeoutError("move_gripper deadline exceeded")
                remaining = target_raw - actual["gripper"]
                target = dict(final)
                target["gripper"] = actual["gripper"] + max(-GRIPPER_STEP, min(GRIPPER_STEP, remaining))
                label = "gripper_command"
                self._active_leg = label
                record = self._run_leg(label, target, actual, phase="move_gripper",
                                       grasp=remaining < 0, hold_on_contact=True)
                records.append(record)
                self._active_leg = None
                actual = record["settled_raw"]
                self._poll_busy_input()
                if record["contact_inferred"] or target["gripper"] == target_raw:
                    break
            else:
                raise RuntimeError("move_gripper exhausted bounded chunks without reaching target")
            self.last_command = self.clock()
            self._sag_recovery_available = False
            self._event("plan_complete", phase="move_gripper", legs=len(records), final_raw=actual)
            state = self.publish_state(event="plan_complete", phase="move_gripper", label=label)
            state["legs"] = records
            state["completed_legs"] = [record["label"] for record in records]
            return state
        except BaseException as error:
            if isinstance(error, ArmSessionCancelled) and self._cancel_reason == "trajectory_cancel_hold":
                state = self.cancel_hold()
                self._cancel_requested = False
                self._active_leg = None
                return {**state, "cancelled": True}
            self._event("plan_error", error=str(error), error_type=type(error).__name__)
            try:
                self.shutdown(reason="control_error")
            except BaseException as cleanup_error:
                error.add_note("arm session cleanup failed: " + str(cleanup_error))
            raise

    def _arm_radians(self, raw):
        result = {}
        for joint in ARM_JOINTS:
            entry = self.arm.calibration[joint]
            midpoint = (entry["range_min"] + entry["range_max"]) / 2
            result[joint] = (raw[joint] - midpoint) * (2 * math.pi / 4095)
        return result

    def _trajectory_raw(self, positions, gripper_raw):
        raw = {"gripper": gripper_raw}
        for joint, radians in zip(ARM_JOINTS, positions):
            entry = self.arm.calibration[joint]
            midpoint = (entry["range_min"] + entry["range_max"]) / 2
            raw[joint] = int(round(midpoint + radians * 4095 / (2 * math.pi)))
        return self._six_ints(raw, "trajectory sample")

    def cancel_hold(self):
        """Replace goals with measured pose while preserving torque state."""
        self._require_initialized()
        if self._component_scope:
            self._component_shutdown("component_cancel_disarm")
            return self.publish_state(event="component_cancel_disarmed")
        self._teleop = None
        measured = self._read("Present_Position")
        self.bus.sync_write("Goal_Position", measured, normalize=False, num_retry=0)
        goals = self._read("Goal_Position")
        if any(abs(goals[joint] - measured[joint]) > 1 for joint in JOINTS):
            raise RuntimeError("cancel hold goal readback mismatch")
        torque = self._read("Torque_Enable")
        state = self.publish_state(event="trajectory_cancel_hold", raw=measured, torque=torque)
        self.last_command = self.clock()
        self._event("trajectory_cancel_hold", raw=measured, torque=torque)
        return state

    def _teleop_start(self, command):
        if set(command) != {"command", "session_id"} or not isinstance(command["session_id"], str) or not command["session_id"]:
            raise ArmPlanPreflightError("teleop_start requires a nonempty session_id")
        if self._teleop is not None or self._motion_busy:
            raise ArmPlanPreflightError("arm session already has active motion")
        self._require_initialized()
        actual = self._read("Present_Position")
        actual = self._six_ints(actual, "teleop_start")
        actual_pose = {}
        limiter_start = {}
        tolerance = float(self.arm.settings["tolerance"])
        for joint in JOINTS:
            value = actual[joint]
            calibration = self.arm.calibration[joint]
            if (not 0 <= value <= 4095 or
                    not self.firmware_min[joint] <= value <= self.firmware_max[joint] or
                    not calibration["range_min"] <= value <= calibration["range_max"]):
                raise ArmPlanPreflightError(
                    "teleop_start measured pose is outside firmware or calibration limits: " + joint)
            if joint == "gripper":
                actual_pose[joint] = ((value - calibration["range_min"]) * 100 /
                                      (calibration["range_max"] - calibration["range_min"]))
            else:
                actual_pose[joint] = ((value - (calibration["range_min"] +
                                                   calibration["range_max"]) / 2) * 360 / 4095)
            limiter_start[joint] = max(self.admitted_min[joint],
                                       min(self.admitted_max[joint], value))
        outside_home = {joint: abs(actual_pose[joint] - self.home_pose[joint])
                        for joint in JOINTS
                        if abs(actual_pose[joint] - self.home_pose[joint]) > tolerance}
        if outside_home:
            joints = ", ".join(sorted(outside_home))
            raise ArmPlanPreflightError(
                f"teleop_start requires measured follower at saved home within {tolerance:g} "
                f"degrees/percent; outside on {joints}")
        # Install the exact measured pose as a hold target. A small configured
        # limit overshoot at saved home is permitted only for this initial hold;
        # the first commanded step starts from the admitted boundary so every
        # subsequent Goal_Position remains inside the unchanged limits.
        target = actual
        self._ensure_profile(teleop=True)
        torque = self._enable_current_goal(target, self.clock(), "teleop_start")
        torque_monotonic, torque_utc = self.clock(), self.wall_clock()
        self._teleop = {"session_id": command["session_id"], "sequence": -1,
                        "start": dict(actual), "previous": limiter_start,
                        "last": self.clock(), "last_valid": self.clock(),
                        "last_verify": self.clock(), "torque": torque,
                        "torque_monotonic": torque_monotonic,
                        "torque_utc": torque_utc}
        state = self.publish_state(event="teleop_started", raw=actual, torque=torque,
                                   torque_monotonic=torque_monotonic,
                                   torque_utc=torque_utc)
        if self._input_fd is not None:
            self._teleop_skip_next_sample = True
        return state

    def _teleop_step(self, command):
        if set(command) != {"command", "session_id", "sequence", "pose"}:
            raise ArmPlanPreflightError("teleop_step requires session_id, sequence, and pose")
        active = self._teleop
        if active is None or command["session_id"] != active["session_id"]:
            raise ArmPlanPreflightError("teleop session is not active")
        if self.clock() - active["last_valid"] > 0.30:
            self._expire_teleop_if_stale()
            raise ArmPlanPreflightError("teleop watchdog expired; measured pose held")
        sequence = command["sequence"]
        pose = command["pose"]
        if type(sequence) is not int or sequence <= active["sequence"]:
            raise ArmPlanPreflightError("teleop sequence must increase strictly")
        if not isinstance(pose, dict) or set(pose) != set(JOINTS):
            raise ArmPlanPreflightError("teleop pose requires exactly six named joints")
        if any(type(pose[j]) not in (int, float) or not math.isfinite(pose[j]) for j in JOINTS):
            raise ArmPlanPreflightError("teleop pose values must be finite numbers")
        if not 0 <= pose["gripper"] <= 100:
            raise ArmPlanPreflightError("teleop gripper must be between 0 and 100 percent")
        now = self.clock()
        target = {}
        for joint in JOINTS:
            entry = self.arm.calibration[joint]
            if joint == "gripper":
                requested = entry["range_min"] + pose[joint] * (entry["range_max"] - entry["range_min"]) / 100
            else:
                requested = (entry["range_min"] + entry["range_max"]) / 2 + pose[joint] * 4095 / 360
            low, high = self.admitted_min[joint], self.admitted_max[joint]
            bounded = max(low, min(high, requested))
            target[joint] = int(round(bounded))
        target = self._validate_target(target, "teleop_step")
        self.bus.sync_write("Goal_Position", target, normalize=False, num_retry=0)
        actual = self._read("Present_Position")
        # Keep the 100 Hz command path to one feedback packet per update. Goal
        # and torque readbacks remain mandatory, at 10 Hz, to catch dropped or
        # clamped writes and torque loss without multiplying bus traffic.
        if now - active["last_verify"] >= 0.10:
            goals = self._read("Goal_Position")
            if any(abs(goals[j] - target[j]) > 1 for j in JOINTS):
                raise RuntimeError("teleop goal readback mismatch")
            torque = self._read("Torque_Enable")
            if any(value != 1 for value in torque.values()):
                raise RuntimeError("teleop torque readback lost")
            active.update(last_verify=now, torque=torque,
                          torque_monotonic=self.clock(), torque_utc=self.wall_clock())
        active.update(sequence=sequence, previous=target, last=now, last_valid=now)
        state = self.publish_state(event="teleop_step", raw=actual, torque=active["torque"],
                                   torque_monotonic=active["torque_monotonic"],
                                   torque_utc=active["torque_utc"])
        if self._input_fd is not None:
            self._teleop_skip_next_sample = True
        return {"ok": True, "state": state, "sequence": sequence}

    def _teleop_stop(self, command):
        if set(command) != {"command", "session_id"}:
            raise ArmPlanPreflightError("teleop_stop requires session_id")
        if self._teleop is None or command["session_id"] != self._teleop["session_id"]:
            raise ArmPlanPreflightError("teleop session is not active")
        self._teleop = None
        return {"ok": True, "state": self.cancel_hold()}

    def _expire_teleop_if_stale(self):
        if self._teleop is None or self.clock() - self._teleop["last_valid"] <= 0.30:
            return None
        self._teleop = None
        return self.cancel_hold()

    def execute_trajectory(self, goal_id, joint_names, points, *,
                           max_velocity=ROS_MAX_VELOCITY_RAD_S,
                           max_acceleration=ROS_MAX_ACCELERATION_RAD_S2,
                           max_jerk=ROS_MAX_JERK_RAD_S3,
                           expected_path_sha256=None, period=PERIOD_S,
                           calibration_endpoint=False):
        """Execute one prevalidated C2 ROS trajectory without waypoint dwells."""
        self._require_initialized()
        if type(calibration_endpoint) is not bool:
            raise ArmPlanPreflightError("calibration_endpoint must be a boolean")
        if not isinstance(goal_id, str) or not goal_id.strip() or len(goal_id) > 128:
            raise ArmPlanPreflightError("goal_id must be a nonempty string up to 128 characters")
        if (max_velocity > ROS_MAX_VELOCITY_RAD_S or
                max_acceleration > ROS_MAX_ACCELERATION_RAD_S2 or
                max_jerk > ROS_MAX_JERK_RAD_S3):
            raise ArmPlanPreflightError("requested dynamics exceed the station software envelope")
        if (not isinstance(expected_path_sha256, str) or len(expected_path_sha256) != 64 or
                any(char not in "0123456789abcdef" for char in expected_path_sha256)):
            raise ArmPlanPreflightError("trajectory requires a collision-checked effective-path SHA-256")
        try:
            start_raw = self._read("Present_Position")
            start_time = self.clock()
            prepared = prepare_trajectory(
                joint_names, points, start_radians=self._arm_radians(start_raw),
                max_velocity=max_velocity, max_acceleration=max_acceleration,
                max_jerk=max_jerk,
                period=period)
            if path_digest(prepared, held_gripper_raw=start_raw["gripper"]) != expected_path_sha256:
                raise ArmPlanPreflightError("measured start or effective spline differs from collision-checked path")
            gripper_raw = start_raw["gripper"]
            raw_samples = [self._validate_target(self._trajectory_raw(sample.positions, gripper_raw),
                                                  f"trajectory/{index}")
                           for index, sample in enumerate(prepared.samples)]
            previous = start_raw
            for index, target_raw in enumerate(raw_samples):
                self._validate_transition(previous, target_raw, f"trajectory/{index}")
                previous = target_raw
        except (ValueError, KeyError, TypeError) as error:
            raise ArmPlanPreflightError(str(error)) from error

        sample_record = [{"time": sample.time,
                          "positions": dict(zip(ARM_JOINTS, sample.positions))}
                         for sample in prepared.samples]
        self.publish_state(event="trajectory_preflight", raw=start_raw,
                           phase="preflight", label=goal_id,
                           trajectory_goal_id=goal_id, trajectory_time=0.0,
                           trajectory_desired=dict(zip(ARM_JOINTS,
                               prepared.samples[0].positions)))
        self._event("trajectory_preflight_passed", goal_id=goal_id,
                    point_count=len(prepared.input_points), sample_count=len(sample_record),
                    original_points=list(prepared.input_points),
                    commanded_samples=sample_record,
                    derivatives_ignored=prepared.ignored_derivatives,
                    max_velocity_rad_s=max_velocity,
                    max_acceleration_rad_s2=max_acceleration,
                    max_jerk_rad_s3=max_jerk,
                    collision_checked_path_sha256=expected_path_sha256,
                    sample_period_s=period)
        previous_feedback = start_time
        feedback_samples = []
        motion_started = False
        try:
            self._ensure_profile()
            # Install measured pose before enabling torque; all validation has
            # completed and the first interpolation sample is exactly this pose.
            motion_started = True
            self._enable_current_goal(start_raw, start_time, "trajectory-start")
            started = self.clock()
            previous_feedback = started
            for index, (sample, target_raw) in enumerate(zip(prepared.samples, raw_samples)):
                self._poll_busy_input()
                deadline = started + sample.time
                wait = deadline - self.clock()
                if wait > 0:
                    self.sleep(wait)
                self._poll_busy_input()
                if self.clock() - deadline > MAX_FEEDBACK_GAP_S:
                    raise RuntimeError("trajectory command missed its scheduled tick")
                self._calibration_guard()
                self.bus.sync_write("Goal_Position", target_raw, normalize=False, num_retry=0)
                goal_read_started = self.clock()
                goals = self._read("Goal_Position")
                if any(abs(goals[joint] - target_raw[joint]) > 1 for joint in JOINTS):
                    raise RuntimeError("trajectory goal readback mismatch")
                actual = self._read("Present_Position")
                read_ended = self.clock()
                self._check_feedback_freshness(
                    label=goal_id, phase="trajectory", previous_read_at=previous_feedback,
                    read_started=goal_read_started, read_ended=read_ended,
                    total_missed=0, previous_tick_timing=None)
                previous_feedback = read_ended
                actual_radians = self._arm_radians(actual)
                for joint in ARM_JOINTS:
                    if abs(actual_radians[joint] - sample.positions[ARM_JOINTS.index(joint)]) > ROS_MAX_TRACK_ERROR_RAD:
                        raise RuntimeError("trajectory tracking error exceeded 10 degrees: " + joint)
                torque = self._read("Torque_Enable")
                if any(torque[joint] != 1 for joint in JOINTS):
                    raise RuntimeError("trajectory torque state changed unexpectedly")
                state = self.publish_state(event="trajectory_feedback", raw=actual,
                                           torque=torque, phase="trajectory", label=goal_id,
                                           trajectory_goal_id=goal_id,
                                           trajectory_time=sample.time,
                                           trajectory_desired=dict(zip(ARM_JOINTS, sample.positions)))
                feedback_samples.append({"time": sample.time, "monotonic": read_ended,
                                         "raw": actual, "goal_raw": target_raw,
                                         "state_timestamp_utc": state["timestamp_utc"]})
                self._poll_busy_input()

            # Keep commanding the terminal pose until measured state is within
            # tolerance for three consecutive observations or deadline expires.
            terminal = raw_samples[-1]
            settled = 0
            settle_deadline = self.clock() + 3.0
            settle_samples = []
            while settled < 3 and self.clock() < settle_deadline:
                self._poll_busy_input()
                self.bus.sync_write("Goal_Position", terminal, normalize=False, num_retry=0)
                actual = self._read("Present_Position")
                read_ended = self.clock()
                self._check_feedback_freshness(
                    label=goal_id, phase="trajectory-settle", previous_read_at=previous_feedback,
                    read_started=read_ended, read_ended=read_ended,
                    total_missed=0, previous_tick_timing=None)
                previous_feedback = read_ended
                errors = [abs(self._arm_radians(actual)[joint] -
                              prepared.samples[-1].positions[ARM_JOINTS.index(joint)])
                          for joint in ARM_JOINTS]
                settled = settled + 1 if max(errors) <= ROS_GOAL_TOLERANCE_RAD else 0
                torque = self._read("Torque_Enable")
                if any(torque[joint] != 1 for joint in JOINTS):
                    raise RuntimeError("trajectory torque state changed during settle")
                settle_samples.append({"monotonic": read_ended,
                                       "raw": dict(actual), "torque": dict(torque)})
                state = self.publish_state(event="trajectory_settle", raw=actual,
                                           torque=torque, phase="settling", label=goal_id,
                                           trajectory_goal_id=goal_id,
                                           trajectory_time=prepared.duration + self.clock() - (started + prepared.duration),
                                           trajectory_desired=dict(zip(ARM_JOINTS,
                                               prepared.samples[-1].positions)))
                feedback_samples.append({"time": prepared.duration + self.clock() - (started + prepared.duration),
                                         "monotonic": read_ended, "raw": actual,
                                         "goal_raw": terminal,
                                         "state_timestamp_utc": state["timestamp_utc"]})
                if settled < 3:
                    self.sleep(period)
            calibration_endpoint_held = False
            endpoint_error = {joint: abs(self._arm_radians(actual)[joint] -
                                         prepared.samples[-1].positions[ARM_JOINTS.index(joint)])
                              for joint in ARM_JOINTS}
            if settled < 3 and calibration_endpoint:
                window_start = settle_samples[-1]["monotonic"] - CALIBRATION_ENDPOINT_WINDOW_S
                stable_window = [sample for sample in settle_samples
                                 if sample["monotonic"] >= window_start]
                spans = {joint: max(sample["raw"][joint] for sample in stable_window) -
                         min(sample["raw"][joint] for sample in stable_window)
                         for joint in JOINTS} if stable_window else {}
                calibration_endpoint_held = (
                    len(stable_window) >= CALIBRATION_ENDPOINT_MIN_SAMPLES and
                    all(value == 1 for sample in stable_window
                        for value in sample["torque"].values()) and
                    all(span <= CALIBRATION_ENDPOINT_MAX_SPAN_COUNTS
                        for span in spans.values()) and
                    max(endpoint_error.values()) <= ROS_MAX_TRACK_ERROR_RAD)
            if settled < 3 and not calibration_endpoint_held:
                raise TimeoutError("trajectory final goal tolerance was not reached")
            if calibration_endpoint_held:
                # Preserve the planned terminal goal and enabled torque. The
                # caller may use this measured stable pose for calibration,
                # but it must not be represented as reaching the planned goal.
                state = self.publish_state(event="trajectory_calibration_endpoint_held",
                                           raw=actual, torque=torque,
                                           phase="calibration_endpoint_held", label=goal_id,
                                           trajectory_goal_id=goal_id,
                                           trajectory_time=prepared.duration +
                                               self.clock() - (started + prepared.duration),
                                           trajectory_desired=dict(zip(ARM_JOINTS,
                                               prepared.samples[-1].positions)))
                record = {"event": "trajectory_calibration_endpoint_held",
                          "goal_id": goal_id, "target_reached": False,
                          "goal_error_rad": endpoint_error,
                          "target_raw": terminal, "measured_raw": actual,
                          "stable_window_samples": len(stable_window),
                          "stable_window_s": (stable_window[-1]["monotonic"] -
                                              stable_window[0]["monotonic"]),
                          "stable_span_raw": spans, "feedback_samples": feedback_samples}
                self._record(record)
                self._sag_recovery_available = False
                self.last_command = self.clock()
                return {"state": state, "trajectory": {
                    "goal_id": goal_id, "duration_s": prepared.duration,
                    "sample_count": len(prepared.samples),
                    "feedback_count": len(feedback_samples),
                    "input_derivatives_ignored": prepared.ignored_derivatives,
                    "status": "calibration_measured_endpoint_held",
                    "target_reached": False, "goal_error_rad": endpoint_error}}
            state = self.publish_state(event="trajectory_complete", raw=actual,
                                       torque=torque, phase="complete", label=goal_id)
            self._record({"event": "trajectory_feedback_complete", "goal_id": goal_id,
                          "samples": feedback_samples})
            self._sag_recovery_available = False
            self.last_command = self.clock()
            return {"state": state, "trajectory": {
                "goal_id": goal_id, "duration_s": prepared.duration,
                "sample_count": len(prepared.samples),
                "feedback_count": len(feedback_samples),
                "input_derivatives_ignored": prepared.ignored_derivatives,
                "final_tolerance_rad": ROS_GOAL_TOLERANCE_RAD}}
        except ArmSessionCancelled as error:
            if self._cancel_reason == "trajectory_cancel_hold":
                try:
                    state = self.cancel_hold()
                except BaseException as hold_error:
                    try:
                        self.shutdown(reason="trajectory_cancel_hold_error")
                    except BaseException as cleanup_error:
                        hold_error.add_note("cancel hold cleanup failed: " + str(cleanup_error))
                    raise
                self._record({"event": "trajectory_cancelled", "goal_id": goal_id,
                              "feedback_samples": feedback_samples, "state": state})
                self._cancel_requested = False
                return {"cancelled": True, "state": state,
                        "trajectory": {"goal_id": goal_id, "feedback_count": len(feedback_samples)}}
            if motion_started:
                self.shutdown(reason=self._cancel_reason or str(error))
            raise
        except BaseException as error:
            if motion_started or self.saved_ram is not None:
                try:
                    self.shutdown(reason="trajectory_error")
                except BaseException as cleanup_error:
                    error.add_note("trajectory fault cleanup failed: " + str(cleanup_error))
            self._record({"event": "trajectory_error", "goal_id": goal_id,
                          "error": str(error), "error_type": type(error).__name__,
                          "feedback_samples": feedback_samples})
            raise

    def home(self):
        """Reach measured home through validated raw sublegs of at most 100 counts."""
        self._require_initialized()
        start = self._read("Present_Position")
        points = []
        max_delta = 100
        # Optional startup sag recovery is a separate first inward step to the
        # nearest firmware boundary. The remaining measured-home route starts
        # only after every joint is back inside its firmware range.
        recovery_target = dict(start)
        outside = {}
        for joint in JOINTS:
            value = start[joint]
            low, high = self.admitted_min[joint], self.admitted_max[joint]
            if not low <= value <= high:
                overrun = value - high if value > high else low - value
                if (not self._sag_recovery_available or joint == "gripper" or overrun > 16):
                    raise ValueError("actual raw position outside admitted configured limits: " + joint)
                recovery_target[joint] = high if value > high else low
                outside[joint] = True
        previous = dict(start)
        if outside:
            if any(abs(recovery_target[j] - start[j]) > max_delta for j in JOINTS):
                raise ValueError("sag recovery home leg exceeds 100 raw counts")
            self.arm._validate_raw(recovery_target)
            self._validate_transition(start, recovery_target, "home_sag_recovery", allow_sag=True)
            points.append(("home_sag_recovery", recovery_target, True))
            previous = recovery_target
        n = max(1, max(math.ceil(abs(self.home_raw[j] - previous[j]) / max_delta) for j in JOINTS))
        recovery_start = dict(previous)
        for step in range(1, n + 1):
            target = {j: int(round(recovery_start[j] + (self.home_raw[j] - recovery_start[j]) * step / n)) for j in JOINTS}
            for joint in JOINTS:
                if abs(target[joint] - previous[joint]) > max_delta:
                    raise ValueError("home subleg exceeds 100 counts")
            if any(not self.firmware_min[j] <= target[j] <= self.firmware_max[j] for j in JOINTS):
                raise ValueError("measured home path outside firmware limits")
            self.arm._validate_raw(target)
            self._validate_transition(previous, target, f"home_{step:03d}_of_{n:03d}")
            points.append((f"home_{step:03d}_of_{n:03d}", target, False))
            previous = target
        actual = start
        try:
            self._ensure_profile()
            self._completed_leg_records = []
            for label, target, is_recovery in points:
                self._active_leg = label
                record = self._run_leg(label, target, actual,
                                       allow_sag=is_recovery, phase="home")
                self._completed_leg_records.append(record)
                self._active_leg = None
                actual = record["settled_raw"]
            self.last_command = self.clock()
            self._sag_recovery_available = False
            state = self.publish_state(event="home_complete", phase="home",
                                       label=points[-1][0] if points else "home_already_there")
            state["completed_legs"] = [label for label, _target, _recovery in points]
            return state
        except BaseException as error:
            self._event("home_error", error=str(error), error_type=type(error).__name__)
            try:
                self.shutdown(reason="home_error")
            except BaseException as cleanup_error:
                error.add_note("arm session cleanup failed: " + str(cleanup_error))
            raise

    def _camera_recovery_check(self, raw, stamp, target=None):
        """Acquire/recheck trusted vision without accepting caller motion proof."""
        policy = self.camera_recovery_policy
        self._poll_busy_input()
        acquired = self.clock()
        scene = self.camera_scene_provider(dict(raw), stamp)
        self._poll_busy_input()
        if self.clock() - acquired > PERIOD_S:
            raise ValueError("camera scene provider exceeded nonblocking access budget")
        if scene is None or getattr(scene, "arm_stamp_monotonic", None) != stamp:
            raise ValueError("camera scene must match the exact owner encoder observation stamp")
        bounds = {j: (self.admitted_min[j], self.admitted_max[j]) for j in JOINTS}
        firmware = {j: (self.firmware_min[j], self.firmware_max[j]) for j in JOINTS}
        plan = plan_bound_reentry(raw, bounds, firmware, scene=scene, now=self.clock(),
            expected_model_sha256=policy.model_sha256,
            expected_calibration_sha256=policy.calibration_sha256,
            expected_camera_serial=policy.camera_serial)
        if target is not None:
            # Once an encoder enters the admitted interval, the planner's
            # no-excursion target is its current pose. Still prove the remaining
            # original inward segment through the two-count interior target.
            for clearance in (scene.minimum_clearance_m(dict(target)),
                              scene.minimum_segment_clearance_m(dict(raw), dict(target))):
                if type(clearance) not in (int, float) or not math.isfinite(clearance) or clearance < .010:
                    raise ValueError("remaining camera recovery segment clearance is unknown or insufficient")
        self._poll_busy_input()
        now = self.clock()
        if (now - stamp > MAX_FEEDBACK_GAP_S or now - acquired > MAX_FEEDBACK_GAP_S
                or not 0 <= now - scene.stamp_monotonic <= .5
                or not 0 <= now - scene.arm_stamp_monotonic <= .5):
            raise ValueError("camera recovery vision or encoder feedback expired while checking geometry")
        return plan

    def _camera_recovery_match(self, raw, stamp, scene_stamp, started):
        """Reject pose changes or expiry during proof before the next write."""
        self._poll_busy_input()
        if self._read("Present_Position") != raw:
            raise RuntimeError("camera recovery measured pose changed during scene proof")
        self._check_cancel()
        now = self.clock()
        if (now - stamp > MAX_FEEDBACK_GAP_S or not 0 <= now - scene_stamp <= .5):
            raise RuntimeError("camera recovery observation expired before goal update")
        if now - started >= LEG_DEADLINE_S:
            raise TimeoutError("camera recovery six-second operation deadline expired")

    def _camera_recovery_hold(self, error):
        if isinstance(error, ArmSessionCancelled) and self._cancel_reason != "trajectory_cancel_hold":
            self.shutdown(reason=self._cancel_reason or "camera_recovery_cancelled")
            raise error
        try:
            held = self.cancel_hold()
        except BaseException as hold_error:
            try:
                self.shutdown(reason="camera_recovery_hold_error")
            except BaseException as cleanup_error:
                hold_error.add_note("camera recovery hold cleanup failed: " + str(cleanup_error))
            raise
        self._active_leg = None
        cancelled = isinstance(error, ArmSessionCancelled)
        if cancelled:
            self._cancel_requested = False
        self._event("camera_recovery_held", error=str(error), cancelled=cancelled, raw=held["raw"])
        return {"ok": False, "state": held, "cancelled": cancelled,
                "recovery": {"status": "cancelled" if cancelled else "held", "error": str(error)}}

    def recover_camera(self):
        """Camera-checked ≤32-count soft-bound re-entry; no recovery-to-home.

        The synchronous provider is trusted, nonblocking in-process code. The
        six-second operation and provider/geometry elapsed checks apply after
        callbacks return; arbitrary blocking Python callbacks cannot be given
        a hard execution deadline by this bus owner.
        """
        self._require_initialized()
        policy = self.camera_recovery_policy
        if (not callable(self.camera_scene_provider) or not isinstance(policy, CameraRecoveryPolicy)):
            raise ArmPlanPreflightError("trusted camera scene provider and recovery policy required")
        if self._teleop is not None:
            raise ArmPlanPreflightError("camera recovery is unavailable during active teleop")
        started = self.clock()
        try:
            if policy.arm_calibration_sha256 != self.calibration_digest:
                raise ValueError("camera recovery policy does not match resident arm calibration")
            self._calibration_guard()
            if (self._read("Min_Position_Limit") != self.firmware_min
                    or self._read("Max_Position_Limit") != self.firmware_max):
                raise ValueError("firmware limits changed since owner admission")
            start = self._read("Present_Position")
            stamp = self.clock()
            plan = self._camera_recovery_check(start, stamp)
            target = self._validate_target(plan.target_raw, "camera_reentry")
            if target == start:
                raise ValueError("camera re-entry requires a measured soft-bound excursion")
        except ArmSessionCancelled as error:
            return self._camera_recovery_hold(error)
        except Exception as error:
            raise ArmPlanPreflightError(str(error)) from error
        self._event("camera_recovery_start", start_raw=start, target_raw=target,
                    path_sha256=plan.path_sha256)
        self._active_leg = "camera_reentry"
        self._completed_leg_records = []
        active = {j for j in JOINTS if start[j] != target[j]}
        previous = dict(start)
        previous_stamp = stamp
        missed_total = 0
        quiet_samples = 0
        try:
            self._ensure_profile()
            current = self._read("Present_Position")
            stamp = self.clock()
            if current != start:
                raise RuntimeError("measured camera recovery start changed during setup")
            checked = self._camera_recovery_check(current, stamp, target)
            self._check_leg_deadline(started, "camera_reentry")
            self._camera_recovery_match(current, stamp, checked.scene_stamp_monotonic, started)
            # Never enable torque against an old goal, including an old home
            # goal that would move this out-of-bounds arm without camera proof.
            self._enable_current_goal(current, started, "camera_reentry")
            commanded = dict(current)
            previous_stamp = self.clock()
            while True:
                self._check_leg_deadline(started, "camera_reentry")
                read_started = self.clock()
                actual = self._read("Present_Position")
                stamp = self.clock()
                self._poll_busy_input()
                gap, missed, missed_total = self._check_feedback_freshness(
                    label="camera_reentry", phase="camera_recovery", previous_read_at=previous_stamp,
                    read_started=read_started, read_ended=stamp, total_missed=missed_total,
                    previous_tick_timing=None)
                for joint in JOINTS:
                    value = actual[joint]
                    if not self.firmware_min[joint] <= value <= self.firmware_max[joint]:
                        raise RuntimeError("camera recovery observation outside firmware limits: " + joint)
                    frame_limit = FRAME_GRIPPER_COUNTS if joint == "gripper" else FRAME_BODY_COUNTS
                    if abs(value - previous[joint]) > frame_limit:
                        raise RuntimeError("camera recovery per-frame movement exceeded: " + joint)
                    if joint in active:
                        direction = 1 if target[joint] > start[joint] else -1
                        if (value - previous[joint]) * direction < 0:
                            raise RuntimeError("camera recovery observed outward movement: " + joint)
                        if abs(value - start[joint]) > 32 or (value - target[joint]) * direction > 2:
                            raise RuntimeError("camera recovery exceeded bounded inward path: " + joint)
                    elif abs(value - start[joint]) > QUIET_COUNTS:
                        raise RuntimeError("camera recovery held joint drift: " + joint)
                self._poll_busy_input()
                if self._read("Goal_Position") != commanded:
                    raise RuntimeError("camera recovery goal changed or was clamped")
                torque = self._read("Torque_Enable")
                if any(value != 1 for value in torque.values()):
                    raise RuntimeError("camera recovery torque state changed")
                checked = self._camera_recovery_check(actual, stamp, target)
                self._check_leg_deadline(started, "camera_reentry")
                self.publish_state(event="sample", raw=actual, torque=torque,
                    phase="camera_recovery", label="camera_reentry", position_monotonic=stamp,
                    position_utc=self.wall_clock(), feedback_timing={"feedback_gap_s": gap,
                        "missed_ticks": missed, "missed_ticks_total": missed_total})
                arrived = all(actual[j] == target[j] for j in active)
                quiet_samples = quiet_samples + 1 if arrived else 0
                if quiet_samples >= 3:
                    self._camera_recovery_match(actual, stamp, checked.scene_stamp_monotonic, started)
                    held = self.cancel_hold()
                    if held["raw"] != target:
                        raise RuntimeError("camera recovery measured terminal pose changed before hold")
                    self._active_leg = None
                    self._sag_recovery_available = False
                    state = self.publish_state(event="camera_recovery_complete", raw=held["raw"],
                        torque=held["torque"], phase="camera_recovery", label="camera_reentry")
                    self._event("camera_recovery_complete", raw=held["raw"], target_raw=target)
                    return {"ok": True, "state": state, "recovery": {"status": "completed",
                        "target_raw": target, "path_sha256": plan.path_sha256}}
                revised = dict(start)
                for joint in active:
                    delta = target[joint] - actual[joint]
                    revised[joint] = actual[joint] + max(-2, min(2, delta))
                # The provider and every remaining continuous segment have
                # been rechecked against this exact measured pose. Recheck
                # freshness once more immediately before the bounded write.
                self._camera_recovery_match(actual, stamp, checked.scene_stamp_monotonic, started)
                if revised != commanded:
                    self.bus.sync_write("Goal_Position", revised, normalize=False, num_retry=0)
                    self._poll_busy_input()
                    if self._read("Goal_Position") != revised:
                        raise RuntimeError("camera recovery goal readback mismatch")
                    commanded = revised
                previous = actual
                previous_stamp = stamp
                self.sleep(max(0., stamp + PERIOD_S - self.clock()))
        except BaseException as error:
            if not isinstance(error, Exception):
                self.shutdown(reason="camera_recovery_interrupted")
                raise
            return self._camera_recovery_hold(error)

    def _component_shutdown(self, reason):
        """Only ID6 writes, including after partial admission/disconnect/stop.

        A nonzero or unreadable body torque is a failure, never a reason to
        write the passive body or to claim an all-off acknowledgement.
        """
        failures = []
        torque = None
        try:
            if not self.bus.is_connected:
                raise ConnectionError("component bus disconnected; torque state unknown")
            self.bus.write("Torque_Enable", "gripper", 0, normalize=False, num_retry=0)
        except BaseException as error:
            failures.append(str(error))
        try:
            if not self.bus.is_connected:
                raise ConnectionError("component bus disconnected; readback unavailable")
            torque = self._read("Torque_Enable")
            if any(torque.values()):
                raise RuntimeError("component cleanup cannot confirm all six torque off")
        except BaseException as error:
            failures.append(str(error))
        # Restore only jaw RAM, and only after jaw-off was actually measured.
        if torque is not None and torque["gripper"] == 0 and self._component_saved_ram:
            for register, value in self._component_saved_ram.items():
                try:
                    self.bus.write(register, "gripper", value, normalize=False, num_retry=0)
                    if self.bus.read(register, "gripper", normalize=False, num_retry=0) != value:
                        raise RuntimeError("component RAM restore differs: " + register)
                    self._component_profile_readback[register] = value
                    self._event("component_profile_readback", register=register, raw_value=value, restoring=True)
                except BaseException as error:
                    failures.append(str(error))
            if not failures:
                self._component_saved_ram = None
        if not failures:
            try:
                raw = self._read("Present_Position")
                position_stamp, position_utc = self.clock(), self.wall_clock()
                torque = self._read("Torque_Enable")
                torque_stamp, torque_utc = self.clock(), self.wall_clock()
                if any(torque.values()):
                    raise RuntimeError("component final readback is not all-off")
                self.publish_state(event="torque_off", raw=raw, torque=torque,
                    position_monotonic=position_stamp, position_utc=position_utc,
                    torque_monotonic=torque_stamp, torque_utc=torque_utc)
            except BaseException as error:
                failures.append(str(error))
        self._event("component_shutdown", reason=reason, failures=failures, torque=torque)
        if failures:
            raise RuntimeError("component cleanup incomplete: " + "; ".join(failures))
        self._shutdown_complete = True
        return True

    def commission_gripper_step(self, delta_counts):
        """One raw-count jaw step; passive body is observed, never written.

        STS3215 RAM profile is deliberately fixed to the lowest nonzero raw
        velocity (physical units unqualified), acceleration 1, torque limit 50.
        No automatic retry, profile escalation or calibration update.
        """
        self._require_initialized()
        if type(delta_counts) is not int or not 0 < abs(delta_counts) <= 16:
            raise ArmPlanPreflightError("delta_counts must be a nonzero integer in [-16,16]")
        self._component_scope = True  # before any fallible I/O; persists after return
        self._shutdown_complete = False
        started = self.clock()
        self._component_command = dict(command_id=uuid.uuid4().hex, delta_counts=delta_counts,
            started_monotonic=started, deadline_monotonic=started+2.0, status="admitting",
            profile_raw={"Acceleration": 1, "Goal_Velocity": 1, "Torque_Limit": 50})
        baseline = None
        previous_at = started
        position_read_at = started
        enabled = False
        target = None

        def sample():
            nonlocal previous_at, position_read_at, baseline
            self._poll_busy_input()
            self._check_cancel()
            self._calibration_guard()
            if not self.bus.is_connected:
                raise ConnectionError("component bus disconnected")
            raw = self._read("Present_Position")
            position_stamp, position_utc = self.clock(), self.wall_clock()
            torque = self._read("Torque_Enable")
            torque_stamp, torque_utc = self.clock(), self.wall_clock()
            now = self.clock()
            # Preserve the actual failed observation before any guard can reject
            # it. A rejected sample must not disappear behind later all-off
            # cleanup snapshots or be mistaken for a missing hardware read.
            self._event("component_raw_observation", raw=raw, torque=torque,
                expected_gripper_torque=int(enabled),
                expected_body_torque=0,
                position_monotonic=position_stamp, torque_monotonic=torque_stamp,
                observed_monotonic=now, target_raw=target, elapsed_s=now-started)
            now = self.clock()  # include diagnostic I/O in admission timing
            if now - started >= 2.0:
                raise TimeoutError("component gripper step 2 second deadline exceeded")
            if now - previous_at > .100000001:
                raise RuntimeError("component feedback exceeded 100 ms")
            previous_at = torque_stamp
            position_read_at = position_stamp
            if now-position_stamp > .100000001:
                raise RuntimeError("component encoder admission feedback exceeded 100 ms")
            if any(not 0 <= value <= 4095 for value in raw.values()):
                raise RuntimeError("component raw encoder outside hardware range")
            if not self.admitted_min["gripper"] <= raw["gripper"] <= self.admitted_max["gripper"]:
                raise RuntimeError("component jaw actual outside ordinary admitted bounds")
            if any(torque[j] != 0 for j in JOINTS if j != "gripper"):
                raise RuntimeError("component passive body torque is not zero")
            if torque["gripper"] != int(enabled):
                raise RuntimeError("component gripper torque readback differs: "
                    f"expected={int(enabled)} actual={torque['gripper']} "
                    f"position_stamp={position_stamp:.9f} torque_stamp={torque_stamp:.9f}")
            if baseline is None:
                baseline = dict(raw)
            if any(abs(raw[j] - baseline[j]) > 2 for j in JOINTS if j != "gripper"):
                raise RuntimeError("component passive body drift exceeded 2 counts")
            if abs(raw["gripper"] - baseline["gripper"]) > (abs(delta_counts) + 2 if enabled else 2):
                raise RuntimeError("component jaw excursion exceeded bounded step")
            self.publish_state(event="commission_gripper_sample", raw=raw, torque=torque,
                position_monotonic=position_stamp, position_utc=position_utc,
                torque_monotonic=torque_stamp, torque_utc=torque_utc)
            self._event("commission_gripper_progress", target_raw=target, elapsed_s=now-started)
            return raw

        def write(register, value):
            # Refresh the passive guard before every potentially enabling write.
            sample()
            self._event("component_write_intent", register=register, requested_raw_value=value)
            # Snapshot/log I/O can block or receive a stop after sampling.
            # Recheck immediately before the single-shot hardware write.
            self._poll_busy_input()
            self._check_cancel()
            if self.clock() - started >= 2.0:
                raise TimeoutError("component gripper step 2 second deadline exceeded")
            if self.clock() - min(previous_at, position_read_at) > .100000001:
                raise RuntimeError("component write admission feedback exceeded 100 ms")
            write_started = self.clock()
            self.bus.write(register, "gripper", value, normalize=False, num_retry=0)
            write_ended = self.clock()
            actual = self.bus.read(register, "gripper", normalize=False, num_retry=0)
            read_ended = self.clock()
            self._event("component_write_readback", register=register,
                requested_raw_value=value, actual_raw_value=actual,
                actual_type=type(actual).__name__,
                write_started_monotonic=write_started, write_ended_monotonic=write_ended,
                readback_monotonic=read_ended)
            if actual != value:
                raise RuntimeError("component RAM readback differs: " + register +
                                   f" expected={value} actual={actual!r}")
            if register in RAM_REGISTERS:
                self._component_profile_readback[register] = value
                self._event("component_profile_readback", register=register, raw_value=value, restoring=False)

        try:
            for index in range(5):
                raw = sample()
                if index < 4:
                    self.sleep(PERIOD_S)
            target = raw["gripper"] + delta_counts
            self._component_command.update(start_raw=raw["gripper"], target_raw=target)
            if not (self.admitted_min["gripper"] <= raw["gripper"] <= self.admitted_max["gripper"] and
                    self.admitted_min["gripper"] <= target <= self.admitted_max["gripper"]):
                raise ArmPlanPreflightError("component jaw start/target outside ordinary admitted bounds")
            if self._component_saved_ram is not None:
                raise RuntimeError("previous component cleanup remains incomplete")
            self._component_saved_ram = {reg: self.bus.read(reg, "gripper", normalize=False, num_retry=0)
                                         for reg in RAM_REGISTERS}
            for register, value in (("Acceleration", 1), ("Goal_Velocity", 1), ("Torque_Limit", 50),
                                    ("Goal_Position", target)):
                write(register, value)
            write("Torque_Enable", 1)
            enabled = True
            self._component_command["status"] = "moving"
            quiet_since = None
            while True:
                raw = sample()
                if abs(raw["gripper"] - target) <= 1:
                    if quiet_since is None:
                        quiet_since = self.clock()
                    if self.clock() - quiet_since >= .2:
                        break
                else:
                    quiet_since = None
                self.sleep(PERIOD_S)
        except BaseException:
            self._component_command["status"] = "cancelled" if self._cancel_requested else "failed"
            raise
        finally:
            self._component_shutdown("commission_gripper_step")
        self._component_command.update(status="complete", completed_monotonic=self.clock())
        self._event("commission_gripper_complete", delta_counts=delta_counts, target_raw=target)
        return self.publish_state(event="commission_gripper_complete")

    def shutdown(self, *, reason="stop"):
        """Disable and verify each servo, then restore saved volatile RAM only."""
        if self._component_scope:
            return self._component_shutdown(reason)
        if self._shutdown_complete:
            return True
        if self.bus is None or not getattr(self.bus, "is_connected", False):
            return True
        failures = []
        for joint in JOINTS:
            try:
                self.bus.write("Torque_Enable", joint, 0, normalize=False, num_retry=0)
            except BaseException as error:
                failures.append(f"{joint} torque disable: {error}")
        torque = None
        try:
            torque = self._read("Torque_Enable")
            if any(value != 0 for value in torque.values()):
                raise RuntimeError("torque-off readback is not all zero")
        except BaseException as error:
            failures.append("torque-off verification: " + str(error))
        if torque is not None and all(value == 0 for value in torque.values()):
            if self.saved_ram is not None:
                for register, values in self.saved_ram.items():
                    for joint, value in values.items():
                        try:
                            if self.bus.read(register, joint, normalize=False, num_retry=0) != value:
                                self.bus.write(register, joint, value, normalize=False, num_retry=0)
                            if self.bus.read(register, joint, normalize=False, num_retry=0) != value:
                                raise RuntimeError("RAM restore readback differs")
                        except BaseException as error:
                            failures.append(f"{joint} {register} restore: {error}")
                if not failures:
                    self.saved_ram = None
                    self.profile = "unmodified"
        try:
            if torque is not None and all(value == 0 for value in torque.values()):
                self.publish_state(event="torque_off", torque=torque)
        except BaseException as error:
            failures.append("final state publish: " + str(error))
        self._event("shutdown", reason=reason, failures=failures, torque=torque)
        if failures:
            raise RuntimeError("arm session cleanup incomplete: " + "; ".join(failures))
        self._shutdown_complete = True
        return True

    def handle(self, command):
        if not isinstance(command, dict) or not isinstance(command.get("command"), str):
            raise ValueError("command must be a JSON object with string command")
        name = command["command"]
        if self._teleop is not None and name in ("move_plan", "move_gripper", "commission_gripper_step", "recover_camera", "home", "execute_trajectory"):
            raise ArmPlanPreflightError("finite motion is unavailable during active teleop")
        if name == "read_state":
            if set(command) != {"command"}:
                raise ValueError("read_state accepts no extra fields")
            return {"ok": True, "state": self.read_state()}
        if name == "move_plan":
            if set(command) != {"command", "legs"}:
                raise ValueError("move_plan requires exactly command and legs")
            return {"ok": True, "state": self.move_plan(command["legs"])}
        if name == "commission_gripper_step":
            if set(command) != {"command", "delta_counts"}:
                raise ArmPlanPreflightError("commission_gripper_step requires exactly command and delta_counts")
            return {"ok": True, "state": self.commission_gripper_step(command["delta_counts"])}
        if name == "move_gripper":
            if set(command) != {"command", "target_raw"}:
                raise ArmPlanPreflightError("move_gripper requires exactly command and target_raw")
            return {"ok": True, "state": self.move_gripper(command["target_raw"])}
        if name == "recover_camera":
            if set(command) != {"command"}:
                raise ArmPlanPreflightError("recover_camera accepts no caller target, path, clearance or policy")
            return self.recover_camera()
        if name == "execute_trajectory":
            required = {"command", "goal_id", "joint_names", "points", "path_sha256"}
            allowed = required | {"calibration_endpoint"}
            if set(command) not in (required, allowed):
                raise ValueError("execute_trajectory requires goal_id, joint_names, points, and collision-checked path_sha256")
            calibration_endpoint = command.get("calibration_endpoint", False)
            if type(calibration_endpoint) is not bool:
                raise ValueError("calibration_endpoint must be a boolean")
            result = self.execute_trajectory(command["goal_id"],
                    command["joint_names"], command["points"],
                    expected_path_sha256=command["path_sha256"],
                    calibration_endpoint=calibration_endpoint)
            response = {"ok": True, **result}
            if result.get("trajectory", {}).get("status") == "calibration_measured_endpoint_held":
                response.update(status=result["trajectory"]["status"],
                                target_reached=False,
                                goal_error_rad=result["trajectory"]["goal_error_rad"])
            return response
        if name == "cancel_hold":
            if set(command) != {"command"}:
                raise ValueError("cancel_hold accepts no extra fields")
            return {"ok": True, "state": self.cancel_hold()}
        if name == "teleop_start":
            return {"ok": True, "command": name, "session_id": command.get("session_id"),
                    "state": self._teleop_start(command)}
        if name == "teleop_step":
            return {**self._teleop_step(command), "command": name,
                    "session_id": command.get("session_id")}
        if name == "teleop_stop":
            return {**self._teleop_stop(command), "command": name,
                    "session_id": command.get("session_id")}
        if name == "home":
            if set(command) != {"command"}:
                raise ValueError("home accepts no extra fields")
            return {"ok": True, "state": self.home()}
        if name == "stop":
            request_id = self._stop_request_id(command)
            self.shutdown(reason="command_stop")
            self._teleop = None
            if request_id is not None:
                self.publish_state(event="torque_off")
            self.last_command = self.clock()
            return {"ok": True, "stopped": True,
                    **({"request_id": request_id} if request_id is not None else {}),
                    "state": self._latest_state(),
                    "completed_legs": [record["label"] for record in self._completed_leg_records]}
        if name == "exit":
            if set(command) != {"command"}:
                raise ValueError("exit accepts no extra fields")
            self.shutdown(reason="command_exit")
            return {"ok": True, "exit": True, "state": self._latest_state(),
                    "completed_legs": [record["label"] for record in self._completed_leg_records]}
        raise ValueError("unsupported arm session command: " + name)

    def _latest_state(self):
        try:
            return json.loads(self.state_file.read_text())
        except (OSError, json.JSONDecodeError):
            return None

    def serve(self, fd=0, *, emit=lambda value: print(json.dumps(value, allow_nan=False), flush=True),
              wait=select.select):
        """Poll stdin without blocking state telemetry or the idle watchdog."""
        self._require_initialized()
        self._input_fd = fd
        self._input_buffer = bytearray()
        self._emit = emit
        while True:
            now = self.clock()
            if self._teleop is not None and now - self._teleop["last_valid"] > 0.30:
                expired_session_id = self._teleop["session_id"]
                try:
                    state = self._expire_teleop_if_stale()
                    emit({"ok": False, "error": "teleop watchdog expired; measured pose held",
                          "command": "teleop_watchdog", "session_id": expired_session_id,
                          "state": state})
                except BaseException:
                    self.shutdown(reason="teleop_watchdog_hold_error")
                    raise
            if not self._idle_handled and now - self.last_command >= self.idle_timeout:
                self._event("idle_timeout", idle_seconds=now - self.last_command)
                self.shutdown(reason="idle_timeout")
                if not self.persistent_idle:
                    return 0
                self._idle_handled = True
            try:
                if self._teleop_skip_next_sample:
                    self._teleop_skip_next_sample = False
                else:
                    self.publish_state(event="sample")
            except BaseException as error:
                self._event("sample_error", error=str(error), error_type=type(error).__name__)
                self.shutdown(reason="sample_error")
                raise
            timeout = (PERIOD_S if self._idle_handled else
                       min(PERIOD_S, max(0.0, self.idle_timeout -
                                         (self.clock() - self.last_command))))
            ready, _, _ = wait([fd], [], [], timeout)
            if not ready:
                continue
            data = os.read(fd, 4096)
            if not data:
                self.shutdown(reason="stdin_eof")
                return 0
            self._input_buffer.extend(data)
            if len(self._input_buffer) > 1_048_576 and b"\n" not in self._input_buffer:
                self.shutdown(reason="oversized_command")
                raise ValueError("JSON-lines command exceeds 1 MiB")
            while b"\n" in self._input_buffer:
                line, _, remainder = self._input_buffer.partition(b"\n")
                self._input_buffer[:] = remainder
                if not line.strip():
                    continue
                if len(line) > 1_048_576:
                    self.shutdown(reason="oversized_command")
                    raise ValueError("JSON-lines command exceeds 1 MiB")
                try:
                    command = json.loads(line)
                    name = command.get("command") if isinstance(command, dict) else None
                    previous_idle_handled = self._idle_handled
                    if name in ("move_plan", "move_gripper", "commission_gripper_step", "recover_camera", "home", "execute_trajectory"):
                        self._motion_busy = True
                        self._cancel_requested = False
                        self._cancel_reason = None
                        self._cancel_request_id = None
                        self._exit_requested = False
                    previous_last_command = self.last_command
                    self.last_command = self.clock()
                    try:
                        result = self.handle(command)
                    except ArmPlanPreflightError as error:
                        # A plan rejected before motion is not a control fault.
                        # Preserve the idle deadline because the command was
                        # not admitted and leave the resident pose/profile held.
                        self.last_command = previous_last_command
                        self._idle_handled = previous_idle_handled
                        rejection = {"ok": False, "error": str(error),
                                     "error_type": type(error).__name__, "command": name}
                        if (name in ("teleop_start", "teleop_step", "teleop_stop") and
                                isinstance(command, dict) and
                                isinstance(command.get("session_id"), str)):
                            rejection["session_id"] = command["session_id"]
                        emit(rejection)
                        continue
                    except ArmSessionCancelled as error:
                        reason = self._cancel_reason or str(error)
                        self._motion_busy = False
                        self.shutdown(reason=reason)
                        self._teleop = None
                        try:
                            final_state = json.loads(self.state_file.read_text())
                        except BaseException:
                            final_state = None
                        malformed = reason.startswith("malformed_") or reason.startswith("oversized_")
                        emit({"ok": not malformed, "stopped": True, "reason": reason,
                              **({"request_id": self._cancel_request_id}
                                 if self._cancel_request_id is not None else {}),
                              "final_state": final_state,
                              "completed_legs": [record["label"] for record in self._completed_leg_records],
                              "interrupted_leg": self._active_leg})
                        self._cancel_requested = False
                        # Commands queued behind the abort were submitted while
                        # the bus was busy. Drop them so a buffered plan cannot
                        # unexpectedly re-arm immediately after stop/EOF.
                        self._input_buffer.clear()
                        if self._exit_requested or reason == "stdin_eof":
                            return 0
                        continue
                    finally:
                        self._motion_busy = False
                    emit(result)
                    self.last_command = self.clock()
                    if name in ("move_plan", "move_gripper", "commission_gripper_step", "recover_camera", "home", "execute_trajectory"):
                        self._idle_handled = False
                    elif name == "stop" and self.persistent_idle:
                        self._idle_handled = True
                        self._input_buffer.clear()
                    if isinstance(result, dict) and result.get("exit"):
                        return 0
                except BaseException as error:
                    self._event("command_error", error=str(error), error_type=type(error).__name__)
                    emit({"ok": False, "error": str(error), "error_type": type(error).__name__})
                    self.shutdown(reason="command_error")
                    raise

    def close_log(self):
        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None


def validate_arm_session_config(config):
    """Require arm identity/home references without admitting EV3 automation."""
    arm_settings = config["arm"]
    if not isinstance(arm_settings["id"], str) or not arm_settings["id"].strip():
        raise ValueError("arm ID required for resident session")
    usb = arm_settings["usb"]
    if (not isinstance(usb.get("serial"), str) or not usb["serial"].strip() or
            any(type(usb.get(field)) is not int or not 0 < usb[field] <= 65535
                for field in ("vendor_id", "product_id"))):
        raise ValueError("exact arm USB vendor, product, and serial identity required")
    calibration = arm_settings["calibration"]
    digest = calibration.get("sha256")
    if (not calibration.get("path") or not isinstance(digest, str) or len(digest) != 64 or
            any(character not in "0123456789abcdef" for character in digest)):
        raise ValueError("arm calibration file and lowercase SHA256 required")
    if not isinstance(arm_settings.get("home_file"), str) or not arm_settings["home_file"].strip():
        raise ValueError("measured arm home file required")
    return config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True,
                        help="station JSON with exact arm identity, calibration, and measured home")
    parser.add_argument("--state-file", type=Path, required=True, help="atomic latest arm state JSON")
    parser.add_argument("--logdir", type=Path, required=True, help="resident command/sample JSONL logs")
    parser.add_argument("--idle-timeout", type=float, default=60.0)
    parser.add_argument("--persistent-idle", action="store_true",
                        help="after the idle timeout, torque off once and keep publishing read-only state until exit")
    parser.add_argument("--recover-sag", action="store_true",
                        help="admit only <=16-count initial body sag moving inward")
    parser.add_argument("--lock", type=Path, default=station_lock_path())
    args = parser.parse_args(argv)
    if not math.isfinite(args.idle_timeout) or args.idle_timeout <= 0:
        parser.error("--idle-timeout must be finite and positive")
    try:
        # The resident tool controls only the already-identified arm. Requiring
        # a commissioned EV3 workflow here would reject valid arm-only sessions
        # whose global automatic-cycle gate remains deliberately disabled.
        config = load_config(args.config, physical=False)
        validate_arm_session_config(config)
        return run_locked_session(config, args.state_file, args.logdir,
                                  lock_path=args.lock,
                                  idle_timeout=args.idle_timeout,
                                  persistent_idle=args.persistent_idle,
                                  recover_sag=args.recover_sag)
    except KeyboardInterrupt:
        return 130
    except BaseException as error:
        print(json.dumps({"ok": False, "error": str(error),
                          "error_type": type(error).__name__}), file=sys.stderr, flush=True)
        return 1


def run_locked_session(config, state_file, logdir, *, lock_path,
                       idle_timeout=60.0, recover_sag=False, persistent_idle=False, fd=0,
                       emit=lambda value: print(json.dumps(value, allow_nan=False), flush=True),
                       lock_factory=station_lock, arm_factory=None,
                       session_factory=ArmSession):
    """Keep bus ownership locked through connect, serve, torque-off and RAM restore."""
    session = None
    failure = None
    with lock_factory(lock_path):
        try:
            if arm_factory is None:
                from .arm import LeRobotArm
                arm_factory = LeRobotArm
            arm = arm_factory(config)
            session = session_factory(arm, state_file, logdir,
                                      idle_timeout=idle_timeout,
                                      persistent_idle=persistent_idle,
                                      recover_sag=recover_sag)
            session.initialize()
            return session.serve(fd=fd, emit=emit)
        except BaseException as error:
            failure = error
            raise
        finally:
            if session is not None:
                try:
                    session.shutdown(reason="process_exit")
                except BaseException as cleanup_error:
                    if failure is not None:
                        failure.add_note("arm session cleanup failed: " + str(cleanup_error))
                    else:
                        raise
                finally:
                    try:
                        if session._component_scope:
                            # LeRobotArm.close() calls its global stop(), which
                            # writes all six servos independently of this owner.
                            # The SDK's explicit disable_torque=False path only
                            # closes the transport; component shutdown above owns
                            # verified jaw cleanup, even when that cleanup fails.
                            bus = session.bus
                            if bus is not None and bus.is_connected:
                                try:
                                    bus.disconnect(disable_torque=False)
                                except BaseException as disconnect_error:
                                    handler = getattr(bus, "port_handler", None)
                                    if handler is not None:
                                        try:
                                            handler.closePort()
                                        except BaseException as close_error:
                                            disconnect_error.add_note(str(close_error))
                                    raise
                        elif hasattr(session.arm, "close"):
                            session.arm.close()
                    except BaseException as close_error:
                        if failure is not None:
                            failure.add_note("arm bus disconnect failed: " + str(close_error))
                        else:
                            raise
                    finally:
                        session.close_log()


if __name__ == "__main__":
    raise SystemExit(main())
