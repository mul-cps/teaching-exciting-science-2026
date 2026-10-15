import hashlib
import json
import math
import select
from datetime import datetime, timedelta, timezone
import os
from contextlib import contextmanager
from pathlib import Path

import pytest

from workshop.station.arm_session import (ArmSession, JOINTS, PROFILE,
                                          run_locked_session, validate_arm_session_config)
from workshop.station.config import load_config
from workshop.station.ros_trajectory import (ROS_MAX_ACCELERATION_RAD_S2,
                                             ROS_MAX_JERK_RAD_S3,
                                             ROS_MAX_VELOCITY_RAD_S,
                                             path_digest, prepare_trajectory)


def _collision_path_digest(session, arm, names, points):
    prepared = prepare_trajectory(
        names, points, start_radians=session._arm_radians(arm.bus.sync_read("Present_Position")),
        max_velocity=ROS_MAX_VELOCITY_RAD_S,
        max_acceleration=ROS_MAX_ACCELERATION_RAD_S2,
        max_jerk=ROS_MAX_JERK_RAD_S3)
    return path_digest(prepared, held_gripper_raw=arm.bus.sync_read("Present_Position")["gripper"])


class FakeClock:
    def __init__(self): self.value = 0.0; self.sleeps = []
    def monotonic(self): return self.value
    def sleep(self, duration): self.sleeps.append(duration); self.value += max(0.001, duration)
    def utc(self): return datetime(2026, 10, 14, tzinfo=timezone.utc) + timedelta(seconds=self.value)


class FakeBus:
    def __init__(self, initial):
        self.is_connected = False
        self.is_calibrated = True
        self.motors = {joint: type("Motor", (), {"id": index})()
                       for index, joint in enumerate(JOINTS, 1)}
        self.data = {"Present_Position": dict(initial),
                     "Goal_Position": dict(initial),
                     "Min_Position_Limit": {j: 1100 for j in JOINTS},
                     "Max_Position_Limit": {j: 2900 for j in JOINTS},
                     "Torque_Enable": dict.fromkeys(JOINTS, 0),
                     "Acceleration": dict.fromkeys(JOINTS, 2),
                     "Goal_Velocity": dict.fromkeys(JOINTS, 3),
                     "Torque_Limit": dict.fromkeys(JOINTS, 4)}
        self.data["Max_Position_Limit"]["elbow_flex"] = 3055
        self.writes = []
        self.goal_history = []
        self.eeprom_writes = []
        self.clamp_goal = False
        self.drift_after_goal = None
        self._drift_pending = False
        self.blocked_gripper = False
        self.gripper_read_sequence = []
        self.stalled_joints = set()
        self.on_goal_write = None
        self.delay_after_goal = None
        self._delay_present_pending = False
        self.present_read_delays = []
        self.on_present_read = None
        self.clock = None
        self.present_read_times = []
        self.sync_read_calls = {}
        self.sync_read_failures = {}
        self.sync_read_failure_latency_s = 0.0
        self.sync_read_success_latency_s = 0.0

    def ping(self, joint, **kwargs): return 777
    def sync_read(self, register, **kwargs):
        self.sync_read_calls[register] = self.sync_read_calls.get(register, 0) + 1
        if self.sync_read_failures.get(register, 0):
            self.sync_read_failures[register] -= 1
            if self.clock is not None:
                self.clock.value += self.sync_read_failure_latency_s
            raise ConnectionError("Incorrect status packet")
        if self.clock is not None:
            self.clock.value += self.sync_read_success_latency_s
        if register == "Present_Position":
            if self.present_read_delays:
                self.clock.value += self.present_read_delays.pop(0)
            if self._delay_present_pending and self.delay_after_goal is not None:
                self.clock.value += self.delay_after_goal
                self._delay_present_pending = False
            for joint in JOINTS:
                current, goal = self.data[register][joint], self.data["Goal_Position"][joint]
                step = 100 if joint == "gripper" else 50
                if (self.data["Torque_Enable"][joint] == 0 or joint in self.stalled_joints or
                        (joint == "gripper" and self.blocked_gripper)):
                    continue
                self.data[register][joint] = current + max(-step, min(step, goal - current))
            if self._drift_pending:
                joint, amount = self.drift_after_goal
                self.data[register][joint] += amount
                self._drift_pending = False
            if self.gripper_read_sequence:
                self.data[register]["gripper"] = self.gripper_read_sequence.pop(0)
            self.present_read_times.append(self.clock.value if self.clock is not None else 0.0)
            if self.on_present_read is not None:
                self.on_present_read()
        return dict(self.data[register])
    def read(self, register, joint, **kwargs): return self.data[register][joint]
    def write(self, register, joint, value, **kwargs):
        self.writes.append((register, joint, value))
        self.data[register][joint] = value
    def sync_write(self, register, values, **kwargs):
        if register == "Goal_Position" and self.clamp_goal:
            values = dict(values)
            values["shoulder_pan"] -= 5
        self.data[register].update(values)
        if register == "Goal_Position" and self.drift_after_goal is not None:
            self._drift_pending = True
        if register == "Goal_Position": self.goal_history.append(dict(values))
        if register == "Goal_Position" and self.on_goal_write is not None:
            self.on_goal_write()
        if register == "Goal_Position" and self.delay_after_goal is not None:
            self._delay_present_pending = True


class FakeArm:
    def __init__(self, tmp_path, initial=None):
        self.settings = {"id": "rig", "home_file": str(tmp_path / "home.json"),
                         "calibration": {}, "limits": {}, "tolerance": 2}
        calibration = {}
        initial = initial or dict.fromkeys(JOINTS, 2000)
        for index, joint in enumerate(JOINTS, 1):
            calibration[joint] = {"id": index, "drive_mode": 0, "homing_offset": 0,
                                  "range_min": 1000,
                                  "range_max": 3055 if joint == "elbow_flex" else 3000}
            if joint == "gripper":
                lo, hi = 0., 100.
            else:
                lo, hi = (-80., 90.42) if joint == "elbow_flex" else (-80., 80.)
            self.settings["limits"][joint] = {"min": lo, "max": hi,
                                               "speed": 15., "acceleration": 25.}
        path = tmp_path / "rig.json"
        payload = json.dumps(calibration, sort_keys=True).encode()
        path.write_bytes(payload)
        digest = hashlib.sha256(payload).hexdigest()
        self.settings["calibration"] = {"path": str(path), "sha256": digest}
        self.settings["hz"] = 125
        self.bus = FakeBus(initial)
        self._cal = calibration
        self.connect_calls = 0
        home = {j: ((initial[j] - (calibration[j]["range_min"] + calibration[j]["range_max"]) / 2) * 360 / 4095 if j != "gripper"
                    else (initial[j] - 1000) * 100 / 2000) for j in JOINTS}
        # Default home is raw 2000 for every joint.
        home = {j: (50. if j == "gripper" else
                    (2000 - (calibration[j]["range_min"] + calibration[j]["range_max"]) / 2) * 360 / 4095)
                for j in JOINTS}
        (tmp_path / "home.json").write_text(json.dumps({
            "schema": 1, "kind": "measured-home", "arm_id": "rig",
            "calibration_sha256": digest,
            "units": "body degrees; gripper percent", "position": home}))

    def connect(self): self.connect_calls += 1; self.bus.is_connected = True; return True
    def _calibration(self): self.calibration = dict(self._cal); return self.calibration
    def _bounds(self):
        self.bounds = {j: (self.settings["limits"][j]["min"], self.settings["limits"][j]["max"])
                       for j in JOINTS}
    def _pose(self, pose):
        if set(pose) != set(JOINTS): raise ValueError("pose requires six joints")
        return dict(pose)
    def _raw_goal(self, pose):
        return {j: (int(pose[j] * 20 + 1000) if j == "gripper"
                    else int(pose[j] * 4095 / 360 + (self._cal[j]["range_min"] + self._cal[j]["range_max"]) / 2)) for j in JOINTS}
    def _validate_raw(self, raw):
        pose = {}
        for j in JOINTS:
            value = raw[j]
            if not 1000 <= value <= self._cal[j]["range_max"]: raise ValueError("raw calibration bounds")
            pose[j] = ((value - 1000) * 100 / (self._cal[j]["range_max"] - 1000)
                       if j == "gripper" else (value - (self._cal[j]["range_min"] + self._cal[j]["range_max"]) / 2) * 360 / 4095)
            lo, hi = self.bounds[j]
            if not lo <= pose[j] <= hi: raise ValueError("configured bounds")
        return dict(raw), pose


def make_session(tmp_path, *, initial=None, recover_sag=False, persistent_idle=False):
    clock = FakeClock()
    arm = FakeArm(tmp_path, initial)
    arm.bus.clock = clock
    session = ArmSession(arm, tmp_path / "actual.json", tmp_path / "logs",
                         clock=clock.monotonic, wall_clock=clock.utc,
                         sleep=clock.sleep, recover_sag=recover_sag,
                         persistent_idle=persistent_idle)
    return session, arm, clock


def target(**overrides):
    result = dict.fromkeys(JOINTS, 2000)
    result.update(overrides)
    return result


def test_sync_read_retries_transient_status_packet_with_diagnostics_and_no_writes(tmp_path):
    session, arm, clock = make_session(tmp_path)
    arm.bus.sync_read_failures["Torque_Enable"] = 1
    arm.bus.sync_read_failure_latency_s = 0.003

    state = session.initialize()

    diagnostic = session._read_diagnostics["Torque_Enable"]
    assert arm.bus.sync_read_calls["Torque_Enable"] == 2
    assert diagnostic["attempts"] == 2
    assert diagnostic["retries"] == 1
    assert diagnostic["success"] is True
    assert diagnostic["duration_s"] >= 0.003
    assert state["torque"] == dict.fromkeys(JOINTS, 0)
    assert state["bus_read_diagnostics"]["Torque_Enable"] == diagnostic
    assert clock.monotonic() < 0.020
    assert arm.bus.writes == []


def test_sync_read_persistent_packet_failure_is_bounded_and_reported(tmp_path):
    session, arm, _ = make_session(tmp_path)
    arm.bus.sync_read_failures["Torque_Enable"] = 10

    with pytest.raises(ConnectionError, match="after 3 attempts"):
        session._read("Torque_Enable")

    diagnostic = session._read_diagnostics["Torque_Enable"]
    assert arm.bus.sync_read_calls["Torque_Enable"] == 3
    assert diagnostic["attempts"] == 3
    assert diagnostic["retries"] == 2
    assert diagnostic["success"] is False
    assert arm.bus.writes == []


def test_sync_read_retry_stops_at_total_read_deadline(tmp_path):
    session, arm, clock = make_session(tmp_path)
    arm.bus.sync_read_failures["Torque_Enable"] = 10
    arm.bus.sync_read_failure_latency_s = 0.021

    with pytest.raises(ConnectionError, match="after 1 attempts"):
        session._read("Torque_Enable")

    assert arm.bus.sync_read_calls["Torque_Enable"] == 1
    assert session._read_diagnostics["Torque_Enable"]["deadline_exceeded"] is True
    assert clock.monotonic() >= 0.020
    assert arm.bus.writes == []


def test_sync_read_retry_success_past_deadline_is_rejected(tmp_path):
    session, arm, _ = make_session(tmp_path)
    arm.bus.sync_read_failures["Torque_Enable"] = 1
    arm.bus.sync_read_failure_latency_s = 0.011
    arm.bus.sync_read_success_latency_s = 0.011

    with pytest.raises(TimeoutError, match="retry deadline"):
        session._read("Torque_Enable")

    diagnostic = session._read_diagnostics["Torque_Enable"]
    assert diagnostic["attempts"] == 2
    assert diagnostic["success"] is False
    assert diagnostic["deadline_exceeded"] is True
    assert arm.bus.writes == []


def test_sync_write_failure_is_never_retried(tmp_path):
    session, arm, _ = make_session(tmp_path)
    calls = []

    def fail_write(register, joint, value, **kwargs):
        calls.append((register, joint, value))
        raise ConnectionError("write acknowledgement lost")

    arm.bus.write = fail_write
    with pytest.raises(ConnectionError, match="acknowledgement lost"):
        session._write_each("Goal_Position", target())
    assert len(calls) == 1


def test_connect_is_read_only_and_publishes_actual_state(tmp_path):
    session, arm, _ = make_session(tmp_path)
    state = session.initialize()
    assert arm.connect_calls == 1
    assert arm.bus.writes == []
    assert arm.bus.goal_history == []
    assert state["raw"] == dict.fromkeys(JOINTS, 2000)
    assert state["torque"] == dict.fromkeys(JOINTS, 0)
    latest = json.loads((tmp_path / "actual.json").read_text())
    assert latest["timestamp_utc"].endswith("+00:00")
    assert latest["monotonic"] == 0
    session.close_log()


def test_successful_plans_hold_torque_and_profile_until_verified_shutdown(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    first = session.move_plan([{"label": "vision-1", "target_raw": target(shoulder_lift=2050, gripper=1600)}])
    assert first["completed_legs"] == ["vision-1"]
    torque_after_first = arm.bus.sync_read("Torque_Enable")
    assert set(torque_after_first.values()) == {1}
    assert {register: arm.bus.sync_read(register) for register in PROFILE} == PROFILE
    first_goal = arm.bus.sync_read("Goal_Position")
    final = session.move_plan([{"label": "vision-2", "target_raw": target(shoulder_lift=2100, gripper=1700)}])
    assert final["completed_legs"] == ["vision-2"]
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {1}
    assert arm.bus.sync_read("Goal_Position") != first_goal
    assert clock.monotonic() > 0
    latest = json.loads((tmp_path / "actual.json").read_text())
    assert latest["raw"] == target(shoulder_lift=2100, gripper=1700)
    assert latest["profile"] == "native-8-160-220-torque-700-250"
    assert latest["profile_registers"] == PROFILE
    assert arm.bus.eeprom_writes == []
    session.shutdown()
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.sync_read("Goal_Velocity") == dict.fromkeys(JOINTS, 3)
    assert arm.bus.sync_read("Torque_Limit") == dict.fromkeys(JOINTS, 4)
    assert session.profile == "unmodified"
    session.close_log()


def test_plan_is_fully_validated_before_profile_or_torque_writes(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    with pytest.raises(ValueError, match="step budget"):
        session.move_plan([{"label": "too-far", "target_raw": target(shoulder_pan=2200)}])
    assert arm.bus.writes == []
    assert arm.bus.goal_history == []
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    session.close_log()


def _serve_invalid_plan_then_read_stop_exit(session, arm, clock, read_only,
                                            writes_before=None, goals_before=None):
    previous_command = session.last_command
    clock.value += 0.25
    invalid = {"command": "move_plan", "legs": [
        {"label": "wrist-out-of-range", "target_raw": target(wrist_flex=3024)}]}
    commands = [invalid, {"command": "read_state"},
                {"command": "stop"}, {"command": "exit"}]
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b"".join(json.dumps(value).encode() + b"\n"
                                           for value in commands))
    results = []
    rejected_snapshot = {}

    def emit(value):
        results.append(value)
        if value.get("command") == "move_plan":
            rejected_snapshot.update(
                writes=list(arm.bus.writes), goals=len(arm.bus.goal_history),
                torque=arm.bus.sync_read("Torque_Enable"), profile=session.profile,
                last_command=session.last_command)

    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
    finally:
        os.close(read_fd)
        os.close(write_fd)

    rejection = next(row for row in results if row.get("command") == "move_plan")
    state_reply = next(row for row in results if row.get("ok") and row.get("state", {}).get("event") == "read_state")
    assert rejection["ok"] is False
    assert rejection["error_type"] == "ArmPlanPreflightError"
    assert rejected_snapshot["last_command"] == previous_command
    if read_only:
        assert rejected_snapshot["writes"] == []
        assert rejected_snapshot["goals"] == 0
        assert set(rejected_snapshot["torque"].values()) == {0}
        assert rejected_snapshot["profile"] == "unmodified"
        assert state_reply["state"]["profile"] == "unmodified"
        assert set(state_reply["state"]["torque"].values()) == {0}
    else:
        assert rejected_snapshot["writes"] == writes_before
        assert rejected_snapshot["goals"] == goals_before
        assert set(rejected_snapshot["torque"].values()) == {1}
        assert rejected_snapshot["profile"] == "native-8-160-220-torque-700-250"
        assert set(state_reply["state"]["torque"].values()) == {1}
        assert state_reply["state"]["profile"] == "native-8-160-220-torque-700-250"
        stop_reply = next(row for row in results if row.get("ok") and row.get("stopped"))
        assert set(stop_reply["state"]["torque"].values()) == {0}
        assert session.profile == "unmodified"
    session.close_log()
    return results


def test_served_preflight_rejection_keeps_held_pose_until_stop(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.move_plan([{"label": "hold", "target_raw": target(shoulder_pan=2050)}])
    writes_before = list(arm.bus.writes)
    goals_before = len(arm.bus.goal_history)
    results = _serve_invalid_plan_then_read_stop_exit(
        session, arm, clock, read_only=False, writes_before=writes_before,
        goals_before=goals_before)
    assert len(arm.bus.goal_history) == goals_before
    assert any(row.get("ok") and row.get("state", {}).get("event") == "read_state"
               for row in results)


def test_served_preflight_rejection_keeps_startup_session_read_only(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    results = _serve_invalid_plan_then_read_stop_exit(session, arm, clock, read_only=True)
    assert arm.bus.goal_history == []
    assert all(register == "Torque_Enable" and value == 0
               for register, _joint, value in arm.bus.writes)
    assert any(row.get("ok") and row.get("state", {}).get("event") == "read_state"
               for row in results)


def test_goal_readback_fault_disables_and_restores_only_saved_ram(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.clamp_goal = True
    with pytest.raises(RuntimeError, match="Goal_Position readback mismatch"):
        session.move_plan([{"label": "readback-fault", "target_raw": target(shoulder_pan=2050)}])
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.sync_read("Goal_Velocity") == dict.fromkeys(JOINTS, 3)
    assert arm.bus.sync_read("Torque_Limit") == dict.fromkeys(JOINTS, 4)
    assert arm.bus.eeprom_writes == []
    session.close_log()


def test_inactive_joint_excursion_fault_disables_and_restores(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.drift_after_goal = ("wrist_roll", 41)
    with pytest.raises(RuntimeError, match="inactive joint drift"):
        session.move_plan([{"label": "nonactive-drift", "target_raw": target(shoulder_pan=2050)}])
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.eeprom_writes == []
    session.close_log()


def test_home_is_split_into_bounded_raw_sublegs(tmp_path):
    initial = dict.fromkeys(JOINTS, 2250)
    session, arm, _ = make_session(tmp_path, initial=initial)
    session.initialize()
    state = session.home()
    history = arm.bus.goal_history
    assert len(history) >= 3
    previous = initial
    for goal in history:
        assert max(abs(goal[j] - previous[j]) for j in JOINTS) <= 100
        previous = goal
    assert state["raw"] == dict.fromkeys(JOINTS, 2000)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {1}
    session.shutdown()
    session.close_log()


def test_sag_recovery_requires_explicit_flag_and_small_body_overrun(tmp_path):
    initial = dict.fromkeys(JOINTS, 2000)
    initial["elbow_flex"] = 3062
    reject_dir = tmp_path / "reject"
    reject_dir.mkdir()
    rejected, _, _ = make_session(reject_dir, initial=initial)
    with pytest.raises(ValueError, match="outside firmware"):
        rejected.initialize()
    rejected.close_log()

    recover_dir = tmp_path / "recover"
    recover_dir.mkdir()
    session, arm, _ = make_session(recover_dir, initial=initial, recover_sag=True)
    state = session.initialize()
    assert state["raw"]["elbow_flex"] == 3062
    assert {v["kind"] for v in state["violations"] if v["joint"] == "elbow_flex"} == {
        "calibration_range", "configured_pose_range", "firmware_range"}
    # A target inside firmware bounds and toward safety admits the first leg.
    destination = dict(initial, elbow_flex=3055)
    session.move_plan([{"label": "recover-inward", "target_raw": destination}])
    assert arm.bus.sync_read("Present_Position")["elbow_flex"] == 3055
    session.shutdown()
    session.close_log()


def test_home_sag_recovery_is_opt_in_and_stays_inside_admitted_limits(tmp_path):
    reject_dir = tmp_path / "home-reject"
    reject_dir.mkdir()
    rejected, rejected_arm, _ = make_session(reject_dir)
    rejected.initialize()
    outside = dict(rejected_arm.bus.data["Present_Position"])
    outside["elbow_flex"] = rejected.admitted_max["elbow_flex"] + 8
    rejected_arm.bus.data["Present_Position"] = dict(outside)
    with pytest.raises(ValueError, match="outside admitted configured limits"):
        rejected.home()
    rejected.shutdown()
    rejected.close_log()

    recover_dir = tmp_path / "home-recover"
    recover_dir.mkdir()
    recovered, recovered_arm, _ = make_session(recover_dir, recover_sag=True)
    recovered.initialize()
    outside = dict(recovered_arm.bus.data["Present_Position"])
    outside["elbow_flex"] = recovered.admitted_max["elbow_flex"] + 8
    recovered_arm.bus.data["Present_Position"] = dict(outside)
    recovered.home()
    assert recovered_arm.bus.goal_history[0]["elbow_flex"] == recovered.admitted_max["elbow_flex"]
    assert recovered_arm.bus.goal_history[0]["elbow_flex"] < outside["elbow_flex"]
    assert recovered_arm.bus.data["Present_Position"] == recovered.home_raw
    recovered.shutdown()
    recovered.close_log()


def test_eof_disables_torque_and_restores_ram(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    session.move_plan([{"label": "small", "target_raw": target(shoulder_pan=2050)}])
    read_fd, write_fd = os.pipe()
    os.close(write_fd)
    try:
        assert session.serve(fd=read_fd, emit=lambda _result: None) == 0
    finally:
        os.close(read_fd)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    session.close_log()


def test_idle_watchdog_keeps_twenty_hz_samples_then_disables_torque(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.idle_timeout = 0.16
    session.initialize()
    session.move_plan([{"label": "small", "target_raw": target(shoulder_pan=2050)}])
    session.last_command = clock.monotonic()
    sample_start = clock.monotonic()
    timestamps = []
    wait_calls = 0

    def fake_wait(readers, writers, errors, timeout):
        nonlocal wait_calls
        wait_calls += 1
        clock.sleep(timeout)
        return [], [], []

    initial_records = sum(1 for line in session.log_path.read_text().splitlines()
                          if json.loads(line).get("event") == "sample")
    assert session.serve(fd=0, emit=lambda _result: None, wait=fake_wait) == 0
    records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    samples = [record for record in records if record.get("event") == "sample"]
    timestamps = [record["monotonic"] for record in samples[initial_records:]]
    assert timestamps == pytest.approx([sample_start, sample_start + 0.05,
                                        sample_start + 0.10, sample_start + 0.15])
    assert wait_calls == 4
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    session.close_log()


def test_persistent_idle_torques_off_once_keeps_sampling_and_accepts_new_motion(tmp_path):
    session, arm, clock = make_session(tmp_path, persistent_idle=True)
    session.idle_timeout = 0.10
    session.initialize()
    read_fd, write_fd = os.pipe()
    emitted = []
    shutdown_reasons = []
    original_shutdown = session.shutdown

    def tracked_shutdown(*, reason="stop"):
        shutdown_reasons.append(reason)
        return original_shutdown(reason=reason)

    session.shutdown = tracked_shutdown
    initial_last_command = session.last_command
    stage = 0
    exit_sent = False

    def fake_wait(readers, writers, errors, timeout):
        nonlocal stage, exit_sent
        clock.sleep(timeout)
        idle_count = shutdown_reasons.count("idle_timeout")
        if idle_count == 1 and stage < 3:
            commands = [
                {"command": "read_state"},
                {"command": "move_plan", "legs": []},
                {"command": "move_plan", "legs": [
                    {"label": "resume-after-idle", "target_raw": target(shoulder_pan=2050)}]},
            ]
            if stage == 1:
                # A rejected request must leave the idle deadline untouched.
                fake_wait.invalid_deadline = session.last_command
            if stage == 2:
                assert session.last_command == fake_wait.invalid_deadline
                assert session._idle_handled is True
                assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
                assert sum(register == "Torque_Enable" and value == 0
                           for register, _joint, value in arm.bus.writes) == len(JOINTS)
            os.write(write_fd, (json.dumps(commands[stage]) + "\n").encode())
            stage += 1
        elif idle_count >= 2 and not exit_sent:
            exit_sent = True
            os.write(write_fd, b'{"command":"exit"}\n')
        return select.select(readers, writers, errors, 0)

    try:
        assert session.serve(fd=read_fd, emit=emitted.append, wait=fake_wait) == 0
    finally:
        os.close(read_fd)
        os.close(write_fd)
    records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    assert initial_last_command == 0.0
    assert shutdown_reasons.count("idle_timeout") == 2
    assert len(arm.bus.goal_history) == 1
    assert any(value.get("command") == "move_plan" and
               value.get("error_type") == "ArmPlanPreflightError" for value in emitted)
    assert any(value.get("ok") and value.get("state", {}).get("completed_legs") ==
               ["resume-after-idle"] for value in emitted)
    assert all(value == 0 for value in arm.bus.sync_read("Torque_Enable").values())
    assert sum(register == "Torque_Enable" and value == 0
               for register, _joint, value in arm.bus.writes) == 2 * len(JOINTS)
    first_idle = next(row["monotonic"] for row in records if row.get("event") == "idle_timeout")
    assert any(row.get("event") == "sample" and row["monotonic"] > first_idle
               for row in records)
    session.close_log()


def test_blocked_grasp_reports_contact_inference_and_keeps_pose_held(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.blocked_gripper = True
    result = session.move_plan([{"label": "grasp-cube", "grasp": True,
                                 "target_raw": target(shoulder_lift=2050, gripper=1500)}])
    assert result["completed_legs"] == ["grasp-cube"]
    assert result["legs"][0]["contact_inferred"] is True
    assert result["legs"][0]["grasp_confirmed"] is False
    assert result["legs"][0]["settled_raw"]["gripper"] == 2000
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {1}
    assert result["profile_registers"] == PROFILE
    session.shutdown()
    session.close_log()


def test_near_empty_gripper_closure_does_not_infer_contact_from_transient_excess(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.blocked_gripper = True
    # The actuator briefly sits 12 counts above the 1900 target, then settles
    # at the measured empty-close baseline of 1909. The transient is stable
    # long enough to satisfy the old 10-count arrival-tolerance heuristic.
    arm.bus.gripper_read_sequence = [1912] * 16 + [1909] * 20
    result = session.move_plan([{"label": "empty-close", "grasp": True,
                                 "target_raw": target(shoulder_lift=2050, gripper=1900)}])
    assert result["legs"][0]["settled_raw"]["gripper"] == 1909
    assert result["legs"][0]["contact_inferred"] is False
    assert result["legs"][0]["grasp_confirmed"] is False
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {1}
    session.shutdown()
    session.close_log()


def test_feedback_timestamps_follow_driver_reads_and_sleep_is_rate_limited(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    arm.bus.delay_after_goal = 0.02
    result = session.move_plan([{"label": "timed", "target_raw": target(shoulder_pan=2050)}])
    assert result["legs"][0]["label"] == "timed"
    records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    samples = [record for record in records if record.get("event") == "sample" and
               record.get("phase") == "move_plan"]
    assert samples
    assert all(record["monotonic"] in arm.bus.present_read_times for record in samples)
    assert all(record["torque_monotonic"] >= record["monotonic"] for record in samples)
    assert clock.sleeps and all(0 < duration <= 0.0500001 for duration in clock.sleeps)
    session.shutdown()
    session.close_log()


def test_single_210ms_feedback_gap_resynchronizes_with_three_missed_ticks(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.delay_after_goal = 0.21
    result = session.move_plan([{"label": "one-late-read",
                                 "target_raw": target(shoulder_pan=2050)}])
    assert result["completed_legs"] == ["one-late-read"]
    records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    overruns = [row for row in records if row.get("event") == "feedback_overrun"]
    assert len(overruns) == 1
    assert overruns[0]["missed_ticks"] == 3
    assert overruns[0]["missed_ticks_total"] == 3
    assert overruns[0]["resynchronized"] is True
    assert overruns[0]["feedback_gap_s"] == pytest.approx(0.21)
    samples = [row for row in records if row.get("event") == "sample" and
               row.get("label") == "one-late-read"]
    assert any(0 < right["monotonic"] - left["monotonic"] <= 0.06
               for left, right in zip(samples, samples[1:]))
    assert all("position_read_duration_s" in row.get("feedback_timing", {})
               for row in samples)
    session.shutdown()
    session.close_log()


@pytest.mark.parametrize(("delay", "message"), [
    (0.30, "feedback freshness exceeded bounded period budget"),
    (6.0, "leg 6 second deadline expired"),
])
def test_delayed_driver_reads_abort_with_torque_off_and_ram_restore(tmp_path, delay, message):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.delay_after_goal = delay
    with pytest.raises((RuntimeError, TimeoutError), match=message):
        session.move_plan([{"label": "delayed-driver", "target_raw": target(shoulder_pan=2050)}])
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.eeprom_writes == []
    if delay == 0.30:
        records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
        fault = next(row for row in records if row.get("event") == "feedback_cadence_fault")
        assert fault["feedback_gap_s"] == pytest.approx(delay)
        assert fault["position_read_duration_s"] == pytest.approx(delay)
        assert fault["missed_ticks"] == 5
        assert fault["previous_tick_timing"] is None
    session.close_log()


def test_repeated_subquarter_second_gaps_exhaust_per_leg_miss_budget(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.present_read_delays = [0.10] * 4
    with pytest.raises(RuntimeError, match="feedback freshness exceeded bounded period budget"):
        session.move_plan([{"label": "repeated-late-read",
                            "target_raw": target(shoulder_pan=2050)}])
    records = [json.loads(line) for line in session.log_path.read_text().splitlines()]
    fault = next(row for row in records if row.get("event") == "feedback_cadence_fault")
    assert fault["feedback_gap_s"] == pytest.approx(0.15)
    assert fault["missed_ticks_total"] > 4
    assert fault["previous_tick_timing"]["sample_publish_duration_s"] >= 0
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.eeprom_writes == []
    session.close_log()


def test_stop_while_feedback_read_is_late_is_serviced_before_gap_fault(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    read_fd, write_fd = os.pipe()
    plan = {"command": "move_plan", "legs": [
        {"label": "late-stop", "target_raw": target(shoulder_pan=2050)}]}
    os.write(write_fd, (json.dumps(plan) + "\n").encode())
    arm.bus.delay_after_goal = 0.21
    def stop_after_motion_read():
        if session._motion_busy and arm.bus.goal_history:
            arm.bus.on_present_read = None
            os.write(write_fd, b'{"command":"stop"}\n')
    arm.bus.on_present_read = stop_after_motion_read
    results = []

    def emit(value):
        results.append(value)
        if value.get("stopped"):
            os.write(write_fd, b'{"command":"exit"}\n')

    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
    finally:
        os.close(read_fd)
        os.close(write_fd)
    assert any(row.get("reason") == "busy_stop" for row in results)
    assert len(arm.bus.goal_history) == 1
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.eeprom_writes == []
    session.close_log()


def test_gravity_trim_stays_inside_firmware_calibration_and_pose_intersection(tmp_path):
    initial = dict.fromkeys(JOINTS, 2000)
    initial["shoulder_lift"] = 2850
    session, arm, _ = make_session(tmp_path, initial=initial)
    session.initialize()
    arm.bus.stalled_joints.add("shoulder_lift")
    target_raw = dict(initial, shoulder_lift=2900)
    with pytest.raises(TimeoutError, match="leg 6 second deadline"):
        session.move_plan([{"label": "trim-bound", "target_raw": target_raw}])
    assert arm.bus.goal_history
    assert all(goal["shoulder_lift"] <= session.admitted_max["shoulder_lift"]
               for goal in arm.bus.goal_history)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    session.close_log()


def test_station_lock_remains_held_through_torque_off_and_ram_restore(tmp_path):
    clock = FakeClock()
    arm = FakeArm(tmp_path)
    held = False
    writes_under_lock = []
    original_write = arm.bus.write

    def tracked_write(register, joint, value, **kwargs):
        assert held, "servo write escaped station lock"
        writes_under_lock.append((register, joint, value))
        return original_write(register, joint, value, **kwargs)

    arm.bus.write = tracked_write

    @contextmanager
    def tracked_lock(_path):
        nonlocal held
        assert not held
        held = True
        try: yield
        finally: held = False

    class MotionSession(ArmSession):
        def initialize(self):
            result = super().initialize()
            self.move_plan([{"label": "lock-check", "target_raw": target(shoulder_pan=2050)}])
            return result
        def serve(self, **_kwargs):
            return 0

    def session_factory(instance, state_path, log_path, **kwargs):
        return MotionSession(instance, state_path, log_path, clock=clock.monotonic,
                             wall_clock=clock.utc, sleep=clock.sleep, **kwargs)

    assert run_locked_session({}, tmp_path / "locked-state.json", tmp_path / "locked-log",
                              lock_path=tmp_path / "station.lock", lock_factory=tracked_lock,
                              arm_factory=lambda _config: arm,
                              session_factory=session_factory) == 0
    assert not held
    assert writes_under_lock
    assert all(value == 0 for value in arm.bus.sync_read("Torque_Enable").values())
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.sync_read("Goal_Velocity") == dict.fromkeys(JOINTS, 3)
    assert arm.bus.sync_read("Torque_Limit") == dict.fromkeys(JOINTS, 4)


def test_arm_only_config_admission_does_not_require_ev3_commissioning(tmp_path):
    arm = FakeArm(tmp_path)
    example = json.loads((Path(__file__).parents[1] /
                          "workshop/station/resources/station.example.json").read_text())
    example["arm"]["id"] = "rig"
    example["arm"]["usb"] = {"vendor_id": 6790, "product_id": 29987,
                               "serial": "FAKE-USB-IDENTITY"}
    example["arm"]["calibration"] = {"path": "rig.json",
                                      "sha256": arm.settings["calibration"]["sha256"]}
    example["arm"]["home_file"] = "home.json"
    config_path = tmp_path / "station.json"
    config_path.write_text(json.dumps(example))
    loaded = load_config(config_path, physical=False)
    assert validate_arm_session_config(loaded) is loaded
    assert loaded["enabled"] is False
    assert loaded["commissioned"] is False
    assert loaded["automatic_start"] is False
    assert loaded["arm"]["usb"]["serial"] == "FAKE-USB-IDENTITY"


def _serve_motion_with_injected_busy_input(tmp_path, injected):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    read_fd, write_fd = os.pipe()
    plan = {"command": "move_plan", "legs": [
        {"label": "first", "target_raw": target(shoulder_pan=2050)},
        {"label": "must-not-run", "target_raw": target(shoulder_pan=2100)}]}
    os.write(write_fd, (json.dumps(plan) + "\n").encode())

    def on_goal():
        arm.bus.on_goal_write = None
        injected(write_fd)

    arm.bus.on_goal_write = on_goal
    results = []

    def emit(value):
        results.append(value)
        if value.get("stopped"):
            # Recoverable stop/malformed cases leave the resident process
            # running; end the fixture's server after its cleanup response.
            if write_fd >= 0:
                try: os.write(write_fd, b'{"command":"exit"}\n')
                except OSError: pass

    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
    finally:
        os.close(read_fd)
        try: os.close(write_fd)
        except OSError: pass
    return session, arm, results


def test_stop_during_first_leg_aborts_before_second_goal_and_restores(tmp_path):
    def inject(fd):
        os.write(fd, b'{"command":"read_state"}\n')
        os.write(fd, b'{"command":"move_plan","legs":[]}\n')
        os.write(fd, b'{"command":"stop"}\n')
    session, arm, results = _serve_motion_with_injected_busy_input(tmp_path, inject)
    assert len(arm.bus.goal_history) == 1
    assert any(value.get("ok") and "state" in value for value in results)
    assert any(value.get("command") == "move_plan" and not value["ok"] for value in results)
    assert any(value.get("reason") == "busy_stop" for value in results)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    session.close_log()


def test_eof_during_motion_aborts_and_restores(tmp_path):
    def inject(fd):
        os.close(fd)
    session, arm, results = _serve_motion_with_injected_busy_input(tmp_path, inject)
    assert len(arm.bus.goal_history) == 1
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert any(value.get("reason") == "stdin_eof" for value in results)
    session.close_log()


def test_malformed_json_during_motion_aborts_and_restores(tmp_path):
    session, arm, results = _serve_motion_with_injected_busy_input(
        tmp_path, lambda fd: os.write(fd, b'{broken json}\n'))
    assert len(arm.bus.goal_history) == 1
    assert any(result.get("reason", "").startswith("malformed_stdin_command")
               and result.get("ok") is False for result in results)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    session.close_log()


def test_ros_trajectory_streams_without_waypoint_dwells_and_keeps_gripper(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    names = ["wrist_roll", "elbow_flex", "shoulder_pan", "wrist_flex", "shoulder_lift"]
    start = session._arm_radians(arm.bus.sync_read("Present_Position"))
    positions = [start[name] for name in names]
    points = [
        {"time": 0.0, "positions": positions, "velocities": [0.0] * 5,
         "accelerations": [0.8] * 5},
        {"time": 1.0, "positions": [value + 0.01 for value in positions], "velocities": [0.0] * 5,
         "accelerations": [-0.8] * 5},
    ]
    result = session.execute_trajectory("goal-test", names, points,
        expected_path_sha256=_collision_path_digest(session, arm, names, points))
    assert result["trajectory"]["sample_count"] == 21
    assert result["trajectory"]["input_derivatives_ignored"] is True
    assert arm.bus.eeprom_writes == []
    assert arm.bus.goal_history
    assert all(sample["gripper"] == 2000 for sample in arm.bus.goal_history)
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {1}
    assert arm.bus.sync_read("Present_Position")["shoulder_pan"] == result["state"]["raw"]["shoulder_pan"]
    session.shutdown(reason="test_cleanup")
    session.close_log()


def _stalled_elbow_trajectory(session, arm, *, goal_id, calibration_endpoint):
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    start = session._arm_radians(arm.bus.sync_read("Present_Position"))
    final = [start[name] for name in names]
    final[names.index("elbow_flex")] += 0.06
    points = [{"time": 0.0, "positions": [start[name] for name in names]},
              {"time": 2.0, "positions": final}]
    digest = _collision_path_digest(session, arm, names, points)
    command = {"command": "execute_trajectory", "goal_id": goal_id,
               "joint_names": names, "points": points, "path_sha256": digest}
    if calibration_endpoint is not None:
        command["calibration_endpoint"] = calibration_endpoint
    terminal = session._trajectory_raw(final, arm.bus.data["Present_Position"]["gripper"])
    return command, terminal


def test_calibration_trajectory_returns_stable_measured_endpoint_without_reaching_target(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    start = dict(arm.bus.data["Present_Position"])
    arm.bus.stalled_joints.add("elbow_flex")
    command, terminal = _stalled_elbow_trajectory(
        session, arm, goal_id="calibration-held", calibration_endpoint=True)
    result = session.handle(command)
    assert result["ok"] is True
    assert result["status"] == "calibration_measured_endpoint_held"
    assert result["target_reached"] is False
    assert result["goal_error_rad"]["elbow_flex"] > math.radians(2)
    assert result["state"]["raw"] == start
    assert arm.bus.data["Goal_Position"] == terminal
    assert arm.bus.sync_read("Torque_Enable") == dict.fromkeys(JOINTS, 1)
    records = [json.loads(line) for line in Path(session.log_path).read_text().splitlines()]
    event = next(row for row in records
                 if row.get("event") == "trajectory_calibration_endpoint_held" and
                 "target_reached" in row)
    assert event["target_reached"] is False
    assert event["measured_raw"] == start
    assert event["stable_window_samples"] >= 5
    assert max(event["stable_span_raw"].values()) <= 4
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_calibration_trajectory_rejects_unstable_measured_endpoint(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.stalled_joints.update(("elbow_flex", "shoulder_pan"))
    command, terminal = _stalled_elbow_trajectory(
        session, arm, goal_id="calibration-unstable", calibration_endpoint=True)
    alternate = False

    def wobble_near_terminal():
        nonlocal alternate
        if arm.bus.data["Goal_Position"]["elbow_flex"] == terminal["elbow_flex"]:
            alternate = not alternate
            arm.bus.data["Present_Position"]["shoulder_pan"] = 1997 if alternate else 2003

    arm.bus.on_present_read = wobble_near_terminal
    with pytest.raises(TimeoutError, match="final goal tolerance"):
        session.handle(command)
    assert arm.bus.data["Goal_Position"] == terminal
    assert arm.bus.sync_read("Torque_Enable") == dict.fromkeys(JOINTS, 0)
    session.close_log()


def test_calibration_endpoint_opt_in_does_not_change_default_tolerance_failure(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    arm.bus.stalled_joints.add("elbow_flex")
    command, _terminal = _stalled_elbow_trajectory(
        session, arm, goal_id="calibration-default-strict", calibration_endpoint=None)
    with pytest.raises(TimeoutError, match="final goal tolerance"):
        session.handle(command)
    assert arm.bus.sync_read("Torque_Enable") == dict.fromkeys(JOINTS, 0)
    session.close_log()


def test_calibration_endpoint_ipc_flag_requires_exact_boolean(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    command = {"command": "execute_trajectory", "goal_id": "bad-calibration-flag",
               "joint_names": [], "points": [], "path_sha256": "0" * 64,
               "calibration_endpoint": 1}
    with pytest.raises(ValueError, match="must be a boolean"):
        session.handle(command)
    assert arm.bus.writes == []
    assert arm.bus.goal_history == []
    session.close_log()


def test_ros_trajectory_preflight_rejection_has_no_writes(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    points = [{"time": 0.0, "positions": [0.2] * 5},
              {"time": 1.0, "positions": [0.21] * 5}]
    with pytest.raises(ValueError, match="start position"):
        session.execute_trajectory("goal-invalid", ["shoulder_pan", "shoulder_lift",
            "elbow_flex", "wrist_flex", "wrist_roll"], points,
            expected_path_sha256="0" * 64)
    assert arm.bus.writes == []
    assert arm.bus.goal_history == []
    assert arm.bus.eeprom_writes == []
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    session.close_log()


def test_ros_trajectory_collision_digest_mismatch_has_no_writes(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    start = session._arm_radians(arm.bus.sync_read("Present_Position"))
    points = [{"time": 0.0, "positions": [start[name] for name in names]},
              {"time": 1.0, "positions": [start[name] + 0.005 for name in names]}]
    checked_digest = _collision_path_digest(session, arm, names, points)
    points[1]["positions"][0] += 0.001
    with pytest.raises(ValueError, match="differs from collision-checked path"):
        session.execute_trajectory("goal-path-mismatch", names, points,
                                   expected_path_sha256=checked_digest)
    assert arm.bus.writes == []
    assert arm.bus.goal_history == []
    session.close_log()


def test_feedback_snapshots_do_not_append_to_durable_owner_log(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    log = session._log_stream
    before = log.tell()
    raw = arm.bus.sync_read("Present_Position")
    torque = arm.bus.sync_read("Torque_Enable")
    state = session.publish_state(event="trajectory_feedback", raw=raw, torque=torque)
    assert Path(session.state_file).is_file()
    assert state["event"] == "trajectory_feedback"
    assert log.tell() == before
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_cancel_hold_rewrites_measured_pose_without_disabling_torque(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    session.move_plan([{"label": "establish", "target_raw": target(shoulder_pan=2030)}])
    before = arm.bus.sync_read("Present_Position")
    torque_before = arm.bus.sync_read("Torque_Enable")
    arm.bus.goal_history.clear()
    result = session.cancel_hold()
    assert result["raw"] == before
    assert arm.bus.goal_history[-1] == before
    assert arm.bus.sync_read("Torque_Enable") == torque_before
    assert not any(register == "Torque_Enable" and value == 0
                   for register, _joint, value in arm.bus.writes)
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_maps_and_rate_limits_steps_and_rejects_stale_or_finite_motion(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    initial = dict(arm.bus.data["Present_Position"])
    started = session.handle({"command": "teleop_start", "session_id": "bridge-1"})
    assert started["ok"]
    assert arm.bus.data["Goal_Position"] == initial
    assert arm.bus.data["Torque_Enable"] == dict.fromkeys(JOINTS, 1)
    clock.sleep(0.1)
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["shoulder_pan"] = 30.0
    pose["gripper"] = 100.0
    result = session.handle({"command": "teleop_step", "session_id": "bridge-1",
                             "sequence": 1, "pose": pose})
    assert result["ok"]
    assert (result["command"], result["session_id"], result["sequence"]) == (
        "teleop_step", "bridge-1", 1)
    assert arm.bus.data["Goal_Position"]["shoulder_pan"] < initial["shoulder_pan"] + 70
    assert arm.bus.data["Goal_Position"]["gripper"] < arm.calibration["gripper"]["range_max"]
    with pytest.raises(ValueError, match="increase strictly"):
        session.handle({"command": "teleop_step", "session_id": "bridge-1",
                        "sequence": 1, "pose": pose})
    with pytest.raises(ValueError, match="finite motion"):
        session.handle({"command": "move_plan", "legs": []})
    stopped = session.handle({"command": "teleop_stop", "session_id": "bridge-1"})
    assert stopped["ok"]
    assert arm.bus.data["Goal_Position"] == arm.bus.data["Present_Position"]
    assert session._teleop is None
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_holds_small_saved_home_limit_overshoot_then_reenters_bounds(tmp_path):
    session, arm, clock = make_session(tmp_path)
    saved_home = json.loads(Path(arm.settings["home_file"]).read_text())["position"]
    arm.settings["limits"]["elbow_flex"]["max"] = saved_home["elbow_flex"]
    home_raw = arm._raw_goal(saved_home)
    actual = dict(arm.bus.data["Present_Position"])
    actual["elbow_flex"] = home_raw["elbow_flex"] + 10
    arm.bus.data["Present_Position"] = dict(actual)
    arm.bus.data["Goal_Position"] = dict(actual)
    session.initialize()
    assert actual["elbow_flex"] > session.admitted_max["elbow_flex"]
    started = session.handle({"command": "teleop_start", "session_id": "saved-home-overshoot"})
    assert started["ok"]
    assert arm.bus.data["Goal_Position"] == actual  # initial goal is an exact measured hold
    assert session._teleop["previous"]["elbow_flex"] == session.admitted_max["elbow_flex"]
    clock.sleep(.1)
    result = session.handle({"command": "teleop_step", "session_id": "saved-home-overshoot",
                             "sequence": 1, "pose": dict(saved_home)})
    assert result["ok"]
    assert arm.bus.data["Goal_Position"]["elbow_flex"] == session.admitted_max["elbow_flex"]
    assert arm.bus.data["Goal_Position"]["elbow_flex"] < actual["elbow_flex"]
    session.handle({"command": "teleop_stop", "session_id": "saved-home-overshoot"})
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_start_rejects_measured_pose_outside_saved_home_tolerance(tmp_path):
    session, arm, _clock = make_session(tmp_path)
    session.initialize()
    actual = dict(arm.bus.data["Present_Position"])
    actual["elbow_flex"] += 60
    arm.bus.data["Present_Position"] = actual
    arm.bus.goal_history.clear()
    with pytest.raises(ValueError, match="saved home"):
        session.handle({"command": "teleop_start", "session_id": "away-from-home"})
    assert not arm.bus.goal_history
    assert arm.bus.data["Torque_Enable"] == dict.fromkeys(JOINTS, 0)
    assert session._teleop is None
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_step_bus_read_budget_keeps_goal_and_torque_checks_periodic(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({"command": "teleop_start", "session_id": "read-budget"})
    before = dict(arm.bus.sync_read_calls)
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["gripper"] = 50.0
    for sequence in range(10):
        clock.sleep(0.025)
        session.handle({"command": "teleop_step", "session_id": "read-budget",
                        "sequence": sequence, "pose": pose})
    present_reads = arm.bus.sync_read_calls["Present_Position"] - before.get("Present_Position", 0)
    goal_reads = arm.bus.sync_read_calls["Goal_Position"] - before.get("Goal_Position", 0)
    torque_reads = arm.bus.sync_read_calls["Torque_Enable"] - before.get("Torque_Enable", 0)
    assert present_reads == 10
    assert 2 <= goal_reads <= 3
    assert goal_reads == torque_reads
    assert len(arm.bus.goal_history) >= 11  # start goal plus every update
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_watchdog_holds_measured_pose_and_invalidates_session(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({"command": "teleop_start", "session_id": "bridge-2"})
    actual = dict(arm.bus.data["Present_Position"])
    actual["wrist_roll"] += 9
    arm.bus.data["Present_Position"] = actual
    clock.sleep(0.301)
    state = session._expire_teleop_if_stale()
    assert state["raw"] == actual
    assert arm.bus.data["Goal_Position"] == actual
    assert session._teleop is None
    with pytest.raises(ValueError, match="not active"):
        session.handle({"command": "teleop_step", "session_id": "bridge-2",
                        "sequence": 1, "pose": dict.fromkeys(JOINTS, 0)})
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_stale_buffered_step_holds_and_cannot_refresh_session(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({"command": "teleop_start", "session_id": "bridge-3"})
    actual = dict(arm.bus.data["Present_Position"])
    actual["shoulder_lift"] += 7
    arm.bus.data["Present_Position"] = actual
    clock.sleep(0.301)
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["gripper"] = 50.0
    with pytest.raises(ValueError, match="watchdog expired"):
        session.handle({"command": "teleop_step", "session_id": "bridge-3",
                        "sequence": 1, "pose": pose})
    assert session._teleop is None
    assert arm.bus.data["Goal_Position"] == actual
    assert not any(register == "Goal_Position" and value != actual
                   for register, _joint, value in arm.bus.writes)
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_rejects_gripper_outside_percent_range_without_goal_write(tmp_path):
    session, arm, clock = make_session(tmp_path)
    session.initialize()
    session.handle({"command": "teleop_start", "session_id": "bridge-4"})
    before = len(arm.bus.goal_history)
    pose = dict.fromkeys(JOINTS, 0.0)
    pose["gripper"] = 101.0
    with pytest.raises(ValueError, match="between 0 and 100"):
        session.handle({"command": "teleop_step", "session_id": "bridge-4",
                        "sequence": 1, "pose": pose})
    assert len(arm.bus.goal_history) == before
    assert session._teleop["sequence"] == -1
    session.shutdown(reason="test_cleanup")
    session.close_log()


def test_resident_teleop_start_and_stop_through_json_stdio(tmp_path):
    session, arm, _clock = make_session(tmp_path)
    session.initialize()
    read_fd, write_fd = os.pipe()
    commands = [
        {"command": "teleop_start", "session_id": "stdio-bridge"},
        {"command": "teleop_stop", "session_id": "stdio-bridge"},
        {"command": "exit"},
    ]
    os.write(write_fd, b"".join((json.dumps(command) + "\n").encode()
                                 for command in commands))
    os.close(write_fd)
    results = []
    try:
        assert session.serve(read_fd, emit=results.append) == 0
    finally:
        os.close(read_fd)
        session.close_log()
    assert len(results) == 3
    assert results[0]["ok"] is True
    assert (results[0]["command"], results[0]["session_id"]) == (
        "teleop_start", "stdio-bridge")
    assert results[0]["state"]["event"] == "teleop_started"
    assert results[1]["ok"] is True
    assert (results[1]["command"], results[1]["session_id"]) == (
        "teleop_stop", "stdio-bridge")
    assert results[2]["exit"] is True
    assert session._teleop is None
    assert arm.bus.data["Torque_Enable"] == dict.fromkeys(JOINTS, 0)


def test_teleop_watchdog_and_stale_stdio_error_echo_session_id(tmp_path):
    session, _arm, clock = make_session(tmp_path)
    session.initialize()
    read_fd, write_fd = os.pipe()
    os.write(write_fd, (json.dumps({"command": "teleop_start", "session_id": "watchdog-1"}) + "\n").encode())
    calls = 0

    def wait(readers, writers, errors, timeout):
        nonlocal calls, write_fd
        calls += 1
        if calls == 2:
            clock.sleep(0.301)
            payload = [
                {"command": "teleop_step", "session_id": "watchdog-1",
                 "sequence": 1, "pose": dict.fromkeys(JOINTS, 0.0)},
                {"command": "exit"},
            ]
            os.write(write_fd, b"".join((json.dumps(item) + "\n").encode()
                                         for item in payload))
            os.close(write_fd)
            write_fd = -1
            return [], [], []
        return select.select(readers, writers, errors, timeout)

    results = []
    try:
        assert session.serve(read_fd, emit=results.append, wait=wait) == 0
    finally:
        os.close(read_fd)
        if write_fd >= 0:
            os.close(write_fd)
        session.close_log()
    watchdog = next(result for result in results
                    if result.get("command") == "teleop_watchdog")
    stale = next(result for result in results
                 if result.get("command") == "teleop_step")
    assert watchdog["session_id"] == "watchdog-1"
    assert stale["session_id"] == "watchdog-1"
    assert stale["ok"] is False


def test_ros_trajectory_feedback_gap_faults_and_restores_torque(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    start = session._arm_radians(arm.bus.sync_read("Present_Position"))
    points = [{"time": 0.0, "positions": [start[j] for j in (
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]},
        {"time": 1.0, "positions": [start[j] + 0.01 for j in (
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]}]
    arm.bus.delay_after_goal = 0.30
    with pytest.raises(RuntimeError, match="feedback freshness"):
        names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        session.execute_trajectory("goal-stale", names, points,
            expected_path_sha256=_collision_path_digest(session, arm, names, points))
    assert set(arm.bus.sync_read("Torque_Enable").values()) == {0}
    assert arm.bus.sync_read("Acceleration") == dict.fromkeys(JOINTS, 2)
    assert arm.bus.eeprom_writes == []
    session.close_log()
