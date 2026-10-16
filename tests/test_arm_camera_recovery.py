"""Resident-only camera re-entry; fake transport, trusted scene acquisition."""
import os

import pytest

from test_arm_session import FakeArm, FakeClock, JOINTS, make_session, target
from workshop.station.arm_session import ArmPlanPreflightError, ArmSession, CameraRecoveryPolicy


class Scene:
    registration_qualified = True
    model_sha256 = "a" * 64
    calibration_sha256 = "b" * 64
    camera_serial = "qualified-test-camera"

    def __init__(self, raw, stamp):
        self.arm_raw = dict(raw)
        self.arm_stamp_monotonic = stamp
        self.stamp_monotonic = stamp
        self.clearance = .020
        self.segments = []

    def minimum_clearance_m(self, raw):
        return self.clearance

    def minimum_segment_clearance_m(self, start, end):
        self.segments.append((dict(start), dict(end)))
        return self.clearance


def recovery_session(tmp_path, *, provider=None):
    clock = FakeClock()
    arm = FakeArm(tmp_path, initial=target(elbow_flex=2993))
    arm.bus.clock = clock
    # Calibrated/configured high=2975, firmware high=3055: an 18-count
    # soft-limit excursion, beyond the unchanged 16-count ordinary sag gate.
    arm.settings["limits"]["elbow_flex"]["max"] = (2975.1 - 2027.5) * 360 / 4095
    scenes = []
    def acquire(raw, stamp):
        value = Scene(raw, stamp)
        scenes.append(value)
        return provider(value, len(scenes), session, arm, clock) if provider else value
    policy = CameraRecoveryPolicy(
        model_sha256="a" * 64, calibration_sha256="b" * 64,
        camera_serial="qualified-test-camera", arm_calibration_sha256=arm.settings["calibration"]["sha256"])
    session = ArmSession(arm, tmp_path / "actual.json", tmp_path / "logs",
        clock=clock.monotonic, wall_clock=clock.utc, sleep=clock.sleep,
        camera_scene_provider=acquire, camera_recovery_policy=policy)
    session.initialize()
    return session, arm, clock, scenes


def close(session):
    session.shutdown()
    session.close_log()


def test_camera_reentry_recovers_18_count_overrun_without_relaxing_normal_home(tmp_path):
    session, arm, clock, scenes = recovery_session(tmp_path)
    try:
        with pytest.raises(ValueError, match="outside admitted"):
            session.home()
        assert arm.bus.goal_history == []
        result = session.handle({"command": "recover_camera"})
        assert result["ok"], result
        assert result["state"]["event"] == "camera_recovery_complete"
        assert result["state"]["raw"]["elbow_flex"] == 2973
        assert arm.bus.goal_history[0] == target(elbow_flex=2993)
        previous = 2993
        for goal in arm.bus.goal_history[1:]:
            assert 0 <= previous - goal["elbow_flex"] <= 2
            assert all(goal[j] == 2000 for j in JOINTS if j != "elbow_flex")
            previous = goal["elbow_flex"]
        assert previous == 2973
        assert len(scenes) >= len(arm.bus.goal_history)
        assert any((start["elbow_flex"], end["elbow_flex"]) == (2975, 2973)
                   for scene in scenes for start, end in scene.segments)
        assert clock.monotonic() <= 6.0
        assert session.admitted_max["elbow_flex"] == 2975
        assert session.firmware_max["elbow_flex"] == 3055
        assert arm.bus.eeprom_writes == []
    finally:
        close(session)


def test_default_constructor_has_no_camera_recovery_authority(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    try:
        with pytest.raises(ArmPlanPreflightError, match="provider"):
            session.handle({"command": "recover_camera"})
        assert arm.bus.writes == []
        assert arm.bus.goal_history == []
    finally:
        close(session)


@pytest.mark.parametrize("field,value", [
    ("stamp_monotonic", -1.), ("arm_raw", target(elbow_flex=2992)),
    ("arm_stamp_monotonic", -.001), ("model_sha256", "c" * 64),
    ("calibration_sha256", "c" * 64), ("camera_serial", "other-camera"),
    ("registration_qualified", False), ("clearance", None), ("clearance", .001)])
def test_bad_initial_camera_proof_cannot_write_goals_or_profile(tmp_path, field, value):
    def provider(scene, *_):
        setattr(scene, field, value)
        return scene
    session, arm, _, _ = recovery_session(tmp_path, provider=provider)
    try:
        with pytest.raises(ArmPlanPreflightError):
            session.handle({"command": "recover_camera"})
        assert arm.bus.writes == []
        assert arm.bus.goal_history == []
    finally:
        close(session)


@pytest.mark.parametrize("extra", [{"target_raw": 2973}, {"clearance": True},
                                   {"path": []}, {"policy": {}}])
def test_json_cannot_supply_recovery_target_path_clearance_or_identity(tmp_path, extra):
    session, arm, _, _ = recovery_session(tmp_path)
    try:
        with pytest.raises(ArmPlanPreflightError):
            session.handle({"command": "recover_camera", **extra})
        assert arm.bus.goal_history == []
        assert arm.bus.writes == []
    finally:
        close(session)


@pytest.mark.parametrize("fault", ["obstacle", "stale", "provider_slow", "geometry_slow", "pose_mismatch"])
def test_changed_scene_stops_remaining_motion_with_confirmed_measured_hold(tmp_path, fault):
    def provider(scene, index, session, arm, clock):
        if len(arm.bus.goal_history) >= 3:
            if fault == "obstacle":
                scene.clearance = None
            elif fault == "stale":
                scene.stamp_monotonic -= 1.
            elif fault == "provider_slow":
                clock.value += .3
            elif fault == "geometry_slow":
                def slow_clearance(raw):
                    clock.value += .3
                    return .020
                scene.minimum_clearance_m = slow_clearance
            else:
                scene.arm_raw["elbow_flex"] -= 1
        return scene
    session, arm, _, _ = recovery_session(tmp_path, provider=provider)
    try:
        result = session.handle({"command": "recover_camera"})
        assert not result["ok"]
        assert result["state"]["event"] == "trajectory_cancel_hold"
        assert result["recovery"]["status"] == "held"
        assert arm.bus.data["Goal_Position"] == arm.bus.data["Present_Position"]
        assert arm.bus.data["Goal_Position"]["elbow_flex"] > 2973
        assert len(arm.bus.goal_history) == 4
        assert set(arm.bus.data["Torque_Enable"].values()) == {1}
    finally:
        close(session)


def test_encoder_outward_motion_is_held_before_another_recovery_update(tmp_path):
    session, arm, _, _ = recovery_session(tmp_path)
    def outward_motion():
        if len(arm.bus.goal_history) == 2:
            arm.bus.on_present_read = None
            arm.bus.data["Present_Position"]["elbow_flex"] = 2994
            arm.bus.stalled_joints.add("elbow_flex")
    arm.bus.on_present_read = outward_motion
    try:
        result = session.handle({"command": "recover_camera"})
        assert not result["ok"]
        assert "outward" in result["recovery"]["error"]
        assert len(arm.bus.goal_history) == 3
        assert arm.bus.data["Goal_Position"] == target(elbow_flex=2994)
    finally:
        close(session)


@pytest.mark.parametrize("phase", ["setup", "motion"])
def test_pose_changed_during_scene_proof_is_held_before_torque_or_next_goal(tmp_path, phase):
    trigger = 2 if phase == "setup" else 3
    def provider(scene, index, _session, arm, _clock):
        if index == trigger:
            arm.bus.data["Present_Position"]["elbow_flex"] = 2994
            arm.bus.stalled_joints.add("elbow_flex")
        return scene
    session, arm, _, _ = recovery_session(tmp_path, provider=provider)
    try:
        result = session.handle({"command": "recover_camera"})
        assert not result["ok"]
        assert [goal["elbow_flex"] for goal in arm.bus.goal_history] == (
            [2994] if phase == "setup" else [2993, 2994])
        assert set(arm.bus.data["Torque_Enable"].values()) == ({0} if phase == "setup" else {1})
    finally:
        close(session)


def test_stalled_inward_recovery_has_a_six_second_deadline_then_holds(tmp_path):
    session, arm, clock, _ = recovery_session(tmp_path)
    arm.bus.stalled_joints.add("elbow_flex")
    try:
        result = session.handle({"command": "recover_camera"})
        assert not result["ok"]
        assert "deadline" in result["recovery"]["error"]
        assert 6 <= clock.monotonic() < 6.1
        assert [goal["elbow_flex"] for goal in arm.bus.goal_history] == [2993, 2991, 2993]
        assert arm.bus.data["Goal_Position"] == arm.bus.data["Present_Position"]
    finally:
        close(session)


def test_camera_recovery_cancel_before_setup_confirms_hold_without_arming(tmp_path):
    session, arm, _, _ = recovery_session(tmp_path)
    session._cancel("trajectory_cancel_hold")
    try:
        result = session.handle({"command": "recover_camera"})
        assert result["cancelled"]
        assert result["state"]["event"] == "trajectory_cancel_hold"
        assert arm.bus.goal_history == [target(elbow_flex=2993)]
        assert arm.bus.writes == []
        assert set(arm.bus.data["Torque_Enable"].values()) == {0}
    finally:
        close(session)


def test_stale_encoder_feedback_stops_before_first_inward_goal(tmp_path):
    session, arm, _, _ = recovery_session(tmp_path)
    arm.bus.delay_after_goal = .3
    try:
        result = session.handle({"command": "recover_camera"})
        assert not result["ok"]
        assert "freshness" in result["recovery"]["error"]
        assert arm.bus.goal_history == [target(elbow_flex=2993), target(elbow_flex=2993)]
        assert arm.bus.data["Goal_Position"] == arm.bus.data["Present_Position"]
    finally:
        close(session)


def test_recovery_cannot_run_during_teleop_even_with_trusted_provider(tmp_path):
    session, arm, _, _ = recovery_session(tmp_path)
    session._teleop = {"session_id": "active-test"}
    try:
        with pytest.raises(ArmPlanPreflightError, match="teleop"):
            session.handle({"command": "recover_camera"})
        assert arm.bus.goal_history == []
        assert arm.bus.writes == []
    finally:
        session._teleop = None
        close(session)


def test_json_lines_camera_recovery_pumps_cancel_and_confirms_hold(tmp_path):
    session, arm, _, _ = recovery_session(tmp_path)
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b'{"command":"recover_camera"}\n')
    def cancel_during_motion():
        if len(arm.bus.goal_history) == 2:
            arm.bus.on_present_read = None
            os.write(write_fd, b'{"command":"cancel_hold"}\n')
    arm.bus.on_present_read = cancel_during_motion
    responses = []
    def emit(response):
        responses.append(response)
        if response.get("cancelled"):
            assert arm.bus.data["Goal_Position"] == arm.bus.data["Present_Position"]
        if not response.get("exit"):
            os.write(write_fd, b'{"command":"exit"}\n')
    try:
        assert session.serve(fd=read_fd, emit=emit) == 0
        assert responses[0]["cancelled"]
        assert responses[0]["state"]["event"] == "trajectory_cancel_hold"
        assert len(arm.bus.goal_history) == 3
    finally:
        os.close(read_fd)
        os.close(write_fd)
        close(session)


@pytest.mark.parametrize("bad", ["large_overrun", "outside_firmware", "firmware_changed", "arm_calibration"])
def test_owner_rejects_bad_start_or_changed_firmware_and_arm_calibration(tmp_path, bad):
    session, arm, _, _ = recovery_session(tmp_path)
    if bad == "large_overrun":
        arm.bus.data["Present_Position"]["elbow_flex"] = 3010
    elif bad == "outside_firmware":
        arm.bus.data["Present_Position"]["elbow_flex"] = 3056
    elif bad == "firmware_changed":
        arm.bus.data["Max_Position_Limit"]["elbow_flex"] = 3056
    else:
        session.camera_recovery_policy = CameraRecoveryPolicy(
            model_sha256="a" * 64, calibration_sha256="b" * 64,
            camera_serial="qualified-test-camera", arm_calibration_sha256="c" * 64)
    try:
        with pytest.raises(ArmPlanPreflightError):
            session.handle({"command": "recover_camera"})
        assert arm.bus.goal_history == []
        assert arm.bus.writes == []
    finally:
        close(session)
