"""Camera-guided re-entry must prove every sample before enabling recovery."""
import pytest

from workshop.station.camera_recovery import plan_bound_reentry


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
START = dict.fromkeys(JOINTS, 2000)
START["elbow_flex"] = 2993
BOUNDS = {joint: (1000, 3000) for joint in JOINTS}
BOUNDS["elbow_flex"] = (1000, 2975)
FIRMWARE = dict.fromkeys(JOINTS, (500, 3500))


class Scene:
    camera_serial = "243322071783"
    model_sha256 = "a" * 64
    calibration_sha256 = "b" * 64
    registration_qualified = True
    stamp_monotonic = 10.
    arm_stamp_monotonic = 10.
    arm_raw = START

    def __init__(self, obstruction=None):
        self.obstruction = obstruction
        self.samples = []

    def minimum_clearance_m(self, raw):
        self.samples.append(dict(raw))
        return .001 if raw["elbow_flex"] == self.obstruction else .020

    def minimum_segment_clearance_m(self, start, end):
        return .020


def plan(scene, **kwargs):
    return plan_bound_reentry(START, BOUNDS, FIRMWARE, scene=scene, now=10.1,
                              expected_model_sha256="a" * 64,
                              expected_calibration_sha256="b" * 64, **kwargs)


def test_small_soft_limit_excursion_gets_camera_checked_inward_path():
    scene = Scene()
    result = plan(scene)
    assert result.target_raw["elbow_flex"] == 2973
    assert all(result.target_raw[j] == START[j] for j in JOINTS if j != "elbow_flex")
    assert scene.samples[0] == START
    assert scene.samples[-1] == result.target_raw
    assert len(scene.samples) >= 11
    assert result.minimum_clearance_m == .020


def test_obstruction_between_safe_endpoints_rejects_whole_recovery():
    with pytest.raises(ValueError, match="clearance"):
        plan(Scene(obstruction=2983))


def test_clear_sample_poses_do_not_prove_the_swept_robot_volume():
    scene = Scene()
    scene.minimum_segment_clearance_m = lambda start, end: None
    with pytest.raises(ValueError, match="swept"):
        plan(scene)


def test_every_continuous_segment_requires_clearance_proof():
    scene = Scene()
    segments = []

    def swept(start, end):
        segments.append((dict(start), dict(end)))
        return .012

    scene.minimum_segment_clearance_m = swept
    result = plan(scene)
    assert segments == list(zip(result.samples_raw, result.samples_raw[1:]))
    assert result.minimum_clearance_m == .012


def test_pose_only_collision_adapter_cannot_admit_recovery():
    scene = Scene()
    scene.minimum_segment_clearance_m = None
    with pytest.raises(ValueError, match="swept"):
        plan(scene)


def test_encoder_observation_must_remain_fresh_through_collision_check(monkeypatch):
    scene = Scene()
    scene.stamp_monotonic = 10.1
    scene.arm_stamp_monotonic = 10.0
    ticks = iter([0.0, .35])
    monkeypatch.setattr("workshop.station.camera_recovery.time.monotonic", lambda: next(ticks, .35))
    with pytest.raises(ValueError, match="expired"):
        plan_bound_reentry(START, BOUNDS, FIRMWARE, scene=scene, now=10.2,
                           expected_model_sha256="a" * 64,
                           expected_calibration_sha256="b" * 64)


@pytest.mark.parametrize("field,value", [("registration_qualified", False),
    ("stamp_monotonic", 9.), ("arm_stamp_monotonic", 9.8),
    ("model_sha256", "c" * 64), ("calibration_sha256", "c" * 64)])
def test_stale_unpaired_or_unqualified_camera_cannot_admit_recovery(field, value):
    scene = Scene()
    setattr(scene, field, value)
    with pytest.raises(ValueError):
        plan(scene)
    assert not scene.samples


def test_large_excursion_or_firmware_violation_never_becomes_recovery():
    for raw in (3010, 3600):
        start = {**START, "elbow_flex": raw}
        with pytest.raises(ValueError):
            plan_bound_reentry(start, BOUNDS, FIRMWARE, scene=Scene(), now=10.1,
                               expected_model_sha256="a" * 64,
                               expected_calibration_sha256="b" * 64)


def test_camera_recovery_does_not_move_gripper_or_invent_boundaries():
    with pytest.raises(ValueError, match="gripper"):
        plan_bound_reentry({**START, "gripper": 3001}, BOUNDS, FIRMWARE,
                           scene=Scene(), now=10.1, expected_model_sha256="a" * 64,
                           expected_calibration_sha256="b" * 64)


@pytest.mark.parametrize("field,value", [("camera_serial", "another-camera"),
    ("arm_raw", {**START, "elbow_flex": 2970})])
def test_scene_must_belong_to_identified_camera_and_exact_encoder_sample(field, value):
    scene = Scene()
    setattr(scene, field, value)
    with pytest.raises(ValueError):
        plan(scene)
    assert not scene.samples


def test_generic_adapter_cannot_assert_alternative_acquisition_clock():
    scene=Scene()
    scene.stamp_monotonic=10.
    scene.arm_stamp_monotonic=10.120
    scene.acquisition_arm_stamp_monotonic=10.
    with pytest.raises(ValueError,match='synchronized'):
        plan_bound_reentry(START,BOUNDS,FIRMWARE,scene=scene,now=10.120,
            expected_model_sha256=scene.model_sha256,
            expected_calibration_sha256=scene.calibration_sha256,
            expected_camera_serial=scene.camera_serial)
