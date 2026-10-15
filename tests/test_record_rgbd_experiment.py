import json
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import sys

from tools.record_rgbd_experiment import (MAX_ARM_PAIRING_DELTA_S,
                                         arm_transition, atomic_json,
                                         build_parser, camera_stream_metadata,
                                         configure_streams,
                                         manifest_pose_raw, pair_arm_state,
                                         read_arm_state,
                                         snapshot_metadata)


def resident_state(monotonic=10.0, *, event="sample", label="pick-cube"):
    return {
        "schema": 1,
        "event": event,
        "timestamp_utc": datetime(2026, 10, 14, tzinfo=timezone.utc).isoformat(),
        "monotonic": monotonic,
        "raw": {joint: 2000 for joint in (
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
            "wrist_roll", "gripper")},
        "pose": {"shoulder_pan": 0.0},
        "label": label,
    }


def test_pairs_camera_frame_with_recent_actual_resident_state():
    state = resident_state()
    paired = pair_arm_state(state, 10.12)
    assert paired["monotonic"] == 10.0
    assert paired["timestamp_utc"] == state["timestamp_utc"]
    assert paired["raw"] == state["raw"]
    assert paired["pose"] == state["pose"]
    assert round(paired["delta_s"], 2) == 0.12
    assert paired["qualified_sync"] is True


def test_stale_sample_is_attached_but_not_marked_synchronized():
    paired = pair_arm_state(resident_state(9.8), 10.0)
    assert paired["raw"] is not None
    assert paired["qualified_sync"] is False
    assert paired["delta_s"] > MAX_ARM_PAIRING_DELTA_S


def test_unlabeled_telemetry_sample_does_not_create_repeated_keyframes():
    state = resident_state(label=None)
    token, label, event = arm_transition(state)
    assert token is None
    assert label is None
    assert event is None
    token, label, event = arm_transition(state, previous_label=label,
                                         previous_event=event)
    assert token is None
    assert label is None
    assert event is None


def test_label_and_lifecycle_edges_create_one_keyframe_each():
    first = resident_state()
    first["phase"] = "move_plan"
    token, label, event = arm_transition(first,
                                         previous_label=None,
                                         previous_event=None)
    assert token == ("label", "pick-cube")
    token, label, event = arm_transition(resident_state(10.05),
                                         previous_label=label,
                                         previous_event=event)
    assert token is None
    token, label, event = arm_transition(resident_state(
        10.1, event="plan_complete"), previous_label=label,
        previous_event=event)
    assert token == ("event", "plan_complete")
    token, label, event = arm_transition(resident_state(10.15),
                                         previous_label=label,
                                         previous_event=event)
    assert token is None


def test_legacy_phase_file_transition_and_atomic_latest_json(tmp_path):
    state_path = tmp_path / "state.json"
    atomic_json(state_path, {"phase": "reach", "event_id": 7,
                             "monotonic": 3.0, "pose_raw": {"joint": 1}})
    state = read_arm_state(state_path)
    assert arm_transition(state)[0] == ("legacy", "reach", 7)
    paired = pair_arm_state(state, 3.02)
    assert paired["raw"] == {"joint": 1}
    assert paired["qualified_sync"] is True
    assert not state_path.with_name("state.json.tmp").exists()
    assert json.loads(state_path.read_text())["phase"] == "reach"


def test_manifest_pose_raw_exposes_only_qualified_resident_raw_counts():
    state = resident_state()
    fresh = pair_arm_state(state, 10.02)
    stale = pair_arm_state(state, 10.3)
    assert manifest_pose_raw(state, fresh) == state["raw"]
    assert manifest_pose_raw(state, stale) is None
    assert manifest_pose_raw({"pose_raw": {"joint": 1}}, None) == {"joint": 1}


def test_fake_camera_frame_metadata_keeps_acquisition_and_arm_provenance():
    timing = {"monotonic": 12.0, "utc": "2026-10-14T00:00:00+00:00",
              "device_ms": 500.0, "depth_device_ms": 499.0}
    arm = pair_arm_state(resident_state(11.96), timing["monotonic"])
    frame_metadata = snapshot_metadata(4, "frames/000004.png",
                                       "frames/000004.npy", timing, arm)
    assert frame_metadata["camera_acquisition"]["monotonic"] == 12.0
    assert frame_metadata["camera_acquisition"]["timestamp_utc"] == timing["utc"]
    assert frame_metadata["paths"]["snapshot_color"] == "frames/000004.png"
    assert frame_metadata["paths"]["snapshot_depth"] == "frames/000004.npy"
    assert frame_metadata["arm"]["raw"] == resident_state()["raw"]
    assert frame_metadata["arm"]["qualified_sync"] is True


def test_direct_script_help_starts_without_pythonpath():
    repo_root = Path(__file__).resolve().parents[1]
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, "tools/record_rgbd_experiment.py", "--help"],
        cwd=repo_root, env=environment, capture_output=True, text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "--phase-file" in result.stdout
    assert "--color-width" in result.stdout
    assert "--depth-fps" in result.stdout


class FakeVideoProfile:
    def __init__(self, stream_type, pixel_format, width, height, fps, intrinsics=None):
        self._type, self._format = stream_type, pixel_format
        self._width, self._height, self._fps = width, height, fps
        self._intrinsics = intrinsics

    def as_video_stream_profile(self):
        return self

    def stream_type(self): return self._type
    def format(self): return self._format
    def stream_index(self): return 0
    def width(self): return self._width
    def height(self): return self._height
    def fps(self): return self._fps
    def get_intrinsics(self): return self._intrinsics


class FakeIntrinsics:
    width, height = 1920, 1080
    fx, fy, ppx, ppy = 1200.5, 1201.5, 960.0, 540.0
    model = "none"
    coeffs = (0.0, 0.0, 0.0, 0.0, 0.0)


class FakeRS:
    class stream:
        depth, color = 1, 2

    class format:
        z16, bgr8 = 3, 4


class FakeConfig:
    def __init__(self): self.calls = []
    def enable_stream(self, *args): self.calls.append(args)


class FakeDevice:
    def first_depth_sensor(self): return self
    def get_depth_scale(self): return 0.0010000000474974513


class FakePipelineProfile:
    def __init__(self, profiles): self.profiles = profiles
    def get_streams(self): return self.profiles
    def get_stream(self, stream_type):
        return next(profile for profile in self.profiles
                    if profile.stream_type() == stream_type)


def test_camera_mode_cli_defaults_and_custom_values():
    parser = build_parser()
    defaults = parser.parse_args(["--output", "/tmp/evidence"])
    assert (defaults.color_width, defaults.color_height, defaults.color_fps) == (640, 480, 15)
    assert (defaults.depth_width, defaults.depth_height, defaults.depth_fps) == (640, 480, 15)

    custom = parser.parse_args([
        "--output", "/tmp/evidence", "--color-width", "1920",
        "--color-height", "1080", "--color-fps", "8",
        "--depth-width", "640", "--depth-height", "480", "--depth-fps", "6",
    ])
    assert (custom.color_width, custom.color_height, custom.color_fps) == (1920, 1080, 8)
    assert (custom.depth_width, custom.depth_height, custom.depth_fps) == (640, 480, 6)


def test_camera_mode_cli_rejects_nonpositive_dimensions_or_fps():
    parser = build_parser()
    for option in ("--color-width", "--color-height", "--color-fps",
                   "--depth-width", "--depth-height", "--depth-fps"):
        try:
            parser.parse_args(["--output", "/tmp/evidence", option, "0"])
        except SystemExit as error:
            assert error.code == 2
        else:
            raise AssertionError(f"{option} accepted zero")


def test_configures_exact_requested_color_and_depth_profiles():
    args = build_parser().parse_args([
        "--output", "/tmp/evidence", "--color-width", "1920",
        "--color-height", "1080", "--color-fps", "8", "--depth-fps", "6",
    ])
    config = FakeConfig()
    configure_streams(config, FakeRS, args)
    assert config.calls == [
        (FakeRS.stream.depth, 640, 480, FakeRS.format.z16, 6),
        (FakeRS.stream.color, 1920, 1080, FakeRS.format.bgr8, 8),
    ]


def test_camera_metadata_reports_exact_selected_profiles_and_aligned_depth_size():
    depth = FakeVideoProfile(FakeRS.stream.depth, FakeRS.format.z16, 640, 480, 6)
    color = FakeVideoProfile(FakeRS.stream.color, FakeRS.format.bgr8, 1920, 1080, 8,
                             FakeIntrinsics())
    profile = FakePipelineProfile([depth, color])
    requested = {"depth": (640, 480, 6), "color": (1920, 1080, 8)}
    metadata = camera_stream_metadata(profile, FakeRS, FakeDevice(), requested)
    assert metadata["profiles"] == [
        {"stream": "1", "format": "3", "fps": 6, "width": 640, "height": 480, "index": 0},
        {"stream": "2", "format": "4", "fps": 8, "width": 1920, "height": 1080, "index": 0},
    ]
    assert metadata["intrinsics"] == {
        "width": 1920, "height": 1080, "fx": 1200.5, "fy": 1201.5,
        "ppx": 960.0, "ppy": 540.0, "model": "none",
        "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    assert metadata["depth_scale_m"] == FakeDevice().get_depth_scale()
    assert metadata["aligned_depth_dimensions"] == {"width": 1920, "height": 1080}


def test_camera_metadata_rejects_sdk_profile_fallback():
    depth = FakeVideoProfile(FakeRS.stream.depth, FakeRS.format.z16, 640, 480, 6)
    color = FakeVideoProfile(FakeRS.stream.color, FakeRS.format.bgr8, 640, 480, 15,
                             FakeIntrinsics())
    profile = FakePipelineProfile([depth, color])
    requested = {"depth": (640, 480, 6), "color": (1920, 1080, 8)}
    try:
        camera_stream_metadata(profile, FakeRS, FakeDevice(), requested)
    except RuntimeError as error:
        assert "Requested color profile" in str(error)
    else:
        raise AssertionError("SDK-selected fallback profile was accepted")
