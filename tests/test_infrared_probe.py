import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np

from tools.capture_infrared_probe import _arm_pair, _extrinsics, _stream_info


def test_transform_records_translation_units_and_direction():
    transform = _extrinsics(SimpleNamespace(get_extrinsics_to=lambda _: SimpleNamespace(
        rotation=[1,0,0,0,1,0,0,0,1], translation=[.1, .2, .3])),
        object(), "ir", "color")
    assert transform["translation_units"] == "meters"
    assert transform["rotation_row_major"] == [1,0,0,0,1,0,0,0,1]
    assert transform["translation_m"] == [.1, .2, .3]
    assert transform["from"] == "ir" and transform["to"] == "color"


def test_extrinsics_converts_sdk_column_major_rotation_and_direction():
    transform = _extrinsics(SimpleNamespace(get_extrinsics_to=lambda _: SimpleNamespace(
        rotation=[0,1,0,-1,0,0,0,0,1], translation=[.01,.02,.03])),
        object(), "depth", "infrared index 1")
    assert transform["rotation_row_major"] == [0,-1,0,1,0,0,0,0,1]
    assert transform["rotation_sdk_column_major"] == [0,1,0,-1,0,0,0,0,1]
    assert transform["from"] == "depth" and transform["to"] == "infrared index 1"
    # A source-frame X basis point maps through target_from_source as requested.
    x = (1, 0, 0)
    r = transform["rotation_row_major"]
    t = transform["translation_m"]
    mapped = tuple(sum(r[row * 3 + col] * x[col] for col in range(3)) + t[row]
                   for row in range(3))
    assert mapped == (0.01, 1.02, 0.03)


def test_profile_keeps_infrared_stream_index():
    intr = SimpleNamespace(width=1280, height=720, fx=600., fy=600., ppx=640., ppy=360.,
                           model="none", coeffs=[0] * 5)
    video = SimpleNamespace(get_intrinsics=lambda: intr, width=lambda: 1280,
                            height=lambda: 720)
    stream = SimpleNamespace(as_video_stream_profile=lambda: video, stream_type=lambda: "infrared",
                             stream_index=lambda: 1, format=lambda: "y8", fps=lambda: 6)
    profile = SimpleNamespace(get_streams=lambda: [stream])
    rs = SimpleNamespace(stream=SimpleNamespace(infrared="infrared"))
    assert _stream_info(profile, rs)["infrared_1"]["index"] == 1


def test_arm_pair_requires_six_valid_raw_counts_and_160ms():
    raw = dict.fromkeys(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                         "wrist_roll", "gripper"), 2048)
    assert _arm_pair({"monotonic": 2., "raw": raw}, 2.159)["qualified_sync"]
    raw.pop("gripper")
    assert not _arm_pair({"monotonic": 2., "raw": raw}, 2.01)["qualified_sync"]


def test_arm_pair_accepts_signed_future_sample_and_rejects_bad_clocks():
    raw = dict.fromkeys(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                         "wrist_roll", "gripper"), 2048)
    future = _arm_pair({"monotonic": 10.01, "raw": raw}, 10.0)
    assert math.isclose(future["delta_s"], -.01)
    assert future["qualified_sync"] is True
    assert "future" in future["sync_policy"]
    for invalid in (float("nan"), float("inf"), float("-inf"), True):
        assert _arm_pair({"monotonic": invalid, "raw": raw}, 10.0) is None
    assert not _arm_pair({"monotonic": 10.160001, "raw": raw}, 10.0)["qualified_sync"]


def test_direct_help_does_not_import_camera_sdk():
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run([sys.executable, "tools/capture_infrared_probe.py", "--help"],
                            cwd=root, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "--phase-file" in result.stdout and "--ir-fps" in result.stdout


def test_main_pins_serial_selects_ir_one_and_cleans_up(monkeypatch, tmp_path):
    import cv2
    import tools.capture_infrared_probe as probe

    calls = {"stopped": 0, "released": 0, "serial": None, "ir_index": None}
    wait_timeouts = []
    class Profile:
        def width(self): return 2
        def height(self): return 2
        def get_intrinsics(self):
            return SimpleNamespace(width=2, height=2, fx=1, fy=1, ppx=0, ppy=0,
                                   model="none", coeffs=[0] * 5)
        def get_extrinsics_to(self, _):
            return SimpleNamespace(rotation=[1,0,0,0,1,0,0,0,1], translation=[0,0,0])
    class Stream:
        def __init__(self, kind, index=0): self.kind, self.index = kind, index
        def as_video_stream_profile(self): return Profile()
        def stream_type(self): return self.kind
        def stream_index(self): return self.index
        def format(self): return "fmt"
        def fps(self): return 6
    class Device:
        def get_info(self, _): return "243322071783"
        def first_depth_sensor(self): return SimpleNamespace(get_depth_scale=lambda: .001)
    class Config:
        def enable_device(self, serial): calls["serial"] = serial
        def enable_stream(self, *args): pass
    class Frame:
        def __init__(self, data): self.data = data
        def get_data(self): return self.data
        def get_timestamp(self): return 1.
        def __bool__(self): return True
    class Frames:
        def get_infrared_frame(self, index):
            calls["ir_index"] = index
            return Frame(np.zeros((2, 2), dtype=np.uint8))
        def get_depth_frame(self): return Frame(np.ones((2, 2), dtype=np.uint16))
        def get_color_frame(self): return Frame(np.zeros((2, 2, 3), dtype=np.uint8))
    class Pipeline:
        def start(self, _): return SimpleNamespace(get_device=lambda: Device(),
            get_stream=lambda kind, *index: Stream(kind, index[0] if index else 0),
            get_streams=lambda: [Stream("infrared", 1), Stream("depth"), Stream("color")])
        def wait_for_frames(self, timeout):
            wait_timeouts.append(timeout)
            return Frames()
        def stop(self): calls["stopped"] += 1
    class Writer:
        def isOpened(self): return True
        def write(self, _): pass
        def release(self): calls["released"] += 1
    class FakeTime:
        now = 0.
        def monotonic(self):
            self.now += .001
            return self.now
    rs = SimpleNamespace(pipeline=Pipeline, config=Config,
        stream=SimpleNamespace(depth="depth", infrared="infrared", color="color"),
        format=SimpleNamespace(z16="z16", y8="y8", bgr8="bgr8"),
        camera_info=SimpleNamespace(serial_number="serial", name="name"),
        align=lambda _: SimpleNamespace(process=lambda frames: frames))
    fakecv = SimpleNamespace(**{name: getattr(cv2, name) for name in
        ("imwrite", "resize", "INTER_LINEAR", "medianBlur", "VideoWriter_fourcc")},
        VideoWriter=lambda *a: Writer(), aruco=None)
    monkeypatch.setitem(sys.modules, "pyrealsense2", rs)
    monkeypatch.setitem(sys.modules, "cv2", fakecv)
    monkeypatch.setattr(probe.time, "monotonic", FakeTime().monotonic)
    monkeypatch.setattr(probe, "_detect", lambda *a: [])
    output = tmp_path / "evidence"
    probe.main(["--output", str(output), "--seconds", ".01"])
    assert calls == {"stopped": 1, "released": 1, "serial": "243322071783", "ir_index": 1}
    meta = json.loads((output / "metadata.json").read_text())
    assert meta["depth_mapping"] == "depth aligned to infrared index 1"
    assert "T_color_from_ir" in meta and "T_ir_from_depth" in meta
    assert (output / "000001-depth.npy").exists()
    rows = [json.loads(line) for line in (output / "frames.jsonl").read_text().splitlines()]
    assert rows
    assert all((output / row["infrared"]).exists() and (output / row["depth"]).exists()
               for row in rows)
    assert wait_timeouts and all(0 < timeout <= 5000 for timeout in wait_timeouts)


def test_duration_is_capped_at_300_seconds_before_camera_import(tmp_path):
    import tools.capture_infrared_probe as probe
    try:
        probe.main(["--output", str(tmp_path / "bounded"), "--seconds", "300.1"])
    except SystemExit as exc:
        assert exc.code == 2
    else:
        raise AssertionError("duration above 300 seconds was accepted")
