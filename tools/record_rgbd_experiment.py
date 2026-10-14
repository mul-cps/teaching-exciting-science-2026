#!/usr/bin/env python3
"""Record aligned RGB-D experiment evidence.

Run: python tools/record_rgbd_experiment.py --output /path/to/evidence
Dependencies: pyrealsense2, numpy, opencv-python (existing host global installs).
Camera access and SDK initialization occur only inside main(). PNGs contain
full-resolution color; NPY files contain aligned raw uint16 depth samples.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import time

from workshop.station.config import JOINTS


MAX_ARM_PAIRING_DELTA_S = 0.160


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def pair_arm_state(state, camera_monotonic, *, max_delta=MAX_ARM_PAIRING_DELTA_S):
    """Describe the latest actual resident-arm sample relative to one frame."""
    if not isinstance(state, dict):
        return None
    arm_clock = state.get("monotonic")
    if not _finite_number(arm_clock):
        return None
    delta = camera_monotonic - arm_clock
    resident_schema = "raw" in state
    if resident_schema:
        raw = state.get("raw")
        raw_valid = (isinstance(raw, dict) and set(raw) == set(JOINTS)
                     and all(type(value) is int and 0 <= value <= 4095
                             for value in raw.values()))
    else:
        # Legacy phase files used pose_raw. Keep their qualification contract
        # separate from the resident controller's six measured raw counts.
        raw = state.get("pose_raw")
        raw_valid = raw is not None
    return {
        "event": state.get("event"),
        "label": state.get("label"),
        "phase": state.get("phase"),
        "raw": raw,
        "pose": state.get("pose"),
        "timestamp_utc": state.get("timestamp_utc"),
        "monotonic": arm_clock,
        "delta_s": delta,
        "qualified_sync": raw_valid and abs(delta) <= max_delta,
    }


def arm_transition(state, *, previous_label=None, previous_event=None):
    """Return (token, label, event) for a meaningful label/event edge."""
    if not isinstance(state, dict):
        return None, previous_label, previous_event
    # Backwards compatibility for the older phase/event-id producer.
    if "raw" not in state and isinstance(state.get("phase"), str):
        return ("legacy", state["phase"], state.get("event_id")), previous_label, previous_event
    label = state.get("label")
    event = state.get("event")
    token = None
    if isinstance(label, str) and label and label != previous_label:
        token = ("label", label)
    if isinstance(event, str) and event and event != "sample" and event != previous_event:
        token = ("event", event)
        previous_event = event
    # An unlabeled read_state/idle telemetry update does not erase the last
    # meaningful leg label; the same resumed label therefore stays the same
    # keyframe rather than appearing to be a new leg.
    if isinstance(label, str) and label:
        previous_label = label
    return token, previous_label, previous_event


def manifest_pose_raw(state, arm_pair):
    """Expose actual resident raw counts to old manifest consumers only if synced."""
    if isinstance(state, dict) and "raw" in state:
        if arm_pair is not None and arm_pair["qualified_sync"]:
            return arm_pair["raw"]
        return None
    return state.get("pose_raw") if isinstance(state, dict) else None


def read_arm_state(path):
    try:
        state = json.loads(path.read_text())
        return state if isinstance(state, dict) else None
    except (OSError, ValueError):
        return None


def atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def snapshot_metadata(index, color_path, depth_path, timing, arm_pair):
    """Build the atomically published metadata for one acquired camera frame."""
    return {
        "index": index, "color": color_path, "depth": depth_path,
        **timing,
        "camera_acquisition": {
            "monotonic": timing["monotonic"], "timestamp_utc": timing["utc"],
            "color_device_ms": timing["device_ms"],
            "depth_device_ms": timing["depth_device_ms"],
        },
        "paths": {
            "snapshot_color": color_path, "snapshot_depth": depth_path,
            "latest_color": "latest.png", "latest_depth": "latest-depth.npy",
        },
        "arm": arm_pair,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", default="243322071783")
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--phase-file", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import cv2
    import numpy as np
    import pyrealsense2 as rs

    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    if (output / "metadata.json").exists() or (output / "overview.avi").exists():
        parser.error("Choose a new experiment output directory to preserve earlier evidence")
    frames_dir = output / "frames"
    keyframes_dir = output / "keyframes"
    frames_dir.mkdir(exist_ok=True)
    keyframes_dir.mkdir(exist_ok=True)
    pipe = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    started = False
    writer = None
    index = 0
    keyframe_index = 0
    phase_state = None
    phase_token = None
    last_arm_label = None
    last_arm_event = None

    def write_png(path, color):
        if not cv2.imwrite(str(path), color):
            raise RuntimeError(f"Could not write {path}")

    def atomic_latest(color, depth, info):
        temporary = output / "latest.tmp.png"
        write_png(temporary, color)
        temporary.replace(output / "latest.png")
        temporary_depth = output / "latest-depth.tmp.npy"
        np.save(temporary_depth, depth)
        temporary_depth.replace(output / "latest-depth.npy")
        atomic_json(output / "latest.json", info)

    try:
        profile = pipe.start(config)
        started = True
        device = profile.get_device()
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        stream_profiles = []
        for stream in profile.get_streams():
            video = stream.as_video_stream_profile()
            stream_profiles.append({
                "stream": str(stream.stream_type()), "format": str(stream.format()),
                "fps": stream.fps(), "width": video.width(), "height": video.height(),
                "index": stream.stream_index(),
            })
        identity = {}
        for name in ("name", "serial_number", "firmware_version", "product_line"):
            field = getattr(rs.camera_info, name)
            if device.supports(field):
                identity[name] = device.get_info(field)
        metadata = {
            "device": identity, "profiles": stream_profiles,
            "depth_scale_m": device.first_depth_sensor().get_depth_scale(),
            "aligned_to": "color", "depth_encoding": "uint16 raw samples",
            "intrinsics": {"width": intrinsics.width, "height": intrinsics.height,
                           "fx": intrinsics.fx, "fy": intrinsics.fy,
                           "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
                           "model": str(intrinsics.model), "coeffs": list(intrinsics.coeffs)},
            "overview": {"file": "overview.avi", "width": 320, "height": 240,
                         "fps": 10, "codec": "MJPG"},
            "snapshot_hz": 5, "started_utc": datetime.now(timezone.utc).isoformat(),
        }
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        align = rs.align(rs.stream.color)
        writer = cv2.VideoWriter(str(output / "overview.avi"),
                                 cv2.VideoWriter_fourcc(*"MJPG"), 10, (320, 240))
        if not writer.isOpened():
            raise RuntimeError("Could not open overview.avi video writer")
        begin = time.monotonic()
        next_snapshot = begin
        next_video = begin
        print(f"Recording RGB-D to {output} for {args.seconds:g}s", flush=True)
        with (output / "timestamps.jsonl").open("a") as timestamps, \
                (keyframes_dir / "manifest.jsonl").open("a") as manifest:
            while time.monotonic() - begin < args.seconds:
                aligned = align.process(pipe.wait_for_frames(5000))
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                frame_clock = time.monotonic()
                utc = datetime.now(timezone.utc).isoformat()
                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                timing = {"monotonic": frame_clock, "utc": utc,
                          "device_ms": color_frame.get_timestamp(),
                          "depth_device_ms": depth_frame.get_timestamp()}
                phase_changed = False
                if args.phase_file:
                    candidate = read_arm_state(args.phase_file)
                    token, candidate_label, candidate_event = arm_transition(
                        candidate, previous_label=last_arm_label,
                        previous_event=last_arm_event)
                    if not (isinstance(candidate, dict) and "raw" not in candidate
                            and isinstance(candidate.get("phase"), str)):
                        last_arm_label, last_arm_event = candidate_label, candidate_event
                    # A resident controller publishes an "event=sample"
                    # record at 20 Hz. Do not turn each telemetry update into
                    # a keyframe; labels and non-sample lifecycle events are
                    # the meaningful edges.
                    phase_changed = token is not None and token != phase_token
                    phase_state = candidate
                    if token is not None:
                        phase_token = token
                arm_pair = pair_arm_state(phase_state, frame_clock)
                if frame_clock >= next_video:
                    writer.write(cv2.resize(color, (320, 240)))
                    next_video += 0.1
                    if next_video <= frame_clock:
                        next_video = frame_clock + 0.1
                if frame_clock >= next_snapshot:
                    index += 1
                    stem = f"{index:06d}"
                    write_png(frames_dir / f"{stem}.png", color)
                    np.save(frames_dir / f"{stem}.npy", depth)
                    color_path = f"frames/{stem}.png"
                    depth_path = f"frames/{stem}.npy"
                    latest = snapshot_metadata(index, color_path, depth_path,
                                               timing, arm_pair)
                    atomic_latest(color, depth, latest)
                    timestamps.write(json.dumps({**latest, "snapshot": True}) + "\n")
                    timestamps.flush()
                    next_snapshot += 0.2
                    if next_snapshot <= frame_clock:
                        next_snapshot = frame_clock + 0.2
                if phase_changed:
                    keyframe_index += 1
                    phase = phase_state.get("phase") or phase_state.get("label") or phase_state.get("event") or "event"
                    safe_phase = re.sub(r"[^A-Za-z0-9_-]+", "_", str(phase))[:80] or "phase"
                    stem = f"{keyframe_index:06d}_{safe_phase}"
                    write_png(keyframes_dir / f"{stem}.png", color)
                    np.save(keyframes_dir / f"{stem}.npy", depth)
                    event_clock = phase_state.get("monotonic")
                    delta = arm_pair["delta_s"] if arm_pair is not None else None
                    manifest.write(json.dumps({
                        "index": keyframe_index, "color": f"{stem}.png",
                        "depth": f"{stem}.npy", **timing, "phase": phase,
                        "pose_raw": manifest_pose_raw(phase_state, arm_pair),
                        "event_id": phase_state.get("event_id"),
                        "phase_event_monotonic": event_clock,
                        "phase_event_delta_s": delta,
                        "arm": arm_pair,
                        "pose_synchronized": (arm_pair is not None
                                              and arm_pair["qualified_sync"]),
                    }) + "\n")
                    manifest.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        if started:
            pipe.stop()
        print(f"Recording ended: {index} snapshots, {keyframe_index} keyframes", flush=True)


if __name__ == "__main__":
    main()
