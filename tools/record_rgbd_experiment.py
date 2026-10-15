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

RAW_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                   "wrist_flex", "wrist_roll", "gripper")
MAX_ARM_PAIRING_DELTA_S = 0.160
DEFAULT_COLOR_PROFILE = (640, 480, 15)
DEFAULT_DEPTH_PROFILE = (640, 480, 15)


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
        raw_valid = (isinstance(raw, dict) and set(raw) == set(RAW_JOINT_NAMES)
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


def positive_int(value):
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return parsed


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", default="243322071783")
    parser.add_argument("--seconds", type=float, default=600)
    parser.add_argument("--phase-file", type=Path)
    for name, default in (("color", DEFAULT_COLOR_PROFILE),
                          ("depth", DEFAULT_DEPTH_PROFILE)):
        parser.add_argument(f"--{name}-width", type=positive_int, default=default[0])
        parser.add_argument(f"--{name}-height", type=positive_int, default=default[1])
        parser.add_argument(f"--{name}-fps", type=positive_int, default=default[2])
    return parser


def configure_streams(config, rs, args):
    config.enable_stream(rs.stream.depth, args.depth_width, args.depth_height,
                         rs.format.z16, args.depth_fps)
    config.enable_stream(rs.stream.color, args.color_width, args.color_height,
                         rs.format.bgr8, args.color_fps)


def camera_stream_metadata(profile, rs, device, requested):
    """Report selected SDK modes and fail if the SDK did not honor requests."""
    stream_profiles = []
    selected = {}
    for stream in profile.get_streams():
        video = stream.as_video_stream_profile()
        kind = stream.stream_type()
        details = {
            "stream": str(kind), "format": str(stream.format()),
            "fps": stream.fps(), "width": video.width(),
            "height": video.height(), "index": stream.stream_index(),
        }
        stream_profiles.append(details)
        if kind == rs.stream.depth:
            selected["depth"] = stream
        elif kind == rs.stream.color:
            selected["color"] = stream

    for name, fmt in (("depth", rs.format.z16), ("color", rs.format.bgr8)):
        if name not in selected:
            raise RuntimeError(f"SDK did not select a {name} video profile")
        stream = selected[name]
        video = stream.as_video_stream_profile()
        wanted = requested[name]
        actual = (video.width(), video.height(), stream.fps())
        if actual != wanted or stream.format() != fmt:
            raise RuntimeError(
                f"Requested {name} profile {wanted} format {fmt}, but SDK selected "
                f"{actual} format {stream.format()}"
            )

    color = selected["color"].as_video_stream_profile()
    intrinsics = color.get_intrinsics()
    return {
        "profiles": stream_profiles,
        "requested_profiles": {
            name: {"width": values[0], "height": values[1], "fps": values[2]}
            for name, values in requested.items()
        },
        "aligned_depth_dimensions": {"width": color.width(), "height": color.height()},
        "depth_scale_m": device.first_depth_sensor().get_depth_scale(),
        "intrinsics": {
            "width": intrinsics.width, "height": intrinsics.height,
            "fx": intrinsics.fx, "fy": intrinsics.fy,
            "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
            "model": str(intrinsics.model), "coeffs": list(intrinsics.coeffs),
        },
    }


def main():
    parser = build_parser()
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
    configure_streams(config, rs, args)
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
        camera_metadata = camera_stream_metadata(profile, rs, device, {
            "color": (args.color_width, args.color_height, args.color_fps),
            "depth": (args.depth_width, args.depth_height, args.depth_fps),
        })
        identity = {}
        for name in ("name", "serial_number", "firmware_version", "product_line"):
            field = getattr(rs.camera_info, name)
            if device.supports(field):
                identity[name] = device.get_info(field)
        metadata = {
            "device": identity, **camera_metadata,
            "aligned_to": "color", "depth_encoding": "uint16 raw samples",
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
