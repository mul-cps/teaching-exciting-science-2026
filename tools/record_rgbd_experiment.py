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
from pathlib import Path
import re
import time


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
    phase_state = {}
    phase_token = None

    def write_png(path, color):
        if not cv2.imwrite(str(path), color):
            raise RuntimeError(f"Could not write {path}")

    def atomic_latest(color, depth):
        temporary = output / "latest.tmp.png"
        write_png(temporary, color)
        temporary.replace(output / "latest.png")
        temporary_depth = output / "latest-depth.tmp.npy"
        np.save(temporary_depth, depth)
        temporary_depth.replace(output / "latest-depth.npy")

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
                    try:
                        candidate = json.loads(args.phase_file.read_text())
                        if isinstance(candidate, dict) and isinstance(candidate.get("phase"), str):
                            token = (candidate["phase"], candidate.get("event_id"))
                            phase_changed = token != phase_token
                            phase_state = candidate
                            phase_token = token
                    except (OSError, ValueError):
                        pass
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
                    atomic_latest(color, depth)
                    timestamps.write(json.dumps({"index": index, **timing}) + "\n")
                    timestamps.flush()
                    next_snapshot += 0.2
                    if next_snapshot <= frame_clock:
                        next_snapshot = frame_clock + 0.2
                if phase_changed:
                    keyframe_index += 1
                    phase = phase_state["phase"]
                    safe_phase = re.sub(r"[^A-Za-z0-9_-]+", "_", phase)[:80] or "phase"
                    stem = f"{keyframe_index:06d}_{safe_phase}"
                    write_png(keyframes_dir / f"{stem}.png", color)
                    np.save(keyframes_dir / f"{stem}.npy", depth)
                    event_clock = phase_state.get("monotonic")
                    delta = (frame_clock - event_clock
                             if isinstance(event_clock, (int, float))
                             and not isinstance(event_clock, bool) else None)
                    manifest.write(json.dumps({
                        "index": keyframe_index, "color": f"{stem}.png",
                        "depth": f"{stem}.npy", **timing, "phase": phase,
                        "pose_raw": phase_state.get("pose_raw"),
                        "event_id": phase_state.get("event_id"),
                        "phase_event_monotonic": event_clock,
                        "phase_event_delta_s": delta,
                        "pose_synchronized": (phase_state.get("pose_raw") is not None
                                              and delta is not None and abs(delta) <= 0.160),
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
