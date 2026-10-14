#!/usr/bin/env python3
"""Bounded D435i IR fiducial probe; SDK imports and camera access start in main()."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
              "wrist_roll", "gripper")


def _intrinsics(video):
    i = video.get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
            "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model),
            "coeffs": list(i.coeffs)}


def _extrinsics(source, target, source_name, target_name):
    e = source.get_extrinsics_to(target)
    # librealsense stores rotation in column-major order; publish row-major matrices.
    rotation = [e.rotation[row + 3 * col] for row in range(3) for col in range(3)]
    return {"from": source_name, "to": target_name,
            "rotation_row_major": rotation, "rotation_sdk_column_major": list(e.rotation),
            "translation_m": list(e.translation),
            "translation_units": "meters"}


def _atomic_png(cv2, path, image):
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    if not cv2.imwrite(str(temporary), image):
        raise RuntimeError(f"Could not write {path}")
    temporary.replace(path)


def _atomic_npy(np, path, array):
    temporary = path.with_name(path.stem + ".tmp" + path.suffix)
    with temporary.open("wb") as stream:
        np.save(stream, array)
    temporary.replace(path)


def _atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def _arm_pair(state, monotonic):
    if not isinstance(state, dict):
        return None
    arm_clock = state.get("monotonic")
    if (isinstance(arm_clock, bool) or not isinstance(arm_clock, (int, float))
            or not math.isfinite(arm_clock)):
        return None
    raw = state.get("raw")
    valid = (isinstance(raw, dict) and set(raw) == set(ARM_JOINTS) and
             all(type(v) is int and 0 <= v <= 4095 for v in raw.values()))
    delta = monotonic - arm_clock
    return {"event": state.get("event"), "label": state.get("label"),
            "raw": raw, "pose": state.get("pose"),
            "timestamp_utc": state.get("timestamp_utc"),
            "monotonic": arm_clock, "delta_s": delta,
            "qualified_sync": valid and abs(delta) <= .160,
            "sync_tolerance_s": 0.160,
            "sync_policy": "nearest actual sample; signed delta allows a sample up to 160 ms in the future"}


def _detect(gray, cv2, np):
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, "DICT_APRILTAG_25h9"):
        return []
    image = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    image = cv2.medianBlur(image, 3)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_25h9)
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(dictionary).detectMarkers(image)
    else:
        corners, ids, _ = aruco.detectMarkers(image, dictionary)
    found = []
    for corners_i, marker_id in zip(corners, [] if ids is None else ids.flatten()):
        points = np.asarray(corners_i).reshape(4, 2) / 2.0
        edges = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
        found.append({"id": int(marker_id), "corners_px": points.tolist(),
                      "mean_edge_px": float(edges.mean()), "min_edge_px": float(edges.min())})
    return found


def _stream_info(profile, rs):
    result = {}
    for stream in profile.get_streams():
        v = stream.as_video_stream_profile()
        name = str(stream.stream_type())
        result[name + (f"_{stream.stream_index()}" if stream.stream_type() == rs.stream.infrared else "")] = {
            "stream": name, "index": stream.stream_index(), "format": str(stream.format()),
            "fps": stream.fps(), "width": v.width(), "height": v.height(),
            "intrinsics": _intrinsics(v)}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--serial", default="243322071783")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--phase-file", type=Path)
    parser.add_argument("--ir-width", type=int, default=1280)
    parser.add_argument("--ir-height", type=int, default=720)
    parser.add_argument("--ir-fps", type=int, default=6)
    parser.add_argument("--depth-width", type=int, default=1280)
    parser.add_argument("--depth-height", type=int, default=720)
    parser.add_argument("--depth-fps", type=int, default=6)
    args = parser.parse_args(argv)
    if not 0 < args.seconds <= 300:
        parser.error("--seconds must be greater than 0 and at most 300")
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    args.output.mkdir(parents=True, exist_ok=True)
    if any(args.output.iterdir()):
        parser.error("Choose an empty output directory to preserve existing evidence")
    pipe, started, writer, count = rs.pipeline(), False, None, 0
    stop_requested = [False]
    old_handlers = {}
    def request_stop(_signum, _frame):
        stop_requested[0] = True
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, request_stop)
        config = rs.config()
        config.enable_device(args.serial)
        config.enable_stream(rs.stream.depth, args.depth_width, args.depth_height, rs.format.z16, args.depth_fps)
        config.enable_stream(rs.stream.infrared, 1, args.ir_width, args.ir_height, rs.format.y8, args.ir_fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        profile = pipe.start(config)
        started = True
        device = profile.get_device()
        irp = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        dp = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        cp = profile.get_stream(rs.stream.color).as_video_stream_profile()
        metadata = {"device": {"serial_number": device.get_info(rs.camera_info.serial_number),
                               "name": device.get_info(rs.camera_info.name)},
                    "profiles": _stream_info(profile, rs),
                    "depth_scale_m": device.first_depth_sensor().get_depth_scale(),
                    "depth_encoding": "uint16 raw samples", "depth_mapping": "depth aligned to infrared index 1",
                    "T_color_from_ir": _extrinsics(irp, cp, "infrared index 1", "color"),
                    "T_ir_from_depth": _extrinsics(dp, irp, "depth", "infrared index 1"),
                    "transform_convention": "column vectors; target_from_source; rotation row-major; translation meters",
                    "host_clock": "time.monotonic seconds", "started_utc": datetime.now(timezone.utc).isoformat(),
                    "fiducial_detector": "DICT_APRILTAG_25h9, native grayscale, median kernel 3, 2x upscale; no physical size assumed"}
        _atomic_json(args.output / "metadata.json", metadata)
        align_ir, align_color = rs.align(rs.stream.infrared), rs.align(rs.stream.color)
        writer = cv2.VideoWriter(str(args.output / "infrared.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                 8, (320, 240), False)
        if not writer.isOpened():
            raise RuntimeError("Could not open infrared.avi")
        begin, next_video = time.monotonic(), time.monotonic()
        deadline = begin + args.seconds
        with (args.output / "frames.jsonl").open("w") as log:
            while not stop_requested[0] and time.monotonic() < deadline:
                remaining_ms = int((deadline - time.monotonic()) * 1000)
                if remaining_ms <= 0:
                    break
                try:
                    frames = pipe.wait_for_frames(min(5000, remaining_ms))
                except RuntimeError:
                    if time.monotonic() >= deadline:
                        break
                    raise
                if stop_requested[0]:
                    break
                aligned = align_ir.process(frames)
                ir = aligned.get_infrared_frame(1)
                depth = aligned.get_depth_frame()
                if not ir or not depth:
                    continue
                acquired, utc = time.monotonic(), datetime.now(timezone.utc).isoformat()
                gray, raw_depth = np.asanyarray(ir.get_data()), np.asanyarray(depth.get_data())
                if gray.shape != raw_depth.shape:
                    raise RuntimeError("IR-aligned depth dimensions do not match infrared index 1")
                count += 1
                stem = f"{count:06d}"
                ir_path, depth_path = args.output / f"{stem}-ir.png", args.output / f"{stem}-depth.npy"
                _atomic_png(cv2, ir_path, gray)
                _atomic_npy(np, depth_path, raw_depth)
                rgb_info = None
                color_aligned = align_color.process(frames)
                color, cdepth = color_aligned.get_color_frame(), color_aligned.get_depth_frame()
                if color and cdepth:
                    rgb_path = args.output / f"{stem}-rgb.png"
                    cdepth_path = args.output / f"{stem}-color-depth.npy"
                    _atomic_png(cv2, rgb_path, np.asanyarray(color.get_data()))
                    _atomic_npy(np, cdepth_path, np.asanyarray(cdepth.get_data()))
                    rgb_info = {"rgb": f"{stem}-rgb.png", "depth": f"{stem}-color-depth.npy",
                                "color_device_ms": color.get_timestamp(), "depth_device_ms": cdepth.get_timestamp()}
                state = None
                if args.phase_file:
                    try:
                        state = json.loads(args.phase_file.read_text())
                    except (OSError, ValueError):
                        pass
                record = {"index": count, "infrared": f"{stem}-ir.png", "depth": f"{stem}-depth.npy",
                          "host_monotonic": acquired, "host_utc": utc,
                          "ir_device_ms": ir.get_timestamp(), "depth_device_ms": depth.get_timestamp(),
                          "arm": _arm_pair(state, acquired), "rgb_snapshot": rgb_info,
                          "fiducials": _detect(gray, cv2, np)}
                # Publish the manifest row only after every referenced snapshot is committed.
                log.write(json.dumps(record, allow_nan=False) + "\n")
                log.flush()
                if acquired >= next_video:
                    writer.write(cv2.resize(gray, (320, 240)))
                    next_video = acquired + .125
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if writer is not None:
                writer.release()
            if started:
                pipe.stop()
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    print(f"Recorded {count} IR frames to {args.output}")


if __name__ == "__main__":
    main()
