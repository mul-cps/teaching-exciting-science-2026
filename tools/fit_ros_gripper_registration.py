#!/usr/bin/env python3
"""Offline fixed-body scan of SO-101 gripper mount roll candidates."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.fit_ros_robot_registration import (_metrics, _project, depth_matches,
    depth_roi_points, exact_surface_metrics, load_visual_mesh, sample_visual_surfaces)
from workshop.digital_twin.tag_tracking import TagTracker

ROLL_DELTAS_DEG = (0.0, 90.0, 180.0, -90.0)
POSE_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def validate_snapshot_diversity(frames, holdout_index):
    """Require three training snapshots spanning two axes and a distinct holdout."""
    if len(frames) < 4 or not 0 <= holdout_index < len(frames):
        raise ValueError("need at least three training snapshots and one held-out pose")
    train = [f for i, f in enumerate(frames) if i != holdout_index]
    matrix = np.asarray([[float(f["q"][joint]) for joint in POSE_JOINTS] for f in train])
    spans = np.ptp(matrix, axis=0)
    active = spans >= np.deg2rad(3)
    if active.sum() < 2 or np.linalg.matrix_rank(matrix[:, active] - matrix[:, active].mean(axis=0), tol=np.deg2rad(1)) < 2:
        raise ValueError("training snapshots need independent movement on at least two joints")
    heldout = np.asarray([float(frames[holdout_index]["q"][joint]) for joint in POSE_JOINTS])
    if np.min(np.linalg.norm(matrix - heldout, axis=1)) < np.deg2rad(3):
        raise ValueError("held-out snapshot must use a distinct joint pose")
    return {"training_snapshot_count": len(train), "excited_joints": [j for j, yes in zip(POSE_JOINTS, active) if yes],
            "heldout_pose_distinct": True}


def rotate_tool_description(urdf_path, total_roll_deg):
    """Return URDF text with only wrist_roll->gripper_link mount rotated."""
    root = ET.parse(urdf_path).getroot()
    joint = root.find("joint[@name='wrist_roll']")
    if joint is None or joint.find("child").get("link") != "gripper_link":
        raise ValueError("expected wrist_roll -> gripper_link joint")
    origin = joint.find("origin")
    if origin is None:
        raise ValueError("wrist_roll joint has no origin")
    old = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" "))
    updated = old * Rotation.from_euler("z", float(total_roll_deg), degrees=True)
    origin.set("rpy", " ".join(f"{x:.17g}" for x in updated.as_euler("xyz")))
    return ET.tostring(root, encoding="unicode")


def _load_capture(bundle, tag_tracker=None):
    bundle = Path(bundle).resolve()
    capture = json.loads((bundle / "capture.json").read_text())
    depth = np.load(bundle / "depth.npy", allow_pickle=False)
    image = cv2.imread(str(bundle / "color.png"), cv2.IMREAD_COLOR)
    if depth.ndim != 2 or image is None or image.shape[:2] != depth.shape:
        raise ValueError(f"invalid saved RGB-D bundle: {bundle}")
    if capture.get("depth_encoding") != "16UC1":
        raise ValueError(f"unsupported depth encoding in {bundle}")
    observation = []
    if tag_tracker is not None:
        cam = capture["camera"]
        detections = tag_tracker.detect(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        observation = tag_tracker.update(detections,
            {"fx": cam["k"][0], "fy": cam["k"][4], "ppx": cam["k"][2], "ppy": cam["k"][5]},
            depth, float(capture["depth_scale_m"]), np.asarray(cam.get("d", [0]*5)))
        for tag in observation:
            size = tag.get("size_m")
            tag["provisional_role"] = ("moving_gripper_tag_candidate" if size is not None and abs(size-.0194) < .002
                                        else "stationary_ev3_tag_candidate" if size is not None and abs(size-.036) < .003
                                        else "unassigned_repeated_id")
    return {"bundle": bundle, "capture": capture, "depth": depth, "image": image,
            "q": capture["joint_positions"], "intrinsics": np.asarray(capture["camera"]["k"], float).reshape(3, 3),
            "distortion": np.asarray(capture["camera"].get("d", [0] * 5), float).reshape(-1, 1),
            "scale": float(capture["depth_scale_m"]),
            "world_from_camera": np.asarray(capture["world_from_camera"], float),
            "world_from_base": np.asarray(capture["world_from_base"], float),
            "tag_observations": observation}


def _score(frame, urdf, samples, observed_world=None):
    c = frame["capture"]
    points, normals, labels, _ = sample_visual_surfaces(
        urdf, c["mesh_package_paths"], frame["q"], samples=samples, seed=17)
    mesh, face_links = load_visual_mesh(urdf, c["mesh_package_paths"], frame["q"])
    world = observed_world
    if world is None:
        world, _ = depth_roi_points(points, normals, frame["world_from_base"],
            frame["world_from_base"], frame["world_from_camera"], frame["depth"],
            frame["scale"], frame["intrinsics"], frame["distortion"], .02, .06)
    inv = np.linalg.inv(frame["world_from_base"])
    observed_base = world @ inv[:3, :3].T + inv[:3, 3]
    observed, _, _ = exact_surface_metrics(mesh, face_links, observed_base, np.zeros(6))
    visible = depth_matches(points, normals, labels, frame["world_from_base"],
        frame["world_from_camera"], frame["depth"], frame["scale"],
        frame["intrinsics"], frame["distortion"], max_depth_gap_m=.06)
    visible_report = _metrics(visible, frame["world_from_base"], labels, .06)
    tight = depth_matches(points, normals, labels, frame["world_from_base"],
        frame["world_from_camera"], frame["depth"], frame["scale"],
        frame["intrinsics"], frame["distortion"], max_depth_gap_m=.01)
    tight_report = _metrics(tight, frame["world_from_base"], labels, .01)
    for name, item in visible_report["per_link"].items():
        item["coverage_within_10mm"] = tight_report["per_link"][name]["overlap_fraction"]
    return {"observed_to_cad": observed, "visible_cad_to_depth": visible_report,
            "fixed_roi_points": len(world)}


def _write_overlay(frame, urdf, total_roll, path, samples=8000):
    c = frame["capture"]
    points, _, _, _ = sample_visual_surfaces(urdf, c["mesh_package_paths"], frame["q"], samples=samples, seed=19)
    world = points @ frame["world_from_base"][:3, :3].T + frame["world_from_base"][:3, 3]
    camera_from_world = np.linalg.inv(frame["world_from_camera"])
    camera = world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
    uv = np.rint(_project(camera, frame["intrinsics"], frame["distortion"])).astype(int)
    h, w = frame["depth"].shape
    valid = (camera[:, 2] > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    ids = np.flatnonzero(valid)
    measured = frame["depth"][uv[ids, 1], uv[ids, 0]].astype(float) * frame["scale"]
    ids = ids[(measured > 0) & (np.abs(measured - camera[ids, 2]) < .06)]
    output = frame["image"].copy()
    for u, v in uv[ids][::2]:
        cv2.circle(output, (int(u), int(v)), 1, (0, 40, 255), -1)
    cv2.putText(output, f"CAD gripper mount total {total_roll:+.0f} deg (red: CAD near depth)",
                (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), output)


def scan_bundles(bundles, *, output, holdout_index=-1, mount_roll_deg=90.0,
                 deltas=ROLL_DELTAS_DEG, samples=10000):
    """Score fixed-body roll hypotheses; last/selected bundle is held out."""
    tracker = TagTracker(detector_scale=2, max_distance_px=1000,
                         size_candidates=(.0075, .0194, .036))
    frames = [_load_capture(path, tracker) for path in bundles]
    if len(frames) < 2:
        raise ValueError("provide at least one training bundle and one held-out bundle")
    if holdout_index < 0:
        holdout_index += len(frames)
    if not 0 <= holdout_index < len(frames):
        raise ValueError("holdout index is outside bundle list")
    diversity = validate_snapshot_diversity(frames, holdout_index)
    hashes = {f["capture"]["urdf_sha256"] for f in frames}
    if len(hashes) != 1:
        raise ValueError("bundles must share the same captured ROS URDF")
    train_ids = [i for i in range(len(frames)) if i != holdout_index]
    report = {"schema": 1, "status": "fixed_body_gripper_roll_scan_unqualified",
        "qualified": False, "live_state_modified": False, "ros_started": False,
        "base_correction": np.eye(4).tolist(), "configured_mount_roll_deg": float(mount_roll_deg),
        "gauge": "AprilTag pose alone identifies a constant gripper_from_tag transform; absolute tool roll requires CAD/depth face geometry. ROS gripper q is an opening-percent proxy and is not calibrated jaw angle.",
        "fixed_body": True, "holdout_index": holdout_index, "pose_diversity": diversity,
        "urdf_sha256": next(iter(hashes)), "tag_observations": [
            {"bundle": str(f["bundle"]), "detections": f["tag_observations"]} for f in frames],
        "hypotheses": []}
    with tempfile.TemporaryDirectory(prefix="so101-gripper-roll-") as temp_name:
        temp = Path(temp_name)
        candidates = []
        for delta in map(float, deltas):
            total = float(mount_roll_deg + delta)
            urdfs = []
            for i, frame in enumerate(frames):
                generated = temp / f"{delta:+g}-{i}.urdf"
                generated.write_text(rotate_tool_description(frame["bundle"] / "robot.urdf", total))
                urdfs.append(generated)
            candidates.append((delta, total, urdfs))
        fixed_observed = []
        for frame_index, frame in enumerate(frames):
            all_points, all_normals = [], []
            for _, _, urdfs in candidates:
                points, normals, _, _ = sample_visual_surfaces(
                    urdfs[frame_index], frame["capture"]["mesh_package_paths"], frame["q"],
                    samples=max(1000, samples // 2), seed=17)
                all_points.append(points); all_normals.append(normals)
            world, _ = depth_roi_points(np.concatenate(all_points), np.concatenate(all_normals),
                frame["world_from_base"], frame["world_from_base"], frame["world_from_camera"],
                frame["depth"], frame["scale"], frame["intrinsics"], frame["distortion"], .02, .06)
            fixed_observed.append(world)
        for delta, total, urdfs in candidates:
            scores = [_score(frame, urdfs[i], samples, fixed_observed[i]) for i, frame in enumerate(frames)]
            def summarize(ids):
                overlap = []
                for i in ids:
                    vis = scores[i]["visible_cad_to_depth"]
                    projected = sum(v["projected_frontface_samples"] for v in vis["per_link"].values())
                    overlap.append(vis["matched_depth_pixels"] / max(1, projected))
                return {"observed_to_cad_median_m": float(np.median([scores[i]["observed_to_cad"]["median_surface_residual_m"] for i in ids])),
                    "visible_cad_to_depth_overlap": float(np.mean(overlap))}
            report["hypotheses"].append({"delta_from_config_deg": delta, "total_mount_roll_deg": total,
                "training": summarize(train_ids), "heldout": scores[holdout_index],
                "per_snapshot": [{"bundle": str(f["bundle"]), "role": "heldout" if i == holdout_index else "training",
                    "observed_to_cad": scores[i]["observed_to_cad"],
                    "visible_cad_to_depth": scores[i]["visible_cad_to_depth"], "fixed_roi_points": scores[i]["fixed_roi_points"]}
                    for i, f in enumerate(frames)]})
        best = min(report["hypotheses"], key=lambda x: (x["heldout"]["observed_to_cad"]["median_surface_residual_m"], -x["heldout"]["visible_cad_to_depth"]["matched_depth_pixels"]))
        report["selected_by_heldout_diagnostic_only"] = {"delta_from_config_deg": best["delta_from_config_deg"], "total_mount_roll_deg": best["total_mount_roll_deg"]}
        output = Path(output); output.mkdir(parents=True, exist_ok=True)
        overlay = temp / "overlay.urdf"
        overlay.write_text(rotate_tool_description(frames[holdout_index]["bundle"] / "robot.urdf", best["total_mount_roll_deg"]))
        _write_overlay(frames[holdout_index], overlay, best["total_mount_roll_deg"], output / "heldout-gripper-roll.png")
        report["overlay"] = str(output / "heldout-gripper-roll.png")
        report["interpretation"] = "Unqualified offline diagnostic only. ROIs are built independently per candidate, so one-way observed-to-CAD scores are not directly comparable; inspect bidirectional per-link metrics and overlays."
        (output / "candidate.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bundles", nargs="+"); p.add_argument("--output", required=True)
    p.add_argument("--holdout-index", type=int, default=-1); p.add_argument("--mount-roll-deg", type=float, default=90.0)
    p.add_argument("--deltas", default="0,90,180,-90"); p.add_argument("--samples", type=int, default=10000)
    a = p.parse_args(argv)
    r = scan_bundles(a.bundles, output=a.output, holdout_index=a.holdout_index,
        mount_roll_deg=a.mount_roll_deg, deltas=[float(x) for x in a.deltas.split(",")], samples=a.samples)
    print(json.dumps({"status": r["status"], "candidate": str(Path(a.output)/"candidate.json"), "overlay": r["overlay"], "selected_diagnostic_only": r["selected_by_heldout_diagnostic_only"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
