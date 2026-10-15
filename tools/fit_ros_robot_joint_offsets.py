#!/usr/bin/env python3
"""Fit an offline, unqualified multi-pose ROS URDF registration candidate."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.fit_ros_robot_registration import (
    _metrics,
    _project,
    _sha256,
    depth_matches,
    depth_roi_points,
    forward_kinematics,
    parse_urdf,
    sample_visual_surfaces,
)

DEFAULT_OFFSETS = ("shoulder_lift", "elbow_flex", "wrist_flex")
MIN_POSE_CHANGE_RAD = np.deg2rad(3.0)


def validate_pose_diversity(frames, holdout_index, offset_names=DEFAULT_OFFSETS):
    """Require a held-out pose and at least two independently excited train axes."""
    if not 0 <= holdout_index < len(frames):
        raise ValueError("holdout index is outside the saved snapshot list")
    training = [frame for index, frame in enumerate(frames) if index != holdout_index]
    if len(training) < 3:
        raise ValueError("need at least three distinct training poses plus a held-out pose")
    matrix = np.asarray([[float(frame["joint_positions"][name]) for name in offset_names]
                         for frame in training])
    spans = np.ptp(matrix, axis=0)
    if np.count_nonzero(spans >= MIN_POSE_CHANGE_RAD) < 2:
        raise ValueError("training snapshots need at least two distinct training poses with two fitted joints moving >3 degrees")
    centered = matrix - matrix.mean(axis=0)
    active = spans >= MIN_POSE_CHANGE_RAD
    rank = np.linalg.matrix_rank(centered[:, active], tol=np.deg2rad(1.0))
    if rank < 2:
        raise ValueError("training snapshots do not contain two independent joint motion directions")
    heldout = np.asarray([float(frames[holdout_index]["joint_positions"][name])
                          for name in offset_names])
    if np.min(np.linalg.norm(matrix - heldout, axis=1)) < MIN_POSE_CHANGE_RAD:
        raise ValueError("held-out snapshot must use a distinct joint pose")
    return {"training_pose_count": len(training),
            "joint_ranges_deg": dict(zip(offset_names, np.rad2deg(spans).tolist())),
            "independent_motion_rank": int(rank),
            "heldout_pose_distinct": True}


def _pose_model(frame, offsets):
    positions = dict(frame["joint_positions"])
    for name, value in offsets.items():
        positions[name] = float(positions.get(name, 0.0)) + float(value)
    transforms = forward_kinematics(frame["links"], frame["joints"], positions)
    points, normals, labels = [], [], []
    for link, local in frame["points_by_link"].items():
        transform = transforms[link]
        points.append(local @ transform[:3, :3].T + transform[:3, 3])
        normals.append(frame["normals_by_link"][link] @ transform[:3, :3].T)
        labels.extend([link] * len(local))
    return np.concatenate(points), np.concatenate(normals), np.asarray(labels)


def _observed_metrics(frame, delta, offsets):
    points, normals, links = _pose_model(frame, offsets)
    rotation = Rotation.from_rotvec(delta[3:]).as_matrix()
    observed = np.asarray(frame["observed_base"], float)
    model_query = (observed - delta[:3]) @ rotation
    distance, indices = cKDTree(points).query(model_query, workers=1)
    nearest = points[indices]
    residual = np.linalg.norm(model_query - nearest, axis=1)
    result = {"observed_points": int(len(residual)),
              "median_surface_residual_m": float(np.median(residual)),
              "p95_surface_residual_m": float(np.percentile(residual, 95)),
              "per_link": {}, "_raw_residuals": residual}
    nearest_links = links[indices]
    for link in sorted(set(links)):
        values = residual[nearest_links == link]
        result["per_link"][link] = {
            "observed_points": int(len(values)),
            "median_surface_residual_m": float(np.median(values)) if len(values) else None,
            "p95_surface_residual_m": float(np.percentile(values, 95)) if len(values) else None,
        }
    return result


def _residuals(parameters, frames, offset_names, max_points):
    delta = np.asarray(parameters[:6])
    offsets = dict(zip(offset_names, parameters[6:]))
    residuals = []
    rotation = Rotation.from_rotvec(delta[3:]).as_matrix()
    for frame in frames:
        points, normals, _ = _pose_model(frame, offsets)
        tree = cKDTree(points)
        observed = np.asarray(frame["observed_base"], float)
        stride = max(1, (len(observed) + max_points - 1) // max_points)
        observed = observed[::stride]
        model_query = (observed - delta[:3]) @ rotation
        distance, indices = tree.query(model_query, workers=1)
        vectors = model_query - points[indices]
        plane = np.einsum("ij,ij->i", vectors, normals[indices])
        residuals.extend((plane, .35 * distance))
    # Weak metric priors keep noisy or nearly planar views from wandering to
    # the parameter bounds; the fixed pan zero is the yaw gauge convention.
    residuals.append(delta[:3] * .12)
    residuals.append(delta[3:] * .035)
    residuals.append(np.asarray(parameters[6:]) * .018)
    return np.concatenate(residuals)


def fit_joint_offsets(frames, *, holdout_index, offset_names=DEFAULT_OFFSETS,
                      samples_per_frame=3500, max_nfev=55):
    """Fit shared SE(3) base correction and joint zeros; score a frozen pose."""
    offset_names = tuple(offset_names)
    if not offset_names or len(set(offset_names)) != len(offset_names):
        raise ValueError("joint offset names must be nonempty and unique")
    if "shoulder_pan" in offset_names:
        raise ValueError("shoulder_pan zero is gauge-equivalent to base yaw and must remain fixed")
    diversity = validate_pose_diversity(frames, holdout_index, offset_names)
    training = [frame for index, frame in enumerate(frames) if index != holdout_index]
    holdout = frames[holdout_index]
    before_delta = np.zeros(6)
    before_offsets = {name: 0.0 for name in offset_names}
    training_before = [_observed_metrics(frame, before_delta, before_offsets) for frame in training]
    holdout_before = _observed_metrics(holdout, before_delta, before_offsets)

    rotation_limit = np.deg2rad(6.0)
    lower = np.r_[[-.03, -.03, -.03], [-rotation_limit] * 3,
                  [-np.deg2rad(12.0)] * len(offset_names)]
    upper = -lower
    result = least_squares(
        _residuals, np.zeros(6 + len(offset_names)), args=(training, offset_names, samples_per_frame),
        bounds=(lower, upper), loss="soft_l1", f_scale=.003, x_scale="jac", max_nfev=max_nfev,
    )
    delta = result.x[:6]
    offsets = dict(zip(offset_names, result.x[6:].tolist()))
    training_after = [_observed_metrics(frame, delta, offsets) for frame in training]
    holdout_after = _observed_metrics(holdout, delta, offsets)
    return {
        "schema": 1, "status": "unqualified_multiview_candidate", "qualified": False,
        "hardware_access": False, "ros_started": False,
        "fit_mode": "shared bounded SE3 plus selected URDF joint-zero offsets; sampled visual CAD; fixed observed ROI; robust point-to-plane and Euclidean residuals",
        "offset_names": list(offset_names),
        "joint_zero_offsets_rad": offsets,
        "joint_zero_offsets_deg": {name: float(np.rad2deg(value)) for name, value in offsets.items()},
        "base_delta": delta.tolist(),
        "world_from_model_base_correction": np.block([
            [Rotation.from_rotvec(delta[3:]).as_matrix(), delta[:3, None]],
            [np.zeros((1, 3)), np.ones((1, 1))]]).tolist(),
        "diversity": diversity,
        "training_before": _aggregate_metrics(training_before),
        "training_after": _aggregate_metrics(training_after),
        "holdout_before": _public_metrics(holdout_before),
        "holdout_after": _public_metrics(holdout_after),
        "optimizer": {"success": bool(result.success), "message": result.message,
                      "cost": float(result.cost), "nfev": int(result.nfev)},
        "parameter_uncertainty": _local_uncertainty(result, lower, upper, offset_names),
        "bound_hits": _bound_hits(result.x, lower, upper, offset_names),
        "gauge": "shoulder_pan zero fixed; its rotation is indistinguishable from base yaw; wrist_roll and gripper zeros fixed unless explicitly selected and independently excited",
    }


def _aggregate_metrics(items):
    values = np.concatenate([item["_raw_residuals"] for item in items])
    return {"observed_points": int(len(values)),
            "median_surface_residual_m": float(np.median(values)),
            "p95_surface_residual_m": float(np.percentile(values, 95))}


def _public_metrics(item):
    return {key: value for key, value in item.items() if not key.startswith("_")}


def _parameter_names(offset_names):
    return ["base_translation_x_m", "base_translation_y_m", "base_translation_z_m",
            "base_rotation_x_rad", "base_rotation_y_rad", "base_rotation_z_rad"] + [
                f"joint_zero_{name}_rad" for name in offset_names]


def _local_uncertainty(result, lower, upper, offset_names):
    """Approximate local least-squares errors; not calibrated physical uncertainty."""
    jacobian = np.asarray(result.jac, float)
    singular = np.linalg.svd(jacobian, compute_uv=False)
    tolerance = (singular[0] * 1e-8) if len(singular) and singular[0] else 0.0
    rank = int(np.sum(singular > tolerance))
    dof = max(1, jacobian.shape[0] - rank)
    variance = 2.0 * float(result.cost) / dof
    covariance = variance * np.linalg.pinv(jacobian.T @ jacobian, rcond=1e-12)
    standard = np.sqrt(np.maximum(0.0, np.diag(covariance)))
    names = _parameter_names(offset_names)
    return {
        "method": "local linearized least-squares covariance from robust optimizer Jacobian",
        "standard_error": dict(zip(names, standard.tolist())),
        "standard_error_deg": {
            name: float(np.rad2deg(standard[index]))
            for index, name in enumerate(names) if name.endswith("_rad")
        },
        "jacobian_rank": rank, "parameter_count": int(jacobian.shape[1]),
        "condition_number": (float(singular[0] / singular[-1])
                             if len(singular) and singular[-1] > 0 else None),
        "interpretation": "diagnostic only; not a physical confidence interval because depth noise, mesh error, correspondence selection, and robust loss are not fully modeled",
    }


def _bound_hits(parameters, lower, upper, offset_names):
    names = _parameter_names(offset_names)
    tolerance = np.maximum(1e-5, (upper - lower) * 1e-3)
    hits = {}
    for name, value, low, high, tol in zip(names, parameters, lower, upper, tolerance):
        side = "lower" if value - low <= tol else "upper" if high - value <= tol else None
        if side:
            hits[name] = side
    return {"any": bool(hits), "parameters": hits,
            "limits": {"translation_component_m": 0.03,
                       "rotation_component_deg": 6.0,
                       "joint_zero_deg": 12.0}}


def _load_bundle(bundle, samples=10000):
    bundle = Path(bundle).resolve()
    capture = json.loads((bundle / "capture.json").read_text())
    urdf = bundle / "robot.urdf"
    image = cv2.imread(str(bundle / "color.png"), cv2.IMREAD_COLOR)
    depth = np.load(bundle / "depth.npy", allow_pickle=False)
    if image is None or depth.ndim != 2 or image.shape[:2] != depth.shape:
        raise ValueError(f"{bundle}: saved color/depth images missing or dimensions differ")
    if capture.get("depth_encoding") != "16UC1":
        raise ValueError(f"{bundle}: only saved 16UC1 depth is supported")
    q = {name: float(value) for name, value in capture["joint_positions"].items()}
    intrinsics = np.asarray(capture["camera"]["k"], float).reshape(3, 3)
    distortion = np.asarray(capture["camera"].get("d", [0] * 5), float).reshape(-1, 1)
    initial = np.asarray(capture["world_from_base"], float)
    world_from_camera = np.asarray(capture["world_from_camera"], float)
    points, normals, labels, _ = sample_visual_surfaces(
        urdf, capture["mesh_package_paths"], q, samples=samples)
    links, joints = parse_urdf(urdf)
    base_from_link = forward_kinematics(links, joints, q)
    points_by_link, normals_by_link = {}, {}
    for name in sorted(set(labels)):
        selected = labels == name
        transform = base_from_link[name]
        points_by_link[name] = (points[selected] - transform[:3, 3]) @ transform[:3, :3]
        normals_by_link[name] = normals[selected] @ transform[:3, :3]
    tag_path = bundle / "tags" / "gripper-tag-pose-candidate.json"
    tag_pose = (np.asarray(json.loads(tag_path.read_text())["tag_only_world_from_base_candidate"], float)
                if tag_path.is_file() else initial)
    observed_world, observed_pixels = depth_roi_points(
        points, normals, initial, tag_pose, world_from_camera, depth,
        float(capture["depth_scale_m"]), intrinsics, distortion,
        min_world_z_m=.02, max_depth_gap_m=.06)
    inverse = np.linalg.inv(initial)
    observed_base = observed_world @ inverse[:3, :3].T + inverse[:3, 3]
    return {
        "bundle": str(bundle), "capture_stamp": capture["stamp"],
        "joint_positions": q, "links": links, "joints": joints,
        "points_by_link": points_by_link, "normals_by_link": normals_by_link,
        "observed_base": observed_base, "observed_pixels": observed_pixels,
        "world_from_base_initial": initial, "world_from_camera": world_from_camera,
        "depth_raw": depth, "depth_scale": float(capture["depth_scale_m"]),
        "intrinsics": intrinsics, "distortion": distortion,
        "image": image, "urdf_path": str(urdf),
        "urdf_sha256": _sha256(urdf),
    }


def _directional_scores(frame, delta, offsets, max_depth_gap_m=.06):
    points, normals, labels = _pose_model(frame, offsets)
    rotation = Rotation.from_rotvec(delta[3:]).as_matrix()
    model_from_initial = np.eye(4)
    model_from_initial[:3, :3] = rotation
    model_from_initial[:3, 3] = delta[:3]
    world_from_base = frame["world_from_base_initial"] @ model_from_initial
    visible_matches = depth_matches(
        points, normals, labels, world_from_base, frame["world_from_camera"],
        frame["depth_raw"], frame["depth_scale"], frame["intrinsics"],
        frame["distortion"], max_depth_gap_m=max_depth_gap_m)
    visible = _metrics(visible_matches, world_from_base, labels, max_depth_gap_m)
    tight_matches = depth_matches(
        points, normals, labels, world_from_base, frame["world_from_camera"],
        frame["depth_raw"], frame["depth_scale"], frame["intrinsics"],
        frame["distortion"], max_depth_gap_m=.01)
    tight = _metrics(tight_matches, world_from_base, labels, .01)
    visible["coverage_within_10mm"] = _coverage_fraction(tight)
    visible["matched_depth_pixels_within_10mm"] = tight["matched_depth_pixels"]
    for link, metrics in visible["per_link"].items():
        tight_link = tight["per_link"][link]
        metrics["coverage_within_10mm"] = tight_link["overlap_fraction"]
        metrics["matched_depth_pixels_within_10mm"] = tight_link["matched_depth_pixels"]
    return {"observed_to_cad_fixed_population": _public_metrics(_observed_metrics(frame, delta, offsets)),
            "visible_cad_to_depth": visible}


def _coverage_fraction(metrics):
    projected = sum(link["projected_frontface_samples"]
                    for link in metrics["per_link"].values())
    return float(metrics["matched_depth_pixels"] / projected) if projected else 0.0


def _write_overlay(frame, before_delta, before_offsets, after_delta, after_offsets, path):
    output = cv2.cvtColor(frame["image"], cv2.COLOR_BGR2RGB)
    panels = []
    for delta, offsets in ((before_delta, before_offsets), (after_delta, after_offsets)):
        points, _, _ = _pose_model(frame, offsets)
        rotation = Rotation.from_rotvec(delta[3:]).as_matrix()
        world_from_base = frame["world_from_base_initial"].copy()
        world_from_base[:3, :3] = world_from_base[:3, :3] @ rotation
        world_from_base[:3, 3] += frame["world_from_base_initial"][:3, :3] @ delta[:3]
        world = points @ world_from_base[:3, :3].T + world_from_base[:3, 3]
        camera_from_world = np.linalg.inv(frame["world_from_camera"])
        camera = world @ camera_from_world[:3, :3].T + camera_from_world[:3, 3]
        pixels = _project(camera, frame["intrinsics"], frame["distortion"])
        u, v = np.rint(pixels[:, 0]).astype(int), np.rint(pixels[:, 1]).astype(int)
        height, width = frame["depth_raw"].shape
        bounds = ((u >= 0) & (u < width) & (v >= 0) & (v < height)
                  & (camera[:, 2] > 0) & np.isfinite(pixels).all(axis=1))
        keep = np.zeros(len(camera), dtype=bool)
        ids = np.flatnonzero(bounds)
        measured = frame["depth_raw"][v[ids], u[ids]].astype(float) * frame["depth_scale"]
        keep[ids] = (measured > 0) & (np.abs(measured - camera[ids, 2]) <= .06)
        panel = output.copy()
        for pu, pv in np.column_stack((u[keep], v[keep]))[::3]:
            cv2.circle(panel, (int(pu), int(pv)), 1, (255, 40, 40), -1)
        for u, v in frame["observed_pixels"][::3]:
            cv2.circle(panel, (int(u), int(v)), 1, (30, 230, 70), -1)
        panel = cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)
        label = "before" if len(panels) == 0 else "fitted candidate"
        cv2.putText(panel, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .8,
                    (255, 255, 255), 2, cv2.LINE_AA)
        panels.append(panel)
    cv2.imwrite(str(path), np.concatenate(panels, axis=1))


def fit_bundles(bundles, *, output, holdout_index=-1, offsets=DEFAULT_OFFSETS,
                samples=10000, max_nfev=55):
    """Fit saved bundle paths; never modifies bundles or any live state."""
    bundles = [Path(path).resolve() for path in bundles]
    if len(bundles) < 4:
        raise ValueError("provide at least three training bundles and one held-out pose bundle")
    if holdout_index < 0:
        holdout_index += len(bundles)
    # Reject repeated/poorly excited poses before loading depth or fitting.
    captures = [json.loads((bundle / "capture.json").read_text()) for bundle in bundles]
    pose_frames = [{"joint_positions": capture["joint_positions"]} for capture in captures]
    diversity = validate_pose_diversity(pose_frames, holdout_index, offsets)
    frames = [_load_bundle(bundle, samples=samples) for bundle in bundles]
    hashes = {frame["urdf_sha256"] for frame in frames}
    if len(hashes) != 1:
        raise ValueError("all snapshots must use the same captured ROS URDF")
    report = fit_joint_offsets(frames, holdout_index=holdout_index,
                               offset_names=offsets, max_nfev=max_nfev)
    train_ids = [index for index in range(len(frames)) if index != holdout_index]
    delta = np.asarray(report["base_delta"], float)
    joint_offsets = report["joint_zero_offsets_rad"]
    before_offsets = {name: 0.0 for name in offsets}
    report.update({
        "snapshots": [{"bundle": frame["bundle"], "capture_stamp": frame["capture_stamp"],
                       "joint_positions_rad": frame["joint_positions"],
                       "role": "heldout" if index == holdout_index else "training"}
                      for index, frame in enumerate(frames)],
        "urdf_sha256": next(iter(hashes)), "diversity": diversity,
        "directional_scores": {
            "training_before": [_directional_scores(frames[i], np.zeros(6), before_offsets)
                                for i in train_ids],
            "training_after": [_directional_scores(frames[i], delta, joint_offsets)
                               for i in train_ids],
            "heldout_before": _directional_scores(frames[holdout_index], np.zeros(6), before_offsets),
            "heldout_after": _directional_scores(frames[holdout_index], delta, joint_offsets),
        },
        "limitations": [
            "candidate is unqualified; no live transform is applied",
            "shoulder_pan zero is fixed because it is gauge-equivalent to base yaw",
            "observed point populations are z-buffer ROI samples fixed from nominal and tag pose seeds",
            "held-out pose is scored only and never included in optimization",
            "gripper angle is the station's opening-percent proxy, not a calibrated encoder-angle mapping; jaw residuals do not calibrate gripper kinematics",
        ],
    })
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    _write_overlay(frames[holdout_index], np.zeros(6), before_offsets,
                   delta, joint_offsets, output / "heldout-registration.png")
    (output / "candidate.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundles", nargs="+", help="saved RGB-D snapshot bundle directories")
    parser.add_argument("--output", required=True, help="directory for candidate JSON and heldout overlay")
    parser.add_argument("--holdout-index", type=int, default=-1,
                        help="bundle index scored without fitting (default: last)")
    parser.add_argument("--offsets", default=",".join(DEFAULT_OFFSETS),
                        help="comma-separated selected joint zeros; pan remains gauge-fixed")
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--max-nfev", type=int, default=55)
    args = parser.parse_args(argv)
    names = tuple(item.strip() for item in args.offsets.split(",") if item.strip())
    report = fit_bundles(args.bundles, output=args.output, holdout_index=args.holdout_index,
                         offsets=names, samples=args.samples,
                         max_nfev=args.max_nfev)
    print(json.dumps({"status": report["status"], "candidate": str(Path(args.output) / "candidate.json"),
                      "overlay": str(Path(args.output) / "heldout-registration.png"),
                      "joint_zero_offsets_deg": report["joint_zero_offsets_deg"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

