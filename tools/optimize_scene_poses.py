#!/usr/bin/env python3
"""Optimize cached station scene poses, or run a small deterministic benchmark."""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workshop.perception.scene_pose_optimizer import optimize_scene_poses


def _benchmark() -> dict:
    """Small CPU-only synthetic timing; excludes detector/camera startup."""
    start = np.eye(4)
    truth = np.eye(4)
    truth[:3, 3] = [.24, -.11, .63]
    mounts = []
    for i in range(12):
        mount = np.eye(4)
        angle = i * .47
        mount[:3, :3] = _yaw(angle)
        mount[:3, 3] = [.09 * np.cos(angle), .07 * np.sin(angle), .015 * (i % 3)]
        mounts.append(mount)
    problem = {"world_frame": "charuco_board", "entities": [
        {"id": "ev3_fixture", "kind": "rigid", "initial_world_from_entity": start.tolist()}],
        "observations": []}
    for i, mount in enumerate(mounts):
        measured = truth @ mount
        if i == 9:
            measured = measured.copy()
            measured[:3, 3] += [.12, -.08, .05]
        problem["observations"].append({
            "track_id": f"fixture-tag-{i}", "entity_id": "ev3_fixture", "pose_kind": "pose",
            "entity_from_tag": mount.tolist(), "world_from_tag": measured.tolist(),
            "sigma_translation_m": .003, "sigma_rotation_rad": .04,
        })
    # Warm SciPy import and solver setup, then report steady calls only.
    candidate = optimize_scene_poses(problem, loss="soft_l1", f_scale=1.)
    timings = []
    for _ in range(5):
        t0 = time.perf_counter()
        candidate = optimize_scene_poses(problem, loss="soft_l1", f_scale=1.,
                                         initial_solution=candidate)
        timings.append((time.perf_counter() - t0) * 1000)
    return {"steady_solve_median_ms": float(np.median(timings)),
            "steady_solve_p95_ms": float(np.percentile(timings, 95)),
            "timed_repetitions": len(timings),
            "observations": len(mounts), "metrics": candidate["metrics"],
            "translation_error_mm": (np.linalg.norm(np.asarray(candidate["entities"]["ev3_fixture"])
                                                       [:3, 3] - truth[:3, 3]) * 1000)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, help="normalized observation problem JSON")
    parser.add_argument("-o", "--output", type=Path, help="write candidate JSON here (default stdout)")
    parser.add_argument("--initial-solution", type=Path,
                        help="reuse a prior candidate JSON as the starting pose")
    parser.add_argument("--max-nfev", type=int, default=100)
    parser.add_argument("--loss", choices=["soft_l1", "huber", "cauchy", "arctan", "linear"], default="soft_l1")
    parser.add_argument("--f-scale", type=float, default=1.0,
                        help="robust-loss transition in normalized residual units")
    parser.add_argument("--benchmark", action="store_true", help="run a deterministic synthetic CPU timing")
    args = parser.parse_args()
    if args.benchmark:
        output = _benchmark()
    else:
        if args.input is None:
            parser.error("input is required unless --benchmark is selected")
        problem = json.loads(args.input.read_text())
        prior = json.loads(args.initial_solution.read_text()) if args.initial_solution else None
        output = optimize_scene_poses(problem, max_nfev=args.max_nfev, loss=args.loss,
                                      f_scale=args.f_scale, initial_solution=prior)
        output["source"] = str(args.input)
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered)
    else:
        print(rendered, end="")
    return 0


def _yaw(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


if __name__ == "__main__":
    raise SystemExit(main())
