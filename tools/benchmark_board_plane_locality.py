"""Offline exact-output and CPU timing comparison; never opens a device."""

import argparse, hashlib, importlib.util, json, pathlib, statistics, subprocess, time, types
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[1]
REL = "ros2/src/so101_station_perception/so101_station_perception/charuco_plane.py"
BASE = "370d8ee82489d58a1bcce9e6a401dbef7738237d"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture", type=pathlib.Path, action="append", required=True)
    p.add_argument("--output", type=pathlib.Path, required=True)
    p.add_argument("--samples", type=int, default=12)
    a = p.parse_args()
    if not 1 <= len(a.capture) <= 8 or not 2 <= a.samples <= 40:
        p.error("at most8 captures;2..40 samples")
    original = subprocess.check_output(
        ["git", "show", BASE + ":" + REL], cwd=ROOT, text=True
    )
    old = types.ModuleType("baseline_plane")
    exec(compile(original, "baseline_plane", "exec"), old.__dict__)
    spec = importlib.util.spec_from_file_location("current_plane", ROOT / REL)
    new = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(new)
    cv2.setNumThreads(1)
    records = []
    for directory in a.capture:
        paths = [directory / "capture.json", directory / "color.png"]
        paths.append(
            directory
            / (
                "depth.npy"
                if (directory / "depth.npy").exists()
                else "aligned-depth-raw.npy"
            )
        )
        meta = json.loads(paths[0].read_text())
        camera = meta.get("camera", meta.get("camera_info"))
        k = np.array(camera["k"], float).reshape(3, 3)
        d = np.array(camera["d"], float)
        rgb = cv2.cvtColor(cv2.imread(str(paths[1])), cv2.COLOR_BGR2RGB)
        depth = np.load(paths[2], allow_pickle=False)
        if np.issubdtype(depth.dtype, np.integer):
            depth = depth.astype(np.float32) * 0.001
        # Use the frozen canonical board transform as the same seed for both calls.
        # This benchmark qualifies only the plane function, not a new PnP solution.
        if "world_from_camera" in meta:
            camera_from_world = np.linalg.inv(
                np.asarray(meta["world_from_camera"], float)
            )
        else:
            t = meta["transforms"]["charuco_board"]["transform"]
            camera_from_world = np.eye(4)
            camera_from_world[:3, :3] = Rotation.from_quat(
                [t["rotation"][x] for x in "xyzw"]
            ).as_matrix()
            camera_from_world[:3, 3] = [t["translation"][x] for x in "xyz"]
        pose = camera_from_world @ np.diag([1.0, -1.0, -1.0, 1.0])
        timings = {"before": [], "after": []}
        last = None
        for i in range(a.samples):
            results = {}
            for name, fn in (
                [("before", old.fit_depth_plane), ("after", new.fit_depth_plane)]
                if i % 2 == 0
                else [("after", new.fit_depth_plane), ("before", old.fit_depth_plane)]
            ):
                start = time.perf_counter_ns()
                results[name] = fn(depth, k, d, pose, color_image=rgb)
                timings[name].append((time.perf_counter_ns() - start) / 1e6)
            for j in [0, 1]:
                np.testing.assert_array_equal(results["before"][j], results["after"][j])
            assert results["before"][2] == results["after"][2]
            last = results["after"][2]
        records.append(
            {
                "capture": str(directory),
                "shape": list(rgb.shape),
                "input_sha256": {
                    str(x): hashlib.sha256(x.read_bytes()).hexdigest() for x in paths
                },
                "exact_parity": True,
                "metrics": last,
                "timings": {
                    n: {
                        "mean_ms": statistics.mean(v),
                        "p95_ms": float(np.percentile(v, 95)),
                        "samples_ms": v,
                    }
                    for n, v in timings.items()
                },
            }
        )
    result = {
        "scope": "offline CPU board-plane function only; no ROS freshness or physical qualification",
        "baseline_ref": BASE,
        "baseline_source_sha256": hashlib.sha256(original.encode()).hexdigest(),
        "current_source_sha256": hashlib.sha256((ROOT / REL).read_bytes()).hexdigest(),
        "opencv": cv2.__version__,
        "numpy": np.__version__,
        "records": records,
    }
    a.output.parent.mkdir(parents=True, exist_ok=True)
    assert not a.output.exists()
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            [
                {k: r[k] for k in ["capture", "exact_parity", "timings"]}
                for r in records
            ],
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
