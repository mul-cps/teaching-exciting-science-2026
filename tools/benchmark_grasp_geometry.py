"""Run: python -m tools.benchmark_grasp_geometry (synthetic CPU timings)."""
import json
import time
import numpy as np
from workshop.digital_twin.grasp_geometry import fuse_depth, fit_support_plane


def main():
    rng = np.random.default_rng(17)
    stack = rng.integers(498, 503, (20, 480, 640), dtype=np.uint16)
    xy = rng.uniform(-.1, .1, (5000, 2))
    points = np.column_stack((xy, .5 + .2 * xy[:, 0] + rng.normal(0, .0003, 5000)))
    results = {}
    for name, operation in [('fusion_N20_150x190', lambda: fuse_depth(stack, (100, 100, 290, 250))),
                            ('plane_N5000_RANSAC120_IRLS8', lambda: fit_support_plane(points))]:
        operation()
        elapsed = []
        for _ in range(20):
            start = time.perf_counter()
            operation()
            elapsed.append((time.perf_counter() - start) * 1000)
        results[name] = dict(median_ms=float(np.median(elapsed)), p95_ms=float(np.percentile(elapsed, 95)))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
