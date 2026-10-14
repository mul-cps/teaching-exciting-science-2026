#!/usr/bin/env python3
"""CPU-only cold CAD build and warm displaced-target SO101 IK benchmark.

The explicit zero pinch offset isolates solver cost; it is not a grasp model.
Run from the checkout: python -m tools.benchmark_so101_ik --samples 30
"""
import argparse
import json

import numpy as np

from workshop.digital_twin.ik import ARM_JOINTS, SO101IK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=30)
    args = parser.parse_args()
    calibration = {name: {"range_min": 1200, "range_max": 2900} for name in (*ARM_JOINTS, "gripper")}
    raw = {name: 2050 for name in calibration}
    with SO101IK(calibration, lambda opening: np.eye(4)) as ik:
        print(json.dumps(ik.benchmark(raw, samples=args.samples), indent=2))


if __name__ == "__main__":
    main()
