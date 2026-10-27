#!/usr/bin/env python3
"""Measure amortized CPU call cost of the canonical principal-axis helper."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from workshop.digital_twin import principal_axes  # noqa: E402


def positive(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def nonnegative(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be nonnegative")
    return number


def native_thread_count() -> int | None:
    """Linux process threads; unavailable platforms/filesystems return None."""
    try:
        return sum(1 for _ in Path("/proc/self/task").iterdir())
    except OSError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=positive, default=2000)
    parser.add_argument("--samples", type=positive, default=7)
    parser.add_argument("--warmup", type=nonnegative, default=256)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        parser.error(f"output already exists: {args.output}")

    axes2 = np.array([[0.6, 0.8], [-0.8, 0.6]], dtype=np.float64)
    axes3 = np.array([[0.6, 0.8, 0.0], [-0.8, 0.6, 0.0],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
    # Every input array is constructed once and reused throughout timing.
    cases = [
        ("distinct2", np.array([3., 1.]), axes2),
        ("tied2", np.array([2., 2.]), axes2),
        ("distinct3", np.array([3., 2., 1.]), axes3),
        ("tied_top_pair3", np.array([3., 3., 1.]), axes3),
        ("tied_bottom_pair3", np.array([3., 1., 1.]), axes3),
        ("tied_all3", np.array([2., 2., 2.]), axes3),
        ("rank_deficient3", np.array([3., 0., 0.]), axes3),
    ]
    helper = principal_axes.canonical_principal_axes
    source = Path(principal_axes.__file__).resolve()
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    results = []
    for name, spectrum, axes in cases:
        for _ in range(args.warmup):
            helper(spectrum, axes, sample_count=128)
        samples = []
        for _ in range(args.samples):
            threads_before = native_thread_count()
            wall_start = time.perf_counter_ns()
            cpu_start = time.process_time_ns()
            for _ in range(args.iterations):
                final_axes, final_mask = helper(spectrum, axes, sample_count=128)
            cpu_elapsed = time.process_time_ns() - cpu_start
            wall_elapsed = time.perf_counter_ns() - wall_start
            threads_after = native_thread_count()
            # Consume the final return value outside both timed intervals.
            checksum = float(np.sum(final_axes)) + float(np.sum(final_mask))
            samples.append({"wall_ns_per_call": wall_elapsed / args.iterations,
                            "cpu_ns_per_call": cpu_elapsed / args.iterations,
                            "native_process_threads_before": threads_before,
                            "native_process_threads_after": threads_after,
                            "checksum": checksum})
        result = {"name": name, "spectrum": spectrum.tolist(),
                  "axes_rows": axes.tolist(), "dtype": str(spectrum.dtype),
                  "sample_count": 128, "samples": samples}
        for clock in ("wall", "cpu"):
            values = [sample[f"{clock}_ns_per_call"] for sample in samples]
            result[f"{clock}_ns_per_call_percentiles_of_amortized_sample_means"] = {
                "p50": float(np.percentile(values, 50)),
                "p95": float(np.percentile(values, 95)),
            }
        results.append(result)
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_sha:
        raise RuntimeError("helper source changed during measurement")
    report = {
        "python": sys.version, "numpy": np.__version__,
        "platform": platform.platform(), "machine": platform.machine(),
        "numerical_thread_environment": {name: os.environ.get(name) for name in
            ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
        "helper_source": str(source.relative_to(ROOT)),
        "helper_source_sha256": source_sha,
        "benchmark_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "iterations_per_sample": args.iterations, "samples_per_case": args.samples,
        "warmup_calls_per_case": args.warmup,
        "method": "perf_counter_ns wall and process_time_ns CPU; p50/p95 over amortized sample means, not individual-call latency",
        "scope": "Python call, validation, output allocation, loop and timing overhead included; preallocated fixed float64 inputs; no per-call input generation or copying by benchmark",
        "limitations": "Numerical-thread environment recorded, not affinity or power isolation; CPU helper only; no GPU, frame latency or FPS claim",
        "cases": results,
    }
    payload = json.dumps(report, indent=2) + "\n"
    if args.output is None:
        sys.stdout.write(payload)
    else:
        with args.output.open("x", encoding="utf-8") as output:
            output.write(payload)


if __name__ == "__main__":
    main()
