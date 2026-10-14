#!/usr/bin/env python3
"""Hardware-free resident-arm CPU benchmark using the development fake bus.

Run from a checkout with the test dependencies installed. No serial SDK is
imported and no device is opened. CPU timing excludes physical bus latency;
the controller's delayed-driver tests cover its cadence fault behavior.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def benchmark(samples=100, plans=10):
    if type(samples) is not int or not 1 <= samples <= 10000:
        raise ValueError("samples must be an integer in 1..10000")
    if type(plans) is not int or not 1 <= plans <= 100:
        raise ValueError("plans must be an integer in 1..100")
    spec = importlib.util.spec_from_file_location(
        "arm_session_benchmark_fakes", ROOT / "tests/test_arm_session.py")
    fakes = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fakes)
    with tempfile.TemporaryDirectory(prefix="arm-session-benchmark-") as folder:
        session, arm, clock = fakes.make_session(Path(folder))
        try:
            session.initialize()
            session.move_plan([{"label": "warm", "target_raw": fakes.target(shoulder_pan=2050)}])
            reads = []
            original_read = arm.bus.sync_read

            def measured_read(register, **kwargs):
                reads.append(register)
                return original_read(register, **kwargs)

            arm.bus.sync_read = measured_read
            durations = []
            for _ in range(samples):
                started = time.perf_counter()
                session.publish_state()
                durations.append((time.perf_counter() - started) * 1000)
            idle_registers = sorted(set(reads))
            reads_per_sample = len(reads) / samples
            arm.bus.writes.clear()
            arm.bus.goal_history.clear()
            reads.clear()
            virtual_started = clock.monotonic()
            started = time.perf_counter()
            for index in range(plans):
                session.move_plan([{"label": f"correction-{index}",
                                    "target_raw": fakes.target(shoulder_pan=2000 if index % 2 == 0 else 2050)}])
            plan_cpu_ms = (time.perf_counter() - started) * 1000
            return {
                "hardware_access": False,
                "samples": samples,
                "publish_mean_ms": statistics.mean(durations),
                "publish_p95_ms": sorted(durations)[min(samples - 1, int(samples * .95))],
                "idle_read_registers": idle_registers,
                "idle_reads_per_sample": reads_per_sample,
                "warm_plans": plans,
                "warm_plan_cpu_ms": plan_cpu_ms,
                "warm_plan_virtual_seconds": clock.monotonic() - virtual_started,
                "warm_profile_writes": list(arm.bus.writes),
                "warm_goal_broadcasts": len(arm.bus.goal_history),
                "warm_read_registers": sorted(set(reads)),
                "target_period_ms": 50,
            }
        finally:
            session.shutdown(reason="benchmark_complete")
            session.close_log()


def benchmark_recorder_pairing(samples=1000):
    """Measure one latest JSON read, pairing, event filter and metadata build."""
    if type(samples) is not int or not 1 <= samples <= 10000:
        raise ValueError("samples must be an integer in 1..10000")
    from tools.record_rgbd_experiment import (arm_transition, pair_arm_state,
                                             read_arm_state, snapshot_metadata)
    from workshop.station.config import JOINTS
    state = {"schema": 1, "event": "sample", "monotonic": 10.,
             "timestamp_utc": "2026-10-14T00:00:00+00:00",
             "label": "reach", "phase": "move_plan",
             "raw": dict.fromkeys(JOINTS, 2000),
             "pose": dict.fromkeys(JOINTS, 0.)}
    durations, edges = [], 0
    label = event = None
    with tempfile.TemporaryDirectory(prefix="recorder-pair-benchmark-") as folder:
        path = Path(folder) / "arm-latest.json"
        path.write_text(json.dumps(state))
        for index in range(samples):
            started = time.perf_counter()
            candidate = read_arm_state(path)
            pair = pair_arm_state(candidate, 10.05)
            token, label, event = arm_transition(candidate, previous_label=label,
                                                  previous_event=event)
            edges += token is not None
            snapshot_metadata(index, "frames/000001.png", "frames/000001.npy",
                              {"monotonic": 10.05, "utc": state["timestamp_utc"],
                               "device_ms": 500., "depth_device_ms": 500.}, pair)
            durations.append((time.perf_counter() - started) * 1000)
        return {"hardware_access": False, "samples": samples,
                "latest_json_bytes": path.stat().st_size,
                "json_reads_per_sample": 1,
                "pairing_mean_ms": statistics.mean(durations),
                "pairing_p95_ms": sorted(durations)[min(samples - 1, int(samples * .95))],
                "repeated_sample_keyframe_edges": edges}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--plans", type=int, default=10)
    args = parser.parse_args()
    print(json.dumps({"controller": benchmark(args.samples, args.plans),
                      "recorder": benchmark_recorder_pairing()}, indent=2))


if __name__ == "__main__":
    main()
