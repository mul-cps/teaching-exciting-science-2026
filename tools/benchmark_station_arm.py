"""CPU-only cost of the actual station follower, with virtual time and fake USB.

Run from the repository root: python -m tools.benchmark_station_arm
No device is opened or motor energized. Virtual scheduling removes the deliberate
8 ms waits; measurements include interpolation, raw validation/conversion, two
fake telemetry reads, one fake command, tracking checks and in-memory tracing.
They exclude USB latency, OS scheduling and startup/disk persistence.
"""
import argparse
import json
import platform
import statistics
import time
from types import SimpleNamespace
from unittest.mock import patch

from workshop.station.arm import LeRobotArm
from workshop.station.config import JOINTS


class FakeBus:
    def __init__(self):
        self.raw = dict.fromkeys(JOINTS, 2048)
        self.reads = self.writes = 0

    def sync_read(self, register, **kwargs):
        assert register == 'Present_Position' and kwargs == dict(normalize=False, num_retry=0)
        self.reads += 1
        return dict(self.raw)

    def sync_write(self, register, values, **kwargs):
        assert register == 'Goal_Position' and kwargs == dict(normalize=False, num_retry=0)
        self.writes += 1
        self.raw.update(values)


class VirtualClock:
    def __init__(self):
        self.now = 0.
        self.previous = None
        self.samples = []

    def monotonic(self):
        return self.now

    def is_set(self):
        return False

    def wait(self, delay):
        stamp = time.perf_counter_ns()
        if self.previous is not None:
            self.samples.append(stamp - self.previous)
        self.previous = stamp
        self.now += delay
        return False


def benchmark(repetitions=40, hz=125):
    if repetitions < 1 or not 100 <= hz <= 200:
        raise ValueError('positive repetitions and rate 100..200 required')
    limits = {k: dict(min=0 if k == 'gripper' else -80, max=100 if k == 'gripper' else 80,
                      speed=6, acceleration=12) for k in JOINTS}
    settings = dict(hz=hz, tolerance=2, limits=limits)
    samples, ticks = [], 0
    for _ in range(repetitions):
        bus, clock = FakeBus(), VirtualClock()
        arm = LeRobotArm(dict(arm=settings), bus=bus, abort_event=clock)
        arm.calibration = {k: dict(range_min=1063, range_max=3033) for k in JOINTS}
        arm._bounds()
        start = dict.fromkeys(JOINTS, 0.)
        start['gripper'] = 50.
        target = {k: start[k] + offset for k, offset in zip(JOINTS, (6, -5, 4, -3, 2, 8))}
        bus.raw = arm._raw_goal(start)
        move = arm._segment(start, target)
        trace = []
        with patch('workshop.station.arm.time', SimpleNamespace(monotonic=clock.monotonic)):
            arm._execute(move, move['duration'] + 1, trace)
        assert bus.reads == 2 * bus.writes == 2 * len(trace)
        assert len(trace) >= int(move['duration'] * hz)
        assert all(abs(trace[-1]['actual'][k] - target[k]) < 1 for k in JOINTS)
        samples.extend(clock.samples)
        ticks += len(trace)
    ordered = sorted(samples)
    return dict(python=platform.python_version(), platform=platform.platform(),
                processor=platform.processor(), hz=hz, repetitions=repetitions,
                ticks=ticks, measured_ticks=len(samples), joints=6,
                segment_duration_s=move['duration'],
                median_us=statistics.median(samples) / 1000,
                p95_us=ordered[min(len(ordered)-1, int(.95 * len(ordered)))] / 1000,
                max_us=max(samples) / 1000,
                scope='CPU fake-bus execution; excludes USB latency, OS scheduling, startup and disk I/O')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repetitions', type=int, default=40)
    parser.add_argument('--hz', type=int, default=125)
    args = parser.parse_args()
    print(json.dumps(benchmark(args.repetitions, args.hz), indent=2))


if __name__ == '__main__':
    main()
