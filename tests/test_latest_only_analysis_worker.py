"""Concurrency contract for the bounded pure RGB-D analysis worker."""
from threading import Event, Lock
import time
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2/src/so101_station_objects"))

from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker


def _wait_result(worker, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = worker.take_result()
        if result is not None:
            return result
        time.sleep(0.002)
    pytest.fail("analysis result did not arrive before timeout")


@pytest.mark.parametrize("interval", [-0.01, float("nan"), float("inf"), "invalid"])
def test_worker_rejects_invalid_start_intervals(interval):
    with pytest.raises(ValueError, match="finite and nonnegative"):
        LatestOnlyAnalysisWorker(lambda value: value, min_start_interval_s=interval)


def test_worker_requires_callable_analyzer():
    with pytest.raises(TypeError, match="callable"):
        LatestOnlyAnalysisWorker(None)


def test_busy_worker_publishes_completion_and_replaces_pending_with_latest_frame():
    first_started = Event()
    release_first = Event()
    latest_started = Event()
    release_latest = Event()
    active_lock = Lock()
    active = 0
    maximum_active = 0
    seen = []

    def analyze(frame):
        nonlocal active, maximum_active
        with active_lock:
            active += 1
            maximum_active = max(maximum_active, active)
        seen.append(frame)
        if frame == 1:
            first_started.set()
            assert release_first.wait(2.0)
        if frame == 50:
            latest_started.set()
            assert release_latest.wait(2.0)
        with active_lock:
            active -= 1
        return frame

    worker = LatestOnlyAnalysisWorker(analyze)
    try:
        assert worker.submit(1) is False  # no pending job replaced
        assert first_started.wait(1.0)
        assert worker.submit(2) is False
        for frame in range(3, 51):
            assert worker.submit(frame) is True  # one replaceable pending slot
        release_first.set()
        assert latest_started.wait(1.0)
        # A pending new frame must not starve publication of completed work.
        assert _wait_result(worker) == (1, 1, None)
        release_latest.set()
        assert _wait_result(worker) == (50, 50, None)
        assert seen == [1, 50]
        assert maximum_active == 1
        assert worker.pending_replacements == 48
    finally:
        release_first.set()
        release_latest.set()
        worker.close()


def test_newer_submission_preserves_completed_but_unconsumed_result():
    first_finished = Event()
    second_started = Event()
    release_second = Event()

    def analyze(frame):
        if frame == 2:
            second_started.set()
            assert release_second.wait(2.0)
        else:
            first_finished.set()
        return frame

    worker = LatestOnlyAnalysisWorker(analyze)
    try:
        worker.submit(1)
        assert first_finished.wait(1.0)
        assert worker.submit(2) is False
        assert second_started.wait(1.0)
        # Submitting a newer input does not erase an already completed result.
        assert _wait_result(worker) == (1, 1, None)
        release_second.set()
        assert _wait_result(worker) == (2, 2, None)
    finally:
        release_second.set()
        worker.close()
