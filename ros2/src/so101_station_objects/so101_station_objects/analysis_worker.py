"""Bounded latest-only worker for pure RGB-D analysis jobs.

The producer may submit camera frames faster than they can be analyzed. One
job may run and one pending job is retained; newer pending jobs replace older
ones. Completed results have a separate slot so incoming frames cannot starve
publication on the ROS executor thread.
"""

from __future__ import annotations

from threading import Condition, Thread
import math
import time


class LatestOnlyAnalysisWorker:
    """Run one analyzer serially with one replaceable pending input."""

    def __init__(self, analyze, *, min_start_interval_s: float = 0.0):
        if not callable(analyze):
            raise TypeError("analyze must be callable")
        try:
            interval = float(min_start_interval_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("min_start_interval_s must be finite and nonnegative") from exc
        if not math.isfinite(interval) or interval < 0:
            raise ValueError("min_start_interval_s must be finite and nonnegative")
        self._analyze = analyze
        self._min_start_interval_s = interval
        self._condition = Condition()
        self._pending = None
        self._result = None
        self._next_sequence = 0
        self._closed = False
        self._last_start = None
        self.pending_replacements = 0
        self.overwritten_results = 0
        self._thread = Thread(target=self._run, name="rgbd-analysis", daemon=True)
        self._thread.start()

    def submit(self, item) -> bool:
        """Submit an item; return True if it replaced an unstarted item."""
        with self._condition:
            if self._closed:
                return False
            self._next_sequence += 1
            replaced = self._pending is not None
            if replaced:
                self.pending_replacements += 1
            self._pending = (self._next_sequence, item)
            self._condition.notify_all()
            return replaced

    def take_result(self):
        """Consume the newest unconsumed ``(item, value, error)`` result."""
        with self._condition:
            if self._result is None:
                return None
            _, item, value, error = self._result
            self._result = None
            return (item, value, error)

    def close(self, timeout: float = 2.0) -> None:
        """Stop accepting work, discard pending input, and join the worker."""
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout)))

    def _run(self):
        while True:
            with self._condition:
                while not self._closed and self._pending is None:
                    self._condition.wait()
                if self._closed:
                    return
                if self._last_start is not None:
                    remaining = self._min_start_interval_s - (time.monotonic() - self._last_start)
                    if remaining > 0:
                        self._condition.wait(timeout=remaining)
                        continue
                sequence, item = self._pending
                self._pending = None
                self._last_start = time.monotonic()

            try:
                value = self._analyze(item)
                error = None
            except Exception as exc:  # deliver analyzer faults on the ROS thread
                value = None
                error = exc

            with self._condition:
                if self._result is not None:
                    old_sequence = self._result[0]
                    if sequence <= old_sequence:
                        continue
                    self.overwritten_results += 1
                self._result = (sequence, item, value, error)
                self._condition.notify_all()

