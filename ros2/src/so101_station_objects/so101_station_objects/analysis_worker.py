"""Bounded latest-only worker for pure RGB-D analysis jobs.

The producer may submit camera frames faster than they can be analyzed. One
job may run and one pending job is retained; newer pending jobs replace older
ones. Completed results have a separate slot so incoming frames cannot starve
publication on the ROS executor thread.
"""

from __future__ import annotations

from threading import Condition, Lock, Thread
import math
import time


class LatestOnlyAnalysisWorker:
    """Run one analyzer serially with one replaceable pending input."""

    def __init__(self, analyze, *, min_start_interval_s: float = 0.0, on_ready=None, diagnostics=False):
        if type(diagnostics) is not bool:
            raise TypeError("diagnostics must be bool")
        self._diagnostics = diagnostics
        self._pending_timing = self._result_timing = None
        self._diagnostic_active = False
        self.diagnostic_losses = {"shutdown": 0} if diagnostics else None
        if not callable(analyze):
            raise TypeError("analyze must be callable")
        try:
            interval = float(min_start_interval_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("min_start_interval_s must be finite and nonnegative") from exc
        if not math.isfinite(interval) or interval < 0:
            raise ValueError("min_start_interval_s must be finite and nonnegative")
        if on_ready is not None and not callable(on_ready):
            raise TypeError("on_ready must be callable or None")
        self._analyze = analyze
        self._on_ready = on_ready
        self._notification_gate = Lock()
        self._notification_error = None
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
            if self._diagnostics:
                self._pending_timing = {"sequence": self._next_sequence, "submit": time.monotonic()}
            self._pending = (self._next_sequence, item)
            self._condition.notify_all()
            return replaced

    def take_result_with_diagnostics(self):
        """Opt-in sidecar; ordinary take_result retains its public triple."""
        if not self._diagnostics:
            raise ValueError("worker diagnostics disabled")
        return self._take_result(True)

    def take_result(self):
        """Consume newest public (item, value, error), in either mode."""
        return self._take_result(False)

    def _take_result(self, with_timing):
        """Consume the newest unconsumed ``(item, value, error)`` result."""
        with self._condition:
            if self._result is None:
                return None
            _, item, value, error = self._result
            self._result = None
            timing = self._result_timing
            self._result_timing = None
            result = (item, value, error)
            return (result, timing) if with_timing else result

    @property
    def notification_error(self):
        with self._condition:
            return self._notification_error

    def close(self, timeout: float = 2.0) -> bool:
        """Revoke wakeups before bounded join; False requires retaining ROS guard.

        Native analyze/notification calls cannot be killed by Python. A caller
        must not destroy its guard/context unless this method returns True.
        """
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError("close timeout must be finite and nonnegative")
        deadline = time.monotonic()+timeout
        with self._condition:
            if self._diagnostics and not self._closed:
                self.diagnostic_losses["shutdown"] += int(self._pending is not None) + int(self._result is not None) + int(self._diagnostic_active)
            self._closed = True
            self._pending = None
            self._result = None
            self._on_ready = None
            self._condition.notify_all()
        acquired = self._notification_gate.acquire(timeout=max(0.,deadline-time.monotonic()))
        if not acquired:
            return False
        self._notification_gate.release()
        self._thread.join(timeout=max(0.,deadline-time.monotonic()))
        return not self._thread.is_alive()

    def _notify_result(self):
        if self._on_ready is None:
            return
        # Callback outside analysis condition; separate gate joins any in-flight
        # notification before ROS resources can be destroyed by the caller.
        with self._notification_gate:
            with self._condition:
                callback = None if self._closed else self._on_ready
            if callback is not None:
                try:
                    callback()
                except Exception as error:
                    with self._condition:
                        self._notification_error = (type(error).__name__+': '+str(error))[:512]
                        self._on_ready = None

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
                timing = self._pending_timing if self._diagnostics else None
                if self._diagnostics:
                    timing["start"] = self._last_start
                    self._diagnostic_active = True

            if timing is not None:
                timing["analysis_enter"] = time.monotonic()
            try:
                value = self._analyze(item)
                error = None
            except Exception as exc:  # deliver analyzer faults on the ROS thread
                value = None
                error = exc

            if timing is not None:
                timing["analysis_return"] = time.monotonic()
            with self._condition:
                self._diagnostic_active = False
                if self._closed:
                    return
                if self._result is not None:
                    old_sequence = self._result[0]
                    if sequence <= old_sequence:
                        continue
                    self.overwritten_results += 1
                if timing is not None:
                    timing["store_enter"] = time.monotonic()
                self._result_timing = timing
                self._result = (sequence, item, value, error)
                self._condition.notify_all()
            self._notify_result()

