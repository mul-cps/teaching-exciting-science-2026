"""One active cohort and one replaceable pending cohort; no image copies."""
import math
from threading import Lock


class LatestCohort:
    def __init__(self, rate_hz):
        if isinstance(rate_hz, bool) or not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('processing rate must be finite and positive')
        self.period = 1.0 / min(5.0, max(.1, rate_hz))
        self._lock = Lock()
        self._pending = None
        self._active = False
        self._closed = False
        self._last_keys = (-1, -1)
        self._next_start = -math.inf
        self._counts = dict(offered=0, overwritten=0, duplicate_or_out_of_order=0,
                            started=0, stale_dropped=0)

    def offer(self, cohort, keys, receipt_monotonic):
        with self._lock:
            if self._closed:
                return False
            if keys[0] <= self._last_keys[0] or keys[1] <= self._last_keys[1]:
                self._counts['duplicate_or_out_of_order'] += 1
                return False
            self._last_keys = keys
            self._counts['offered'] += 1
            if self._pending is not None:
                self._counts['overwritten'] += 1
            self._pending = (cohort, receipt_monotonic)
            return True

    def take(self, now_monotonic):
        with self._lock:
            if self._closed or self._active or self._pending is None or now_monotonic < self._next_start:
                return None
            result, self._pending = self._pending, None
            self._active = True
            self._next_start = now_monotonic + self.period
            self._counts['started'] += 1
            return result

    def complete(self, *, stale=False):
        with self._lock:
            self._active = False
            self._counts['stale_dropped'] += int(stale)

    def metrics(self):
        with self._lock:
            return dict(self._counts)

    def close(self):
        with self._lock:
            self._closed = True
            self._pending = None
