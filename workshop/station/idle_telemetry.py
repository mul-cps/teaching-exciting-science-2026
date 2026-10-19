"""Idle-only latest telemetry. No bus/device access or authority clocks here."""
from collections import deque
from contextlib import contextmanager
import threading
import time


class _Frozen(dict):
    def _deny(self, *args, **kwargs):
        raise TypeError('idle telemetry is immutable')
    __setitem__ = __delitem__ = clear = pop = popitem = setdefault = update = __ior__ = _deny


def _freeze(value):
    if isinstance(value, dict):
        return _Frozen((key, _freeze(item)) for key, item in value.items())
    if isinstance(value, (tuple, list)):
        return tuple(_freeze(item) for item in value)
    if value is None or type(value) in (str, bool, int, float):
        return value
    raise TypeError('nonprimitive idle telemetry')


class IdleTelemetry:
    """One writer, one pending immutable sample, sequenced synchronous barrier.

    A stuck sink cannot be killed by Python. close() has a bounded join; on
    failure the caller must retain the stream/resources for external supervision.
    No daemon thread is silently discarded at process exit.
    """
    def __init__(self, sink):
        self._sink = sink
        self._condition = threading.Condition()
        self._gate = threading.RLock()
        self._pending = None
        self._sequence = self._floor = 0
        self._closed = False
        self._error = None
        self._timing = deque(maxlen=256)
        self._active = {"owner": [], "writer": []}
        self._timing_lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name='owner-idle-telemetry', daemon=False)
        self._thread.start()

    @contextmanager
    def stage(self, name):
        wall, cpu = time.monotonic(), time.thread_time()
        lane = "writer" if self.is_writer_thread else "owner"
        with self._timing_lock:
            if len(self._active[lane]) >= 8:
                raise RuntimeError('idle timing nesting exceeded fixed bound')
            self._active[lane].append(dict(stage=name, started_monotonic=wall,
                                           started_thread_cpu=cpu))
        try:
            yield
        finally:
            event = dict(stage=name, started_monotonic=wall,
                         wall_s=max(0., time.monotonic()-wall),
                         thread_cpu_s=max(0., time.thread_time()-cpu))
            with self._timing_lock:
                self._active[lane].pop()
                self._timing.append(event)

    @property
    def is_writer_thread(self):
        return threading.current_thread() is self._thread

    @property
    def pending_count(self):
        with self._condition:
            return int(self._pending is not None)

    def check(self):
        if self._error is not None:
            raise RuntimeError('idle telemetry writer failed: '+self._error)

    def offer(self, state):
        self.check()
        with self.stage('offer'):
            owned = _freeze(state)
            with self._condition:
                if self._closed:
                    raise RuntimeError('idle telemetry closed')
                self._sequence += 1
                self._pending = (self._sequence, owned)
                self._condition.notify()

    @contextmanager
    def serialized(self, timeout=.05):
        # Advance floor BEFORE gate acquisition. A popped old entry that is
        # waiting for the gate cannot later overwrite a synchronous command.
        with self._condition:
            self._floor = self._sequence
            self._pending = None
        with self.stage('publication_barrier'):
            acquired = self._gate.acquire(timeout=max(0., min(.05, timeout)))
        if not acquired:
            raise TimeoutError('idle telemetry publication barrier exceeded budget')
        try:
            self.check()
            yield
        finally:
            self._gate.release()

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                sequence, state = self._pending
                self._pending = None
            with self._gate:
                if sequence <= self._floor:
                    continue
                try:
                    with self.stage('writer'):
                        self._sink(state)
                except BaseException as error:
                    # Publish failure BEFORE releasing the same gate used by
                    # command admission. No waiter may observe a false healthy
                    # writer between sink failure and its error handoff.
                    self._error = type(error).__name__+': '+str(error)[:512]
                    with self._condition:
                        self._pending = None
                        self._closed = True
                    return

    def diagnostics(self):
        with self._timing_lock:
            timing = list(self._timing)
            active = {lane: [dict(stage) for stage in stages]
                      for lane, stages in self._active.items()}
        return dict(schema='owner-idle-timing-v1', timing=timing, active=active,
                    observed_monotonic=time.monotonic(),
                    pending_count=self.pending_count, writer_error=self._error,
                    writer_alive=self._thread.is_alive())

    def close(self, timeout=.05):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        self._thread.join(max(0., timeout))
        return not self._thread.is_alive()
