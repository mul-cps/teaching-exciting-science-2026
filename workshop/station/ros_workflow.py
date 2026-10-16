"""Thread-safe lifecycle for one bounded station job, with explicit fault reset."""
import threading
import time
import uuid


class StationWorkflow:
    def __init__(self, *, clock=time.monotonic):
        self._lock = threading.RLock()
        self.clock = clock
        self.cancel = threading.Event()
        self.thread = None
        self.state = 'idle'
        self.fault = None
        self.job_id = None
        self.result = None
        self.events = []

    def record(self, record):
        with self._lock:
            self.events.append(dict(record, job_id=self.job_id))
            self.events = self.events[-100:]

    def snapshot(self):
        with self._lock:
            return dict(state=self.state, fault=self.fault, job_id=self.job_id,
                        result=self.result, events=list(self.events))

    def start(self, operation, work):
        with self._lock:
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError('station job already active')
            if self.fault is not None:
                raise RuntimeError('station fault latched; inspect and reset explicitly: ' + self.fault)
            self.cancel = threading.Event()
            self.state, self.result = operation, None
            self.job_id = str(uuid.uuid4())
            def run():
                try:
                    result = work(self.cancel)
                    with self._lock:
                        if self.cancel.is_set():
                            raise InterruptedError('station operation cancelled')
                        self.result, self.state = result, 'idle'
                        self.record({'event': 'complete', 'operation': operation})
                except BaseException as error:
                    with self._lock:
                        self.fault, self.state = str(error), 'fault'
                        self.record({'event': 'fault', 'operation': operation, 'error': str(error)})
            self.thread = threading.Thread(target=run, daemon=True)
            self.thread.start()
            return self.job_id

    def stop(self):
        with self._lock:
            self.cancel.set()
            self.record({'event': 'cancel_requested'})

    def reset(self):
        with self._lock:
            if self.thread is not None and self.thread.is_alive():
                raise RuntimeError('cannot reset while a job is active')
            self.fault, self.state = None, 'idle'
            self.record({'event': 'fault_reset'})
