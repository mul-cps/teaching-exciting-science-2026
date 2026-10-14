"""CPU fake drivers and TCP loopback costs of the persistent native EV3 API.

Run: python -m tools.benchmark_ev3_live_api
No hardware, SSH, external network, GPU or native daemon is opened. This measures
real LiveAPI/Peer JSON processing and the PC client's socket round trips against
a loopback adapter. Virtual 20 ms ticks exercise actual finite shake jobs, LCD
and sensor sampling. Results exclude physical driver latency and EV3 scheduling.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import platform
import socket
import statistics
import threading
import time
from types import SimpleNamespace

from workshop.station.ev3_live_client import EV3LiveClient

NATIVE = Path(__file__).resolve().parent / 'ev3_native' / 'live_api.py'


def load_native():
    spec = importlib.util.spec_from_file_location('ev3_live_benchmark_native', NATIVE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Clock:
    def __init__(self): self.ms = 0
    def __call__(self): return self.ms


class Motor:
    def __init__(self):
        self.value, self.target = 0, None
        self.reads = self.speed_reads = self.dispatches = 0
        self.settings = (999, 999, 99)
        self.control = SimpleNamespace(limits=self.limits, done=self.done, stalled=lambda: False,
                                       target_tolerances=lambda: (10, 2),
                                       stall_tolerances=lambda: (30, 100))

    def limits(self, *args, **kwargs):
        if args: self.settings = args
        if kwargs: self.settings = (kwargs['speed'], kwargs['acceleration'], kwargs['actuation'])
        return self.settings

    def done(self): return self.target is None or abs(self.value - self.target) <= .1
    def angle(self): self.reads += 1; return int(round(self.value))
    def speed(self): self.speed_reads += 1; return 0
    def brake(self): self.target = None
    def reset_angle(self, value): self.value = value

    def run_target(self, speed, target, then, wait):
        if wait is not False: raise AssertionError('blocking motor call')
        self.target = target
        self.dispatches += 1

    def advance(self):
        if self.target is not None:
            delta = self.target - self.value
            self.value += min(abs(delta), 5) * (1 if delta > 0 else -1)


class Touch:
    def __init__(self, motor): self.motor, self.reads = motor, 0
    def pressed(self): self.reads += 1; return self.motor.value <= 0


class Color:
    def __init__(self): self.reads = 0
    def color(self): self.reads += 1; return 'RED'


class Screen:
    def __init__(self): self.frames, self.lines = 0, []
    def clear(self): self.frames += 1; self.lines = []
    def draw_text(self, x, y, text): self.lines.append((x, y, text))


class Speaker:
    def beep(self, **kwargs):
        if kwargs.get('duration', 0) >= 0: raise AssertionError('blocking beep')
    def set_volume(self, volume, which): pass


class Harness:
    """Fake hardware around the actual native public request/tick interface."""
    def __init__(self):
        self.native = load_native()
        self.clock = Clock()
        self.motors = {'sorter': Motor(), 'feeder': Motor()}
        self.touch, self.color = Touch(self.motors['sorter']), Color()
        self.brick = SimpleNamespace(screen=Screen(), speaker=Speaker())
        self.api = self.native.LiveAPI(self.motors, self.touch, self.color, self.brick,
                                       self.clock, SimpleNamespace(BRAKE='BRAKE'), 'benchmark')
        self.peer = self.native.Peer(self.api)
        self.next_id = 1

    def message(self, operation, args):
        result = {'id': self.next_id, 'operation': operation, 'args': args}
        self.next_id += 1
        return result

    def request(self, operation, **args):
        return self.api.request(self.message(operation, args))

    def json_roundtrip(self, operation, args=None):
        payload = json.dumps(self.message(operation, {} if args is None else args), separators=(',', ':')).encode('ascii') + b'\n'
        replies = self.peer.feed(payload)
        if len(replies) != 1: raise AssertionError('missing native reply')
        return json.loads(self.native.encoded(replies[0]))

    def read_counts(self):
        return {**{name: {'angle': motor.reads, 'speed': motor.speed_reads}
                   for name, motor in self.motors.items()},
                'touch': self.touch.reads, 'color': self.color.reads}

    def start_shake(self):
        confirmation = self.request('confirm_reference', name='sorter', reference='home')
        if not confirmation['ok']: raise AssertionError(confirmation)
        self.motors['sorter'].value = 132
        reply = self.request('shake', frequency_hz=12.5, speed_dps=600,
                             acceleration=30000, duration_ms=2000)
        if not reply['ok']: raise AssertionError(reply)

    def advance(self, samples=None):
        self.clock.ms += 20
        for motor in self.motors.values(): motor.advance()
        self.request('heartbeat')
        if samples is None: self.api.tick()
        else: measure(samples, self.api.tick)


class Loopback:
    """One real TCP connection, using native framing and response generation."""
    def __init__(self):
        self.harness = Harness()
        self.requests, self.errors = 0, []
        self.stopped = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(1)
        self.listener.settimeout(.1)
        self.port = self.listener.getsockname()[1]
        self.connection = None
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        try:
            while not self.stopped.is_set():
                try: self.connection, _ = self.listener.accept(); break
                except socket.timeout: continue
            if self.connection is None: return
            self.connection.settimeout(.1)
            self.connection.sendall(self.harness.native.encoded(self.harness.api.ready()))
            while not self.stopped.is_set():
                try: data = self.connection.recv(4096)
                except socket.timeout: continue
                if not data: break
                for reply in self.harness.peer.feed(data):
                    self.requests += 1
                    self.connection.sendall(self.harness.native.encoded(reply))
        except BaseException as error:
            if not self.stopped.is_set(): self.errors.append(repr(error))
        finally:
            self.harness.peer.close('benchmark_exit')
            if self.connection is not None: self.connection.close()

    def close(self):
        self.stopped.set()
        self.listener.close()
        if self.connection is not None:
            try: self.connection.shutdown(socket.SHUT_RDWR)
            except OSError: pass
        self.thread.join(2)
        if self.thread.is_alive() or self.errors:
            raise AssertionError('loopback cleanup failed: ' + str(self.errors))


def summary(samples):
    ordered = sorted(samples)
    return {'samples': len(samples), 'median_us': statistics.median(samples) / 1000,
            'p95_us': ordered[min(len(ordered) - 1, int(.95 * len(ordered)))] / 1000,
            'max_us': max(samples) / 1000}


def measure(samples, callback):
    started = time.perf_counter_ns()
    result = callback()
    samples.append(time.perf_counter_ns() - started)
    return result


def benchmark(iterations=1000, ticks=250):
    for name, value in (('iterations', iterations), ('ticks', ticks)):
        if type(value) is not int or not 1 <= value <= 10000:
            raise ValueError(name + ' must be an integer in 1..10000')
    harness = Harness()
    harness.api.tick()
    timings = {name: [] for name in ('status_json', 'heartbeat_json', 'status_loopback',
                                    'heartbeat_loopback', 'tick', 'shake_tick', 'display_json', 'screen_tick')}
    for _ in range(iterations):
        for operation in ('status', 'heartbeat'):
            reply = measure(timings[operation + '_json'], lambda op=operation: harness.json_roundtrip(op))
            if not reply['ok']: raise AssertionError(reply)
    loopback = Loopback()
    try:
        with EV3LiveClient(port=loopback.port, timeout=2) as client:
            for _ in range(iterations):
                for operation in ('status', 'heartbeat'):
                    measure(timings[operation + '_loopback'], lambda op=operation: client.call(op))
        requests = loopback.requests
    finally:
        loopback.close()
    for _ in range(ticks): harness.advance(timings['tick'])
    idle_frames = harness.brick.screen.frames
    harness.request('power_save')
    speed_reads_before_sleep_status = sum(motor.speed_reads for motor in harness.motors.values())
    for _ in range(iterations):
        harness.json_roundtrip('status')
        harness.json_roundtrip('heartbeat')
    sleep_status_speed_reads = (sum(motor.speed_reads for motor in harness.motors.values()) -
                                speed_reads_before_sleep_status)
    shake = Harness()
    shake.start_shake()
    # Fixed virtual run covers the job and quiet settling without physical time.
    for _ in range(130): shake.advance(timings['shake_tick'])
    last = shake.api.status()['last_job']
    if last is None or last['state'] != 'complete': raise AssertionError(last)
    screen = Harness()
    for index in range(iterations):
        text = 'Sample ' + str(index)
        reply = measure(timings['display_json'], lambda: screen.json_roundtrip('display', {'text': text}))
        if not reply['ok']: raise AssertionError(reply)
        screen.clock.ms += 500
        measure(timings['screen_tick'], screen.api.tick)
    return {'scope': 'CPU fake drivers and TCP loopback; excludes physical driver latency, EV3 scheduling and SSH',
            'python': platform.python_version(), 'platform': platform.platform(),
            'iterations': iterations, 'tick_ms': 20, 'timings': {k: summary(v) for k, v in timings.items()},
            'loopback_requests': requests, 'display': {'updates': iterations, 'screen_frames': screen.brick.screen.frames},
            'idle': {'screen_frames': idle_frames, 'driver_reads': harness.read_counts(),
                     'sleep_status_heartbeat_speed_reads': sleep_status_speed_reads},
            'shake': {'state': last['state'], 'ticks': 130, 'screen_frames': shake.brick.screen.frames,
                      'motor_dispatches': shake.motors['sorter'].dispatches, 'driver_reads': shake.read_counts()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=1000)
    parser.add_argument('--ticks', type=int, default=250)
    args = parser.parse_args()
    print(json.dumps(benchmark(args.iterations, args.ticks), indent=2))


if __name__ == '__main__': main()
