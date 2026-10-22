"""Offline paired cold calibration status costs; no ROS initialization or actions."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time
import types

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'workshop/blockly_ros.py'
BASE = '25dfc5ef9e640cb4c267e6d5fc4bf4905f8e709f'
ENDPOINT = '/follower/gripper_calibration_status'
sys.path.insert(0, str(ROOT))


def load_source(source):
    module = types.ModuleType('workshop._offline_calibration')
    module.__package__ = 'workshop'
    exec(compile(source, str(SOURCE), 'exec'), module.__dict__)
    return module


def response(case):
    si, angular = {'si_only': (True, False), 'both': (True, True),
                   'neither': (False, False), 'malformed': (True, False),
                   'max_reason': (True, True), 'max_payload': (True, True)}[case]
    data = dict(schema='so101-gripper-calibration-status-v1', si_gap_calibrated=si,
                moveit_joint_calibrated=angular, si_position_units='metres',
                moveit_position_units='radians', visual_proxy_only=not angular,
                reason=('qualified gap and joint-angle calibration loaded' if si and angular else
                        'MoveIt gripper requires qualified gap and identity-matched joint-angle calibration; no motion commanded'))
    if case == 'malformed': data['si_gap_calibrated'] = 1
    if case == 'max_reason': data['reason'] = 'x'*1000
    if case == 'max_payload':
        data['unused_diagnostic'] = ''
        data['unused_diagnostic'] = 'x'*(8192-len(json.dumps(data)))
    return types.SimpleNamespace(success=si and angular, message=json.dumps(data))


class Future:
    def __init__(self, result): self.value = result
    def done(self): return True
    def cancelled(self): return False
    def result(self): return self.value
    def add_done_callback(self, callback): callback(self)


class Client:
    def __init__(self, future): self.future = future
    def service_is_ready(self): return True
    def call_async(self, request): return self.future
    def remove_pending_request(self, future): raise AssertionError('unexpected cancellation')


def fixture(module, result):
    obj = object.__new__(module.ROSBackend)
    obj.lock = threading.Lock(); obj.dispatch_lock = threading.Lock(); obj.cache = {}
    obj.calibration_pending = None; obj.calibration_pending_at = 0.
    obj.calibration_client = Client(Future(result))
    obj.clients = {ENDPOINT: obj.calibration_client}; obj.pending_mutations = set()
    obj.Trigger = types.SimpleNamespace(Request=lambda: None)
    return obj


def benchmark(iterations=5000, batches=7):
    source = {'baseline': subprocess.check_output(['git', 'show', BASE+':workshop/blockly_ros.py'],
                                                 cwd=ROOT, text=True), 'candidate': SOURCE.read_text()}
    modules = {key: load_source(value) for key, value in source.items()}
    results = {}
    for case in ('si_only', 'both', 'neither', 'malformed', 'max_reason', 'max_payload'):
        result = response(case)
        measurements = {}
        for route in ('cold_refresh_and_decode', 'current_readonly_service'):
            samples = {key: [] for key in modules}
            for batch in range(batches):
                for name in (list(modules) if batch % 2 == 0 else list(reversed(modules))):
                    obj = fixture(modules[name], result)
                    cancel = threading.Event(); deadline = time.monotonic()+3600.
                    def invoke():
                        if route == 'cold_refresh_and_decode':
                            obj._refresh_calibration()
                            return obj.cache.get('gripper_calibration') is not None
                        try:
                            obj.trigger(ENDPOINT, cancel, deadline)
                            return True
                        except RuntimeError:
                            return False
                    for _ in range(30): invoke()
                    started = time.perf_counter_ns()
                    for _ in range(iterations): accepted = invoke()
                    samples[name].append((time.perf_counter_ns()-started)/iterations/1000.)
                    expected = (case != 'malformed' if name == 'candidate' else
                                route == 'cold_refresh_and_decode' or case in ('both', 'max_reason', 'max_payload'))
                    assert accepted is expected, (name, case, route, accepted)
                    assert not obj.pending_mutations
            timings = {name: dict(median_batch_us=statistics.median(values),
                                  max_batch_us=max(values)) for name, values in samples.items()}
            measurements[route] = dict(**timings, delta_median_us=(
                timings['candidate']['median_batch_us']-timings['baseline']['median_batch_us']))
        results[case] = dict(message_chars=len(result.message), **measurements)
    return dict(host=platform.node(), python=platform.python_version(), executable=sys.executable,
                baseline_commit=BASE, candidate_sha256=hashlib.sha256(source['candidate'].encode()).hexdigest(),
                iterations_per_batch=iterations, batches=batches, results=results,
                scope='Actual baseline/candidate methods with immediate local fake futures; constructor never called; no ROS, threads, actions or device calls',
                limitation='CPU batch means for cold polling and current read-only metadata; excludes DDS/network/action latency and physical calibration')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=5000)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.iterations < 1: parser.error('--iterations must be positive')
    encoded = json.dumps(benchmark(args.iterations), indent=2)+'\n'
    if args.output: args.output.write_text(encoded)
    print(encoded, end='')
