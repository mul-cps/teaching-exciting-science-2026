"""CPU-only EV3 terminal classification and cold completion evidence benchmark.

Run: python tests/ev3_job_benchmark.py --iterations 1000 --output PATH
Frozen old production work() is compiled without importing/initializing ROS.
All proxy replies and waits are local fixtures; no hardware, sockets or services.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time
import timeit
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from workshop.station.ev3_job import _terminal, run_ev3_operation

BASELINE = 'ce2362c54500f185a6c877af7e7136accbc846b6'
HELPER = ROOT / 'workshop/station/ev3_job.py'
API = ROOT / 'tools/ros_station_api.py'


def old_work():
    source = subprocess.check_output(['git', '-C', str(ROOT), 'show',
                                      BASELINE + ':tools/ros_station_api.py'], text=True)
    main = next(node for node in ast.parse(source).body
                if isinstance(node, ast.FunctionDef) and node.name == 'main')
    simple = next(node for node in main.body
                  if isinstance(node, ast.FunctionDef) and node.name == 'simple_ev3')
    work = next(node for node in simple.body
                if isinstance(node, ast.FunctionDef) and node.name == 'work')
    namespace = {'operation': 'home_touch',
                 'time': SimpleNamespace(monotonic=lambda: 0., sleep=lambda _: None)}
    exec(compile(ast.Module(body=[work], type_ignores=[]), '<frozen simple_ev3.work>', 'exec'), namespace)
    return namespace['work'], hashlib.sha256(source.encode()).hexdigest()


def snapshot(busy):
    job = {'id': 7, 'state': 'running' if busy else 'complete', 'operation': 'home_touch'}
    return {'session': 'benchmark', 'busy': busy, 'job': job if busy else None,
            'last_job': None if busy else job, 'last_error': None}


class Proxy:
    session = 'benchmark'

    def __init__(self):
        self.polls = 0

    def call(self, operation):
        if operation == 'wake':
            return {'power_mode': 'awake'}
        if operation == 'home_touch':
            return {'accepted': True, 'job': {'id': 7, 'state': 'running'}}
        if operation == 'status':
            self.polls += 1
            return RUNNING if self.polls <= 100 else COMPLETE
        raise AssertionError('unexpected operation: ' + operation)


RUNNING, COMPLETE = snapshot(True), snapshot(False)


def benchmark(iterations=1000, batches=7):
    previous, previous_sha = old_work()
    cancel = threading.Event()

    def baseline_complete():
        proxy = Proxy()
        assert previous(None, proxy, cancel) is COMPLETE
        assert proxy.polls == 101

    def current_complete():
        proxy = Proxy()
        events = []
        assert run_ev3_operation(proxy, cancel, 'home_touch', events.append,
                                 {'request_id': 'benchmark-stop'},
                                 clock=lambda: 0., sleep=lambda _: None) is COMPLETE
        assert proxy.polls == 101 and len(events) == 2

    def old_classify(status):
        # Exact former session/terminal predicate, without RPC or polling sleep.
        if status.get('session') != 'benchmark':
            raise RuntimeError('EV3 session changed')
        if status.get('busy') is False:
            last = status.get('last_job') or {}
            if last.get('id') != 7 or last.get('state') != 'complete' or last.get('cleanup_fault') or status.get('last_error'):
                raise RuntimeError('EV3 job failed: ' + str(status))
            return True
        return False

    operations = {'baseline_running_classifier': lambda: old_classify(RUNNING),
                  'current_running_classifier': lambda: _terminal(RUNNING, 'benchmark', 7),
                  'baseline_complete_classifier': lambda: old_classify(COMPLETE),
                  'current_complete_classifier': lambda: _terminal(COMPLETE, 'benchmark', 7),
                  'baseline_100_polls_and_completion': baseline_complete,
                  'current_100_polls_and_completion_two_events': current_complete}
    samples = {name: [] for name in operations}
    for operation in operations.values():
        for _ in range(5):
            operation()
    for batch in range(batches):
        order = list(operations)
        if batch % 2:
            order.reverse()
        for name in order:
            samples[name].append(timeit.timeit(operations[name], number=iterations,
                                               timer=time.process_time) / iterations * 1e6)
    events = []
    run_ev3_operation(Proxy(), cancel, 'home_touch', events.append, {'request_id': 'benchmark-stop'},
                      clock=lambda: 0., sleep=lambda _: None)
    return {'baseline_revision': BASELINE, 'baseline_api_sha256': previous_sha,
            'current_source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                      for path in (HELPER, API)},
            'host': platform.node(), 'machine': platform.machine(), 'python': platform.python_version(),
            'timer': 'time.process_time', 'iterations_per_batch': iterations, 'batches': batches,
            'results': {name: {'median_batch_us': statistics.median(values), 'max_batch_us': max(values)}
                        for name, values in samples.items()},
            'evidence_events_per_100_running_polls': len(events),
            'evidence_event_json_bytes': [len(json.dumps(event).encode()) for event in events],
            'scope': 'Actual portable helper versus frozen former production work; 100 running polls plus one completion; two cold evidence events',
            'limitations': 'Synthetic CPU fixtures; no ROS initialization, transport, native device processing, filesystem writes, real sleeps or hardware. Includes local evidence construction, excludes persistence I/O; no physical latency claim or timing threshold'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=1000)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('--iterations must be positive')
    report = json.dumps(benchmark(args.iterations), indent=2) + '\n'
    if args.output:
        args.output.write_text(report)
    print(report, end='')
