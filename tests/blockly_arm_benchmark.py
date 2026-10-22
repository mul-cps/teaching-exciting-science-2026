"""CPU-only Blockly admission and cached recording benchmark; no ROS initialization.

Run: python tests/blockly_arm_benchmark.py --iterations 100 --output PATH
The frozen baseline is loaded with git show; no checkout, device, RPC or sleep.
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
import types

ROOT = Path(__file__).resolve().parents[1]
BASELINE = 'b5d2a06efe2cff688f00f8985c303fb94ff9f78f'
PATHS = ('workshop/blockly_program.py', 'workshop/blockly_recording.py',
         'workshop/blockly_ros.py', 'tools/run_blockly_station.py')


def sources(revision=None):
    return {path: (subprocess.check_output(['git', '-C', str(ROOT), 'show',
                                          revision + ':' + path], text=True)
                   if revision else (ROOT / path).read_text()) for path in PATHS}


def load_modules(source, label):
    """Import only portable module definitions; ROS imports stay inside __init__."""
    package = types.ModuleType(label)
    package.__path__ = []
    sys.modules[label] = package
    result = {}
    for short in ('blockly_program', 'blockly_recording', 'blockly_ros'):
        module = types.ModuleType(label + '.' + short)
        module.__package__ = label
        sys.modules[module.__name__] = module
        exec(compile(source['workshop/' + short + '.py'], short, 'exec'), module.__dict__)
        result[short] = module
    tree = ast.parse(source['tools/run_blockly_station.py'])
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Application')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in {'_check_recordings', '_preflight_program'}]
    namespace = {}
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<admission>', 'exec'), namespace)
    result['admission'] = namespace
    return result


def cached_recording_operation(modules):
    """100 consecutive logical 10 Hz cached samples, excluding waits and finish/save."""
    ros = modules['blockly_ros']
    # A module-local clock keeps fresh caches deterministic without touching time globally.
    ros.time = types.SimpleNamespace(monotonic=lambda: 1000000.)
    backend = ros.ROSBackend.__new__(ros.ROSBackend)
    backend.lock = threading.Lock()
    backend.latest = {'positions': [0.] * 5, 'jaw': 0., 'stamp': 100., 'received': 1000000.}
    backend.owner_cache = (1000000., {
        'schema': 1, 'kind': 'follower_status', 'sampled_monotonic': 1000000.,
        'fresh': True, 'position_age_s': 0., 'torque_age_s': 0.,
        'owner_source_sha256': {'owner': 'b' * 64}, 'owner_started_monotonic': 123.,
        'arm_id': 'benchmark', 'calibration_sha256': 'a' * 64})
    identity = backend.sample()['identity']
    recorder_type = modules['blockly_recording'].Recorder

    def process():
        recorder = recorder_type('bench', identity, clock=lambda: 0.)
        for index in range(100):
            backend.latest['stamp'] = 100. + index * .1
            sample = backend.sample()
            if sample['identity'] != identity:
                raise AssertionError('identity changed')
            recorder.add(sample['positions'], sample['jaw'], sample['stamp'])
        assert len(recorder.rows) == 100
    return process


def benchmark(iterations=100, batches=7):
    source_sets = {'baseline': sources(BASELINE), 'current': sources()}
    modules = {label: load_modules(source, '_arm_bench_' + label)
               for label, source in source_sets.items()}
    recordings = {'stroke' + str(i): object() for i in range(64)}
    program = [{'op': 'replay', 'name': 'stroke' + str(i % 64), 'speed': .5}
               for i in range(100)]
    operations = {}
    for label, loaded in modules.items():
        application = types.SimpleNamespace(recordings=recordings, directory=ROOT,
                                            backend=types.SimpleNamespace(physical=False), instructor=False)
        admission = loaded['admission']

        def preflight(loaded=loaded, application=application, admission=admission):
            loaded['blockly_program'].validate_program(program)
            if '_preflight_program' in admission:
                admission['_preflight_program'](application, program)
            admission['_check_recordings'](application, program)

        operations[label + '_preflight_100'] = preflight
        operations[label + '_cached_samples_100'] = cached_recording_operation(loaded)
    # New ARM operation has no baseline equivalent; report it without inventing a delta.
    arm_program = [{'op': 'arm', 'command': 'teleop_relative'} for _ in range(100)]
    current = modules['current']
    app = types.SimpleNamespace(recordings=recordings, directory=ROOT,
                                backend=types.SimpleNamespace(physical=True), instructor=True)

    def arm_preflight():
        current['blockly_program'].validate_program(arm_program)
        current['admission']['_preflight_program'](app, arm_program)
        current['admission']['_check_recordings'](app, arm_program)
    operations['current_arm_preflight_100'] = arm_preflight
    samples = {name: [] for name in operations}
    for operation in operations.values():
        for _ in range(5):
            operation()
    for batch in range(batches):
        order = list(operations)
        if batch % 2:
            order.reverse()
        for name in order:
            elapsed = timeit.timeit(operations[name], number=iterations, timer=time.process_time)
            samples[name].append(elapsed / iterations * 1e6)
    timings = {name: {'median_batch_us': statistics.median(values),
                      'max_batch_us': max(values), 'items_per_operation': 100,
                      'median_us_per_item': statistics.median(values) / 100}
               for name, values in samples.items()}
    return {'baseline_revision': BASELINE, 'python': platform.python_version(),
            'host': platform.node(), 'machine': platform.machine(),
            'timer': 'time.process_time', 'iterations_per_batch': iterations, 'batches': batches,
            'source_sha256': {label: {path: hashlib.sha256(source.encode()).hexdigest()
                                    for path, source in files.items()}
                              for label, files in source_sets.items()},
            'results': timings,
            'scope': '100-command local validation/admission; 100 cached ROSBackend.sample + identity comparison + Recorder.add operations',
            'limitations': 'CPU-only synthetic fixtures; excludes Application construction/run, ROS initialization, RPCs, recording waits, finish/save, callbacks and hardware; no physical latency or timing pass threshold'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=100)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.iterations < 1:
        parser.error('--iterations must be positive')
    report = json.dumps(benchmark(args.iterations), indent=2) + '\n'
    if args.output:
        args.output.write_text(report)
    print(report, end='')
