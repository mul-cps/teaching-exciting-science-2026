"""Offline per-program admission benchmark; never starts a program or ROS.

Run: python tests/blockly_preflight_benchmark.py [--iterations 100] [--output PATH]
Measures the existing validator, local recording check, and their combined cost.
Batch timings are evidence, not machine-dependent test thresholds.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import sys
import timeit

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.run_blockly_station import Application
from workshop.blockly_program import validate_program


def nested_program(names, depth=4):
    """Four binary branches leave 96 commands per leaf within the 100-step bound."""
    cursor = iter(names)

    def branch(level):
        if level == 0:
            return [{'op': 'replay', 'name': next(cursor), 'speed': .5} for _ in range(96)]
        return [{'op': 'if_color', 'color': 'red',
                 'then': branch(level-1), 'else': branch(level-1)}]

    return branch(depth)


def command_count(rows):
    return sum(1 + (command_count(row['then']) + command_count(row['else'])
                    if row['op'] == 'if_color' else 0) for row in rows)


def benchmark(iterations=100, batches=7):
    recordings = {'s'*36+f'{i:04d}': object() for i in range(64)}
    names = list(recordings)
    application = Application.__new__(Application)
    application.recordings = recordings
    cases = {
        'flat_100_known': ([{'op': 'replay', 'name': names[i % 64], 'speed': .5}
                            for i in range(100)], False),
        'nested_1551_known': (nested_program([names[i % 64] for i in range(1536)]), False),
        'nested_1551_missing': (nested_program(['m'*36+f'{i:04d}' for i in range(1536)]), True),
    }
    results = {}
    for label, (program, missing) in cases.items():
        validate_program(program)

        def preflight():
            try:
                application._check_recordings(program)
            except ValueError:
                if not missing:
                    raise
            else:
                if missing:
                    raise AssertionError('missing recordings were admitted')

        def combined():
            validate_program(program)
            preflight()

        operations = {'baseline_validation': lambda: validate_program(program),
                      'preflight_only': preflight, 'validation_and_preflight': combined}
        for operation in operations.values():
            for _ in range(10):
                operation()
        samples = {name: [] for name in operations}
        for batch in range(batches):
            order = list(operations)
            if batch % 2:
                order.reverse()
            for name in order:
                elapsed = timeit.timeit(operations[name], number=iterations)
                samples[name].append(elapsed / iterations * 1e6)
        encoded = json.dumps(program, sort_keys=True).encode()
        assert len(encoded) <= 262144
        timings = {name: {'median_batch_us': statistics.median(values),
                          'max_batch_us': max(values)} for name, values in samples.items()}
        results[label] = {'syntactic_commands': command_count(program),
                          'max_expanded_commands': 100, 'recording_library_names': len(recordings),
                          'recording_name_characters': 40,
                          'request_program_bytes': len(encoded),
                          'program_sha256': hashlib.sha256(encoded).hexdigest(), **timings,
                          'added_median_us': (timings['validation_and_preflight']['median_batch_us']
                                              - timings['baseline_validation']['median_batch_us'])}
    return {'host': platform.node(), 'machine': platform.machine(),
            'processor': platform.processor(), 'logical_cpus': os.cpu_count(),
            'python': platform.python_version(), 'executable': sys.executable,
            'source_sha256': hashlib.sha256((ROOT/'tools/run_blockly_station.py').read_bytes()).hexdigest(),
            'iterations_per_batch': iterations, 'batches': batches, 'results': results,
            'scope': 'Local per-program validation only; no Application constructor/run, thread, backend, ROS or device calls',
            'limitation': 'Batch mean CPU timings; combined-minus-baseline includes timing noise; excludes dispatch and physical execution; no pass/fail timing threshold'}


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
