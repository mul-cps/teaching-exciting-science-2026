"""Offline intake comparison; synthetic metadata, no ROS initialization or pixels."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import platform
import runpy
import statistics
import struct
import subprocess
import sys
from threading import RLock
import time
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'ros2/src/so101_station_perception/so101_station_perception/derived_aligned.py'
BASE = '053635c0a8ad818654535a1b9b4e8438e9a36925'
sys.path.insert(0, str(ROOT/'tests'))
INPUTS = runpy.run_path(str(ROOT/'tests/test_aligned_cohort.py'))


def adapter_class(source):
    from so101_station_perception.derived_lineage import canonical_gid, finite, require
    original = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef))
    tree = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
        name='Adapter', bases=[], keywords=[], body=original.body, decorator_list=[])], type_ignores=[]))
    scope = dict(time=NS(monotonic=lambda: 100.12), struct=struct, json=json,
                 canonical_gid=canonical_gid, finite=finite, require=require)
    exec(compile(tree, str(SOURCE), 'exec'), scope)
    return scope['Adapter']


def adapter_fixture(cls, case='valid_info'):
    core, value = INPUTS['gate'](); items = INPUTS['feed'](core, value)
    core.emitted = value['sequence']
    obj = object.__new__(cls); obj.core = core; obj.lock = RLock(); obj.closed = False; obj.fault = None
    obj.period = .2; obj.last_take = 100.; obj.last_proof = object()
    obj.consumer_epoch = 'fixture-epoch'; obj.source_pin_sha256 = 'a'*64
    obj._early_info_drops = {'depth_info': 0, 'color_info': 0}
    obj._early_info_last_age_ns = {'depth_info': None, 'color_info': None}
    obj.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_120_000_000)))
    offered = []; obj.metadata_offer = offered.append
    obj.callback = lambda *args: (_ for _ in ()).throw(AssertionError('image callback'))
    kind = 'color' if case == 'future_image' else 'depth_info'
    message = INPUTS['messages'](value)[kind]
    gid = bytes.fromhex(value['source'][kind+'_publisher_gid'])
    if case != 'valid_info': message.header.stamp = NS(sec=10, nanosec=125_000_000)
    if case == 'valid_new_info': message.header.stamp = NS(sec=10, nanosec=110_000_000)
    if case == 'stale_info': message.header.stamp = NS(sec=9, nanosec=0)
    if case == 'wrong_gid': gid = b'\xff'*16
    if case == 'legacy_future_info': obj.metadata_offer = None
    return obj, kind, message, gid, offered


def snapshot(obj):
    return (dict(obj.core.last), obj.core.emitted, tuple(obj.core.ledger.entries),
            {kind: tuple(rows) for kind, rows in obj.core.streams.items()},
            obj.last_proof, obj.fault)


def benchmark(iterations=100, batches=7):
    sources = {'baseline': subprocess.check_output(
        ['git', 'show', BASE+':'+str(SOURCE.relative_to(ROOT))], cwd=ROOT, text=True),
               'candidate': SOURCE.read_text()}
    classes = {name: adapter_class(source) for name, source in sources.items()}
    results = {}
    for case in ('valid_info', 'valid_new_info', 'future_info', 'wrong_gid', 'stale_info',
                 'future_image', 'legacy_future_info'):
        samples = {name: [] for name in classes}
        for batch in range(batches):
            for name in (list(classes) if batch % 2 == 0 else list(reversed(classes))):
                # Restore all four real core buffers before each measured call.
                fixtures = [adapter_fixture(classes[name], case) for _ in range(iterations)]
                started = time.perf_counter_ns()
                for obj, kind, message, gid, _ in fixtures:
                    obj._intake(kind, message, gid)
                samples[name].append((time.perf_counter_ns()-started)/iterations/1000.)
                for obj, _, _, _, offered in fixtures:
                    if case in ('valid_info', 'valid_new_info') or (case == 'future_info' and name == 'candidate'):
                        assert len(obj.core.ledger.entries) == 1
                        assert all(len(rows) == (2 if case == 'valid_new_info' and kind == 'depth_info' else 1)
                                   for kind, rows in obj.core.streams.items())
                        assert not offered and obj.fault is None
                        assert all(record[2] == 100.12 for rows in obj.core.streams.values() for record in rows)
                    else:
                        assert obj.fault and not obj.core.ledger.entries and not any(obj.core.streams.values())
        timings = {name: {'median_batch_us': statistics.median(values),
                          'max_batch_us': max(values)} for name, values in samples.items()}
        results[case] = {**timings, 'delta_median_us': timings['candidate']['median_batch_us']-timings['baseline']['median_batch_us']}
    return dict(host=platform.node(), python=platform.python_version(), executable=sys.executable,
                baseline_commit=BASE, candidate_sha256=hashlib.sha256(sources['candidate'].encode()).hexdigest(),
                iterations_per_batch=iterations, batches=batches, results=results,
                scope='Actual AST-extracted adapter methods and real AlignedCohorts; reference-only images; no ROS constructor, TF, device or live calls',
                limitation='CPU batch means on prepared buffers; not callback scheduling or end-to-end performance')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=100)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.iterations < 1: parser.error('--iterations must be positive')
    report = json.dumps(benchmark(args.iterations), indent=2)+'\n'
    if args.output: args.output.write_text(report)
    print(report, end='')
