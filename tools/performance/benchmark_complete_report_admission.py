#!/usr/bin/env python3
"""Retained-report resource probe. No analyzer, device, ROS node or DDS transport.

Run through ros2/conda/so101-conda.sh with native ROS Python and isolated DDS
settings. Reports must be the private frozen CPU oracle; no capacity admission
or native-source/physical qualification is established by this probe.
"""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import sys
import time
import tracemalloc
from types import MethodType, SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'ros2/src/so101_station_objects/so101_station_objects/objects.py'
HISTORICAL_SOURCE = '816acf8f348e8cb12d49f25611b8b2980cb89a83dbd87f5a5aff2499202c4d1d'
PRE_FIX_NATIVE_SOURCE = '6eec975e9b06892789cc8ff7daa744766a818b801f5697d398716b27b3186b27'
FROZEN_SOURCE = '08d6d09b8768539a063bf26f30d9e4120110ee457a458af1899fd619a397b321'
TRACKER_SOURCE = '691a85ecbb8cbc2cfbbbd44cc77a10efe6739da6c29db582fb46dc6e9914cb0e'


def stable_ast_digest(node):
    def canonical(value):
        if isinstance(value, ast.AST):
            fields = list(ast.iter_fields(value))
            if isinstance(value, (ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)) and not any(key == 'type_params' for key, _ in fields):
                fields.append(('type_params', []))
            return [type(value).__name__, [[key, canonical(item)] for key, item in fields]]
        if isinstance(value, list):
            return [canonical(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(canonical(node), separators=(',', ':')).encode()).hexdigest()


def resource_guard(source):
    """Conservative source guard: review required on tracker or hot-path drift."""
    tree = ast.parse(source)
    tracker = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StableObjectTracker')
    assert hashlib.sha256(ast.get_source_segment(source, tracker).encode()).hexdigest() == TRACKER_SOURCE, 'tracker aliasing assumption changed'
    methods = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    sink, publisher = methods['_complete_report'], methods['_publish_report']
    forbidden = {'deepcopy', 'analyze_rgbd', '_analyze_job', 'wait', 'synchronize', 'readback', 'download', 'cpu', 'numpy', 'asnumpy', 'get_data', 'to_host'}
    for method in (sink, publisher):
        for node in ast.walk(method):
            if isinstance(node, ast.Call):
                name = node.func.id if isinstance(node.func, ast.Name) else getattr(node.func, 'attr', '')
                assert name not in forbidden, f'forbidden hot-path operation: {name}'
            if isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Constant):
                assert node.slice.value not in {'rgb', 'depth', 'mask', 'points', 'contours'}, 'dense frame traversal'
    text = ast.unparse(sink)
    assert 'staged_tracker = copy.copy(self.tracker)' in text
    assert 'report = dict(report)' in text
    commit = next(n for n in sink.body if isinstance(n, ast.FunctionDef) and n.name == 'commit')
    assert [ast.unparse(n) for n in commit.body] == [
        'self.tracker = staged_tracker', 'self.last_support_plane = staged_cache',
        'self.last_analysis_result_stamp = stamp']
    allowed = {id(n) for n in ast.walk(commit)}
    for n in ast.walk(sink):
        if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store) and isinstance(n.value, ast.Name) and n.value.id == 'self' and n.attr in {'tracker', 'last_support_plane', 'last_analysis_result_stamp'}:
            assert id(n) in allowed, 'accepted state written outside private commit'
    # Task3 moves the unchanged commit/send block into a synchronous fenced
    # action. Prove that exact envelope, then apply the original Task1 checks.
    start = next(i for i, n in enumerate(publisher.body) if isinstance(n, ast.Assign)
                 and ast.unparse(n).startswith('native_mode ='))
    end = next(i for i, n in enumerate(publisher.body[start:], start)
               if isinstance(n, ast.Expr) and ast.unparse(n).startswith('TimingRecorder.clock(timing, 13)'))
    envelope = publisher.body[start:end]
    action = next(n for n in envelope if isinstance(n, ast.FunctionDef) and n.name == 'publish_action')
    sends = action.body[3:]
    action.body = action.body[:3]
    assert stable_ast_digest(ast.Module(body=envelope, type_ignores=[])) == 'a0c6393c396af4e7148944bae79251a9cb55d34c954de2dbc2b8866d7744b312', 'native fence envelope changed'
    publisher.body[start:end] = sends
    final = next(i for i, n in enumerate(publisher.body) if isinstance(n, ast.If) and ast.unparse(n.test) == 'derived_token is not None')
    assert ast.unparse(publisher.body[final + 1]) == 'if commit is not None:\n    commit()'
    assert ast.unparse(publisher.body[final + 2]).startswith('self.detections_pub.publish(')
    assert 'return False' in ast.unparse(publisher.body[final])
    return True


def decode(value):
    import numpy as np
    if isinstance(value, list):
        return [decode(v) for v in value]
    if not isinstance(value, dict):
        return value
    if value.get('__ndarray__') is True:
        return np.asarray(value['data'], dtype=value['dtype']).reshape(value['shape'])
    if '__numpy_scalar__' in value:
        return np.dtype(value['__numpy_scalar__']).type(value['data'])
    if value.get('__type__') == 'builtins.set' and value['repr'] == 'set()':
        return set()
    return {k: decode(v) for k, v in value.items()}


def load_methods(source):
    """Execute unchanged method AST with real ROS types, without node construction."""
    sys.path[:0] = [str(ROOT / 'ros2/src/so101_station_objects'), str(ROOT / 'ros2/src/so101_station_perception')]
    from so101_station_objects import objects
    from geometry_msgs.msg import Pose, PoseArray
    from std_msgs.msg import String
    from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
    from visualization_msgs.msg import Marker, MarkerArray
    from rclpy.time import Time
    from tf2_ros import TransformException
    namespace = dict(vars(objects))
    namespace.update(locals())
    names = {'_complete_report', '_publish_report'}
    selected = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return objects, {name: namespace[name] for name in names}


def summary(samples):
    values = sorted(samples)
    return {'count': len(values), 'min_ns': min(values), 'median_ns': statistics.median(values),
            'p95_ns': values[min(len(values)-1, int(.95 * len(values)))], 'max_ns': max(values)}


def run(args):
    assert os.environ.get('ROS_DOMAIN_ID') == '177' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    source = SOURCE.read_text()
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == FROZEN_SOURCE
    resource_guard(source)
    objects, methods = load_methods(source)
    import rclpy
    import numpy as np
    hashes = json.loads((args.oracle / 'artifact_sha256.json').read_text())
    def retained(name):
        path = args.oracle / (name + '.json')
        assert hashlib.sha256(path.read_bytes()).hexdigest() == hashes[path.name]
        return decode(json.loads(path.read_text()))
    typical = retained('complete_3_analyzer_report')
    empty = retained('rejected_empty_analyzer_report')
    before = retained('complete_3_before')
    assert len(typical['objects']) == 1 and len(before['tracker']['tracks']) == 1
    assert not empty['objects']
    result = {'source_sha256': FROZEN_SOURCE, 'oracle_manifest_sha256': hashlib.sha256((args.oracle / 'artifact_sha256.json').read_bytes()).hexdigest(),
              'runtime': {'python': sys.executable, 'version': sys.version, 'platform': platform.platform(),
                          'domain': os.environ['ROS_DOMAIN_ID'], 'localhost': os.environ['ROS_LOCALHOST_ONLY'],
                          'numpy': np.__version__, 'rclpy_module': rclpy.__file__,
                          'objects_module': objects.__file__},
              'scope': 'precomputed retained CPU reports; actual common sink and original publisher method, real ROS messages; capture sinks, no DDS/CDR serialization; synthetic clock/registration; no producer throughput, capacity or physical claim',
              'fixture_max_objects': 1, 'fixture_max_tracks': 1, 'cases': {}}
    class Capture:
        def __init__(self):
            self.count = 0
            self.types = set()
        def publish(self, message):
            self.count += 1
            self.types.add(type(message).__module__ + '.' + type(message).__name__)
    def node(report):
        n = SimpleNamespace(tracker=objects.StableObjectTracker(), last_support_plane=None,
                            last_analysis_result_stamp=None, require_registration_tracking=True,
                            registration_last_tracking_stamp=0., board_frame='charuco_board', now=0.,
                            last_marker_update_mono=None, last_marker_update_stamp=None, marker_clear_sent=True)
        if report['objects']:
            n.tracker.__dict__.update(copy.deepcopy(before['tracker']))
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(n.now * 1e9)))
        n.rejected = 0
        def reject(*a, **kw):
            n.rejected += 1
        n._publish_unavailable = reject
        n.capture = Capture()
        for name in ('detections', 'markers', 'grasps', 'contacts', 'status'):
            setattr(n, name + '_pub', n.capture)
        for name, method in methods.items():
            setattr(n, name, MethodType(method, n))
        return n
    start_stamp = max(float(before['tracker']['last_acquisition_stamp']) + 1., time.time())
    from builtin_interfaces.msg import Time as Stamp
    for label, report in [('empty', empty), ('typical_equals_frozen_max_1', typical)]:
        n = node(report)
        # The actual new shallow staging operations, isolated from tracker/materialization.
        staging = []
        for _ in range(args.samples):
            begin = time.perf_counter_ns()
            staged = copy.copy(n.tracker)
            mapping = dict(report)
            staging.append(time.perf_counter_ns() - begin)
        rows = []
        def complete(stamp, *, stale=False, registration=False):
            n.now = stamp + (.6 if stale else .001)
            n.registration_last_tracking_stamp = stamp - (.6 if registration else 0.)
            seconds = int(stamp)
            job = {'stamp': stamp, 'ros_stamp': Stamp(sec=seconds, nanosec=int((stamp-seconds)*1e9))}
            count = n.capture.count
            wall, cpu = time.perf_counter_ns(), time.process_time_ns()
            n._complete_report(job, report)
            row = {'stamp': stamp, 'wall_ns': time.perf_counter_ns()-wall,
                   'process_cpu_ns': time.process_time_ns()-cpu, 'completed': n.capture.count-count == 5,
                   'stale_injected': stale, 'registration_expiry_injected': registration}
            return row
        for index in range(args.samples):
            rows.append(complete(start_stamp + index / 30.))
        # Allocation tracing runs separately; never mix these samples into latency.
        tracemalloc.start()
        tracemalloc.reset_peak()
        allocation_start = tracemalloc.get_traced_memory()[0]
        for index in range(20):
            complete(start_stamp + (args.samples+index) / 30.)
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        paced = {}
        for hz in (20, 30):
            offered = []
            epoch = time.perf_counter()
            base = start_stamp + args.samples + hz
            for index in range(round(args.duration * hz)):
                deadline = epoch + index / hz
                delay = deadline-time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                row = complete(base+index/hz, stale=index == 3, registration=index == 7)
                row['arrival_lateness_ns'] = int(max(0., time.perf_counter()-deadline)*1e9)-row['wall_ns']
                offered.append(row)
            assert sum(r['completed'] for r in offered) == len(offered)-2
            assert all(r['completed'] == (not r['stale_injected'] and not r['registration_expiry_injected']) for r in offered)
            paced[str(hz)] = {'offered': len(offered), 'completed': sum(r['completed'] for r in offered),
                              'rejected': sum(not r['completed'] for r in offered),
                              'elapsed_s': time.perf_counter()-epoch, 'samples': offered}
        result['cases'][label] = {'staging_wall_ns': staging, 'staging_summary': summary(staging),
                                 'samples': rows, 'wall_summary': summary([r['wall_ns'] for r in rows]),
                                 'cpu_summary': summary([r['process_cpu_ns'] for r in rows]),
                                 'allocation_20_calls': {'scope': 'includes completion and probe row/clock overhead; traced Python allocations only, not ROS C/native or RSS', 'net_traced_bytes': current-allocation_start, 'peak_increment_bytes': peak-allocation_start},
                                 'paced': paced, 'message_types': sorted(n.capture.types)}
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == FROZEN_SOURCE
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: {key: value[key] for key in ('wall_summary', 'cpu_summary', 'staging_summary', 'allocation_20_calls')} for k, value in result['cases'].items()}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--oracle', type=Path, default=Path('/tmp/complete-report-publication-baseline-20261027'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=300)
    parser.add_argument('--duration', type=float, default=2.)
    run(parser.parse_args())
