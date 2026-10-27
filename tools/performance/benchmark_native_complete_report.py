#!/usr/bin/env python3
"""Synthetic compact ingress + actual sink/ROS message resource probe; no Node/DDS."""
from __future__ import annotations
import argparse
import ast
import copy
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import time
import tracemalloc
from threading import RLock, get_ident
from types import MethodType, SimpleNamespace as NS

from benchmark_complete_report_admission import (
    ROOT, SOURCE, FROZEN_SOURCE, decode, load_methods, resource_guard, summary,
    stable_ast_digest)

INGRESS = SOURCE.with_name('native_complete_report.py')
INGRESS_SHA = 'f566dcb87f3b3849a3e684c923d6dfc9bfc81e47ebabde954a2f622fa10bcd8c'
BASELINE = ROOT / 'tests/fixtures/complete_report_admission/cpu_baseline.json'
NATIVE_GRASP_BOUNDARY = '9b5d925bf5525805385365b1f8a8363fed8711ec808c8a67ebbab0abaaa797fe'
BASELINE_SHA = '1a95eccf560bc89c2d23a6d316c1c8e1592e0d952ac0ed1c68369f9d2d812e5a'
# Full approved ASTs pin lock boundaries, bounded traversal and notification,
# rather than attempting to recognize every spelling of forbidden operations.
NATIVE_GLUE = {'offer_native_complete': '99a5485178e1087d619fa7ecc9a6b7001bf3402f8f06974f417ba615560157cc', '_native_current': '39e98fd2eca587c316fdf09b234db9cbfa45f43e345682c14a86dd4ba616dbee', '_consume_native_complete': '036cb9aa271fcd0831f098f17524606b37133a3ad4119f634ac6708ee4cefcdc'}


def combined_guard(source, ingress):
    resource_guard(source)
    methods = {n.name: n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef)}
    forbidden = {'deepcopy', 'analyze_rgbd', '_analyze_job', 'wait', 'sleep',
                 'synchronize', 'readback', 'download', 'cpu', 'numpy', 'asnumpy',
                 'get_data', 'to_host', 'asarray', 'array', 'tolist'}
    for name, digest in NATIVE_GLUE.items():
        method = methods[name]
        for call in (n for n in ast.walk(method) if isinstance(n, ast.Call)):
            called = call.func.id if isinstance(call.func, ast.Name) else getattr(call.func, 'attr', '')
            assert called not in forbidden, f'forbidden native glue call: {called}'
        assert stable_ast_digest(method) == digest, f'native glue changed: {name}'
    constructors = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_guard_condition'
                    and n.args and ast.unparse(n.args[0]) == 'self._consume_native_complete']
    assert len(constructors) == 1 and not constructors[0].keywords, 'native completion must use default serialized group'
    ingress_calls = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Name) and n.func.id == 'NativeCompleteIngress']
    assert len(ingress_calls) == 1 and [ast.unparse(arg) for arg in ingress_calls[0].args] == ['self._native_provider', 'self.native_ready_guard.trigger'], 'notify must only trigger guard'
    publisher = methods['_publish_report']
    grasp_loop = next(n for n in ast.walk(publisher) if isinstance(n, ast.For)
                      and ast.unparse(n.target) == '(grasp_index, grasp)')
    # Exactly native-only shallow local record + two [:3] copies, followed by
    # the historical helper call. No helper/CPU behavior or admitted ownership change.
    assert stable_ast_digest(ast.Module(body=grasp_loop.body[:2], type_ignores=[])) == NATIVE_GRASP_BOUNDARY, 'native grasp copy boundary changed'
    # Exact approved ingress source includes pre-conversion key/shape checks,
    # single _pending slot, and provider work outside the short slot lock.
    assert hashlib.sha256(ingress.encode()).hexdigest() == INGRESS_SHA, 'approved ingress changed'
    return True


def projection(report):
    """Portable frozen CPU projection only; never a production report adapter."""
    value = copy.deepcopy(report)
    for key in ('inlier_mask', 'residuals_m'):
        value['support_plane'].pop(key, None)
    for obj in value['objects']:
        obj.pop('contour', None)
    return value


class Provider:
    """Fixture-only original registry, clock/epoch/capability and same-thread fence.

    Exactly one current original/handle is retained, keeping the benchmark's
    scaffolding bounded independently of the production ingress slot.
    """
    def __init__(self, identity, backend, limits):
        self.original = identity
        self.backend_status = backend
        self.frame_ids = ('color', 'depth')
        self.limits = limits
        self.handle = None
        self.last_capture = None
        self.lock = RLock()
        self.revoked = False
        self.fence_ns = []
        self.action_ns = []
        self.measure = True

    def admit(self, identity, report):
        assert not self.revoked and identity is self.original
        assert report['color_mask_backend'] == self.backend_status
        capture = (identity.session_token, identity.color_frame_number, identity.depth_frame_number)
        assert capture != self.last_capture
        self.last_capture = capture
        self.handle = object()
        return self.handle

    def check(self, handle):
        with self.lock:
            assert not self.revoked and handle is self.handle

    def publish_if_current(self, handle, action):
        begin = time.perf_counter_ns()
        with self.lock:
            self.check(handle)
            action_begin = time.perf_counter_ns()
            owner = get_ident()
            action()
            assert owner == get_ident()
            end = time.perf_counter_ns()
        if self.measure:
            self.action_ns.append(end-action_begin)
            self.fence_ns.append(end-begin)
        return True


def run(args):
    assert os.environ.get('ROS_DOMAIN_ID') == '177' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    source = SOURCE.read_text()
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == FROZEN_SOURCE
    assert hashlib.sha256(BASELINE.read_bytes()).hexdigest() == BASELINE_SHA
    combined_guard(source, INGRESS.read_text())
    objects, methods = load_methods(source)
    from rclpy.time import Time
    from builtin_interfaces.msg import Time as Stamp
    from so101_station_objects.native_complete_report import NativeIdentity, NativeCompleteReport, NativeCompleteIngress, ReportLimits
    import numpy as np
    import rclpy
    namespace = dict(vars(objects), Time=Time)
    selected = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name in NATIVE_GLUE]
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    fixture = json.loads(BASELINE.read_text())
    results = {'scope': 'actual source compact ingress + common sink + real ROS five-message construction; local capture publishers; no Node/context/DDS/executor/GPU',
               'unqualified': ['GuardCondition.trigger', 'executor scheduling', 'DDS/CDR', 'native/RSS allocations', 'production provider', 'exposure clock authority', 'whole pipeline performance'],
               'capture_overhead': 'timed local publishers include output cardinality/header assertions and status JSON decode validation; this is conservative message materialization/capture time, not DDS latency',
               'fixture_limits': {'max_objects': 1, 'max_tracks': 1, 'typical_equals_admitted_fixture_max': True},
               'runtime': {'python': sys.executable, 'version': sys.version, 'platform': platform.platform(), 'numpy': np.__version__, 'rclpy': rclpy.__file__, 'objects_module': objects.__file__, 'domain': os.environ['ROS_DOMAIN_ID'], 'localhost': os.environ['ROS_LOCALHOST_ONLY'], 'argv': sys.argv},
               'hashes': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in (SOURCE, INGRESS, BASELINE, Path(__file__), ROOT/'tools/performance/benchmark_complete_report_admission.py')}, 'cases': {}}
    for label, index in [('empty_0_objects_0_tracks', 4), ('typical_equals_fixture_max_1', 2)]:
        report = projection(decode(fixture['cases'][index]['analyzer_report']))
        identity = NativeIdentity(object(), 1, 0, 0, 100., 100., 1, 1, 'color', 'depth', object(), object(), object(), 10_000_000_000, object())
        p = Provider(identity, report['color_mask_backend'], ReportLimits(1, 1))
        notifications = [0]
        notification_ns = []
        def notify():
            begin = time.perf_counter_ns()
            notifications[0] += 1
            if p.measure:notification_ns.append(time.perf_counter_ns()-begin)
        ingress = NativeCompleteIngress(p, notify)
        n = NS(tracker=objects.StableObjectTracker(), last_support_plane=None, last_analysis_result_stamp=None,
               require_registration_tracking=True, registration_last_tracking_stamp=0., board_frame='charuco_board',
               _native_mode=True, _native_provider=p, native_ingress=ingress,
               last_marker_update_mono=None, last_marker_update_stamp=None, marker_clear_sent=True)
        if report['objects']:
            seed = decode(fixture['cases'][2]['accepted_state']['tracker'])
            # JSON stringifies dictionary keys; restore original integer IDs in
            # benchmark state only, never change fixture or production adapters.
            seed['tracks'] = {int(key): value for key, value in seed['tracks'].items()}
            n.tracker.__dict__.update(seed)
        n.now_ns = 0
        n.get_clock = lambda: NS(now=lambda: Time(nanoseconds=n.now_ns))
        rejected = [0]
        n._publish_unavailable = lambda *a, **kw: rejected.__setitem__(0, rejected[0]+1)
        sends = [0]
        types = set()
        message_counts = {name: 0 for name in ('detections', 'markers', 'grasps', 'contacts', 'status')}
        expected_grasps = sum(len(obj['grasps']) for obj in report['objects'])
        def publish(name, message):
            if hasattr(message, 'detections'):
                assert len(message.detections) == len(report['objects']), 'fixture population lost before materialization'
            if name in ('grasps', 'contacts'):
                assert len(message.poses) == expected_grasps, 'fixture grasp/contact population lost'
            if name in ('detections', 'grasps', 'contacts'):
                assert message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec == n.expected_stamp_ns, 'original acquisition stamp changed'
            if name == 'status':
                assert len(json.loads(message.data)['grasp_candidates']) == expected_grasps
            message_counts[name] += 1
            sends[0] += 1
            types.add(type(message).__module__+'.'+type(message).__name__)
        for name in ('detections', 'markers', 'grasps', 'contacts', 'status'):
            setattr(n, name+'_pub', NS(publish=lambda message, name=name: publish(name, message)))
        for name, method in methods.items():
            setattr(n, name, MethodType(method, n))
        for name in NATIVE_GLUE:
            setattr(n, name, MethodType(namespace[name], n))
        publisher_ns = []
        actual_publisher = n._publish_report
        def measured_publisher(*a, **kw):
            begin = time.perf_counter_ns()
            try:return actual_publisher(*a, **kw)
            finally:
                if p.measure:publisher_ns.append(time.perf_counter_ns()-begin)
        n._publish_report = measured_publisher
        serial = 0
        def offer(stamp_ns):
            nonlocal serial
            serial += 1
            p.original = replace(identity, color_frame_number=serial, depth_frame_number=serial, observation_ros_ns=stamp_ns)
            packet = NativeCompleteReport(p.original, report)
            wall, cpu = time.perf_counter_ns(), time.process_time_ns()
            ok = n.offer_native_complete(packet)
            return dict(wall_ns=time.perf_counter_ns()-wall, cpu_ns=time.process_time_ns()-cpu, accepted=ok, source_stamp_ns=stamp_ns, capture_id=serial)
        def consume(stamp_ns, stale=False):
            n.expected_stamp_ns = stamp_ns
            n.now_ns = stamp_ns+(600_000_000 if stale else 1_000_000)
            n.registration_last_tracking_stamp = stamp_ns*1e-9
            count = sends[0]
            wall, cpu = time.perf_counter_ns(), time.process_time_ns()
            n._consume_native_complete()
            assert len(n.tracker.tracks) == len(report['objects']), 'fixture track population lost'
            return dict(wall_ns=time.perf_counter_ns()-wall, cpu_ns=time.process_time_ns()-cpu, completed=sends[0]-count == 5, stale=stale)
        # Instrument actual take/provider validation separately without replacing it.
        take = ingress.take
        take_ns = []
        def measured_take():
            begin = time.perf_counter_ns()
            value = take()
            if p.measure:take_ns.append(time.perf_counter_ns()-begin)
            return value
        ingress.take = measured_take
        base_ns = round(float(fixture['cases'][2]['accepted_state']['tracker']['last_acquisition_stamp'])*1e9)+50_000_000
        offers, completions, roundtrips = [], [], []
        for i in range(args.samples):
            stamp_ns = base_ns+i*50_000_000
            wall, cpu = time.perf_counter_ns(), time.process_time_ns()
            offers.append(offer(stamp_ns)); completions.append(consume(stamp_ns))
            roundtrips.append({'wall_ns': time.perf_counter_ns()-wall, 'cpu_ns': time.process_time_ns()-cpu})
        assert all(row['accepted'] for row in offers) and all(row['completed'] for row in completions)
        paced = {}
        for hz in (20, 30):
            offered, consumed = [], []
            before_replace = ingress.replaced
            epoch = time.perf_counter()
            start_ns = round(n.last_analysis_result_stamp*1e9)+50_000_000
            for i in range(round(args.duration*hz)):
                deadline = epoch+i/hz
                delay = deadline-time.perf_counter()
                if delay > 0:time.sleep(delay)
                # Deterministic overload: three unique originals before executor
                # consumption, every fifth offer period. Single-slot newest wins.
                burst = 3 if i % 5 == 0 else 1
                for b in range(burst):
                    stamp_ns = start_ns+round(i*1e9/hz)+b
                    row = offer(stamp_ns)
                    row['arrival_lateness_ns'] = int(max(0., time.perf_counter()-deadline)*1e9)
                    offered.append(row)
                consumed.append(consume(stamp_ns, stale=i == 3))
            replaced = ingress.replaced-before_replace
            completed = sum(row['completed'] for row in consumed)
            assert all(row['accepted'] for row in offered)
            assert completed == len(consumed)-1
            assert replaced == len(offered)-len(consumed)
            paced[str(hz)] = {'offered': len(offered), 'accepted': len(offered), 'completed_unique': completed,
                              'replaced': replaced, 'completion_rejected_stale': 1, 'ingress_rejected': 0,
                              'elapsed_s': time.perf_counter()-epoch, 'offers': offered, 'completions': consumed}
        # Trace separately with timing sample collection disabled; fixed fixture
        # storage is prebuilt. Traced peak is not native allocation or RSS.
        p.measure = False
        tracemalloc.start();tracemalloc.reset_peak()
        initial = tracemalloc.get_traced_memory()[0]
        for i in range(20):
            stamp_ns = round(n.last_analysis_result_stamp*1e9)+50_000_000
            offer(stamp_ns);consume(stamp_ns)
        current, peak = tracemalloc.get_traced_memory();tracemalloc.stop()
        results['cases'][label] = {'offers': offers, 'completions': completions, 'roundtrip_samples': roundtrips,
            'roundtrip_wall': summary([row['wall_ns'] for row in roundtrips]), 'roundtrip_cpu': summary([row['cpu_ns'] for row in roundtrips]),
            'offer_wall': summary([row['wall_ns'] for row in offers]), 'offer_cpu': summary([row['cpu_ns'] for row in offers]),
            'completion_wall': summary([row['wall_ns'] for row in completions]), 'completion_cpu': summary([row['cpu_ns'] for row in completions]),
            'publisher_materialization_fence_wall': summary(publisher_ns), 'synthetic_notification_callback_wall': summary(notification_ns), 'take_provider_wall': summary(take_ns), 'fence_action_wall': summary(p.action_ns), 'fence_including_check_wall': summary(p.fence_ns),
            'stage_samples': {'publisher_materialization_fence_wall_ns': publisher_ns, 'synthetic_notification_callback_wall_ns': notification_ns, 'take_provider_wall_ns': take_ns, 'fence_action_wall_ns': p.action_ns, 'fence_including_check_wall_ns': p.fence_ns},
            'allocation_20_calls': {'traced_peak_increment_bytes': peak-initial, 'traced_net_bytes': current-initial,
                'scope': 'Python traced only; includes fixture identity/packet/timing-row overhead; excludes ROS C/native/RSS/CDR'},
            'paced': paced, 'notifications': notifications[0], 'validated_population': {'objects': len(report['objects']), 'tracks': len(n.tracker.tracks), 'grasps': expected_grasps, 'contacts': expected_grasps}, 'captured_counts': message_counts, 'message_types': sorted(types),
            'freshness': 'original registered acquisition ROS nanoseconds; synthetic now +1ms, deliberate +600ms rejection; no receipt restamping'}
        ingress.close()
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == FROZEN_SOURCE
    args.output.write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps({key: {k: v[k] for k in ('offer_wall', 'completion_wall', 'completion_cpu', 'take_provider_wall', 'fence_action_wall', 'allocation_20_calls')} for key,v in results['cases'].items()}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=300)
    parser.add_argument('--duration', type=float, default=2.)
    run(parser.parse_args())
