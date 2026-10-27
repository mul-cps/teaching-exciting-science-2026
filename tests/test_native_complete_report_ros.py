"""Synthetic native source through the real serialized ROS completion sink.

Run actual ROS only in DDS177/localhost1 after the independent resource gate.
"""
import os
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_objects'))


def test_actual_native_unavailable_constructor_and_offer():
    rclpy = pytest.importorskip('rclpy')
    assert os.environ.get('ROS_DOMAIN_ID') == '177'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    from so101_station_objects.objects import ColoredObjectsNode
    rclpy.init()
    n = None
    try:
        n = ColoredObjectsNode(native_complete_provider=None)
        assert n.analysis_worker is None
        assert not n.offer_native_complete(None)
    finally:
        if n is not None:
            n.destroy_node()
        rclpy.shutdown()

# These extracted tests execute real completion/publisher code without ROS. They
# are pre-resource source checks, never native executor qualification.
import ast
import copy
import json
from dataclasses import replace
from threading import get_ident, Thread
from types import MethodType, SimpleNamespace as NS

from test_complete_report_admission import node as cpu_node, job, report, methods, encode_baseline
from test_native_complete_report import Provider, identity, retained
from so101_station_objects.native_complete_report import NativeCompleteIngress, NativeCompleteReport, ReportLimits


def native_node():
    n = cpu_node(real_publisher=True)
    provider = Provider(identity())
    n._native_mode = True
    n._native_provider = provider
    n.native_ingress = NativeCompleteIngress(provider, lambda: None)
    from test_complete_report_admission import SOURCE
    tree = ast.parse(SOURCE.read_text())
    fn = next(v for v in ast.walk(tree) if isinstance(v, ast.FunctionDef) and v.name == '_native_current')
    ns = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), ns)
    n._native_current = MethodType(ns['_native_current'], n)
    handle = object()
    provider.handles[handle] = identity()
    return n, provider, handle


@pytest.mark.parametrize('failure', ['missing', 'forged', 'revoked', 'bool_check', 'closed'])
def test_native_rejected_handle_never_commits(failure):
    n, p, handle = native_node()
    if failure == 'missing':handle = None
    if failure == 'forged':handle = object()
    if failure == 'revoked':p.revoked = True
    if failure == 'bool_check':p.check = lambda h: True
    if failure == 'closed':n.native_ingress.close()
    old = n.tracker
    n._complete_report(dict(job(), native_admission=handle), report())
    assert n.tracker is old and n.last_support_plane is None
    assert n.last_analysis_result_stamp is None
    assert len(n.published) == 1 and json.loads(n.published[0].data)['state'] == 'unavailable'


@pytest.mark.parametrize('behavior', ['success', 'false', 'true_without_action', 'raises', 'twice', 'thread', 'retained_callback'])
def test_final_native_fence_protocol_and_at_most_once(behavior):
    n, p, handle = native_node()
    actions, saved = [], []
    def fence(h, action):
        p.check(h)
        if behavior == 'false':return False
        if behavior == 'true_without_action':return True
        if behavior == 'raises':raise ValueError('untrusted exception')
        if behavior == 'retained_callback':saved.append(action);return False
        if behavior == 'thread':
            def run():
                try:action()
                except RuntimeError:actions.append('wrong thread rejected')
            t = Thread(target=run);t.start();t.join()
            return False
        action()
        if behavior == 'twice':action()
        return True
    p.publish_if_current = fence
    payload = report();payload.update(stamp=10., support_plane={})
    commit = lambda: actions.append('commit')
    if behavior == 'twice':
        with pytest.raises(RuntimeError, match='after action began'):
            n._publish_report(payload, job()['ros_stamp'], commit=commit, native_admission=handle)
        assert actions == ['commit'] and len(n.published) == 5
    else:
        accepted = n._publish_report(payload, job()['ros_stamp'], commit=commit, native_admission=handle)
        assert accepted is (behavior == 'success')
        assert actions.count('commit') == int(behavior == 'success')
        assert len(n.published) == (5 if accepted else 1)
    for callback in saved:
        with pytest.raises(RuntimeError, match='protocol'):
            callback()
    assert actions.count('commit') <= 1


@pytest.mark.parametrize('stage', ['tf', 'materialization', 'final'])
def test_native_late_revocation_discards_staged_observation(stage):
    n, p, handle = native_node()
    old = n.tracker
    n.tracker.attachment_records[1] = dict(phase='locked', world_frame='world', world_position=[0., 0., 0.])
    def lookup(*args):
        if stage == 'tf':p.revoked = True
        return NS(transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    n.tf_buffer = NS(lookup_transform=lookup)
    payload = report()
    if stage == 'materialization':
        class Revoke(dict):
            def items(self):p.revoked = True;return super().items()
        payload['validation_diagnostics'] = Revoke(value=1)
    if stage == 'final':
        def fence(handle, action):p.revoked = True;p.check(handle)
        p.publish_if_current = fence
    n._complete_report(dict(job(), native_admission=handle), payload)
    assert n.tracker is old and n.last_support_plane is None and n.last_analysis_result_stamp is None
    assert len(n.published) == 1


def test_native_track_capacity_rejects_whole_staged_observation():
    n, p, handle = native_node()
    n.native_ingress.limits = ReportLimits(1, 0)
    from test_complete_report_admission import observation
    old = n.tracker
    n._complete_report(dict(job(), native_admission=handle), report([observation()]))
    assert n.tracker is old and old.next_id == 1 and not old.tracks
    assert n.last_support_plane is None and n.last_analysis_result_stamp is None


@pytest.mark.parametrize('visible,ownership', [(False, None), (True, None), (False, 'locked'), (False, 'attached')])
def test_actual_common_sink_five_messages_cpu_native_frozen_cases(visible, ownership):
    rclpy = pytest.importorskip('rclpy')
    assert os.environ.get('ROS_DOMAIN_ID') == '177'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    from so101_station_objects.objects import ColoredObjectsNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.serialization import serialize_message
    from rclpy.time import Time
    from test_complete_report_admission import ROOT, decode_baseline
    fixture = json.loads((ROOT / 'tests/fixtures/complete_report_admission/cpu_baseline.json').read_text())
    ident = identity()
    p = Provider(ident)
    if ownership is not None:p.limits = ReportLimits(2, 2)
    rclpy.init()
    cpu = native = executor = None
    try:
        cpu = ColoredObjectsNode()
        native = ColoredObjectsNode(native_complete_provider=p)
        assert native.analysis_worker is None and native.derived is None
        assert not hasattr(native, 'color_mask_backend') and not hasattr(native, 'sync')
        executor = SingleThreadedExecutor();executor.add_node(native)
        outputs = [[], []]
        tf_calls = [[], []]
        for n, captured in zip((cpu, native), outputs):
            n.require_registration_tracking = False
            before = decode_baseline(fixture['initial_state'])
            n.tracker.__dict__.update(before['tracker'])
            n.last_support_plane = before['last_support_plane']
            n.last_analysis_result_stamp = before['last_analysis_result_stamp']
            for name in ('detections_pub', 'markers_pub', 'grasps_pub', 'contacts_pub', 'status_pub'):
                setattr(n, name, NS(publish=lambda msg, name=name, captured=captured: captured.append((name, type(msg), serialize_message(msg)))))
        for index, case in enumerate(fixture['cases']):
            if ownership is not None and index == 3:
                for node_index, n in enumerate((cpu, native)):
                    assert list(n.tracker.tracks) == [1]
                    position = n.tracker.tracks[1]['position'].copy()
                    n.tracker.owned.add(1)
                    n.tracker.attachment_records[1] = dict(phase=ownership,
                        world_frame='world', world_position=position,
                        link_name='gripper', link_pose={'position': position})
                    def lookup(board, frame, at, node_index=node_index):
                        tf_calls[node_index].append((board, frame, at.nanoseconds))
                        return NS(transform=NS(translation=NS(x=0., y=0., z=0.),
                            rotation=NS(x=0., y=0., z=0., w=1.)))
                    n.tf_buffer = NS(lookup_transform=lookup)
            ns = case['original_ros_stamp']['sec'] * 1_000_000_000 + case['original_ros_stamp']['nanosec']
            stamp = Time(nanoseconds=ns).to_msg()
            for n in (cpu, native):
                n.get_clock = lambda ns=ns: NS(now=lambda: Time(nanoseconds=ns + 10_000_000))
            original = replace(ident, observation_ros_ns=ns, color_frame_number=10+index, depth_frame_number=10+index)
            p.originals.append(original)
            cpu_report = decode_baseline(case['analyzer_report'])
            native_report = retained(index)
            if ownership is not None and index >= 3:
                for value in (cpu_report, native_report):
                    value['objects'] = [] if index == 3 else [copy.deepcopy(retained()['objects'][0]) for _ in range(2)]
            if visible:
                for value in (cpu_report, native_report):
                    for obj in value['objects']:
                        obj.update(height_m=None, grasps=[], pickup_eligible=False,
                                   eligibility_reason='not_support_plane_qualified',
                                   observation_geometry='visible_surface_centroid',
                                   dimensions_semantics='visible_surface_extent_only')
            seconds = stamp.sec + stamp.nanosec * 1e-9
            cpu._complete_report(dict(stamp=seconds, ros_stamp=stamp), cpu_report)
            assert native.offer_native_complete(NativeCompleteReport(original, native_report))
            from test_object_result_delivery_ros import spin_until
            spin_until(executor, lambda: native.last_analysis_result_stamp == seconds)
            assert outputs[0] == outputs[1]  # Exact order, ROS types and CDR bytes, including original headers.
            assert encode_baseline(native.tracker.__dict__) == encode_baseline(cpu.tracker.__dict__)
            if not visible and ownership is None:
                assert encode_baseline(native.tracker.__dict__) == case['accepted_state']['tracker']
            assert encode_baseline(native.last_support_plane) == case['accepted_state']['last_support_plane']
        if ownership is not None:
            assert tf_calls[0] == tf_calls[1]
            assert [v[1] for v in tf_calls[0]] == [('world' if ownership == 'locked' else 'gripper')] * 2
            assert [v[2] for v in tf_calls[0]] == [c['original_ros_stamp']['sec'] * 1_000_000_000 + c['original_ros_stamp']['nanosec'] for c in fixture['cases'][3:]]
            assert native.tracker.owned == {1} and list(native.tracker.tracks) == [1]
            assert native.tracker.next_id == 2
    finally:
        if executor is not None:
            executor.remove_node(native);executor.shutdown(timeout_sec=1)
        for n in (native, cpu):
            if n is not None:assert n.destroy_node() is not False
        rclpy.shutdown()


@pytest.mark.parametrize('failure', ['stale', 'future', 'duplicate', 'registration', 'source_mismatch'])
def test_native_stamp_registration_and_source_rejection(failure):
    n, p, handle = native_node()
    stamp = 10.
    if failure == 'stale':n.now = 10.6
    if failure == 'future':n.now = 9.9
    if failure == 'duplicate':n.last_analysis_result_stamp = stamp
    if failure == 'registration':n.require_registration_tracking = True;n.registration_last_tracking_stamp = 9.
    completion = dict(job(stamp), native_admission=handle)
    if failure == 'source_mismatch':completion['derived_token'] = object()
    old = n.tracker
    n._complete_report(completion, report())
    assert n.tracker is old and n.last_support_plane is None
    assert n.last_analysis_result_stamp == (stamp if failure == 'duplicate' else None)
    assert not n.published or all(json.loads(v.data)['state'] != 'tracking' for v in n.published)


def test_native_cpu_handle_mismatch_rejects_without_publication():
    n = cpu_node(real_publisher=True)
    n._complete_report(dict(job(), native_admission=object()), report())
    assert not n.published and n.last_analysis_result_stamp is None


def test_native_guard_close_and_source_scheduler_contract():
    from test_complete_report_admission import SOURCE
    tree = ast.parse(SOURCE.read_text())
    fs = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    guard = ast.unparse(fs['_consume_native_complete'])
    assert 'divmod(admitted.identity.observation_ros_ns, 1000000000)' in guard
    assert 'self._complete_report(' in guard
    assert not any(word in guard for word in ('period', 'analysis_worker', 'analyze_rgbd', 'lookup_transform'))
    assert ast.unparse(fs['offer_native_complete'].body[-1]) == 'return False if ingress is None else ingress.offer(packet)'
    close = ast.unparse(fs['destroy_node'])
    assert close.index('self.native_ingress.close()') < close.index('super().destroy_node()')
    n, p, handle = native_node()
    n.native_ingress.close()
    assert n.native_ingress.take() is None and not n.native_ingress.offer(None)
    assert not n._native_current(handle)
