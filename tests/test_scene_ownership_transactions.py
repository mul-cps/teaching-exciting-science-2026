"""Native ROS messages/futures exercising scene ownership without hardware."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for package in ('so101_station_scene', 'so101_station_objects'):
    sys.path.insert(0, str(ROOT / 'ros2/src' / package))

from so101_station_objects.objects import StableObjectTracker
from so101_station_scene.scene_geometry import LiveObjectRegistry


def obj(x=0, color='red'):
    return {'position': np.array([x, 0., .02]), 'color': color}


def test_owned_tracker_keeps_identity_after_occlusion_and_large_pickup_motion():
    tracker = StableObjectTracker()
    assert tracker.update([obj()], 1.)[0]['track_id'] == 1
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'reserved'})
    assert tracker.update([], 4.) == []
    assert tracker.update([obj(.2)], 5., expected_owned_positions={1: [.2, 0, .02]})[0]['track_id'] == 1
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'fault'})
    assert tracker.update([obj(.3)], 9., expected_owned_positions={1: [.3, 0, .02]})[0]['track_id'] == 1
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'detached'})
    tracker.update([], 12.)
    assert tracker.update([obj(.3)], 13.)[0]['track_id'] == 2


def test_owned_tracker_does_not_invent_pickup_evidence_for_ambiguous_same_color():
    tracker = StableObjectTracker()
    tracker.update([obj()], 1.)
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'reserved'})
    assert tracker.update([obj(.2), obj(.3)], 4.) == []


def test_registry_rejects_unknown_duplicate_ids_while_an_object_is_reserved():
    registry = LiveObjectRegistry(ttl_s=1, min_extent_m=.002, max_extent_m=.3)
    registry.update('red-0001', (0, 0, .02), (.02, .02, .02), (0, 0, 0, 1), now_mono_s=1.)
    registry.attach('red-0001', link_name='gripper', touch_links=['jaw'],
                    link_pose={'position': (0, 0, 0), 'orientation_xyzw': (0, 0, 0, 1)})
    with pytest.raises(ValueError, match='reserved'):
        registry.update('red-0002', (.2, 0, .02), (.02, .02, .02), (0, 0, 0, 1), now_mono_s=4.)


@pytest.fixture
def node():
    rclpy = pytest.importorskip("rclpy", reason="native ROS ownership transaction test")
    from rclpy.task import Future
    from so101_station_scene.scene_updater import SceneUpdater
    rclpy.init()
    value = SceneUpdater()
    value._static_sent = True
    value._lookup = lambda *args: SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    value.registry.update('red-0001', (0, 0, .02), (.02, .02, .02),
                          (0, 0, 0, 1), now_mono_s=1.)
    value.future = Future()
    value.scene_client = SimpleNamespace(service_is_ready=lambda: True,
                                         call_async=lambda request: value.future)
    yield value
    value.destroy_node()
    rclpy.shutdown()


def request(node, request_id='test-1', action='attach', **fields):
    from std_msgs.msg import String
    data = {'schema': 1, 'request_id': request_id, 'action': action,
            'track_id': 'red-0001', 'link_name': 'gripper', 'touch_links': ['jaw'], **fields}
    node._attachment_request(String(data=json.dumps(data)))
    node._maintenance()
    return data


def test_native_attach_failure_retains_reservation_and_replay_is_idempotent(node):
    from moveit_msgs.srv import ApplyPlanningScene
    from std_msgs.msg import String
    data = request(node)
    assert node.registry.is_attached('red-0001')
    node.future.set_result(ApplyPlanningScene.Response(success=False))
    assert node.registry.is_attached('red-0001')
    assert node._attachment_results['test-1']['state'] == 'fault'
    node._attachment_request(String(data=json.dumps(data)))
    assert not node._attachment_queue


def test_native_timeout_is_bounded_and_late_ack_cannot_clear_fault(node, monkeypatch):
    request(node)
    monkeypatch.setattr('so101_station_scene.scene_updater.time.monotonic', lambda: 10**12)
    node._maintenance()
    assert node._attachment_results['test-1']['state'] == 'fault'
    assert node.registry.is_attached('red-0001')
    from moveit_msgs.srv import ApplyPlanningScene
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    assert node._attachment_results['test-1']['state'] == 'fault'


def test_native_rejects_pose_in_wrong_frame_before_any_scene_call(node):
    request(node, action='detach', world_pose={'frame_id': 'camera', 'position': [0, 0, 0],
                                             'orientation_xyzw': [0, 0, 0, 1]})
    assert node._attachment_results['test-1']['state'] == 'rejected'
    assert not node.future.done()


def test_native_attachment_queue_and_request_size_are_bounded(node):
    from std_msgs.msg import String
    node._scene_busy = True
    for index in range(70):
        node._attachment_request(String(data=json.dumps({'schema': 1, 'request_id': str(index),
            'track_id': 'red-0001', 'action': 'attach', 'link_name': 'gripper', 'touch_links': ['jaw']})))
    assert len(node._attachment_queue) <= 32
    assert len(node._attachment_inflight) <= 32


def query_client(node):
    from rclpy.task import Future
    pending = Future()
    node.scene_read_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: pending)
    return pending


def test_native_recover_reads_actual_world_before_releasing_reservation(node):
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from so101_station_scene.scene_updater import box_collision_object
    request(node)
    node.future.set_exception(RuntimeError('transport lost'))
    assert node.registry.is_attached('red-0001')
    pending = query_client(node)
    request(node, request_id='recover', action='recover')
    assert node.registry.is_attached('red-0001')
    response = GetPlanningScene.Response()
    response.scene.world.collision_objects = [box_collision_object(
        'colorbrick_red-0001', 'base_link', node.get_clock().now().to_msg(),
        (.1, .2, .03), node.registry._objects['colorbrick_red-0001'].size_m, (0, 0, 0, 1))]
    pending.set_result(response)
    assert not node.registry.is_attached('red-0001')
    assert node.registry._objects['colorbrick_red-0001'].center_m == (.1, .2, .03)
    assert node._scene_fault is None
    assert node._attachment_results['recover']['state'] == 'detached'


def test_native_recover_cannot_release_while_timed_out_rpc_is_still_outstanding(node, monkeypatch):
    from moveit_msgs.srv import GetPlanningScene
    request(node)
    monkeypatch.setattr('so101_station_scene.scene_updater.time.monotonic', lambda: 10**12)
    node._maintenance()
    original = node.future
    pending = query_client(node)
    request(node, request_id='recover', action='recover')
    response = GetPlanningScene.Response()
    pending.set_result(response)
    assert node.registry.is_attached('red-0001')
    assert node._scene_fault['future'] is original
    assert 'outstanding' in node._attachment_results['recover']['reason']


def test_native_detach_only_releases_after_acknowledged_world_replacement(node):
    from moveit_msgs.srv import ApplyPlanningScene
    from rclpy.task import Future
    request(node)
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    node.future = Future()
    request(node, request_id='detach', action='detach', world_pose={
        'frame_id': 'base_link', 'position': [.2, 0, .02], 'orientation_xyzw': [0, 0, 0, 1]})
    assert node.registry.is_attached('red-0001')
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    assert not node.registry.is_attached('red-0001')
    assert node.registry._objects['colorbrick_red-0001'].center_m == (.2, 0, .02)


def test_native_contact_read_timeout_faults_and_suppresses_queued_mutations(node, monkeypatch):
    request(node, request_id='lock', action='lock')
    pending = query_client(node)
    request(node, action='allow_contact')
    monkeypatch.setattr('so101_station_scene.scene_updater.time.monotonic', lambda: 10**12)
    node._maintenance()
    assert node._attachment_results['test-1']['state'] == 'fault'
    assert node._scene_fault
    from moveit_msgs.srv import GetPlanningScene
    pending.set_result(GetPlanningScene.Response())
    assert node._scene_fault  # late read must never send an allow-contact diff


def test_native_same_request_id_different_payload_never_repeats_mutation(node):
    from std_msgs.msg import String
    data = request(node)
    data['touch_links'] = ['other_jaw']
    node._attachment_request(String(data=json.dumps(data)))
    assert not node._attachment_queue
    assert node._attachment_payloads['test-1']['touch_links'] == ['jaw']


def test_owned_tracker_requires_timestamped_gripper_prediction_to_identify_a_lift():
    tracker = StableObjectTracker()
    tracker.update([obj()], 1.)
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'reserved'})
    assert tracker.update([obj(.2)], 4.) == []
    assert tracker.update([obj(.2)], 5., expected_owned_positions={1: [.5, 0, .02]}) == []
    assert tracker.update([obj(.2)], 6., expected_owned_positions={1: [.2, 0, .02]})[0]['track_id'] == 1


def test_owned_tracker_withholds_two_candidates_near_expected_gripper_pose():
    tracker = StableObjectTracker()
    tracker.update([obj()], 1.)
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'reserved'})
    assert tracker.update([obj(.2), obj(.21)], 4., expected_owned_positions={1: [.2, 0, .02]}) == []


def test_contact_fault_does_not_create_phantom_tracker_ownership():
    tracker = StableObjectTracker()
    tracker.update([obj()], 1.)
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'fault',
                              'action': 'allow_contact', 'ownership_reserved': False})
    tracker.ownership_status({'track_id': 'red-0001', 'state': 'contact_disallowed'})
    assert not tracker.owned
    tracker.update([], 3.)
    assert tracker.update([obj()], 4.)[0]['track_id'] == 2


def test_native_lock_freezes_world_identity_until_unlock_or_attachment(node):
    request(node, action='lock')
    assert node._attachment_results['test-1']['state'] == 'locked'
    assert node.registry.is_attached('red-0001')
    assert node.registry.expire(now_mono_s=10**12) == []
    assert not node.future.done()
    request(node, request_id='unlock', action='unlock')
    assert not node.registry.is_attached('red-0001')
    assert node._attachment_results['unlock']['state'] == 'unlocked'


def test_native_contact_restores_exact_original_acm_and_refuses_external_clobber(node):
    from moveit_msgs.msg import AllowedCollisionEntry
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rclpy.task import Future
    import copy
    request(node, request_id='lock', action='lock')
    original = GetPlanningScene.Response()
    original.scene.allowed_collision_matrix.entry_names = ['jaw', 'other']
    original.scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[True, True]), AllowedCollisionEntry(enabled=[True, False])]
    pending = query_client(node)
    submitted = []
    node.scene_client.call_async = lambda request: (submitted.append(request) or node.future)
    request(node, action='allow_contact')
    pending.set_result(original)
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    assert node._attachment_results['test-1']['state'] == 'contact_allowed'
    applied = copy.deepcopy(submitted[-1].scene.allowed_collision_matrix)
    node.future = Future()
    pending = query_client(node)
    request(node, request_id='restore', action='disallow_contact')
    current = GetPlanningScene.Response()
    current.scene.allowed_collision_matrix = applied
    pending.set_result(current)
    assert submitted[-1].scene.allowed_collision_matrix == original.scene.allowed_collision_matrix
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    assert node._attachment_results['restore']['state'] == 'contact_disallowed'
    assert not node._contact_previous


def test_native_contact_cleanup_does_not_overwrite_an_external_matrix_update(node):
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rclpy.task import Future
    import copy
    request(node, request_id='lock', action='lock')
    pending = query_client(node)
    request(node, action='allow_contact')
    pending.set_result(GetPlanningScene.Response())
    node.future.set_result(ApplyPlanningScene.Response(success=True))
    snapshot = node._contact_previous['colorbrick_red-0001']
    node.future = Future()
    pending = query_client(node)
    request(node, request_id='restore', action='disallow_contact')
    current = GetPlanningScene.Response()
    current.scene.allowed_collision_matrix = copy.deepcopy(snapshot['applied'])
    current.scene.allowed_collision_matrix.entry_values[0].enabled[0] = True
    pending.set_result(current)
    assert node._attachment_results['restore']['state'] == 'fault'
    assert 'externally' in node._attachment_results['restore']['reason']
    assert not node.future.done()
    assert node.registry.is_attached('red-0001')


def test_native_acm_comparison_accepts_moveit_serialization_order():
    pytest.importorskip('rclpy')
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    from so101_station_scene.scene_updater import _acm_equivalent
    left = AllowedCollisionMatrix(entry_names=['z', 'a'], entry_values=[
        AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
    right = AllowedCollisionMatrix(entry_names=['a', 'z'], entry_values=[
        AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
    assert _acm_equivalent(left, right)
