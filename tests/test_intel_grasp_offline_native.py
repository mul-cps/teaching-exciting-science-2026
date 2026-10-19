"""Native ROS message/service contracts using explicit synthetic recorded fixtures."""
from copy import deepcopy
import json
from pathlib import Path
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GraspPlanning
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tools.intel_grasp_backend import ros_offline_backend as backend
from so101_station_scene.intel_grasp_adapter import checked_grasps
from tools.intel_grasp_backend.run_reference import digest


def recorded():
    return {'recorded_source': True, 'live_timestamp_refresh': False,
        'source_provenance': {'schema': 'station-frozen-native-pcd-v1',
            'acquisition_stamp_ns': 123000000456, 'frame_id': 'camera_optical',
            'pcd_sha256': 'a'*64, 'live_timestamp_refresh': False},
        'classification': {'scored_images': 9, 'images_at_original_threshold': 0,
                           'used_geometric_fallback': True},
        'grasps': [{'bottom': [.1, .2, .3], 'binormal': [1., 0., 0.], 'axis': [0., 1., 0.],
                    'approach': [0., 0., 1.], 'score': 1., 'origin': 'upstream_geometric_fallback'}]}


def test_recorded_moveit_pose_never_refreshes_time_or_invents_jaw_mapping():
    response = backend.response_from_recorded(recorded())
    grasp = response.grasps[0]
    assert grasp.grasp_pose.header.frame_id == 'camera_optical'
    assert grasp.grasp_pose.header.stamp.sec == 123 and grasp.grasp_pose.header.stamp.nanosec == 456
    assert not grasp.grasp_posture.points and not grasp.pre_grasp_posture.joint_names
    assert 'Recorded diagnostic' in response.error_code.message
    with pytest.raises(ValueError, match='frame/stamp'):
        checked_grasps(response.grasps, frame='camera_optical', now=1000., max_age=.5,
                       earliest_stamp=123., max_candidates=8)


@pytest.mark.parametrize('field', ['bottom', 'axis', 'approach'])
def test_bad_recorded_basis_or_position_rejected(field):
    report = recorded()
    report['grasps'][0][field] = [float('nan'), 0., 0.]
    with pytest.raises(ValueError):
        backend.response_from_recorded(report)


def test_improper_rotation_rejected():
    report = recorded()
    report['grasps'][0]['axis'] = [1., 0., 0.]
    with pytest.raises(ValueError, match='rotation'):
        backend.response_from_recorded(report)


def spin_until(executor, condition, timeout=2.):
    end = time.monotonic()+timeout
    while not condition():
        assert time.monotonic() < end
        executor.spin_once(timeout_sec=.005)


def test_offline_ros_service_status_keeps_recorded_contract(tmp_path, monkeypatch):
    report = recorded()
    from tools.intel_grasp_backend.capture_to_pcd import export
    import numpy as np
    capture = tmp_path/'capture'; capture.mkdir()
    np.savez(capture/'cloud-stride2.npz', points_optical_m=np.array([[0., 0., .4]], np.float32), rgb=np.array([[1, 2, 3]], np.uint8))
    (capture/'capture.json').write_text(json.dumps({'stamp_ns': 123000000456, 'color_frame': 'camera_optical', 'model_sha256': 'a'*64}))
    pcd = tmp_path/'fixture.pcd'; export(capture, pcd)
    report['source_provenance']['pcd_sha256'] = digest(pcd)
    provenance = tmp_path/'source.json'; provenance.write_text(json.dumps(report['source_provenance']))
    engine = tmp_path/'fixture-engine'; engine.write_bytes(b'synthetic non-executable fixture')
    model = tmp_path/'fixture-model'; model.write_bytes(b'synthetic model')
    manifest = tmp_path/'build.json'; manifest.write_text(json.dumps({
        'schema': 'station-offline-gpd-build-v1', 'model': str(model), 'model_sha256': digest(model),
        'engine': str(engine), 'binaries_sha256': {str(engine): digest(engine)}}))
    monkeypatch.setattr(backend, 'run_once', lambda *args, **kwargs: deepcopy(report))
    rclpy.init()
    executor = SingleThreadedExecutor()
    server = backend.OfflineGraspBackend(manifest, pcd, provenance, tmp_path/'attempts')
    client_node = Node('synthetic_offline_client')
    executor.add_node(server); executor.add_node(client_node)
    client = client_node.create_client(GraspPlanning, '/offline/intel_grasp/plan')
    statuses, poses = [], []
    client_node.create_subscription(String, '/offline/intel_grasp/status', lambda m: statuses.append(json.loads(m.data)), 10)
    client_node.create_subscription(PoseArray, '/offline/intel_grasp/poses', poses.append, 1)
    try:
        spin_until(executor, lambda: client.service_is_ready() and server.status.get_subscription_count() > 0 and server.poses.get_subscription_count() > 0)
        request = GraspPlanning.Request(group_name='offline_reference')
        request.target.id = server.dataset_id
        future = client.call_async(request)
        spin_until(executor, lambda: future.done() and statuses and poses)
        assert future.result().error_code.val == MoveItErrorCodes.SUCCESS
        assert poses[0].header.stamp.sec == 123
        assert statuses[-1]['recorded_source'] is True
        assert statuses[-1]['target_association_verified'] is False
        assert statuses[-1]['live_timestamp_refresh'] is False
        assert statuses[-1]['tool_mapping_qualified'] is False
        assert statuses[-1]['proposal_origins'] == ['upstream_geometric_fallback']
        request.target.header.stamp = client_node.get_clock().now().to_msg()
        future = client.call_async(request)
        spin_until(executor, future.done)
        assert future.result().error_code.val == MoveItErrorCodes.FAILURE
        assert 'invalid_dataset_request' in future.result().error_code.message
    finally:
        executor.remove_node(server); executor.remove_node(client_node)
        server.destroy_node(); client_node.destroy_node()
        executor.shutdown(); rclpy.shutdown()


@pytest.mark.parametrize('domain', ['0', '-1', '233'])
def test_offline_node_entry_rejects_invalid_domain_before_init(monkeypatch, domain):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    calls = []
    monkeypatch.setattr(backend.rclpy, 'init', lambda **kwargs: calls.append('init'))
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID'):
        backend.main(['--build-manifest', '/fixture/build', '--pcd', '/fixture/pcd',
                      '--provenance', '/fixture/source', '--output-root', '/fixture/output'])
    assert not calls


def test_offline_node_defaults_and_constructor_cleanup(monkeypatch):
    import os
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    calls = []
    def init(**kwargs):
        assert os.environ['ROS_DOMAIN_ID'] == '76'
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        calls.append('init')
    def fail(*args):
        raise ValueError('synthetic constructor failure')
    monkeypatch.setattr(backend.rclpy, 'init', init)
    monkeypatch.setattr(backend.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    monkeypatch.setattr(backend, 'OfflineGraspBackend', fail)
    with pytest.raises(ValueError, match='constructor failure'):
        backend.main(['--build-manifest', '/fixture/build', '--pcd', '/fixture/pcd',
                      '--provenance', '/fixture/source', '--output-root', '/fixture/output'])
    assert calls == ['init', 'shutdown']


@pytest.mark.parametrize('domain', ['0', '76'])
def test_actual_ros_checker_refuses_domain_zero_and_live_station(monkeypatch, domain):
    from tools.intel_grasp_backend.check_offline_ros import check
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    with pytest.raises(ValueError, match='isolated'):
        check('/fixture/build', '/fixture/pcd', '/fixture/source', Path('/fixture/output'))
