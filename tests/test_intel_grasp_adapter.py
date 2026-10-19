"""Synthetic DDS services only: no arm owner, camera, serial or motion endpoints."""
import ast
import importlib.util
from pathlib import Path
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import Grasp, MoveItErrorCodes
from moveit_msgs.srv import GraspPlanning
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'ros2/src/so101_station_scene/so101_station_scene/intel_grasp_adapter.py'
spec = importlib.util.spec_from_file_location('intel_grasp_adapter_under_test', MODULE)
adapter_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter_module)


def grasp(stamp=100.):
    value = Grasp()
    value.grasp_pose.header.frame_id = 'base_link'
    value.grasp_pose.header.stamp.sec = int(stamp)
    value.grasp_pose.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
    value.grasp_pose.pose.position.x = .25
    value.grasp_pose.pose.orientation.w = 1.
    value.grasp_quality = .7
    approach = value.pre_grasp_approach
    approach.direction.header.frame_id = 'base_link'
    approach.direction.vector.z = -1.
    approach.min_distance = .02
    approach.desired_distance = .05
    return value


def checked(values, now=100.1):
    return adapter_module.checked_grasps(values, frame='base_link', now=now,
                                         max_age=.5, earliest_stamp=100., max_candidates=8)


def test_checked_batch_preserves_source():
    value = grasp()
    assert checked([value]) == [value]
    assert value.grasp_pose.header.stamp.sec == 100


@pytest.mark.parametrize('change', [
    lambda g: setattr(g.grasp_pose.header, 'frame_id', 'camera'),
    lambda g: setattr(g.grasp_pose.header.stamp, 'sec', 99),
    lambda g: setattr(g.grasp_pose.header.stamp, 'sec', 101),
    lambda g: setattr(g.grasp_pose.pose.position, 'x', float('nan')),
    lambda g: setattr(g.grasp_pose.pose.orientation, 'w', 0.),
    lambda g: setattr(g.pre_grasp_approach.direction.vector, 'z', -2.),
    lambda g: setattr(g.pre_grasp_approach, 'desired_distance', -.1),
    lambda g: setattr(g.pre_grasp_approach, 'desired_distance', .3),
    lambda g: setattr(g.pre_grasp_approach.direction.header, 'frame_id', 'tool'),
])
def test_malformed_batch_rejected(change):
    value = grasp()
    change(value)
    with pytest.raises(ValueError):
        checked([value])


def test_empty_oversized_and_stale_rejected():
    for values in ([], [grasp()]*9):
        with pytest.raises(ValueError):
            checked(values)
    with pytest.raises(ValueError):
        checked([grasp()], now=101.)


@pytest.fixture
def ros():
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node
    yield executor, add
    for node in reversed(nodes):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


def spin_until(executor, predicate, timeout=2.):
    deadline = time.monotonic()+timeout
    while not predicate():
        assert time.monotonic() < deadline, 'synthetic ROS operation timed out'
        executor.spin_once(timeout_sec=.005)


def make_adapter(add, enabled=True, timeout=.15, service="/synthetic/station/intel/plan"):
    return add(adapter_module.IntelGraspAdapter(parameter_overrides=[
        Parameter('enabled', value=enabled), Parameter('timeout_s', value=timeout),
        Parameter('backend_service', value='/synthetic/intel/plan'),
        Parameter('service', value=service),
        Parameter('proposal_topic', value='/synthetic/station/intel/poses'),
        Parameter('status_topic', value='/synthetic/station/intel/status')]))


def client_request(node):
    request = GraspPlanning.Request()
    request.group_name = 'manipulator'
    request.target.id = 'red-1'
    request.target.header.frame_id = 'base_link'
    request.target.header.stamp = node.get_clock().now().to_msg()
    return request


def make_client(add, executor):
    client_node = add(Node('synthetic_intel_client'))
    client = client_node.create_client(GraspPlanning, '/synthetic/station/intel/plan')
    spin_until(executor, client.service_is_ready)
    return client_node, client


def test_disabled_and_absent_backend(ros):
    executor, add = ros
    adapter = make_adapter(add, enabled=False)
    assert adapter.timer is None
    node, client = make_client(add, executor)
    future = client.call_async(client_request(node))
    spin_until(executor, future.done)
    assert future.result().error_code.val == MoveItErrorCodes.FAILURE
    adapter.enabled = True
    future = client.call_async(client_request(node))
    spin_until(executor, future.done)
    assert future.result().error_code.val == MoveItErrorCodes.FAILURE
    assert adapter.pending is None


def test_success_publishes_separate_unchanged_proposals(ros):
    executor, add = ros
    adapter = make_adapter(add)
    backend = add(Node('synthetic_intel_backend'))
    calls = []
    def serve(request, response):
        calls.append(request)
        value = grasp()
        value.grasp_pose.header.stamp = backend.get_clock().now().to_msg()
        response.grasps = [value]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response
    backend.create_service(GraspPlanning, '/synthetic/intel/plan', serve)
    spin_until(executor, adapter.backend.service_is_ready)
    node, client = make_client(add, executor)
    received = []
    node.create_subscription(PoseArray, '/synthetic/station/intel/poses', received.append, 1)
    future = client.call_async(client_request(node))
    spin_until(executor, lambda: future.done() and received)
    assert future.result().error_code.val == MoveItErrorCodes.SUCCESS
    assert received[0].header == future.result().grasps[0].grasp_pose.header
    assert received[0].poses[0] == future.result().grasps[0].grasp_pose.pose
    assert len(calls) == 1


def test_timeout_busy_and_late_response_never_published(ros):
    from rclpy.task import Future
    executor, add = ros
    adapter = make_adapter(add, timeout=.06)
    backend = add(Node('synthetic_slow_intel_backend'))
    release = Future(executor=executor)
    calls = []
    async def serve(request, response):
        calls.append(request)
        await release
        value = grasp()
        value.grasp_pose.header.stamp = backend.get_clock().now().to_msg()
        response.grasps = [value]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response
    backend.create_service(GraspPlanning, '/synthetic/intel/plan', serve)
    spin_until(executor, adapter.backend.service_is_ready)
    node, client = make_client(add, executor)
    published = []
    node.create_subscription(PoseArray, '/synthetic/station/intel/poses', published.append, 1)
    first = client.call_async(client_request(node))
    spin_until(executor, lambda: bool(calls))
    second = client.call_async(client_request(node))
    spin_until(executor, second.done)
    assert second.result().error_code.val == MoveItErrorCodes.FAILURE
    spin_until(executor, first.done)
    assert first.result().error_code.val == MoveItErrorCodes.TIMED_OUT
    assert adapter.pending is None
    release.set_result(None)
    for _ in range(30):
        executor.spin_once(timeout_sec=.005)
    assert not published
    assert len(calls) == 1
    assert not adapter.backend._pending_requests
    third = client.call_async(client_request(node))
    spin_until(executor, third.done)
    assert third.result().error_code.val == MoveItErrorCodes.FAILURE
    assert len(calls) == 1


def test_proposal_only_static_contract():
    tree = ast.parse(MODULE.read_text())
    calls = {ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)}
    forbidden = {'ActionClient', 'sleep', 'lookup_transform', 'synchronize', 'cpu', 'item',
                 'numpy', 'wait_for_service', 'spin_until_future_complete'}
    assert not any(call.rsplit('.', 1)[-1] in forbidden for call in calls)
    assert not any(isinstance(node, ast.While) for node in ast.walk(tree))
    imports = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    imports |= {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import)
                for alias in node.names}
    assert not ({'torch', 'pyrealsense2', 'control_msgs.action'} & imports)
    launch = (MODULE.parents[1] / 'launch/intel_grasp.launch.py').read_text()
    assert 'condition=IfCondition' in launch and 'default_value="false"' in launch


def test_invalid_request_never_reaches_backend(ros):
    executor, add = ros
    adapter = make_adapter(add)
    backend = add(Node('synthetic_validation_backend'))
    calls = []
    def serve(request, response):
        calls.append(request)
        return response
    backend.create_service(GraspPlanning, '/synthetic/intel/plan', serve)
    spin_until(executor, adapter.backend.service_is_ready)
    node, client = make_client(add, executor)
    requests = []
    stale = client_request(node)
    stale.target.header.stamp.sec -= 2
    requests.append(stale)
    unframed = client_request(node)
    unframed.target.header.frame_id = ''
    requests.append(unframed)
    unsupported = client_request(node)
    unsupported.candidate_grasps = [grasp()]
    requests.append(unsupported)
    for request in requests:
        future = client.call_async(request)
        spin_until(executor, future.done)
        assert future.result().error_code.val == MoveItErrorCodes.INVALID_OBJECT_NAME
    assert not calls
    assert adapter.pending is None


@pytest.mark.parametrize('mode', ['error', 'stale', 'mixed'])
def test_rejected_backend_result_cannot_publish(ros, mode):
    executor, add = ros
    adapter = make_adapter(add)
    backend = add(Node('synthetic_bad_intel_backend'))
    def serve(request, response):
        value = grasp()
        value.grasp_pose.header.stamp = backend.get_clock().now().to_msg()
        if mode == 'stale':
            value.grasp_pose.header.stamp.sec -= 2
        response.grasps = [value]
        if mode == 'mixed':
            other = grasp()
            other.grasp_pose.header.stamp = backend.get_clock().now().to_msg()
            other.grasp_pose.header.stamp.nanosec -= 1
            response.grasps.append(other)
        response.error_code.val = MoveItErrorCodes.FAILURE if mode == 'error' else MoveItErrorCodes.SUCCESS
        return response
    backend.create_service(GraspPlanning, '/synthetic/intel/plan', serve)
    spin_until(executor, adapter.backend.service_is_ready)
    node, client = make_client(add, executor)
    published = []
    node.create_subscription(PoseArray, '/synthetic/station/intel/poses', published.append, 1)
    future = client.call_async(client_request(node))
    spin_until(executor, future.done)
    assert future.result().error_code.val == MoveItErrorCodes.FAILURE
    assert adapter.pending is None
    assert not published


def test_quick_action_dispatch(monkeypatch):
    from tools import ros_station_action, ros_intel_grasp
    args = ros_station_action.parse_args(['intel-grasp', '/tmp/source.json'])
    assert args.request_json == '/tmp/source.json'
    seen = []
    monkeypatch.setattr(ros_intel_grasp, 'main', lambda argv: seen.append(argv) or 0)
    assert ros_station_action.main(['intel-grasp', '/tmp/source.json']) == 0
    assert seen == [['/tmp/source.json']]


@pytest.mark.parametrize('change', [
    lambda g: setattr(g, 'id', 'x'*129),
    lambda g: setattr(g, 'allowed_touch_objects', ['x']*9),
    lambda g: setattr(g, 'max_contact_force', float('nan')),
    lambda g: setattr(g.pre_grasp_posture, 'joint_names', ['j']*9),
    lambda g: setattr(g.post_grasp_retreat.direction.vector, 'z', float('inf')),
])
def test_full_forwarded_message_bounds(change):
    value = grasp()
    change(value)
    with pytest.raises(ValueError):
        checked([value])


def test_posture_values_and_geometry_request_rejected(ros):
    from trajectory_msgs.msg import JointTrajectoryPoint
    from shape_msgs.msg import SolidPrimitive
    value = grasp()
    value.grasp_posture.joint_names = ['jaw']
    value.grasp_posture.points = [JointTrajectoryPoint(positions=[float('nan')])]
    with pytest.raises(ValueError):
        checked([value])
    value.grasp_posture.points = [JointTrajectoryPoint(positions=[1.])]*5
    with pytest.raises(ValueError):
        checked([value])
    _, add = ros
    node = add(Node('synthetic_id_only'))
    request = client_request(node)
    request.target.primitives = [SolidPrimitive()]
    assert not adapter_module.id_only_request(request, 'base_link', 'manipulator',
                                               node.get_clock().now().nanoseconds*1e-9, .5)


@pytest.mark.parametrize('phase', ['call', 'result', 'copy'])
def test_slow_admission_result_and_copy_cannot_publish(ros, monkeypatch, phase):
    from types import SimpleNamespace
    from rclpy.task import Future
    executor, add = ros
    adapter = make_adapter(add)
    clock = [10.]
    monkeypatch.setattr(adapter_module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    published = []
    monkeypatch.setattr(adapter.poses, 'publish', published.append)
    response = GraspPlanning.Response()
    value = grasp()
    value.grasp_pose.header.stamp = adapter.get_clock().now().to_msg()
    response.grasps = [value]
    response.error_code.val = MoveItErrorCodes.SUCCESS
    backend = Future()
    completion = Future()
    pending = {'completion': completion, 'future': backend, 'deadline': 10.05,
               'stamp': adapter_module.stamp_seconds(value.grasp_pose.header.stamp),
               'target_id': 'red-1'}
    adapter.pending = pending
    if phase == 'call':
        adapter.pending = None
        monkeypatch.setattr(adapter.backend, 'service_is_ready', lambda: True)
        def slow_call(request):
            clock[0] = 10.5
            return backend
        monkeypatch.setattr(adapter.backend, 'call_async', slow_call)
        completion = executor.create_task(adapter._request, client_request(adapter),
                                          GraspPlanning.Response())
        spin_until(executor, completion.done)
    else:
        if phase == 'result':
            def result():
                clock[0] = 10.1
                return response
            done = SimpleNamespace(result=result)
        else:
            done = SimpleNamespace(result=lambda: response)
            original = adapter_module.deepcopy
            def slow_copy(value):
                clock[0] = 10.1
                return original(value)
            monkeypatch.setattr(adapter_module, 'deepcopy', slow_copy)
        adapter._complete(pending, done)
    assert completion.done()
    assert completion.result().error_code.val == MoveItErrorCodes.TIMED_OUT
    assert not published
    assert adapter.backend_timeout_latched


def test_native_cli_to_adapter_to_fake_backend(ros, tmp_path):
    import json
    import os
    import subprocess
    import sys
    executor, add = ros
    adapter = make_adapter(add, timeout=.4, service='/station/intel_grasp/plan')
    backend = add(Node('synthetic_cli_intel_backend'))
    calls, returned, published = [], [], []
    def serve(request, response):
        calls.append(request)
        value = grasp()
        value.grasp_pose.header.stamp = backend.get_clock().now().to_msg()
        returned.append(value)
        response.grasps = [value]
        response.error_code.val = MoveItErrorCodes.SUCCESS
        return response
    backend.create_service(GraspPlanning, '/synthetic/intel/plan', serve)
    backend.create_subscription(PoseArray, '/synthetic/station/intel/poses', published.append, 1)
    spin_until(executor, adapter.backend.service_is_ready)
    source_stamp = backend.get_clock().now().to_msg()
    request_file = tmp_path/'source-request.json'
    request_file.write_text(json.dumps({'group_name': 'manipulator', 'target': {
        'header': {'frame_id': 'base_link', 'stamp': {'sec': source_stamp.sec,
                                                   'nanosec': source_stamp.nanosec}},
        'id': 'red-1'}}))
    child = subprocess.Popen([sys.executable, str(ROOT/'tools/ros_intel_grasp.py'),
                              str(request_file)], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        spin_until(executor, lambda: child.poll() is not None, timeout=4.)
        stdout, stderr = child.communicate(timeout=1.)
        assert child.returncode == 0, (stdout, stderr)
        result = json.loads(stdout)
        assert result['qualified'] is False
        assert result['execution_enabled'] is False
        assert result['pose_semantics'] == 'upstream_eef_parent'
        assert len(calls) == 1 and calls[0].target.header.stamp == source_stamp
        spin_until(executor, lambda: bool(published))
        assert published[0].header == returned[0].grasp_pose.header
        assert published[0].poses[0] == returned[0].grasp_pose.pose
        assert result['response']['grasps'][0]['grasp_pose']['header']['stamp'] == {
            'sec': returned[0].grasp_pose.header.stamp.sec,
            'nanosec': returned[0].grasp_pose.header.stamp.nanosec}
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=1.)


def test_all_statuses_disclaim_track_and_eef_association(ros, monkeypatch):
    import json
    _, add = ros
    adapter = make_adapter(add)
    messages = []
    monkeypatch.setattr(adapter.status, 'publish', messages.append)
    for state in ('disabled', 'backend_unavailable', 'proposals'):
        adapter._report(state, **({'request_target_id': 'red-1'} if state == 'proposals' else {}))
    for message in messages:
        data = json.loads(message.data)
        assert data['target_association_verified'] is False
        assert data['target_id_semantics'] == 'request provenance only; backend does not establish track association'
        assert data['eef_parent_link_verified'] is False
        assert data['tool_mapping_qualified'] is False
        assert 'target_id' not in data
    assert json.loads(messages[-1].data)['request_target_id'] == 'red-1'


@pytest.mark.parametrize('domain', ['0', '-1', '233'])
def test_direct_entrypoint_rejects_invalid_domain_before_ros_init(monkeypatch, domain):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    calls = []
    monkeypatch.setattr(adapter_module.rclpy, 'init', lambda **kwargs: calls.append('init'))
    with pytest.raises(ValueError, match='ROS_DOMAIN_ID'):
        adapter_module.main()
    assert not calls


def test_direct_entrypoint_defaults_and_constructor_failure_cleanup(monkeypatch):
    import os
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    calls = []
    def init(**kwargs):
        assert os.environ['ROS_DOMAIN_ID'] == '76'
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        calls.append('init')
    def fail():
        raise ValueError('synthetic constructor failure')
    monkeypatch.setattr(adapter_module.rclpy, 'init', init)
    monkeypatch.setattr(adapter_module.rclpy, 'shutdown', lambda: calls.append('shutdown'))
    monkeypatch.setattr(adapter_module, 'IntelGraspAdapter', fail)
    with pytest.raises(ValueError, match='constructor failure'):
        adapter_module.main()
    assert calls == ['init', 'shutdown']
