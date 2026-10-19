"""Explicit reset isolation: CPU fake worker; no GPU/device or live ROS calls."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
import time
import sys

import pytest

pytest.importorskip('rclpy')
from test_ros_rocm_rgbd import module, FakeWorker, config
from std_srvs.srv import Trigger


class ResetWorker(FakeWorker):
    instances = []
    def __init__(self, configuration, python, library, *, enabled):
        super().__init__(configuration, python, library, enabled=enabled)
        self.process = NS(returncode=None)
        self.provenance['worker_python'] = str(Path(python).resolve())
        self.provenance['compatibility_sha256'] = hashlib.sha256(Path(library).read_bytes()).hexdigest()
        self.__class__.instances.append(self)
    def close(self):
        super().close(); self.process.returncode = 0


@pytest.fixture
def reset_node(tmp_path):
    configuration = tmp_path/'oracle.json'; configuration.write_text(json.dumps(config()))
    library = tmp_path/'cpu-fixture.so'; library.write_bytes(b'CPUfixtureNotELF')
    ResetWorker.instances = []
    module.rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'allow_explicit_reset:=true',
        '-p', 'configuration_file:='+str(configuration), '-p', 'compatibility_library:='+str(library),
        '-p', 'worker_python:='+sys.executable])
    node = module.ROCmRGBD(worker_factory=ResetWorker)
    endpoint = NS(node_namespace='/', node_name='camera', endpoint_gid=bytes(range(16)))
    node.get_publishers_info_by_topic = lambda topic: [endpoint]
    try: yield node, configuration, library
    finally:
        node.destroy_node(); module.rclpy.shutdown()


def request(node):
    return node._reset_source(Trigger.Request(), Trigger.Response())


def test_explicit_reset_rotates_generation_retires_before_new_worker_and_revokes_old(reset_node):
    node, _, _ = reset_node
    old = node.worker; generation = node.generation
    future_events = []
    future = NS(cancel=lambda: future_events.append('cancel'))
    client = NS(remove_pending_request=lambda f: future_events.append('remove'))
    node.device_client = node.parameter_client = client
    node.pending_identity = {'futures': [future, future]}
    node.destroy_client = lambda client: None
    node.last_header = {'depth': 123}; node.last_receipt = {'depth': time.monotonic()}
    node.last_result = {'producer_generation': generation}
    response = request(node); payload = json.loads(response.message)
    assert response.success and payload['accepted'] is True and payload['ready'] is False
    assert payload['accepted_reset_id'] and payload['new_producer_generation'] != generation
    assert node.invalid and node.identity is None
    assert old.closed is False and len(ResetWorker.instances) == 1
    assert future_events == ['remove', 'cancel', 'remove', 'cancel']
    factory = node._worker_factory
    def retired_first(*args,**kwargs):
        assert old.closed and old.process.returncode == 0
        return factory(*args,**kwargs)
    node._worker_factory = retired_first
    node._perform_reset()
    assert old.closed and old.process.returncode == 0
    assert node.worker is not old and len(ResetWorker.instances) == 2
    assert node.generation == payload['new_producer_generation']
    assert node.invalid is None and node.pending_identity is None
    assert node.last_header == node.last_receipt == {} and node.last_result is None
    assert node.binding is node.client_binding is node.output_binding is node.ledger is None
    assert all(not b for b in node.buffers.values())
    # Reset status cannot report readiness from a stale pre-reset result.
    assert node._reset_status()['ready'] is False


def test_disabled_busy_cooldown_and_missing_graph_never_retire_worker(reset_node):
    node, _, _ = reset_node; old = node.worker
    node.allow_explicit_reset = False
    assert not request(node).success and not old.closed
    node.allow_explicit_reset = True
    node.get_publishers_info_by_topic = lambda topic: []
    assert not request(node).success and not old.closed
    node.get_publishers_info_by_topic = lambda topic: [NS(),NS()]
    assert not request(node).success and not old.closed
    node.get_publishers_info_by_topic = lambda topic: [NS(node_namespace='/',node_name='camera',endpoint_gid=bytes(range(16)))]
    assert request(node).success
    assert not request(node).success and not old.closed
    node._perform_reset()
    assert not request(node).success
    assert len(ResetWorker.instances) == 2


@pytest.mark.parametrize('failure', ['exception', 'unknown_returncode'])
def test_cleanup_failure_permanently_blocks_replacement_and_retry(reset_node, failure):
    node, _, _ = reset_node; old = node.worker
    if failure == 'exception':
        def close(): raise TimeoutError('owned child not known retired')
        old.close = close
    else: old.close = lambda: None
    assert request(node).success
    node._perform_reset()
    assert len(ResetWorker.instances) == 1
    assert node._reset_status()['state'] == 'reset_failed'
    assert not request(node).success
    # Test teardown only: restore the fake's normal cleanup, never a real PID.
    old.close = lambda: ResetWorker.close(old)


@pytest.mark.parametrize('target', ['configuration', 'library', 'source', 'node', 'python'])
def test_changed_startup_pins_rejected_before_worker_touch(reset_node, monkeypatch, target):
    node, configuration, library = reset_node
    if target == 'source': monkeypatch.setattr(module, 'source_hashes', lambda: {'bad': 'f'*64})
    elif target in ('node','python'):
        actual = Path.read_bytes
        monkeypatch.setattr(Path, 'read_bytes', lambda p: b'changed' if p==node._reset_paths[target] else actual(p))
    else: (configuration if target == 'configuration' else library).write_bytes(b'changed')
    assert not request(node).success
    assert not node.worker.closed and len(ResetWorker.instances) == 1


def test_changes_between_acceptance_and_deferred_job_fail_closed(reset_node):
    node, _, library = reset_node
    assert request(node).success
    library.write_bytes(b'changed after acceptance')
    node._perform_reset()
    assert node._reset_status()['state'] == 'reset_failed'
    assert len(ResetWorker.instances) == 1 and not node.worker.closed


def test_reset_budget_exhausted_after_constructor_retires_replacement(reset_node, monkeypatch):
    node, _, _ = reset_node
    assert request(node).success
    clock = [0.]; node._reset_deadline = 2.; actual = module.time.monotonic
    def factory(*args, **kwargs):
        replacement = ResetWorker(*args, **kwargs)
        clock[0] = 2.1
        return replacement
    node._worker_factory = factory
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node._perform_reset()
    monkeypatch.setattr(module.time, 'monotonic', actual)
    assert node._reset_status()['state'] == 'reset_failed'
    assert len(ResetWorker.instances) == 2
    assert node.worker.closed and node.worker.process.returncode == 0
    assert not request(node).success


def test_cold_worker_timeout_is_permanent_not_a_blind_retry(reset_node):
    node, _, _ = reset_node
    assert request(node).success; node._perform_reset()
    node.worker.ready = False
    def expired(): raise TimeoutError('worker startup/job deadline expired')
    node.worker.poll = expired
    node._tick()
    assert node._reset_status()['state'] == 'reset_failed'
    node._last_reset -= 31.
    assert not request(node).success


def test_old_result_and_cancelled_rpc_cannot_publish_or_repopulate(reset_node):
    node, _, _ = reset_node
    old_generation = node.generation
    assert request(node).success; node._perform_reset()
    before = node.counters['published']
    # A retired worker reply cannot survive the changed producer generation,
    # even if the camera source generation and output GIDs are unchanged.
    node.identity = {'request_started_monotonic':time.monotonic()}
    node.first_source = 'same-camera-source'
    result = {'metadata': {'source_generation':'same-camera-source',
                          'producer_generation':old_generation}}
    node._publish(result)
    assert node.counters['published'] == before
    assert node.pending_identity is None


def test_reset_static_no_new_image_tick_io_and_no_external_process_control():
    import ast
    tree = ast.parse(Path(module.__file__).read_text())
    functions = {n.name:n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    for name in ('_receive', '_admit', '_publish', '_tick', '_reset_status'):
        calls = [ast.unparse(n.func) for n in ast.walk(functions[name]) if isinstance(n, ast.Call)]
        assert not any(c.endswith(('.read_bytes','.open','.sha256','.wait','.sleep','.synchronize')) for c in calls)
    reset = '\n'.join(ast.unparse(functions[n]) for n in ('_reset_source','_perform_reset','_cancel_source_rpc'))
    for forbidden in ('os.kill', 'subprocess', 'signal.', 'pyrealsense2', 'importlib', 'create_subscription'):
        assert forbidden not in reset
    launch = Path(module.__file__).parents[1]/'launch/rocm_rgbd.launch.py'
    assert "DeclareLaunchArgument('allow_explicit_reset', default_value='false')" in launch.read_text()


def test_native_dds_reset_service_new_generation_requires_fresh_lineages(tmp_path):
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from realsense2_camera_msgs.srv import DeviceInfo
    configuration = tmp_path/'oracle.json'; configuration.write_text(json.dumps(config()))
    library = tmp_path/'fixture.so'; library.write_bytes(b'CPUfake')
    ResetWorker.instances = []
    module.rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'allow_explicit_reset:=true',
        '-p', 'configuration_file:='+str(configuration), '-p', 'compatibility_library:='+str(library),
        '-p', 'worker_python:='+sys.executable])
    camera = Node('camera'); camera.declare_parameter('serial_no', '_'+module.SERIAL)
    def identity(req, response): response.serial_number = module.SERIAL; return response
    camera.create_service(DeviceInfo, '/camera/device_info', identity)
    publishers = {k:camera.create_publisher(CameraInfo if k.endswith('info') else Image, v, qos_profile_sensor_data)
                  for k,v in module.TOPICS.items()}
    broadcaster = StaticTransformBroadcaster(camera)
    transform = TransformStamped(); transform.header.frame_id = 'camera_color_optical_frame'
    transform.child_frame_id = 'camera_depth_optical_frame'; transform.transform.rotation.w = 1.
    transform.transform.translation.x = .02; broadcaster.sendTransform(transform)
    node = module.ROCmRGBD(worker_factory=ResetWorker); probe = Node('reset_probe')
    statements = []
    probe.create_subscription(String, '/so101_perception/rocm/lineage',
        lambda msg:statements.append(json.loads(msg.data)), qos_profile_sensor_data)
    client = probe.create_client(Trigger, '/so101_rocm_rgbd/reset_source')
    executor = SingleThreadedExecutor()
    for obj in (camera,node,probe): executor.add_node(obj)
    def emit():
        stamp = camera.get_clock().now().nanoseconds
        for stream,publisher in publishers.items():
            depth = stream.startswith('depth'); info = stream.endswith('info')
            msg = CameraInfo() if info else Image()
            msg.header.stamp.sec,msg.header.stamp.nanosec = divmod(stamp,1_000_000_000)
            msg.header.frame_id = 'camera_depth_optical_frame' if depth else 'camera_color_optical_frame'
            msg.width,msg.height = (4,3) if depth else (6,4)
            if info:
                msg.distortion_model = 'plumb_bob'; msg.d = [0.]*5
                msg.k = [4.,0.,2. if depth else 3.,0.,4.,1. if depth else 2.,0.,0.,1.]
            else:
                msg.encoding = '16UC1' if depth else 'rgb8';msg.step = msg.width*(2 if depth else 3)
                msg.data = bytes(msg.step*msg.height)
            publisher.publish(msg)
    timer = camera.create_timer(.025,emit)
    def wait(predicate, seconds=3.):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline and not predicate(): executor.spin_once(timeout_sec=.005)
        assert predicate(), (node.invalid,node.preparing_reason,node._reset_status())
    try:
        wait(lambda:len(statements)>=3)
        old = node.worker; old_generation = node.generation
        old_source = statements[-1]['source_generation']; old_outputs = statements[-1]['outputs']
        future = client.call_async(Trigger.Request()); wait(future.done)
        response = future.result(); payload = json.loads(response.message)
        assert response.success and payload['ready'] is False
        newgen = payload['new_producer_generation']; assert newgen != old_generation
        wait(lambda:len([v for v in statements if v['producer_generation']==newgen])>=2)
        new = [v for v in statements if v['producer_generation']==newgen]
        assert old.closed and old.process.returncode == 0 and len(ResetWorker.instances)==2
        assert node._reset_status()['ready'] is True
        assert node._reset_status()['accepted_reset_id']==payload['accepted_reset_id']
        assert all(v['source_generation']==old_source for v in new)
        assert new[0]['outputs']['cloud_publisher_gid']==old_outputs['cloud_publisher_gid']
        assert new[0]['outputs']['aligned_publisher_gid']==old_outputs['aligned_publisher_gid']
        for a,b in zip(new,new[1:]):
            assert b['original']['depth_stamp_ns']>a['original']['depth_stamp_ns']
            assert b['original']['color_stamp_ns']>a['original']['color_stamp_ns']
        # Actual ROS Trigger cooldown response, without constructing another worker.
        future = client.call_async(Trigger.Request()); wait(future.done)
        assert future.result().success is False and len(ResetWorker.instances)==2
    finally:
        camera.destroy_timer(timer); executor.shutdown()
        node.destroy_node();probe.destroy_node();camera.destroy_node();module.rclpy.shutdown()


def test_default_off_is_immutable_and_does_not_query_or_touch_worker(tmp_path):
    from rclpy.parameter import Parameter
    path = tmp_path/'oracle.json';path.write_text(json.dumps(config()))
    module.rclpy.init(args=['--ros-args','-p','enabled:=true','-p','configuration_file:='+str(path)])
    node = module.ROCmRGBD(worker_factory=FakeWorker)
    def forbidden(*args): pytest.fail('disabled reset must not inspect/touch resources')
    node._check_reset_inputs = forbidden
    node.get_publishers_info_by_topic = forbidden
    node.worker.close = forbidden
    try:
        assert node.allow_explicit_reset is False
        assert not request(node).success
        assert node.set_parameters([Parameter('allow_explicit_reset',value=True)])[0].successful is False
    finally:
        node.worker.close = lambda: None
        node.destroy_node();module.rclpy.shutdown()


def test_replacement_cleanup_unknown_preserves_handle_and_stays_failed(reset_node,monkeypatch):
    node,_,_ = reset_node;assert request(node).success
    clock=[0.];node._reset_deadline=2.;actual=module.time.monotonic
    def factory(*args,**kwargs):
        new = ResetWorker(*args,**kwargs)
        new.close = lambda: None
        clock[0]=2.1
        return new
    node._worker_factory=factory
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node._perform_reset()
    monkeypatch.setattr(module.time,'monotonic',actual)
    assert node.worker is ResetWorker.instances[-1] and node.worker.process.returncode is None
    assert node._reset_status()['state']=='reset_failed'
    assert 'cleanup unknown' in node._reset_status()['reason']
    assert not request(node).success
    node.worker.close=lambda:ResetWorker.close(node.worker)


def test_retirement_elapsed_timeout_prevents_any_replacement(reset_node, monkeypatch):
    node,_,_=reset_node;assert request(node).success
    clock=[0.];node._reset_deadline=2.;actual=module.time.monotonic;old=node.worker
    def slow_retire():
        ResetWorker.close(old);clock[0]=2.1
    old.close=slow_retire
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node._perform_reset()
    monkeypatch.setattr(module.time,'monotonic',actual)
    assert len(ResetWorker.instances)==1
    assert node._reset_status()['state']=='reset_failed'


def test_cohort_readiness_needs_two_publications_and_current_original_health(reset_node):
    node,_,_=reset_node;assert request(node).success;node._perform_reset()
    now=node.get_clock().now().nanoseconds;mono=time.monotonic()
    node.identity={'request_started_monotonic':mono}
    node.last_result=dict(producer_generation=node.generation, identity={'request_started_monotonic':mono},
        source_received_monotonic={k:mono for k in module.TOPICS},
        **{k:now-1_000_000 for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')})
    node.counters['published']=node._reset['published_at_start']+1
    assert node._reset_status()['ready'] is False
    node.counters['published']+=1
    assert node._reset_status()['ready'] is True
    node.last_result['identity']['request_started_monotonic']=mono-1.01
    assert node._reset_status()['ready'] is False
    node.last_result['identity']['request_started_monotonic']=mono
    node.last_result['color_info_stamp_ns']=now-600_000_000
    assert node._reset_status()['ready'] is False
    node.last_result['color_info_stamp_ns']=now-1_000_000
    node.last_result['source_received_monotonic']['depth']=mono-.251
    assert node._reset_status()['ready'] is False


@pytest.mark.parametrize('stage', ['preaccept_hash','preaccept_graph','queue','retire_hash','construct_hash'])
def test_one_request_deadline_blocks_resource_mutation_after_expiry(reset_node,monkeypatch,stage):
    node,_,_=reset_node;clock=[100.];actual=module.time.monotonic
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    original=node._check_reset_inputs;calls=[0]
    def slow_check(*args,**kwargs):
        calls[0]+=1
        result=original(*args,**kwargs)
        if ((stage=='preaccept_hash' and calls[0]==1)
                or (stage=='retire_hash' and calls[0]==2)
                or (stage=='construct_hash' and calls[0]==3)):
            clock[0]+=2.01
        return result
    node._check_reset_inputs=slow_check
    graph_calls=[0]
    if stage=='preaccept_graph':
        original_graph=node.get_publishers_info_by_topic
        def slow_graph(topic):
            graph_calls[0]+=1
            result=original_graph(topic);clock[0]+=2.01;return result
        node.get_publishers_info_by_topic=slow_graph
    response=request(node)
    if stage in ('preaccept_hash','preaccept_graph'):
        assert response.success is False
        assert not node.worker.closed and len(ResetWorker.instances)==1
        if stage=='preaccept_graph':assert graph_calls[0]==1
    else:
        assert response.success
        if stage=='queue':clock[0]+=2.01
        node._perform_reset()
        assert node._reset_status()['state']=='reset_failed'
        assert len(ResetWorker.instances)==1
        assert node.worker.closed is (stage=='construct_hash')
    monkeypatch.setattr(module.time,'monotonic',actual)


def test_late_timer_creation_is_cancelled_without_launching_worker(reset_node,monkeypatch):
    node,_,_=reset_node;clock=[100.];actual_clock=module.time.monotonic
    create=node.create_timer
    def late_timer(*args,**kwargs):
        timer=create(*args,**kwargs);clock[0]+=2.01;return timer
    node.create_timer=late_timer
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    response=request(node)
    monkeypatch.setattr(module.time,'monotonic',actual_clock)
    assert response.success is False
    assert node._reset_timer is None and node._reset_status()['state']=='reset_failed'
    node._perform_reset()
    assert not node.worker.closed and len(ResetWorker.instances)==1
