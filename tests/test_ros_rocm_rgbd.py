"""Native ROS metadata/scheduling tests use an explicit CPU fake, never Torch."""
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from realsense2_camera_msgs.srv import DeviceInfo

ROOT = Path(__file__).resolve().parents[1]
path = ROOT/'ros2/src/so101_station_perception/so101_station_perception/rocm_rgbd.py'
spec = importlib.util.spec_from_file_location('reviewed_rocm_rgbd', path)
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
from workshop.perception.rocm_transport import LatestSlots, pack_cloud, source_hashes


def config():
    return dict(schema='realsense-cpu-projection-oracle-v1', qualified=False, align_name='Align',
        pointcloud_name='Pointcloud', occlusion_option=2., depth_scale_m=.001,
        depth_size=[4, 3], color_size=[6, 4], depth_K=[4., 4., 2., 1.], color_K=[4., 4., 3., 2.],
        rotation_column_major=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation_m=[.02, 0., 0.])


@pytest.mark.parametrize('value', [None, {}, {'publisher_gid': b''}, {'publisher_gid': bytes(16)},
                                  {'publisher_gid': bytes([1])*17}, {'publisher_gid': bytes([1])*24}])
def test_missing_or_wrong_abi_gid_rejected(value):
    with pytest.raises(ValueError): module.publisher_gid(value)


def test_gid_zero_padding_only_and_source_unique_graph_binding():
    raw = bytes(range(16))
    assert module.publisher_gid({'publisher_gid': raw}) == module.publisher_gid({'publisher_gid': raw+bytes(8)})
    endpoint = NS(node_namespace='/', node_name='camera', endpoint_gid=raw)
    fake = NS(get_publishers_info_by_topic=lambda topic: [endpoint])
    result = module.source_snapshot(fake)
    assert result['node'] == '/camera' and set(result['endpoints']) == set(module.TOPICS)
    fake.get_publishers_info_by_topic = lambda topic: [endpoint, endpoint]
    with pytest.raises(ValueError): module.source_snapshot(fake)


@pytest.mark.parametrize('change', [dict(width=5), dict(step=9), dict(encoding='32FC1'),
                                   dict(is_bigendian=1), dict(data=b'1')])
def test_image_storage_profile_rejects_before_array_view(change):
    image = NS(width=4, height=3, step=8, encoding='16UC1', is_bigendian=0, data=bytes(24))
    for key, value in change.items(): setattr(image, key, value)
    with pytest.raises(ValueError): module.image_array(image, (4, 3), '16UC1')


class FakeWorker:
    """CPU protocol fixture: deliberately not a GPU or geometry parity proof."""
    def __init__(self, configuration, python, library, *, enabled):
        assert enabled is True
        self.ready = True; self.closed = False; self.scheduler = LatestSlots(); self.data = {}
        self.hardware = {'test_only_cpu_protocol_fixture': True}
        self.provenance = dict(source_sha256=source_hashes(), worker_python=sys.executable,
            worker_python_sha256=hashlib.sha256(Path(sys.executable).resolve().read_bytes()).hexdigest(), compatibility_sha256='c'*64)
    def offer(self, depth, rgb, metadata):
        item = self.scheduler.offer(metadata); self.data[item['slot']] = depth.copy()
    def poll(self):
        job = self.scheduler.begin()
        if job is None: return None
        depth = self.data[job['slot']]
        xyz = np.zeros((*depth.shape, 3), np.float32); xyz[..., 2] = depth*.001
        packed = np.full(depth.shape, 0x112233, np.uint32)
        cloud = pack_cloud(xyz, packed, (depth > 0).astype(np.uint8))
        self.scheduler.complete(job)
        return dict(job, kind='result', qualified=False, cloud_bytes=len(cloud), cloud=cloud,
                    aligned=np.full((4, 6), 123, np.uint16).tobytes(), backend_ms=.1, worker_ms=.2)
    def close(self): self.closed = True; self.scheduler.invalidate()


def test_native_derived_source_original_stamps_dedup_and_replacement(tmp_path):
    configuration = tmp_path/'configuration.json'; configuration.write_text(json.dumps(config()))
    rclpy.init(args=['--ros-args', '-p', 'enabled:=true', '-p', 'configuration_file:='+str(configuration)])
    camera = Node('camera'); camera.declare_parameter('serial_no', '_'+module.SERIAL)
    def identity(request, response): response.serial_number = module.SERIAL; return response
    service = camera.create_service(DeviceInfo, '/camera/device_info', identity)
    publishers = {name: camera.create_publisher(CameraInfo if name.endswith('info') else Image, topic, qos_profile_sensor_data)
                  for name, topic in module.TOPICS.items()}
    broadcaster = StaticTransformBroadcaster(camera)
    transform = TransformStamped(); transform.header.frame_id = 'camera_color_optical_frame'
    transform.child_frame_id = 'camera_depth_optical_frame'; transform.transform.rotation.w = 1.
    transform.transform.translation.x = .02; broadcaster.sendTransform(transform)
    node = module.ROCmRGBD(worker_factory=FakeWorker); probe = Node('rocm_probe')
    clouds = []; lineages = []; cloud_gids = []; lineage_gids = []; aligned_gids = []
    def cloud_received(message, info): clouds.append(message); cloud_gids.append(module.publisher_gid(info))
    def lineage_received(message, info): lineages.append(json.loads(message.data)); lineage_gids.append(module.publisher_gid(info))
    def aligned_received(message, info): aligned_gids.append(module.publisher_gid(info))
    probe.create_subscription(PointCloud2, '/so101_perception/rocm/points', cloud_received, qos_profile_sensor_data)
    probe.create_subscription(String, '/so101_perception/rocm/lineage', lineage_received, qos_profile_sensor_data)
    probe.create_subscription(Image, '/so101_perception/rocm/aligned_depth', aligned_received, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(); executor.add_node(camera); executor.add_node(node); executor.add_node(probe)
    tick = [0]; last_stamp = [None]
    def emit():
        tick[0] += 1
        # Every second frameset repeats BOTH acquisitions; not independent data.
        if tick[0] % 2 or last_stamp[0] is None: last_stamp[0] = camera.get_clock().now().nanoseconds
        for label, publisher in publishers.items():
            depth = label.startswith('depth'); width, height = (4, 3) if depth else (6, 4)
            message = CameraInfo() if label.endswith('info') else Image()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(last_stamp[0], 1_000_000_000)
            message.header.frame_id = 'camera_depth_optical_frame' if depth else 'camera_color_optical_frame'
            message.width = width; message.height = height
            if label.endswith('info'):
                message.distortion_model = 'plumb_bob'; message.d = [0.]*5
                message.k = [4., 0., 2. if depth else 3., 0., 4., 1. if depth else 2., 0., 0., 1.]
            else:
                message.encoding = '16UC1' if depth else 'rgb8'; message.step = width*(2 if depth else 3)
                message.data = np.full((height, width), 500, np.uint16).tobytes() if depth else bytes(width*height*3)
            publisher.publish(message)
    timer = camera.create_timer(.025, emit)
    try:
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline and len(lineages) < 3:
            executor.spin_once(timeout_sec=.005)
        assert len(lineages) >= 3, (node.invalid, node.preparing_reason, node.received, node.counters)
        if os.environ.get('SO101_ROCM_TEST_EVIDENCE'):
            chosen = lineages[0]
            snapshot = chosen['identity']['device_info_request_source']
            canonical = json.dumps(snapshot, sort_keys=True)
            evidence = dict(schema='rocm-lineage-native-CPU-contract-fixture-v1',
                physical_camera_used=False, gpu_executed=False, qualified=False,
                source_snapshot=snapshot, source_canonical_utf8=canonical,
                source_generation_recomputed=hashlib.sha256(canonical.encode()).hexdigest(),
                configuration_utf8=configuration.read_text(),
                configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
                observed_cloud_publisher_gid=cloud_gids[0], observed_status_publisher_gid=lineage_gids[0],
                observed_aligned_publisher_gid=aligned_gids[0],
                lineage=chosen)
            Path(os.environ['SO101_ROCM_TEST_EVIDENCE']).write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')
        assert len({x['original']['depth_stamp_ns'] for x in lineages}) == len(lineages)
        assert len({x['original']['color_stamp_ns'] for x in lineages}) == len(lineages)
        for lineage in lineages:
            assert lineage['schema'] == 'so101-derived-rocm-rgbd-cohort-v1' and lineage['qualified'] is False
            assert lineage['source']['serial_number'] == module.SERIAL
            assert lineage['outputs']['cloud_publisher_gid'] == cloud_gids[0]
            assert lineage['outputs']['status_publisher_gid'] == lineage_gids[0]
            assert lineage['outputs']['aligned_publisher_gid'] == aligned_gids[0]
            assert lineage['outputs']['aligned_topic'] == '/so101_perception/rocm/aligned_depth'
            cloud = next(c for c in clouds if module.stamp_ns(c) == lineage['original']['depth_stamp_ns'])
            assert cloud.row_step == cloud.width*20 == len(cloud.data)
            assert cloud.header.frame_id == 'camera_depth_optical_frame'
            assert lineage['outputs']['cloud_stamp_ns'] == lineage['original']['depth_stamp_ns']
            assert lineage['original']['pair_delta_ns'] == 0
        # Stalling final materialization cannot mint fresh acquisition authority.
        metadata = deepcopy(node.last_result)
        result = dict(metadata=metadata, sequence=100, aligned=bytes(48), cloud=bytes(20),
                      cloud_bytes=20, backend_ms=.1, worker_ms=.2)
        before = node.counters['published']; actual_clock = node.get_clock
        now = actual_clock().now().nanoseconds; clocks = iter((now, now, now+600_000_000))
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=next(clocks)))
        try: node._publish(result)
        finally: node.get_clock = actual_clock
        assert node.counters['published'] == before
        # Info stamps have the same strict future/age gates as image stamps,
        # even when a future Info lies within the40ms pairing tolerance.
        for key in ('depth_info_stamp_ns', 'color_info_stamp_ns'):
            future = deepcopy(metadata); future[key] = actual_clock().now().nanoseconds+1_000_000
            node._publish(dict(result, metadata=future))
            assert node.counters['published'] == before
        # A future Info within40ms of otherwise fresh image exposures must fail
        # admission too, before the fixed input slot can be offered.
        saved_buffers = {key: list(value) for key, value in node.buffers.items()}
        try:
            fresh_now = actual_clock().now().nanoseconds
            for stream in module.TOPICS:
                previous = node.buffers[stream][-1]
                message = deepcopy(previous[2]); original_ns = fresh_now+(20_000_000 if stream.endswith('info') else 0)
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(original_ns, 1_000_000_000)
                node.buffers[stream].clear(); node.buffers[stream].append((original_ns, previous[1], message, previous[3]))
            with pytest.raises(ValueError, match='CameraInfo exposure'): node._admit()
        finally:
            for key, values in saved_buffers.items(): node.buffers[key] = deque(values, maxlen=4)
        # Real DDS publisher replacement, same node name, fresh headers: latch.
        camera.destroy_publisher(publishers['depth'])
        publishers['depth'] = camera.create_publisher(Image, module.TOPICS['depth'], qos_profile_sensor_data)
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline and not node.invalid: executor.spin_once(timeout_sec=.005)
        assert node.invalid is not None
        count = node.counters['published']; deadline = time.monotonic()+.1
        while time.monotonic() < deadline: executor.spin_once(timeout_sec=.005)
        assert node.counters['published'] == count
    finally:
        camera.destroy_timer(timer); executor.shutdown(); node.destroy_node(); probe.destroy_node(); camera.destroy_node(); rclpy.shutdown()


def test_static_no_control_resources_or_callback_filework():
    import ast
    tree = ast.parse(path.read_text())
    for name in ('_info', '_receive', '_admit', '_publish', '_tick'):
        function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
        calls = [ast.unparse(n.func) for n in ast.walk(function) if isinstance(n, ast.Call)]
        assert not any(x.endswith(('.read_bytes', '.open', '.sha256', '.wait', '.sleep', '.synchronize', '.item')) for x in calls)
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom)) and any(x in ast.unparse(n) for x in ('torch', 'triton', 'pyrealsense2', 'serial', 'control_msgs')) for n in ast.walk(tree))
    launch = ROOT/'ros2/src/so101_station_perception/launch/rocm_rgbd.launch.py'
    assert "default_value='false'" in launch.read_text() and "condition=IfCondition" in launch.read_text()


@pytest.mark.parametrize('domain', ['0', '-1', '233', 'wrong'])
def test_direct_entry_rejects_domain_before_ros_init(monkeypatch, domain):
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: pytest.fail('must reject before init'))
    with pytest.raises(ValueError): module.main()


def test_constructor_failure_always_shuts_down_context(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '179'); events = []
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: events.append('init'))
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('shutdown'))
    def fail(): raise ValueError('fixture constructor failure')
    monkeypatch.setattr(module, 'ROCmRGBD', fail)
    with pytest.raises(ValueError): module.main()
    assert events == ['init', 'shutdown']


@pytest.mark.parametrize('shape,model', [([], 'plumb_bob'), ([0.]*5, 'none')])
def test_unproven_distortion_shape_not_reported_as_five_zero_coefficients(shape, model):
    info = CameraInfo(); info.header.frame_id = 'camera_depth_optical_frame'; info.width = 4; info.height = 3
    info.k = [4., 0., 2., 0., 4., 1., 0., 0., 1.]; info.d = shape; info.distortion_model = model
    fake = NS(buffers={'depth_info': [(1_000_000_000, 'a'*32, info, time.monotonic())]},
        binding={'endpoints': {'depth_info': {'gid': 'a'*32}}})
    with pytest.raises(ValueError): module.ROCmRGBD._info(fake, 'depth_info', 1_000_000_000, (4, 3), [4., 4., 2., 1.], info.header.frame_id)


def delayed_info_fixture():
    """Replay native metadata from the first fault; no ROS node/device/worker."""
    now = 1_792_440_445_900_963_893
    image_ns = 1_792_440_445_854_934_814
    offers = []; records = []
    fake = NS(buffers={}, binding={'endpoints': {}}, identity={
        'request_started_monotonic': time.monotonic(), 'confirmed_ros_ns': now-200_000_000},
        worker=NS(ready=True, scheduler=NS(pending=None), offer=lambda d, rgb, m: offers.append(m)),
        ledger=NS(admit=lambda metadata, current: records.append((metadata, current)) or True),
        output_binding={'test': True}, calibration=module.calibration_from_oracle(config()),
        first_source='recorded-source', generation='test-generation', configuration_sha='a'*64,
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=now)),
        last_receipt={k: time.monotonic()-.05 for k in module.TOPICS},
        counters=dict(stale=0, awaiting_identity=0, duplicate=0, superseded=0, admitted=0))
    for stream in module.TOPICS:
        depth = stream.startswith('depth'); info = stream.endswith('info')
        message = CameraInfo() if info else Image()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(image_ns, 1_000_000_000)
        message.header.frame_id = 'camera_depth_optical_frame' if depth else 'camera_color_optical_frame'
        message.width, message.height = (4, 3) if depth else (6, 4)
        if info:
            message.distortion_model = 'plumb_bob'; message.d = [0.]*5
            message.k = [4., 0., 2. if depth else 3., 0., 4., 1. if depth else 2., 0., 0., 1.]
        else:
            message.encoding = '16UC1' if depth else 'rgb8'; message.step = message.width*(2 if depth else 3)
            message.data = bytes(message.step*message.height)
        gid = ('%02x' % (list(module.TOPICS).index(stream)+1))*16
        fake.binding['endpoints'][stream] = {'gid': gid}
        fake.buffers[stream] = deque([(image_ns, gid, message, fake.last_receipt[stream])], maxlen=4)
    fake._info = lambda *args: module.ROCmRGBD._info(fake, *args)
    return fake, offers, records, now, image_ns


def set_replay_stamp(fake, stream, stamp):
    _, gid, message, receipt = fake.buffers[stream][-1]; message = deepcopy(message)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    fake.buffers[stream].append((stamp, gid, message, receipt))


def test_recorded_100ms_delayed_info_waits_then_admits_original_cohort():
    fake, offers, records, now, image_ns = delayed_info_fixture()
    original_receipts = fake.last_receipt.copy()
    fake.buffers['color_info'].clear()
    # Exact colorInfo lag at the20:07:25.900UTC first fault.
    info_ns = image_ns-100_065_918
    # Recover the message from the independent fresh fixture, preserving storage.
    message = delayed_info_fixture()[0].buffers['color_info'][0][2]
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(info_ns, 1_000_000_000)
    fake.buffers['color_info'].append((info_ns, fake.binding['endpoints']['color_info']['gid'], message, fake.last_receipt['color_info']))
    module.ROCmRGBD._admit(fake)
    assert offers == records == [] and fake.counters['admitted'] == 0
    # A distinct actual Info exposure within the unchanged40ms window arrives.
    set_replay_stamp(fake, 'color_info', image_ns-1_000_000)
    module.ROCmRGBD._admit(fake)
    assert len(offers) == len(records) == 1
    assert offers[0]['color_stamp_ns'] == image_ns
    assert offers[0]['color_info_stamp_ns'] == image_ns-1_000_000
    assert offers[0]['source_received_monotonic'] == original_receipts
    assert fake.last_receipt == original_receipts


@pytest.mark.parametrize('mutation', ['K', 'D', 'profile', 'frame', 'GID'])
def test_delayed_info_does_not_hide_real_immutable_mutation(mutation):
    fake, offers, _, now, image_ns = delayed_info_fixture()
    set_replay_stamp(fake, 'color_info', image_ns-100_065_918)
    stamp, gid, info, receipt = fake.buffers['color_info'][-1]
    if mutation == 'K': info.k[0] += 1.
    elif mutation == 'D': info.d[0] = .01
    elif mutation == 'profile': info.width += 1
    elif mutation == 'frame': info.header.frame_id = 'changed_frame'
    else: fake.buffers['color_info'][-1] = (stamp, 'f'*32, info, receipt)
    # Ensure the mutated delayed item is the only candidate.
    fake.buffers['color_info'] = deque([fake.buffers['color_info'][-1]], maxlen=4)
    with pytest.raises(ValueError, match='calibration/profile/cohort changed'):
        module.ROCmRGBD._admit(fake)
    assert offers == []


@pytest.mark.parametrize('age_ns', [-1_000_000, 501_000_000])
def test_future_or_stale_info_inside_pair_window_still_rejected(age_ns):
    fake, offers, _, now, image_ns = delayed_info_fixture()
    # For stale Info keep images499ms old, within original500ms lease.
    if age_ns > 0:
        fake.identity['confirmed_ros_ns'] = now-600_000_000
        for stream in ('depth', 'color'):
            set_replay_stamp(fake, stream, now-499_000_000)
        for stream in module.TOPICS:
            fake.buffers[stream] = deque([fake.buffers[stream][-1]], maxlen=4)
        set_replay_stamp(fake, 'depth_info', now-499_000_000)
    else:
        for stream in module.TOPICS:
            set_replay_stamp(fake, stream, now-10_000_000)
            fake.buffers[stream] = deque([fake.buffers[stream][-1]], maxlen=4)
    set_replay_stamp(fake, 'color_info', now-age_ns)
    fake.buffers['color_info'] = deque([fake.buffers['color_info'][-1]], maxlen=4)
    with pytest.raises(ValueError, match='CameraInfo exposure'):
        module.ROCmRGBD._admit(fake)
    assert offers == []


@pytest.mark.parametrize('delta_ns,available', [(40_000_000, True), (40_000_001, False)])
def test_info_temporal_boundary_remains_exactly_40ms(delta_ns, available):
    fake, _, _, _, image_ns = delayed_info_fixture()
    set_replay_stamp(fake, 'color_info', image_ns-delta_ns)
    fake.buffers['color_info'] = deque([fake.buffers['color_info'][-1]], maxlen=4)
    result = fake._info('color_info', image_ns, (6, 4), [4., 4., 3., 2.],
                        'camera_color_optical_frame')
    assert (result is not None) is available
