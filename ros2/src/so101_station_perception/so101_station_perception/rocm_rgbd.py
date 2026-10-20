"""Default-off derived RGB-D diagnostics; no camera, TF or control authority."""
from collections import deque
from array import array
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from so101_station_projection.rocm_projection import calibration_from_oracle
    from so101_station_projection.rocm_client import ProjectionWorker
    from so101_station_projection.rocm_transport import ExposureLedger, MAX_AGE_NS, MAX_PAIR_NS, build_lineage, source_hashes
except ModuleNotFoundError as error:
    if error.name != 'so101_station_projection': raise
    from workshop.perception.rocm_projection import calibration_from_oracle
    from workshop.perception.rocm_client import ProjectionWorker
    from workshop.perception.rocm_transport import ExposureLedger, MAX_AGE_NS, MAX_PAIR_NS, build_lineage, source_hashes


TOPICS = dict(depth='/camera/depth/image_rect_raw', color='/camera/color/image_raw',
              depth_info='/camera/depth/camera_info', color_info='/camera/color/camera_info')
SERIAL = '243322071783'


class TypedSourceReceiptGap(ValueError):
    """Only the two original source-activity gap sites are recoverable."""
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def stamp_ns(message):
    stamp = message.header.stamp
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int or not 0 <= stamp.sec < 2147483648
            or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError('invalid original exposure stamp')
    result = stamp.sec*1_000_000_000+stamp.nanosec
    if result <= 0: raise ValueError('missing original exposure stamp')
    return result


def publisher_gid(info):
    value = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
    if not isinstance(value, (bytes, bytearray, memoryview, list, tuple, array)) or len(value) not in (16, 24):
        raise ValueError('message publisher GID unavailable')
    try: value = bytes(value)
    except (ValueError, TypeError): raise ValueError('message publisher GID unavailable') from None
    if len(value) not in (16, 24) or not any(value) or (len(value) == 24 and any(value[16:])):
        raise ValueError('unsupported/missing DDS publisher GID')
    return value[:16].hex()


def source_snapshot(node, *, check=None):
    """Background graph only: bind all actual image/info endpoints to one camera."""
    records = {}
    for stream, topic in TOPICS.items():
        endpoints = node.get_publishers_info_by_topic(topic)
        if check is not None: check()
        if len(endpoints) != 1: raise ValueError('source must have one publisher per topic')
        endpoint = endpoints[0]
        name = endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name
        gid = publisher_gid({'publisher_gid': endpoint.endpoint_gid})
        records[stream] = dict(node=name, gid=gid)
    names = {record['node'] for record in records.values()}
    if len(names) != 1: raise ValueError('source image/info publishers are not the same camera node')
    name = next(iter(names))
    if len(name) > 128 or name == '/': raise ValueError('invalid camera source node')
    return dict(node=name, endpoints=records)


def image_array(message, size, encoding):
    width, height = size; channels = 2 if encoding == '16UC1' else 3
    if (message.width != width or message.height != height or message.encoding != encoding
            or message.is_bigendian or message.step != width*channels
            or len(message.data) != width*height*channels):
        raise ValueError('input image profile/encoding/stride/storage changed')
    dtype = '<u2' if encoding == '16UC1' else np.uint8
    shape = (height, width) if encoding == '16UC1' else (height, width, 3)
    return np.frombuffer(message.data, dtype=dtype).reshape(shape)


def extrinsic_matrix(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    values = np.array([q.x, q.y, q.z, q.w, t.x, t.y, t.z], np.float64)
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[:4])-1.) > 1e-5:
        raise ValueError('invalid actual optical extrinsics')
    x, y, z, w = values[:4]; matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = values[4:]
    return matrix


class ROCmRGBD(Node):
    def __init__(self, *, worker_factory=ProjectionWorker):
        super().__init__('so101_rocm_rgbd')
        self.worker = None; self.closed = False
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('enabled', False, readonly)
        self.declare_parameter('allow_explicit_reset', False, readonly)
        self.declare_parameter('configuration_file', '', readonly)
        self.declare_parameter('worker_python', '/home/bjoern/miniforge3/bin/python', readonly)
        self.declare_parameter('compatibility_library', '', readonly)
        if self.get_parameter('enabled').value is not True:
            raise ValueError('derived ROCm producer requires explicit enabled=true')
        configuration_path = Path(self.get_parameter('configuration_file').value)
        if not configuration_path.is_file() or configuration_path.stat().st_size > 65536:
            raise ValueError('explicit bounded CPU-oracle configuration required')
        configuration_bytes = configuration_path.read_bytes()
        self.configuration = json.loads(configuration_bytes)
        self._reset_configuration = deepcopy(self.configuration)
        self.calibration = calibration_from_oracle(self.configuration)
        self.configuration_sha = hashlib.sha256(configuration_bytes).hexdigest()
        self.producer_node_sha = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.allow_explicit_reset = self.get_parameter('allow_explicit_reset').value is True
        self._worker_factory = worker_factory
        self._reset = None; self._reset_timer = None
        self._last_reset = float('-inf'); self._reset_deadline = None
        self._reset_paths = dict(configuration=configuration_path.resolve(), node=Path(__file__).resolve(),
            python=Path(self.get_parameter('worker_python').value).resolve(),
            library=Path(self.get_parameter('compatibility_library').value).resolve())
        c = self.calibration
        self.lineage_calibration = dict(sha256=self.configuration_sha, depth_size=list(c.depth_size),
            color_size=list(c.color_size), depth_K=c.depth_k.tolist(), color_K=c.color_k.tolist(),
            depth_D=[0.]*5, color_D=[0.]*5, rotation_column_major=c.rotation.tolist(),
            translation_m=c.translation.tolist(), depth_scale_m=.001,
            depth_scale_source='existingROS16UC1millimetercontract; notindependentphysicalmeasurement')
        self.generation = uuid.uuid4().hex
        self.invalid = None; self.fault = None; self.binding = None; self.identity = None; self.pending_identity = None
        self.last_query = float('-inf'); self.first_source = None; self.ledger = None
        self.startup_started = time.monotonic(); self.output_binding = None; self.preparing_reason = None
        self.buffers = {stream: deque(maxlen=4) for stream in TOPICS}
        self.received = {stream: 0 for stream in TOPICS}; self.last_header = {}; self.last_receipt = {}
        self.unique = {stream: 0 for stream in TOPICS}
        self.counters = dict(admitted=0, superseded=0, published=0, duplicate=0, stale=0, awaiting_identity=0)
        self.last_status = 0.; self.last_result = None
        self.last_tick_started_monotonic = None; self.last_tick_gap_s = None
        self.device_client = self.parameter_client = None
        self.client_binding = None
        self.depth_publisher = self.create_publisher(Image, '/so101_perception/rocm/aligned_depth', qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(CameraInfo, '/so101_perception/rocm/aligned_camera_info', qos_profile_sensor_data)
        self.cloud_publisher = self.create_publisher(PointCloud2, '/so101_perception/rocm/points', qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(String, '/so101_perception/rocm/status', 1)
        self.lineage_publisher = self.create_publisher(String, '/so101_perception/rocm/lineage', qos_profile_sensor_data)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        # Required two-argument callbacks retain real DDS message publisher GIDs.
        self.subscriptions_ = []
        for stream, topic in TOPICS.items():
            def callback(message, info, stream=stream): self._receive(stream, message, info)
            self.subscriptions_.append(self.create_subscription(CameraInfo if stream.endswith('info') else Image,
                topic, callback, qos_profile_sensor_data))
        self.subscriptions_.append(self.create_subscription(ParameterEvent, '/parameter_events', self._parameter_event, 10))
        try:
            self.worker = worker_factory(self.configuration, self.get_parameter('worker_python').value,
                self.get_parameter('compatibility_library').value, enabled=True)
            self._reset_expected = dict(configuration=self.configuration_sha, node=self.producer_node_sha,
                python=self.worker.provenance['worker_python_sha256'],
                library=self.worker.provenance['compatibility_sha256'],
                sources=deepcopy(self.worker.provenance['source_sha256']))
            self.reset_service = self.create_service(Trigger, '~/reset_source', self._reset_source)
            self.timer = self.create_timer(.005, self._tick)
            self.identity_timer = self.create_timer(.05, self._identity_tick)
        except Exception:
            if self.worker is not None: self.worker.close()
            raise

    def _check_reset_deadline(self):
        if self._reset_deadline is None or time.monotonic() > self._reset_deadline:
            raise TimeoutError('parent reset request deadline2s expired')

    def _check_reset_inputs(self):
        # Explicit reset only: never file/hash work in image or normal timer paths.
        for label, path in self._reset_paths.items():
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            self._check_reset_deadline()
            if actual != self._reset_expected[label]:
                raise ValueError('startup '+label+' bytes changed')
        actual = source_hashes()
        self._check_reset_deadline()
        if actual != self._reset_expected['sources']:
            raise ValueError('startup projection source bytes changed')

    def _reset_status(self):
        if self._reset is None: return None
        result = dict(self._reset)
        count = self.counters['published']-result.pop('published_at_start')
        result['published_new_cohorts'] = count
        result['producer_generation'] = self.generation
        result['cooldown_remaining_s'] = max(0., 30.-(time.monotonic()-self._last_reset))
        ready = False
        if (result['state'] == 'preparing' and not self.invalid and self.worker.ready
                and not self.worker.closed and self.identity is not None
                and self.last_result is not None and count >= 2
                and result.get('preparation_completed_monotonic') is not None
                and result['preparation_completed_monotonic'] <= result['preparation_deadline_monotonic']):
            now = self.get_clock().now().nanoseconds; mono = time.monotonic()
            ready = (self.last_result['producer_generation'] == self.generation
                and all(0 <= now-self.last_result[k] <= MAX_AGE_NS for k in
                        ('depth_stamp_ns', 'color_stamp_ns', 'depth_info_stamp_ns', 'color_info_stamp_ns'))
                and 0 <= mono-self.identity['request_started_monotonic'] <= 1.
                and 0 <= mono-self.last_result['identity']['request_started_monotonic'] <= 1.
                and all(0 <= mono-v <= .250 for v in self.last_result['source_received_monotonic'].values()))
        result['ready'] = ready
        if ready: result['state'] = 'ready'
        elif result['state'] == 'preparing' and self.invalid:
            result['state'] = 'unavailable'; result['reason'] = self.invalid
        return result

    def _cancel_source_rpc(self):
        pending = self.pending_identity
        self.pending_identity = None  # late completion has no owner/generation
        if pending is not None:
            for client, future in zip((self.device_client, self.parameter_client), pending['futures']):
                try: client.remove_pending_request(future)
                except (KeyError, RuntimeError): pass
                future.cancel()

    def _reset_source(self, request, response):
        # Default mutually-exclusive callback group serializes acceptance/job/data.
        deadline = time.monotonic()+2.
        reason = None; accepted = False
        if not self.allow_explicit_reset: reason = 'explicit reset disabled'
        elif self.closed: reason = 'producer closed'
        elif self._reset is not None and self._reset['state'] == 'reset_failed':
            reason = 'permanent reset_failed; external intervention required'
        elif self._reset is not None and self._reset['state'] == 'resetting': reason = 'reset busy'
        elif self._reset is not None and self._reset_status()['state'] == 'preparing': reason = 'reset preparing'
        elif time.monotonic()-self._last_reset < 30.: reason = 'reset cooldown30s'
        if reason is None:
            self._reset_deadline = deadline
            try:
                self._check_reset_inputs()
                self._check_reset_deadline()
                source_snapshot(self, check=self._check_reset_deadline)  # graph, NOT source health
                self._check_reset_deadline()
            except (OSError, ValueError, TypeError, AttributeError) as error:
                reason = str(error)[:512]
        if reason is None:
            self._last_reset = time.monotonic()
            self._reset = dict(accepted_reset_id=uuid.uuid4().hex,
                new_producer_generation=uuid.uuid4().hex, state='resetting', reason=None,
                published_at_start=self.counters['published'],
                preparation_deadline_monotonic=self._last_reset+30., source_reacquisitions=0)
            self.generation = self._reset['new_producer_generation']
            self.fault = None  # Explicit accepted reset retires the previous fault identity.
            self._invalidate('explicit source reset accepted; not ready')
            try:
                self._cancel_source_rpc()
                self._check_reset_deadline()
                self.binding = self.identity = self.pending_identity = None
                self.client_binding = self.first_source = self.ledger = self.output_binding = None
                self.last_header.clear(); self.last_receipt.clear(); self.last_result = None
                for client in (self.device_client, self.parameter_client):
                    if client is not None:
                        self._check_reset_deadline()
                        self.destroy_client(client)
                        self._check_reset_deadline()
                self.device_client = self.parameter_client = None
                self._check_reset_deadline()
                self._reset_timer = self.create_timer(.001, self._perform_reset)
                self._check_reset_deadline()
                accepted = True
            except Exception as error:
                self._reset['state'] = 'reset_failed'
                if self._reset_timer is not None:
                    try:
                        self.destroy_timer(self._reset_timer); self._reset_timer = None
                    except Exception as cleanup:
                        error = RuntimeError(str(error)[:256]+'; reset timer cleanup unknown: '+str(cleanup)[:256])
                self.invalid = 'reset_failed: '+str(error)[:512]
                self._invalidate(self.invalid)
                reason = self.invalid
        payload = dict(accepted=accepted, accepted_reset_id=self._reset['accepted_reset_id'] if accepted else None,
            new_producer_generation=self.generation if accepted else None, producer_generation=self.generation,
            state='resetting' if accepted else 'rejected', ready=False, reason=reason)
        response.success = accepted
        response.message = json.dumps(payload, allow_nan=False, separators=(',', ':'))
        return response

    def _perform_reset(self):
        if self._reset is None or self._reset['state'] != 'resetting' or self.closed: return
        if self._reset_timer is not None:
            self.destroy_timer(self._reset_timer); self._reset_timer = None
        replacement = None
        try:
            self._check_reset_deadline()
            self._check_reset_inputs()
            self._check_reset_deadline()
            old = self.worker
            old.close()  # owned stable pidfd group; no arbitrary PID discovery
            if type(old.process.returncode) is not int:
                raise RuntimeError('owned worker retirement unproven')
            self._check_reset_deadline()
            self._check_reset_inputs()
            self._check_reset_deadline()
            replacement = self._worker_factory(deepcopy(self._reset_configuration),
                str(self._reset_paths['python']), str(self._reset_paths['library']), enabled=True)
            self.worker = replacement  # retain ownership even on later failure
            expected = dict(source_sha256=self._reset_expected['sources'],
                worker_python_sha256=self._reset_expected['python'],
                compatibility_sha256=self._reset_expected['library'],
                worker_python=str(self._reset_paths['python']))
            self._check_reset_deadline()
            if any(self.worker.provenance.get(k) != v for k, v in expected.items()):
                raise RuntimeError('reset construction provenance changed')
            self._check_reset_inputs()
            self._check_reset_deadline()
            self.last_query = float('-inf'); self.startup_started = time.monotonic()
            self.preparing_reason = 'explicit reset: awaiting fresh full source attestation'
            self.invalid = None; self.fault = None; self._reset['state'] = 'preparing'
        except Exception as error:
            if replacement is not None:
                try:
                    replacement.close()
                    if type(replacement.process.returncode) is not int:
                        raise RuntimeError('replacement retirement unproven')
                except Exception as cleanup:
                    error = RuntimeError(str(error)[:256]+'; replacement cleanup unknown: '+str(cleanup)[:256])
            self._reset['state'] = 'reset_failed'
            self._reset['reason'] = str(error)[:512]
            self.invalid = 'reset_failed: '+str(error)[:512]
            self._invalidate(self.invalid)

    def _invalidate(self, reason):
        now = time.monotonic()
        transient = isinstance(reason, TypedSourceReceiptGap)
        snapshot = dict(reason=str(reason)[:1024],
            code=reason.code if transient else 'unknown_nonrecoverable',
            producer_generation=self.generation,
            last_receipt=dict(self.last_receipt),
            receipt_age_s={stream: now-self.last_receipt[stream] if stream in self.last_receipt else None
                           for stream in TOPICS},
            last_header=dict(self.last_header), received=dict(self.received),
            worker_ready=self.worker.ready if self.worker is not None else None,
            worker_closed=self.worker.closed if self.worker is not None else None,
            reset_state=self._reset['state'] if self._reset is not None else None,
            tick_gap_s=self.last_tick_gap_s)
        if (transient and self._reset is not None and self._reset['state'] == 'preparing'
                and self.counters['published']-self._reset['published_at_start'] == 0
                and now < self._reset['preparation_deadline_monotonic']
                and self.worker is not None and not self.worker.closed):
            self._reset['source_reacquisitions'] += 1
            self._reset['last_source_fault'] = snapshot
            self._cancel_source_rpc()
            self.identity = None; self.last_query = float('-inf')
            self.last_receipt.clear()
            for buffer in self.buffers.values(): buffer.clear()
            self.worker.scheduler.pending = None
            self.preparing_reason = 'explicit reset: reacquiring fresh source after '+str(reason)[:1024]
            return
        if getattr(self, 'fault', None) is None:
            self.fault = dict(code=reason.code if transient else 'unknown_nonrecoverable',
                fault_id=uuid.uuid4().hex, producer_generation=self.generation,
                first_observed_monotonic=now,
                first_observed_ros_ns=self.get_clock().now().nanoseconds,
                producer_node_sha256=self.producer_node_sha, source_snapshot=snapshot,
                automatic_recovery_class='transient_private_availability' if transient else 'nonrecoverable')
        if (self._reset is not None and self._reset['state'] == 'preparing'
                and self.counters['published']-self._reset['published_at_start'] < 2):
            self._reset['state'] = 'reset_failed'
            self._reset['reason'] = str(reason)[:512]
        if self.invalid is None: self.invalid = str(reason)[:1024]
        self.identity = None
        for buffer in self.buffers.values(): buffer.clear()
        # Keep active slot owned until worker exits; old results cannot publish.
        if self.worker is not None: self.worker.scheduler.pending = None

    def _receive(self, stream, message, info):
        self.received[stream] += 1
        if self.invalid or self.closed: return
        try:
            gid = publisher_gid(info); stamp = stamp_ns(message)
            if self.binding is not None and gid != self.binding['endpoints'][stream]['gid']:
                raise ValueError('source publisher generation changed')
            previous = self.last_header.get(stream)
            if previous is not None and stamp < previous: raise ValueError('source header rollback')
            receipt = time.monotonic()
            if stream in self.last_receipt and receipt-self.last_receipt[stream] > .250:
                raise TypedSourceReceiptGap('private_source_receipt_gap', 'source stream gap')
            # Activity may advance on duplicates; acquisition receipts never do.
            self.last_receipt[stream] = receipt
            if previous == stamp: return
            self.unique[stream] += 1
            self.last_header[stream] = stamp
            self.buffers[stream].append((stamp, gid, message, receipt))
        except (AttributeError, TypeError, ValueError) as error: self._invalidate(error)

    def _parameter_event(self, event):
        if (self.binding is not None and event.node == self.binding['node']
                and (event.changed_parameters or event.new_parameters or event.deleted_parameters)):
            self._invalidate('camera parameter context changed')

    def _identity_tick(self):
        if self.invalid or self.closed: return
        now = time.monotonic()
        try:
            current = source_snapshot(self)
            if self.client_binding is not None and current != self.client_binding:
                raise ValueError('source changed after identity service binding')
            actual_transform = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'camera_depth_optical_frame', Time())
            actual_extrinsics = extrinsic_matrix(actual_transform)
            expected = np.eye(4); expected[:3, :3] = self.calibration.rotation.reshape(3, 3, order='F')
            expected[:3, 3] = self.calibration.translation
            if not np.allclose(actual_extrinsics, expected, atol=1e-7, rtol=0):
                raise ValueError('actual optical TF differs from pinned projection calibration')
            outputs = {}
            for label, topic in (('cloud', '/so101_perception/rocm/points'), ('status', '/so101_perception/rocm/lineage'),
                                 ('aligned', '/so101_perception/rocm/aligned_depth')):
                endpoints = self.get_publishers_info_by_topic(topic)
                if len(endpoints) != 1: raise ValueError('derived output must have one publisher')
                endpoint = endpoints[0]
                if endpoint.node_namespace.rstrip('/')+'/'+endpoint.node_name != self.get_fully_qualified_name():
                    raise ValueError('derived output is published by a different node')
                outputs[label+'_publisher_gid'] = publisher_gid({'publisher_gid': endpoint.endpoint_gid})
            if self.output_binding is not None and outputs != self.output_binding:
                raise ValueError('derived output generation changed')
            self.output_binding = outputs
            if self.binding is not None and current != self.binding:
                raise ValueError('source graph generation changed')
            if self.pending_identity is not None:
                pending = self.pending_identity
                if now-pending['request'] > .4: raise TimeoutError('source identity query timed out')
                if not all(f.done() for f in pending['futures']): return
                if current != pending['binding']: raise ValueError('source changed during DeviceInfo query')
                device, parameters = [future.result() for future in pending['futures']]
                values = parameters.values
                raw_serial = values[0].string_value if len(values) == 1 and values[0].type == 4 else None
                configured = raw_serial[1:] if isinstance(raw_serial, str) and raw_serial.startswith('_') else raw_serial
                if device.serial_number != SERIAL or configured != SERIAL: raise ValueError('camera serial/configured serial mismatch')
                if source_snapshot(self) != pending['binding'] or time.monotonic()-pending['request'] > .4:
                    raise ValueError('source identity confirmation context changed')
                generation = hashlib.sha256(json.dumps(current, sort_keys=True).encode()).hexdigest()
                if self.first_source is not None and generation != self.first_source: raise ValueError('source generation replacement')
                self.first_source = generation
                if self.ledger is None: self.ledger = ExposureLedger(generation)
                self.binding = current
                self.preparing_reason = None
                self.identity = dict(source_generation=generation, serial_number=SERIAL,
                    request_started_monotonic=pending['request'], confirmed_monotonic=time.monotonic(),
                    confirmed_ros_ns=self.get_clock().now().nanoseconds,
                    expires_monotonic=pending['request']+1.,
                    device_info_request_source=deepcopy(pending['binding']))
                self.pending_identity = None
            if self.identity is not None and now-self.identity['request_started_monotonic'] > 1.:
                raise TimeoutError('source identity lease expired')
            if self.pending_identity is not None or now-self.last_query < .5: return
            if self.device_client is None:
                self.device_client = self.create_client(DeviceInfo, current['node']+'/device_info')
                self.parameter_client = self.create_client(GetParameters, current['node']+'/get_parameters')
                self.client_binding = deepcopy(current)
            if not self.device_client.service_is_ready() or not self.parameter_client.service_is_ready():
                return
            self.last_query = now
            self.pending_identity = dict(request=now, binding=deepcopy(current), futures=[])
            self.pending_identity['futures'].append(self.device_client.call_async(DeviceInfo.Request()))
            self.pending_identity['futures'].append(self.parameter_client.call_async(GetParameters.Request(names=['serial_no'])))
        except (AttributeError, RuntimeError, TypeError, ValueError, TimeoutError, TransformException) as error:
            if self.binding is None and self.pending_identity is None and self.device_client is None and now-self.startup_started <= 30.:
                self.preparing_reason = str(error)[:512]  # await initial DDS/TF discovery only
            else: self._invalidate(error)

    def _info(self, stream, stamp, size, expected_k, frame):
        candidates = self.buffers[stream]
        if not candidates: return None
        selected = min(candidates, key=lambda item: abs(item[0]-stamp))
        _, gid, info, _ = selected
        expected = [float(expected_k[0]), 0., float(expected_k[2]), 0., float(expected_k[1]), float(expected_k[3]), 0., 0., 1.]
        if (info.header.frame_id != frame
                or (info.width, info.height) != size or info.distortion_model != 'plumb_bob'
                or len(info.d) != 5 or any(not math.isfinite(x) or x != 0 for x in info.d)
                or len(info.k) != 9 or not np.array_equal(np.asarray(info.k, np.float32), np.asarray(expected, np.float32))
                or gid != self.binding['endpoints'][stream]['gid']):
            raise ValueError('source CameraInfo calibration/profile/cohort changed')
        # A matching Info exposure can arrive after its image. Keep immutable
        # source checks strict, but wait/drop this unavailable temporal pair.
        if abs(selected[0]-stamp) > MAX_PAIR_NS: return None
        return selected

    def _admit(self):
        if not self.worker.ready or self.identity is None or self.ledger is None or self.output_binding is None: return
        if not self.buffers['depth'] or not self.buffers['color']: return
        if not 0 <= time.monotonic()-self.identity['request_started_monotonic'] <= 1.:
            raise ValueError('source lease expired before admission')
        depth = self.buffers['depth'][-1]
        color = min(self.buffers['color'], key=lambda item: abs(item[0]-depth[0]))
        if (depth[1] != self.binding['endpoints']['depth']['gid']
                or color[1] != self.binding['endpoints']['color']['gid']):
            raise ValueError('pending frame predates qualified camera publisher generation')
        now = self.get_clock().now().nanoseconds
        if (not 0 <= now-depth[0] <= MAX_AGE_NS or not 0 <= now-color[0] <= MAX_AGE_NS
                or abs(depth[0]-color[0]) > MAX_PAIR_NS):
            self.counters['stale'] += 1; return
        if min(depth[0], color[0]) <= self.identity['confirmed_ros_ns']:
            self.counters['awaiting_identity'] += 1; return
        c = self.calibration
        if depth[2].header.frame_id != 'camera_depth_optical_frame' or color[2].header.frame_id != 'camera_color_optical_frame':
            raise ValueError('source optical frame changed')
        di = self._info('depth_info', depth[0], c.depth_size, c.depth_k, depth[2].header.frame_id)
        ci = self._info('color_info', color[0], c.color_size, c.color_k, color[2].header.frame_id)
        if di is None or ci is None: return
        selected_receipts = dict(depth=depth[3], color=color[3], depth_info=di[3], color_info=ci[3])
        receipt_now = time.monotonic()
        if any(not 0 <= receipt_now-receipt <= .250 for receipt in selected_receipts.values()):
            self.counters['stale'] += 1; return
        info_stamps = (stamp_ns(di[2]), stamp_ns(ci[2]))
        if any(not 0 <= now-stamp <= MAX_AGE_NS for stamp in info_stamps):
            raise ValueError('source CameraInfo exposure is future/stale')
        metadata = dict(depth_stamp_ns=depth[0], color_stamp_ns=color[0], source_generation=self.first_source,
            depth_info_stamp_ns=info_stamps[0], color_info_stamp_ns=info_stamps[1],
            producer_generation=self.generation, depth_frame=depth[2].header.frame_id,
            color_frame=color[2].header.frame_id, configuration_sha256=self.configuration_sha,
            source=deepcopy(self.binding), identity=deepcopy(self.identity), qualified=False,
            source_received_monotonic=selected_receipts)
        if not self.ledger.admit(metadata, now): self.counters['duplicate'] += 1; return
        d = image_array(depth[2], c.depth_size, '16UC1'); rgb = image_array(color[2], c.color_size, 'rgb8')
        if self.worker.scheduler.pending is not None: self.counters['superseded'] += 1
        self.worker.offer(d, rgb, metadata); self.counters['admitted'] += 1

    def _publish(self, result):
        metadata = result['metadata']; now = self.get_clock().now().nanoseconds
        if (self.invalid or self.identity is None or metadata['source_generation'] != self.first_source
                or metadata['producer_generation'] != self.generation
                or not 0 <= time.monotonic()-self.identity['request_started_monotonic'] <= 1.
                or not 0 <= time.monotonic()-metadata['identity']['request_started_monotonic'] <= 1.
                or any(not 0 <= time.monotonic()-receipt <= .250 for receipt in metadata['source_received_monotonic'].values())
                or any(not 0 <= now-metadata[key] <= MAX_AGE_NS for key in ('depth_stamp_ns', 'color_stamp_ns', 'depth_info_stamp_ns', 'color_info_stamp_ns'))
                or min(metadata['depth_stamp_ns'], metadata['color_stamp_ns']) <= self.identity['confirmed_ros_ns']):
            self.counters['stale'] += 1; return
        sec, ns = divmod(metadata['depth_stamp_ns'], 1_000_000_000)
        image = Image(); image.header.stamp.sec = sec; image.header.stamp.nanosec = ns
        image.header.frame_id = metadata['color_frame']; image.width, image.height = self.calibration.color_size
        image.encoding = '16UC1'; image.is_bigendian = 0; image.step = image.width*2; image.data = result['aligned']
        info = CameraInfo(); info.header = deepcopy(image.header); info.width = image.width; info.height = image.height
        fx, fy, cx, cy = map(float, self.calibration.color_k)
        info.distortion_model = 'plumb_bob'; info.d = [0.]*5
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]; info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        cloud = PointCloud2(); cloud.header = deepcopy(image.header); cloud.header.frame_id = metadata['depth_frame']
        cloud.height = 1; cloud.width = len(result['cloud'])//20; cloud.is_bigendian = False; cloud.is_dense = True
        cloud.point_step = 20; cloud.row_step = len(result['cloud']); cloud.data = result['cloud']
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in (('x', 0), ('y', 4), ('z', 8), ('rgb', 16))]
        lineage = build_lineage(metadata, result, self.output_binding, self.lineage_calibration,
                                dict(self.worker.provenance, hardware=self.worker.hardware,
                                     producer_node_sha256=self.producer_node_sha),
                                self.get_clock().now().nanoseconds)
        lineage_message = String(data=json.dumps(lineage, allow_nan=False))
        # Final checks follow all message materialization/copy work.
        final_now = self.get_clock().now().nanoseconds
        if (self.invalid or self.identity is None
                or any(not 0 <= final_now-metadata[key] <= MAX_AGE_NS for key in ('depth_stamp_ns', 'color_stamp_ns', 'depth_info_stamp_ns', 'color_info_stamp_ns'))
                or not 0 <= time.monotonic()-self.identity['request_started_monotonic'] <= 1.
                or not 0 <= time.monotonic()-metadata['identity']['request_started_monotonic'] <= 1.
                or any(not 0 <= time.monotonic()-receipt <= .250 for receipt in metadata['source_received_monotonic'].values())):
            self.counters['stale'] += 1; return
        started = time.monotonic()
        if (self._reset is not None and self._reset['state'] == 'preparing'
                and self.counters['published']-self._reset['published_at_start'] < 2
                and time.monotonic() > self._reset['preparation_deadline_monotonic']):
            raise TimeoutError('source reset preparation deadline30s expired')
        self.depth_publisher.publish(image); self.info_publisher.publish(info); self.cloud_publisher.publish(cloud)
        self.lineage_publisher.publish(lineage_message)
        self.counters['published'] += 1
        if (self._reset is not None and self._reset['state'] == 'preparing'
                and self.counters['published']-self._reset['published_at_start'] == 2):
            self._reset['preparation_completed_monotonic'] = time.monotonic()
            if self._reset['preparation_completed_monotonic'] > self._reset['preparation_deadline_monotonic']:
                self._reset['state'] = 'reset_failed'
                self._reset['reason'] = 'source reset preparation deadline30s expired during publication'
                raise TimeoutError(self._reset['reason'])
        self.last_result = dict(metadata, backend_ms=result['backend_ms'], worker_ms=result['worker_ms'],
            source_age_ms=(final_now-min(metadata['depth_stamp_ns'], metadata['color_stamp_ns']))/1e6,
            cloud_bytes=len(result['cloud']), points=cloud.width, publish_ms=(time.monotonic()-started)*1000)

    def _tick(self):
        now = time.monotonic()
        self.last_tick_gap_s = (now-self.last_tick_started_monotonic
                               if self.last_tick_started_monotonic is not None else None)
        self.last_tick_started_monotonic = now
        if self.closed: return
        try:
            if (self._reset is not None and self._reset['state'] == 'preparing'
                    and self.counters['published']-self._reset['published_at_start'] < 2
                    and now > self._reset['preparation_deadline_monotonic']):
                raise TimeoutError('source reset preparation deadline30s expired')
            if not self.invalid:
                if self.identity is not None and any(time.monotonic()-self.last_receipt.get(k, self.identity['confirmed_monotonic']) > .250 for k in TOPICS):
                    raise TypedSourceReceiptGap('private_source_stream_lost', 'source stream lost before publication')
                result = self.worker.poll()
                if result is not None: self._publish(result)
                self._admit()
        except (AttributeError, RuntimeError, TypeError, ValueError, TimeoutError, OSError) as error: self._invalidate(error)
        now = time.monotonic()
        if now-self.last_status >= .2:
            self.last_status = now
            status = dict(schema='so101-derived-rocm-rgbd-v1', qualified=False, execution_enabled=False,
                direct_camera_attestation=False, producer_generation=self.generation,
                state='unavailable' if self.invalid else ('ready' if self.worker.ready and self.identity and self.last_result else 'preparing'),
                reason=self.invalid or self.preparing_reason, fault=getattr(self, 'fault', None), counters=self.counters, received=self.received,
                unique_exposures=self.unique,
                hardware=self.worker.hardware, source=self.binding, last_result=self.last_result,
                reset=self._reset_status())
            self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))

    def destroy_node(self):
        self.closed = True
        if self._reset_timer is not None:
            self.destroy_timer(self._reset_timer); self._reset_timer = None
        if self.pending_identity is not None:
            for client, future in zip((self.device_client, self.parameter_client), self.pending_identity['futures']):
                try: client.remove_pending_request(future)
                except (KeyError, RuntimeError): pass
                future.cancel()
        if self.worker is not None: self.worker.close()
        return super().destroy_node()


def main(args=None):
    value = os.environ.setdefault('ROS_DOMAIN_ID', '76'); os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    if not value.isdigit() or not 1 <= int(value) <= 232: raise ValueError('nonzero ROS domain1..232 required')
    node = None; rclpy.init(args=args)
    try:
        node = ROCmRGBD(); rclpy.spin(node)
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown()
