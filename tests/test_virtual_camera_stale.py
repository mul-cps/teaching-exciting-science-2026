"""CPU-only production method replay: no ROS initialization, EGL or devices."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_perception/so101_station_perception/virtual_gripper_camera.py'

class Message:
    def __init__(self):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')

class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


def node():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'VirtualGripperCamera')
    names = {'_mark_valid', '_invalidate', '_watchdog', '_status'}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    scope = {'np': np, 'json': json, 'math': __import__('math'),
             'time': SimpleNamespace(monotonic=lambda: 10.), 'Image': Message, 'String': Message}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), scope)
    obj = scope['VirtualGripperCamera']()
    obj.width, obj.height, obj.max_age = 320, 240, .5
    obj.optical_frame = 'synthetic_gripper_camera_optical'
    obj.last_stamp = -1
    obj.last_valid_stamp_ns = None
    obj.last_valid_tool_tf_stamp_ns = None
    obj.last_valid_monotonic = None
    obj.display_valid, obj.invalid_frame_sent = False, False
    obj.invalid_reason = 'waiting_for_first_render'
    obj.now_ns = 10_000_000_000
    obj.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=obj.now_ns))
    for key in ('images', 'scenes', 'annotated', 'annotation_masks', 'provenance_masks', 'status_publisher'):
        setattr(obj, key, Publisher())
    return obj, scope


def test_no_cloud_watchdog_blanks_once_without_inventing_acquisition():
    obj, _ = node()
    obj._watchdog(); obj._watchdog()
    assert len(obj.annotated.messages) == 1
    assert obj.annotated.messages[0].header.stamp.sec == 0
    assert obj.annotated.messages[0].header.frame_id == obj.optical_frame
    assert b'STALE' not in obj.annotated.messages[0].data  # rendered text, not a text payload
    assert np.count_nonzero(np.frombuffer(obj.annotated.messages[0].data, np.uint8)) > 0
    status = json.loads(obj.status_publisher.messages[-1].data)
    assert status['visual_valid'] is False and status['last_valid_acquisition_stamp_ns'] is None
    assert status['checked_ros_ns'] == obj.now_ns


@pytest.mark.parametrize('reason', ['missing_acquisition_tf', 'rejected_after_render'])
def test_render_failure_clears_all_pixels_masks_and_retains_original_header(reason):
    obj, _ = node()
    obj._mark_valid(9_900_000_123, 9_900_000_123, 9.9)
    obj._invalidate(reason)
    for key in ('images', 'scenes', 'annotation_masks', 'provenance_masks'):
        message = getattr(obj, key).messages[0]
        assert not any(message.data)
        assert message.header.stamp.sec == 9 and message.header.stamp.nanosec == 900_000_123
        assert message.header.frame_id == obj.optical_frame
    assert obj.images.messages[0].encoding == 'rgba8'
    assert obj.scenes.messages[0].encoding == 'rgba8'
    assert obj.annotated.messages[0].encoding == 'rgb8'
    assert obj.provenance_masks.messages[0].encoding == 'mono8'


@pytest.mark.parametrize('now_ns,monotonic', [(10_500_000_001, 10.), (10_000_000_000, 10.500001),
                                            (9_999_999_999, 10.), (10_000_000_000, 9.99),
                                            (10_000_000_000, float('nan'))])
def test_watchdog_rejects_original_age_receipt_age_and_clock_jumps(now_ns, monotonic):
    obj, scope = node()
    obj._mark_valid(10_000_000_000, 10_000_000_000, 10.)
    obj.now_ns = now_ns
    scope['time'].monotonic = lambda: monotonic
    obj._watchdog()
    assert obj.display_valid is False
    assert len(obj.images.messages) == 1


def test_fresh_restore_rearms_one_blank_transition_and_no_early_expiry():
    obj, _ = node()
    obj._mark_valid(10_000_000_000, 10_000_000_000, 10.)
    obj._watchdog()
    assert obj.display_valid and not obj.images.messages
    obj._invalidate('tf_failure'); obj._invalidate('tf_failure')
    assert len(obj.images.messages) == 1
    obj._mark_valid(10_000_000_001, 10_000_000_001, 10.)
    assert obj.display_valid and not obj.invalid_frame_sent
    obj._invalidate('cloud_stopped')
    assert len(obj.images.messages) == 2


def test_invalidation_has_no_gpu_tf_info_or_cached_pixel_work_and_teardown_cancels_timer():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    assert '_watchdog' in methods
    text = ast.unparse(methods['_invalidate'])
    assert not any(word in text for word in ('renderer', 'self.infos', 'tf_publisher', 'last_image', 'lookup_transform'))
    assert not any(isinstance(n, (ast.While, ast.ListComp, ast.DictComp)) for n in ast.walk(methods['_watchdog']))
    shutdown = ast.unparse(methods['destroy_node'])
    assert shutdown.index('cancel()') < shutdown.index('self.renderer.close()')
    cloud = ast.unparse(methods['_cloud'])
    assert cloud.count('self._invalidate(') >= 3
    assert 'self._mark_valid(' in cloud


def cloud_node():
    obj, scope = node()
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_cloud')
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), scope)
    obj._cloud = scope['_cloud'].__get__(obj)
    class TFError(Exception): pass
    scope.update(TransformException=TFError, Time=SimpleNamespace(from_msg=lambda value: value),
                 Duration=lambda **kwargs: None,
                 quaternion_matrix=lambda *args: np.eye(3),
                 sampled_cloud=lambda *args: (np.zeros((0, 3)), None, 'absent'))
    def transform_message():
        result = Message()
        result.header.stamp.sec = 10
        result.transform = SimpleNamespace(translation=SimpleNamespace(x=0., y=0., z=0.),
                                            rotation=SimpleNamespace(x=0., y=0., z=0., w=1.))
        return result
    scope['TransformStamped'] = transform_message
    scope['CameraInfo'] = Message
    obj.buffer = SimpleNamespace(lookup_transform=lambda *args, **kwargs: transform_message())
    obj.period, obj.last_attempt, obj.max_points = 1 / 3, -float('inf'), 20000
    obj.tool_frame = 'gripper_frame_link'
    obj.optical_from_tool = obj.projection = obj.mount = np.eye(4)
    obj.mount_quat = [0., 0., 0., 1.]
    obj.k = np.eye(3)
    obj.render_gripper_mesh = False
    obj.meshes, obj.mesh_provenance = [], {}
    obj.renderer = SimpleNamespace(render_layers=lambda *args, **kwargs: {
        'raw': np.zeros((240, 320, 4), np.uint8),
        'scene': np.zeros((240, 320, 4), np.uint8),
        'provenance': np.zeros((240, 320), np.uint8)}, readback_bytes=614400, renderer='FAKE_CPU_TEST')
    obj.infos, obj.tf_publisher = Publisher(), Publisher()
    obj.tf_publisher.sendTransform = obj.tf_publisher.publish
    obj.publish_annotation = True
    obj._annotation = lambda *args: {'state': 'test_only'}
    message = Message()
    message.header.stamp.sec, message.header.frame_id = 10, 'camera_depth_optical_frame'
    return obj, scope, message


@pytest.mark.parametrize('failure', ['tf', 'render', 'late_render', 'mesh_tf'])
def test_actual_cloud_callback_rejects_tf_render_and_late_result_without_tf_info_renewal(failure):
    obj, scope, cloud = cloud_node()
    obj._mark_valid(9_900_000_000, 9_900_000_000, 9.9)
    def fail(*args, **kwargs): raise RuntimeError('injected failure')
    if failure == 'tf':
        obj.buffer.lookup_transform = fail
    elif failure == 'mesh_tf':
        obj.render_gripper_mesh, obj.meshes = True, [object()]
        scope['LINKS'] = ('gripper_link', 'moving_jaw_so101_v1_link')
        lookup = obj.buffer.lookup_transform
        def mesh_fail(target, source, *args, **kwargs):
            if source == 'gripper_link': raise scope['TransformException']('missing mesh TF')
            return lookup(target, source, *args, **kwargs)
        obj.buffer.lookup_transform = mesh_fail
    elif failure == 'render': obj.renderer.render_layers = fail
    else:
        render = obj.renderer.render_layers
        def late(*args, **kwargs):
            obj.now_ns = 10_500_000_001
            return render(*args, **kwargs)
        obj.renderer.render_layers = late
    obj._cloud(cloud)
    assert not obj.display_valid
    assert len(obj.annotated.messages) == 1
    assert not obj.infos.messages and not obj.tf_publisher.messages
    assert obj.images.messages[0].header.stamp.sec == 9


def test_actual_fresh_callback_restores_render_and_original_status_stamps():
    obj, _, cloud = cloud_node()
    obj._invalidate('previous_failure')
    obj._cloud(cloud)
    assert obj.display_valid and not obj.invalid_frame_sent
    assert obj.images.messages[-1].header.stamp is cloud.header.stamp
    assert obj.images.messages[-1].header.frame_id == obj.optical_frame
    status = json.loads(obj.status_publisher.messages[-1].data)
    assert status['state'] == 'published' and status['visual_valid']
    assert status['last_valid_acquisition_stamp_ns'] == status['last_valid_tool_tf_stamp_ns'] == 10**10


def test_watchdog_cancel_precedes_renderer_close_in_actual_teardown():
    obj, scope = node()
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'destroy_node')
    events = []
    scope['super'] = lambda: SimpleNamespace(destroy_node=lambda: events.append('destroy'))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), scope)
    obj.watchdog = SimpleNamespace(cancel=lambda: events.append('cancel'))
    obj.renderer = SimpleNamespace(close=lambda: events.append('close'))
    scope['destroy_node'](obj)
    assert events == ['cancel', 'close', 'destroy']


def test_pinch_annotation_disabled_still_restores_stale_display_without_crosshair():
    obj, _, cloud = cloud_node()
    obj.publish_annotation = False
    obj._invalidate('previous_failure')
    obj._annotation = lambda *args: pytest.fail('pinch annotation must stay disabled')
    obj._cloud(cloud)
    assert obj.display_valid
    assert len(obj.annotated.messages) == 2
    display = obj.annotated.messages[-1]
    assert display.encoding == 'rgb8' and display.header.stamp.sec == 10
    assert not any(display.data)
