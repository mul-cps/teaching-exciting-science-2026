"""Native ROS message contract tests, no DDS or robot connection."""
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_perception'))
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_gpu'))
# Match CMake's private pure-helper package without requiring a colcon build.
private = types.ModuleType('so101_station_perception_tag_tracking')
private.__path__ = [str(ROOT / 'workshop/digital_twin')]
sys.modules.setdefault(private.__name__, private)
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from so101_station_perception.virtual_gripper_camera import sampled_xyz, VirtualGripperCamera
from so101_station_perception_tag_tracking.virtual_camera import virtual_intrinsics, optical_projection


def cloud(points=None, stamp_ns=10_000_000_000):
    points = np.array([[0.,0.,.5], [.1,.1,.4]], dtype='<f4') if points is None else points
    msg = PointCloud2()
    msg.header.frame_id = 'source_optical'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    msg.width, msg.height = len(points), 1
    msg.point_step, msg.row_step = 12, len(points)*12
    msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1) for i,n in enumerate(('x','y','z'))]
    msg.data = points.tobytes()
    return msg


def test_xyz_sampling_caps_filters_nonfinite_and_handles_padded_rows():
    a = np.arange(60006, dtype='<f4').reshape(-1,3)
    a[0, 0] = np.nan
    result = sampled_xyz(cloud(a), 20000)
    assert len(result) <= 20000 and np.isfinite(result).all()
    msg = cloud(np.array([[1,2,3], [4,5,6]], dtype='<f4'))
    msg.width, msg.height, msg.row_step = 1, 2, 16
    msg.data = np.array([1,2,3,0,4,5,6,0], dtype='<f4').tobytes()
    assert sampled_xyz(msg) == pytest.approx(np.array([[1,2,3],[4,5,6]]))
    msg.is_bigendian = True
    msg.data = np.array([1,2,3,0,4,5,6,0], dtype='>f4').tobytes()
    assert sampled_xyz(msg) == pytest.approx(np.array([[1,2,3],[4,5,6]]))


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self, message):self.messages.append(message)
    def sendTransform(self, message):self.messages.append(message)


def fixture_node():
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    lookups=[]
    def lookup(*args, **kwargs):lookups.append((args,kwargs));return tf
    rendered=[]
    def render(points,matrix):
        rendered.append((points,matrix))
        return np.zeros((240,320,4),np.uint8)
    k = virtual_intrinsics()
    states=[]
    n = NS(period=1/3, last_attempt=-float('inf'),last_stamp=-1,max_age=.5,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
        _status=lambda state,**kw:states.append((state,kw)),
        buffer=NS(lookup_transform=lookup),tool_frame='gripper_frame_link',
        optical_frame='synthetic_gripper_camera_optical',mount=np.eye(4),optical_from_tool=np.eye(4),mount_quat=[0.,0.,0.,1.],max_points=20000,
        renderer=NS(render=render,renderer='FAKE-MESSAGE-TEST-NOT-GPU'),
        projection=optical_projection(k,320,240),width=320,height=240,k=k,
        images=Publisher(),infos=Publisher(),tf_publisher=Publisher(),publish_annotation=False)
    return n,states,lookups,rendered


def test_synthetic_image_info_and_mount_tf_preserve_exact_acquisition_stamp():
    n,states,lookups,rendered=fixture_node()
    msg=cloud()
    VirtualGripperCamera._cloud(n,msg)
    assert states[-1][0]=='published'
    image,info,tf = n.images.messages[0],n.infos.messages[0],n.tf_publisher.messages[0]
    assert image.header.stamp==info.header.stamp==tf.header.stamp==msg.header.stamp
    assert image.header.frame_id==info.header.frame_id==tf.child_frame_id==n.optical_frame
    assert tf.header.frame_id==n.tool_frame
    assert image.encoding=='rgba8' and image.step==1280 and len(image.data)==320*240*4
    assert info.k==pytest.approx(list(n.k.flat))
    args,kwargs=lookups[0]
    assert args[:2]==(n.tool_frame,msg.header.frame_id)
    assert args[2].nanoseconds==10_000_000_000
    assert kwargs['timeout'].nanoseconds==20_000_000
    assert len(rendered)==1


@pytest.mark.parametrize('stamp',[0, 9_000_000_000, 10_200_000_000])
def test_stale_future_zero_stamp_drops_before_decode_tf_render_publish(stamp):
    n,states,lookups,rendered=fixture_node()
    VirtualGripperCamera._cloud(n,cloud(stamp_ns=stamp))
    assert states[-1][0]=='rejected_stamp'
    assert not lookups and not rendered and not n.images.messages


def test_missing_acquisition_time_tf_drops_without_output():
    from tf2_ros import LookupException
    n,states,lookups,rendered=fixture_node()
    def missing(*a,**kw):raise LookupException('missing exact TF')
    n.buffer.lookup_transform=missing
    VirtualGripperCamera._cloud(n,cloud())
    assert states[-1][0]=='rejected' and not rendered and not n.images.messages


def test_cloud_duplicate_and_rate_limit_do_not_decode_or_render_twice():
    n,states,lookups,rendered=fixture_node();msg=cloud()
    VirtualGripperCamera._cloud(n,msg)
    VirtualGripperCamera._cloud(n,msg)
    assert len(rendered)==1
    n.last_attempt=-float('inf')
    VirtualGripperCamera._cloud(n,msg)
    assert states[-1][0]=='rejected_stamp' and len(rendered)==1


def test_annotation_does_not_change_unknown_rgba_or_admit_overlay_as_depth():
    from so101_station_perception_tag_tracking.virtual_camera import tool_from_virtual_optical
    n,states,lookups,rendered=fixture_node()
    n.mount=tool_from_virtual_optical()
    pose=np.eye(4);pose[:3,3]=[.01,0,.1]
    n.pinch={'schema':'so101-virtual-pinch-overlay-v1','qualified':False,
        'parent_frame':'urdf:gripper_frame_link','parent_from_pinch':pose.tolist(), 'opening_angle_rad':-.06}
    n.opening=(10_000_000_000,-.06);n.pinch_sha='artifact-test'
    n.near,n.far=.01,2.
    n.annotated,n.annotation_masks=Publisher(),Publisher()
    rgba=np.zeros((240,320,4),np.uint8);original=rgba.copy()
    from sensor_msgs.msg import Image
    image=Image()
    result=VirtualGripperCamera._annotation(n,image,rgba,10_000_000_000)
    assert np.array_equal(rgba,original)
    assert result['state']=='nominal_uncalibrated' and result['annotation_is_depth'] is False
    assert result['pixel_xy'][0]>159.5
    assert n.annotated.messages[0].encoding=='rgb8'
    assert n.annotation_masks.messages[0].encoding=='mono8'
    assert np.count_nonzero(np.frombuffer(n.annotation_masks.messages[0].data,dtype=np.uint8))>0
    n.opening=(10_000_000_000,.1)
    result=VirtualGripperCamera._annotation(n,image,rgba,10_000_000_000)
    assert result['pixel_xy'] is None


def test_joint_receipt_rejects_future_sample_even_within_cloud_sync_window():
    from sensor_msgs.msg import JointState
    n,states,lookups,rendered=fixture_node()
    n.opening=None
    j=JointState();j.name=['gripper'];j.position=[-.06]
    j.header.stamp.sec=10;j.header.stamp.nanosec=120_000_000
    VirtualGripperCamera._joints(n,j)
    assert n.opening is None


def test_future_joint_within_50ms_sync_window_hides_crosshair_and_preserves_raw():
    n,states,lookups,rendered=fixture_node()
    pose=np.eye(4);pose[:3,3]=[.01,0,.1]
    n.pinch={'schema':'so101-virtual-pinch-overlay-v1','qualified':False,
        'parent_frame':'urdf:gripper_frame_link','parent_from_pinch':pose.tolist(),'opening_angle_rad':-.06}
    n.opening=(10_120_000_000,-.06);n.pinch_sha='future-test';n.near,n.far=.01,2.
    n.annotated,n.annotation_masks=Publisher(),Publisher()
    rgba=np.zeros((240,320,4),np.uint8)
    from sensor_msgs.msg import Image
    result=VirtualGripperCamera._annotation(n,Image(),rgba,10_100_000_000)
    assert result['state']=='opening_mismatch_or_stale' and result['pixel_xy'] is None
    assert np.count_nonzero(rgba)==0
