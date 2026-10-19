"""Native ROS message contract tests, no DDS or robot connection."""
import sys
import json
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
from so101_station_perception_tag_tracking.virtual_camera import virtual_intrinsics, optical_projection, JointOpeningHistory


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
    def __init__(self, on_publish=None):self.messages=[];self.on_publish=on_publish
    def publish(self, message):
        self.messages.append(message)
        if self.on_publish is not None:self.on_publish(message)
    def sendTransform(self, message):self.messages.append(message)


def fixture_node():
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    lookups=[]
    def lookup(*args, **kwargs):
        lookups.append((args,kwargs))
        tf.header.stamp.sec,tf.header.stamp.nanosec=divmod(args[2].nanoseconds,10**9)
        tf.header.frame_id,tf.child_frame_id=args[:2]
        return tf
    rendered=[]
    def render_layers(points,matrix,colors=None,mesh_matrices=None):
        rendered.append((points,matrix,colors,mesh_matrices))
        return {"raw":np.zeros((240,320,4),np.uint8),"scene":np.zeros((240,320,4),np.uint8),
                "provenance":np.zeros((240,320),np.uint8)}
    k = virtual_intrinsics()
    states=[]
    n = NS(period=1/3, last_attempt=-float('inf'),last_stamp=-1,max_age=.5,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
        buffer=NS(lookup_transform=lookup),tool_frame='gripper_frame_link',
        optical_frame='synthetic_gripper_camera_optical',mount=np.eye(4),optical_from_tool=np.eye(4),mount_quat=[0.,0.,0.,1.],max_points=20000,
        renderer=NS(render_layers=render_layers,renderer='FAKE-MESSAGE-TEST-NOT-GPU',readback_bytes=614400),
        render_gripper_mesh=False,meshes=[],mesh_provenance={'state':'disabled'},
        projection=optical_projection(k,320,240),width=320,height=240,k=k,
        images=Publisher(),infos=Publisher(),tf_publisher=Publisher(),publish_annotation=False,
        opening_history=JointOpeningHistory(),scenes=Publisher(),provenance_masks=Publisher(),
        annotated=Publisher(),annotation_masks=Publisher(),last_valid_stamp_ns=None,
        last_valid_tool_tf_stamp_ns=None,last_valid_monotonic=None,
        display_valid=False,invalid_frame_sent=False,invalid_reason='waiting_for_first_render')
    def record_status(message):
        value=json.loads(message.data)
        states.append((value['state'],value))
    n.status_publisher=Publisher(record_status)
    # Execute real production transitions/status/watchdog with native ROS messages.
    # Only TF lookup and GPU rendering are fake: this fixture never initializes DDS/EGL.
    for name in ('_mark_valid','_invalidate','_watchdog','_status'):
        setattr(n,name,types.MethodType(getattr(VirtualGripperCamera,name),n))
    return n,states,lookups,rendered


def assert_invalid_display(n,states,*,stamp_ns=0,last_valid_stamp_ns=None):
    assert states[-1][0]=='invalid' and states[-1][1]['visual_valid'] is False
    assert states[-1][1]['last_valid_acquisition_stamp_ns']==last_valid_stamp_ns
    assert n.display_valid is False and n.invalid_frame_sent is True
    for publisher,encoding,step in ((n.images,'rgba8',1280),(n.scenes,'rgba8',1280),
                                    (n.provenance_masks,'mono8',320),(n.annotation_masks,'mono8',320)):
        image=publisher.messages[-1]
        assert image.encoding==encoding and image.step==step
        assert image.width==320 and image.height==240 and len(image.data)==step*240
        assert not any(image.data)
        assert image.header.frame_id==n.optical_frame
        assert image.header.stamp.sec*10**9+image.header.stamp.nanosec==stamp_ns
    diagnostic=n.annotated.messages[-1]
    assert diagnostic.encoding=='rgb8' and diagnostic.step==960
    assert diagnostic.header==n.images.messages[-1].header
    assert len(diagnostic.data)==320*240*3 and any(diagnostic.data) # rendered STALE text only


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
    assert n.display_valid and not n.invalid_frame_sent
    assert n.last_valid_stamp_ns==n.last_valid_tool_tf_stamp_ns==10_000_000_000
    assert states[-1][1]['visual_valid'] is True
    assert states[-1][1]['last_valid_acquisition_stamp_ns']==10_000_000_000
    assert n.annotated.messages[-1].encoding=='rgb8' # display restores even with pinch disabled


@pytest.mark.parametrize('stamp',[0, 9_000_000_000, 10_200_000_000])
def test_stale_future_zero_stamp_blanks_before_decode_tf_render_without_renewal(stamp):
    n,states,lookups,rendered=fixture_node()
    VirtualGripperCamera._cloud(n,cloud(stamp_ns=stamp))
    assert_invalid_display(n,states)
    assert states[-1][1]['reason']=='rejected_stamp'
    assert not lookups and not rendered
    assert len(n.images.messages)==len(n.annotated.messages)==1
    assert not n.infos.messages and not n.tf_publisher.messages


def test_missing_acquisition_time_tf_blanks_without_render_info_or_tf_renewal():
    from tf2_ros import LookupException
    n,states,lookups,rendered=fixture_node()
    def missing(*a,**kw):raise LookupException('missing exact TF')
    n.buffer.lookup_transform=missing
    VirtualGripperCamera._cloud(n,cloud())
    assert_invalid_display(n,states)
    assert 'missing exact TF' in states[-1][1]['reason']
    assert not rendered and not n.infos.messages and not n.tf_publisher.messages


def test_cloud_duplicate_and_rate_limit_do_not_decode_or_render_twice():
    n,states,lookups,rendered=fixture_node();msg=cloud()
    VirtualGripperCamera._cloud(n,msg)
    VirtualGripperCamera._cloud(n,msg)
    assert len(rendered)==1
    n.last_attempt=-float('inf')
    VirtualGripperCamera._cloud(n,msg)
    assert_invalid_display(n,states,stamp_ns=10_000_000_000,last_valid_stamp_ns=10_000_000_000)
    assert states[-1][1]['reason']=='rejected_stamp' and len(rendered)==1
    assert len(n.images.messages)==2 # accepted render, then one invalid-transition diagnostic
    assert len(n.infos.messages)==len(n.tf_publisher.messages)==1
    n.last_attempt=-float('inf')
    VirtualGripperCamera._cloud(n,msg)
    assert len(n.images.messages)==2 and len(rendered)==1 # no repeated blank pixels


def test_annotation_does_not_change_unknown_rgba_or_admit_overlay_as_depth():
    from so101_station_perception_tag_tracking.virtual_camera import tool_from_virtual_optical
    n,states,lookups,rendered=fixture_node()
    n.mount=tool_from_virtual_optical()
    pose=np.eye(4);pose[:3,3]=[.01,0,.1]
    n.pinch={'schema':'so101-virtual-pinch-overlay-v1','qualified':False,
        'parent_frame':'urdf:gripper_frame_link','parent_from_pinch':pose.tolist(), 'opening_angle_rad':-.06}
    n.opening_history.samples.append((10_000_000_000,-.06));n.pinch_sha='artifact-test'
    n.near,n.far=.01,2.
    n.annotated,n.annotation_masks=Publisher(),Publisher()
    rgba=np.zeros((240,320,4),np.uint8);original=rgba.copy()
    from sensor_msgs.msg import Image
    image=Image()
    result=VirtualGripperCamera._annotation(n,image,rgba,10_000_000_000)
    assert np.array_equal(rgba,original)
    assert result['state']=='nominal_uncalibrated' and result['annotation_is_depth'] is False
    assert result['pixel_xy'][0]<159.5  # Shared optical roll moves +toolX left.
    assert n.annotated.messages[0].encoding=='rgb8'
    assert n.annotation_masks.messages[0].encoding=='mono8'
    assert np.count_nonzero(np.frombuffer(n.annotation_masks.messages[0].data,dtype=np.uint8))>0
    n.opening_history.samples.clear();n.opening_history.samples.append((10_000_000_000,.1))
    result=VirtualGripperCamera._annotation(n,image,rgba,10_000_000_000)
    assert result['pixel_xy'] is None


def test_joint_receipt_rejects_future_sample_even_within_cloud_sync_window():
    from sensor_msgs.msg import JointState
    n,states,lookups,rendered=fixture_node()
    j=JointState();j.name=['gripper'];j.position=[-.06]
    j.header.stamp.sec=10;j.header.stamp.nanosec=120_000_000
    VirtualGripperCamera._joints(n,j)
    assert not n.opening_history.samples


def test_future_joint_within_50ms_sync_window_hides_crosshair_and_preserves_raw():
    n,states,lookups,rendered=fixture_node()
    pose=np.eye(4);pose[:3,3]=[.01,0,.1]
    n.pinch={'schema':'so101-virtual-pinch-overlay-v1','qualified':False,
        'parent_frame':'urdf:gripper_frame_link','parent_from_pinch':pose.tolist(),'opening_angle_rad':-.06}
    n.opening_history.samples.append((10_120_000_000,-.06));n.pinch_sha='future-test';n.near,n.far=.01,2.
    n.annotated,n.annotation_masks=Publisher(),Publisher()
    rgba=np.zeros((240,320,4),np.uint8)
    from sensor_msgs.msg import Image
    result=VirtualGripperCamera._annotation(n,Image(),rgba,10_100_000_000)
    assert result['state']=='opening_mismatch_or_stale' and result['pixel_xy'] is None
    assert np.count_nonzero(rgba)==0


def test_delayed_cloud_selects_historical_opening_instead_of_latest_joint_sample():
    n,states,lookups,rendered=fixture_node()
    pose=np.eye(4);pose[:3,3]=[.01,0,.1]
    n.pinch={'schema':'so101-virtual-pinch-overlay-v1','qualified':False,
        'parent_frame':'urdf:gripper_frame_link','parent_from_pinch':pose.tolist(),'opening_angle_rad':-.06}
    n.pinch_sha='history-test';n.near,n.far=.01,2.
    n.annotated,n.annotation_masks=Publisher(),Publisher()
    from sensor_msgs.msg import JointState, Image
    for stamp in (10_000_000_000,10_090_000_000):
        j=JointState();j.name=['gripper'];j.position=[-.06]
        j.header.stamp.sec,j.header.stamp.nanosec=divmod(stamp,10**9)
        VirtualGripperCamera._joints(n,j)
    rgba=np.zeros((240,320,4),np.uint8)
    result=VirtualGripperCamera._annotation(n,Image(),rgba,10_000_000_000)
    assert result['state']=='nominal_uncalibrated'
    assert result['joint_stamp_ns']==10_000_000_000 and result['joint_cloud_delta_ns']==0
    assert np.count_nonzero(rgba)==0
    # The temporally nearest opening changed: never search past it merely to
    # find the CAD artifact's desired angle.
    n.opening_history.samples.append((10_010_000_000,.1))
    result=VirtualGripperCamera._annotation(n,Image(),rgba,10_010_000_000)
    assert result['state']=='opening_mismatch_or_stale' and result['pixel_xy'] is None


def test_mesh_link_tfs_use_exact_cloud_stamp_and_scene_headers_remain_original():
    n,states,lookups,rendered=fixture_node()
    n.render_gripper_mesh=True
    n.meshes=[{'name':'fixed','link':'gripper_link'},{'name':'jaw','link':'moving_jaw_so101_v1_link'}]
    n.mesh_provenance={'state':'loaded','qualified':False}
    msg=cloud();VirtualGripperCamera._cloud(n,msg)
    assert len(lookups)==3
    assert {args[1] for args,kw in lookups}=={msg.header.frame_id,'gripper_link','moving_jaw_so101_v1_link'}
    assert all(args[2].nanoseconds==10_000_000_000 for args,kw in lookups)
    assert set(rendered[0][3])=={'fixed','jaw'}
    assert n.images.messages[0].header==n.scenes.messages[0].header==n.provenance_masks.messages[0].header
    assert states[-1][1]['model_visuals_rendered']==2


def test_browser_state_admission_shares_rgb_decoder_and_bounds_snapshot_color_payload():
    from workshop.digital_twin.ros_state_node import RosStatePublisher
    from workshop.digital_twin.point_cloud_samples import decode_rgb8
    msg=cloud(np.array([[0,0,.5]],dtype='<f4'))
    msg.point_step=20;msg.row_step=20
    msg.fields.append(PointField(name='rgb',offset=16,datatype=7,count=1))
    raw=bytearray(20);raw[:12]=np.array([0,0,.5],'<f4').tobytes()
    raw[16:20]=np.array([0x0061626d],'<u4').tobytes();msg.data=raw
    errors=[]
    n=NS(state=NS(point_cloud=None),get_logger=lambda:NS(warning=errors.append))
    RosStatePublisher._point_cloud(n,msg)
    assert not errors
    saved=n.state.point_cloud
    assert saved['color_source']=='packed_rgb' and len(saved['rgb8_b64'])==4
    assert decode_rgb8(saved['rgb8_b64'],1).tolist()==[[97,98,109]]
    assert saved['stamp_ns']==10_000_000_000 and len(saved['points_m'])==1


def test_observed_packed_rgb_passes_to_gpu_once_and_is_reported_without_uniform_fallback():
    n,states,lookups,rendered=fixture_node()
    msg=cloud(np.array([[0,0,.5]],'<f4'));msg.point_step=20;msg.row_step=20
    msg.fields.append(PointField(name='rgb',offset=16,datatype=7,count=1))
    raw=bytearray(20);raw[:12]=np.array([0,0,.5],'<f4').tobytes()
    raw[16:20]=np.array([0x0061626d],'<u4').tobytes();msg.data=raw
    VirtualGripperCamera._cloud(n,msg)
    assert len(rendered)==1 and rendered[0][2].tolist()==[[97,98,109]]
    assert states[-1][1]['color_source']=='packed_rgb'


def test_present_malformed_color_drops_before_gpu_without_uniform_substitution():
    n,states,lookups,rendered=fixture_node()
    msg=cloud();msg.fields.append(PointField(name='rgb',offset=8,datatype=2,count=1))
    VirtualGripperCamera._cloud(n,msg)
    assert_invalid_display(n,states)
    assert states[-1][1]['reason'].startswith('render_or_acquisition_tf_failed:')
    assert not rendered and not n.infos.messages and not n.tf_publisher.messages


def test_robot_description_mesh_work_runs_only_on_changed_description(monkeypatch):
    from std_msgs.msg import String
    import so101_station_perception.virtual_gripper_camera as module
    loads=[];uploads=[]
    monkeypatch.setattr(module,'load_gripper_visuals',lambda description,resolver:
        (loads.append(description) or [],{'qualified':False,'triangles':0}))
    n=NS(render_gripper_mesh=True,mesh_description=None,renderer=NS(set_meshes=uploads.append))
    msg=String();msg.data='<fixture description="one"/>'
    VirtualGripperCamera._description(n,msg);VirtualGripperCamera._description(n,msg)
    assert len(loads)==len(uploads)==1
    msg.data='<fixture description="two"/>'
    VirtualGripperCamera._description(n,msg)
    assert len(loads)==len(uploads)==2


def test_fault_after_valid_render_and_fresh_restore_use_real_transitions():
    from tf2_ros import LookupException
    n,states,lookups,rendered=fixture_node()
    VirtualGripperCamera._cloud(n,cloud())
    original_lookup=n.buffer.lookup_transform
    def missing(*args,**kwargs):raise LookupException('exact TF unavailable')
    n.buffer.lookup_transform=missing;n.last_attempt=-float('inf')
    VirtualGripperCamera._cloud(n,cloud(stamp_ns=10_010_000_000))
    assert_invalid_display(n,states,stamp_ns=10_000_000_000,last_valid_stamp_ns=10_000_000_000)
    assert len(rendered)==1 and len(n.infos.messages)==len(n.tf_publisher.messages)==1
    n.buffer.lookup_transform=original_lookup;n.last_attempt=-float('inf')
    fresh=cloud(stamp_ns=10_050_000_000);VirtualGripperCamera._cloud(n,fresh)
    assert states[-1][0]=='published' and states[-1][1]['visual_valid'] is True
    assert n.display_valid and not n.invalid_frame_sent
    assert n.last_valid_stamp_ns==n.last_valid_tool_tf_stamp_ns==10_050_000_000
    assert n.images.messages[-1].header.stamp==n.infos.messages[-1].header.stamp==fresh.header.stamp
    assert n.annotated.messages[-1].header==n.images.messages[-1].header
    assert len(n.images.messages)==3 and len(rendered)==2


def test_real_watchdog_blanks_once_on_receipt_expiry_without_tf_info_or_gpu(monkeypatch):
    import so101_station_perception.virtual_gripper_camera as module
    n,states,lookups,rendered=fixture_node();VirtualGripperCamera._cloud(n,cloud())
    accepted=n.last_valid_monotonic
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:accepted+.501))
    n._watchdog();n._watchdog()
    assert_invalid_display(n,states,stamp_ns=10_000_000_000,last_valid_stamp_ns=10_000_000_000)
    assert len(n.images.messages)==2 and len(n.annotated.messages)==2
    assert len(n.infos.messages)==len(n.tf_publisher.messages)==len(lookups)==len(rendered)==1
    assert states[-1][1]['reason']=='last_render_expired_or_clock_invalid'
