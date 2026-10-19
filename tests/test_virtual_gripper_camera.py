"""Synthetic camera geometry and GPU guards; never connect physical inputs."""
import ast
import inspect
import os
from pathlib import Path
import sys

import numpy as np
import pytest
from workshop.digital_twin.virtual_camera import (tool_from_virtual_optical,
    virtual_intrinsics, optical_projection)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_perception'))
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_gpu'))
from so101_station_perception.virtual_camera_gpu import GpuPointCamera


@pytest.mark.parametrize('axis', ['+X', '-X', '+Y', '-Y', '+Z', '-Z'])
def test_shared_virtual_mount_is_proper_optical_frame(axis):
    m = tool_from_virtual_optical(axis, .04)
    assert m[:3, :3].T @ m[:3, :3] == pytest.approx(np.eye(3))
    assert np.linalg.det(m[:3, :3]) == pytest.approx(1.)
    assert m[:3, 3] == pytest.approx(-m[:3, 2] * .04)
    assert np.cross(m[:3, 0], m[:3, 1]) == pytest.approx(m[:3, 2])


def test_optical_projection_matches_camera_info_pixels_and_clips_depth():
    w, h = 320, 240
    k = virtual_intrinsics(w, h)
    p = optical_projection(k, w, h, .01, 2.)
    points = np.array([[0, 0, .5, 1], [.07, -.04, .4, 1]])
    clip = points @ p.T
    ndc = clip[:, :3] / clip[:, 3:]
    pixels = np.column_stack(((ndc[:, 0] + 1)*w/2-.5, (1-ndc[:, 1])*h/2-.5))
    expected = points[:, :3] @ k.T
    assert pixels == pytest.approx(expected[:, :2]/expected[:, 2:], abs=1e-5)
    for z, expected_z in ((.01, -1.), (2., 1.)):
        clip = p @ [0, 0, z, 1]
        assert clip[2]/clip[3] == pytest.approx(expected_z, abs=1e-6)


@pytest.mark.parametrize('size', [(0, 240), (641, 240), (320, 481)])
def test_renderer_image_bounds_fail_before_context_creation(size):
    with pytest.raises(ValueError): GpuPointCamera(*size)


def test_gpu_hot_path_is_bounded_one_readback_no_per_point_loop():
    source = inspect.getsource(GpuPointCamera.render_layers)
    tree = ast.parse('if True:\n' + source)
    loops=[n for n in ast.walk(tree) if isinstance(n,(ast.For,ast.While))]
    assert len(loops)==1 and ast.unparse(loops[0].iter)=='mesh_matrices.items()'
    assert not any(isinstance(n,(ast.ListComp,ast.GeneratorExp)) for n in ast.walk(tree))
    assert source.count('glReadPixels(') == 1
    assert 'glFinish' not in source and 'glGetBufferSubData' not in source
    assert 'len(points) > self.max_points' in source
    init = inspect.getsource(GpuPointCamera.__init__)
    assert 'require_hardware_renderer' in init


def test_ros_feed_is_opt_in_and_process_local_gpu_guarded():
    source = (ROOT / 'ros2/src/so101_station_perception/launch/virtual_gripper_camera.launch.py').read_text()
    assert 'default_value="false"' in source and 'condition=IfCondition(enabled)' in source
    assert '"LIBGL_ALWAYS_SOFTWARE": "false"' in source
    assert '"LD_PRELOAD": rviz_preload_value()' in source


@pytest.mark.skipif(os.getenv('SO101_TEST_VIRTUAL_CAMERA_EGL') != '1', reason='opt-in GPU EGL qualification')
def test_hardware_egl_point_pixels_unknown_mask_frustum_and_orientation():
    r = GpuPointCamera()
    try:
        projection = optical_projection(virtual_intrinsics(), 320, 240)
        # Center, upper-right, behind eye, too close, beyond far, outside FOV.
        points = np.array([[0,0,.5], [.1,-.1,.5], [0,0,-.1],
                           [0,0,.001], [0,0,3.], [10,0,.5]], np.float32)
        image = r.render(points, projection)
        mask = image[:, :, 3]
        assert np.count_nonzero(mask) == 2
        y, x = np.nonzero(mask)
        assert np.any((x >= 159) & (x <= 160) & (y >= 119) & (y <= 120))
        assert np.any((x > 160) & (y < 120))
        assert set(np.unique(mask)) == {0, 255}
        assert np.all(image[mask == 0] == 0)
        assert np.count_nonzero(r.render(np.empty((0, 3)), projection)) == 0
    finally:r.close()


def pinch_artifact(xyz=(.02, -.01, .1)):
    matrix = np.eye(4);matrix[:3,3]=xyz
    return {'schema':'so101-virtual-pinch-overlay-v1', 'qualified':False,
        'parent_frame':'urdf:gripper_frame_link', 'parent_from_pinch':matrix.tolist(),
        'opening_angle_rad':-.06787391535533503}


def test_nominal_crosshair_projects_configured_off_axis_pinch_not_image_center():
    from workshop.digital_twin.virtual_camera import project_pinch_overlay
    artifact=pinch_artifact();k=virtual_intrinsics()
    uv,reason=project_pinch_overlay(artifact, np.eye(4), k, 320,240,
        opening_angle_rad=artifact['opening_angle_rad'],joint_stamp_ns=10**9,acquisition_stamp_ns=10**9,now_ns=10**9)
    expected=k @ np.array([.02,-.01,.1]);expected=expected[:2]/expected[2]
    assert uv==pytest.approx(expected) and reason=='nominal_uncalibrated'
    assert uv!=pytest.approx([159.5,119.5])


@pytest.mark.parametrize('xyz,angle,stamp,reason', [
    ((0,0,-.1),-.06787391535533503,10**9,'pinch_outside_depth_frustum'),
    ((1,0,.1),-.06787391535533503,10**9,'pinch_outside_image'),
    ((0,0,.1),0.,10**9,'opening_mismatch_or_stale'),
    ((0,0,.1),-.06787391535533503,900_000_000,'opening_mismatch_or_stale'),
    ((0,0,.1),-.06787391535533503,1_020_000_000,'opening_mismatch_or_stale')])
def test_crosshair_hides_for_behind_outside_opening_change_and_stale_pose(xyz,angle,stamp,reason):
    from workshop.digital_twin.virtual_camera import project_pinch_overlay
    uv,observed=project_pinch_overlay(pinch_artifact(xyz),np.eye(4),virtual_intrinsics(),320,240,
        opening_angle_rad=angle,joint_stamp_ns=stamp,acquisition_stamp_ns=10**9,now_ns=10**9)
    assert uv is None and observed==reason


def test_station_rviz_exposes_nominal_camera_with_matching_sensor_qos_and_raw_opt_in():
    import yaml
    config=yaml.safe_load((ROOT/'ros2/src/so101_station_perception/config/station_live.rviz').read_text())
    displays={d['Name']:d for d in config['Visualization Manager']['Displays']}
    annotated=displays['NOMINAL Virtual Gripper Camera']
    raw=displays['Virtual Gripper Camera raw samples (alpha means observed)']
    assert annotated['Class']=='rviz_default_plugins/Image' and annotated['Enabled'] is True
    assert annotated['Topic']['Value']=='/so101/synthetic_gripper_camera/image_annotated'
    assert raw['Topic']['Value']=='/so101/synthetic_gripper_camera/image_raw' and raw['Enabled'] is False
    for display in (annotated,raw):
        assert display['Topic']['Reliability Policy']=='Best Effort'
        assert display['Topic']['Durability Policy']=='Volatile'
        assert display['Topic']['Depth']==2


def test_xyz_decoder_and_cloud_callback_have_no_per_point_python_work():
    source=(ROOT/'ros2/src/so101_station_perception/so101_station_perception/virtual_gripper_camera.py').read_text()
    tree=ast.parse(source)
    decoder=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='sampled_xyz')
    assert not any(isinstance(n,(ast.For,ast.While,ast.ListComp,ast.DictComp,ast.GeneratorExp)) for n in ast.walk(decoder))
    assert ast.unparse(decoder.body[-1].value)=='sampled_cloud(message, max_points)[0]'
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='VirtualGripperCamera')
    callback=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_cloud')
    loops=[ast.unparse(n.iter) for n in ast.walk(callback) if isinstance(n,ast.For)]
    assert sorted(loops)==sorted(['LINKS','(scene_image, provenance_image)'])
    assert not any(isinstance(n,(ast.While,ast.ListComp,ast.SetComp,ast.GeneratorExp)) for n in ast.walk(callback))
    comps=[n for n in ast.walk(callback) if isinstance(n,ast.DictComp)]
    assert len(comps)==1 and ast.unparse(comps[0].generators[0].iter)=='self.meshes'
    calls=[ast.unparse(n.func) for n in ast.walk(callback) if isinstance(n,ast.Call)]
    assert calls.count('sampled_cloud')==1 and calls.count('self.renderer.render_layers')==1
    assert not any(name in {'open','Path','Rotation.from_matrix','load_gripper_visuals'} or
        name.startswith(('hashlib.','mujoco.')) or name.endswith(('.read_bytes','.read_text')) for name in calls)


def test_joint_history_handles_delayed_cloud_and_out_of_order_receipts():
    from workshop.digital_twin.virtual_camera import JointOpeningHistory
    h=JointOpeningHistory();now=10_100_000_000
    for stamp,angle in ((10_000_000_000,-.06),(10_090_000_000,-.06),(9_990_000_000,-.07)):
        assert h.add(stamp,angle,now)
    assert h.nearest(10_000_000_000,now)==(10_000_000_000,-.06)
    assert h.nearest(9_990_000_000,now)==(9_990_000_000,-.07)
    assert h.nearest(9_800_000_000,now) is None
    # Timestamp matching happens before opening matching.
    assert h.add(10_010_000_000,.1,now)
    assert h.nearest(10_010_000_000,now)==(10_010_000_000,.1)


def test_joint_history_rejects_future_old_nonfinite_samples_and_stale_clouds():
    from workshop.digital_twin.virtual_camera import JointOpeningHistory
    h=JointOpeningHistory();now=10_100_000_000
    assert h.add(10_000_000_000,-.06,now)
    for stamp,angle in ((10_120_000_000,-.06),(9_000_000_000,-.06),(0,-.06),(10_010_000_000,np.nan)):
        assert not h.add(stamp,angle,now)
    assert len(h.samples)==1
    assert h.nearest(10_000_000_000,10_600_000_001) is None
    assert h.nearest(10_120_000_000,now) is None
    # Previously admitted samples also become inadmissible on a clock rewind.
    assert h.nearest(9_990_000_000,9_990_000_000) is None


def test_joint_history_capacity_and_static_hot_path_bound():
    from workshop.digital_twin.virtual_camera import JointOpeningHistory
    h=JointOpeningHistory();now=10_100_000_000
    for i in range(100):assert h.add(10_000_000_000+i*1_000_000,-.06,now)
    assert len(h.samples)==h.samples.maxlen==64
    assert h.samples[0][0]==10_036_000_000
    source=inspect.getsource(JointOpeningHistory)
    assert 'deque(maxlen=self.CAPACITY)' in source and 'CAPACITY = 64' in source
    add_tree=ast.parse('if True:\n'+inspect.getsource(JointOpeningHistory.add))
    assert not any(isinstance(n,(ast.For,ast.While,ast.ListComp,ast.SetComp,
                                ast.DictComp,ast.GeneratorExp)) for n in ast.walk(add_tree))
    assert 'MAX_AGE_NS = 500_000_000' in source and 'MAX_SYNC_NS = 50_000_000' in source


@pytest.mark.skipif(os.getenv('SO101_TEST_VIRTUAL_CAMERA_EGL') != '1',reason='opt-in GPU EGL qualification')
def test_gpu_observed_rgb_depth_and_nominal_triangle_provenance_preserve_raw_cloud():
    r=GpuPointCamera()
    try:
        p=optical_projection(virtual_intrinsics(),320,240)
        points=np.array([[0,0,.5],[.1,0,.5],[0,0,.8]],np.float32)
        colors=np.array([[255,0,0],[0,0,255],[0,255,0]],np.uint8)
        triangle=np.array([[-.025,-.025,.25],[.025,-.025,.25],[0,.025,.25]],np.float32)
        r.set_meshes([{'name':'nominal/visual_0','points':triangle,
                       'colors':np.full((3,3),180,np.uint8)}])
        cloud=r.render(points,p,colors=colors)
        layers=r.render_layers(points,p,colors=colors,mesh_matrices={'nominal/visual_0':p})
        assert np.array_equal(layers['raw'],cloud)
        red=np.argwhere(np.all(cloud[:,:,:3]==[255,0,0],axis=2)&(cloud[:,:,3]==255))
        blue=np.argwhere(np.all(cloud[:,:,:3]==[0,0,255],axis=2)&(cloud[:,:,3]==255))
        assert len(red)==len(blue)==1
        assert not np.any(np.all(cloud[:,:,:3]==[0,255,0],axis=2))
        y,x=red[0];assert layers['provenance'][y,x]==2
        assert layers['scene'][y,x].tolist()==[180,180,180,255]
        y,x=blue[0];assert layers['provenance'][y,x]==1
        assert layers['scene'][y,x].tolist()==[0,0,255,255]
        assert set(np.unique(layers['provenance']))=={0,1,2}
        assert np.all(layers['raw'][layers['raw'][:,:,3]==0]==0)
        assert r.readback_bytes==614400
    finally:r.close()
