"""Offline geometry/provenance checks; no cameras or actuators are accessed."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip('scipy')

from workshop.digital_twin.scene import ASSETS, fit_table, make_model, unproject


def test_depth_unprojection_metres_and_invalid_mask():
    depth = np.array([[0, 500], [1000, 2000]], dtype=np.uint16)
    points, valid = unproject(depth, {'fx': 100, 'fy': 200, 'ppx': 0, 'ppy': 0, 'coeffs': [0] * 5}, .001)
    np.testing.assert_allclose(points[0, 1], [.005, 0, .5])
    np.testing.assert_allclose(points[1, 0], [0, .005, 1.])
    assert valid.tolist() == [[False, True], [True, False]]


def test_nonzero_distortion_is_not_silently_treated_as_pinhole():
    with pytest.raises(ValueError, match='distortion'):
        unproject(np.ones((3, 3)), {'fx': 100, 'fy': 100, 'coeffs': [.1, 0, 0, 0, 0]}, .001)


def test_table_fit_resists_outliers_and_produces_right_handed_frame():
    rng = np.random.default_rng(4)
    xy = rng.uniform(-.3, .3, (500, 2))
    plane = np.column_stack((xy, np.ones(500) + rng.normal(0, .001, 500)))
    points = np.vstack((plane, rng.uniform(-1, 1, (100, 3))))
    center, basis, evidence = fit_table(points, trials=100)
    assert abs(center[2] - 1) < .002
    assert basis[2, 2] < -.99  # Table up faces optical origin.
    np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-8)
    assert np.linalg.det(basis) > .999
    assert evidence['inlier_fraction'] > .8
    assert evidence['residual_p95_m'] < .003


def test_generated_chain_preserves_official_urdf_origins_axes():
    original = ET.parse(ASSETS / 'so101_new_calib.urdf').getroot()
    scene = ET.fromstring(make_model([.2, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    for joint in original.findall('joint'):
        if joint.get('type') == 'fixed': continue
        target = scene.find(".//joint[@name='" + joint.get('name') + "']")
        assert target is not None and target.get('axis') == joint.find('axis').get('xyz')
        body = scene.find(".//body[@name='" + joint.find('child').get('link').removesuffix('_link') + "']")
        np.testing.assert_allclose(np.fromstring(body.get('pos'), sep=' '), np.fromstring(joint.find('origin').get('xyz'), sep=' '))
    assert len(scene.findall('actuator/position')) == 6
    assert len(scene.findall('.//inertial')) == 7


def test_mujoco_compile_and_forward_pose():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import set_pose
    model = mujoco.MjModel.from_xml_string(make_model([.2, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    set_pose(model, data, dict(shoulder_pan=-10, shoulder_lift=-110, elbow_flex=97, wrist_flex=76, wrist_roll=2, gripper=4))
    assert model.njnt == 8 and model.nu == 6
    assert np.isfinite(data.geom_xpos).all()


def test_arm_base_fk_and_jacobian_do_not_depend_on_guessed_registration():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import export_fk, set_pose
    poses = []
    for base in ([.2, 0, 0], [1.1, -.3, .2]):
        model = mujoco.MjModel.from_xml_string(make_model(base, [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
        data = mujoco.MjData(model)
        set_pose(model, data, dict(shoulder_pan=-10, shoulder_lift=-110, elbow_flex=97, wrist_flex=76, wrist_roll=2, gripper=4))
        poses.append(export_fk(model, data))
    np.testing.assert_allclose(poses[0]['links']['gripper_frame_link']['arm_base_from_link'],
                               poses[1]['links']['gripper_frame_link']['arm_base_from_link'], atol=1e-10)
    np.testing.assert_allclose(poses[0]['gripper_position_jacobian_arm_base'],
                               poses[1]['gripper_position_jacobian_arm_base'], atol=1e-10)
    assert len(poses[0]['joints']) == 6
    assert len(poses[0]['jacobian_column_joint_names']) == 8
    assert poses[0]['joints']['shoulder_lift']['qpos_rad_unclipped'] < -1.9


def test_registration_recovers_asymmetric_surface_transform():
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.registration import fit_registration
    rng = np.random.default_rng(19)
    cad = rng.normal(size=(100, 3))*[.03, .015, .06]
    rotation = Rotation.from_euler('z', 135, degrees=True).as_matrix()
    actual = np.array([.15, .13, .01])
    observed = cad @ rotation.T + actual
    result = fit_registration(cad, observed, yaw_seeds=(120, 140))
    np.testing.assert_allclose(result['position_m'], actual, atol=.002)
    assert abs(result['yaw_deg']-135) < 2
    assert 'UNQUALIFIED' in result['status']


def test_rgbd_surface_does_not_bridge_missing_depth_or_long_edges():
    from workshop.digital_twin.surface import surface_mesh
    y, x = np.indices((3, 3))
    points = np.stack((x*.005, y*.005, x*0), axis=-1)
    valid = np.ones((3, 3), bool); valid[1, 1] = False
    vertices, faces, _ = surface_mesh(points, np.zeros((3, 3, 3), np.uint8), valid, step=1)
    assert not np.isin(faces, 4).any()
    points[0, 0] = [1, 0, 0]
    _, faces, _ = surface_mesh(points, np.zeros((3, 3, 3), np.uint8), np.ones((3, 3), bool), step=1)
    assert not np.isin(faces, 0).any()


def test_cached_cad_samples_match_uncached_and_follow_joint_motion():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import set_pose
    from workshop.digital_twin.registration import cad_samples, cad_surface, transform_samples
    model = mujoco.MjModel.from_xml_string(make_model([.2, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    pose = dict(shoulder_pan=-10, shoulder_lift=-88, elbow_flex=89, wrist_flex=73, wrist_roll=2, gripper=4)
    set_pose(model, data, pose)
    samples = cad_samples(model, count=500)
    original = transform_samples(model, data, samples)
    np.testing.assert_allclose(original, cad_surface(model, data, count=500))
    pose['shoulder_pan'] += 12; set_pose(model, data, pose)
    changed = transform_samples(model, data, samples)
    assert np.max(np.linalg.norm(changed-original, axis=1)) > .015


def test_patch_tracker_recovers_image_displacement_with_brightness_change():
    from workshop.digital_twin.tracking import track_patch
    rng = np.random.default_rng(9)
    image = rng.integers(0, 200, (80, 100, 3), dtype=np.uint8)
    shifted = np.zeros_like(image); shifted[4:, 7:] = np.minimum(image[:-4, :-7].astype(int)+15,255)
    reference, _ = track_patch(image,image,rectangle=(30,25,50,45),search=(20,15,70,65))
    target, score = track_patch(image,shifted,rectangle=(30,25,50,45),search=(20,15,70,65))
    np.testing.assert_allclose(np.array(target)-reference,[7,4]); assert score > .99


def test_explicit_frame_requires_rgb_and_depth_pair(tmp_path):
    from workshop.digital_twin.scene import prepare
    (tmp_path/'capture.json').write_text('{}')
    with pytest.raises(ValueError, match='paired image and depth'):
        prepare(tmp_path, tmp_path/'unused.json',tmp_path/'output',image_override=tmp_path/'image.png')


def test_nominal_fk_remains_separate_from_visual_fit_corrections():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import export_pose_pair
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    nominal=dict(shoulder_pan=-10,shoulder_lift=-88,elbow_flex=90,wrist_flex=73,wrist_roll=2,gripper=4)
    modeled=dict(nominal,wrist_flex=100)
    original,fitted=export_pose_pair(model,data,{'nominal_joint_degrees':nominal,'model_joint_degrees':modeled})
    assert original['joints']['wrist_flex']['qpos_rad_unclipped']==pytest.approx(np.deg2rad(73))
    assert fitted['joints']['wrist_flex']['qpos_rad_unclipped']==pytest.approx(np.deg2rad(100))
    assert 'visual-fit' in fitted['source']


def test_jaw_target_geometry_requires_valid_measured_depth():
    from workshop.digital_twin.tracking import jaw_target_observation
    image=np.zeros((60,80,3),np.uint8);image[15:35,25:35]=220;image[40:47,35:44]=[220,20,20]
    depth=np.full((60,80),500,np.uint16)
    metadata={'intrinsics':{'fx':100,'fy':100,'ppx':0,'ppy':0,'coeffs':[0]*5},'depth_scale_m':.001}
    result=jaw_target_observation(image,depth,metadata,(20,10,40,38),(30,38,50,50))
    assert result['red_minus_finger_camera_m'][1]>0
    assert result['red_minus_finger_camera_m'][2]==pytest.approx(0)
    depth[15:35,25:35]=0
    assert jaw_target_observation(image,depth,metadata,(20,10,40,38),(30,38,50,50)) is None


def test_native_old_convention_is_separate_and_preserves_actuators():
    mujoco=pytest.importorskip('mujoco')
    scene=make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0],convention='old')
    model=mujoco.MjModel.from_xml_string(scene)
    assert model.nu==6
    lift=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'shoulder_lift')
    np.testing.assert_allclose(np.rad2deg(model.jnt_range[lift]),[-190,10],atol=.001)


def test_virtual_pose_update_never_reparses_assets(monkeypatch):
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import set_pose
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    def unexpected(*args,**kwargs):raise AssertionError('asset IO on virtual pose update')
    monkeypatch.setattr(ET,'parse',unexpected)
    set_pose(model,data,dict(shoulder_pan=0,shoulder_lift=0,elbow_flex=0,wrist_flex=0,wrist_roll=0,gripper=50))


def test_cad_pinch_center_tracks_jaw_opening_while_official_site_is_fixed():
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import export_pinch_geometry,set_pose
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    pose=dict(shoulder_pan=0,shoulder_lift=-88,elbow_flex=90,wrist_flex=73,wrist_roll=2,gripper=3)
    set_pose(model,data,pose);closed=export_pinch_geometry(model,data)
    pose['gripper']=30;set_pose(model,data,pose);opened=export_pinch_geometry(model,data)
    np.testing.assert_allclose(closed['official_site_world_m'],opened['official_site_world_m'],atol=1e-10)
    assert opened['inner_face_gap_m']>closed['inner_face_gap_m']+.01
    assert np.linalg.norm(np.array(opened['pinch_center_world_m'])-closed['pinch_center_world_m'])>.005
    assert 'unqualified' in closed['status']


def test_measured_camera_low_level_frustum_preserves_optical_principal_point():
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin.scene import add_measured_camera
    root=ET.Element('mujoco');world=ET.SubElement(root,'worldbody')
    intrinsic=dict(fx=605.,fy=604.,ppx=327.,ppy=247.)
    add_measured_camera(world,intrinsic,640,480,[0,0,0],[0,1,0,0])
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    scene=mujoco.MjvScene(model,100);camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FIXED;camera.fixedcamid=0
    mujoco.mjv_updateScene(model,data,mujoco.MjvOption(),None,camera,mujoco.mjtCatBit.mjCAT_ALL,scene)
    gl=scene.camera[0]
    px=640*(gl.frustum_width-gl.frustum_center)/(2*gl.frustum_width)
    py=480*gl.frustum_top/(gl.frustum_top-gl.frustum_bottom)
    np.testing.assert_allclose([px,py],[327,247],atol=1e-3)


def test_point_to_plane_icp_refines_known_pose_with_hard_yaw_branch():
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.icp import point_to_plane_icp
    rng=np.random.default_rng(23);uv=rng.uniform(-.08,.08,(500,2))
    points=np.vstack((np.c_[uv,np.zeros(500)],np.c_[np.zeros(500),uv],np.column_stack((uv[:,0],np.zeros(500),uv[:,1]))))
    normals=np.repeat([[0,0,1],[1,0,0],[0,1,0]],500,axis=0)
    actual=np.array([.12,.14,.03]);observed=points@Rotation.from_euler('z',18,degrees=True).as_matrix().T+actual
    result=point_to_plane_icp(points,normals,observed,[.11,.13,.02],18,iterations=20)
    assert result['after_median_m']<result['before_median_m']*.2
    np.testing.assert_allclose(result['position_m'],actual,atol=.002)
    assert abs(result['yaw_deg']-18)<2
    assert abs(result['yaw_deg']-result['yaw_prior_deg'])<=30


def test_fixed_base_cad_sampling_is_independent_of_joint_pose():
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin.icp import oriented_cad_surface
    from workshop.digital_twin.kinematics import set_pose
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    set_pose(model,data,dict(shoulder_pan=0,shoulder_lift=-88,elbow_flex=90,wrist_flex=73,wrist_roll=2,gripper=3))
    first=oriented_cad_surface(model,data,count=300,body_name='arm_base')
    set_pose(model,data,dict(shoulder_pan=12,shoulder_lift=-65,elbow_flex=80,wrist_flex=60,wrist_roll=90,gripper=25))
    second=oriented_cad_surface(model,data,count=300,body_name='arm_base')
    for before,after in zip(first,second):np.testing.assert_allclose(before,after,atol=1e-10)
    with pytest.raises(ValueError,match='unknown CAD sampling body'):
        oriented_cad_surface(model,data,body_name='missing')


def test_fixed_base_transform_has_holdout_and_preserves_hard_orientation_prior():
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.icp import fit_fixed_base
    rng=np.random.default_rng(24);uv=rng.uniform(-.08,.08,(400,2))
    cad=np.vstack((np.c_[uv,np.zeros(400)],np.c_[np.zeros(400),uv],np.c_[uv[:,0],np.zeros(400),uv[:,1]]))
    normals=np.repeat([[0,0,1],[1,0,0],[0,1,0]],400,axis=0)
    actual=np.eye(4);actual[:3,:3]=Rotation.from_euler('z',18,degrees=True).as_matrix();actual[:3,3]=[.12,.14,.03]
    camera=np.eye(4);camera[:3,3]=[.02,.03,.04]
    observed=cad@actual[:3,:3].T+actual[:3,3]
    result=fit_fixed_base(cad,normals,observed,camera,[.11,.13,.02],18,iterations=20)
    np.testing.assert_allclose(result['camera_from_arm_base'],np.linalg.inv(camera)@actual,atol=.004)
    assert result['holdout']['samples']==600
    assert result['holdout']['median_m']<.002
    assert abs(result['fit']['yaw_deg']-18)<=30
    assert not result['global_ik_qualified']
    camera[0,0]=2
    with pytest.raises(ValueError,match='proper rigid transform'):
        fit_fixed_base(cad,normals,observed,camera,[.11,.13,.02],18)
