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


def test_prepare_skips_rgb_arm_fit_when_explicit_full_se3_is_supplied(tmp_path, monkeypatch):
    import json
    from PIL import Image
    from workshop.digital_twin import registration as registration_module
    from workshop.digital_twin.scene import prepare

    snapshot = tmp_path / 'snapshot'
    output = tmp_path / 'output'
    snapshot.mkdir()
    height, width = 480, 640
    (snapshot / 'capture.json').write_text(json.dumps({
        'intrinsics': {'fx': 600., 'fy': 600., 'ppx': width / 2, 'ppy': height / 2, 'coeffs': [0] * 5},
        'depth_scale_m': .001}))
    Image.fromarray(np.full((height, width, 3), 2, dtype=np.uint8)).save(snapshot / 'color.png')
    np.save(snapshot / 'aligned-depth.npy', np.full((height, width), 500, dtype=np.uint16))
    raw = {'shoulder_pan': 2048, 'shoulder_lift': 2048, 'elbow_flex': 2048,
           'wrist_flex': 2048, 'wrist_roll': 2048, 'gripper': 2000}
    (snapshot / 'arm-readonly.json').write_text(json.dumps({'raw_position': raw}))
    ranges = {name: {'range_min': 0, 'range_max': 4095} for name in raw}
    (tmp_path / 'calibration.json').write_text(json.dumps(ranges))
    explicit = {'world_from_camera': np.eye(4).tolist(),
                'world_from_base': np.eye(4).tolist(), 'qualified': False,
                'status': 'PROVISIONAL_UNQUALIFIED'}
    def unexpected_fit(*args, **kwargs):
        raise AssertionError('dark RGB must not trigger a surface fit when full SE(3) is supplied')
    monkeypatch.setattr(registration_module, 'fit_registration', unexpected_fit)

    report = prepare(snapshot, tmp_path / 'calibration.json', output,
                     pose_series=False, external_camera_arm_registration=explicit)

    assert report['arm']['registration']['status'] == 'SKIPPED_RGB_FIT_EXPLICIT_SE3_UNQUALIFIED'
    assert 'skipped' in report['arm']['registration']['method'].lower()
    assert report['arm']['registration']['qualified'] is False
    expected_base = (np.asarray(report['world_from_camera'])
                     @ np.linalg.inv(np.asarray(explicit['world_from_camera']))
                     @ np.asarray(explicit['world_from_base']))
    np.testing.assert_allclose(report['arm']['position_estimate_m'], expected_base[:3, 3])
    assert (output / 'scene.xml').is_file()


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


def test_pinch_site_offset_is_invariant_to_global_arm_rotation():
    mujoco=pytest.importorskip('mujoco')
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.kinematics import export_pinch_geometry,set_pose
    xml=make_model([0,0,0],[1,0,0],[0,0,1],[1,0,0,0])
    pose=dict(shoulder_pan=23,shoulder_lift=-35,elbow_flex=41,wrist_flex=-21,wrist_roll=15,gripper=20)
    arm=mujoco.mjtObj.mjOBJ_BODY
    offsets=[]
    for quat in ([1,0,0,0],Rotation.from_euler('xyz',[22,-13,39],degrees=True).as_quat()[[3,0,1,2]]):
        model=mujoco.MjModel.from_xml_string(xml)
        data=mujoco.MjData(model)
        base=mujoco.mj_name2id(model,arm,'arm_base')
        model.body_quat[base]=quat
        set_pose(model,data,pose)
        offsets.append(export_pinch_geometry(model,data)['pinch_center_gripper_site_m'])
    np.testing.assert_allclose(offsets[0],offsets[1],atol=1e-8)


def test_pinch_patches_keep_canonical_mesh_vertices_through_full_jaw_sweep():
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin.kinematics import export_pinch_geometry,set_pose
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    pose=dict(shoulder_pan=0,shoulder_lift=-88,elbow_flex=90,wrist_flex=73,wrist_roll=2,gripper=0)
    outputs=[]
    for opening in (0,20,40,60,80,100):
        pose['gripper']=opening
        set_pose(model,data,pose)
        outputs.append(export_pinch_geometry(model,data))
    for kind in ('fixed','moving'):
        indices=outputs[0]['contacts'][kind]['selected_vertex_indices']
        for result in outputs[1:]:
            assert result['contacts'][kind]['selected_vertex_indices']==indices
    # The point patch is transformed from the same mesh vertices at every pose.
    for opening,result in zip((0,20,40,60,80,100),outputs):
        pose['gripper']=opening
        set_pose(model,data,pose)
        for kind in ('fixed','moving'):
            contact=result['contacts'][kind]
            geom=contact['geom_id']; mesh=int(model.geom_dataid[geom])
            start=int(model.mesh_vertadr[mesh])
            local_vertices=model.mesh_vert[start+np.asarray(contact['selected_vertex_indices'])]
            xmat=data.geom_xmat[geom].reshape(3,3)
            world_vertices=local_vertices@xmat.T+data.geom_xpos[geom]
            np.testing.assert_allclose(contact['selected_vertices_world_m'],world_vertices,atol=1e-10)
            local_median=np.median(local_vertices,axis=0)
            expected_world=local_median@xmat.T+data.geom_xpos[geom]
            np.testing.assert_allclose(contact['world_position_m'],expected_world,atol=1e-10)
            body=int(model.geom_bodyid[geom]); body_rotation=data.xmat[body].reshape(3,3)
            body_vertices=(world_vertices-data.xpos[body])@body_rotation
            np.testing.assert_allclose(contact['selected_vertices_body_m'],body_vertices,atol=1e-10)
            np.testing.assert_allclose(contact['jaw_body_position_m'],
                                       (contact['world_position_m']-data.xpos[body])@body_rotation,atol=1e-10)
    gaps=np.array([result['inner_face_gap_m'] for result in outputs])
    # A closing pivot arc naturally changes the gap by centimetres per 20%;
    # its increments stay smooth without a patch-switch discontinuity.
    assert np.max(np.abs(np.diff(gaps, n=2)))<.01
    assert outputs[0]['geometry_status']=='OBSERVED_CAD_ESTIMATE'
    assert outputs[0]['physical_calibration_status']=='UNQUALIFIED'
    np.testing.assert_allclose([result['gripper_opening_percent_current'] for result in outputs],
                               [0,20,40,60,80,100],atol=1e-5)
    assert outputs[0]['contact_eligibility']['basis']=='CAD face normals and opposing-contact direction; not physical calibration'
    assert [result['contact_eligibility']['eligible'] for result in outputs]==[True,True,False,False,False,False]
    for result in outputs:
        for jaw in ('fixed','moving'):
            normal=np.asarray(result['contacts'][jaw]['normal_world'])
            assert np.linalg.norm(normal)==pytest.approx(1,abs=1e-6)
            site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
            np.testing.assert_allclose(result['contacts'][jaw]['normal_site'],
                                       data.site_xmat[site].reshape(3,3).T@normal,atol=1e-10)


def test_pinch_export_reuses_cached_cad_patch_without_allocating_mesh_data(monkeypatch):
    mujoco=pytest.importorskip('mujoco')
    from workshop.digital_twin import kinematics
    model=mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    data=mujoco.MjData(model)
    kinematics.set_pose(model,data,dict(gripper=0))
    kinematics.export_pinch_geometry(model,data)
    patches=kinematics._canonical_pinch_patches(model)
    def unexpected(_):
        raise AssertionError('per-frame CAD mesh data allocation')
    monkeypatch.setattr(kinematics.mujoco,'MjData',unexpected)
    for opening in range(0,101,10):
        kinematics.set_pose(model,data,dict(gripper=opening))
        kinematics.export_pinch_geometry(model,data)
        assert kinematics._canonical_pinch_patches(model) is patches


def test_provisional_camera_extrinsics_compose_full_se3_and_flip_board_vertical(tmp_path):
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.evidence_bridge import compose_camera_arm_registration
    def transform(translation, angles):
        result=np.eye(4)
        result[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
        result[:3,3]=translation
        return result
    camera_from_board=transform([-.04,.12,.51],[12,-8,25])
    base_from_camera=transform([.31,-.52,.19],[92,-5,32])
    result=compose_camera_arm_registration(camera_from_board,base_from_camera)
    target_from_world=np.diag([1.,-1.,-1.,1.])
    expected_world_from_camera=np.linalg.inv(camera_from_board@target_from_world)
    expected_world_from_base=expected_world_from_camera@np.linalg.inv(base_from_camera)
    np.testing.assert_allclose(result['world_from_camera'],expected_world_from_camera,atol=1e-10)
    np.testing.assert_allclose(result['world_from_base'],expected_world_from_base,atol=1e-10)
    assert not np.allclose(np.asarray(result['world_from_base'])[:3,:3],np.eye(3))
    assert result['qualified'] is False
    assert 'provisional' in result['status'].lower()


def test_phase_evidence_marks_last_raw_pose_stale_after_half_second(tmp_path):
    import json
    from workshop.digital_twin.evidence_bridge import read_phase_evidence
    path=tmp_path/'phase.json'
    path.write_text(json.dumps({'phase':'leg_settled_home','pose_raw':{'shoulder_pan':1999},'monotonic':12.0}))
    fresh=read_phase_evidence(path,now_monotonic=12.5)
    stale=read_phase_evidence(path,now_monotonic=12.501)
    assert fresh['freshness']=='fresh' and fresh['pose_raw']['shoulder_pan']==1999
    assert stale['freshness']=='stale' and stale['pose_raw']['shoulder_pan']==1999
    assert stale['age_seconds']==pytest.approx(.501)


def test_raw_phase_pose_uses_saved_calibration_mapping_without_touching_devices():
    from workshop.digital_twin.evidence_bridge import raw_pose_to_joint_degrees
    calibration={name:{'range_min':0,'range_max':4095} for name in
                 ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')}
    converted=raw_pose_to_joint_degrees({'shoulder_pan':1999,'gripper':1935},calibration)
    assert converted['shoulder_pan']==pytest.approx((1999-2047.5)*360/4095)
    assert converted['gripper']==pytest.approx(1935*100/4095)


def test_scene_frame_delta_reexpresses_camera_points_in_board_up_frame():
    from scipy.spatial.transform import Rotation
    from workshop.digital_twin.evidence_bridge import scene_from_camera_world_delta
    scene_world_from_camera=np.eye(4);scene_world_from_camera[:3,3]=[.3,-.2,.1]
    board_world_from_camera=np.eye(4)
    board_world_from_camera[:3,:3]=Rotation.from_euler('xyz',[.1,-.2,.5]).as_matrix()
    board_world_from_camera[:3,3]=[-.1,.4,.2]
    delta=scene_from_camera_world_delta(scene_world_from_camera,board_world_from_camera)
    np.testing.assert_allclose(delta@scene_world_from_camera,board_world_from_camera,atol=1e-10)
    point_scene=np.array([.2,.1,.3,1.])
    point_camera=np.linalg.inv(scene_world_from_camera)@point_scene
    np.testing.assert_allclose(delta@point_scene,board_world_from_camera@point_camera,atol=1e-10)


def test_camera_sample_freshness_requires_matching_pose_timestamp():
    from workshop.digital_twin.evidence_bridge import camera_sample_status
    now_wall=1000.;now_monotonic=200.
    report={'camera':{'timestamp':999.8}}
    fresh_phase={'event_monotonic':199.8}
    stale_phase={'event_monotonic':198.9}
    assert camera_sample_status(report,fresh_phase,now_wall,now_monotonic)['freshness']=='fresh'
    result=camera_sample_status(report,stale_phase,now_wall,now_monotonic,cache_reusable=True)
    assert result['freshness']=='stale'
    assert result['pointcloud_cache_reusable'] is True
    assert result['pose_sample_delta_seconds']==pytest.approx(.9)


def test_camera_sample_uses_capture_monotonic_age_for_recorded_keyframes():
    from workshop.digital_twin.evidence_bridge import camera_sample_status
    report={'camera_sample_timestamp_wall':1000.,'camera_sample_timestamp_monotonic':50.,
            'camera':{'timestamp':1000.}}
    phase={'event_monotonic':49.97}
    status=camera_sample_status(report,phase,now_wall=1100.,now_monotonic=52.)
    assert status['freshness']=='stale'
    assert status['age_seconds']==pytest.approx(2.)
    assert status['pose_sample_delta_seconds']==pytest.approx(.03)


def test_pointcloud_cache_provenance_rejects_changed_camera_or_depth(tmp_path):
    import json
    from workshop.digital_twin.evidence_bridge import pointcloud_cache_status,write_pointcloud_provenance
    image=tmp_path/'camera.png';depth=tmp_path/'depth.npy';output=tmp_path/'out';output.mkdir()
    image.write_bytes(b'rgb-frame');depth.write_bytes(b'depth-frame')
    (output/'pointcloud.npz').write_bytes(b'points')
    (output/'measured-surface.npz').write_bytes(b'surface')
    report={'image':str(image),'depth':str(depth),'camera':{'timestamp':123.},
            'scene_world_from_camera':np.eye(4).tolist(),
            'camera':{'timestamp':123.,'intrinsics':{'fx':10.,'fy':10.},'depth_scale_m':.001}}
    write_pointcloud_provenance(report,output)
    assert pointcloud_cache_status(report,output)['reusable'] is True
    report['scene_world_from_camera'][0][3]=.01
    assert pointcloud_cache_status(report,output)['reusable'] is False
    report['scene_world_from_camera'][0][3]=0.
    image.write_bytes(b'new-rgb-frame')
    assert pointcloud_cache_status(report,output)['reusable'] is False


def test_latest_saved_rgbd_pair_uses_paired_newest_camera_files(tmp_path):
    from workshop.digital_twin.evidence_bridge import select_latest_rgbd_pair
    (tmp_path/'color.png').write_bytes(b'old-rgb');(tmp_path/'aligned-depth.npy').write_bytes(b'old-depth')
    folder=tmp_path/'initial-align-retry'/'frames';folder.mkdir(parents=True)
    rgb=folder/'newest.png';depth=folder/'newest.npy'
    rgb.write_bytes(b'new-rgb');depth.write_bytes(b'new-depth')
    pair=select_latest_rgbd_pair(tmp_path)
    assert pair['image']==rgb and pair['depth']==depth
    depth.unlink()
    pair=select_latest_rgbd_pair(tmp_path)
    assert pair['image']==tmp_path/'color.png'


def test_saved_keyframe_manifest_selects_newest_synchronized_pair(tmp_path):
    import json
    from workshop.digital_twin.evidence_bridge import select_latest_keyframe_pair
    frames=tmp_path/'keyframes';frames.mkdir()
    records=[]
    for index, synchronized in ((44, True),(45, True),(46, False)):
        image=frames/f'{index:06d}.png';depth=frames/f'{index:06d}.npy'
        image.write_bytes(b'rgb');depth.write_bytes(b'depth')
        records.append({'index':index,'color':image.name,'depth':depth.name,
                        'monotonic':100.+index,'utc':f'2026-10-14T15:01:{index:02d}+00:00',
                        'phase':'pose_step_complete_','phase_event_monotonic':100.+index-.03,
                        'phase_event_delta_s':.03,'pose_synchronized':synchronized,
                        'pose_raw':{'shoulder_pan':1999}})
    (frames/'manifest.jsonl').write_text(''.join(json.dumps(record)+'\n' for record in records))
    pair=select_latest_keyframe_pair(frames)
    assert pair['image']==frames/'000045.png'
    assert pair['depth']==frames/'000045.npy'
    assert pair['pose_raw']=={'shoulder_pan':1999}
    assert pair['pose_synchronized'] is True
    assert pair['phase_event_delta_s']==pytest.approx(.03)


def test_full_board_se3_places_native_arm_base_and_reexpresses_scene():
    from scipy.spatial.transform import Rotation
    import mujoco
    from workshop.digital_twin.evidence_bridge import apply_world_registration,compose_camera_arm_registration
    scene=ET.fromstring(make_model([.2,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0]))
    ET.SubElement(scene.find('worldbody'),'camera',name='measured_camera',pos='.2 -.5 .3',quat='1 0 0 0')
    model=mujoco.MjModel.from_xml_string(ET.tostring(scene,encoding='unicode'))
    data=mujoco.MjData(model)
    old_world_from_camera=np.eye(4)
    camera_from_board=np.eye(4)
    camera_from_board[:3,:3]=Rotation.from_euler('xyz',[13,-17,8],degrees=True).as_matrix()
    camera_from_board[:3,3]=[-.1,.2,.4]
    base_from_camera=np.eye(4)
    base_from_camera[:3,:3]=Rotation.from_euler('xyz',[90,2,-4],degrees=True).as_matrix()
    base_from_camera[:3,3]=[.3,-.5,.2]
    registration=compose_camera_arm_registration(camera_from_board,base_from_camera)
    ev3=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'ev3_estimated_root')
    old_ev3=model.body_pos[ev3].copy()
    camera=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'measured_camera')
    old_camera=model.cam_pos[camera].copy()
    delta=apply_world_registration(model,data,registration,old_world_from_camera)
    arm=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'arm_base')
    np.testing.assert_allclose(model.body_pos[arm],np.asarray(registration['world_from_base'])[:3,3],atol=1e-10)
    np.testing.assert_allclose(model.body_quat[arm][[1,2,3,0]],
                               Rotation.from_matrix(np.asarray(registration['world_from_base'])[:3,:3]).as_quat(),atol=1e-8)
    np.testing.assert_allclose(model.body_pos[ev3],delta[:3,:3]@old_ev3+delta[:3,3],atol=1e-10)
    np.testing.assert_allclose(model.cam_pos[camera],delta[:3,:3]@old_camera+delta[:3,3],atol=1e-10)


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
