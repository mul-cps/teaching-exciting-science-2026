"""RGB-D table-frame estimation and official-URDF kinematic scene generation."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ASSETS = Path(__file__).parent / 'assets'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def unproject(depth, intrinsics, scale, near=.15, far=1.5):
    if depth.ndim != 2 or not np.isfinite(scale) or scale <= 0:
        raise ValueError('2-D depth and positive measured depth scale required')
    if any(intrinsics.get('coeffs', [])):
        raise ValueError('nonzero distortion requires calibrated RealSense deprojection')
    fx, fy = intrinsics['fx'], intrinsics['fy']
    if not all(np.isfinite(v) and v > 0 for v in (fx, fy)):
        raise ValueError('positive focal lengths required')
    if not all(np.isfinite(intrinsics[key]) for key in ('ppx', 'ppy')):
        raise ValueError('finite optical centers required')
    y, x = np.indices(depth.shape)
    z = depth.astype(np.float64) * scale
    points = np.stack(((x - intrinsics['ppx']) * z / fx,
                       (y - intrinsics['ppy']) * z / fy, z), axis=-1)
    valid = np.isfinite(z) & (z > near) & (z < far)
    return points, valid


def fit_table(points, threshold=.008, trials=240, seed=17):
    """Robust dominant plane in an explicitly selected tabletop region."""
    points = np.asarray(points, dtype=float)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 30:
        raise ValueError('not enough valid tabletop depth samples')
    rng = np.random.default_rng(seed)
    best = np.zeros(len(points), dtype=bool)
    for _ in range(trials):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        mask = np.abs((points - a) @ (normal / norm)) < threshold
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < max(30, len(points) * .35):
        raise ValueError('tabletop region does not contain a dominant plane')
    for _ in range(3):
        center = points[best].mean(axis=0)
        normal = np.linalg.svd(points[best] - center, full_matrices=False)[2][-1]
        best = np.abs((points - center) @ normal) < threshold
    center = points[best].mean(axis=0)
    if normal @ center > 0:  # Up points toward the optical origin.
        normal = -normal
    x_axis = np.array([1., 0., 0.])
    x_axis -= (x_axis @ normal) * normal
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-8: raise ValueError('table normal is parallel to optical horizontal axis')
    x_axis /= x_norm
    y_axis = np.cross(normal, x_axis)
    basis = np.column_stack((x_axis, y_axis, normal))
    residuals = np.abs((points[best] - center) @ normal)
    return center, basis, {'sample_count': len(points), 'inlier_count': int(best.sum()),
                          'inlier_fraction': float(best.mean()),
                          'residual_median_m': float(np.median(residuals)),
                          'residual_p95_m': float(np.percentile(residuals, 95)),
                          'threshold_m': threshold, 'method': 'seeded RANSAC + SVD; manually selected image ROI'}


def xyz(text):
    return np.fromstring(text, sep=' ')


def nums(values):
    return ' '.join(f'{v:.9g}' for v in values)


def quat(rpy):
    q = Rotation.from_euler('xyz', rpy).as_quat()
    return np.r_[q[3], q[:3]]


def make_model(arm_position, sorter_position, camera_position, camera_rotation, convention="new"):
    """Layer the scene around the pinned official native SO101 MuJoCo model.

    Preserve supplied meshes, joints, collision shapes, inertials and actuators.
    Only mount placement, actual white-plastic color and added scene objects differ.
    """
    if convention not in ('new','old'): raise ValueError('unknown official calibration convention')
    root = ET.parse(ASSETS / f'so101_{convention}_calib.xml').getroot()
    root.set('model', 'RGB-D station digital twin / official SO101')
    root.find('compiler').set('meshdir', str((ASSETS / 'cad').resolve()))
    option = root.find('option')
    if option is None: option = ET.SubElement(root, 'option')
    option.set('timestep', '.002')
    visual = root.find('visual')
    if visual is None: visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720')
    ET.SubElement(visual, 'headlight', diffuse='.7 .7 .7', ambient='.3 .3 .3')
    for material in root.findall('asset/material'):
        if 'sts3215' not in material.get('name', ''): material.set('rgba', '.83 .85 .82 1')
    world = root.find('worldbody')
    arm = world.find("body[@name='base']")
    arm.set('name', 'arm_base'); arm.set('pos', nums(arm_position))
    # Name every visual for browser synchronization; source unnamed collisions remain.
    for index, geom in enumerate(arm.findall('.//geom')):
        if geom.get('name') is None: geom.set('name', 'official_arm_geom_' + str(index))
    gripper = arm.find(".//body[@name='gripper']")
    site = gripper.find("site[@name='gripperframe']")
    if site is None: site = gripper.find("site[@name='gripper']")
    ET.SubElement(gripper, 'site', name='gripper_frame_site', pos=site.get('pos'), quat=site.get('quat'), size='.004', rgba='1 .5 .1 1')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 0 -1', diffuse='.8 .8 .8')
    ET.SubElement(world, 'geom', name='estimated_table_extent', type='box', size='.65 .5 .015', pos='0 0 -.018', rgba='.10 .14 .18 1', group='3')
    sorter = ET.SubElement(world, 'body', name='ev3_estimated_root', pos=nums(sorter_position))
    ET.SubElement(sorter, 'geom', name='sorter_bed_estimated', type='box', size='.21 .065 .04', pos='0 0 .06', rgba='.30 .34 .36 1', group='3')
    ET.SubElement(sorter, 'geom', name='sorter_rail_estimated', type='box', size='.23 .012 .012', pos='0 .06 .15', rgba='.15 .17 .18 1', group='3')
    carriage = ET.SubElement(sorter, 'body', name='sorter_carriage', pos='-.16 0 .15')
    ET.SubElement(carriage, 'joint', name='ev3_carriage_estimated', type='slide', axis='1 0 0', limited='true', range='0 .32')
    ET.SubElement(carriage, 'geom', name='carriage_estimated', type='box', size='.045 .055 .06', rgba='.12 .25 .32 1', group='3')
    feed = ET.SubElement(carriage, 'body', name='ev3_feeder', pos='0 -.055 .065')
    ET.SubElement(feed, 'joint', name='ev3_feeder_estimated', type='hinge', axis='1 0 0', limited='false')
    ET.SubElement(feed, 'geom', name='feed_pusher_estimated', type='box', size='.032 .014 .055', pos='0 0 .05', rgba='.13 .3 .45 1', group='3')
    camera = ET.SubElement(world, 'body', name='realsense_optical_frame', pos=nums(camera_position), quat=nums(camera_rotation))
    ET.SubElement(camera, 'geom', name='d435i_estimated_housing', type='box', size='.045 .012 .012', rgba='.22 .25 .29 1', group='3')
    return ET.tostring(root, encoding='unicode')


def add_measured_camera(world, intrinsics, width, height, position, rotation):
    # MuJoCo's sensor-plane offset has the opposite sign to image principal
    # displacement; camera +Y points up while optical image +Y points down.
    return ET.SubElement(world,'camera',name='measured_camera',pos=nums(position),quat=nums(rotation),
                         resolution=f'{width} {height}',sensorsize=f'{width} {height}',
                         focalpixel=f"{intrinsics['fx']} {intrinsics['fy']}",
                         principalpixel=f"{width/2-intrinsics['ppx']} {height/2-intrinsics['ppy']}")


def prepare(snapshot, calibration_path, output, latest=True, home_path=None, image_override=None, depth_override=None, pose_series=True, convention="new", base_yaw=None, register_icp=False):
    snapshot, output = Path(snapshot), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((snapshot / 'capture.json').read_text())
    use_latest = latest and all((snapshot / f).is_file() for f in ('home-recheck.png', 'home-recheck-depth.npy', 'arm-home-recheck.json'))
    image_path = snapshot / ('home-recheck.png' if use_latest else 'color.png')
    depth_path = snapshot / ('home-recheck-depth.npy' if use_latest else 'aligned-depth.npy')
    arm_path = snapshot / ('arm-home-recheck.json' if use_latest else 'arm-readonly.json')
    current_frames = snapshot / 'initial-align-retry/frames'
    if latest and current_frames.is_dir():
        paired = sorted(path for path in current_frames.glob('*.npy') if path.with_suffix('.png').is_file())
        if paired: depth_path = paired[-1]; image_path = depth_path.with_suffix('.png')
    if home_path is not None: arm_path = Path(home_path)
    if (image_override is None) != (depth_override is None): raise ValueError('paired image and depth overrides required')
    if image_override is not None: image_path, depth_path = Path(image_override), Path(depth_override)
    from PIL import Image
    rgb = np.asarray(Image.open(image_path).convert('RGB'))
    depth = np.load(depth_path, allow_pickle=False)
    if depth.shape != rgb.shape[:2]: raise ValueError('RGB and aligned depth dimensions differ')
    points, valid = unproject(depth, metadata['intrinsics'], metadata['depth_scale_m'])
    roi = np.zeros(depth.shape, bool); roi[340:470, 180:580] = True
    selected = points[roi & valid][::3]
    center, basis, plane = fit_table(selected)
    world_points = (points - center) @ basis
    stride = np.zeros(depth.shape, bool); stride[::2, ::2] = True
    mask = valid & stride
    focus = mask & (np.indices(depth.shape)[0] > 150) & (np.abs(world_points[..., 0]) < .5) & (np.abs(world_points[..., 1]) < .38) & (world_points[..., 2] > -.035) & (world_points[..., 2] < .5)
    np.savez_compressed(output / 'pointcloud.npz', points=world_points[mask].astype(np.float32), colors=rgb[mask],
                        focused_points=world_points[focus].astype(np.float32), focused_colors=rgb[focus])
    camera_position = -center @ basis
    camera_rotation = Rotation.from_matrix(basis.T).as_quat()[[3, 0, 1, 2]]
    # Pixel ROIs locate objects roughly; no camera-to-arm registration is claimed.
    pixel_y, pixel_x = np.indices(depth.shape)
    arm_position = np.median(world_points[valid & (pixel_x > 380) & (pixel_y > 290)], axis=0)
    arm_position[2] = 0
    sorter_position = np.median(world_points[valid & (pixel_x < 300) & (pixel_y > 290)], axis=0)
    sorter_position[2] = 0
    arm_record = json.loads(arm_path.read_text())
    follower = arm_record if isinstance(arm_record, dict) else next(a for a in arm_record if a['role'] == 'follower')
    if 'initial_raw' in follower: follower = {'raw_position': follower['initial_raw']}
    calibration = json.loads(Path(calibration_path).read_text())
    joint_degrees, violations = {}, []
    for joint, raw in follower['raw_position'].items():
        c = calibration[joint]
        if not c['range_min'] <= raw <= c['range_max']:
            violations.append({'joint': joint, 'raw': raw, 'range_min': c['range_min'], 'range_max': c['range_max']})
        joint_degrees[joint] = ((raw - c['range_min']) * 100 / (c['range_max'] - c['range_min']) if joint == 'gripper'
                                else (raw - (c['range_min'] + c['range_max']) / 2) * 360 / 4095)
    nominal_joint_degrees = dict(joint_degrees)
    from .registration import white_arm_mask, cad_surface, fit_registration
    from .kinematics import set_pose
    import mujoco
    arm_mask = white_arm_mask(rgb, world_points, valid)
    initial_model = mujoco.MjModel.from_xml_string(make_model([0, 0, 0], sorter_position, camera_position, camera_rotation, convention=convention))
    initial_data = mujoco.MjData(initial_model)
    set_pose(initial_model, initial_data, joint_degrees)
    registration = fit_registration(cad_surface(initial_model, initial_data), world_points[arm_mask])
    from .registration import load_pose_series, fit_pose_series
    frames = []
    if pose_series:
        frames = load_pose_series(snapshot, center, basis, metadata, result_name='scene-calibration-recorded-result.json', folder='scene-calibration-recorded')
        if len(frames) < 3: frames = load_pose_series(snapshot, center, basis, metadata)
    if pose_series and len(frames) >= 3:
        registration = fit_pose_series(initial_model, frames)
        for name, offset in registration['joint_zero_offsets_deg'].items(): joint_degrees[name] += offset
    user_landmark = None
    if base_yaw is not None:
        from .tracking import detect_rear_motor
        user_landmark = detect_rear_motor(rgb,world_points,valid)
        if user_landmark is not None:
            set_pose(initial_model,initial_data,joint_degrees)
            gripper_body = mujoco.mj_name2id(initial_model,mujoco.mjtObj.mjOBJ_BODY,'gripper')
            motor_geoms = [i for i in range(initial_model.ngeom) if initial_model.geom_bodyid[i]==gripper_body and initial_model.geom_group[i]==2 and 'sts3215' in (mujoco.mj_id2name(initial_model,mujoco.mjtObj.mjOBJ_MATERIAL,int(initial_model.geom_matid[i])) or '')]
            local_center = initial_data.geom_xpos[motor_geoms[0]]
            user_rotation = Rotation.from_euler('z',float(base_yaw),degrees=True).as_matrix()
            registration['user_landmark_position_m'] = (np.array(user_landmark['world_position_m'])-user_rotation@local_center).tolist()
    arm_position = np.array(registration['position_m'])
    if user_landmark is not None: arm_position=np.array(registration['user_landmark_position_m'])
    icp_report = None
    if register_icp:
        if base_yaw is None: raise ValueError('point-to-plane registration requires explicit user base yaw prior')
        from .icp import oriented_cad_surface, follower_cloud_mask, point_to_plane_icp
        set_pose(initial_model,initial_data,joint_degrees)
        cad,normals=oriented_cad_surface(initial_model,initial_data)
        observed_mask=follower_cloud_mask(rgb,points,valid,world_points)
        np.savez_compressed(output/'registration-points.npz',cad=cad,normals=normals,observed=world_points[observed_mask])
        icp_report=point_to_plane_icp(cad,normals,world_points[observed_mask],arm_position,float(base_yaw))
        arm_position=np.array(icp_report['position_m'])
    model_xml = make_model(arm_position, sorter_position, camera_position, camera_rotation, convention=convention)
    model_root = ET.fromstring(model_xml)
    display_yaw = icp_report['yaw_deg'] if icp_report else registration['yaw_deg'] if base_yaw is None else float(base_yaw)%360
    model_root.find("worldbody/body[@name='arm_base']").set('quat', nums(quat([0, 0, np.deg2rad(display_yaw)])))
    from .surface import write_surface, colored_pieces
    from scipy.ndimage import binary_closing, binary_fill_holes, binary_dilation
    follower_silhouette = binary_dilation(binary_fill_holes(binary_closing(arm_mask, iterations=3)), iterations=5)
    surface_valid = valid & ~follower_silhouette & (pixel_y > 150) & (world_points[..., 2] < .45)
    surface_report = write_surface(output, world_points, rgb, surface_valid)
    asset = model_root.find('asset')
    ET.SubElement(asset, 'texture', name='measured_rgb', type='2d', file=str((output / 'measured-texture.png').resolve()))
    ET.SubElement(asset, 'material', name='measured_rgb', texture='measured_rgb', texuniform='false', reflectance='0', specular='0', emission='.5')
    ET.SubElement(asset, 'mesh', name='measured_surface', file=str((output / 'measured-surface.obj').resolve()))
    ET.SubElement(model_root.find('worldbody'), 'geom', name='measured_front_surfaces', type='mesh', mesh='measured_surface', material='measured_rgb', contype='0', conaffinity='0', group='1')
    optical_rotation = basis.T @ np.diag([1, -1, -1])
    qcamera = Rotation.from_matrix(optical_rotation).as_quat()[[3, 0, 1, 2]]
    intr = metadata['intrinsics']
    add_measured_camera(model_root.find('worldbody'),intr,rgb.shape[1],rgb.shape[0],camera_position,qcamera)
    model_xml = ET.tostring(model_root, encoding='unicode')
    (output / 'scene.xml').write_text(model_xml)
    source = json.loads((ASSETS / 'sources.json').read_text())
    world_from_camera = np.eye(4); world_from_camera[:3, :3] = basis.T; world_from_camera[:3, 3] = camera_position
    report = {'schema': 1, 'mode': 'offline snapshot; model controls never actuate hardware',
              'snapshot': str(snapshot), 'image': str(image_path), 'depth': str(depth_path), 'arm_snapshot': str(arm_path),
              'camera': metadata, 'projection_depth_range_m': [.15, 1.5], 'table_fit': plane, 'table_roi_pixels': [180, 340, 580, 470],
              'world_from_camera': world_from_camera.tolist(), 'frame_convention': 'world z=table normal toward camera; x=projected optical right; optical x right/y down/z forward',
              'global_ik_qualified': False, 'arm': {'serial': follower.get('usb_serial', follower.get('serial')), 'raw_position': follower['raw_position'],
                      'nominal_joint_degrees': nominal_joint_degrees, 'model_joint_degrees': joint_degrees, 'old_calibration_range_comparisons': violations,
                      'position_estimate_m': arm_position.tolist(), 'rear_motor_observation':user_landmark, 'point_to_plane_registration':icp_report, 'orientation_estimate_deg': display_yaw,
                      'registration': registration},
              'ev3': {'ports': {'feeder': 'A medium', 'sorter': 'D large'}, 'position_estimate_m': sorter_position.tolist(),
                      'geometry': 'estimated primitives; 0.32 m rail span and encoder-to-distance mapping unmeasured'},
              'official_model_convention': convention, 'urdf_source': source, 'hashes': {str(p): digest(p) for p in (snapshot / 'capture.json', image_path, depth_path, arm_path, Path(calibration_path))},
              'observed_colored_pieces': colored_pieces(rgb, world_points, valid), 'measured_surface': surface_report, 'point_count': int(mask.sum()), 'focused_point_count': int(focus.sum()),
              'focused_cloud_crop': {'minimum_pixel_row': 150, 'world_bounds_m': {'x': [-.5, .5], 'y': [-.38, .38], 'z': [-.035, .5]}, 'purpose': 'display crop only; full context retained'}, 'limitations': ['sensor and arm files are separate snapshots, not a synchronized capture', 'latest image/depth reuse the supplied capture.json intrinsics; unchanged camera profile assumed', 'table fit is estimated, not a surveyed frame', 'camera-to-arm extrinsic and joint zero mapping not commissioned', 'no contact, dynamics, grasp or sim-to-real qualification', 'range comparisons are reported, not used to prevent visualization']}
    if base_yaw is not None: report['arm']['user_visual_alignment'] = {'base_yaw_deg':display_yaw,'status':'user-aligned yaw plus approximate observed motor-face translation; not surveyed; globalIKqualified=false'}
    from .tracking import fit_tracked_tcp
    report['tracked_gripper_registration'] = fit_tracked_tcp(snapshot, metadata, output, world_from_camera, convention=convention) if pose_series else None
    (output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
    return report
