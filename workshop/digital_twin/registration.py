"""Offline CAD-to-RGB-D registration; estimates never authorize hardware motion."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def white_arm_mask(rgb, world, valid):
    y, x = np.indices(valid.shape)
    minimum = rgb.min(axis=-1).astype(float)
    saturation = rgb.max(axis=-1).astype(float) - minimum
    return (valid & (x >= 300) & (x < 565) & (y >= 110) & (y < 375)
            & (minimum > 125) & (saturation < 48) & (world[..., 2] > .015)
            & (world[..., 2] < .36))


def cad_samples(model, count=6000, seed=19):
    """Sample once in compiled mesh-local frames; updates only transform samples."""
    import mujoco
    rng = np.random.default_rng(seed)
    triangles, ids = [], []
    for index in range(model.ngeom):
        if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_group[index] != 2: continue
        material = int(model.geom_matid[index])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, material) or ''
        if 'sts3215' in name: continue
        mesh = model.geom_dataid[index]
        start, length = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        face_start, face_count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        vertices = model.mesh_vert[start:start+length]
        triangle = vertices[model.mesh_face[face_start:face_start+face_count]]
        triangles.append(triangle); ids.append(np.full(len(triangle), index))
    triangles, ids = np.concatenate(triangles), np.concatenate(ids)
    area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1)
    selected = rng.choice(len(triangles), count, p=area/area.sum())
    chosen = triangles[selected]
    uv = rng.random((count, 2)); uv[uv.sum(axis=1)>1] = 1-uv[uv.sum(axis=1)>1]
    points = chosen[:, 0]+uv[:, :1]*(chosen[:, 1]-chosen[:, 0])+uv[:, 1:]*(chosen[:, 2]-chosen[:, 0])
    return points, ids[selected]


def transform_samples(model, data, samples):
    import mujoco
    points, ids = samples
    transformed = np.einsum('nij,nj->ni', data.geom_xmat[ids].reshape(-1, 3, 3), points)+data.geom_xpos[ids]
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    return (transformed-data.xpos[base]) @ data.xmat[base].reshape(3, 3)


def cad_surface(model, data, count=6000, seed=19):
    """Area-weighted white plastic samples, in the current arm-base frame."""
    return transform_samples(model, data, cad_samples(model, count, seed))


def fit_registration(cad_points, observed_points, initial_position=(.15, .13, 0), yaw_seeds=(0, 90, 180, 270)):
    """Fit table-aligned yaw/XYZ; one-way visible-surface residual avoids occluded backs.

    A single pose cannot establish joint zeros or independent extrinsic accuracy.
    Report this explicitly, even if its nearest-surface residual is small.
    """
    cad_points = np.asarray(cad_points, float)
    observed = np.asarray(observed_points, float)
    if len(cad_points) < 30 or len(observed) < 30: raise ValueError('insufficient CAD/observed surface points')
    observed = observed[::max(1, len(observed) // 1800)]
    tree = cKDTree(cad_points)
    def residual(parameters):
        rotation = Rotation.from_euler('z', parameters[3]).as_matrix()
        local = (observed - parameters[:3]) @ rotation
        return tree.query(local, workers=1)[0]
    solutions = []
    for yaw in yaw_seeds:
        result = least_squares(residual, [*initial_position, np.deg2rad(yaw)],
                               bounds=([-.05, -.1, -.035, -2*np.pi], [.35, .35, .035, 2*np.pi]),
                               loss='soft_l1', f_scale=.008, max_nfev=80, diff_step=1e-3)
        solutions.append(result)
    best = min(solutions, key=lambda result: np.mean(np.minimum(residual(result.x), .04) ** 2))
    distances = residual(best.x)
    transform = np.eye(4); transform[:3, :3] = Rotation.from_euler('z', best.x[3]).as_matrix(); transform[:3, 3] = best.x[:3]
    return {'world_from_arm_base': transform.tolist(), 'position_m': best.x[:3].tolist(),
            'yaw_deg': float(np.rad2deg(best.x[3]) % 360), 'observed_samples': len(observed),
            'cad_samples': len(cad_points), 'surface_residual_median_m': float(np.median(distances)),
            'surface_residual_p95_m': float(np.percentile(distances, 95)),
            'fraction_within_10mm': float(np.mean(distances < .01)),
            'status': 'UNQUALIFIED single-pose surface fit; joint zeros fixed to nominal; no independent holdout',
            'method': 'four yaw seeds, bounded XYZ+yaw robust nearest CAD surface least squares',
            'uncertainty': 'occlusion, segmentation, nominal joint-zero gauge and depth error not independently resolved'}


def fit_pose_series(model, frames, initial_position=(.15, .13, 0), yaw_seeds=(0, 90, 180, 270)):
    """Joint-zero / extrinsic fit across saved poses, with last distinct pose held out.

    Pan zero is fixed: it is indistinguishable from the mount yaw. Gripper and
    roll zeros also remain fixed because pan-only data cannot identify them.
    """
    import mujoco
    from .kinematics import set_pose, gripper_motor_offset
    data = mujoco.MjData(model)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    model.body_pos[base] = 0; model.body_quat[base] = [1, 0, 0, 0]
    holdout = next((frame for frame in frames if 'wrist_flex_minus' in frame['label']), frames[-1]) if len(frames)>2 else None
    selected = [frame for frame in frames if frame is not holdout]
    offset_names = ('shoulder_lift', 'elbow_flex', 'wrist_flex')
    # Pan-only series cannot distinguish planar joint zeros reliably. Keep
    # zero priors strong until recorded independent joint dithers are available.
    independent = sum(np.ptp([f['degrees'][joint] for f in frames]) > 3 for joint in offset_names)
    free_offsets = independent >= 2
    sample_template = cad_samples(model, count=3500)
    set_pose(model,data,frames[0]['degrees'])
    motor_offset=gripper_motor_offset(model,data)
    def evaluate(parameters, frame, include_landmark=True):
        degrees = dict(frame['degrees'])
        for name, offset in zip(offset_names, parameters[4:]): degrees[name] += np.rad2deg(offset)
        set_pose(model, data, degrees)
        samples = transform_samples(model, data, sample_template)
        rotation = Rotation.from_euler('z', parameters[3]).as_matrix()
        observed = frame['observed'][::max(1, len(frame['observed']) // 700)]
        distances = cKDTree(samples).query((observed-parameters[:3]) @ rotation, workers=1)[0]
        if include_landmark and 'landmark_world' in frame:
            site = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
            landmark = data.site_xpos[site]+data.site_xmat[site].reshape(3,3)@motor_offset
            prediction = landmark @ rotation.T+parameters[:3]
            return np.r_[distances,np.tile(prediction-frame['landmark_world'],max(1,len(distances)//3))]
        return distances
    def residual(parameters):
        values = [evaluate(parameters, frame) for frame in selected]
        # Broad depth-derived base prior removes the reversed-endpoint solution;
        # this prior is reported, not represented as an independent measurement.
        prior = (parameters[:3]-np.array(initial_position)) * np.sqrt(sum(map(len, values))) * .35
        return np.concatenate([*values, prior, parameters[4:] * .05])
    low = [-.05, -.1, -.035, -2*np.pi, *([-np.pi]*3 if free_offsets else [-1e-8]*3)]
    high = [.35, .35, .035, 2*np.pi, *([np.pi]*3 if free_offsets else [1e-8]*3)]
    seeds = [(yaw, (0,0,0)) for yaw in yaw_seeds]
    if free_offsets: seeds += [(180, (0,-2.7,0)), (180, (0,2.7,0)), (180, (-1.5,-1.5,1.5)), (0, (0,-2.7,0))]
    solutions = [least_squares(residual, [*initial_position, np.deg2rad(yaw), *offsets],
                               bounds=(low, high), loss='soft_l1', f_scale=.008,
                               max_nfev=65, diff_step=1e-3) for yaw,offsets in seeds]
    best = min(solutions, key=lambda result: result.cost)
    metrics = []
    for frame in frames:
        distance = evaluate(best.x, frame, include_landmark=False)
        metrics.append({'label': frame['label'], 'source_image': frame['image'], 'timestamp_difference_s': frame['timestamp_difference_s'],
                        'role': 'holdout' if frame is holdout else 'training',
                        'median_m': float(np.median(distance)), 'p95_m': float(np.percentile(distance, 95)),
                        'fraction_within_10mm': float(np.mean(distance < .01))})
    transform = np.eye(4); transform[:3, :3] = Rotation.from_euler('z', best.x[3]).as_matrix(); transform[:3, 3] = best.x[:3]
    return {'world_from_arm_base': transform.tolist(), 'position_m': best.x[:3].tolist(),
            'yaw_deg': float(np.rad2deg(best.x[3]) % 360),
            'joint_zero_offsets_deg': dict(zip(offset_names, np.rad2deg(best.x[4:]).tolist())),
            'joint_zero_fit_enabled': bool(free_offsets), 'frames': metrics,
            'base_prior_m': list(initial_position), 'landmark_constraint': 'tracked rear gripper motor center, official mesh-center offset, coordinate residuals repeated to match surface sample count',
            'status': 'UNQUALIFIED multi-pose candidate; approximate white segmentation, depth-derived base prior; independent validation pending',
            'gauge': 'pan zero fixed to nominal; mount yaw absorbs pan zero; gripper/roll fixed',
            'surface_residual_median_m': float(np.median(np.concatenate([evaluate(best.x,f,include_landmark=False) for f in selected]))),
            'surface_residual_p95_m': float(np.percentile(np.concatenate([evaluate(best.x,f,include_landmark=False) for f in selected]),95))}


def load_pose_series(snapshot, center, basis, metadata, result_name='visible-pan-result.json', folder='visible-pan-retry'):
    """Read immutable motion logs/frames; match recorded dwell end via host monotonic time."""
    import json
    from pathlib import Path
    from PIL import Image
    from .scene import unproject
    snapshot = Path(snapshot)
    path = snapshot / result_name
    if not path.is_file(): return []
    timestamps = [json.loads(line) for line in (snapshot / folder / 'timestamps.jsonl').read_text().splitlines()]
    frames = []
    reference_rgb = None
    from .tracking import track_patch
    for record in json.loads(path.read_text())['records']:
        timestamp = min(timestamps, key=lambda item: abs(item['monotonic']-record['dwell_end_monotonic']))
        delta = abs(timestamp['monotonic']-record['dwell_end_monotonic'])
        if delta > .16: continue
        image = snapshot / folder / 'frames' / f"{timestamp['index']:05d}.png"
        depth_path = image.with_suffix('.npy')
        if not image.is_file() or not depth_path.is_file(): continue
        rgb = np.asarray(Image.open(image).convert('RGB'))
        points, valid = unproject(np.load(depth_path, allow_pickle=False), metadata['intrinsics'], metadata['depth_scale_m'])
        world = (points-center) @ basis
        mask = white_arm_mask(rgb, world, valid)
        if reference_rgb is None: reference_rgb = rgb
        feature_pixel, correlation = track_patch(reference_rgb,rgb)
        fx,fy = map(lambda value:int(round(value)),feature_pixel)
        feature_patch = world[max(0,fy-2):fy+3,max(0,fx-2):fx+3]
        feature_valid = valid[max(0,fy-2):fy+3,max(0,fx-2):fx+3]
        landmark = np.median(feature_patch[feature_valid],axis=0) if feature_valid.any() else None
        if mask.sum() < 30: continue
        frames.append({'label': record['label'], 'degrees': record['dwell_end_position'], 'observed': world[mask],
                       'image': str(image), 'timestamp_difference_s': delta})
        if landmark is not None and correlation>.7: frames[-1]['landmark_world']=landmark
    return frames
