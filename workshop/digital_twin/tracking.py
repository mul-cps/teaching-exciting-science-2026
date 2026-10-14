"""Saved-image rigid gripper feature tracking and local camera Jacobian evidence."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from scipy.signal import fftconvolve
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def track_patch(reference, image, rectangle=(440, 212, 500, 258), search=(390, 165, 560, 330)):
    """Normalized correlation around a documented visible rear follower gripper patch."""
    a, b, c, d = rectangle; x0, y0, x1, y1 = search
    template = reference[b:d, a:c].astype(float).mean(axis=-1)
    target = image[y0:y1, x0:x1].astype(float).mean(axis=-1)
    template -= template.mean(); count = template.size
    numerator = fftconvolve(target, template[::-1, ::-1], mode='valid')
    window = np.ones(template.shape)
    sums = fftconvolve(target, window, mode='valid')
    squares = fftconvolve(target*target, window, mode='valid')
    denom = np.sqrt(np.maximum(squares-sums*sums/count, 1e-12)*np.sum(template*template))
    scores = numerator/denom
    row, column = np.unravel_index(np.argmax(scores), scores.shape)
    return (x0+column+(c-a)/2, y0+row+(d-b)/2), float(scores[row, column])


def fit_tracked_tcp(snapshot, metadata, output, world_from_camera=None, convention="new"):
    """Nominal FK + observed rigid patch; gripper patch is not the grasp TCP."""
    import mujoco
    from .scene import make_model, unproject
    from .kinematics import set_pose
    snapshot, output = Path(snapshot), Path(output)
    result_path = snapshot/'scene-calibration-recorded-result.json'
    if not result_path.is_file(): return None
    records = json.loads(result_path.read_text())['records']
    timestamps = [json.loads(line) for line in (snapshot/'scene-calibration-recorded/timestamps.jsonl').read_text().splitlines()]
    images = []
    for record in records:
        stamp = min(timestamps,key=lambda t: abs(t['monotonic']-record['dwell_end_monotonic']))
        if abs(stamp['monotonic']-record['dwell_end_monotonic'])>.16: raise ValueError('gripper tracking frame exceeds160ms pose timing bound')
        path = snapshot/'scene-calibration-recorded/frames'/f"{stamp['index']:05d}.png"
        images.append((record, stamp, path))
    reference = np.asarray(Image.open(images[0][2]).convert('RGB'))
    model = mujoco.MjModel.from_xml_string(make_model([0,0,0],[-.2,0,0],[0,-.5,.3],[1,0,0,0],convention=convention))
    data = mujoco.MjData(model); site = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
    observations, poses, rotations, evidence = [], [], [], []
    for record,stamp,path in images:
        image = np.asarray(Image.open(path).convert('RGB'))
        pixel, score = track_patch(reference,image)
        depth = np.load(path.with_suffix('.npy'),allow_pickle=False)
        points,valid = unproject(depth,metadata['intrinsics'],metadata['depth_scale_m'])
        x,y = map(lambda v:int(round(v)),pixel)
        patch = points[max(0,y-2):y+3,max(0,x-2):x+3]
        mask = valid[max(0,y-2):y+3,max(0,x-2):x+3]
        if not mask.any(): raise ValueError('tracked gripper feature has no measured depth')
        measured = np.median(patch[mask],axis=0)
        set_pose(model,data,record['dwell_end_position'])
        observations.append(measured); poses.append(data.site_xpos[site].copy()); rotations.append(data.site_xmat[site].reshape(3,3).copy())
        evidence.append({'label':record['label'],'image':str(path),'pixel':list(pixel),'correlation':score,
                         'camera_position_m':measured.tolist(),'actual_joint_degrees':record['dwell_end_position'],
                         'timestamp_difference_s':abs(stamp['monotonic']-record['dwell_end_monotonic'])})
        annotation = Image.fromarray(image); ImageDraw.Draw(annotation).ellipse((x-5,y-5,x+5,y+5),outline='red',width=2)
        annotation.save(output/f"tracked-{record['label']}.png")
    observations,poses,rotations = map(np.asarray,(observations,poses,rotations))
    # Keep native joint zeros fixed. Fit an unknown rigid patch offset and a
    # camera mount transform; small dithers may be ill-conditioned, so report rank.
    def residual(parameters, indices):
        rotation = Rotation.from_rotvec(parameters[:3]).as_matrix()
        feature = poses[indices]+np.einsum('nij,j->ni',rotations[indices],parameters[6:])
        return (feature @ rotation.T+parameters[3:6]-observations[indices]).ravel()
    holdout = next(i for i,r in enumerate(records) if 'wrist_flex_minus' in r['label'])
    training = np.array([i for i in range(len(records)) if i != holdout])
    import itertools
    from .kinematics import gripper_motor_offset
    source_offset=gripper_motor_offset(model,data)
    candidates = []
    for signs in itertools.product((-1,1),repeat=4):
        sign_map = dict(zip(('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex'), signs))
        for index, record in enumerate(records):
            degrees = dict(record['dwell_end_position'])
            for name, sign in sign_map.items(): degrees[name] *= sign
            set_pose(model,data,degrees)
            poses[index] = data.site_xpos[site]; rotations[index] = data.site_xmat[site].reshape(3,3)
        initial = np.r_[0.,0.,0., observations[0]-poses[0],source_offset]
        solution = least_squares(lambda p:residual(p,training),initial,
                                 bounds=(np.r_[[-np.pi]*3,[-2.]*3,source_offset-.015],np.r_[[np.pi]*3,[2.]*3,source_offset+.015]),
                                 max_nfev=160,loss='soft_l1',f_scale=.004)
        candidates.append((float(np.mean(residual(solution.x,training)**2)),sign_map,solution,poses.copy(),rotations.copy()))
    candidates.sort(key=lambda item:item[0])
    _, sign_map, solution, poses, rotations = candidates[0]
    errors = np.linalg.norm(residual(solution.x,np.arange(len(records))).reshape(-1,3),axis=1)
    transform = np.eye(4);transform[:3,:3]=Rotation.from_rotvec(solution.x[:3]).as_matrix();transform[:3,3]=solution.x[3:6]
    derivatives = {}
    for i,record in enumerate(records):
        for joint in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll'):
            if record['label'].startswith(joint) and i>0:
                delta = record['dwell_end_position'][joint]-records[i-1]['dwell_end_position'][joint]
                derivatives[joint] = {'camera_m_per_body_degree':((observations[i]-observations[i-1])/delta).tolist(),
                                      'actual_delta_deg':delta,'displacement_m':float(np.linalg.norm(observations[i]-observations[i-1])),
                                      'tracking_correlation':evidence[i]['correlation']}
    local_joints = ('shoulder_pan','elbow_flex','wrist_flex')
    local_jacobian = np.column_stack([derivatives[joint]['camera_m_per_body_degree'] for joint in local_joints])
    local_singular = np.linalg.svd(local_jacobian,compute_uv=False)
    singular = np.linalg.svd(solution.jac,compute_uv=False)
    report = {'status':'UNQUALIFIED rigid gripper-patch candidate; feature is not grasp TCP; no hardware commands',
              'official_model_convention':convention, 'rectangle_pixels':[440,212,500,258],'observations':evidence,'camera_from_arm_base':transform.tolist(),
              'feature_offset_in_gripper_frame_m':solution.x[6:].tolist(),'joint_zero_offsets_deg':'fixed nominal; none fitted', 'joint_sign_candidate':sign_map,
              'feature_offset_prior_in_gripper_frame_m':source_offset.tolist(),
              'feature_offset_bound_m':.015,
              'sign_variant_training_rmse_m':[{'signs':item[1],'rmse_m':float(np.sqrt(item[0]))} for item in candidates],
              'training_error_median_m':float(np.median(errors[training])),'training_error_p95_m':float(np.percentile(errors[training],95)),
              'holdout_label':records[holdout]['label'],'holdout_error_m':float(errors[holdout]),
              'jacobian_singular_values':singular.tolist(),'condition_number':float(singular[0]/max(singular[-1],1e-15)),
              'local_observed_feature_jacobian':derivatives,
              'local_feature_ik':{'joint_names':list(local_joints),'camera_m_per_degree':local_jacobian.tolist(),
                                  'body_degrees_per_camera_m':np.linalg.pinv(local_jacobian,rcond=.05).tolist(),
                                  'singular_values_m_per_degree':local_singular.tolist(),
                                  'condition_number':float(local_singular[0]/max(local_singular[-1],1e-15)),
                                  'validity':'single-sided finite differences near recorded home, not globally valid; feature not TCP'},
              'limitations':['small single-sided joint dithers','patch-center depth is not a surveyed rigid marker','other joints settle by fractional degrees','joint camera frame timestamps differ up to 160ms','ICP/silhouette candidate and this feature fit are independent, not fused']}
    if world_from_camera is not None:
        transform = np.asarray(world_from_camera)
        observed_world = observations @ transform[:3,:3].T + transform[:3,3]
        report['table_aligned_kinematic_candidate'] = fit_table_kinematics(model,records,observed_world,source_offset,holdout)
    (output/'tracked-tcp-registration.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def fit_table_kinematics(model, records, observed_world, feature_offset, holdout, initial_position=(.15,.13,0)):
    """Table-aligned native FK fit to a known gripper-motor landmark.

    Fitted zero corrections are candidates; survey/marker uncertainty remains.
    """
    import mujoco
    from .kinematics import set_pose
    data=mujoco.MjData(model); site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
    names=('shoulder_lift','elbow_flex','wrist_flex')
    training=np.array([i for i in range(len(records)) if i!=holdout])
    def predictions(parameters):
        points=[]
        for record in records:
            degrees=dict(record['dwell_end_position'])
            for name,offset in zip(names,parameters[4:]): degrees[name]+=np.rad2deg(offset)
            set_pose(model,data,degrees)
            points.append(data.site_xpos[site]+data.site_xmat[site].reshape(3,3)@feature_offset)
        return np.array(points)@Rotation.from_euler('z',parameters[3]).as_matrix().T+parameters[:3]
    def residual(parameters):
        # Preserve a broad depth-derived base prior, independently disclosed.
        prior=(parameters[:3]-initial_position)*.05
        return np.r_[(predictions(parameters)[training]-observed_world[training]).ravel(),prior]
    seeds=[(yaw,elbow) for yaw in (0,90,180,270) for elbow in (0,2.7,-2.7)]
    candidates=[least_squares(residual,[*initial_position,np.deg2rad(yaw),0,elbow,0],
                             bounds=([-.1,-.1,-.05,-2*np.pi,-np.pi,-np.pi,-np.pi],[.4,.4,.1,2*np.pi,np.pi,np.pi,np.pi]),
                             loss='soft_l1',f_scale=.004,max_nfev=180) for yaw,elbow in seeds]
    best=min(candidates,key=lambda candidate:np.mean(residual(candidate.x)**2))
    predicted=predictions(best.x);errors=np.linalg.norm(predicted-observed_world,axis=1)
    singular=np.linalg.svd(best.jac,compute_uv=False)
    transform=np.eye(4);transform[:3,:3]=Rotation.from_euler('z',best.x[3]).as_matrix();transform[:3,3]=best.x[:3]
    return {'world_from_arm_base':transform.tolist(),'position_m':best.x[:3].tolist(),
            'yaw_deg':float(np.rad2deg(best.x[3])%360),'joint_zero_offsets_deg':dict(zip(names,np.rad2deg(best.x[4:]).tolist())),
            'training_median_m':float(np.median(errors[training])),'training_p95_m':float(np.percentile(errors[training],95)),
            'holdout_error_m':float(errors[holdout]),'condition_number':float(singular[0]/max(singular[-1],1e-15)),
            'predicted_landmark_world_m':predicted.tolist(),'observed_landmark_world_m':observed_world.tolist(),
            'feature_offset_in_gripper_frame_m':feature_offset.tolist(),
            'status':'UNQUALIFIED table-aligned known CAD landmark fit; nominal joint signs; fitted joint-zero candidates',
            'limitations':['black-patch center is not a surveyed landmark','table z and base prior estimated','pan zero fixed as yaw gauge','small dithers constrain local predictions more strongly than global pose']}


def detect_rear_motor(rgb, world, valid):
    """Approximate visible rear-gripper black servo; image/depth evidence retained."""
    from scipy.ndimage import label,binary_opening,binary_closing
    y,x=np.indices(valid.shape)
    mask=(rgb.max(axis=-1)<90)&(x>390)&(x<520)&(y>175)&(y<270)&valid
    mask=binary_closing(binary_opening(mask,iterations=2),iterations=2)
    labels,count=label(mask);candidates=[]
    for index in range(1,count+1):
        selected=labels==index
        if selected.sum()<100: continue
        xmin,xmax=x[selected].min(),x[selected].max();ymin,ymax=y[selected].min(),y[selected].max()
        ratio=(xmax-xmin+1)/(ymax-ymin+1)
        if not .9<ratio<2.2 or np.median(y[selected])>255:continue
        candidates.append((selected.sum(),selected,[int(xmin),int(ymin),int(xmax),int(ymax)]))
    if not candidates:return None
    _,selected,bounds=max(candidates,key=lambda candidate:candidate[0])
    return {'world_position_m':np.median(world[selected],axis=0).tolist(),
            'pixel_center':[float(np.median(x[selected])),float(np.median(y[selected]))],
            'pixel_bounds':bounds,'pixel_count':int(selected.sum()),
            'status':'approximate dark motor-face segmentation; not surveyed mesh center'}


def jaw_target_observation(rgb, depth, metadata, jaw_roi=(420,275,480,332), target_roi=(420,320,480,360)):
    """Observed fixed-finger lower white surface and red target, in optical metres.

    The selected finger surface is not the midpoint between both grasp faces.
    Caller supplies ROIs for the saved layout; no device or motion interface.
    """
    from scipy.ndimage import label
    from .scene import unproject
    points,valid=unproject(depth,metadata['intrinsics'],metadata['depth_scale_m'])
    y,x=np.indices(valid.shape)
    def roi(rectangle):
        a,b,c,d=rectangle
        return (x>=a)&(x<c)&(y>=b)&(y<d)
    white=(rgb.min(axis=-1)>125)&roi(jaw_roi)
    labels,count=label(white);components=[]
    for index in range(1,count+1):
        selected=labels==index
        if selected.sum()>20:components.append((int(y[selected].max()),selected))
    if not components:return None
    bottom,selected=max(components,key=lambda candidate:candidate[0])
    tip=selected&(y>=bottom-4)
    red=(rgb[:,:,0]>120)&(rgb[:,:,1]<105)&(rgb[:,:,2]<105)&roi(target_roi)
    if not (tip&valid).any() or not (red&valid).any():return None
    finger=np.median(points[tip&valid],axis=0);target=np.median(points[red&valid],axis=0)
    return {'finger_pixel':[float(np.median(x[tip])),float(np.median(y[tip]))],
            'red_pixel':[float(np.median(x[red])),float(np.median(y[red]))],
            'finger_camera_m':finger.tolist(),'red_camera_m':target.tolist(),
            'red_minus_finger_camera_m':(target-finger).tolist(),
            'finger_depth_samples':int((tip&valid).sum()),'red_depth_samples':int((red&valid).sum()),
            'jaw_roi':list(jaw_roi),'target_roi':list(target_roi),
            'status':'visible finger surface and red surface; approximate segmentation, not grasp TCP or motion command'}
