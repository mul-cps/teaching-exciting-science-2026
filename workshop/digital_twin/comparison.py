"""Measured-camera inverse render diagnostics, separated from observed-surface reconstruction."""
import numpy as np
import mujoco
from PIL import Image


def compare_camera(model, data, report, output):
    from pathlib import Path
    output = Path(output)
    rgb = np.asarray(Image.open(report['image']).convert('RGB'))
    depth = np.load(report['depth'], allow_pickle=False) * report['camera']['depth_scale_m']
    option = mujoco.MjvOption(); option.geomgroup[3] = 0
    with mujoco.Renderer(model, rgb.shape[0], rgb.shape[1]) as renderer:
        renderer.update_scene(data, camera='measured_camera', scene_option=option)
        rendered = renderer.render().copy()
        renderer.enable_depth_rendering(); predicted_depth = renderer.render().copy(); renderer.disable_depth_rendering()
        renderer.enable_segmentation_rendering(); labels = renderer.render().copy(); renderer.disable_segmentation_rendering()
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or '' for index in range(model.ngeom)]
    arm_ids = np.array([index for index, name in enumerate(names) if name.startswith('official_arm_geom_')])
    arm = np.isin(labels[..., 0], arm_ids) & (labels[..., 1] == mujoco.mjtObj.mjOBJ_GEOM)
    from .scene import unproject
    from .icp import follower_cloud_mask
    points,valid=unproject(np.load(report['depth'],allow_pickle=False),report['camera']['intrinsics'],report['camera']['depth_scale_m'])
    transform=np.asarray(report['world_from_camera'])
    world=points@transform[:3,:3].T+transform[:3,3]
    measured=follower_cloud_mask(rgb,points,valid,world)

    overlap = arm & measured
    union = arm | measured
    residual = np.abs(predicted_depth[overlap] - depth[overlap])
    overlay = rgb.copy(); overlay[arm] = (.5*overlay[arm] + .5*np.array([255, 40, 30])).astype(np.uint8)
    Image.fromarray(rendered).save(output / 'measured-camera-render.png')
    Image.fromarray(overlay).save(output / 'robot-silhouette-overlay.png')
    return {'camera': 'actual intrinsics including optical center; fixed measured table transform',
            'robot_silhouette_iou': float(overlap.sum()/max(1, union.sum())),
            'rendered_robot_pixels': int(arm.sum()), 'measured_white_roi_pixels': int(measured.sum()),
            'overlap_pixels': int(overlap.sum()),
            'overlap_depth_median_m': float(np.median(residual)) if len(residual) else None,
            'overlap_depth_p95_m': float(np.percentile(residual, 95)) if len(residual) else None,
            'status': 'training-frame diagnostics only; white ROI is approximate segmentation; static textured mesh reuses photograph and is not predictive validation'}


def compare_registration(model,data,report,output):
    """Before/after same-camera renders for the ICP transform, preserving qpos."""
    from pathlib import Path
    output=Path(output)
    registration=report['arm'].get('point_to_plane_registration')
    if not registration:return {'after':compare_camera(model,data,report,output)}
    from scipy.spatial.transform import Rotation
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'arm_base')
    position,rotation=model.body_pos[base].copy(),model.body_quat[base].copy()
    before_dir=output/'registration-before';before_dir.mkdir(exist_ok=True)
    try:
        model.body_pos[base]=registration['initial_position_m']
        q=Rotation.from_euler('z',registration.get('initial_yaw_deg',registration['yaw_prior_deg']),degrees=True).as_quat()
        model.body_quat[base]=q[[3,0,1,2]];mujoco.mj_forward(model,data)
        before=compare_camera(model,data,report,before_dir)
    finally:
        model.body_pos[base]=position;model.body_quat[base]=rotation;mujoco.mj_forward(model,data)
    after=compare_camera(model,data,report,output)
    first=Image.open(before_dir/'robot-silhouette-overlay.png');second=Image.open(output/'robot-silhouette-overlay.png')
    from PIL import ImageDraw
    pair=Image.new('RGB',(first.width*2,first.height+28),'white');pair.paste(first,(0,28));pair.paste(second,(first.width,28))
    draw=ImageDraw.Draw(pair);draw.text((10,7),'Before point-to-plane ICP',fill='black');draw.text((first.width+10,7),'After point-to-plane ICP',fill='black')
    pair.save(output/'registration-before-after.png')
    return {'before':before,'after':after}
