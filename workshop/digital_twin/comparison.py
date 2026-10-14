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
    measured = (rgb.min(axis=-1) > 125) & (rgb.max(axis=-1).astype(float)-rgb.min(axis=-1) < 48)
    y, x = np.indices(depth.shape); measured &= (x >= 300) & (x < 565) & (y >= 110) & (y < 375) & (depth > .15) & (depth < 1.5) & ((y < 210) | (x > 425))
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
