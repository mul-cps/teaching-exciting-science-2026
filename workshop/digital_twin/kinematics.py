"""Offline FK in the official URDF chain, without any hardware interfaces."""
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .scene import ASSETS


def set_pose(model, data, degrees, carriage=0, feeder=0):
    for joint, value in degrees.items():
        ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        q = math.radians(value)
        if joint == 'gripper':
            low, high = model.jnt_range[ident]
            q = low + value / 100 * (high - low)
        data.qpos[model.jnt_qposadr[ident]] = q
    for name, value in (('ev3_carriage_estimated', carriage), ('ev3_feeder_estimated', math.radians(feeder))):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    for actuator, transmission in enumerate(model.actuator_trnid):
        joint = int(transmission[0])
        data.ctrl[actuator] = data.qpos[model.jnt_qposadr[joint]]
    mujoco.mj_forward(model, data)



def export_fk(model, data):
    """Export link frames / joint axes relative to arm base, independent of guessed placement."""
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    origin = data.xpos[base]
    rotation = data.xmat[base].reshape(3, 3)
    links = {}
    urdf = ET.parse(ASSETS / 'so101_new_calib.urdf').getroot()
    for link in urdf.findall('link'):
        name = link.get('name')
        if name == 'gripper_frame_link':
            ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
            pose_rotation, pose_position = data.site_xmat[ident].reshape(3, 3), data.site_xpos[ident]
        else:
            alias = 'arm_base' if name == 'base_link' else name.removesuffix('_link')
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, alias)
            if body < 0: continue
            pose_rotation, pose_position = data.xmat[body].reshape(3, 3), data.xpos[body]
        transform = np.eye(4)
        transform[:3, :3] = rotation.T @ pose_rotation
        transform[:3, 3] = rotation.T @ (pose_position - origin)
        links[name] = {'arm_base_from_link': transform.tolist()}
    joints = {}
    for joint in urdf.findall('joint'):
        if joint.get('type') == 'fixed': continue
        name = joint.get('name')
        ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        joints[name] = {'position_in_arm_base_m': (rotation.T @ (data.xanchor[ident] - origin)).tolist(),
                        'axis_in_arm_base': (rotation.T @ data.xaxis[ident]).tolist(),
                        'qpos_rad_unclipped': float(data.qpos[model.jnt_qposadr[ident]])}
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
    jac_position = np.zeros((3, model.nv)); jac_rotation = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jac_position, jac_rotation, site)
    return {'source': 'official native SO101 MuJoCo joint/body/site frames; nominal normalized snapshot qpos; no range clipping',
            'body_degrees_formula': '(raw-(range_min+range_max)/2)*360/4095',
            'gripper_mapping': 'raw calibrated percent -> linear URDF angular limits; visual estimate',
            'links': links, 'joints': joints, 'gripper_site': 'gripper_frame_site',
            'gripper_position_jacobian_arm_base': (rotation.T @ jac_position).tolist(),
            'gripper_rotation_jacobian_arm_base': (rotation.T @ jac_rotation).tolist(),
            'jacobian_column_joint_names': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in model.dof_jntid],
            'registration_status': 'arm-base-local FK only; camera-to-arm transform remains unmeasured'}


def export_pose_pair(model, data, arm):
    """Keep exact raw-normalized FK separate from fitted visual joint corrections."""
    set_pose(model, data, arm['nominal_joint_degrees'])
    nominal = export_fk(model, data)
    set_pose(model, data, arm.get('model_joint_degrees', arm['nominal_joint_degrees']))
    modeled = export_fk(model, data)
    modeled['source'] = 'official native model at unqualified visual-fit corrected qpos'
    return nominal, modeled


def gripper_motor_offset(model, data):
    """Compiled motor mesh center in the selected native gripper-site frame."""
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'gripper')
    site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
    geoms=[index for index in range(model.ngeom) if model.geom_bodyid[index]==body
           and model.geom_group[index]==2
           and 'sts3215' in (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MATERIAL,int(model.geom_matid[index])) or '')]
    if not geoms:raise ValueError('native gripper motor visual missing')
    return data.site_xmat[site].reshape(3,3).T@(data.geom_xpos[geoms[0]]-data.site_xpos[site])
