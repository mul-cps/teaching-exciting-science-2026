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
            'pinch_geometry': export_pinch_geometry(model,data),
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


def export_pinch_geometry(model, data):
    """CAD distal inner faces and pinch midpoint; official site lies on fixed finger.

    Vertex-based visual geometry estimate only, not force/contact qualification.
    Returned world positions do not imply measured camera registration.
    """
    gripper=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'gripper')
    origin=data.xpos[gripper];rotation=data.xmat[gripper].reshape(3,3)
    site=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'gripper_frame_site')
    site_rotation=data.site_xmat[site].reshape(3,3)
    contacts={}
    for index in range(model.ngeom):
        if model.geom_group[index]!=2 or model.geom_type[index]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        material=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MATERIAL,int(model.geom_matid[index])) or ''
        kind='fixed' if 'wrist_roll_follower' in material else 'moving' if 'moving_jaw' in material else None
        if kind is None:continue
        mesh=model.geom_dataid[index];start,count=model.mesh_vertadr[mesh],model.mesh_vertnum[mesh]
        vertices=model.mesh_vert[start:start+count]@data.geom_xmat[index].reshape(3,3).T+data.geom_xpos[index]
        local=(vertices-origin)@rotation
        distal=local[local[:,2]<local[:,2].min()+.005]
        inner=distal[:,0].max() if kind=='fixed' else distal[:,0].min()
        face=distal[np.abs(distal[:,0]-inner)<.0015]
        contact=np.median(face,axis=0)
        world=rotation@contact+origin
        contacts[kind]={'world_position_m':world.tolist(),'gripper_body_position_m':contact.tolist(),
                        'gripper_site_position_m':(site_rotation.T@(world-data.site_xpos[site])).tolist(),
                        'selected_vertices':len(face)}
    if set(contacts)!={'fixed','moving'}:raise ValueError('official finger visual meshes missing')
    fixed=np.array(contacts['fixed']['world_position_m']);moving=np.array(contacts['moving']['world_position_m'])
    center=(fixed+moving)/2
    return {'contacts':contacts,'pinch_center_world_m':center.tolist(),
            'pinch_center_gripper_site_m':(site_rotation.T@(center-data.site_xpos[site])).tolist(),
            'inner_face_gap_m':float(np.linalg.norm(moving-fixed)),
            'official_site_world_m':data.site_xpos[site].tolist(),
            'method':'distal5mm along native gripper-body minusZ; innerX face within1.5mm; median CAD vertices',
            'status':'visual CAD pinch estimate; official site near fixed face, not opening-dependent midpoint; globalIK unqualified'}
