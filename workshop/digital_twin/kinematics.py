"""Offline FK in the official URDF chain, without any hardware interfaces."""
import math
import xml.etree.ElementTree as ET
import weakref
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


_PINCH_PATCHES = weakref.WeakKeyDictionary()


def _canonical_pinch_patches(model):
    """Select each jaw's mesh vertices once at canonical gripper joint q=0."""
    cached = _PINCH_PATCHES.get(model)
    if cached is not None:
        return cached
    gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
    data = mujoco.MjData(model)
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'gripper')
    data.qpos[model.jnt_qposadr[joint]] = 0.0
    mujoco.mj_forward(model, data)
    origin = data.xpos[gripper]
    rotation = data.xmat[gripper].reshape(3, 3)
    patches = {}
    for geom in range(model.ngeom):
        if model.geom_group[geom] != 2 or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        material = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL,
                                     int(model.geom_matid[geom])) or ''
        kind = ('fixed' if 'wrist_roll_follower' in material else
                'moving' if 'moving_jaw' in material else None)
        if kind is None:
            continue
        mesh = int(model.geom_dataid[geom])
        start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        mesh_vertices = model.mesh_vert[start:start + count]
        world = mesh_vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        body_local = (world - origin) @ rotation
        distal_mask = body_local[:, 2] < body_local[:, 2].min() + .005
        distal = body_local[distal_mask]
        extreme = distal[:, 0].max() if kind == 'fixed' else distal[:, 0].min()
        selected = np.flatnonzero(distal_mask & (np.abs(body_local[:, 0] - extreme) < .0015))
        if len(selected) < 3:
            raise ValueError(f'official {kind} jaw CAD contact patch missing at canonical q=0')

        # Average the outward normals of triangles fully contained in the selected
        # surface patch. Keep them in mesh-local coordinates for current-pose use.
        face_start = int(model.mesh_faceadr[mesh])
        faces = model.mesh_face[face_start:face_start + int(model.mesh_facenum[mesh])]
        in_patch = np.isin(faces, selected).all(axis=1)
        triangles = mesh_vertices[faces[in_patch]]
        if len(triangles) == 0:
            raise ValueError(f'official {kind} jaw CAD patch has no mesh faces')
        face_vectors = np.cross(triangles[:, 1] - triangles[:, 0],
                                triangles[:, 2] - triangles[:, 0])
        normal = face_vectors.sum(axis=0)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            raise ValueError(f'official {kind} jaw CAD patch has degenerate normals')
        patches[kind] = {'geom': geom, 'body': int(model.geom_bodyid[geom]),
                         'mesh': mesh, 'vertex_indices': selected,
                         'vertices_local': mesh_vertices[selected].copy(),
                         'position_local': np.median(mesh_vertices[selected], axis=0),
                         'normal_local': normal / norm}
    if set(patches) != {'fixed', 'moving'}:
        raise ValueError('official finger visual meshes missing')
    _PINCH_PATCHES[model] = patches
    return patches


def export_pinch_geometry(model, data):
    """Transform persistent native CAD jaw patches into world/body/site frames.

    Contact patches are selected once from compiled visual meshes at gripper q=0
    and retain those physical vertex indices through jaw motion. This is an
    observed CAD geometry estimate, not physical jaw calibration or force/contact
    qualification. World positions do not imply camera registration.
    """
    patches = _canonical_pinch_patches(model)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
    gripper_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
    site_rotation = data.site_xmat[site].reshape(3, 3)
    gripper_rotation = data.xmat[gripper_body].reshape(3, 3)
    contacts = {}
    for kind, patch in patches.items():
        geom, body = patch['geom'], patch['body']
        geom_rotation = data.geom_xmat[geom].reshape(3, 3)
        world_vertices = patch['vertices_local'] @ geom_rotation.T + data.geom_xpos[geom]
        # The coordinate-wise median is not rotation-equivariant. Select the
        # physical patch point once in mesh-local coordinates, then transform it.
        world = patch['position_local'] @ geom_rotation.T + data.geom_xpos[geom]
        body_rotation = data.xmat[body].reshape(3, 3)
        body_vertices = (world_vertices - data.xpos[body]) @ body_rotation
        body_position = (world - data.xpos[body]) @ body_rotation
        gripper_body_position = (world - data.xpos[gripper_body]) @ gripper_rotation
        site_position = (world - data.site_xpos[site]) @ site_rotation
        normal_world = geom_rotation @ patch['normal_local']
        normal_body = body_rotation.T @ normal_world
        normal_site = site_rotation.T @ normal_world
        contacts[kind] = {
            'world_position_m': world.tolist(),
            'gripper_body_position_m': gripper_body_position.tolist(),
            'jaw_body_position_m': body_position.tolist(),
            'jaw_body_name': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body),
            'gripper_site_position_m': site_position.tolist(),
            'selected_vertices': int(len(patch['vertex_indices'])),
            'selected_vertex_indices': patch['vertex_indices'].tolist(),
            'selected_vertices_world_m': world_vertices.tolist(),
            'selected_vertices_body_m': body_vertices.tolist(),
            'geom_id': int(geom),
            'normal_world': normal_world.tolist(),
            'normal_body': normal_body.tolist(),
            'normal_site': normal_site.tolist(),
        }
    fixed = np.asarray(contacts['fixed']['world_position_m'])
    moving = np.asarray(contacts['moving']['world_position_m'])
    across = moving - fixed
    gap = float(np.linalg.norm(across))
    direction = across / gap if gap > 1e-12 else np.zeros(3)
    fixed_normal = np.asarray(contacts['fixed']['normal_world'])
    moving_normal = np.asarray(contacts['moving']['normal_world'])
    facing = {'fixed_toward_moving': float(fixed_normal @ direction),
              'moving_toward_fixed': float(-moving_normal @ direction),
              'normal_opposition': float(-fixed_normal @ moving_normal)}
    eligible = all(value > 0 for value in facing.values())
    center = (fixed + moving) / 2
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'gripper')
    qpos = float(data.qpos[model.jnt_qposadr[joint]])
    low, high = model.jnt_range[joint]
    return {'contacts': contacts, 'pinch_center_world_m': center.tolist(),
            'pinch_center_gripper_site_m': (site_rotation.T @ (center - data.site_xpos[site])).tolist(),
            'inner_face_gap_m': gap,
            'gripper_joint_position_rad': qpos,
            'gripper_opening_percent_current': float((qpos - low) / (high - low) * 100),
            'official_site_world_m': data.site_xpos[site].tolist(),
            'method': 'persistent canonical q=0 native CAD vertex patches; mesh-local median transformed rigidly; triangle normals',
            'contact_eligibility': {'eligible': eligible, 'facing_scores': facing,
                                    'basis': 'CAD face normals and opposing-contact direction; not physical calibration'},
            'geometry_status': 'OBSERVED_CAD_ESTIMATE',
            'physical_calibration_status': 'UNQUALIFIED',
            'status': 'observed CAD geometry estimate; contact eligibility is geometric only; gripper opening and global IK remain unqualified'}
