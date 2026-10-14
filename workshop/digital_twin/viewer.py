"""MuJoCo forward kinematics/rendering displayed through an interactive Viser view."""
import json
import math
import os
from pathlib import Path
import threading
import time

import mujoco
import numpy as np
from PIL import Image
import trimesh
import viser

from .kinematics import set_pose, export_pose_pair
from .evidence_bridge import (apply_world_registration, camera_sample_status,
                              pointcloud_cache_status, read_phase_evidence,
                              raw_pose_to_joint_degrees, select_latest_keyframe_pair_tail,
                              select_latest_rgbd_pair,
                              transform_points)

LIVE_SURFACE_REFRESH_INTERVAL_S = .2
DEPTH_VIEW_NEAR_M = .3
DEPTH_VIEW_FAR_M = .9


def _depth_visualization_texture(depth_m, valid, near_m=DEPTH_VIEW_NEAR_M,
                                 far_m=DEPTH_VIEW_FAR_M):
    """Map valid metric depth to display-only color; invalid pixels stay black."""
    if not (0 < near_m < far_m):
        raise ValueError('depth display range must be positive and ordered')
    depth_m = np.asarray(depth_m, dtype=np.float32)
    valid = np.asarray(valid, dtype=bool) & np.isfinite(depth_m)
    safe_depth = np.where(valid, depth_m, near_m)
    normalized = np.clip((safe_depth - near_m) / (far_m - near_m), 0., 1.)
    index = np.rint(normalized * 255).astype(np.uint8)
    stops = np.array([[20, 18, 75], [10, 105, 190], [35, 205, 150],
                      [245, 220, 55], [190, 25, 35]], dtype=np.float32)
    position = index.astype(np.float32) * (len(stops) - 1) / 255.
    left = np.floor(position).astype(np.int16)
    right = np.minimum(left + 1, len(stops) - 1)
    fraction = (position - left)[..., None]
    color = np.rint(stops[left] * (1. - fraction) + stops[right] * fraction).astype(np.uint8)
    color[~valid] = 0
    return color


def _display_rgbd_texture(rgb, depth_m, valid, rows, row_cutoff):
    """Use RGB when the observed table is lit; otherwise return a depth-only visualization."""
    roi = valid & (rows > row_cutoff)
    if not roi.any():
        roi = valid
    if not roi.any():
        return np.zeros_like(rgb), 'DEPTH_VIEW', 0.
    luminance = (0.2126 * rgb[..., 0].astype(np.float32)
                 + 0.7152 * rgb[..., 1].astype(np.float32)
                 + 0.0722 * rgb[..., 2].astype(np.float32))
    mean_luminance = float(luminance[roi].mean())
    if mean_luminance < 15.:
        return _depth_visualization_texture(depth_m, valid), 'DEPTH_VIEW', mean_luminance
    return rgb, 'RGB', mean_luminance



def render(model, data, path, width=960, height=640, include_measured_surface=True,
           lookat=None, distance=None):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .13] if lookat is None else lookat
    camera.distance = 1.25 if distance is None else float(distance)
    camera.azimuth = 135
    camera.elevation = -27
    option = mujoco.MjvOption()
    if not include_measured_surface:
        option.geomgroup[1] = 0
    with mujoco.Renderer(model, height, width) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        pixels = renderer.render()
    Image.fromarray(pixels).save(path)
    return pixels


def _rgbd_frame(image_path, depth_path, report, world_from_scene, max_surface_faces=None):
    from .scene import unproject
    from .surface import surface_mesh
    rgb = np.asarray(Image.open(image_path).convert('RGB'))
    depth = np.load(depth_path, allow_pickle=False)
    if depth.shape != rgb.shape[:2]:
        raise ValueError('refreshed RGB and depth dimensions differ')
    points, valid = unproject(depth, report['camera']['intrinsics'], report['camera']['depth_scale_m'])
    depth_m = depth.astype(np.float32) * float(report['camera']['depth_scale_m'])
    scene_world_from_camera = np.asarray(report.get('scene_world_from_camera', report['world_from_camera']))
    scene_points = points @ scene_world_from_camera[:3, :3].T + scene_world_from_camera[:3, 3]
    world = transform_points(scene_points, world_from_scene)
    rows, columns = np.indices(valid.shape)
    sampled = valid & (rows % 2 == 0) & (columns % 2 == 0)
    focused = (sampled & (rows > 150) & (np.abs(world[..., 0]) < .5)
               & (np.abs(world[..., 1]) < .38) & (world[..., 2] > -.035)
               & (world[..., 2] < .5))
    from .registration import white_arm_mask
    from scipy.ndimage import binary_closing, binary_dilation, binary_fill_holes
    arm_mask = white_arm_mask(rgb, scene_points, valid)
    silhouette = binary_dilation(binary_fill_holes(binary_closing(arm_mask, iterations=3)), iterations=5)
    row_cutoff = min(150, max(0, rgb.shape[0] - 2))
    display_texture, visualization_mode, mean_luminance = _display_rgbd_texture(
        rgb, depth_m, valid, rows, row_cutoff)
    surface_valid = valid & ~silhouette & (rows > row_cutoff) & (scene_points[..., 2] < .45)
    vertices_scene, faces, vertex_colors = surface_mesh(scene_points, display_texture, surface_valid)
    if len(faces) == 0:
        raise ValueError('refreshed RGB-D frame has no valid textured surface triangles')
    if max_surface_faces is not None and len(faces) > max_surface_faces:
        faces = faces[np.linspace(0, len(faces) - 1, max_surface_faces, dtype=np.int64)]
    used_vertices, compact_faces = np.unique(faces, return_inverse=True)
    vertices_world = transform_points(vertices_scene, world_from_scene)
    measured_surface = trimesh.Trimesh(vertices=vertices_world[used_vertices],
                                      faces=compact_faces.reshape(-1, 3), process=False)
    measured_surface.visual.vertex_colors = np.column_stack((vertex_colors[used_vertices],
                                                              np.full(len(used_vertices), 255, dtype=np.uint8)))
    return ({'points': world[sampled].astype(np.float32), 'colors': rgb[sampled],
             'focused_points': world[focused].astype(np.float32), 'focused_colors': rgb[focused]},
            {'mesh': measured_surface, 'vertices_scene': vertices_scene, 'faces': faces,
             'colors': vertex_colors, 'used_vertices': used_vertices, 'rgb': rgb,
             'display_texture': display_texture, 'visualization_mode': visualization_mode,
             'mean_luminance': mean_luminance})


def _update_compiled_surface(model, geom_id, surface):
    """Update the fixed-size compiled mesh and texture without recompilation."""
    from scipy.spatial.transform import Rotation

    mesh_id = int(model.geom_dataid[geom_id])
    vertex_start, vertex_count = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
    face_start, face_count = int(model.mesh_faceadr[mesh_id]), int(model.mesh_facenum[mesh_id])
    normal_start = int(model.mesh_normaladr[mesh_id])
    normal_count = int(model.mesh_normalnum[mesh_id])
    texcoord_start = int(model.mesh_texcoordadr[mesh_id])
    texcoord_count = int(model.mesh_texcoordnum[mesh_id])
    vertices = np.asarray(surface['vertices_scene'], dtype=np.float32)
    faces = np.asarray(surface['faces'], dtype=np.int32)
    if vertices.shape != (vertex_count, 3) or not 0 < len(faces) <= face_count:
        raise ValueError('live measured surface topology does not fit the compiled mesh')
    if faces.min() < 0 or faces.max() >= vertex_count:
        raise ValueError('live measured surface face index is outside the compiled vertex buffer')
    mesh_rotation = Rotation.from_quat(model.mesh_quat[mesh_id][[1, 2, 3, 0]]).as_matrix()
    mesh_origin = model.mesh_pos[mesh_id].copy()
    local_vertices = (vertices - mesh_origin) @ mesh_rotation
    used = np.zeros(vertex_count, dtype=bool)
    used[surface['used_vertices']] = True
    local_vertices[~used] = local_vertices[surface['used_vertices']].mean(axis=0)
    face_buffer = np.zeros((face_count, 3), dtype=np.int32)
    face_buffer[:len(faces)] = faces
    if normal_count != vertex_count or texcoord_count != vertex_count:
        raise ValueError('live measured mesh requires matching vertex, normal, and UV index buffers')

    tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, 'measured_rgb')
    if tex_id < 0:
        raise ValueError('compiled model has no measured_rgb texture')
    height, width = surface['rgb'].shape[:2]
    channels = int(model.tex_nchannel[tex_id])
    if (width != int(model.tex_width[tex_id]) or height != int(model.tex_height[tex_id])
            or channels not in (3, 4)):
        raise ValueError('live photograph dimensions do not match the compiled texture')
    display_texture = surface.get('display_texture', surface['rgb'])
    texture = display_texture if channels == 3 else np.dstack((display_texture, np.full((height, width), 255, np.uint8)))
    texture_start = int(model.tex_adr[tex_id])
    texture_size = width * height * channels
    texture_bytes = np.asarray(texture, dtype=np.uint8).reshape(-1)
    if len(texture_bytes) != texture_size:
        raise ValueError('live photograph has the wrong measured texture byte count')

    # Prepare all arrays before mutation so invalid frames preserve the current mesh.
    normals = np.zeros_like(local_vertices)
    triangles = local_vertices[faces]
    face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    np.add.at(normals, faces[:, 0], face_normals)
    np.add.at(normals, faces[:, 1], face_normals)
    np.add.at(normals, faces[:, 2], face_normals)
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 1e-12
    normals[nonzero] /= lengths[nonzero, None]
    normals[~nonzero] = [0., 0., 1.]

    geom_aabb = np.r_[((local_vertices.min(axis=0) + local_vertices.max(axis=0)) / 2),
                      ((local_vertices.max(axis=0) - local_vertices.min(axis=0)) / 2)]
    geom_rbound = float(np.linalg.norm(local_vertices, axis=1).max())

    model.mesh_vert[vertex_start:vertex_start + vertex_count] = local_vertices
    model.mesh_face[face_start:face_start + face_count] = face_buffer
    model.mesh_facenormal[face_start:face_start + face_count] = face_buffer
    model.mesh_facetexcoord[face_start:face_start + face_count] = face_buffer
    model.mesh_normal[normal_start:normal_start + normal_count] = normals
    model.tex_data[texture_start:texture_start + texture_size] = texture_bytes
    model.geom_aabb[geom_id] = geom_aabb
    model.geom_rbound[geom_id] = geom_rbound


def _view_frame(surface_vertices, anchors=()):
    vertices = np.asarray(surface_vertices, dtype=float).reshape(-1, 3)
    anchor_array = np.asarray(anchors, dtype=float).reshape(-1, 3) if len(anchors) else np.empty((0, 3))
    points = np.concatenate((vertices, anchor_array), axis=0)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        center, radius = np.array([0., 0., .13]), .25
    else:
        lower, upper = points.min(axis=0), points.max(axis=0)
        center = (lower + upper) / 2
        radius = max(float(np.linalg.norm(upper - lower)) / 2, .1)
    distance = max(.4, radius * 1.15 / math.sin(math.radians(22.5)))
    direction = np.array([1., -1., .72]); direction /= np.linalg.norm(direction)
    return center, center + direction * distance, distance


def _startup_surface_mesh(initial_frame, cache_reusable, output, world_from_scene):
    """Prefer the current display texture over cached RGB when startup is in DEPTH_VIEW."""
    use_cached_rgb_surface = cache_reusable and not (
        initial_frame is not None and initial_frame.get('visualization_mode') == 'DEPTH_VIEW')
    if use_cached_rgb_surface:
        surface_data = np.load(Path(output) / 'measured-surface.npz', allow_pickle=False)
        vertices = transform_points(surface_data['vertices'], world_from_scene)
        observed_surface = trimesh.Trimesh(vertices=vertices, faces=surface_data['faces'], process=False)
        observed_surface.visual.vertex_colors = np.column_stack((surface_data['colors'],
            np.full(len(surface_data['colors']), 255, dtype=np.uint8)))
        return observed_surface
    return initial_frame['mesh'] if initial_frame is not None else None


def _file_signature(path):
    try:
        stat = Path(path).stat()
        return stat.st_mtime_ns, stat.st_size
    except OSError:
        return None


def serve(output, port=8096, host='127.0.0.1', phase_file=None, calibration_path=None,
          watch_latest_saved_camera=None):
    output = Path(output)
    report = json.loads((output / 'scene-evidence.json').read_text())
    model = mujoco.MjModel.from_xml_path(str(output / 'scene.xml'))
    data = mujoco.MjData(model)
    scene_world_from_camera = report.get('scene_world_from_camera', report['world_from_camera'])
    registration = report.get('camera_arm_registration')
    world_from_scene = np.eye(4)
    if registration:
        world_from_scene = apply_world_registration(model, data, registration, scene_world_from_camera)
    cache_status = pointcloud_cache_status(report, output)
    cache_state = {'valid': cache_status['reusable']}
    phase = read_phase_evidence(phase_file) if phase_file else report.get('phase_evidence', {'freshness': 'missing', 'pose_raw': None})
    calibration = json.loads(Path(calibration_path).read_text()) if calibration_path else None
    degrees = dict(report['arm'].get('model_joint_degrees', report['arm']['nominal_joint_degrees']))
    if phase.get('pose_raw') and calibration:
        degrees.update(raw_pose_to_joint_degrees(phase['pose_raw'], calibration))
        report['arm']['raw_position'] = phase['pose_raw']
        report['arm']['nominal_joint_degrees'] = dict(degrees)
        report['arm']['model_joint_degrees'] = dict(degrees)
    report['phase_evidence'] = phase
    report['camera_sample_status'] = camera_sample_status(report, phase,
                                                          cache_reusable=cache_status['reusable'])
    set_pose(model, data, degrees)
    report['kinematics'], report['visual_fit_kinematics'] = export_pose_pair(model, data, report['arm'])
    if report['camera_sample_status']['freshness'] == 'fresh':
        from .comparison import compare_registration
        report['registration_camera_comparison'] = compare_registration(model,data,report,output)
        report['camera_comparison'] = report['registration_camera_comparison']['after']
    else:
        report['registration_camera_comparison'] = {'status': 'SKIPPED_STALE_POSE_OR_CAMERA_SAMPLE',
            'reason': report['camera_sample_status']['reason']}
        report['camera_comparison'] = report['registration_camera_comparison']
    (output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
    surface_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'measured_front_surfaces')
    face_capacity = int(model.mesh_facenum[model.geom_dataid[surface_geom_id]]) if surface_geom_id >= 0 else None
    surface_state = {'available': False, 'freshness': 'unavailable', 'capture_wall': None,
                     'capture_monotonic': None, 'error': None,
                     'visualization_mode': 'RGB', 'mean_luminance': None,
                     'visualization_note': None}
    initial_frame = None
    try:
        _, initial_frame = _rgbd_frame(report['image'], report['depth'], report,
                                       world_from_scene, max_surface_faces=face_capacity)
        if surface_geom_id >= 0:
            _update_compiled_surface(model, surface_geom_id, initial_frame)
        surface_state.update(available=True, freshness='snapshot', capture_wall=float(
            report.get('camera_sample_timestamp_wall', report['camera']['timestamp'])))
        surface_state.update(visualization_mode=initial_frame['visualization_mode'],
            mean_luminance=initial_frame['mean_luminance'],
            visualization_note=('Depth view: low room light'
                if initial_frame['visualization_mode'] == 'DEPTH_VIEW' else None))
    except Exception as error:
        surface_state['error'] = f'{type(error).__name__}: {error}'
    surface_vertices = (initial_frame['mesh'].vertices if initial_frame is not None else np.empty((0, 3)))
    anchor_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                  for name in ('arm_base', 'ev3_base')]
    anchors = [data.xpos[index].copy() for index in anchor_ids if index >= 0]
    view_center, view_position, view_distance = _view_frame(surface_vertices, anchors)
    initial_render = render(model, data, output / 'mujoco-render.png',
                            include_measured_surface=surface_state['available'],
                            lookat=view_center, distance=view_distance)
    server = viser.ViserServer(host=host, port=port, label='CPS Station · RGB-D Digital Twin')
    server.scene.set_up_direction('+z')
    handles = []
    for index in range(model.ngeom):
        if model.geom_group[index] == 3: continue
        kind, size = model.geom_type[index], model.geom_size[index]
        name = '/model/' + mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) == 'measured_front_surfaces': continue
        material = int(model.geom_matid[index])
        rgba = model.mat_rgba[material] if material >= 0 else model.geom_rgba[index]
        color = tuple((rgba[:3] * 255).astype(int))
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            handle = server.scene.add_box(name, color=color, dimensions=2 * size)
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            handle = server.scene.add_icosphere(name, radius=float(size[0]), color=color)
        elif kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            mesh = trimesh.creation.capsule(radius=float(size[0]), height=2 * float(size[1]), count=[12, 12])
            mesh.visual.face_colors = np.r_[color, 255]
            handle = server.scene.add_mesh_trimesh(name, mesh)
        elif kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[index]
            start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
            face_start, face_count = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
            mesh = trimesh.Trimesh(vertices=model.mesh_vert[start:start + count].copy(),
                                   faces=model.mesh_face[face_start:face_start + face_count].copy(), process=False)
            mesh.visual.face_colors = np.r_[color, 255]
            handle = server.scene.add_mesh_trimesh(name, mesh)
        else:
            continue
        handles.append((index, handle))
    def update_model():
        for index, handle in handles:
            q = np.zeros(4); mujoco.mju_mat2Quat(q, data.geom_xmat[index])
            handle.position = data.geom_xpos[index].copy(); handle.wxyz = q
    update_model()
    initial_surface_mesh = _startup_surface_mesh(initial_frame, cache_status['reusable'],
                                                  output, world_from_scene)
    observed_mesh = (server.scene.add_mesh_trimesh('/measured/front_surfaces/0', initial_surface_mesh)
                     if initial_surface_mesh is not None else None)
    if cache_status['reusable']:
        cloud_data = np.load(output / 'pointcloud.npz')
        cloud_data = {key: cloud_data[key] for key in cloud_data.files}
        for key in ('points', 'focused_points'):
            cloud_data[key] = transform_points(cloud_data[key], world_from_scene).astype(np.float32)
    else:
        cloud_data, _ = _rgbd_frame(report['image'], report['depth'], report, world_from_scene,
                                    max_surface_faces=face_capacity)
    clouds = {}
    def replace_clouds(values, show_focus=False, show_context=False):
        for name in ('focused', 'context'):
            if name in clouds:
                clouds[name].remove()
        clouds['focused'] = server.scene.add_point_cloud('/measured/rgbd_focus', values['focused_points'],
                    values['focused_colors'], point_size=.003, point_shape='circle', precision='float32', visible=show_focus)
        clouds['context'] = server.scene.add_point_cloud('/measured/rgbd_context', values['points'],
                    values['colors'], point_size=.003, point_shape='circle', precision='float32', visible=show_context)
    replace_clouds(cloud_data)
    camera_transform = np.array(report['world_from_camera'])
    q = np.zeros(4); mujoco.mju_mat2Quat(q, camera_transform[:3, :3].reshape(-1))
    rgb = np.asarray(Image.open(report['image']).convert('RGB'))
    photo_handle = None
    intrinsic = report['camera']['intrinsics']
    frustum = server.scene.add_camera_frustum('/measured/realsense', fov=2 * math.atan(rgb.shape[0] / (2 * intrinsic['fy'])),
                                    aspect=rgb.shape[1] / rgb.shape[0], scale=.035, position=camera_transform[:3, 3], wxyz=q,
                                    variant='wireframe', visible=False)
    server.scene.add_frame('/measured/table_frame', axes_length=.12, axes_radius=.002)
    # Viser's built-in label color is not configurable. Use unlit dark raster
    # labels with a contrasting background; orient them toward the active viewer.
    from PIL import ImageDraw, ImageFont
    label_handles = []
    arm_position = np.asarray(registration['world_from_base'])[:3, 3] if registration else np.asarray(report['arm']['position_estimate_m'])
    ev3_position = transform_points(np.asarray(report['ev3']['position_estimate_m'])[None, :], world_from_scene)[0]
    for name, text, position in (
        ('arm', 'SO-ARM101 · official MuJoCo CAD\nProvisional full-SE(3) registration', arm_position + [0, 0, .40]),
        ('ev3', 'EV3 · A feeder / D carriage\nEstimated geometry / rail scale', ev3_position + [0, 0, .42])):
        label_image = Image.new('RGB', (400, 68), (245, 248, 250))
        font = ImageFont.load_default(size=17)
        ImageDraw.Draw(label_image).multiline_text((9, 9), text, font=font, fill=(12, 20, 29), spacing=4)
        handle = server.scene.add_image('/labels/' + name, np.asarray(label_image), .28, .0476,
                                         position=position, cast_shadow=False, receive_shadow=False)
        label_handles.append(handle)
    def face_labels(camera_position):
        for handle in label_handles:
            normal = np.asarray(camera_position) - np.asarray(handle.position)
            distance = np.linalg.norm(normal)
            if distance < 1e-9: continue
            normal /= distance
            horizontal = np.cross([0, 0, 1], normal)
            if np.linalg.norm(horizontal) < 1e-9: horizontal = np.cross([0, 1, 0], normal)
            horizontal /= np.linalg.norm(horizontal)
            vertical = np.cross(normal, horizontal)
            q = np.zeros(4); mujoco.mju_mat2Quat(q, np.column_stack((horizontal, vertical, normal)).reshape(-1))
            handle.wxyz = q
    face_labels((.85, -.85, .70))
    lock = threading.Lock()
    phase_status_handle = None
    sample_status_handle = None
    with server.gui.add_folder('Evidence / registration'):
        server.gui.add_markdown('**Saved RealSense RGB-D + read-only actual-pose evidence + MuJoCo kinematics.** '
                                'The display polls saved root evidence at 5 Hz and never reads a camera, serial port or controller. '
                                '**Camera-arm registration is provisional and unqualified. Controls change only the virtual model.**\n\n'
                                f"{report['point_count']:,} measured points · table residual p95 {report['table_fit']['residual_p95_m'] * 1000:.1f} mm.\n\n"
                                'Actual raw joint events and saved RGB-D sample freshness are shown below.')
        calibration_note = 'Camera-arm transform: **PROVISIONAL / UNQUALIFIED**'
        if registration:
            component = registration.get('component_status', {})
            calibration_note += ('\n\nHeld-out camera-arm closure: 1.6–4.4 mm / 1.1–3.7°. '
                                 'Branch identity and both component calibrations remain provisional. '
                                 'No physical calibration is claimed.')
        server.gui.add_markdown(calibration_note)
        phase_status_handle = server.gui.add_text('Actual pose evidence', initial_value='', disabled=True)
        sample_status_handle = server.gui.add_text('Camera sample / point cloud', initial_value='', disabled=True)
        view_mode_handle = server.gui.add_text('Surface texture mode',
            initial_value=surface_state['visualization_note'] or 'RGB photograph', disabled=True)
        if 'user_visual_alignment' in report['arm']: server.gui.add_markdown(f"**User-aligned base yaw {report['arm']['orientation_estimate_deg']:.1f}°. Visual registration remains unqualified.**")
        icp=report['arm'].get('point_to_plane_registration')
        if icp:
            server.gui.add_markdown(f"**Point-to-plane ICP:** median {icp['before_median_m']*1000:.1f} → {icp['after_median_m']*1000:.1f} mm. "
                                    f"{icp['fraction_within_10mm']*100:.1f}% within10mm; all-point p95 {icp['after_p95_m']*1000:.1f}mm. "
                                    f"Yaw {icp['yaw_deg']:.1f}° bounded around user prior {icp['yaw_prior_deg']:.1f}±{icp['yaw_radius_deg']:.0f}°.")
            server.gui.add_image(np.asarray(Image.open(output/'registration-before-after.png')),label='Measured-camera overlay before / after ICP')
        else:
            registration_fit = report['arm']['registration']
            if registration_fit.get('surface_residual_median_m') is None:
                server.gui.add_markdown('RGB-dependent CAD surface fit was skipped because a full-SE(3) camera/arm registration was supplied. '
                                        'That registration remains **PROVISIONAL / UNQUALIFIED**; no RGB surface-fit residual is available.')
            else:
                server.gui.add_markdown(f"CAD fit median {registration_fit['surface_residual_median_m']*1000:.1f} mm / p95 {registration_fit['surface_residual_p95_m']*1000:.1f} mm. Single pose; no holdout qualification.")

        surface_toggle = server.gui.add_checkbox('Measured textured front surfaces', observed_mesh is not None,
                                                 disabled=observed_mesh is None)
        surface_toggle.on_update(lambda event: setattr(observed_mesh, 'visible', event.target.value)
                                 if observed_mesh is not None else None)
        points_toggle = server.gui.add_checkbox('Measured RGB-D points', False)
        model_toggle = server.gui.add_checkbox('Official SO101 CAD model', True)
        context_toggle = server.gui.add_checkbox('Full room context (chair / walls)', False)
        frustum_toggle = server.gui.add_checkbox('Small camera frustum', False)
        def show_cloud(event):
            clouds['focused'].visible = points_toggle.value and not context_toggle.value
            clouds['context'].visible = points_toggle.value and context_toggle.value
        points_toggle.on_update(show_cloud); context_toggle.on_update(show_cloud)
        frustum_toggle.on_update(lambda event: setattr(frustum, 'visible', event.target.value))
        def show_model(event):
            for _, handle in handles: handle.visible = event.target.value
        model_toggle.on_update(show_model)
        display_rgb = initial_frame['display_texture'] if initial_frame is not None else rgb
        photo_handle = server.gui.add_image(display_rgb,
            label='Saved image display (RGB or display-only depth view)')
    sliders = {}
    with server.gui.add_folder('Virtual SO-ARM101 joints'):
        for joint, initial in degrees.items():
            low, high, unit = (0, 100, '%') if joint == 'gripper' else (-180, 180, 'deg')
            sliders[joint] = server.gui.add_slider(joint + ' ' + unit, min=low, max=high, step=.2, initial_value=float(initial))
    with server.gui.add_folder('Virtual EV3 sorter'):
        carriage = server.gui.add_slider('Carriage displacement · estimated m', min=0., max=.32, step=.001, initial_value=0.)
        feeder = server.gui.add_slider('Feeder angle · model deg', min=-180., max=180., step=1., initial_value=0.)
    with server.gui.add_folder('Visual registration correction · unqualified'):
        offsets = [server.gui.add_slider('Base ' + axis + ' offset m', min=-.4, max=.4, step=.005, initial_value=0.) for axis in 'xy']
        yaw = server.gui.add_slider('Board-frame yaw correction deg', min=-30., max=30., step=.5, initial_value=0.)
        server.gui.add_markdown('Corrections compose on the provisional full-SE(3) transform. They change the virtual view only and never overwrite calibration or device state.')
    arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    base_position = model.body_pos[arm_id].copy()
    base_quaternion = model.body_quat[arm_id].copy()
    virtual_override = [False]
    def change(event):
        with lock:
            virtual_override[0] = True
            degrees.update({key: value.value for key, value in sliders.items()})
            model.body_pos[arm_id] = base_position + [offsets[0].value, offsets[1].value, 0]
            angle = math.radians(yaw.value) / 2
            from scipy.spatial.transform import Rotation
            adjustment = Rotation.from_euler('z', yaw.value, degrees=True)
            native = Rotation.from_quat(base_quaternion[[1, 2, 3, 0]])
            model.body_quat[arm_id] = (adjustment * native).as_quat()[[3, 0, 1, 2]]
            set_pose(model, data, degrees, carriage.value, feeder.value)
            update_model()
    for handle in [*sliders.values(), carriage, feeder, *offsets, yaw]: handle.on_update(change)
    with server.gui.add_folder('MuJoCo render · estimated geometry'):
        render_image = server.gui.add_image(initial_render, label='MuJoCo render (not a photograph)')
        button = server.gui.add_button('Render current virtual scene')
        def render_current(event):
            with lock:
                render_image.image = render(model, data, output / 'mujoco-render-current.png',
                                            include_measured_surface=surface_state['available'],
                                            lookat=view_center, distance=view_distance)
        button.on_click(render_current)
    @server.on_client_connect
    def connect(client):
        client.camera.position = tuple(view_position)
        client.camera.look_at = tuple(view_center)
        client.camera.up_direction = (0, 0, 1)
        @client.camera.on_update
        def camera_changed(event): face_labels(client.camera.position)
    current_pair = {'image': Path(report['image']), 'depth': Path(report['depth'])}
    source_signature = [(_file_signature(current_pair['image']), _file_signature(current_pair['depth']))]
    sample_timestamp = [float(report.get('camera_sample_timestamp_wall', report['camera']['timestamp']))]
    sample_monotonic = [report.get('camera_sample_timestamp_monotonic')]
    last_phase_event = [phase.get('event_monotonic')]

    def phase_label(current):
        freshness = current.get('freshness', 'missing').upper()
        age = current.get('age_seconds')
        age_text = 'unknown age' if age is None else f'{age:.2f} s old'
        raw = current.get('pose_raw') or {}
        raw_text = ', '.join(f'{key}={value}' for key, value in raw.items())
        rendered_as = 'virtual preview override' if virtual_override[0] else 'last saved raw pose'
        return f'{freshness} · {age_text} · {current.get("phase", "no phase event")} · {rendered_as}' + (f' · {raw_text}' if raw_text else '')

    def sample_label(current, cache_reusable):
        age = current.get('age_seconds')
        delta = current.get('pose_sample_delta_seconds')
        age_text = 'unknown' if age is None else f'{age:.2f} s old'
        delta_text = 'unknown' if delta is None else f'{delta:.2f} s from pose event'
        reuse_text = 'timestamp/hash matched' if cache_reusable else 'cache OLD / not reusable'
        surface_text = f'measured surface {surface_state["freshness"].upper()}'
        if surface_state.get('visualization_note'):
            surface_text += ' · ' + surface_state['visualization_note']
        pairing = current.get('pairing_status', 'unknown').upper()
        return f'{current.get("freshness", "invalid").upper()} / {pairing} · camera {age_text} · {delta_text} · {reuse_text} · {surface_text} · calibration UNQUALIFIED'

    def publish_live_status(current_phase, current_camera, cache_reusable):
        payload = {'updated_wall_time': time.time(), 'calibration_status': 'PROVISIONAL_UNQUALIFIED',
                   'phase': current_phase, 'camera_sample': current_camera,
                   'pointcloud_cache_reusable': bool(cache_reusable),
                   'measured_surface_status': surface_state['freshness'],
                   'measured_surface_error': surface_state['error'],
                   'measured_surface_capture_wall_time': surface_state['capture_wall'],
                   'measured_surface_capture_monotonic': surface_state['capture_monotonic'],
                   'surface_visualization_mode': surface_state['visualization_mode'],
                   'surface_visualization_mean_luminance': surface_state['mean_luminance'],
                   'surface_visualization_note': surface_state['visualization_note'],
                   'displayed_pose_source': 'virtual preview override' if virtual_override[0] else 'last saved actual raw event',
                   'camera_image': str(current_pair['image']), 'camera_depth': str(current_pair['depth']),
                   'camera_keyframe': report.get('camera_keyframe'),
                   'camera_sample_pose_raw': report.get('camera_sample_pose_raw'),
                   'camera_timestamp_source': report.get('camera_sample_timestamp_source')}
        temporary = output / 'live-status.json.tmp'
        temporary.write_text(json.dumps(payload, indent=2) + '\n')
        os.replace(temporary, output / 'live-status.json')

    initial_camera_status = camera_sample_status(report, phase,
        cache_reusable=cache_status['reusable'])
    phase_status_handle.value = phase_label(phase)
    sample_status_handle.value = sample_label(initial_camera_status, cache_status['reusable'])
    publish_live_status(phase, initial_camera_status, cache_status['reusable'])

    def refresh_evidence():
        nonlocal observed_mesh, view_distance
        last_surface_refresh = [0.0]
        while True:
            try:
                current_phase = read_phase_evidence(phase_file) if phase_file else phase
                event_time = current_phase.get('event_monotonic')
                if current_phase.get('pose_raw') and event_time != last_phase_event[0] and calibration:
                    next_degrees = raw_pose_to_joint_degrees(current_phase['pose_raw'], calibration)
                    with lock:
                        degrees.update(next_degrees)
                        for key, slider in sliders.items():
                            if key in degrees:
                                slider.value = float(degrees[key])
                        set_pose(model, data, degrees, carriage.value, feeder.value)
                        update_model()
                        virtual_override[0] = False
                    last_phase_event[0] = event_time

                pair = ((select_latest_keyframe_pair_tail(watch_latest_saved_camera)
                         if (Path(watch_latest_saved_camera) / 'manifest.jsonl').is_file()
                         else select_latest_rgbd_pair(watch_latest_saved_camera)) if watch_latest_saved_camera
                        else {'image': current_pair['image'], 'depth': current_pair['depth'],
                              'timestamp_wall': sample_timestamp[0],
                              'timestamp_monotonic': sample_monotonic[0]})
                if pair is not None:
                    image_path, depth_path = Path(pair['image']), Path(pair['depth'])
                    signature = (_file_signature(image_path), _file_signature(depth_path))
                    if (image_path != current_pair['image'] or depth_path != current_pair['depth']
                            or signature != source_signature[0]):
                        now_mono = time.monotonic()
                        if now_mono - last_surface_refresh[0] < LIVE_SURFACE_REFRESH_INTERVAL_S:
                            time.sleep(LIVE_SURFACE_REFRESH_INTERVAL_S - (now_mono - last_surface_refresh[0]))
                            continue
                        before = (_file_signature(image_path), _file_signature(depth_path))
                        current_report = dict(report)
                        current_report['camera_sample_timestamp_wall'] = pair.get('timestamp_wall')
                        current_report['camera_sample_timestamp_monotonic'] = pair.get('timestamp_monotonic')
                        new_cloud, new_surface = _rgbd_frame(image_path, depth_path, current_report,
                            world_from_scene, max_surface_faces=face_capacity)
                        after = (_file_signature(image_path), _file_signature(depth_path))
                        if before != after or before != signature:
                            continue
                        refreshed_rgb = new_surface['display_texture']
                        surface_generation = int(surface_state.get('generation', 0)) + 1
                        next_mesh = server.scene.add_mesh_trimesh(
                            f'/measured/front_surfaces/{surface_generation}', new_surface['mesh'])
                        next_mesh.visible = False
                        new_center, new_position, new_distance = _view_frame(
                            new_surface['mesh'].vertices, anchors)
                        try:
                            with lock:
                                # The renderer callback reads model mesh and bounds under
                                # this same lock; keep the compiled arrays coherent per frame.
                                _update_compiled_surface(model, surface_geom_id, new_surface)
                                old_mesh = observed_mesh
                                observed_mesh = next_mesh
                                observed_mesh.visible = surface_toggle.value
                                if old_mesh is not None:
                                    old_mesh.remove()
                                surface_toggle.disabled = False
                                current_pair.update(image=image_path, depth=depth_path)
                                report['image'], report['depth'] = str(image_path), str(depth_path)
                                report['camera_keyframe'] = ({'index': pair.get('keyframe_index'),
                                    'phase': pair.get('phase'), 'pose_raw': pair.get('pose_raw'),
                                    'pose_synchronized': pair.get('pose_synchronized'),
                                    'phase_event_delta_s': pair.get('phase_event_delta_s'),
                                    'recorded': True} if pair.get('pose_synchronized') else None)
                                report['camera_sample_timestamp_source'] = ('synchronized keyframe manifest'
                                    if pair.get('pose_synchronized') else pair.get('timestamp_source', 'paired RGB/depth file mtime'))
                                photo_handle.image = refreshed_rgb
                                replace_clouds(new_cloud, show_focus=points_toggle.value and not context_toggle.value,
                                               show_context=points_toggle.value and context_toggle.value)
                                view_center[:] = new_center
                                view_position[:] = new_position
                                view_distance = new_distance
                                surface_state.update(available=True, freshness='fresh-live',
                                    capture_wall=float(pair.get('timestamp_wall') or time.time()),
                                    capture_monotonic=pair.get('timestamp_monotonic'), error=None,
                                    generation=surface_generation,
                                    visualization_mode=new_surface['visualization_mode'],
                                    mean_luminance=new_surface['mean_luminance'],
                                    visualization_note=('Depth view: low room light'
                                        if new_surface['visualization_mode'] == 'DEPTH_VIEW' else None))
                                view_mode_handle.value = (surface_state['visualization_note']
                                    or 'RGB photograph')
                                cache_state['valid'] = False
                        except Exception:
                            next_mesh.remove()
                            raise
                        source_signature[0] = signature
                        sample_timestamp[0] = float(pair['timestamp_wall'])
                        sample_monotonic[0] = pair.get('timestamp_monotonic')
                        report['camera_sample_pose_raw'] = pair.get('pose_raw')
                        report['camera_sample_pose_monotonic'] = pair.get('pose_monotonic')
                        report['camera_sample_pose_delta_seconds'] = pair.get('pose_sample_delta_seconds')
                        report['camera_sample_pose_qualified_sync'] = pair.get('pose_qualified_sync')
                        report['camera_sample_timestamp_wall'] = sample_timestamp[0]
                        report['camera_sample_timestamp_monotonic'] = sample_monotonic[0]
                        last_surface_refresh[0] = now_mono

                current_report = dict(report)
                current_report['camera_sample_timestamp_wall'] = sample_timestamp[0]
                current_report['camera_sample_timestamp_monotonic'] = sample_monotonic[0]
                capture_wall = surface_state.get('capture_wall')
                if capture_wall is not None and time.time() - capture_wall > .5:
                    surface_state['freshness'] = 'stale-live' if surface_state.get('generation') else 'stale-snapshot'
                sample_status = camera_sample_status(current_report, current_phase,
                    cache_reusable=cache_state['valid'])
                phase_status_handle.value = phase_label(current_phase)
                sample_status_handle.value = sample_label(sample_status, cache_state['valid'])
                publish_live_status(current_phase, sample_status, cache_state['valid'])
            except Exception as error:
                phase_status_handle.value = f'EVIDENCE REFRESH ERROR · {type(error).__name__}: {error}'
            time.sleep(.2)

    threading.Thread(target=refresh_evidence, name='digital-twin-evidence-refresh', daemon=True).start()
    print(f'Digital twin ready: http://{host}:{server.get_port()} | evidence {output / "scene-evidence.json"}', flush=True)
    return server
