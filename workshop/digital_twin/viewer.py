"""MuJoCo forward kinematics/rendering displayed through an interactive Viser view."""
import json
import math
from pathlib import Path
import threading

import mujoco
import numpy as np
from PIL import Image
import trimesh
import viser

from .kinematics import set_pose, export_pose_pair




def render(model, data, path, width=960, height=640):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .13]
    camera.distance = 1.25
    camera.azimuth = 135
    camera.elevation = -27
    with mujoco.Renderer(model, height, width) as renderer:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
    Image.fromarray(pixels).save(path)
    return pixels


def serve(output, port=8096, host='127.0.0.1'):
    output = Path(output)
    report = json.loads((output / 'scene-evidence.json').read_text())
    model = mujoco.MjModel.from_xml_path(str(output / 'scene.xml'))
    data = mujoco.MjData(model)
    degrees = dict(report['arm'].get('model_joint_degrees', report['arm']['nominal_joint_degrees']))
    set_pose(model, data, degrees)
    report['kinematics'], report['visual_fit_kinematics'] = export_pose_pair(model, data, report['arm'])
    from .comparison import compare_camera
    report['camera_comparison'] = compare_camera(model, data, report, output)
    (output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
    initial_render = render(model, data, output / 'mujoco-render.png')
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
    surface_data = np.load(output / 'measured-surface.npz')
    observed_surface = trimesh.Trimesh(vertices=surface_data['vertices'], faces=surface_data['faces'], process=False)
    observed_surface.visual.vertex_colors = np.column_stack((surface_data['colors'], np.full(len(surface_data['colors']),255)))
    observed_mesh = server.scene.add_mesh_trimesh('/measured/front_surfaces', observed_surface)
    cloud_data = np.load(output / 'pointcloud.npz')
    cloud = server.scene.add_point_cloud('/measured/rgbd_focus', cloud_data['focused_points'], cloud_data['focused_colors'], point_size=.003, point_shape='circle', precision='float32')
    full_cloud = server.scene.add_point_cloud('/measured/rgbd_context', cloud_data['points'], cloud_data['colors'], point_size=.003, point_shape='circle', precision='float32', visible=False)
    camera_transform = np.array(report['world_from_camera'])
    q = np.zeros(4); mujoco.mju_mat2Quat(q, camera_transform[:3, :3].reshape(-1))
    rgb = np.asarray(Image.open(report['image']).convert('RGB'))
    intrinsic = report['camera']['intrinsics']
    frustum = server.scene.add_camera_frustum('/measured/realsense', fov=2 * math.atan(rgb.shape[0] / (2 * intrinsic['fy'])),
                                    aspect=rgb.shape[1] / rgb.shape[0], scale=.035, position=camera_transform[:3, 3], wxyz=q,
                                    variant='wireframe', visible=False)
    server.scene.add_frame('/measured/table_frame', axes_length=.12, axes_radius=.002)
    # Viser's built-in label color is not configurable. Use unlit dark raster
    # labels with a contrasting background; orient them toward the active viewer.
    from PIL import ImageDraw, ImageFont
    label_handles = []
    for name, text, position in (
        ('arm', 'SO-ARM101 · official MuJoCo CAD\nVisual registration', np.array(report['arm']['position_estimate_m']) + [0, 0, .40]),
        ('ev3', 'EV3 · A feeder / D carriage\nEstimated geometry / rail scale', np.array(report['ev3']['position_estimate_m']) + [0, 0, .42])):
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
    with server.gui.add_folder('Evidence / registration'):
        server.gui.add_markdown('**Offline RGB-D snapshot + MuJoCo kinematics**\n\nCamera intrinsics/depth are measured. '
                                'Table frame is fitted. **Camera-to-arm transform, joint-zero mapping, sorter dimensions and robot registration remains unqualified.** '
                                'Controls change this model only; there are no device interfaces.\n\n'
                                f"{report['point_count']:,} measured points · table residual p95 {report['table_fit']['residual_p95_m'] * 1000:.1f} mm.\n\n"
                                'Model joint positions come from the saved encoder capture.')
        if 'user_visual_alignment' in report['arm']: server.gui.add_markdown(f"**User-aligned base yaw {report['arm']['orientation_estimate_deg']:.1f}°. Visual registration remains unqualified.**")
        server.gui.add_markdown(f"CAD fit median {report['arm']['registration']['surface_residual_median_m']*1000:.1f} mm / p95 {report['arm']['registration']['surface_residual_p95_m']*1000:.1f} mm. Single pose; no holdout qualification.")
        surface_toggle = server.gui.add_checkbox('Measured textured front surfaces', True)
        surface_toggle.on_update(lambda event: setattr(observed_mesh, 'visible', event.target.value))
        points_toggle = server.gui.add_checkbox('Measured RGB-D points', False)
        cloud.visible = False
        model_toggle = server.gui.add_checkbox('Official SO101 CAD model', True)
        context_toggle = server.gui.add_checkbox('Full room context (chair / walls)', False)
        frustum_toggle = server.gui.add_checkbox('Small camera frustum', False)
        def show_cloud(event):
            cloud.visible = points_toggle.value and not context_toggle.value
            full_cloud.visible = points_toggle.value and context_toggle.value
        points_toggle.on_update(show_cloud); context_toggle.on_update(show_cloud)
        frustum_toggle.on_update(lambda event: setattr(frustum, 'visible', event.target.value))
        def show_model(event):
            for _, handle in handles: handle.visible = event.target.value
        model_toggle.on_update(show_model)
        server.gui.add_image(rgb, label='Measured camera snapshot (photograph)')
    sliders = {}
    with server.gui.add_folder('Virtual SO-ARM101 joints'):
        for joint, initial in degrees.items():
            low, high, unit = (0, 100, '%') if joint == 'gripper' else (-180, 180, 'deg')
            sliders[joint] = server.gui.add_slider(joint + ' ' + unit, min=low, max=high, step=.2, initial_value=float(initial))
    with server.gui.add_folder('Virtual EV3 sorter'):
        carriage = server.gui.add_slider('Carriage displacement · estimated m', min=0., max=.32, step=.001, initial_value=0.)
        feeder = server.gui.add_slider('Feeder angle · model deg', min=-180., max=180., step=1., initial_value=0.)
    with server.gui.add_folder('Unmeasured arm registration · manual visual alignment'):
        offsets = [server.gui.add_slider('Base ' + axis + ' offset m', min=-.4, max=.4, step=.005, initial_value=0.) for axis in 'xy']
        yaw = server.gui.add_slider('Base yaw estimate deg', min=-180., max=180., step=1., initial_value=float((report['arm']['orientation_estimate_deg']+180)%360-180))
        server.gui.add_markdown('These controls are **visual estimates**, not camera-to-robot calibration. They do not overwrite evidence or a robot configuration.')
    arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    base_position = model.body_pos[arm_id].copy()
    def change(event):
        with lock:
            degrees.update({key: value.value for key, value in sliders.items()})
            model.body_pos[arm_id] = base_position + [offsets[0].value, offsets[1].value, 0]
            angle = math.radians(yaw.value) / 2
            model.body_quat[arm_id] = [math.cos(angle), 0, 0, math.sin(angle)]
            set_pose(model, data, degrees, carriage.value, feeder.value)
            update_model()
    for handle in [*sliders.values(), carriage, feeder, *offsets, yaw]: handle.on_update(change)
    with server.gui.add_folder('MuJoCo render · estimated geometry'):
        render_image = server.gui.add_image(initial_render, label='MuJoCo render (not a photograph)')
        button = server.gui.add_button('Render current virtual scene')
        def render_current(event):
            with lock: render_image.image = render(model, data, output / 'mujoco-render-current.png')
        button.on_click(render_current)
    @server.on_client_connect
    def connect(client):
        client.camera.position = (.85, -.85, .70)
        client.camera.look_at = (0, 0, .14)
        client.camera.up_direction = (0, 0, 1)
        @client.camera.on_update
        def camera_changed(event): face_labels(client.camera.position)
    print(f'Digital twin ready: http://{host}:{server.get_port()} | evidence {output / "scene-evidence.json"}', flush=True)
    return server
