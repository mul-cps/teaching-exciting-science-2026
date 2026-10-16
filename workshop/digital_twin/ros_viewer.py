"""Passive MuJoCo + Viser twin driven only by a ROS-state snapshot file."""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import viser
from scipy.spatial.transform import Rotation

from .mujoco_ros import (apply_ros_state, effective_urdf_to_mjcf_model,
                         compare_tool_frame, ros_pose_to_viewer_xyz, transform_between)


def _pose_matrix_in_root(snapshot, frame_id, local_pose=None):
    """Compose a frame-local ROS pose into the configured twin root frame."""
    root = snapshot.get("root_frame", "charuco_board")
    root_from_frame = transform_between(snapshot["transforms"], root, frame_id,
                                        snapshot.get("transform_fresh"))
    if local_pose is None:
        return root_from_frame
    return root_from_frame @ np.asarray(local_pose, dtype=float).reshape(4, 4)


def _scene_object_spec(name, item):
    """Map supported ROS Marker/PoseArray records to faithful viewer geometry."""
    source = item.get("source")
    if source == "/so101/object_markers":
        kind = item.get("kind")
        if kind == 1:  # visualization_msgs/Marker.CUBE
            dimensions = np.asarray(item.get("scale_m", ()), dtype=float)
            rgba = np.asarray(item.get("color_rgba", ()), dtype=float)
            if (dimensions.shape != (3,) or not np.isfinite(dimensions).all()
                    or np.any(dimensions <= 0) or rgba.shape != (4,)
                    or not np.isfinite(rgba).all()):
                return None
            rgb = tuple(int(round(value * 255)) for value in np.clip(rgba[:3], 0., 1.))
            return {"kind": "box", "dimensions": tuple(map(float, dimensions)),
                    "color": rgb, "opacity": float(np.clip(rgba[3], 0., 1.))}
        if kind == 9:  # visualization_msgs/Marker.TEXT_VIEW_FACING
            text = str(item.get("text", ""))
            if not text:
                return None
            scale = np.asarray(item.get("scale_m", ()), dtype=float)
            height = float(scale[2]) if scale.shape == (3,) and np.isfinite(scale[2]) else .015
            return {"kind": "label", "text": text,
                    "font_scene_height": max(.006, height)}
        return None
    if source == "/so101/grasp_contacts":
        return {"kind": "point", "color": (255, 120, 30), "radius": .004}
    if source == "/so101/grasp_candidates":
        return {"kind": "point", "color": (40, 170, 255), "radius": .004}
    return None


def _upsert_scene_object(scene, handles, name, item, snapshot):
    spec = _scene_object_spec(name, item)
    if spec is None:
        previous = handles.get(name)
        if previous is not None:
            previous["handle"].visible = False
        return
    pose = _pose_matrix_in_root(snapshot, item["frame_id"], item["matrix"])
    position = pose[:3, 3]
    wxyz = Rotation.from_matrix(pose[:3, :3]).as_quat()[[3, 0, 1, 2]]
    previous = handles.get(name)
    if previous is not None and previous["spec"] != spec:
        previous["handle"].remove()
        previous = None
    if previous is None:
        path = "/scene" + ("/" + name.lstrip("/") if name else "")
        if spec["kind"] == "box":
            handle = scene.add_box(path, dimensions=spec["dimensions"], color=spec["color"],
                                  opacity=spec["opacity"], position=position, wxyz=wxyz)
        elif spec["kind"] == "label":
            handle = scene.add_label(path, spec["text"], position=position, wxyz=wxyz,
                                     font_scene_height=spec["font_scene_height"], depth_test=True)
        else:
            handle = scene.add_icosphere(path, radius=spec["radius"], color=spec["color"],
                                         position=position, wxyz=wxyz)
        handles[name] = {"spec": spec, "handle": handle}
    else:
        handle = previous["handle"]
        handle.position = position
        if hasattr(handle, "wxyz"):
            handle.wxyz = wxyz
    handle.visible = True


def _render(model, data, root_position, *, width=800, height=640):
    visual = model.vis.global_
    visual.offwidth = max(int(visual.offwidth), int(width))
    visual.offheight = max(int(visual.offheight), int(height))
    camera = mujoco.MjvCamera()
    camera.lookat[:] = np.asarray(root_position) + [0., 0., .15]
    camera.distance, camera.azimuth, camera.elevation = 1.2, 135., -27.
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera=camera)
        return renderer.render()


def _initial_camera_pose(model, data):
    """Fit a one-time client view to the effective robot geometry.

    Perception markers and camera point clouds are deliberately excluded:
    transient object detections and room/depth outliers should not determine
    the initial station framing.
    """
    centers = np.asarray(data.geom_xpos, dtype=float).reshape(-1, 3)
    radii = np.asarray(model.geom_rbound, dtype=float).reshape(-1, 1)
    if not len(centers) or not np.isfinite(centers).all() or not np.isfinite(radii).all():
        raise ValueError("effective model has no finite geometry bounds for camera framing")
    low = np.min(centers - radii, axis=0)
    high = np.max(centers + radii, axis=0)
    target = (low + high) * .5
    half_extent = (high - low) * .5
    radius = float(np.linalg.norm(half_extent))
    fov = np.deg2rad(60.)
    distance = float(np.clip(radius / np.sin(fov * .5) * 1.05, .55, 1.4))
    direction = np.asarray([1., -1., .72])
    direction /= np.linalg.norm(direction)
    position = target + direction * distance
    up = np.asarray([0., 0., 1.])
    return position, target, up


def serve_ros_twin(state_file, port=8099, host="0.0.0.0", poll_s=0.1):
    state_file = Path(state_file)
    server = viser.ViserServer(host=host, port=int(port), label="SO-ARM101 · ROS Passive Twin")
    server.scene.set_up_direction("+z")
    status = server.gui.add_markdown("**ROS passive twin**\nWaiting for authoritative RSP inputs.")
    image_handle = None
    cloud_handle = None
    frame_handles = {}
    object_handles = {}
    model = data = None
    description_sha = None
    last_image_update = 0.
    camera_state = {"pose": None, "pending_clients": set()}
    camera_state_lock = threading.Lock()

    def set_initial_camera(client, pose):
        position, look_at, up = pose
        client.camera.position = position
        client.camera.look_at = look_at
        client.camera.up_direction = up
        client.camera.fov = float(np.deg2rad(60.))

    def frame_client_once(client):
        with camera_state_lock:
            pose = camera_state["pose"]
            if pose is None:
                camera_state["pending_clients"].add(client.client_id)
                return
            camera_state["pending_clients"].discard(client.client_id)
        set_initial_camera(client, pose)

    server.on_client_connect(frame_client_once)
    try:
        while True:
            now = time.monotonic()
            try:
                snapshot = json.loads(state_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                status.content = "**ROS passive twin · WAITING**\nNo valid ROS state snapshot yet."
                time.sleep(poll_s)
                continue
            snapshot_age = max(0., time.time() - state_file.stat().st_mtime)
            description = snapshot.get("robot_description")
            if description:
                digest = hash(description)
                if digest != description_sha:
                    model = effective_urdf_to_mjcf_model(description)
                    data = mujoco.MjData(model)
                    description_sha = digest
                    with camera_state_lock:
                        camera_state["pose"] = None
                    status.content = "**ROS passive twin · MODEL LOADED**\nUsing live `/follower/robot_description`."
            diagnostic = snapshot.get("diagnostics", {})
            diagnostic["bridge_snapshot_age_s"] = snapshot_age
            fresh = (diagnostic.get("freshness") == "fresh" and snapshot_age <= 0.5)
            if not fresh and snapshot_age > 0.5:
                diagnostic["freshness"] = "stale_bridge_snapshot"
            joint = snapshot.get("joint_state") or {}
            if model is not None and fresh:
                applied = apply_ros_state(model, data, snapshot, require_fresh=True)
                base = applied["world_from_base"][:3, 3]
                if camera_state["pose"] is None:
                    pose = _initial_camera_pose(model, data)
                    with camera_state_lock:
                        if camera_state["pose"] is None:
                            camera_state["pose"] = pose
                with camera_state_lock:
                    pending = camera_state["pending_clients"].copy()
                if pending:
                    for client in server.get_clients().values():
                        if client.client_id in pending:
                            frame_client_once(client)
                pixels = _render(model, data, base)
                try:
                    parity = compare_tool_frame(model, data, snapshot, description)
                    parity_line = (f"RSP/MuJoCo `{parity['frame']}` error: "
                        f"{parity['translation_error_m'] * 1000:.1f} mm / "
                        f"{parity['rotation_error_deg']:.2f} deg")
                except (KeyError, ValueError) as error:
                    parity_line = f"RSP/MuJoCo tool parity unavailable: `{error}`"
                if image_handle is None:
                    image_handle = server.gui.add_image(pixels, label="MuJoCo · ROS state", format="jpeg")
                else:
                    image_handle.image = pixels
                last_image_update = now
                frame_rows = []
                for name, transform in snapshot.get("transforms", {}).items():
                    if not snapshot.get("transform_fresh", {}).get(name, False):
                        if name in frame_handles:
                            frame_handles[name].visible = False
                        continue
                    try:
                        matrix = _pose_matrix_in_root(snapshot, name)
                    except KeyError:
                        if name in frame_handles:
                            frame_handles[name].visible = False
                        continue
                    handle = frame_handles.get(name)
                    if handle is None:
                        handle = server.scene.add_frame("/tf/" + name, axes_length=.055,
                                                        axes_radius=.002, origin_radius=.003)
                        frame_handles[name] = handle
                    handle.position = matrix[:3, 3]
                    handle.wxyz = Rotation.from_matrix(matrix[:3, :3]).as_quat()[[3, 0, 1, 2]]
                    handle.visible = True
                    frame_rows.append(name)
                cloud = snapshot.get("point_cloud")
                cloud_age = (now - cloud.get("received_monotonic", now)) if cloud else float("inf")
                if cloud and cloud_age <= 0.5:
                    pts = np.asarray(cloud.get("points_m", []), dtype=np.float32)
                    if len(pts):
                        root_from_cloud = transform_between(snapshot["transforms"],
                            snapshot.get("root_frame", "charuco_board"), cloud["frame_id"],
                            snapshot.get("transform_fresh"))
                        points_world = (pts @ root_from_cloud[:3, :3].T
                                        + root_from_cloud[:3, 3]).astype(np.float32)
                        if cloud_handle is not None:
                            cloud_handle.remove()
                        cloud_handle = server.scene.add_point_cloud("/scene/ros_pointcloud",
                            points=points_world, colors=np.tile([145, 175, 190], (len(pts), 1)),
                            point_size=.0025)
                objects = snapshot.get("objects", {})
                for name, item in objects.items():
                    if now - item.get("received_monotonic", now) > 0.5:
                        if name in object_handles:
                            object_handles[name]["handle"].visible = False
                        continue
                    try:
                        _upsert_scene_object(server.scene, object_handles, name, item, snapshot)
                    except KeyError:
                        if name in object_handles:
                            object_handles[name]["handle"].visible = False
                delta = diagnostic.get("joint_tf_delta_s")
                sync_basis = diagnostic.get("joint_tf_sync_basis", "dynamic_base_stamp")
                sync_text = (f"joint/arm TF delta: `{delta!s}` s; static base anchor"
                             if sync_basis == "dynamic_arm_tf_stamps_with_static_base" else
                             f"joint/arm TF delta: `{delta!s}` s")
                status.content = (
                    "**ROS passive MuJoCo twin · FRESH**\n"
                    "Inputs: `/follower/robot_description`, `/follower/joint_states`, `/tf`, `/tf_static`; "
                    "scene: `/camera/depth/color/points`, `/so101/object_markers`, grasp topics.\n"
                    f"{sync_text} · measured joints: `{len(joint.get('positions_rad', {}))}` · "
                    f"fresh TF frames: `{len(frame_rows)}` · {parity_line} · "
                    f"point cloud: `{cloud.get('source') if cloud else 'waiting'}`\n"
                    "Passive pose display; MuJoCo never publishes joint states or commands hardware.")
            else:
                reason = diagnostic.get("freshness", "waiting_for_ros_inputs")
                sync_basis = diagnostic.get("joint_tf_sync_basis", "dynamic_base_stamp")
                sync_text = (f"joint/arm TF delta: `{diagnostic.get('joint_tf_delta_s')}` s; static base anchor"
                             if sync_basis == "dynamic_arm_tf_stamps_with_static_base" else
                             f"joint/arm TF delta: `{diagnostic.get('joint_tf_delta_s')}` s")
                status.content = ("**ROS passive MuJoCo twin · STALE / WAITING**\n"
                    f"State: `{reason}` · joint age: `{diagnostic.get('joint_state_age_s')}` s · "
                    f"{sync_text} · "
                    f"bridge snapshot age: `{snapshot_age:.3f}` s.\n"
                    "Cached TF and image are held; stale state is not presented as fresh.")
                for handle in frame_handles.values():
                    handle.visible = False
                for record in object_handles.values():
                    record["handle"].visible = False
                if cloud_handle is not None:
                    cloud_handle.visible = False
            time.sleep(poll_s)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
