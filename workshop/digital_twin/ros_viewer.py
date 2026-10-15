"""Passive MuJoCo + Viser twin driven only by a ROS-state snapshot file."""
from __future__ import annotations

import json
import time
from pathlib import Path

import mujoco
import numpy as np
import viser

from .mujoco_ros import (apply_ros_state, effective_urdf_to_mjcf_model,
                         compare_tool_frame, ros_pose_to_viewer_xyz, transform_between)


def _render(model, data, root_position):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = np.asarray(root_position) + [0., 0., .15]
    camera.distance, camera.azimuth, camera.elevation = 1.2, 135., -27.
    with mujoco.Renderer(model, height=640, width=800) as renderer:
        renderer.update_scene(data, camera=camera)
        return renderer.render()


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
                    matrix = np.asarray(transform["matrix"], dtype=float)
                    handle = frame_handles.get(name)
                    if handle is None:
                        handle = server.scene.add_frame("/tf/" + name, axes_length=.055,
                                                        axes_radius=.002, origin_radius=.003)
                        frame_handles[name] = handle
                    handle.position = matrix[:3, 3]
                    from scipy.spatial.transform import Rotation
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
                            object_handles[name].visible = False
                        continue
                    pose = np.asarray(item["matrix"], dtype=float)
                    try:
                        xyz = ros_pose_to_viewer_xyz(pose[:3, 3], item["frame_id"], snapshot)
                    except KeyError:
                        continue
                    handle = object_handles.get(name)
                    if handle is None:
                        handle = server.scene.add_icosphere("/scene" + name, radius=.012,
                                                            color=(245, 160, 30))
                        object_handles[name] = handle
                    handle.position = xyz
                    handle.visible = True
                delta = diagnostic.get("joint_tf_delta_s")
                status.content = (
                    "**ROS passive MuJoCo twin · FRESH**\n"
                    "Inputs: `/follower/robot_description`, `/follower/joint_states`, `/tf`, `/tf_static`; "
                    "scene: `/camera/depth/color/points`, `/so101/object_markers`, grasp topics.\n"
                    f"Joint/`base_link` TF delta: `{delta!s}` s · measured joints: `{len(joint.get('positions_rad', {}))}` · "
                    f"fresh TF frames: `{len(frame_rows)}` · {parity_line} · "
                    f"point cloud: `{cloud.get('source') if cloud else 'waiting'}`\n"
                    "Passive pose display; MuJoCo never publishes joint states or commands hardware.")
            else:
                reason = diagnostic.get("freshness", "waiting_for_ros_inputs")
                status.content = ("**ROS passive MuJoCo twin · STALE / WAITING**\n"
                    f"State: `{reason}` · joint age: `{diagnostic.get('joint_state_age_s')}` s · "
                    f"joint/`base_link` TF delta: `{diagnostic.get('joint_tf_delta_s')}` s · "
                    f"bridge snapshot age: `{snapshot_age:.3f}` s.\n"
                    "Cached TF and image are held; stale state is not presented as fresh.")
                for handle in frame_handles.values():
                    handle.visible = False
                for handle in object_handles.values():
                    handle.visible = False
                if cloud_handle is not None:
                    cloud_handle.visible = False
            time.sleep(poll_s)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
