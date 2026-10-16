"""Passive MuJoCo + Viser twin driven only by a ROS-state snapshot file."""
from __future__ import annotations

import json
import os
import tempfile
import threading
import time
import uuid
from pathlib import Path

import mujoco
import numpy as np
import viser
from scipy.spatial.transform import Rotation
from PIL import Image

from .mujoco_ros import (apply_ros_state, effective_urdf_to_mjcf_model,
                         compare_tool_frame, ros_pose_to_viewer_xyz, transform_between)
from .placement_region import intersect_ray_with_image_plane


def _camera_image_plane_spec(frame, distance_m=.35):
    """Return a visualization plane for one exact camera acquisition."""
    shape = frame.get("image_shape")
    matrix_value = frame.get("root_from_camera")
    matrix = np.eye(4) if matrix_value is None else np.asarray(matrix_value, dtype=float)
    intrinsics = np.asarray(frame.get("camera_matrix"), dtype=float).reshape(3, 3)
    if (not isinstance(shape, (list, tuple)) or len(shape) != 2 or
            any(int(value) <= 0 for value in shape) or matrix.shape != (4, 4) or
            not np.isfinite(matrix).all() or not np.isfinite(intrinsics).all() or
            intrinsics[0, 0] <= 0 or intrinsics[1, 1] <= 0 or
            not np.isfinite(distance_m) or distance_m <= 0):
        raise ValueError("camera image plane requires valid frame, intrinsics, and transform")
    height, width = map(int, shape)
    # Center the rendered image rectangle on the pixel-grid center. The
    # optical principal point can be offset from that center, so the center
    # ray is not necessarily the optical Z axis.
    center_camera = np.array([
        ((width / 2. - intrinsics[0, 2]) / intrinsics[0, 0]) * distance_m,
        ((height / 2. - intrinsics[1, 2]) / intrinsics[1, 1]) * distance_m,
        float(distance_m), 1.])
    center = matrix @ center_camera
    # Viser ImageHandle already rotates its plane by pi about local X to
    # orient the texture. Keep the node in optical-frame orientation so the
    # displayed source top-left remains pixel (0, 0), without changing TF.
    wxyz = Rotation.from_matrix(matrix[:3, :3]).as_quat()[[3, 0, 1, 2]]
    return {"position": center[:3], "wxyz": wxyz,
            "render_width": width / intrinsics[0, 0] * distance_m,
            "render_height": height / intrinsics[1, 1] * distance_m,
            "distance_m": float(distance_m),
            "camera_transform_qualified": matrix_value is not None}


def _write_placement_selection_request(state_file, request):
    """Atomically hand one bounded UI selection to the ROS-only adapter."""
    target = Path(state_file).with_name("placement-region-request.json")
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".placement-request-", dir=target.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(request, stream, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _placement_rect_from_pixels(corners_xy, image_shape, *, min_extent_px=4):
    """Bound two UI clicks to one in-image half-open pixel rectangle."""
    corners = np.asarray(corners_xy, dtype=float)
    if corners.shape != (2, 2) or not np.isfinite(corners).all():
        raise ValueError("two finite clicked image pixels are required")
    height, width = map(int, image_shape)
    if height <= 0 or width <= 0:
        raise ValueError("positive image dimensions required")
    low = np.floor(corners.min(axis=0)).astype(int)
    high = np.ceil(corners.max(axis=0)).astype(int)
    x0, y0 = np.clip(low, [0, 0], [width, height])
    x1, y1 = np.clip(high, [0, 0], [width, height])
    if x1 - x0 < min_extent_px or y1 - y0 < min_extent_px:
        raise ValueError(f"selection must be at least {min_extent_px} pixels in both dimensions")
    return [int(x0), int(y0), int(x1), int(y1)]


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
    placement_ui = server.gui.add_markdown(
        "**Placement goal observation**\nWaiting for synchronized RGB-D camera frames.")
    frame_camera_button = server.gui.add_button("Frame live placement camera")
    anchor_button = server.gui.add_button("Select feeder tracking anchor")
    goal_button = server.gui.add_button("Select placement area")
    cancel_selection_button = server.gui.add_button("Cancel area selection")
    clear_selection_button = server.gui.add_button("Clear placement selection")
    image_handle = None
    cloud_handle = None
    frame_handles = {}
    object_handles = {}
    model = data = None
    description_sha = None
    last_image_update = 0.
    camera_state = {"pose": None, "pending_clients": set()}
    camera_state_lock = threading.Lock()
    camera_plane = {"handle": None, "frame": None, "generation": None,
                    "last_update": 0.}
    selection_state = {"mode": None, "generation": None, "corners": [],
                       "notice_until": 0.}
    snapshot_cache = {}
    camera_plane_distance_m = .35
    placement_line = None
    selection_request_path = state_file.with_name("placement-region-request.json")

    def focus_camera(client, frame):
        spec = _camera_image_plane_spec(frame, camera_plane_distance_m)
        root_from_camera = (np.eye(4) if frame.get("root_from_camera") is None else
                            np.asarray(frame["root_from_camera"], dtype=float))
        optical_origin = root_from_camera[:3, 3]
        optical_forward = root_from_camera[:3, 2]
        center = np.asarray(spec["position"], dtype=float)
        client.camera.position = optical_origin - optical_forward * .14
        client.camera.look_at = center
        client.camera.up_direction = -root_from_camera[:3, 1]
        client.camera.fov = float(np.deg2rad(42.))

    def placement_notice(content, duration_s=2.):
        selection_state["notice_until"] = time.monotonic() + max(4., float(duration_s))
        placement_ui.content = content

    def arm_selection(action):
        frame = camera_plane["frame"]
        if frame is None or camera_plane["handle"] is None:
            placement_notice("**Placement goal observation**\nNo live camera image is available yet.")
            return
        age = time.monotonic() - float(frame.get("received_monotonic", 0.))
        if age < 0 or age > .4:
            placement_notice(("**Placement goal observation · STALE**\n"
                "Wait for a fresh RGB-D frame, then select again."))
            return
        goal_state = snapshot_cache.get("placement_goal", {})
        if (action == "select_placement_area"
                and not (goal_state.get("anchor_tracking_valid") or goal_state.get("valid"))):
            placement_notice("**Placement goal observation**\n"
                "Select and track the feeder anchor before selecting its placement area.")
            return
        frame = dict(frame)
        _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
            "action": "pin_frame", "generation": frame["generation"]})
        selection_state.update(mode=action, generation=frame["generation"], corners=[])
        selection_ui_action = ("feeder anchor" if action == "select_feeder_anchor"
                               else "placement area")
        placement_notice((f"**Select {selection_ui_action}**\n"
            "Click two opposite corners on the displayed camera image. The acquisition is "
            "pinned for a bounded interval; expired selections are rejected."))
        for client in server.get_clients().values():
            focus_camera(client, frame)

    def cancel_selection(_event=None):
        generation = selection_state.get("generation")
        if generation:
            _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
                "action": "unpin_frame", "generation": generation})
        selection_state.update(mode=None, generation=None, corners=[])
        placement_ui.content = "**Placement goal observation**\nSelection cancelled."

    def clear_placement_selection(_event=None):
        generation = (selection_state.get("generation") or
                      (camera_plane.get("frame") or {}).get("generation", ""))
        selection_state.update(mode=None, generation=None, corners=[])
        _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
            "action": "clear_selection", "generation": generation})
        placement_ui.content = "**Placement goal observation**\nClearing selected feeder and area."

    def on_camera_click(frame, event):
        if (selection_state.get("mode") is None
                or frame.get("generation") != selection_state.get("generation")):
            return
        point = intersect_ray_with_image_plane(event.ray_origin, event.ray_direction,
            np.eye(4) if frame.get("root_from_camera") is None else frame["root_from_camera"],
            frame["camera_matrix"], frame["image_shape"],
            distance_m=camera_plane_distance_m)
        if point is None:
            placement_ui.content = ("**Select camera image area**\n"
                "Click on the camera image plane; the clicked point was outside it.")
            return
        selection_state["corners"].append(point)
        if len(selection_state["corners"]) == 1:
            placement_ui.content = ("**Select camera image area**\n"
                "First corner captured. Click the opposite corner on the same pinned frame.")
            return
        points = np.asarray(selection_state["corners"], dtype=float)
        action = selection_state["mode"]
        generation = selection_state["generation"]
        selection_state.update(mode=None, generation=None, corners=[])
        try:
            rectangle = _placement_rect_from_pixels(points, frame["image_shape"])
        except ValueError as error:
            placement_ui.content = ("**Placement goal observation**\n"
                f"{error}")
            return
        _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
            "action": action, "generation": generation,
            "rect_xyxy": rectangle})
        placement_ui.content = ("**Placement goal observation**\n"
            "Selection request sent. Waiting for RGB-D depth and feature validation.")

    def frame_camera(_event):
        frame = camera_plane.get("frame")
        if frame is None:
            placement_ui.content = "**Placement goal observation**\nWaiting for live camera frame."
            return
        for client in server.get_clients().values():
            focus_camera(client, frame)

    frame_camera_button.on_click(frame_camera)
    anchor_button.on_click(lambda _event: arm_selection("select_feeder_anchor"))
    goal_button.on_click(lambda _event: arm_selection("select_placement_area"))
    cancel_selection_button.on_click(cancel_selection)
    clear_selection_button.on_click(clear_placement_selection)

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
            snapshot_cache.clear()
            snapshot_cache.update(snapshot)
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
            camera_frame = snapshot.get("placement_camera_frame")
            camera_age = (float("inf") if not camera_frame else
                          max(0., now - float(camera_frame.get("received_monotonic", now))))
            if (camera_frame and camera_age <= .5 and selection_state["mode"] is None
                    and camera_frame.get("generation") != camera_plane["generation"]
                    and now - camera_plane["last_update"] >= .2):
                image_path = Path(camera_frame.get("image_path", ""))
                try:
                    rgb = np.asarray(Image.open(image_path).convert("RGB"))
                    spec = _camera_image_plane_spec(camera_frame, camera_plane_distance_m)
                    previous = camera_plane.get("handle")
                    if previous is not None:
                        previous.remove()
                    path = "/placement/camera_preview/" + str(camera_frame["generation"])
                    handle = server.scene.add_image(path, rgb, spec["render_width"],
                        spec["render_height"], format="jpeg", jpeg_quality=78,
                        cast_shadow=False, receive_shadow=False,
                        position=spec["position"], wxyz=spec["wxyz"])
                    frozen = dict(camera_frame)
                    handle.on_click(lambda event, frame=frozen: on_camera_click(frame, event))
                    camera_plane.update(handle=handle, frame=frozen,
                        generation=camera_frame["generation"], last_update=now)
                except (OSError, ValueError, KeyError, TypeError) as error:
                    placement_ui.content = ("**Placement goal observation**\n"
                        f"Live camera image unavailable: `{error}`")
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
                if cloud_handle is not None:
                    # The camera image plane is the interaction surface. Hide the
                    # live cloud while selecting so nearer sampled points cannot
                    # occlude its pixels or intercept clicks.
                    cloud_handle.visible = selection_state["mode"] is None
                if cloud and cloud_age <= 0.5 and selection_state["mode"] is None:
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
                goal_status = snapshot.get("placement_goal", {})
                goal_corners = goal_status.get("goal_corners_root")
                if goal_status.get("valid") and goal_corners is not None:
                    corners = np.asarray(goal_corners, dtype=float).reshape(4, 3)
                    points = np.stack([corners[[0, 1]], corners[[1, 2]],
                                       corners[[2, 3]], corners[[3, 0]]])
                    if placement_line is None:
                        placement_line = server.scene.add_line_segments(
                            "/placement/tracked_goal_region", points,
                            np.tile([40, 235, 135], (4, 1)), thickness=.003,
                            thickness_units="world")
                    else:
                        placement_line.points = points
                    placement_line.visible = True
                elif placement_line is not None:
                    placement_line.visible = False
                if (goal_status and selection_state["mode"] is None
                        and now >= selection_state["notice_until"]):
                    track_text = ("anchor and area tracked" if goal_status.get("valid") else
                                  "anchor tracked; area not valid" if goal_status.get("anchor_tracking_valid") else
                                  str(goal_status.get("reason", "waiting")))
                    if goal_status.get("identity_reacquired_after_gap"):
                        track_text += "; identity reacquired after an unobserved gap"
                    residual = goal_status.get("fit_rms_m")
                    residual_text = ("n/a" if residual is None else f"{1000 * residual:.1f} mm")
                    geometry = goal_status.get("goal_geometry") or {}
                    dimensions_text = ("n/a" if not geometry else
                        f"{1000 * geometry.get('width_m', 0.):.0f} × "
                        f"{1000 * geometry.get('height_m', 0.):.0f} mm")
                    normal = geometry.get("normal_camera")
                    normal_text = ("n/a" if normal is None else
                                   ", ".join(f"{value:+.2f}" for value in normal))
                    plane = goal_status.get("plane_metrics") or {}
                    plane_text = ("n/a" if plane.get("plane_rms_m") is None else
                                  f"{1000 * plane['plane_rms_m']:.1f} mm")
                    placement_ui.content = (
                        "**Placement goal observation · " +
                        ("VALID CAMERA TRACK" if goal_status.get("valid") else "NOT VALID") + "**\n"
                        f"State: `{track_text}` · camera `{goal_status.get('camera_frame', 'waiting')}` · "
                        f"3D fit residual: `{residual_text}` · goal generation: "
                        f"`{goal_status.get('camera_from_goal_stamp_ns', 'not selected')}` · "
                        f"selection `{goal_status.get('selection_id', 'none')}`.\n"
                        f"Observed area: `{dimensions_text}` · camera normal `[{normal_text}]` · "
                        f"depth-plane RMS: `{plane_text}`.\n"
                        "Camera-frame tracking is diagnostic only. Base registration and placement "
                        "motion admission remain unqualified.")
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
                if placement_line is not None:
                    placement_line.visible = False
                goal_status = snapshot.get("placement_goal", {})
                placement_ui.content = (
                    "**Placement goal observation · STALE / WAITING**\n"
                    f"State: `{goal_status.get('reason', 'waiting_for_camera')}` · "
                    "No current goal pose is shown when acquisition freshness is lost.")
            time.sleep(poll_s)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
