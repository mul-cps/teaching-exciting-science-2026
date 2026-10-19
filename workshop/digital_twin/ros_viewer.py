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
from .virtual_camera import tool_from_virtual_optical


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


def _line_segment_colors(color, count):
    """Repeat one RGB color for both endpoints of each Viser line segment."""
    rgb = np.asarray(color, dtype=np.uint8)
    if rgb.shape != (3,) or int(count) < 1:
        raise ValueError("a line color must be RGB and segment count positive")
    return np.broadcast_to(rgb, (int(count), 2, 3)).copy()


def _root_polygon_on_camera_plane(points_root, camera_frame, distance_m):
    """Project a tracked root-frame polygon onto the visible RGB image plane."""
    if not camera_frame or camera_frame.get("root_from_camera") is None:
        return None
    points = np.asarray(points_root, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("tracked polygon must contain finite root-frame 3D points")
    root_from_camera = np.asarray(camera_frame["root_from_camera"], dtype=float).reshape(4, 4)
    intrinsics = np.asarray(camera_frame["camera_matrix"], dtype=float).reshape(3, 3)
    if (not np.isfinite(root_from_camera).all() or not np.isfinite(intrinsics).all()
            or intrinsics[0, 0] <= 0 or intrinsics[1, 1] <= 0
            or not np.isfinite(distance_m) or distance_m <= 0):
        raise ValueError("camera image plane requires finite transform, intrinsics, and distance")
    camera_from_root = np.linalg.inv(root_from_camera)
    points_camera = (points - root_from_camera[:3, 3]) @ camera_from_root[:3, :3].T
    if np.any(points_camera[:, 2] <= 1e-6):
        return None
    pixels = np.column_stack((
        intrinsics[0, 0] * points_camera[:, 0] / points_camera[:, 2] + intrinsics[0, 2],
        intrinsics[1, 1] * points_camera[:, 1] / points_camera[:, 2] + intrinsics[1, 2]))
    plane_camera = np.column_stack((
        (pixels[:, 0] - intrinsics[0, 2]) / intrinsics[0, 0] * distance_m,
        (pixels[:, 1] - intrinsics[1, 2]) / intrinsics[1, 1] * distance_m,
        np.full(len(points), distance_m)))
    return plane_camera @ root_from_camera[:3, :3].T + root_from_camera[:3, 3]


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


def _gripper_camera_pose(snapshot, *, forward_axis="+Z", backoff_m=.04):
    """Diagnostic virtual eye in the current tool frame; no camera calibration.

    A small retreat along the chosen axis keeps the eye behind the tool origin. The
    source cloud remains untouched and is rendered by the existing browser GPU.
    """
    mount = tool_from_virtual_optical(forward_axis, backoff_m)
    pose = transform_between(snapshot["transforms"],
        snapshot.get("root_frame", "charuco_board"), "gripper_frame_link",
        snapshot.get("transform_fresh", {})) @ mount
    if not np.isfinite(pose).all():
        raise ValueError("virtual eye requires finite fresh tool TF")
    return pose[:3, 3], pose[:3, 3] + pose[:3, 2] * .24, -pose[:3, 1]


def _cloud_is_fresh(cloud, now):
    if not cloud:
        return False
    received = cloud.get("received_monotonic")
    return (isinstance(received, (int, float)) and np.isfinite(received)
            and 0 <= now - received <= .5)


def serve_ros_twin(state_file, port=8099, host="0.0.0.0", poll_s=0.1):
    state_file = Path(state_file)
    server = viser.ViserServer(host=host, port=int(port), label="SO-ARM101 · ROS Passive Twin")
    server.scene.set_up_direction("+z")
    status = server.gui.add_markdown("**ROS passive twin**\nWaiting for authoritative RSP inputs.")
    placement_ui = server.gui.add_markdown(
        "**Placement goal observation**\nWaiting for synchronized RGB-D camera frames.")
    ego_toggle = server.gui.add_checkbox("Follow virtual gripper camera", False)
    ego_axis = server.gui.add_dropdown("Virtual eye forward (tool axes)",
        options=("+Z", "-Z", "+X", "-X", "+Y", "-Y"), initial_value="+Z")
    ego_backoff = server.gui.add_slider("Virtual eye retreat (mm)", min=0., max=150.,
        step=5., initial_value=40.)
    ego_ui = server.gui.add_markdown(
        "**Virtual gripper camera · OFF**\nUses the existing GPU scene, not a physical camera.")
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
                       "pending_action": None, "pending_request_id": None,
                       "notice_until": 0.}
    snapshot_cache = {}
    camera_plane_distance_m = .35
    placement_line = None
    selection_request_path = state_file.with_name("placement-region-request.json")

    def focus_camera(client, frame):
        ego_toggle.value = False
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
        goal_state = snapshot_cache.get("placement_goal", {})
        if (action == "select_placement_area"
                and not (goal_state.get("anchor_tracking_valid") or goal_state.get("valid"))):
            placement_notice("**Placement goal observation**\n"
                "Select and track the feeder anchor before selecting its placement area.")
            return
        request_id = uuid.uuid4().hex
        selection_state.update(mode=None, generation=None, corners=[],
            pending_action=action, pending_request_id=request_id)
        _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
            "action": "pin_latest_frame", "mode": action,
            "request_id": request_id})
        selection_ui_action = ("feeder anchor" if action == "select_feeder_anchor" else
                               "placement area")
        placement_notice((f"**Pinning {selection_ui_action} frame**\n"
            "Waiting for the ROS bridge to pin its newest fresh synchronized RGB-D frame. "
            "The image will freeze before corner clicks are enabled."))

    def activate_pinned_selection(frame):
        action = selection_state.get("pending_action")
        if action is None:
            return
        selection_state.update(mode=action, generation=frame["generation"], corners=[],
            pending_action=None, pending_request_id=None)
        selection_ui_action = ("feeder anchor" if action == "select_feeder_anchor" else
                               "placement area")
        placement_notice((f"**Select {selection_ui_action}** · frame `{frame['generation']}` pinned\n"
            "Click two opposite corners on this frozen RGB-D image. The selection expires "
            "after the displayed timeout."))
        for client in server.get_clients().values():
            focus_camera(client, frame)

    def cancel_selection(_event=None):
        generation = selection_state.get("generation")
        if generation:
            _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
                "action": "unpin_frame", "generation": generation})
        pending_request_id = selection_state.get("pending_request_id")
        if pending_request_id:
            _write_placement_selection_request(state_file, {"request_id": uuid.uuid4().hex,
                "action": "cancel_pin", "pin_request_id": pending_request_id})
        selection_state.update(mode=None, generation=None, corners=[],
            pending_action=None, pending_request_id=None)
        placement_ui.content = "**Placement goal observation**\nSelection cancelled."

    def clear_placement_selection(_event=None):
        generation = (selection_state.get("generation") or
                      (camera_plane.get("frame") or {}).get("generation", ""))
        selection_state.update(mode=None, generation=None, corners=[])
        selection_state.update(pending_action=None, pending_request_id=None)
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

    @ego_toggle.on_update
    def restore_overview(_event):
        if not ego_toggle.value:
            for client in server.get_clients().values():
                frame_client_once(client)

    server.on_client_connect(frame_client_once)
    try:
        while True:
            now = time.monotonic()
            try:
                snapshot = json.loads(state_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                status.content = "**ROS passive twin · WAITING**\nNo valid ROS state snapshot yet."
                if ego_toggle.value:
                    ego_ui.content = "**Virtual gripper camera · STALE / WAITING**\nState snapshot unavailable; view held."
                if cloud_handle is not None:
                    cloud_handle.visible = False
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
            pinned_frame = snapshot.get("placement_selection_frame")
            pending_request_id = selection_state.get("pending_request_id")
            pinned_ack = bool(pinned_frame and pending_request_id and
                pinned_frame.get("request_id") == pending_request_id and
                pinned_frame.get("mode") == selection_state.get("pending_action"))
            if (selection_state.get("mode") is not None and
                    (pinned_frame is None or
                     pinned_frame.get("generation") != selection_state.get("generation"))):
                selection_state.update(mode=None, generation=None, corners=[])
                placement_notice("**Placement goal observation · EXPIRED**\n"
                    "The pinned RGB-D generation expired. Select again on a fresh image.")
            display_frame = pinned_frame if pinned_ack else camera_frame
            camera_age = (float("inf") if not camera_frame else
                          max(0., now - float(camera_frame.get("received_monotonic", now))))
            display_age = (float("inf") if not display_frame else
                           max(0., now - float(display_frame.get("received_monotonic", now))))
            needs_pinned_display = pinned_ack and display_frame.get("generation") != camera_plane["generation"]
            needs_live_display = (not pinned_ack and display_frame and camera_age <= .5 and
                display_frame.get("generation") != camera_plane["generation"] and
                now - camera_plane["last_update"] >= .2)
            if (selection_state["mode"] is None and display_frame and
                    (needs_pinned_display or needs_live_display) and
                    (pinned_ack or display_age <= .5)):
                image_path = Path(display_frame.get("image_path", ""))
                try:
                    rgb = np.asarray(Image.open(image_path).convert("RGB"))
                    spec = _camera_image_plane_spec(display_frame, camera_plane_distance_m)
                    previous = camera_plane.get("handle")
                    if previous is not None:
                        previous.remove()
                    path = "/placement/camera_preview/" + str(display_frame["generation"])
                    handle = server.scene.add_image(path, rgb, spec["render_width"],
                        spec["render_height"], format="jpeg", jpeg_quality=78,
                        cast_shadow=False, receive_shadow=False,
                        position=spec["position"], wxyz=spec["wxyz"])
                    frozen = dict(display_frame)
                    handle.on_click(lambda event, frame=frozen: on_camera_click(frame, event))
                    camera_plane.update(handle=handle, frame=frozen,
                        generation=display_frame["generation"], last_update=now)
                    if pinned_ack:
                        activate_pinned_selection(frozen)
                except (OSError, ValueError, KeyError, TypeError) as error:
                    placement_ui.content = ("**Placement goal observation**\n"
                        f"Live camera image unavailable: `{error}`")
            elif pinned_ack and display_frame.get("generation") == camera_plane["generation"]:
                # The newest fresh frame happened to be the one already shown. Its
                # acquisition metadata and image are identical, so acknowledge it
                # without recreating the scene node.
                camera_plane["frame"] = dict(display_frame)
                activate_pinned_selection(camera_plane["frame"])
            if camera_plane["handle"] is not None:
                camera_plane["handle"].visible = not ego_toggle.value
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
                cloud = snapshot.get("point_cloud")
                ego_pose = None
                if ego_toggle.value and _cloud_is_fresh(cloud, now):
                    try:
                        ego_pose = _gripper_camera_pose(snapshot,
                            forward_axis=ego_axis.value, backoff_m=ego_backoff.value / 1000.)
                    except (KeyError, ValueError) as error:
                        ego_ui.content = f"**Virtual gripper camera · WAITING**\nFresh tool TF unavailable: `{error}`"
                elif ego_toggle.value:
                    ego_ui.content = "**Virtual gripper camera · WAITING**\nNo fresh depth cloud; view held."
                if ego_pose is not None:
                    for client in server.get_clients().values():
                        set_initial_camera(client, ego_pose)
                    ego_ui.content = (
                        "**Virtual gripper camera · FOLLOWING**\n"
                        f"`gripper_frame_link` · forward `{ego_axis.value}` · retreat "
                        f"`{ego_backoff.value:.0f} mm` · 60° vertical field of view.\n"
                        "Reprojected depth samples and diagnostic markers only. Holes, hidden "
                        "surfaces and backsides are unknown; sparse points do not certify free space. "
                        "Tool/camera registration is unqualified. The RGB preview plane is hidden.")
                elif not ego_toggle.value:
                    ego_ui.content = "**Virtual gripper camera · OFF**\nUses the existing GPU scene, not a physical camera."
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
                if cloud_handle is not None:
                    # The camera image plane is the interaction surface. Hide the
                    # live cloud while selecting so nearer sampled points cannot
                    # occlude its pixels or intercept clicks.
                    cloud_handle.visible = (selection_state["mode"] is None and _cloud_is_fresh(cloud, now))
                if _cloud_is_fresh(cloud, now) and selection_state["mode"] is None:
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
                    # Goal geometry is measured at the feeder depth, behind the
                    # synthetic RGB image plane. Project the tracked polygon onto
                    # that exact displayed image plane so the outline stays visible.
                    outline = _root_polygon_on_camera_plane(corners,
                        snapshot.get("placement_camera_frame"),
                        camera_plane_distance_m - .001)
                    if outline is None:
                        if placement_line is not None:
                            placement_line.visible = False
                    else:
                        points = np.stack([outline[[0, 1]], outline[[1, 2]],
                                           outline[[2, 3]], outline[[3, 0]]])
                        if placement_line is None:
                            placement_line = server.scene.add_line_segments(
                                "/placement/tracked_goal_region", points,
                                _line_segment_colors((40, 235, 135), len(points)), thickness=.003,
                                thickness_units="world")
                        else:
                            placement_line.points = points
                        placement_line.visible = not ego_toggle.value
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
                if ego_toggle.value:
                    ego_ui.content = "**Virtual gripper camera · STALE / WAITING**\nFresh robot pose unavailable; view held."
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
