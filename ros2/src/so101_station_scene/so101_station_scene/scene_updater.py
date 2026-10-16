"""Apply only tracked color-brick boxes to a MoveIt planning-scene diff."""

import json
import math
import time
import copy

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from .scene_geometry import (
    LiveObjectRegistry,
    object_collision_id,
    quaternion_matrix_xyzw,
    stamp_is_fresh,
    table_patches_around_keepout,
    transform_box_pose,
)


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def transform_matrix(transform_msg):
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    rotation = quaternion_matrix_xyzw((q.x, q.y, q.z, q.w))
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def _pose(center, orientation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = map(float, orientation)
    return pose


def box_collision_object(object_id, frame_id, stamp, center, size, orientation,
                         operation=CollisionObject.ADD):
    item = CollisionObject()
    item.id = object_id
    item.header.frame_id = frame_id
    item.header.stamp = stamp
    item.operation = operation
    if operation != CollisionObject.REMOVE:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(map(float, size))
        item.primitives = [primitive]
        item.primitive_poses = [_pose(center, orientation)]
    return item


class SceneUpdater(Node):
    def __init__(self):
        super().__init__("so101_scene_updater")
        defaults = {
            "detections_topic": "/so101/objects",
            "status_topic": "/so101_scene/status",
            "world_frame": "base_link",
            "board_frame": "charuco_board",
            "apply_service": "/follower/apply_planning_scene",
            "get_scene_service": "/follower/get_planning_scene",
            "attachment_request_topic": "/so101_scene/attachment_request",
            "attachment_status_topic": "/so101_scene/attachment_status",
            "max_detection_age_s": 0.5,
            "track_ttl_s": 0.75,
            "max_objects": 24,
            "min_object_extent_m": 0.002,
            "max_object_extent_m": 0.30,
            "object_padding_m": 0.006,
            "frames_qualified": False,
            "board_collision_enabled": True,
            "board_size_m": [0.175, 0.125, 0.004],
            "board_center_m": [0.0875, -0.0625, -0.002],
            "table_collision_enabled": False,
            "table_size_xy_m": [0.60, 0.50],
            "table_center_xy_m": [0.0875, -0.0625],
            "table_surface_z_m": -0.004,
            "table_thickness_m": 0.012,
            "base_keepout_size_xy_m": [0.22, 0.22],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.world_frame = str(get("world_frame"))
        self.board_frame = str(get("board_frame"))
        self.max_age_s = float(get("max_detection_age_s"))
        self.max_objects = int(get("max_objects"))
        self.frames_qualified = bool(get("frames_qualified"))
        self.registry = LiveObjectRegistry(
            ttl_s=float(get("track_ttl_s")),
            min_extent_m=float(get("min_object_extent_m")),
            max_extent_m=float(get("max_object_extent_m")),
            padding_m=float(get("object_padding_m")),
            max_objects=self.max_objects)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_client = self.create_client(ApplyPlanningScene, str(get("apply_service")))
        self.scene_read_client = self.create_client(GetPlanningScene, str(get("get_scene_service")))
        self.status_pub = self.create_publisher(String, str(get("status_topic")), 10)
        self.attachment_status_pub = self.create_publisher(
            String, str(get("attachment_status_topic")), 10)
        self._scene_busy = False
        self._static_sent = False
        self._static_pending = []
        self._pending_by_id = {}
        self._attachment_queue = []
        self._attachment_results = {}
        self._attachment_inflight = set()
        self._table_cfg = {
            "enabled": bool(get("table_collision_enabled")),
            "size_xy": tuple(float(v) for v in get("table_size_xy_m")),
            "center_xy": tuple(float(v) for v in get("table_center_xy_m")),
            "surface_z": float(get("table_surface_z_m")),
            "thickness": float(get("table_thickness_m")),
            "keepout_xy": tuple(float(v) for v in get("base_keepout_size_xy_m")),
        }
        self._board_cfg = {
            "enabled": bool(get("board_collision_enabled")),
            "size": tuple(float(v) for v in get("board_size_m")),
            "center": tuple(float(v) for v in get("board_center_m")),
        }
        self.create_subscription(Detection3DArray, str(get("detections_topic")),
                                 self._detections, 10)
        self.create_subscription(String, str(get("attachment_request_topic")),
                                 self._attachment_request, 10)
        self.create_timer(0.2, self._maintenance)
        self.get_logger().info(
            f"Scene updates are bounded to {self.max_objects} tracked boxes; "
            f"only colorbrick_* world objects are owned by this node")

    def _status(self, state, **fields):
        msg = String()
        msg.data = json.dumps({"state": state, **fields}, sort_keys=True)
        self.status_pub.publish(msg)

    def _lookup(self, target, source, stamp):
        return self.tf_buffer.lookup_transform(target, source, stamp)

    def _detections(self, array):
        if not array.header.frame_id:
            self._status("rejected", reason="Detection3DArray has no frame_id")
            return
        stamp_s = stamp_seconds(array.header.stamp)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(now_s=now_s, stamp_s=stamp_s, max_age_s=self.max_age_s):
            self._status("stale", stamp=stamp_s, reason="detection array timestamp is stale")
            return
        try:
            from rclpy.time import Time
            stamp = Time.from_msg(array.header.stamp)
            transform = transform_matrix(self._lookup(
                self.world_frame, array.header.frame_id, stamp))
        except (TransformException, ValueError) as exc:
            self._status("rejected", stamp=stamp_s,
                         reason=f"no transform at detection stamp: {exc}")
            return

        changed = []
        dropped = 0
        now_mono = time.monotonic()
        for detection in list(array.detections)[:self.max_objects]:
            try:
                if self.registry.is_attached(detection.id):
                    continue
                center, orientation = transform_box_pose(
                    transform,
                    (detection.bbox.center.position.x,
                     detection.bbox.center.position.y,
                     detection.bbox.center.position.z),
                    (detection.bbox.center.orientation.x,
                     detection.bbox.center.orientation.y,
                     detection.bbox.center.orientation.z,
                     detection.bbox.center.orientation.w))
                tracked = self.registry.update(
                    detection.id, center,
                    (detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z),
                    orientation, now_mono_s=now_mono)
                changed.append(box_collision_object(
                    tracked.collision_id, self.world_frame, array.header.stamp,
                    tracked.center_m, tracked.size_m, tracked.orientation_xyzw))
            except (ValueError, TypeError) as exc:
                dropped += 1
                self.get_logger().warning(f"Ignoring untracked/invalid detection: {exc}")
        if changed:
            self._apply(changed)
        self._status("updated" if changed else "empty", stamp=stamp_s,
                     accepted=len(changed), rejected=dropped,
                     dimensions="provisional; padded boxes",
                     frames_qualified=self.frames_qualified)

    def _static_geometry(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        # Require the board-to-planning transform at current ROS time before
        # inserting geometry whose source frame is the measured board.
        self._lookup(self.world_frame, self.board_frame, now)
        boxes = []
        if self._board_cfg["enabled"]:
            boxes.append(box_collision_object(
                "station_charuco_board", self.board_frame, stamp,
                self._board_cfg["center"], self._board_cfg["size"], (0, 0, 0, 1)))
        if self._table_cfg["enabled"]:
            # The table top sits beneath the board's z=0 top surface. Split its
            # finite ROI around the live robot pedestal so it cannot fill the
            # arm base or erase other planning-scene objects.
            board_from_base = transform_matrix(self._lookup(
                self.board_frame, self.world_frame, now))
            base_xy = tuple(float(v) for v in board_from_base[:2, 3])
            patches = table_patches_around_keepout(
                self._table_cfg["center_xy"], self._table_cfg["size_xy"],
                base_xy, self._table_cfg["keepout_xy"])
            thickness = self._table_cfg["thickness"]
            if not math.isfinite(thickness) or thickness <= 0:
                raise ValueError("table thickness must be positive")
            for index, (x, y, width, height) in enumerate(patches):
                if min(width, height) <= 0:
                    continue
                boxes.append(box_collision_object(
                    f"station_table_patch_{index}", self.board_frame, stamp,
                    (x, y, self._table_cfg["surface_z"] - thickness / 2),
                    (width, height, thickness), (0, 0, 0, 1)))
        return boxes

    def _maintenance(self):
        if self._attachment_queue and not self._scene_busy and not self._pending_by_id:
            self._apply_attachment(self._attachment_queue.pop(0))
        if self._pending_by_id and not self._scene_busy:
            self._apply([])
        if not self._static_sent and not self._scene_busy:
            try:
                self._static_pending = self._static_geometry()
            except (TransformException, ValueError) as exc:
                self._status("waiting_for_board_tf", reason=str(exc))
            else:
                if not self._static_pending:
                    self._static_sent = True
                else:
                    self._apply(self._static_pending, static=True)
        expired = self.registry.expire(now_mono_s=time.monotonic())
        if expired:
            remove = [box_collision_object(
                key, self.world_frame, self.get_clock().now().to_msg(),
                (0, 0, 0), (0.01, 0.01, 0.01), (0, 0, 0, 1),
                operation=CollisionObject.REMOVE) for key in expired]
            self._apply(remove)
            self._status("expired", ids=expired)

    def _attachment_request(self, message):
        try:
            data = json.loads(message.data)
            if (not isinstance(data, dict) or data.get("schema") != 1 or
                    not isinstance(data.get("request_id"), str) or not data["request_id"] or
                    data.get("action") not in {"attach", "detach", "allow_contact", "disallow_contact"} or
                    not isinstance(data.get("track_id"), str) or not data["track_id"]):
                raise ValueError("request requires schema 1, request_id, action and track_id")
            request_id = data["request_id"]
            if request_id in self._attachment_results:
                self._attachment_status_pub.publish(String(data=json.dumps(
                    self._attachment_results[request_id], sort_keys=True)))
                return
            if request_id in self._attachment_inflight:
                return
            self._attachment_inflight.add(request_id)
            self._attachment_queue.append(data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._attachment_status({"state": "rejected", "reason": str(exc)})

    def _attachment_status(self, data):
        request_id = data.get("request_id")
        if request_id:
            self._attachment_inflight.discard(request_id)
            self._attachment_results[request_id] = dict(data)
            if len(self._attachment_results) > 128:
                self._attachment_results.pop(next(iter(self._attachment_results)))
        message = String()
        message.data = json.dumps(data, sort_keys=True, allow_nan=False)
        self.attachment_status_pub.publish(message)

    def _apply_attachment(self, data):
        request_id, action, track_id = data["request_id"], data["action"], data["track_id"]
        try:
            key = object_collision_id(track_id)
            if action in {"allow_contact", "disallow_contact"}:
                if key not in self.registry._objects:
                    raise ValueError("contact target is not present in the live scene registry")
                links = data.get("touch_links")
                if not isinstance(links, list) or not links or any(
                        not isinstance(link, str) or not link.strip() for link in links):
                    raise ValueError("contact matrix update requires explicit touch_links")
                self._update_contact_matrix(data, key, links, action == "allow_contact")
                return
            item = self.registry._objects[key]
            if action == "attach":
                link_from_world = transform_matrix(
                    self._lookup(data["link_name"], self.world_frame,
                                 self.get_clock().now()))
                link_center, link_orientation = transform_box_pose(
                    link_from_world, item.center_m, item.orientation_xyzw)
                self.registry.attach(track_id, link_name=data.get("link_name"),
                                     touch_links=data.get("touch_links"),
                                     link_pose={"position": link_center,
                                                "orientation_xyzw": link_orientation})
                record = self.registry.attached_record(track_id)
            else:
                record = self.registry.attached_record(track_id)
                world = data["world_pose"]
                center = tuple(float(v) for v in world["position"])
                orientation = tuple(float(v) for v in world["orientation_xyzw"])
                if len(center) != 3 or len(orientation) != 4 or not all(
                        math.isfinite(v) for v in (*center, *orientation)):
                    raise ValueError("world_pose must contain finite XYZ and XYZW")
                quaternion_matrix_xyzw(orientation)
            request = ApplyPlanningScene.Request()
            request.scene = PlanningScene()
            request.scene.is_diff = True
            request.scene.robot_state.is_diff = True
            stamp = self.get_clock().now().to_msg()
            if action == "attach":
                request.scene.world.collision_objects = [box_collision_object(
                    key, self.world_frame, stamp, (0, 0, 0), item.size_m, (0, 0, 0, 1),
                    operation=CollisionObject.REMOVE)]
                attached = AttachedCollisionObject()
                attached.link_name = record["link_name"]
                attached.touch_links = list(record["touch_links"])
                attached.object = box_collision_object(
                    key, record["link_name"], stamp, record["link_pose"]["position"],
                    item.size_m, record["link_pose"]["orientation_xyzw"])
                request.scene.robot_state.attached_collision_objects = [attached]
            else:
                attached = AttachedCollisionObject()
                attached.link_name = record["link_name"]
                attached.object.id = key
                attached.object.operation = CollisionObject.REMOVE
                request.scene.robot_state.attached_collision_objects = [attached]
                request.scene.world.collision_objects = [box_collision_object(
                    key, self.world_frame, stamp, center, item.size_m, orientation)]
        except (KeyError, TypeError, ValueError, TransformException, np.linalg.LinAlgError) as exc:
            self._attachment_status({"request_id": request_id, "state": "rejected",
                                     "track_id": track_id, "reason": str(exc)})
            return
        if not self.scene_client.service_is_ready():
            if action == "attach":
                self.registry.detach(track_id)
            self._attachment_status({"request_id": request_id, "state": "failed",
                                     "track_id": track_id, "reason": "apply service unavailable"})
            return
        self._scene_busy = True
        future = self.scene_client.call_async(request)

        def done(result):
            self._scene_busy = False
            try:
                response = result.result()
                applied = bool(response and response.success)
                reason = "planning-scene service rejected diff"
            except Exception as exc:
                applied, reason = False, str(exc)
            if applied and action == "detach":
                self.registry.detach(track_id, world_pose=data["world_pose"],
                                     now_mono_s=time.monotonic())
            elif not applied and action == "attach":
                self.registry.detach(track_id)
            self._attachment_status({"request_id": request_id,
                                     "state": ("attached" if action == "attach" else "detached")
                                     if applied else "failed", "track_id": track_id,
                                     "collision_id": key,
                                     **({} if applied else {"reason": reason})})

        future.add_done_callback(done)

    def _update_contact_matrix(self, data, collision_id, links, allow):
        if not self.scene_read_client.service_is_ready() or not self.scene_client.service_is_ready():
            self._attachment_status({"request_id": data["request_id"], "state": "failed",
                                     "reason": "planning-scene read/apply service unavailable"})
            return
        read = GetPlanningScene.Request()
        read.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self._scene_busy = True
        read_future = self.scene_read_client.call_async(read)

        def got_scene(done_read):
            try:
                existing = done_read.result().scene.allowed_collision_matrix
                matrix = copy.deepcopy(existing)
                names = list(matrix.entry_names)
                for link in [collision_id, *links]:
                    if link not in names:
                        names.append(link)
                old_n = len(matrix.entry_names)
                for row in matrix.entry_values:
                    row.enabled.extend([False] * (len(names) - old_n))
                from moveit_msgs.msg import AllowedCollisionEntry
                for _ in range(len(names) - len(matrix.entry_values)):
                    matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(names)))
                for row in matrix.entry_values:
                    if len(row.enabled) < len(names):
                        row.enabled.extend([False] * (len(names) - len(row.enabled)))
                matrix.entry_names = names
                for link in links:
                    i, j = names.index(collision_id), names.index(link)
                    matrix.entry_values[i].enabled[j] = bool(allow)
                    matrix.entry_values[j].enabled[i] = bool(allow)
                apply = ApplyPlanningScene.Request()
                apply.scene = PlanningScene()
                apply.scene.is_diff = True
                apply.scene.robot_state.is_diff = True
                apply.scene.allowed_collision_matrix = matrix
                apply_future = self.scene_client.call_async(apply)
                apply_future.add_done_callback(lambda done_apply: self._contact_done(
                    done_apply, data, allow))
            except Exception as exc:
                self._scene_busy = False
                self._attachment_status({"request_id": data["request_id"], "state": "failed",
                                         "reason": str(exc)})

        read_future.add_done_callback(got_scene)

    def _contact_done(self, future, data, allow):
        self._scene_busy = False
        try:
            response = future.result()
            applied = bool(response and response.success)
            reason = "planning-scene rejected allowed-collision update"
        except Exception as exc:
            applied, reason = False, str(exc)
        self._attachment_status({"request_id": data["request_id"],
                                 "state": ("contact_allowed" if allow else "contact_disallowed")
                                 if applied else "failed",
                                 "track_id": data["track_id"],
                                 **({} if applied else {"reason": reason})})

    def _apply(self, objects, *, static=False):
        for item in objects:
            self._pending_by_id[item.id] = item
        if self._scene_busy or not self._pending_by_id or not self.scene_client.service_is_ready():
            return
        queued = list(self._pending_by_id.values())
        self._pending_by_id.clear()
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = queued
        self._scene_busy = True
        future = self.scene_client.call_async(request)

        def done(result):
            self._scene_busy = False
            try:
                response = result.result()
                applied = bool(response and response.success)
            except Exception as exc:  # ROS service errors become status, not a node crash.
                self._status("apply_failed", reason=str(exc))
                return
            if static and applied:
                self._static_sent = True
                self._static_pending = []
            if not applied:
                # Requeue the exact diff; subsequent updates for the same ID
                # replace it, while unrelated scene objects remain untouched.
                for item in queued:
                    self._pending_by_id.setdefault(item.id, item)
            self._status("applied" if applied else "apply_failed",
                         count=len(queued), static=static)
            if applied and self._pending_by_id:
                self._apply([])

        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = SceneUpdater()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

