"""Continuous provisional marker alignment; never writes canonical control TF."""
from copy import deepcopy
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from realsense2_camera_msgs.srv import DeviceInfo
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from so101_station_perception_tag_tracking.tag_alignment import (
    SeededTagAssociator, fit_tag_alignment, point_residual, validate_bindings,
)

PREFIX = "live_aligned/"


def transform_matrix(message):
    translation, quaternion = message.transform.translation, message.transform.rotation
    values = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values)-1.) > .001:
        raise ValueError("invalid TF quaternion")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values).as_matrix()
    result[:3, 3] = [translation.x, translation.y, translation.z]
    if not np.isfinite(result).all():
        raise ValueError("nonfinite TF")
    return result


def transform_message(matrix, parent, child, stamp):
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = parent, child
    message.header.stamp = stamp
    xyz = np.asarray(matrix)[:3, 3]
    quat = Rotation.from_matrix(np.asarray(matrix)[:3, :3]).as_quat()
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = map(float, xyz)
    (message.transform.rotation.x, message.transform.rotation.y,
     message.transform.rotation.z, message.transform.rotation.w) = map(float, quat)
    return message


def visual_description(description):
    """Prefix URDF identities and remove collision/inertia from visual-only copy."""
    root = ET.fromstring(description)
    links = {link.attrib["name"] for link in root.findall("link")}
    children = set()
    edges = []
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent not in links or child not in links:
            raise ValueError("URDF references unknown link")
        edges.append((parent, child))
        children.add(child)
    roots = links-children
    if roots != {"base_link"}:
        raise ValueError("expected unique base_link URDF root")
    for link in root.findall("link"):
        link.attrib["name"] = PREFIX+link.attrib["name"]
        for tag in ("collision", "inertial"):
            for element in list(link.findall(tag)):
                link.remove(element)
    for joint in root.findall("joint"):
        joint.attrib["name"] = PREFIX+joint.attrib["name"]
        for name in ("parent", "child"):
            element = joint.find(name)
            element.attrib["link"] = PREFIX+element.attrib["link"]
        mimic = joint.find("mimic")
        if mimic is not None:
            mimic.attrib["joint"] = PREFIX+mimic.attrib["joint"]
    root.attrib["name"] = "live_aligned_visual_only"
    return ET.tostring(root, encoding="unicode"), edges, links


class LiveTagAlignment(Node):
    def __init__(self):
        super().__init__("so101_live_tag_alignment")
        self.declare_parameter("bindings_file", "")
        self.declare_parameter("max_age_s", .5)
        self.declare_parameter("device_info_service", "/camera/device_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.max_age = float(self.get_parameter("max_age_s").value)
        if not math.isfinite(self.max_age) or not 0 < self.max_age <= 1.:
            raise ValueError("max_age_s must be in (0,1]")
        path = Path(str(self.get_parameter("bindings_file").value)).expanduser()
        raw = path.read_bytes()
        if len(raw) > 65536:
            raise ValueError("binding configuration exceeds 64 KiB")
        self.config = validate_bindings(json.loads(raw))
        if len(self.config["bindings"]) > 16:
            raise ValueError("at most 16 compact marker bindings are supported")
        self.config_sha = hashlib.sha256(raw).hexdigest()
        self.reference = np.asarray(self.config["camera_from_base_reference"], dtype=float)
        self.associator = SeededTagAssociator(self.config)
        self.tf = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.status = self.create_publisher(String, "/so101_perception/live_tag_alignment/status", 10)
        self.candidate = self.create_publisher(TransformStamped, "/so101_perception/live_tag_alignment/candidate", 10)
        self.markers = self.create_publisher(MarkerArray, "/so101_perception/live_tag_alignment/markers", 10)
        self.description = self.create_publisher(String, "/so101_perception/aligned_robot_description", latch)
        self.create_subscription(String, "/follower/robot_description", self._model, latch)
        self.create_subscription(String, "/so101_perception/tag_detections", self._detections, 10)
        self.create_service(Trigger, "/so101_perception/live_tag_alignment/reseed", self._reseed)
        self.device = self.create_client(DeviceInfo, self.get_parameter("device_info_service").value)
        self.rpc = None
        self.next_identity_request = 0.
        self.identity_until = 0.
        self.identity_reason = "waiting for independent camera identity"
        self.model_ok = False
        self.aligned_description = None
        self.edges, self.links = [], set()
        self.previous = None
        self.last_input = None
        self.last_stamp = None
        self.stream_latched = False
        self.source_fingerprint = None
        self.source_context = None
        self.visual_visible = False
        self.latest_status = None
        self.create_timer(.1, self._watchdog)
        self.create_timer(1., self._heartbeat)
        self.create_timer(.1, self._identity)
        self._unavailable("awaiting model, camera identity and seeded detections")

    def _model(self, message):
        digest = hashlib.sha256(message.data.encode()).hexdigest()
        try:
            if digest != self.config["model_sha256"]:
                raise ValueError("robot_description SHA differs from binding model")
            aligned, edges, links = visual_description(message.data)
            if any(binding["link"] not in links for binding in self.config["bindings"]):
                raise ValueError("binding references unknown model link")
            self.aligned_description, self.edges, self.links = aligned, edges, links
            self.model_ok = True
        except (ValueError, KeyError, ET.ParseError) as error:
            self.model_ok = False
            if self.source_fingerprint is not None:
                self.stream_latched = True
            self._unavailable(str(error))

    def _identity(self):
        now = time.monotonic()
        if self.rpc is not None:
            future, requested = self.rpc
            if future.done():
                self.rpc = None
                try:
                    response = future.result()
                    if now-requested > .5 or response.serial_number != self.config["camera_serial"]:
                        raise ValueError("camera DeviceInfo identity mismatch or late response")
                    self.identity_until = now+1.5
                    self.identity_reason = None
                except Exception as error:
                    self.identity_until = 0.
                    self.identity_reason = str(error)
                    if self.source_fingerprint is not None:
                        self.stream_latched = True
                    self._unavailable(self.identity_reason)
            elif now-requested > .5:
                future.cancel()
                self.rpc = None
                self.identity_until = 0.
                self.identity_reason = "camera DeviceInfo request expired"
                if self.source_fingerprint is not None:
                    self.stream_latched = True
                self._unavailable(self.identity_reason)
        if self.rpc is None and now >= self.next_identity_request and self.device.service_is_ready():
            self.rpc = (self.device.call_async(DeviceInfo.Request()), now)
            self.next_identity_request = now+1.

    def _reseed(self, request, response):
        self.associator.reset()
        self.previous = None
        self.last_stamp = None
        self.last_input = None
        self.stream_latched = False
        self.source_fingerprint = None
        self._unavailable("explicit reseed; waiting for next exposure")
        response.success, response.message = True, "Original configured pixel seeds restored; qualification remains false"
        return response

    def _publish_status(self, status):
        status.update(schema="so101-live-tag-alignment-v1", qualified=False,
                      bindings_sha256=self.config_sha, model_sha256=self.config["model_sha256"],
                      camera_serial=self.config["camera_serial"],
                      reference_provenance=self.config.get("provenance"),
                      canonical_control_tf_modified=False,
                      source_admission_proof="derived admission checked upstream; compact carries stable identity fields and pin digest, not full provenance",
                      source_context=self.source_context,
                      independent_device_info_identity_current=time.monotonic() <= self.identity_until)
        self.latest_status = status
        self.status.publish(String(data=json.dumps(status, allow_nan=False, separators=(",", ":"))))

    def _unavailable(self, reason):
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.markers.publish(MarkerArray(markers=[clear]))
        if self.visual_visible or self.latest_status is None:
            # A geometry-free link keeps the hidden placeholder valid URDF.
            robot = ET.Element("robot", name="alignment_unavailable")
            ET.SubElement(robot, "link", name=self.get_parameter("camera_frame").value)
            self.description.publish(String(data=ET.tostring(robot, encoding="unicode")))
            self.visual_visible = False
        self._publish_status(dict(status="unavailable", reason=reason, candidate_available=False))

    def _watchdog(self):
        if self.last_input is not None and time.monotonic()-self.last_input > self.max_age:
            self.stream_latched = True  # Only the explicit reseed service may reacquire.
            self._unavailable("tag exposure stream stale; explicit reseed may be required")
        elif time.monotonic() > self.identity_until:
            if self.source_fingerprint is not None:
                self.stream_latched = True
            self._unavailable(self.identity_reason or "independent camera identity lease expired")

    def _heartbeat(self):
        """Repeat availability for late subscribers without renewing an exposure."""
        if self.latest_status is None:
            return
        status = dict(self.latest_status)
        stamp = status.get("stamp")
        age = None if stamp is None else (self.get_clock().now().nanoseconds*1e-9
            - int(stamp["sec"]) - int(stamp["nanosec"])*1e-9)
        if status.get("candidate_available") and (
                age is None or not 0 <= age <= self.max_age
                or self.last_input is None or time.monotonic()-self.last_input > self.max_age
                or time.monotonic() > self.identity_until or not self.model_ok or self.stream_latched):
            self._unavailable("candidate exposure or identity expired; no heartbeat renewal")
            return
        status["exposure_age_s"] = age
        self._publish_status(status)

    def _lookup(self, target, source, stamp):
        if target == source:
            return np.eye(4)
        return transform_matrix(self.tf.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.)))

    def _detections(self, message, message_info):
        started = time.monotonic()
        try:
            if len(message.data.encode()) > 65536:
                raise ValueError("compact detection payload exceeds 64 KiB")
            payload = json.loads(message.data)
            if not isinstance(payload.get("observations"), list) or len(payload["observations"]) > 64:
                raise ValueError("compact detection count exceeds 64")
            if self.stream_latched:
                raise ValueError("source stream continuity lost; explicit reseed required")
            if payload.get("schema") != "so101-tag-detections-v1" or payload.get("qualified") is not False:
                raise ValueError("unexpected detection schema or qualification")
            stamp = payload["stamp"]
            seconds = int(stamp["sec"])+int(stamp["nanosec"])*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-seconds
            if not 0 <= age <= self.max_age:
                raise ValueError("detection exposure is stale or future")
            if self.last_stamp is not None and seconds <= self.last_stamp:
                raise ValueError("repeated or out-of-order detection exposure")
            if self.last_input is not None and started-self.last_input > self.max_age:
                self.stream_latched = True
                raise ValueError("source receipt gap; explicit reseed required")
            self.last_stamp = seconds
            self.last_input = started
            if (payload["image_width"], payload["image_height"]) != (self.config["image_width"], self.config["image_height"]):
                raise ValueError("binding image dimensions differ")
            if not self.model_ok or started > self.identity_until:
                raise ValueError("model or fresh independent camera identity unavailable")
            if self.count_publishers("/so101_perception/tag_detections") != 1:
                raise ValueError("expected exactly one compact detection publisher")
            if self.count_publishers("/so101_perception/aligned_robot_description") != 1:
                raise ValueError("aligned visual model already has another publisher")
            frame = str(payload["frame_id"])
            if frame != self.get_parameter("camera_frame").value or frame.startswith(PREFIX):
                raise ValueError("invalid camera frame")
            admission = payload.get("source_admission_kind")
            if admission != "derived_checked":
                raise ValueError("native_unattested observations cannot produce live alignment")
            context = payload.get("source_context")
            if not isinstance(context, dict) or context.get("camera_serial") != self.config["camera_serial"]:
                raise ValueError("compact admitted source camera serial differs from configuration")
            for key, length in (("consumer_epoch", 32), ("source_pin_sha256", 64),
                                ("producer_generation", 32), ("source_generation", 64)):
                value = context.get(key)
                if type(value) is not str or len(value) != length or any(c not in "0123456789abcdef" for c in value):
                    raise ValueError("invalid compact source identity field: "+key)
            publishers = self.get_publishers_info_by_topic("/so101_perception/tag_detections")
            # Jazzy delivers MessageInfo as a dictionary; newer bindings may
            # expose attributes. Both must supply the actual received GID.
            received_gid = (message_info.get("publisher_gid") if isinstance(message_info, dict)
                            else getattr(message_info, "publisher_gid", None))
            actual_gid = bytes(received_gid).hex()
            if len(publishers) != 1 or actual_gid != bytes(publishers[0].endpoint_gid).hex():
                raise ValueError("compact message publisher GID differs from unique endpoint")
            fingerprint = (frame, admission, actual_gid, tuple(context[key] for key in (
                "consumer_epoch", "source_pin_sha256", "producer_generation", "source_generation", "camera_serial")))
            if self.source_fingerprint is not None and fingerprint != self.source_fingerprint:
                self.stream_latched = True
                raise ValueError("compact source publisher/frame/admission changed; explicit reseed required")
            self.source_fingerprint = fingerprint
            self.source_context = {key: context[key] for key in (
                "consumer_epoch", "source_pin_sha256", "producer_generation", "source_generation", "camera_serial")}
            exposure = Time(seconds=int(stamp["sec"]), nanoseconds=int(stamp["nanosec"]))
            # A timestamped lookup also requires the canonical camera/base chain to exist.
            self._lookup(frame, "base_link", exposure)
            tracked = self.associator.update(payload["observations"], stamp=seconds)
            model_points, camera_points, available = [], [], []
            unavailable_residuals = []
            for observation in tracked:
                if observation["association"] != "tracked" or not observation.get("measurement_valid"):
                    if observation["moving"]:
                        unavailable_residuals.append(dict(instance_name=observation["instance_name"],
                            moving=True, status="unavailable", qualified=False,
                            reason=observation.get("reason", observation["association"])))
                        continue
                    raise ValueError(f"{observation['instance_name']}: {observation.get('reason', observation['association'])}")
                base_from_link = self._lookup("base_link", observation["link"], exposure)
                point = (base_from_link @ np.r_[observation["link_point_m"], 1.])[:3]
                available.append((observation, point))
                if not observation["moving"]:
                    model_points.append(point)
                    camera_points.append(observation["center_camera_m"])
            reference_fit = fit_tag_alignment(model_points, camera_points, previous_camera_from_base=self.reference)
            if reference_fit["status"] != "usable_candidate":
                raise ValueError(reference_fit["reason"])
            fit = fit_tag_alignment(model_points, camera_points, previous_camera_from_base=self.previous if self.previous is not None else self.reference)
            if fit["status"] != "usable_candidate":
                raise ValueError(fit["reason"])
            matrix = np.asarray(fit["camera_from_base"])
            # Resolve the complete visual tree before publishing any candidate/TF.
            transforms = [transform_message(matrix, frame, PREFIX+"base_link", exposure.to_msg())]
            for parent, child in self.edges:
                transforms.append(transform_message(self._lookup(parent, child, exposure), PREFIX+parent, PREFIX+child, exposure.to_msg()))
            residuals = list(unavailable_residuals)
            output = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            output.markers.append(clear)
            for index, (observation, model_point) in enumerate(available):
                residual = point_residual(model_point, matrix, observation["center_camera_m"])
                residual.update(instance_name=observation["instance_name"], moving=observation["moving"], qualified=False)
                residuals.append(residual)
                self._add_markers(output, index, frame, exposure.to_msg(), residual)
            fit.update(candidate_available=True, stamp=stamp, source_admission_kind=payload.get("source_admission_kind"),
                       marker_residuals=residuals, update_ms=(time.monotonic()-started)*1000.)
            final_publishers = self.get_publishers_info_by_topic("/so101_perception/tag_detections")
            if (len(final_publishers) != 1
                    or bytes(final_publishers[0].endpoint_gid).hex() != actual_gid
                    or self.source_fingerprint != fingerprint):
                self.stream_latched = True
                raise ValueError("compact source changed before publication")
            final_age = self.get_clock().now().nanoseconds*1e-9-seconds
            final_now = time.monotonic()
            if (not 0 <= final_age <= self.max_age or final_now-started > self.max_age
                    or final_now > self.identity_until or not self.model_ok or self.stream_latched):
                raise ValueError("original exposure, receipt or identity expired before publication")
            self.candidate.publish(transform_message(matrix, frame, PREFIX+"base_link", exposure.to_msg()))
            if not self.visual_visible:
                self.description.publish(String(data=self.aligned_description))
                self.visual_visible = True
            self.broadcaster.sendTransform(transforms)
            self.markers.publish(output)
            self._publish_status(fit)
            self.previous = matrix
        except Exception as error:
            self._unavailable(str(error))

    def _add_markers(self, output, index, frame, stamp, residual):
        for offset, key, color in ((0, "measured_camera_m", (0., 1., 1.)), (1, "predicted_camera_m", (1., 0., 1.))):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame, stamp
            marker.ns, marker.id = "live_tag_alignment", index*4+offset
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.orientation.w = 1.
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, residual[key])
            marker.scale.x = marker.scale.y = marker.scale.z = .006
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = .95
            marker.lifetime = Duration(seconds=self.max_age).to_msg()
            output.markers.append(marker)
        line = Marker()
        line.header.frame_id, line.header.stamp = frame, stamp
        line.ns, line.id = "live_tag_alignment", index*4+2
        line.type, line.action = Marker.LINE_LIST, Marker.ADD
        line.pose.orientation.w = 1.
        line.scale.x = .001
        line.color.r = line.color.g = line.color.a = 1.
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (residual["measured_camera_m"], residual["predicted_camera_m"])]
        line.lifetime = Duration(seconds=self.max_age).to_msg()
        output.markers.append(line)
        label = Marker()
        label.header, label.pose = line.header, deepcopy(output.markers[-3].pose)
        label.ns, label.id = "live_tag_alignment", index*4+3
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z += .012
        label.scale.z = .012
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.text = f"{residual['instance_name']}: {residual['residual_m']*1000.:.1f} mm (provisional)"
        label.lifetime = Duration(seconds=self.max_age).to_msg()
        output.markers.append(label)


def main(args=None):
    rclpy.init(args=args)
    node = LiveTagAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
