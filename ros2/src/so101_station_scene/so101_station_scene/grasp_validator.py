"""Check geometric pregrasp candidates with MoveIt IK/state validity only."""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .scene_geometry import (matrix_quaternion_xyzw, quaternion_matrix_xyzw,
                             stamp_is_fresh, transform_box_pose)
from .scene_updater import stamp_seconds


def base_vertical_variant(target_from_source, position, orientation_xyzw):
    """Transform a grasp position, then make tool +Z point down in base_link.

    The source candidate's closing axis is projected into the base horizontal
    plane. This changes orientation only; it preserves the transformed XYZ.
    Returns (position, quaternion_xyzw, approach_error_deg, closing_error_deg).
    """
    position, source_orientation = transform_box_pose(
        target_from_source, position, orientation_xyzw)
    target_rotation = quaternion_matrix_xyzw(source_orientation)
    desired_approach = target_rotation[:, 2]
    desired_closing = target_rotation[:, 0]
    down = (0.0, 0.0, -1.0)
    closing = desired_closing.copy()
    closing[2] = 0.0
    norm = float((closing @ closing) ** 0.5)
    if norm < 1e-8:
        raise ValueError("grasp closing axis is parallel to base vertical")
    closing /= norm
    lateral = np.cross(down, closing)
    lateral /= np.linalg.norm(lateral)
    closing = np.cross(lateral, down)
    rotation = np.column_stack((closing, lateral, down))
    quaternion = matrix_quaternion_xyzw(rotation)

    def angle_degrees(a, b):
        cosine = max(-1.0, min(1.0, float(a @ b)))
        return math.degrees(math.acos(cosine))

    return (position, quaternion, angle_degrees(desired_approach, down),
            angle_degrees(desired_closing, closing))


def should_wait_for_batch(*, active, pending, now, last_batch_start, min_period):
    """Rate-limit batch starts while allowing candidates within a batch through."""
    return (not active and pending is not None
            and now - last_batch_start < min_period)


def async_request_expired(*, now, deadline):
    return now >= deadline


def async_result_is_current(*, token, active_token, now_mono, deadline_mono,
                            received_mono, max_age_s, now_ros, source_stamp_s,
                            joint_stamp_s, joint_max_age_s):
    """Reject late callbacks and any result based on stale source/state data."""
    if token != active_token or async_request_expired(now=now_mono, deadline=deadline_mono):
        return False
    if now_mono < received_mono or now_mono - received_mono > max_age_s:
        return False
    return (stamp_is_fresh(now_s=now_ros, stamp_s=source_stamp_s,
                           max_age_s=max_age_s)
            and stamp_is_fresh(now_s=now_ros, stamp_s=joint_stamp_s,
                               max_age_s=joint_max_age_s))


class GraspValidator(Node):
    """Validate isolated pregrasp IK states; never plan or execute motion."""

    def __init__(self):
        super().__init__("so101_grasp_validator")
        defaults = {
            "candidates_topic": "/so101/grasp_candidates",
            "joint_states_topic": "/follower/joint_states",
            "validation_topic": "/so101/grasp_validation",
            "world_frame": "base_link",
            "planning_group": "manipulator",
            "ik_link": "gripper_frame_link",
            "ik_service": "/follower/compute_ik",
            "state_validity_service": "/follower/check_state_validity",
            "max_age_s": 0.5,
            "joint_state_max_age_s": 0.5,
            "max_candidates": 8,
            "max_processing_hz": 1.0,
            "max_async_s": 0.45,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.world_frame = str(get("world_frame"))
        self.max_age_s = float(get("max_age_s"))
        self.max_async_s = float(get("max_async_s"))
        self.joint_age_s = float(get("joint_state_max_age_s"))
        self.max_candidates = int(get("max_candidates"))
        self.min_period = 1.0 / min(1.0, max(0.1, float(get("max_processing_hz"))))
        self.group_name = str(get("planning_group"))
        self.ik_link = str(get("ik_link"))
        self.last_batch_started_mono = 0.0
        self.latest_joints = None
        self._pending_batch = None
        self._active_batch = None
        self._busy = False
        self._request_sequence = 0
        self._inflight_token = None
        self._inflight_deadline_mono = 0.0
        self._inflight_candidate = None
        self._inflight_future = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, str(get("ik_service")))
        self.validity_client = self.create_client(
            GetStateValidity, str(get("state_validity_service")))
        self.output = self.create_publisher(String, str(get("validation_topic")), 10)
        self.create_subscription(JointState, str(get("joint_states_topic")),
                                 self._joint_state, 10)
        self.create_subscription(PoseArray, str(get("candidates_topic")),
                                 self._candidates, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "Validating pregrasp IK and instantaneous state validity only; "
            "trajectory planning/execution is not enabled")

    def _publish(self, payload):
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.output.publish(msg)

    def _joint_state(self, msg):
        stamp_s = stamp_seconds(msg.header.stamp)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(now_s=now_s, stamp_s=stamp_s,
                              max_age_s=self.joint_age_s):
            return
        self.latest_joints = msg

    def _candidates(self, msg):
        stamp_s = stamp_seconds(msg.header.stamp)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if not msg.header.frame_id or not stamp_is_fresh(
                now_s=now_s, stamp_s=stamp_s, max_age_s=self.max_age_s):
            self._publish({"state": "stale_or_unframed", "stamp": stamp_s,
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            return
        try:
            transform_msg = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as exc:
            self._publish({"state": "missing_candidate_tf", "stamp": stamp_s,
                           "reason": str(exc), "geometric_only": True,
                           "ik_validated": False, "execution_enabled": False})
            return
        matrix = np.eye(4)
        transform = transform_msg.transform
        matrix[:3, :3] = quaternion_matrix_xyzw((
            transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w))
        matrix[:3, 3] = (transform.translation.x, transform.translation.y,
                         transform.translation.z)
        candidates = []
        for index, source_pose in enumerate(msg.poses[:self.max_candidates]):
            try:
                position, orientation, approach_error, closing_error = base_vertical_variant(
                    matrix,
                    (source_pose.position.x, source_pose.position.y,
                     source_pose.position.z),
                    (source_pose.orientation.x, source_pose.orientation.y,
                     source_pose.orientation.z, source_pose.orientation.w))
            except ValueError as exc:
                self._publish({"state": "invalid_candidate_orientation", "stamp": stamp_s,
                               "candidate_index": index, "reason": str(exc),
                               "geometric_only": True, "ik_validated": False,
                               "execution_enabled": False})
                continue
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = position
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = orientation
            candidates.append({"source_index": index, "pose": pose,
                               "approach_error_deg": approach_error,
                               "closing_error_deg": closing_error})
        if not candidates:
            self._publish({"state": "empty", "stamp": stamp_s,
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            return
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.world_frame
        self._pending_batch = (header, candidates, 0, time.monotonic())

    def _tick(self):
        if self._busy:
            if async_request_expired(now=time.monotonic(),
                                     deadline=self._inflight_deadline_mono):
                if self._inflight_future is not None:
                    self._inflight_future.cancel()
                self._finish_candidate(
                    self._inflight_candidate["stamp_s"], self._inflight_candidate,
                    "validation_timeout", "IK/state-validity response exceeded freshness deadline")
            return
        if self._active_batch is None:
            if should_wait_for_batch(active=False, pending=self._pending_batch,
                                     now=time.monotonic(),
                                     last_batch_start=self.last_batch_started_mono,
                                     min_period=self.min_period):
                return
            self._active_batch = self._pending_batch
            self._pending_batch = None
            if self._active_batch is not None:
                self.last_batch_started_mono = time.monotonic()
        if self._active_batch is None:
            return
        header, poses, index, received_mono = self._active_batch
        now_mono = time.monotonic()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if (now_mono - received_mono > self.max_age_s
                or not stamp_is_fresh(now_s=now_s,
                                      stamp_s=stamp_seconds(header.stamp),
                                      max_age_s=self.max_age_s)):
            self._publish({"state": "stale_before_ik", "stamp": stamp_seconds(header.stamp),
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            self._active_batch = None
            return
        if index >= len(poses):
            self._active_batch = None
            return
        if self.latest_joints is None:
            self._publish({"state": "waiting_for_fresh_joint_state",
                           "stamp": stamp_seconds(header.stamp),
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            self._active_batch = None
            return
        joint_stamp_s = stamp_seconds(self.latest_joints.header.stamp)
        if not stamp_is_fresh(now_s=now_s, stamp_s=joint_stamp_s,
                              max_age_s=self.joint_age_s):
            self.latest_joints = None
            self._publish({"state": "waiting_for_fresh_joint_state",
                           "stamp": stamp_seconds(header.stamp),
                           "reason": "latest joint state became stale before IK",
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            self._active_batch = None
            return
        if not self.ik_client.service_is_ready() or not self.validity_client.service_is_ready():
            return

        candidate = poses[index]
        pose = candidate["pose"]
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.robot_state.joint_state = self.latest_joints
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=0.35).to_msg()
        self._busy = True
        header_stamp = stamp_seconds(header.stamp)
        self._request_sequence += 1
        token = self._request_sequence
        candidate.update({"stamp_s": header_stamp,
                          "received_mono": received_mono,
                          "joint_stamp_s": joint_stamp_s,
                          "request_token": token})
        self._inflight_candidate = candidate
        self._inflight_token = token
        self._inflight_deadline_mono = min(
            received_mono + self.max_age_s, time.monotonic() + self.max_async_s)
        self._inflight_future = self.ik_client.call_async(request)
        self._inflight_future.add_done_callback(
            lambda future: self._ik_done(future, header_stamp, candidate, token))

    def _async_current(self, candidate, token):
        now_mono = time.monotonic()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        return async_result_is_current(
            token=token, active_token=self._inflight_token,
            now_mono=now_mono, deadline_mono=self._inflight_deadline_mono,
            received_mono=candidate["received_mono"], max_age_s=self.max_age_s,
            now_ros=now_ros, source_stamp_s=candidate["stamp_s"],
            joint_stamp_s=candidate["joint_stamp_s"],
            joint_max_age_s=self.joint_age_s)

    def _ik_done(self, future, stamp_s, candidate, token):
        if token != self._inflight_token:
            return
        if not self._async_current(candidate, token):
            self._finish_candidate(stamp_s, candidate, "stale_async_result",
                                   "candidate or IK joint state became stale before IK response")
            return
        try:
            response = future.result()
        except Exception as exc:
            self._finish_candidate(stamp_s, candidate, "ik_service_error", str(exc))
            return
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = "no_response" if response is None else str(response.error_code.val)
            self._finish_candidate(stamp_s, candidate, "ik_failed", code)
            return
        validity = GetStateValidity.Request()
        validity.robot_state = response.solution
        validity.group_name = self.group_name
        self._inflight_future = self.validity_client.call_async(validity)
        self._inflight_future.add_done_callback(
            lambda result: self._validity_done(result, stamp_s, candidate, token))

    def _validity_done(self, future, stamp_s, candidate, token):
        if token != self._inflight_token:
            return
        if not self._async_current(candidate, token):
            self._finish_candidate(stamp_s, candidate, "stale_async_result",
                                   "candidate or IK joint state became stale before validity response")
            return
        try:
            response = future.result()
            valid = bool(response and response.valid)
            reason = "valid" if valid else "state is in collision or invalid"
            state = "pregrasp_ik_collision_valid" if valid else "pregrasp_collision_invalid"
        except Exception as exc:
            valid, reason, state = False, str(exc), "state_validity_service_error"
        self._finish_candidate(stamp_s, candidate, state, reason,
                               ik_validated=True, collision_valid=valid)

    def _finish_candidate(self, stamp_s, candidate, state, reason,
                          *, ik_validated=False, collision_valid=False):
        index, pose = candidate["source_index"], candidate["pose"]
        self._publish({
            "state": state,
            "reason": reason,
            "candidate_index": index,
            "candidate_variant": "base_vertical_down",
            "requested_approach_tilt_deg": candidate["approach_error_deg"],
            "requested_closing_axis_change_deg": candidate["closing_error_deg"],
            "candidate_kind": "pregrasp",
            "pregrasp_position_m": [float(pose.position.x), float(pose.position.y),
                                    float(pose.position.z)],
            "stamp": stamp_s,
            "geometric_only": True,
            "ik_validated": bool(ik_validated),
            "collision_valid": bool(collision_valid),
            "execution_enabled": False,
            "path_validated": False,
        })
        self._busy = False
        self._inflight_token = None
        self._inflight_future = None
        if self._active_batch is not None:
            header, poses, _, received_mono = self._active_batch
            next_index = self._active_batch[2] + 1
            self._active_batch = (header, poses, next_index, received_mono)


def main(args=None):
    rclpy.init(args=args)
    node = GraspValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

