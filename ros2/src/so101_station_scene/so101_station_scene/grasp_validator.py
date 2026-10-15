"""Check geometric pregrasp candidates with MoveIt IK/state validity only."""

import json
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .scene_geometry import stamp_is_fresh
from .scene_updater import stamp_seconds


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
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.world_frame = str(get("world_frame"))
        self.max_age_s = float(get("max_age_s"))
        self.joint_age_s = float(get("joint_state_max_age_s"))
        self.max_candidates = int(get("max_candidates"))
        self.min_period = 1.0 / min(1.0, max(0.1, float(get("max_processing_hz"))))
        self.group_name = str(get("planning_group"))
        self.ik_link = str(get("ik_link"))
        self.last_started_mono = 0.0
        self.latest_joints = None
        self._pending_batch = None
        self._active_batch = None
        self._busy = False
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
            self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException as exc:
            self._publish({"state": "missing_candidate_tf", "stamp": stamp_s,
                           "reason": str(exc), "geometric_only": True,
                           "ik_validated": False, "execution_enabled": False})
            return
        poses = list(msg.poses[:self.max_candidates])
        if not poses:
            self._publish({"state": "empty", "stamp": stamp_s,
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            return
        self._pending_batch = (msg.header, poses, 0, time.monotonic())

    def _tick(self):
        if self._busy or time.monotonic() - self.last_started_mono < self.min_period:
            return
        if self._active_batch is None:
            self._active_batch = self._pending_batch
            self._pending_batch = None
        if self._active_batch is None:
            return
        header, poses, index, received_mono = self._active_batch
        if time.monotonic() - received_mono > self.max_age_s:
            self._publish({"state": "stale_before_ik", "stamp": stamp_seconds(header.stamp),
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            self._active_batch = None
            return
        if index >= len(poses):
            self._active_batch = self._pending_batch
            self._pending_batch = None
            return
        if self.latest_joints is None:
            self._publish({"state": "waiting_for_fresh_joint_state",
                           "stamp": stamp_seconds(header.stamp),
                           "geometric_only": True, "ik_validated": False,
                           "execution_enabled": False})
            self._active_batch = None
            return
        joint_stamp_s = stamp_seconds(self.latest_joints.header.stamp)
        now_s = self.get_clock().now().nanoseconds * 1e-9
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

        pose = poses[index]
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
        self.last_started_mono = time.monotonic()
        header_stamp = stamp_seconds(header.stamp)
        self.ik_client.call_async(request).add_done_callback(
            lambda future: self._ik_done(future, header_stamp, index, pose))

    def _ik_done(self, future, stamp_s, index, pose):
        try:
            response = future.result()
        except Exception as exc:
            self._finish_candidate(stamp_s, index, pose, "ik_service_error", str(exc))
            return
        if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
            code = "no_response" if response is None else str(response.error_code.val)
            self._finish_candidate(stamp_s, index, pose, "ik_failed", code)
            return
        validity = GetStateValidity.Request()
        validity.robot_state = response.solution
        validity.group_name = self.group_name
        self.validity_client.call_async(validity).add_done_callback(
            lambda result: self._validity_done(result, stamp_s, index, pose))

    def _validity_done(self, future, stamp_s, index, pose):
        try:
            response = future.result()
            valid = bool(response and response.valid)
            reason = "valid" if valid else "state is in collision or invalid"
            state = "pregrasp_ik_collision_valid" if valid else "pregrasp_collision_invalid"
        except Exception as exc:
            valid, reason, state = False, str(exc), "state_validity_service_error"
        self._finish_candidate(stamp_s, index, pose, state, reason, ik_validated=True,
                               collision_valid=valid)

    def _finish_candidate(self, stamp_s, index, pose, state, reason,
                          *, ik_validated=False, collision_valid=False):
        self._publish({
            "state": state,
            "reason": reason,
            "candidate_index": index,
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
        if self._active_batch is not None:
            header, poses, _, received_mono = self._active_batch
            self._active_batch = (header, poses, index + 1, received_mono)


def main(args=None):
    rclpy.init(args=args)
    node = GraspValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

