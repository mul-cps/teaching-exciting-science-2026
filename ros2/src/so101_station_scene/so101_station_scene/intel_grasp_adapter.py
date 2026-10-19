"""Optional bounded Intel GraspPlanning bridge; proposals only, never execution."""
from copy import deepcopy
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GraspPlanning
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

UPSTREAM_REVISION = "980b7ddd4348d9aa78c6e84c42da1984746867db"


def stamp_seconds(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        return float("nan")
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(stamp, now, max_age):
    return math.isfinite(stamp) and stamp > 0 and 0 <= now - stamp <= max_age


def bounded_text(value):
    return len(value) <= 128


def checked_posture(posture):
    if (len(posture.joint_names) > 8 or len(posture.points) > 4
            or not bounded_text(posture.header.frame_id)
            or not math.isfinite(stamp_seconds(posture.header.stamp))
            or any(not bounded_text(name) for name in posture.joint_names)):
        raise ValueError("unbounded or malformed backend posture")
    previous = -1.
    for point in posture.points:
        for values in (point.positions, point.velocities, point.accelerations, point.effort):
            if len(values) not in (0, len(posture.joint_names)) or not all(math.isfinite(v) for v in values):
                raise ValueError("unbounded or nonfinite backend posture values")
        duration = stamp_seconds(point.time_from_start)
        if not math.isfinite(duration) or not 0 <= duration <= 10. or duration < previous:
            raise ValueError("invalid backend posture duration")
        previous = duration


def checked_translation(translation, frame, *, optional=False):
    vector = translation.direction.vector
    components = (vector.x, vector.y, vector.z)
    header = translation.direction.header
    minimum, desired = translation.min_distance, translation.desired_distance
    if (optional and header.frame_id == "" and stamp_seconds(header.stamp) == 0
            and components == (0., 0., 0.) and minimum == desired == 0.):
        return
    if (header.frame_id != frame or not math.isfinite(stamp_seconds(header.stamp))
            or not all(math.isfinite(v) for v in (*components, minimum, desired))
            or abs(sum(v*v for v in components) ** .5 - 1.) > 1e-3
            or not 0 <= minimum <= desired <= .25):
        raise ValueError("invalid backend translation frame/direction/distance")


def id_only_request(request, frame, group, now, max_age):
    target = request.target
    pose = target.pose
    default_pose = (pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w) == (0., 0., 0., 0., 0., 0., 1.)
    return (request.group_name == group and bounded_text(group)
            and bool(target.id) and bounded_text(target.id)
            and target.header.frame_id == frame and bounded_text(frame)
            and fresh(stamp_seconds(target.header.stamp), now, max_age)
            and default_pose and not target.type.key and not target.type.db
            and target.operation == target.ADD
            and not any((request.candidate_grasps, request.movable_obstacles,
                         request.support_surfaces, target.primitives, target.primitive_poses,
                         target.meshes, target.mesh_poses, target.planes, target.plane_poses,
                         target.subframe_names, target.subframe_poses)))


def checked_grasps(grasps, *, frame, now, max_age, earliest_stamp, max_candidates):
    """Validate a whole bounded batch; preserve source stamps and upstream EEF poses.

    No TF latest-time substitution, timestamp refresh, posture execution, tool offset
    assumption or score-as-probability conversion is permitted here.
    """
    if not 0 < len(grasps) <= max_candidates:
        raise ValueError("empty or oversized backend grasp batch")
    result = []
    for grasp in grasps:
        if (not bounded_text(grasp.id) or len(grasp.allowed_touch_objects) > 8
                or any(not bounded_text(name) for name in grasp.allowed_touch_objects)
                or not math.isfinite(grasp.max_contact_force)):
            raise ValueError("unbounded or nonfinite backend grasp metadata")
        checked_posture(grasp.pre_grasp_posture)
        checked_posture(grasp.grasp_posture)
        checked_translation(grasp.post_grasp_retreat, frame, optional=True)
        checked_translation(grasp.post_place_retreat, frame, optional=True)
        header = grasp.grasp_pose.header
        stamp = stamp_seconds(header.stamp)
        if header.frame_id != frame or not fresh(stamp, now, max_age) or stamp < earliest_stamp:
            raise ValueError("backend grasp frame/stamp does not match fresh request")
        pose = grasp.grasp_pose.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z,
                  pose.orientation.w, grasp.grasp_quality]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite backend pose or quality")
        norm = sum(v*v for v in values[3:7]) ** .5
        if abs(norm - 1.) > 1e-3:
            raise ValueError("backend quaternion is not unit length")
        checked_translation(grasp.pre_grasp_approach, frame)
        result.append(grasp)
    return result


class IntelGraspAdapter(Node):
    """Isolated proposal service with one bounded asynchronous backend request."""

    def __init__(self, **kwargs):
        super().__init__("intel_grasp_adapter", **kwargs)
        defaults = {"enabled": False, "backend_service": "/intel_grasp/plan_grasps",
                    "service": "/station/intel_grasp/plan",
                    "frame_id": "base_link", "planning_group": "manipulator",
                    "max_age_s": .5, "timeout_s": .4, "max_candidates": 8,
                    "proposal_topic": "/station/intel_grasp/poses",
                    "status_topic": "/station/intel_grasp/status"}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.enabled = bool(get("enabled"))
        self.frame = str(get("frame_id"))
        self.group = str(get("planning_group"))
        self.max_age = float(get("max_age_s"))
        self.timeout = float(get("timeout_s"))
        self.max_candidates = int(get("max_candidates"))
        if (not self.frame or not self.group or not bounded_text(self.frame)
                or not bounded_text(self.group) or not .01 <= self.timeout <= 2.
                or not .01 <= self.max_age <= .5 or not 1 <= self.max_candidates <= 32):
            raise ValueError("invalid adapter frame/group/bounds")
        if str(get("backend_service")) == str(get("service")):
            raise ValueError("backend and adapter services must differ")
        self.pending = None
        self.backend_timeout_latched = False
        callbacks = ReentrantCallbackGroup()
        self.backend = self.create_client(GraspPlanning, str(get("backend_service")),
                                          callback_group=callbacks)
        self.poses = self.create_publisher(PoseArray, str(get("proposal_topic")), 1)
        self.status = self.create_publisher(String, str(get("status_topic")), 10)
        self.service = self.create_service(GraspPlanning, str(get("service")),
                                           self._request, callback_group=callbacks)
        self.timer = (self.create_timer(.01, self._expire, callback_group=callbacks)
                      if self.enabled else None)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _report(self, state, detail="", **extra):
        data = {"state": state, "detail": str(detail)[:512], "qualified": False,
                "execution_enabled": False, "ik_validated": False,
                "collision_checked": False, "tool_mapping_qualified": False,
                "backend": "intel/ros2_grasp_library", "inspected_upstream_revision": UPSTREAM_REVISION,
                "backend_runtime_verified": False,
                "target_association_verified": False,
                "target_id_semantics": "request provenance only; backend does not establish track association",
                "pose_semantics": "upstream_eef_parent", "eef_parent_link_verified": False,
                **extra}
        self.status.publish(String(data=json.dumps(data, sort_keys=True)))

    def _failure(self, code, state, detail=""):
        response = GraspPlanning.Response()
        response.error_code.val = code
        self._report(state, detail)
        return response

    async def _request(self, request, response):
        if not self.enabled:
            return self._failure(MoveItErrorCodes.FAILURE, "disabled")
        stamp = stamp_seconds(request.target.header.stamp)
        if not id_only_request(request, self.frame, self.group, self._now(), self.max_age):
            return self._failure(MoveItErrorCodes.INVALID_OBJECT_NAME, "invalid_request",
                                 "need fresh ID-only target/group; geometry/optional inputs unsupported")
        if self.backend_timeout_latched:
            return self._failure(MoveItErrorCodes.FAILURE, "backend_timeout_latched",
                                 "restart and inspect external backend and adapter before retry")
        if self.pending is not None:
            return self._failure(MoveItErrorCodes.FAILURE, "busy")
        if not self.backend.service_is_ready():
            return self._failure(MoveItErrorCodes.FAILURE, "backend_unavailable")
        completion = Future(executor=self.executor)
        deadline = time.monotonic() + self.timeout
        try:
            backend_future = self.backend.call_async(request)
        except Exception as error:
            return self._failure(MoveItErrorCodes.FAILURE, "backend_error", str(error))
        pending = {"completion": completion, "future": backend_future,
                   "deadline": deadline, "stamp": stamp,
                   "target_id": request.target.id}
        self.pending = pending
        backend_future.add_done_callback(lambda done: self._complete(pending, done))
        self._expire()
        return await completion

    def _finish(self, pending, response):
        if self.pending is not pending:
            return
        self.pending = None
        pending["completion"].set_result(response)

    def _expire(self):
        pending = self.pending
        if pending is not None and time.monotonic() >= pending["deadline"]:
            self.backend_timeout_latched = True
            response = self._failure(MoveItErrorCodes.TIMED_OUT, "backend_timeout",
                                     "no retry; late results discarded")
            self._finish(pending, response)
            # Remove rclpy's pending request bookkeeping as well as our own slot.
            pending["future"].cancel()
            self.backend.remove_pending_request(pending["future"])

    def _complete(self, pending, done):
        if self.pending is not pending:
            return
        if time.monotonic() >= pending["deadline"]:
            self._expire()
            return
        try:
            response = done.result()
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise ValueError("backend reported error " + str(response.error_code.val))
            grasps = checked_grasps(response.grasps, frame=self.frame, now=self._now(),
                                    max_age=self.max_age, earliest_stamp=pending["stamp"],
                                    max_candidates=self.max_candidates)
            # Preserve all individual grasp stamps in the service result. PoseArray
            # cannot represent mixed acquisition stamps, so reject mixed batches.
            stamps = {stamp_seconds(g.grasp_pose.header.stamp) for g in grasps}
            if len(stamps) != 1:
                raise ValueError("mixed backend acquisition stamps")
            poses = PoseArray(header=deepcopy(grasps[0].grasp_pose.header),
                              poses=[deepcopy(g.grasp_pose.pose) for g in grasps])
            # Materialization and result retrieval can consume time too. Never
            # publish a response that expired while we validated/copied it.
            if time.monotonic() >= pending["deadline"]:
                self._expire()
                return
            now = self._now()
            if (not fresh(pending["stamp"], now, self.max_age)
                    or not all(fresh(stamp, now, self.max_age) for stamp in stamps)):
                raise ValueError("request/result expired during materialization")
            self.poses.publish(poses)
            self._report("proposals", request_target_id=pending["target_id"],
                         source_stamp=next(iter(stamps)), frame_id=self.frame,
                         count=len(grasps), pose_semantics="upstream_eef_parent",
                         request_stamp=pending["stamp"])
            self._finish(pending, response)
        except Exception as error:
            self._finish(pending, self._failure(MoveItErrorCodes.FAILURE,
                                               "backend_rejected", str(error)))


def main(args=None):
    domain = int(os.environ.get("ROS_DOMAIN_ID", "76"))
    if not 1 <= domain <= 232:
        raise ValueError("Intel grasp adapter requires ROS_DOMAIN_ID from 1 through 232")
    os.environ.setdefault("ROS_DOMAIN_ID", "76")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=args)
    node = None
    try:
        node = IntelGraspAdapter()
        # Single threaded executor + asynchronous service: state mutations stay serial.
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
