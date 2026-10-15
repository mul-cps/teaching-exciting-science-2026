#!/usr/bin/env python3
"""Plan SO-ARM101 grasp pose segments through MoveIt; never execute motions."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))

from tools.ros_plan_current import ALL_JOINTS, ARM_JOINTS, _joint_state_map, _wait_future
from tools.so101_grasp_plan import PoseTarget, candidate_for_track, staged_targets


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--track-id", help="tracked detection id such as red-0003")
    selection.add_argument("--pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                           help="explicit contact pose in --frame")
    parser.add_argument("--frame", default="charuco_board")
    parser.add_argument("--status-topic", default="/so101/grasp_status")
    parser.add_argument("--joint-states-topic", default="/follower/joint_states")
    parser.add_argument("--planning-service", default="/follower/plan_kinematic_path")
    parser.add_argument("--group", default="manipulator")
    parser.add_argument("--tip-link", default="gripper_frame_link")
    parser.add_argument("--output", required=True, help="new JSON output file")
    parser.add_argument("--place-pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                        help="optional place contact pose in --frame")
    parser.add_argument("--planning-time", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not 0 < args.planning_time <= 60 or not 0 < args.timeout <= 180:
        parser.error("planning time and timeout must be positive and bounded")
    return args


def main(argv=None):
    args = parse_args(argv)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import (Constraints, MoveItErrorCodes, MotionPlanRequest,
                                 OrientationConstraint, PositionConstraint, RobotState)
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String

    class Client(Node):
        def __init__(self):
            super().__init__("so101_grasp_plan")
            self.joints = None
            self.status = None
            self.status_received = 0.0
            self.create_subscription(JointState, args.joint_states_topic, self.on_joints,
                                     qos_profile_sensor_data)
            self.create_subscription(String, args.status_topic, self.on_status, 10)
            self.planner = self.create_client(GetMotionPlan, args.planning_service)

        def on_joints(self, msg):
            self.joints = msg

        def on_status(self, msg):
            try:
                self.status = json.loads(msg.data)
                self.status_received = time.monotonic()
            except (ValueError, TypeError):
                self.status = None

    rclpy.init()
    node = Client()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok():
            now_ros = node.get_clock().now().nanoseconds * 1e-9
            try:
                start = _joint_state_map(node.joints, now_ros) if node.joints else None
            except ValueError:
                start = None
            tracked_ready = (args.track_id is None or
                             (node.status is not None and
                              time.monotonic() - node.status_received <= 0.5))
            if start is not None and tracked_ready:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("fresh complete measured joints and target status were not received")
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.planner.wait_for_service(timeout_sec=max(0.0, deadline-time.monotonic())):
            raise TimeoutError(f"MoveIt planning service unavailable: {args.planning_service}")

        contact = (candidate_for_track(node.status, args.track_id) if args.track_id else
                   PoseTarget(args.frame, tuple(args.pose[:3]), tuple(args.pose[3:])))
        locked_target = contact
        place = (PoseTarget(args.frame, tuple(args.place_pose[:3]), tuple(args.place_pose[3:]))
                 if args.place_pose else None)
        segments = staged_targets(contact, place=place)
        result = {
            "schema": 1, "state": "planning", "execution_enabled": False,
            "source": "tracked geometric proposal" if args.track_id else "explicit pose",
            "track_id": args.track_id, "planning_group": args.group,
            "tip_link": args.tip_link, "joint_state_stamp": {
                "sec": node.joints.header.stamp.sec,
                "nanosec": node.joints.header.stamp.nanosec},
            "gripper": {"open": "required before pregrasp; no qualified command interface",
                        "close": "required only at contact; no force/contact acknowledgement",
                        "state": "uncommanded"},
            "object_attachment": "not implemented; scene remains unchanged",
            "segments": [],
        }
        current = start
        for name, target in segments:
            if time.monotonic() - node.status_received > 0.5 and args.track_id:
                result["state"] = "target_status_stale"
                result["segments"].append({"name": name, "state": result["state"]})
                break
            if args.track_id:
                try:
                    observed_target = candidate_for_track(node.status, args.track_id)
                except (KeyError, TypeError, ValueError):
                    observed_target = None
                q0 = locked_target.orientation_xyzw
                q1 = observed_target.orientation_xyzw if observed_target else (0, 0, 0, 0)
                dot = abs(sum(a*b for a, b in zip(q0, q1)))
                if observed_target is None or any(
                        abs(a - b) > 0.003 for a, b in zip(
                            observed_target.position, locked_target.position)) or dot < 0.999:
                    result["state"] = "target_changed_during_planning"
                    result["segments"].append({"name": name, "state": result["state"]})
                    break
            req = MotionPlanRequest()
            req.group_name = args.group
            req.pipeline_id = "ompl"
            req.planner_id = "RRTConnectkConfigDefault"
            req.allowed_planning_time = args.planning_time
            req.num_planning_attempts = 3
            req.max_velocity_scaling_factor = 0.20
            req.max_acceleration_scaling_factor = 0.20
            req.start_state = RobotState()
            req.start_state.joint_state.header = node.joints.header
            req.start_state.joint_state.name = list(ALL_JOINTS)
            req.start_state.joint_state.position = [current[j] for j in ALL_JOINTS]
            req.start_state.is_diff = False
            pose = PoseStamped()
            pose.header.frame_id = target.frame_id
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target.position
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = target.orientation_xyzw
            constraints = Constraints()
            position = PositionConstraint()
            position.header = pose.header
            position.link_name = args.tip_link
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.008]
            position.constraint_region.primitives = [sphere]
            position.constraint_region.primitive_poses = [pose.pose]
            position.weight = 1.0
            constraints.position_constraints = [position]
            orientation = OrientationConstraint()
            orientation.header = pose.header
            orientation.link_name = args.tip_link
            orientation.orientation = pose.pose.orientation
            # Five arm joints cannot satisfy arbitrary six-axis pose constraints.
            orientation.absolute_x_axis_tolerance = 0.35
            orientation.absolute_y_axis_tolerance = 0.35
            orientation.absolute_z_axis_tolerance = 0.45
            orientation.weight = 1.0
            constraints.orientation_constraints = [orientation]
            req.goal_constraints = [constraints]
            service_request = GetMotionPlan.Request()
            service_request.motion_plan_request = req
            response = _wait_future(node, node.planner.call_async(service_request),
                                    max(0.1, deadline-time.monotonic()), f"{name} plan")
            planned = response.motion_plan_response
            segment = {"name": name, "error_code": planned.error_code.val,
                       "planning_time_s": planned.planning_time}
            if planned.error_code.val != MoveItErrorCodes.SUCCESS:
                segment["state"] = "path_plan_failed"
                result["segments"].append(segment)
                result["state"] = "incomplete"
                break
            points = planned.trajectory.joint_trajectory.points
            names = planned.trajectory.joint_trajectory.joint_names
            if (not points or len(names) != len(set(names)) or
                    set(names) != set(ARM_JOINTS) or
                    len(points[-1].positions) != len(names) or
                    not all(math.isfinite(float(value)) for value in points[-1].positions)):
                segment["state"] = "invalid_planner_trajectory"
                result["segments"].append(segment)
                result["state"] = "incomplete"
                break
            current.update(dict(zip(names, points[-1].positions)))
            segment.update({"state": "path_planned", "joint_names": list(names),
                            "point_count": len(points),
                            "trajectory": json.loads(__import__("rosidl_runtime_py.convert", fromlist=["message_to_yaml"])
                                                      .message_to_yaml(planned.trajectory))})
            result["segments"].append(segment)
        else:
            result["state"] = "all_paths_planned_not_collision_checked_after_retime"
        result["note"] = ("MoveIt planned the requested pose paths against its current scene. "
                          "Contact may be rejected because the brick remains a world collision; "
                          "no grasp, attachment, jaw motion, or physical success is claimed.")
        with open(args.output, "x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        print(f"{result['state']}: plan artifact written to {args.output}", flush=True)
        return 0 if result["state"] == "all_paths_planned_not_collision_checked_after_retime" else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
