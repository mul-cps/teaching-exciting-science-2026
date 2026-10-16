#!/usr/bin/env python3
"""Plan SO-ARM101 grasp paths; hardware motion requires explicit --execute."""

import argparse
import hashlib
import json
import math
import numpy as np
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))
sys.path.insert(0, str(REPOSITORY / "ros2/src/so101_station_scene"))

from tools.ros_plan_current import ALL_JOINTS, ARM_JOINTS, _joint_state_map, _wait_future
from tools.so101_grasp_plan import PoseTarget, candidate_for_track, staged_targets, transform_pose
from so101_station_scene.scene_geometry import matrix_quaternion_xyzw, quaternion_matrix_xyzw


def load_execution_calibration(jaw_path, tcp_path, *, tip_link):
    """Fail closed unless SI jaw and registered TCP/object transforms are qualified."""
    import numpy as np
    jaw = json.loads(Path(jaw_path).read_text(encoding="utf-8"))
    tcp = json.loads(Path(tcp_path).read_text(encoding="utf-8"))
    if (jaw.get("schema") != "so101-jaw-gap-calibration-v1" or
            jaw.get("qualified") is not True):
        raise ValueError("jaw calibration must be explicitly qualified")
    if (tcp.get("schema") != "so101-grasp-tcp-calibration-v1" or
            tcp.get("qualified") is not True or tcp.get("tcp_link") != tip_link or
            tcp.get("units") != {"translation": "m", "rotation": "quaternion_xyzw"}):
        raise ValueError("TCP calibration must be qualified, SI, and match --tip-link")
    matrix = np.asarray(tcp.get("tcp_from_object"), dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) or
            np.linalg.det(matrix[:3, :3]) < 0.999):
        raise ValueError("tcp_from_object must be a proper finite SE(3) transform")
    if tcp.get("grasp_frame_convention") != "x_closing_z_approach":
        raise ValueError("TCP grasp_frame_convention must declare x_closing_z_approach")
    try:
        transform_pose(PoseTarget("calibration", (0., 0., 0.), (0., 0., 0., 1.)),
                       tcp.get("tip_from_grasp"))
    except (ValueError, TypeError) as error:
        raise ValueError("tip_from_grasp must be a calibrated proper SE(3) transform") from error
    return jaw, tcp, matrix


def verify_observed_lift(status_samples, track_id, baseline_position, *, rise_m=0.04,
                         max_spread_m=0.018, expected_position=None, max_tracking_error_m=0.06,
                         now_ros=None, expected_frame_id=None):
    """Require three fresh observations of the same brick remaining elevated."""
    track = str(track_id)
    observations = []
    last_stamp = None
    for received, status in status_samples:
        stamp = status.get("stamp")
        if (status.get("state") != "tracking" or not status.get("frame_id") or
                (expected_frame_id is not None and status.get("frame_id") != expected_frame_id) or
                not verify_live_registration(status.get("registration"),
                                             status.get("registration_received", -1), now_ros=now_ros) or
                not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or
                (now_ros is not None and not 0 <= now_ros - stamp < 0.5) or
                time.monotonic() - received >= 0.5 or
                (last_stamp is not None and stamp <= last_stamp)):
            continue
        last_stamp = stamp
        item = next((obj for obj in status.get("objects", [])
                     if str(obj.get("track_id")) == track), None)
        if (item is None or item.get("geometric_only") is not True
                or item.get("position_semantics") not in (None, "support_plane_box_estimate")
                or item.get("dimensions_semantics") not in (None, "support_plane_box_extent_estimate")):
            continue
        position = item.get("position_world")
        if isinstance(position, list) and len(position) == 3 and all(
                math.isfinite(float(v)) for v in position):
            observations.append(tuple(float(v) for v in position))
    if len(observations) < 3:
        return False
    observations = observations[-3:]
    z_values = [position[2] for position in observations[-3:]]
    return (min(z_values) - float(baseline_position[2]) >= rise_m and
            max(z_values) - min(z_values) <= max_spread_m and
            (expected_position is None or all(
                math.dist(position, expected_position) <= max_tracking_error_m
                for position in observations[-3:])))


def verify_camera_arm_calibration(path):
    calibration = json.loads(Path(path).read_text(encoding="utf-8"))
    if (calibration.get("schema") != "camera-arm-calibration-v1" or
            calibration.get("transform_direction") != "base_from_camera" or
            calibration.get("qualified") is not True):
        raise ValueError("execution requires qualified camera-arm base registration")
    import numpy as np
    matrix = np.asarray(calibration.get("base_from_camera"), dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=2e-4) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=2e-4)):
        raise ValueError("camera-arm base_from_camera must be a proper SE(3) matrix")
    return calibration


def verify_effective_model(robot_description, expected_sha256):
    digest = hashlib.sha256(str(robot_description).encode("utf-8")).hexdigest()
    if digest != expected_sha256:
        raise ValueError("effective robot_description SHA256 does not match execution config")
    return digest


def validate_model_links(robot_description, links):
    root = ET.fromstring(str(robot_description))
    available = {item.get("name") for item in root.findall("link")}
    missing = sorted(set(links) - available)
    if missing:
        raise ValueError("contact/tool links are absent from effective robot model: " + ", ".join(missing))
    return True


def robot_description_from_parameter_response(response):
    """Read robot_description from the native GetParameters service response."""
    values = getattr(response, "values", None)
    if not values or not getattr(values[0], "string_value", ""):
        raise ValueError("effective robot_description is unavailable")
    return values[0].string_value


def ros_message_json(message):
    """Serialize ROSIDL messages without the YAML tags emitted by message_to_yaml."""
    from rosidl_runtime_py.convert import message_to_ordereddict
    return json.loads(json.dumps(message_to_ordereddict(message), allow_nan=False))


def validate_clear_joint_pose(value):
    if not isinstance(value, dict) or set(value) != set(ARM_JOINTS):
        raise ValueError("return joint pose must map exactly the five arm joint names to radians")
    pose = {name: float(value[name]) for name in ARM_JOINTS}
    if not all(math.isfinite(rad) for rad in pose.values()):
        raise ValueError("return joint pose radians must be finite")
    return pose


def canonical_scene_track_id(candidate_record):
    """Return Detection3D.id used by scene ownership, distinct from numeric track_id."""
    candidate_id = candidate_record.get("candidate_id") if isinstance(candidate_record, dict) else None
    if isinstance(candidate_id, str) and "-g" in candidate_id:
        detection_id = candidate_id.rsplit("-g", 1)[0]
        if detection_id:
            return detection_id
    detection_id = candidate_record.get("detection_id") if isinstance(candidate_record, dict) else None
    if isinstance(detection_id, str) and detection_id:
        return detection_id
    raise ValueError("grasp candidate has no canonical Detection3D ID for scene ownership")


def scene_request_payload(action, track_id, request_id, **fields):
    if not isinstance(track_id, str) or not track_id or not isinstance(request_id, str) or not request_id:
        raise ValueError("scene ownership requests require canonical track and request IDs")
    return {"schema": 1, "request_id": request_id, "action": action,
            "track_id": track_id, **fields}


def verify_live_registration(status, received_at, now=None, *, now_ros=None):
    now = time.monotonic() if now is None else now
    stamp = status.get("stamp") if isinstance(status, dict) else None
    return bool(isinstance(status, dict) and status.get("state") == "tracking" and
                status.get("observation_valid") is True and
                isinstance(stamp, (int, float)) and math.isfinite(stamp) and
                (now_ros is None or 0 <= now_ros - stamp < 0.5) and
                0 <= now - received_at < 0.5)


def require_live_registration(status, received_at, *, now=None, now_ros=None):
    if not verify_live_registration(status, received_at, now, now_ros=now_ros):
        raise RuntimeError("fresh Charuco base registration observation required immediately before arm motion")


def submit_localized_goal(client, goal, *, registration, received_at, now_ros, deadline):
    """Validate at submission, after any action-server discovery delay."""
    require_live_registration(registration, received_at, now_ros=now_ros)
    if time.monotonic() >= deadline:
        raise TimeoutError("execution deadline expired before action goal submission")
    return client.send_goal_async(goal)


def scene_ack_terminal_state(action, ack):
    state = ack.get("state") if isinstance(ack, dict) else None
    if state in {"reserved", "pending", "applying"}:
        return None
    if state in {"failed", "rejected", "error", "fault"}:
        raise RuntimeError(f"scene {action} failed: {ack.get('reason', state)}")
    expected = {"attach": "attached", "detach": "detached", "lock": "locked",
                "unlock": "unlocked", "allow_contact": "contact_allowed",
                "disallow_contact": "contact_disallowed"}[action]
    if state != expected:
        raise RuntimeError(f"scene {action} failed: {ack.get('reason', state)}")
    return state


def verify_observed_release(status_samples, track_id, expected_position, *, tolerance_m=0.05,
                            now_ros=None, expected_frame_id=None):
    track = str(track_id)
    observations = []
    last_stamp = None
    for received, status in status_samples:
        stamp = status.get("stamp")
        if (status.get("state") != "tracking" or not status.get("frame_id") or
                (expected_frame_id is not None and status.get("frame_id") != expected_frame_id) or
                not verify_live_registration(status.get("registration"),
                                             status.get("registration_received", -1), now_ros=now_ros) or
                not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or
                (now_ros is not None and not 0 <= now_ros - stamp < 0.5) or
                time.monotonic() - received >= 0.5 or
                (last_stamp is not None and stamp <= last_stamp)):
            continue
        last_stamp = stamp
        item = next((obj for obj in status.get("objects", [])
                     if str(obj.get("track_id")) == track), None)
        if (item is None or item.get("geometric_only") is not True
                or item.get("position_semantics") not in (None, "support_plane_box_estimate")
                or item.get("dimensions_semantics") not in (None, "support_plane_box_extent_estimate")):
            continue
        position = item.get("position_world")
        if isinstance(position, list) and len(position) == 3 and all(
                math.isfinite(float(v)) for v in position):
            observations.append(tuple(float(v) for v in position))
    if len(observations) < 3:
        return False
    expected = tuple(float(v) for v in expected_position)
    return all(math.dist(position, expected) <= tolerance_m for position in observations[-3:])


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
    parser.add_argument("--placement-selection", help="pinned moving placement UUID; requires qualified placement_goal_from_tip")
    parser.add_argument("--planning-time", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--execute", action="store_true",
                        help="execute a qualified tracked pick/place through resident actions")
    parser.add_argument("--execution-config", help="qualified jaw/TCP/contact configuration JSON")
    parser.add_argument("--jaw-calibration", help="qualified jaw-gap calibration JSON")
    parser.add_argument("--tcp-calibration", help="qualified TCP/object calibration JSON")
    parser.add_argument("--gripper-action", default="/follower/gripper_controller/gripper_cmd")
    parser.add_argument("--trajectory-action", default="/follower/arm_trajectory_controller/follow_joint_trajectory")
    parser.add_argument("--attachment-request-topic", default="/so101_scene/attachment_request")
    parser.add_argument("--attachment-status-topic", default="/so101_scene/attachment_status")
    parser.add_argument("--touch-link", action="append", default=[])
    parser.add_argument("--return-joint-pose", help="JSON file with commissioned arm joint radians for post-release clearance")
    parser.add_argument("--open-gap-m", type=float)
    parser.add_argument("--close-gap-m", type=float)
    args = parser.parse_args(argv)
    if not 0 < args.planning_time <= 60 or not 0 < args.timeout <= 180:
        parser.error("planning time and timeout must be positive and bounded")
    if args.execute and (not args.track_id or not args.place_pose or not args.execution_config):
        parser.error("--execute requires --track-id, --place-pose, and --execution-config")
    return args


def main(argv=None):
    args = parse_args(argv)
    import rclpy
    import signal
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import PoseStamped
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes, MotionPlanRequest,
                                 OrientationConstraint, PositionConstraint, RobotState)
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class Client(Node):
        def __init__(self):
            super().__init__("so101_grasp_plan")
            self.joints = None
            self.status = None
            self.status_received = 0.0
            self.status_history = []
            self.attachment_acks = {}
            self.active_goal_handle = None
            self.placement_status = None
            self.placement_received = 0.
            self.registration_status = None
            self.registration_received = 0.0
            self.create_subscription(JointState, args.joint_states_topic, self.on_joints,
                                     qos_profile_sensor_data)
            self.create_subscription(String, "/station/placement_goal_observation", self.on_placement, 10)
            self.create_subscription(String, args.status_topic, self.on_status, 10)
            self.create_subscription(String, "/so101_perception/charuco_world/status",
                                     self.on_registration_status, 10)
            self.create_subscription(String, args.attachment_status_topic, self.on_attachment_status, 10)
            self.attachment_pub = self.create_publisher(String, args.attachment_request_topic, 10)
            self.planner = self.create_client(GetMotionPlan, args.planning_service)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            if args.execute:
                self.arm_action = ActionClient(self, FollowJointTrajectory, args.trajectory_action)
                self.gripper_action = ActionClient(self, GripperCommand, args.gripper_action)

        def on_placement(self, msg):
            try:
                self.placement_status = json.loads(msg.data)
                self.placement_received = time.monotonic()
            except (ValueError, TypeError):
                self.placement_status = None

        def on_joints(self, msg):
            self.joints = msg

        def on_status(self, msg):
            try:
                self.status = json.loads(msg.data)
                self.status_received = time.monotonic()
                self.status["registration"] = self.registration_status
                self.status["registration_received"] = self.registration_received
                self.status_history.append((self.status_received, self.status))
                self.status_history = self.status_history[-20:]
            except (ValueError, TypeError):
                self.status = None

        def on_attachment_status(self, msg):
            try:
                item = json.loads(msg.data)
                if item.get("request_id"):
                    self.attachment_acks[item["request_id"]] = item
            except (ValueError, TypeError):
                pass

        def on_registration_status(self, msg):
            try:
                self.registration_status = json.loads(msg.data)
                self.registration_received = time.monotonic()
            except (ValueError, TypeError):
                self.registration_status = None

    # Keep the ROS context alive while interruption cancels and waits for the
    # resident goal. rclpy's default signal handler shuts it down first, making
    # the cancellation requests below unable to reach the action server.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt_execution(_signum, _frame):
        raise KeyboardInterrupt
    previous_signal_handlers = {
        signum: signal.signal(signum, interrupt_execution)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = Client()
    try:
        jaw_calibration = tcp_calibration = tcp_from_object = None
        selected_track_id = None
        scene_track_id = None
        baseline_object = None
        attached = False
        scene_locked = False
        attachment_attempted = False
        pickup_verified = lift_verified = release_verified = False
        if args.execute:
            config_path = Path(args.execution_config).resolve()
            execution_config = json.loads(config_path.read_text(encoding="utf-8"))
            if (execution_config.get("schema") != "so101-grasp-execution-config-v1" or
                    execution_config.get("qualified") is not True):
                raise ValueError("execution config must be qualified and use so101-grasp-execution-config-v1")
            def config_path_for(field):
                value = execution_config.get(field)
                if not isinstance(value, str) or not value:
                    raise ValueError(f"execution config requires {field}")
                path = Path(value)
                return path if path.is_absolute() else config_path.parent / path
            args.jaw_calibration = str(config_path_for("jaw_calibration"))
            args.tcp_calibration = str(config_path_for("tcp_calibration"))
            camera_arm_path = config_path_for("camera_arm_calibration")
            verify_camera_arm_calibration(camera_arm_path)
            expected_model_sha256 = execution_config.get("effective_model_sha256")
            if (not isinstance(expected_model_sha256, str) or len(expected_model_sha256) != 64 or
                    any(c not in "0123456789abcdef" for c in expected_model_sha256)):
                raise ValueError("execution config requires lowercase effective_model_sha256")
            model_parameter_node = execution_config.get("model_parameter_node", "/move_group")
            if not isinstance(model_parameter_node, str) or not model_parameter_node.startswith("/"):
                raise ValueError("model_parameter_node must be an absolute ROS node name")
            args.touch_link = execution_config.get("touch_links")
            if not isinstance(args.touch_link, list) or not args.touch_link:
                raise ValueError("execution config requires explicit touch_links")
            args.open_gap_m = float(execution_config["open_gap_m"])
            args.close_gap_m = float(execution_config["close_gap_m"])
            jaw_calibration, tcp_calibration, tcp_from_object = load_execution_calibration(
                args.jaw_calibration, args.tcp_calibration, tip_link=args.tip_link)
            if not (math.isfinite(args.open_gap_m) and math.isfinite(args.close_gap_m) and
                    jaw_calibration["closed_gap_m"] <= args.close_gap_m < args.open_gap_m <=
                    jaw_calibration["open_gap_m"]):
                raise ValueError("open/close jaw gaps must be finite and within qualified calibration")
            if not args.place_pose:
                raise ValueError("execution requires an explicit place pose")

        def spin_future(future, deadline, label):
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                raise TimeoutError(f"{label} timed out")
            return future.result()

        def cancel_active_goal(deadline):
            handle = node.active_goal_handle
            if handle is None:
                return True
            cancel_future = handle.cancel_goal_async()
            response = spin_future(cancel_future, deadline, "action cancel acknowledgement")
            if not getattr(response, "goals_canceling", []):
                if handle.status not in (4, 5, 6):
                    raise RuntimeError("resident action did not confirm cancellation")
            spin_future(handle.get_result_async(), deadline, "action terminal result")
            node.active_goal_handle = None
            return True

        def send_action_goal(client, goal, deadline, label):
            future = submit_localized_goal(client, goal,
                registration=node.registration_status,
                received_at=node.registration_received,
                now_ros=node.get_clock().now().nanoseconds * 1e-9, deadline=deadline)
            try:
                return spin_future(future, deadline, label)
            except BaseException:
                recovery_deadline = time.monotonic() + 5.0
                while rclpy.ok() and not future.done() and time.monotonic() < recovery_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if future.done():
                    late_handle = future.result()
                    if late_handle.accepted:
                        node.active_goal_handle = late_handle
                        cancel_active_goal(recovery_deadline)
                raise

        def command_gap(gap, deadline):
            if not node.gripper_action.wait_for_server(timeout_sec=max(0.0, deadline-time.monotonic())):
                raise TimeoutError("resident GripperCommand action is unavailable")
            goal = GripperCommand.Goal()
            goal.command.position = float(gap)
            goal.command.max_effort = 0.0
            handle = send_action_goal(node.gripper_action, goal, deadline, "gripper goal")
            if not handle.accepted:
                raise RuntimeError("resident gripper action rejected goal; calibration or owner unavailable")
            node.active_goal_handle = handle
            try:
                wrapped = spin_future(handle.get_result_async(), deadline, "gripper result")
            except BaseException:
                cancel_active_goal(time.monotonic() + 4.0)
                raise
            node.active_goal_handle = None
            return wrapped.result

        def move_trajectory(trajectory, deadline):
            if not node.arm_action.wait_for_server(timeout_sec=max(0.0, deadline-time.monotonic())):
                raise TimeoutError("resident FollowJointTrajectory action is unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory.joint_trajectory
            handle = send_action_goal(node.arm_action, goal, deadline, "arm goal")
            if not handle.accepted:
                raise RuntimeError("resident trajectory action rejected path")
            node.active_goal_handle = handle
            try:
                wrapped = spin_future(handle.get_result_async(), deadline, "arm trajectory result")
            except BaseException:
                cancel_active_goal(time.monotonic() + 4.0)
                raise
            node.active_goal_handle = None
            if wrapped.status != 4 or wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(f"resident trajectory failed: {wrapped.result.error_string}")

        def scene_attachment(action, track_id, deadline, **fields):
            request_id = f"grasp-{time.monotonic_ns()}"
            payload = scene_request_payload(action, track_id, request_id, **fields)
            msg = String()
            msg.data = json.dumps(payload, sort_keys=True, allow_nan=False)
            node.attachment_pub.publish(msg)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                ack = node.attachment_acks.pop(request_id, None)
                if ack:
                    terminal_state = scene_ack_terminal_state(action, ack)
                    if terminal_state is None:
                        continue
                    return ack
            raise TimeoutError(f"scene {action} acknowledgement timed out; ownership is unresolved")

        def execute_clear_pose(path):
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            clear_pose = validate_clear_joint_pose(data)
            now_ros = node.get_clock().now().nanoseconds * 1e-9
            measured = _joint_state_map(node.joints, now_ros)
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
            req.start_state.joint_state.position = [measured[j] for j in ALL_JOINTS]
            constraints = Constraints()
            for joint in ARM_JOINTS:
                constraint = JointConstraint()
                constraint.joint_name = joint
                constraint.position = clear_pose[joint]
                constraint.tolerance_above = 0.015
                constraint.tolerance_below = 0.015
                constraint.weight = 1.0
                constraints.joint_constraints.append(constraint)
            req.goal_constraints = [constraints]
            service_request = GetMotionPlan.Request()
            service_request.motion_plan_request = req
            planned = _wait_future(node, node.planner.call_async(service_request),
                                   max(0.1, deadline-time.monotonic()), "post-release clear pose plan").motion_plan_response
            if planned.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError("collision checked return joint pose planning failed")
            points = planned.trajectory.joint_trajectory.points
            if not points or set(planned.trajectory.joint_trajectory.joint_names) != set(ARM_JOINTS):
                raise RuntimeError("clear pose planner returned an invalid arm trajectory")
            require_live_registration(node.registration_status, node.registration_received,
                now_ros=node.get_clock().now().nanoseconds * 1e-9)
            move_trajectory(planned.trajectory, deadline)
            result["segments"].append({"name": "post_release_clear_pose", "state": "trajectory_succeeded",
                                       "trajectory": ros_message_json(planned.trajectory)})

        def base_link_pose(pose_in_frame):
            if pose_in_frame["frame_id"] == "base_link":
                return pose_in_frame
            transform = node.tf_buffer.lookup_transform("base_link", pose_in_frame["frame_id"], Time())
            transform_matrix = np.eye(4)
            t = transform.transform.translation
            q = transform.transform.rotation
            transform_matrix[:3, :3] = quaternion_matrix_xyzw((q.x, q.y, q.z, q.w))
            transform_matrix[:3, 3] = [t.x, t.y, t.z]
            object_matrix = np.eye(4)
            object_matrix[:3, :3] = quaternion_matrix_xyzw(pose_in_frame["orientation_xyzw"])
            object_matrix[:3, 3] = pose_in_frame["position"]
            composed = transform_matrix @ object_matrix
            return {"frame_id": "base_link", "position": composed[:3, 3].tolist(),
                    "orientation_xyzw": list(matrix_quaternion_xyzw(composed[:3, :3]))}

        def execute_tool_retreat(target):
            # Retreat in the calibrated physical grasp frame, then convert
            # back to the commanded link; tool-link Z can differ from grasp Z.
            grasp_pose = transform_pose(target, tcp_calibration["tip_from_grasp"])
            shifted = staged_targets(grasp_pose,
                tip_from_grasp=tcp_calibration["tip_from_grasp"])[0][1]
            now_ros = node.get_clock().now().nanoseconds * 1e-9
            measured = _joint_state_map(node.joints, now_ros)
            req = MotionPlanRequest()
            req.group_name, req.pipeline_id, req.planner_id = args.group, "ompl", "RRTConnectkConfigDefault"
            req.allowed_planning_time, req.num_planning_attempts = args.planning_time, 3
            req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.20
            req.start_state = RobotState()
            req.start_state.joint_state.header = node.joints.header
            req.start_state.joint_state.name = list(ALL_JOINTS)
            req.start_state.joint_state.position = [measured[j] for j in ALL_JOINTS]
            pose = PoseStamped()
            pose.header.frame_id = shifted.frame_id
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = shifted.position
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = shifted.orientation_xyzw
            constraints = Constraints()
            position = PositionConstraint()
            position.header, position.link_name = pose.header, args.tip_link
            sphere = SolidPrimitive()
            sphere.type, sphere.dimensions = SolidPrimitive.SPHERE, [0.008]
            position.constraint_region.primitives = [sphere]
            position.constraint_region.primitive_poses = [pose.pose]
            position.weight = 1.0
            orientation = OrientationConstraint()
            orientation.header, orientation.link_name = pose.header, args.tip_link
            orientation.orientation = pose.pose.orientation
            orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = 0.35
            orientation.absolute_z_axis_tolerance, orientation.weight = 0.45, 1.0
            constraints.position_constraints = [position]
            constraints.orientation_constraints = [orientation]
            req.goal_constraints = [constraints]
            service_request = GetMotionPlan.Request()
            service_request.motion_plan_request = req
            planned = _wait_future(node, node.planner.call_async(service_request),
                                   max(0.1, deadline-time.monotonic()), "tool retreat plan").motion_plan_response
            if planned.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError("collision checked post-release tool retreat planning failed")
            require_live_registration(node.registration_status, node.registration_received,
                now_ros=node.get_clock().now().nanoseconds * 1e-9)
            move_trajectory(planned.trajectory, deadline)
            result["segments"].append({"name": "post_release_tool_retreat",
                                       "state": "trajectory_succeeded",
                                       "trajectory": ros_message_json(planned.trajectory)})

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

        contact = (candidate_for_track(node.status, args.track_id,
                                      require_pickup_eligible=args.execute) if args.track_id else
                   PoseTarget(args.frame, tuple(args.pose[:3]), tuple(args.pose[3:])))
        if args.execute:
            candidate_record = next(item for item in node.status.get("grasp_candidates", [])
                                    if (str(item.get("track_id")) == str(args.track_id) or
                                        str(item.get("candidate_id", "")).rsplit("-g", 1)[0] == str(args.track_id)))
            selected_track_id = candidate_record["track_id"]
            scene_track_id = canonical_scene_track_id(candidate_record)
            baseline_object = next((item for item in node.status.get("objects", [])
                                    if str(item.get("track_id")) == str(selected_track_id)), None)
            if (baseline_object is None or not baseline_object.get("geometric_only")
                    or baseline_object.get("pickup_eligible") is not True):
                raise RuntimeError("fresh geometric object observation required before execution")
        locked_target = contact
        place = (PoseTarget(args.frame, tuple(args.place_pose[:3]), tuple(args.place_pose[3:]))
                 if args.place_pose else None)
        def selected_place():
            from workshop.digital_twin.placement_region import load_qualified_frame_registration
            from workshop.station.placement_execution import resolve_tool_destination, DestinationPose, require_observed_footprint
            require_observed_footprint(baseline_object.get("size"), execution_config.get("placement_footprint_m"))
            registration = load_qualified_frame_registration(camera_arm_path)
            if registration.sha256 != placement_registration.sha256:
                raise RuntimeError("trusted placement registration changed during execution")
            resolved, position, orientation, area = resolve_tool_destination(node.placement_status,
                selection_id=args.placement_selection, registration=registration,
                received_monotonic=node.placement_received, now_monotonic=time.monotonic(),
                now_ros_ns=node.get_clock().now().nanoseconds,
                goal_from_tip=execution_config.get("placement_goal_from_tip"),
                footprint_m=execution_config.get("placement_footprint_m"),
                tool_footprint_m=execution_config.get("placement_tool_footprint_m"),
                clearance_m=execution_config.get("placement_clearance_m"),
                registration_uncertainty_m=execution_config.get("placement_registration_uncertainty_m"))
            frame = node.status["frame_id"]
            if frame == resolved.frame_id:
                return DestinationPose(frame, position, orientation, resolved.selection_id,
                    resolved.registration_sha256, area, resolved.observation_stamp_ns)
            # Compose the placement into the same observation frame used for
            # independent release verification, at image acquisition time.
            tf = node.tf_buffer.lookup_transform(frame, resolved.frame_id,
                Time(nanoseconds=resolved.observation_stamp_ns))
            matrix = np.eye(4)
            q, t = tf.transform.rotation, tf.transform.translation
            matrix[:3, :3] = quaternion_matrix_xyzw((q.x, q.y, q.z, q.w))
            matrix[:3, 3] = [t.x, t.y, t.z]
            tool = np.eye(4)
            tool[:3, :3] = quaternion_matrix_xyzw(orientation)
            tool[:3, 3] = position
            composed = matrix @ tool
            return DestinationPose(frame, tuple(composed[:3, 3]),
                tuple(matrix_quaternion_xyzw(composed[:3, :3])), resolved.selection_id,
                resolved.registration_sha256, area, resolved.observation_stamp_ns)
        if args.placement_selection:
            if not args.execute:
                raise ValueError("tracked placement requires qualified execution configuration")
            from workshop.digital_twin.placement_region import load_qualified_frame_registration
            placement_registration = load_qualified_frame_registration(camera_arm_path)
            place = selected_place()
        if args.execute and not args.placement_selection and node.status.get("frame_id") != place.frame_id:
            raise ValueError("execute place pose frame must match the live geometric status frame")
        segments = staged_targets(contact, place=place,
            tip_from_grasp=tcp_calibration["tip_from_grasp"] if args.execute else None)
        result = {
            "schema": 1, "state": "planning", "execution_enabled": bool(args.execute),
            "source": "tracked geometric proposal" if args.track_id else "explicit pose",
            "track_id": args.track_id, "planning_group": args.group,
            "tip_link": args.tip_link, "joint_state_stamp": {
                "sec": node.joints.header.stamp.sec,
                "nanosec": node.joints.header.stamp.nanosec},
            "gripper": {"open": "calibrated SI gap" if args.execute else "uncommanded",
                        "close": "calibrated SI gap; no force unit supported" if args.execute else "uncommanded",
                        "state": "pending" if args.execute else "uncommanded"},
            "selected_track_id": selected_track_id,
            "object_attachment": {"state": "not_attached", "pickup_verified": False,
                                  "lift_verified": False, "release_verified": False},
            "pickup_verified": False, "lift_verified": False, "release_verified": False,
            "segments": [],
        }
        if args.execute:
            from rclpy.parameter_client import AsyncParameterClient
            model_client = AsyncParameterClient(node, model_parameter_node)
            if not model_client.wait_for_services(timeout_sec=max(0.0, deadline-time.monotonic())):
                raise TimeoutError(f"model parameter service unavailable at {model_parameter_node}")
            model_response = spin_future(model_client.get_parameters(["robot_description"]),
                                         deadline, "effective robot model")
            robot_description = robot_description_from_parameter_response(model_response)
            effective_model_sha256 = verify_effective_model(
                robot_description, expected_model_sha256)
            validate_model_links(robot_description,
                                 [args.tip_link, *args.touch_link])
            result["calibration"] = {"jaw": str(Path(args.jaw_calibration).resolve()),
                                     "tcp": str(Path(args.tcp_calibration).resolve()),
                                     "camera_arm": str(camera_arm_path.resolve()),
                                     "effective_model_sha256": effective_model_sha256,
                                     "model_parameter_node": model_parameter_node,
                                     "execution_config": str(config_path),
                                     "tcp_link": args.tip_link,
                                     "units": {"position": "m", "orientation": "quaternion_xyzw"}}
            scene_attachment("lock", scene_track_id, deadline)
            scene_locked = True
            open_result = command_gap(args.open_gap_m, deadline)
            if not open_result.reached_goal:
                raise RuntimeError("gripper did not reach calibrated open gap")
            result["gripper"]["state"] = "open"
        current = start
        contact_matrix_enabled = False
        for name, target in segments:
            if time.monotonic() - node.status_received > 0.5 and args.track_id:
                result["state"] = "target_status_stale"
                result["segments"].append({"name": name, "state": result["state"]})
                break
            if args.execute:
                require_live_registration(node.registration_status, node.registration_received,
                    now_ros=node.get_clock().now().nanoseconds * 1e-9)
            if args.track_id:
                if not (args.execute and attached and name == "place"):
                    try:
                        observed_target = candidate_for_track(node.status, args.track_id,
                                                              require_pickup_eligible=args.execute)
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
                elif not any(str(item.get("track_id")) == str(selected_track_id)
                             for item in node.status.get("objects", [])):
                    result["state"] = "attached_track_observation_lost"
                    result["segments"].append({"name": name, "state": result["state"]})
                    break
            if args.execute and name == "contact":
                scene_attachment("allow_contact", scene_track_id, deadline,
                                 touch_links=args.touch_link)
                contact_matrix_enabled = True
            for placement_attempt in range(3):
                if name == "place" and args.placement_selection:
                    target = place = selected_place()
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
                if name != "place" or not args.placement_selection:
                    break
                from workshop.station.placement_execution import same_destination
                if same_destination(target, selected_place()):
                    break
            else:
                raise RuntimeError("placement kept moving during bounded replanning")
            planned = response.motion_plan_response
            segment = {"name": name, "error_code": planned.error_code.val,
                       "planning_time_s": planned.planning_time}
            if name == "place" and args.placement_selection:
                segment["selected_placement"] = dict(selection_id=target.selection_id,
                    registration_sha256=target.registration_sha256,
                    observation_stamp_ns=target.observation_stamp_ns,
                    area_half_extents_m=target.area_half_extents_m,
                    planning_attempts=placement_attempt+1)
            if planned.error_code.val != MoveItErrorCodes.SUCCESS:
                if contact_matrix_enabled:
                    scene_attachment("disallow_contact", scene_track_id, deadline,
                                     touch_links=args.touch_link)
                    contact_matrix_enabled = False
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
                if contact_matrix_enabled:
                    scene_attachment("disallow_contact", scene_track_id, deadline,
                                     touch_links=args.touch_link)
                    contact_matrix_enabled = False
                segment["state"] = "invalid_planner_trajectory"
                result["segments"].append(segment)
                result["state"] = "incomplete"
                break
            if args.execute and args.track_id and not (attached and name == "place"):
                if (node.status is None or time.monotonic() - node.status_received >= 0.5 or
                        node.status.get("state") != "tracking" or
                        node.status.get("frame_id") != locked_target.frame_id):
                    raise RuntimeError("target tracking became stale while planning")
                current_candidate = candidate_for_track(node.status, args.track_id,
                                                        require_pickup_eligible=True)
                if (any(abs(a-b) > 0.003 for a, b in zip(current_candidate.position,
                                                           locked_target.position)) or
                        abs(sum(a*b for a, b in zip(current_candidate.orientation_xyzw,
                                                    locked_target.orientation_xyzw))) < 0.999):
                    raise RuntimeError("target moved while planning; motion refused")
            current.update(dict(zip(names, points[-1].positions)))
            segment.update({"state": "path_planned", "joint_names": list(names),
                            "point_count": len(points),
                            "trajectory": ros_message_json(planned.trajectory)})
            if args.execute:
                # Planner latency can outlive the registration observation used
                # to admit the request; recheck immediately before every action.
                require_live_registration(node.registration_status, node.registration_received,
                    now_ros=node.get_clock().now().nanoseconds * 1e-9)
                if name == "place" and args.placement_selection:
                    from workshop.station.placement_execution import same_destination
                    validated_place = selected_place()
                    if not same_destination(target, validated_place):
                        raise RuntimeError("placement moved after planning; trajectory refused")
                    segment["selected_placement"]["validated_stamp_ns"] = validated_place.observation_stamp_ns
                move_trajectory(planned.trajectory, deadline)
                segment["execution_state"] = "trajectory_succeeded"
                if name == "contact":
                    close_result = command_gap(args.close_gap_m, deadline)
                    segment["gripper_contact"] = {
                        "reached_goal": bool(close_result.reached_goal),
                        "stalled": bool(close_result.stalled),
                        "position_m": float(close_result.position),
                        "effort": "unavailable"}
                    if close_result.reached_goal or not close_result.stalled:
                        raise RuntimeError("close did not infer contact; refusing provisional attachment")
                    attachment_attempted = True
                    scene_attachment("attach", scene_track_id, deadline,
                                     link_name=args.tip_link, touch_links=args.touch_link)
                    attached = True
                    scene_locked = False
                    scene_attachment("disallow_contact", scene_track_id, deadline,
                                     touch_links=args.touch_link)
                    contact_matrix_enabled = False
                    result["object_attachment"]["state"] = "provisionally_attached"
                    result["gripper"]["state"] = "contact_inferred"
                if name == "lift":
                    lift_started = time.monotonic()
                    lift_tcp = np.eye(4)
                    lift_tcp[:3, :3] = quaternion_matrix_xyzw(target.orientation_xyzw)
                    lift_tcp[:3, 3] = target.position
                    expected_object_position = (lift_tcp @ tcp_from_object)[:3, 3].tolist()
                    samples = []
                    while time.monotonic() < min(deadline, lift_started + 3.0):
                        rclpy.spin_once(node, timeout_sec=0.05)
                        samples = [(received, status) for received, status in node.status_history
                                   if received >= lift_started and received <= time.monotonic()]
                        if verify_observed_lift(samples, selected_track_id,
                                                baseline_object["position_world"],
                                                expected_position=expected_object_position,
                                                now_ros=node.get_clock().now().nanoseconds * 1e-9,
                                                expected_frame_id=target.frame_id):
                            pickup_verified = lift_verified = True
                            break
                    result["pickup_verified"] = pickup_verified
                    result["lift_verified"] = lift_verified
                    result["object_attachment"].update(
                        state="attached" if attached else "not_attached",
                        pickup_verified=pickup_verified, lift_verified=lift_verified,
                        lift_observation_count=len(samples))
                    if not lift_verified:
                        result["state"] = "pickup_unverified_attached_stop"
                        result["note"] = ("Object elevation was not independently observed; scene attachment "
                                          "is retained and the CLI stops before place or release.")
                        segment["state"] = "lift_completed_pickup_evidence_missing"
                        result["segments"].append(segment)
                        break
                if name == "place":
                    if not lift_verified:
                        raise RuntimeError("place is prohibited until independent lift verification")
                    open_result = command_gap(args.open_gap_m, deadline)
                    if not open_result.reached_goal:
                        raise RuntimeError("jaw failed to reach open gap at place; retaining attachment")
                    # Place pose targets the calibrated TCP. Compose the qualified TCP-to-object pose.
                    tcp_from_world = np.eye(4)
                    tcp_from_world[:3, :3] = quaternion_matrix_xyzw(target.orientation_xyzw)
                    tcp_from_world[:3, 3] = target.position
                    world_from_object = tcp_from_world @ tcp_from_object
                    release_expected = {"position": world_from_object[:3, 3].tolist(),
                                        "orientation_xyzw": list(matrix_quaternion_xyzw(world_from_object[:3, :3]))}
                    release_pose = base_link_pose({"frame_id": target.frame_id, **release_expected})
                    scene_attachment("detach", scene_track_id, deadline,
                                     world_pose=release_pose)
                    attached = False
                    scene_locked = False
                    result["object_attachment"]["state"] = "detached"
                    result["gripper"]["state"] = "released"
                    release_started = time.monotonic()
                    released_samples = []
                    while time.monotonic() < min(deadline, release_started + 3.0):
                        rclpy.spin_once(node, timeout_sec=0.05)
                        released_samples = [(received, status) for received, status in node.status_history
                                            if received >= release_started]
            result["segments"].append(segment)
        else:
            if args.execute and result.get("state") == "planning" and not attached and 'release_started' in locals():
                if args.return_joint_pose:
                    execute_clear_pose(args.return_joint_pose)
                else:
                    execute_tool_retreat(place)
                release_started = time.monotonic()
                released_samples = []
                while time.monotonic() < min(deadline, release_started + 3.0):
                    rclpy.spin_once(node, timeout_sec=0.05)
                    released_samples = [(received, status) for received, status in node.status_history
                                        if received >= release_started]
                    if verify_observed_release(released_samples, selected_track_id,
                                               release_expected["position"],
                                               now_ros=node.get_clock().now().nanoseconds * 1e-9,
                                               expected_frame_id=place.frame_id):
                        release_verified = True
                        break
                result["release_verified"] = release_verified
                result["object_attachment"]["release_verified"] = release_verified
                result["object_attachment"]["release_observation_count"] = len(released_samples)
            if args.execute and result["state"] == "planning":
                result["state"] = "pick_place_complete" if pickup_verified and release_verified else "execution_incomplete"
            elif not args.execute:
                result["state"] = "all_paths_planned_not_collision_checked_after_retime"
        result.setdefault("note", "Jaw gap reached does not prove pickup; pickup verification requires fresh stable elevated RGB-D track observations.")
        with open(args.output, "x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        success = (result["state"] == "pick_place_complete" and result.get("track_id") and
                   result.get("pickup_verified") is True and result.get("lift_verified") is True and
                   result.get("release_verified") is True)
        print(f"{result['state']}: plan artifact written to {args.output}", flush=True)
        return 0 if success or result["state"] == "all_paths_planned_not_collision_checked_after_retime" else 2
    except (Exception, KeyboardInterrupt) as error:
        failed = locals().get("result")
        if locals().get("scene_locked") and not locals().get("attachment_attempted"):
            try:
                scene_attachment("unlock", locals().get("scene_track_id"),
                                 time.monotonic() + 2.0)
                scene_locked = False
            except Exception as unlock_error:
                if isinstance(failed, dict):
                    failed["scene_unlock_error"] = str(unlock_error)
        if locals().get("contact_matrix_enabled") and locals().get("selected_track_id") is not None:
            try:
                scene_attachment("disallow_contact", locals().get("scene_track_id"),
                                 time.monotonic() + 2.0,
                                 touch_links=locals().get("args").touch_link)
                contact_matrix_enabled = False
            except Exception as cleanup_error:
                if isinstance(failed, dict):
                    failed["contact_matrix_cleanup_error"] = str(cleanup_error)
        if isinstance(failed, dict) and not Path(args.output).exists():
            failed["state"] = "execution_failed_attached_stop" if locals().get("attached") else "execution_failed"
            failed["error"] = f"{type(error).__name__}: {error}"
            failed["pickup_verified"] = locals().get("pickup_verified", False)
            failed["lift_verified"] = locals().get("lift_verified", False)
            failed["release_verified"] = locals().get("release_verified", False)
            if locals().get("attached"):
                failed["object_attachment"]["state"] = "attached_stop_retain_ownership"
            with open(args.output, "x", encoding="utf-8") as stream:
                json.dump(failed, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
            print(f"{failed['state']}: failure artifact written to {args.output}", flush=True)
            return 2
        raise
    finally:
        while node.active_goal_handle is not None and rclpy.ok():
            try:
                cancel_active_goal(time.monotonic() + 5.0)
            except BaseException as cancel_error:
                node.get_logger().error(f"retrying until resident action cancellation is confirmed: {cancel_error}")
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for signum, handler in previous_signal_handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
