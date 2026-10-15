#!/usr/bin/env python3
"""Plan a small joint-space move from fresh measured state; execution is opt-in."""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY))

from workshop.station import ros_trajectory as trajectory_helper

ARM_JOINTS = trajectory_helper.ARM_JOINTS
ALL_JOINTS = (*ARM_JOINTS, "gripper")
DEFAULT_OFFSETS = {"shoulder_pan": 0.05}


def parse_joint_offsets(value: str | None) -> dict[str, float]:
    """Parse a partial JSON joint-offset map; unspecified joints stay at zero."""
    if value is None:
        parsed = dict(DEFAULT_OFFSETS)
    else:
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError as error:
            raise ValueError("--joint-offset must be a JSON object") from error
        if not isinstance(parsed, dict):
            raise ValueError("--joint-offset must be a JSON object")
    unknown = set(parsed) - set(ARM_JOINTS)
    if unknown:
        raise ValueError(f"unknown arm joints in --joint-offset: {sorted(unknown)}")
    offsets = {joint: 0.0 for joint in ARM_JOINTS}
    for joint, value in parsed.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"offset for {joint} must be a finite number")
        if abs(value) > 0.25:
            raise ValueError(f"offset for {joint} exceeds the 0.25 rad planning bound")
        offsets[joint] = float(value)
    return offsets


def validate_scaling(value: float, label: str) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise ValueError(f"{label} must be in (0, 1]")
    return value


def scale_points(points: list[dict], factor: float) -> list[dict]:
    """Time-scale points while preserving their spatial path and derivatives."""
    if not math.isfinite(factor) or factor < 1:
        raise ValueError("retime factor must be finite and at least one")
    scaled = []
    for point in points:
        result = dict(point)
        result["time"] = float(point["time"]) * factor
        if "velocities" in result:
            result["velocities"] = [float(x) / factor for x in result["velocities"]]
        if "accelerations" in result:
            result["accelerations"] = [float(x) / (factor * factor)
                                        for x in result["accelerations"]]
        scaled.append(result)
    return scaled


def prepare_with_retiming(joint_names, points, *, start_radians,
                          velocity_scale=0.25, acceleration_scale=0.25):
    """Retry only dynamics-limit failures with uniform time scaling."""
    velocity_limit = trajectory_helper.ROS_MAX_VELOCITY_RAD_S * validate_scaling(
        velocity_scale, "velocity scale")
    acceleration_limit = trajectory_helper.ROS_MAX_ACCELERATION_RAD_S2 * validate_scaling(
        acceleration_scale, "acceleration scale")
    factor = 1.0
    for _ in range(24):
        effective_points = scale_points(points, factor)
        try:
            prepared = trajectory_helper.prepare_trajectory(
                joint_names, effective_points, start_radians=start_radians,
                max_velocity=velocity_limit, max_acceleration=acceleration_limit,
                max_jerk=trajectory_helper.ROS_MAX_JERK_RAD_S3, period=0.05)
            return factor, effective_points, prepared
        except ValueError as error:
            if not any(term in str(error) for term in (
                    "velocity limit exceeded", "acceleration limit exceeded", "jerk limit exceeded")):
                raise
            factor *= 1.2
    raise ValueError("trajectory still exceeds dynamics limits after bounded retiming")


def max_deviation_from_linear_knots(prepared, points, factor: float) -> float:
    """Report max per-joint deviation from linearly interpolated planner knots."""
    knots = [(float(point["time"]), tuple(float(x) for x in point["positions"]))
             for point in points]
    maximum = 0.0
    segment = 0
    for sample in prepared.samples:
        source_time = sample.time / factor
        while segment < len(knots) - 2 and source_time > knots[segment + 1][0]:
            segment += 1
        left_time, left = knots[segment]
        right_time, right = knots[segment + 1]
        ratio = min(1.0, max(0.0, (source_time - left_time) / (right_time - left_time)))
        for actual, a, b in zip(sample.positions, left, right):
            maximum = max(maximum, abs(actual - (a + ratio * (b - a))))
    return maximum


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _duration_message(seconds, Duration):
    nanoseconds = int(round(seconds * 1e9))
    return Duration(sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000)


def _parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", required=True,
                        help="new directory for original/effective trajectories and metadata")
    parser.add_argument("--joint-offset", default=None,
                        help='partial JSON object, e.g. \'{"shoulder_pan":0.05}\'')
    parser.add_argument("--velocity-scale", type=float, default=0.25)
    parser.add_argument("--acceleration-scale", type=float, default=0.25)
    parser.add_argument("--namespace", default="/follower",
                        help="MoveIt and measured-state namespace (default: /follower)")
    parser.add_argument("--planning-service", default=None,
                        help="override the MoveIt service name (default: <namespace>/plan_kinematic_path)")
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--execute", action="store_true",
                        help="send the retimed trajectory to FollowJointTrajectory after saving")
    args = parser.parse_args(argv)
    try:
        args.offsets = parse_joint_offsets(args.joint_offset)
        args.velocity_scale = validate_scaling(args.velocity_scale, "velocity scale")
        args.acceleration_scale = validate_scaling(args.acceleration_scale, "acceleration scale")
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.planning_time) or not 0 < args.planning_time <= 120:
        parser.error("--planning-time must be in (0, 120]")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 300:
        parser.error("--timeout must be in (0, 300]")
    args.namespace = "/" + args.namespace.strip("/") if args.namespace.strip("/") else ""
    args.planning_service = args.planning_service or args.namespace + "/plan_kinematic_path"
    if not args.planning_service.startswith("/"):
        parser.error("--planning-service must be an absolute ROS name")
    return args


def _wait_future(node, future, timeout_s, description):
    import rclpy

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            future.cancel()
            raise TimeoutError(f"timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    if not future.done():
        raise RuntimeError(f"ROS shut down while waiting for {description}")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{description} returned no result")
    return result


def _joint_state_map(message, now_seconds):
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise ValueError("JointState names and positions are malformed or duplicated")
    stamp = _duration_seconds(message.header.stamp)
    if abs(now_seconds - stamp) > 0.5:
        raise ValueError("JointState header is older than 0.5 seconds or from the future")
    values = dict(zip(message.name, message.position))
    missing = set(ALL_JOINTS) - set(values)
    if missing:
        raise ValueError(f"fresh JointState is missing joints: {sorted(missing)}")
    state = {joint: float(values[joint]) for joint in ALL_JOINTS}
    if not all(math.isfinite(value) for value in state.values()):
        raise ValueError("JointState contains non-finite positions")
    return state


def _trajectory_points(trajectory):
    output = []
    for point in trajectory.joint_trajectory.points:
        item = {"time": _duration_seconds(point.time_from_start),
                "positions": list(point.positions)}
        if point.velocities:
            item["velocities"] = list(point.velocities)
        if point.accelerations:
            item["accelerations"] = list(point.accelerations)
        output.append(item)
    return output


def _write_artifacts(directory, original, effective, metadata, message_to_yaml):
    target = Path(directory).expanduser().resolve()
    target.mkdir(parents=True, exist_ok=False)
    (target / "original_trajectory.yaml").write_text(message_to_yaml(original), encoding="utf-8")
    (target / "effective_trajectory.yaml").write_text(message_to_yaml(effective), encoding="utf-8")
    (target / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return target


def main(argv=None) -> int:
    args = _parse_arguments(argv)
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, MotionPlanRequest, RobotState
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_yaml
    from sensor_msgs.msg import JointState

    class PlanClient(Node):
        def __init__(self):
            super().__init__("so101_plan_current")
            self.latest_joint_state = None
            self.create_subscription(JointState, args.namespace + "/joint_states",
                                     self._joint_state, qos_profile_sensor_data)
            self.plan_client = self.create_client(GetMotionPlan, args.planning_service)

        def _joint_state(self, message):
            self.latest_joint_state = message

    rclpy.init()
    node = PlanClient()
    action_handle = None
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok():
            now = node.get_clock().now().nanoseconds * 1e-9
            message = node.latest_joint_state
            if message is not None:
                try:
                    start = _joint_state_map(message, now)
                    break
                except ValueError:
                    pass
            if time.monotonic() >= deadline:
                raise TimeoutError("no complete fresh /follower/joint_states sample within timeout")
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("ROS shut down before a fresh JointState arrived")

        if not node.plan_client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
            raise TimeoutError(f"MoveIt service unavailable: {args.planning_service}")

        request = MotionPlanRequest()
        request.group_name = "manipulator"
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnectkConfigDefault"
        request.allowed_planning_time = args.planning_time
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = args.velocity_scale
        request.max_acceleration_scaling_factor = args.acceleration_scale
        request.start_state = RobotState()
        request.start_state.joint_state.header = message.header
        request.start_state.joint_state.name = list(ALL_JOINTS)
        request.start_state.joint_state.position = [start[joint] for joint in ALL_JOINTS]
        request.start_state.is_diff = False
        constraints = Constraints()
        for joint in ARM_JOINTS:
            constraint = JointConstraint()
            constraint.joint_name = joint
            constraint.position = start[joint] + args.offsets[joint]
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        service_request = GetMotionPlan.Request()
        service_request.motion_plan_request = request
        remaining = max(0.1, deadline - time.monotonic())
        response = _wait_future(node, node.plan_client.call_async(service_request),
                                remaining, "MoveIt kinematic path plan")
        planned = response.motion_plan_response
        if planned.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt planning failed with error code {planned.error_code.val}")
        original = planned.trajectory
        trajectory = original.joint_trajectory
        if set(trajectory.joint_names) != set(ARM_JOINTS) or len(trajectory.joint_names) != len(ARM_JOINTS):
            raise ValueError(f"planner returned unexpected joint names: {trajectory.joint_names}")
        input_points = _trajectory_points(original)
        start_arm = {joint: start[joint] for joint in ARM_JOINTS}
        factor, effective_points, prepared = prepare_with_retiming(
            trajectory.joint_names, input_points, start_radians=start_arm,
            velocity_scale=args.velocity_scale, acceleration_scale=args.acceleration_scale)
        effective = copy.deepcopy(original)
        for point in effective.joint_trajectory.points:
            point.time_from_start = _duration_message(
                _duration_seconds(point.time_from_start) * factor, Duration)
            for index in range(len(point.velocities)):
                point.velocities[index] /= factor
            for index in range(len(point.accelerations)):
                point.accelerations[index] /= factor * factor
        goals = {joint: start[joint] + args.offsets[joint] for joint in ARM_JOINTS}
        metadata = {
            "schema": 1,
            "status": "planned_validated_not_executed",
            "planning_group": "manipulator",
            "pipeline_id": request.pipeline_id,
            "planner_id": request.planner_id,
            "error_code": planned.error_code.val,
            "planning_time_s": planned.planning_time,
            "joint_state_topic": args.namespace + "/joint_states",
            "planning_service": args.planning_service,
            "joint_state_stamp": {"sec": message.header.stamp.sec,
                                   "nanosec": message.header.stamp.nanosec},
            "start_state_rad": start,
            "goal_offsets_rad": args.offsets,
            "goal_rad": goals,
            "velocity_scale": args.velocity_scale,
            "acceleration_scale": args.acceleration_scale,
            "retime_factor": factor,
            "maximum_deviation_from_linear_knots_rad": max_deviation_from_linear_knots(
                prepared, input_points, factor),
            "effective_path_samples": [
                {"time": sample.time, "positions": list(sample.positions)}
                for sample in prepared.samples],
            "original_points": input_points,
            "effective_points": effective_points,
        }
        output_directory = _write_artifacts(
            args.output_directory, original, effective, metadata, message_to_yaml)
        print(f"PLAN VALIDATED: saved original and retimed plan in {output_directory}; no execution requested",
              flush=True)

        if args.execute:
            action_name = args.namespace + "/arm_trajectory_controller/follow_joint_trajectory"
            action_client = ActionClient(node, FollowJointTrajectory, action_name)
            if not action_client.wait_for_server(timeout_sec=max(0.0, deadline - time.monotonic())):
                raise TimeoutError(f"FollowJointTrajectory action unavailable: {action_name}")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = effective.joint_trajectory
            goal_handle = _wait_future(node, action_client.send_goal_async(goal),
                                       max(0.1, deadline - time.monotonic()), "trajectory goal acceptance")
            action_handle = goal_handle
            if not goal_handle.accepted:
                raise RuntimeError("FollowJointTrajectory goal was rejected")
            result = _wait_future(node, goal_handle.get_result_async(), args.timeout,
                                  "FollowJointTrajectory result")
            result_code = result.result.error_code
            result_text = result.result.error_string
            metadata["status"] = "executed_successfully" if result_code == 0 else "execution_failed"
            metadata["action_result"] = {"error_code": result_code, "error_string": result_text}
            (output_directory / "metadata.json").write_text(
                json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8")
            if result_code != 0:
                raise RuntimeError(f"FollowJointTrajectory failed with result code {result_code}")
        return 0
    except KeyboardInterrupt:
        if action_handle is not None and action_handle.accepted:
            try:
                _wait_future(node, action_handle.cancel_goal_async(), 2.0, "trajectory cancellation")
            except Exception:
                pass
        raise
    except Exception as error:
        print(f"ros_plan_current: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
