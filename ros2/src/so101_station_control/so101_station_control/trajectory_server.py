"""FollowJointTrajectory adapter that delegates all bus access to ArmSession."""
from __future__ import annotations

import json
import importlib.util
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import uuid

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


class _PreflightCancelled(RuntimeError):
    pass


class _PathCollisionError(ValueError):
    def __init__(self, message, artifact):
        super().__init__(message)
        self.artifact = artifact


def _seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def _duration(seconds):
    seconds = max(0.0, seconds)
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


class TrajectoryBridge(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("so101_trajectory_bridge", parameter_overrides=parameter_overrides)
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("external_state_only", False)
        self.declare_parameter("publish_gripper_visual_proxy", True)
        self.declare_parameter("station_python", os.environ.get("SO101_STATION_PYTHON", ""))
        self.declare_parameter("station_root", os.environ.get("SO101_STATION_ROOT", os.getcwd()))
        self.declare_parameter("station_config", "")
        self.declare_parameter("state_file", "")
        self.declare_parameter("log_dir", "")
        self.declare_parameter("trajectory_log_dir", "")
        self.declare_parameter("station_lock", "")
        self.declare_parameter("action_name", "/follower/arm_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("state_validity_service", "/follower/check_state_validity")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("state_freshness_s", 0.25)
        self.declare_parameter("collision_check_timeout_s", 30.0)
        self.declare_parameter("max_jerk_rad_s3", 2.0)
        self._active_lock = threading.Lock()
        self._active_goal = None
        self._cancel_before_start = False
        self._write_lock = threading.Lock()
        self._action_group = ReentrantCallbackGroup()
        self._response_queue = queue.Queue()
        self._closed = False
        self._process = None
        self._owner_started = 0.0
        self._owner_faulted = False
        self._state_file = self._resolve_state_file()
        self._external_state_only = bool(self.get_parameter("external_state_only").value)
        self._trajectory_helper = None
        self._state_validity = None
        # Tail a separately owned resident without starting, locking, or commanding another owner.
        if not self._external_state_only:
            self._start_owner()
        self._action = ActionServer(
            self, FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            cancel_callback=self._cancel_request,
            callback_group=self._action_group)
        self._joint_state_pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self.create_timer(0.05, self._publish_joint_state)
        if not self._external_state_only:
            self._trajectory_helper = self._load_trajectory_helper()
            self._state_validity = self.create_client(
                GetStateValidity, str(self.get_parameter("state_validity_service").value),
                callback_group=self._action_group)
        if self._external_state_only:
            self.get_logger().warning(
                f"READ ONLY: mirroring external state from {self._state_file}; trajectory goals are disabled; "
                "gripper joint is a visual-only normalized-percent-to-URDF-range estimate")
        else:
            self.get_logger().warning(
                "Gripper radians are unqualified; the published gripper joint is a visual-only normalized-percent-to-URDF-range estimate and is never commanded")

    def _publish_joint_state(self):
        state = self._read_state()
        if state is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(ARM_JOINTS)
        message.position = [math.radians(float(state["pose"][name])) for name in ARM_JOINTS]
        if bool(self.get_parameter("publish_gripper_visual_proxy").value):
            # Digital-twin convention: normalized opening percent maps linearly to the
            # canonical URDF joint range [-10, 100] degrees. This is visual only.
            opening = min(100.0, max(0.0, float(state["pose"]["gripper"])))
            message.name.append("gripper")
            message.position.append(math.radians(-10.0 + 1.1 * opening))
        self._joint_state_pub.publish(message)

    def _resolve_state_file(self):
        configured = str(self.get_parameter("state_file").value)
        if configured:
            return Path(configured)
        runtime = os.environ.get("XDG_RUNTIME_DIR")
        root = Path(runtime) if runtime and Path(runtime).is_dir() else Path("/dev/shm")
        return root / f"so101-arm-{os.getuid()}.json"

    @staticmethod
    def _is_runtime_path(path):
        resolved = Path(path).resolve()
        runtime_value = os.environ.get("XDG_RUNTIME_DIR")
        roots = [Path("/dev/shm").resolve()]
        if runtime_value:
            roots.append(Path(runtime_value).resolve())
        return any(resolved == root or root in resolved.parents for root in roots)

    def _load_trajectory_helper(self):
        source = Path(str(self.get_parameter("station_root").value)) / "workshop/station/ros_trajectory.py"
        if not source.is_file():
            raise RuntimeError(f"pure trajectory helper not found: {source}")
        spec = importlib.util.spec_from_file_location("so101_ros_trajectory_shared", source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    def _start_owner(self):
        if self._external_state_only:
            raise RuntimeError("external_state_only mode must never start an ArmSession owner")
        if not bool(self.get_parameter("enable_hardware").value):
            raise RuntimeError("hardware action bridge requires enable_hardware:=true")
        station_python = str(self.get_parameter("station_python").value)
        station_root = Path(str(self.get_parameter("station_root").value)).resolve()
        config = str(self.get_parameter("station_config").value)
        if not station_python or not Path(station_python).is_file():
            raise RuntimeError("station_python must name the station venv Python executable")
        if not station_root.is_dir() or not config:
            raise RuntimeError("station_root and station_config are required")
        logs = str(self.get_parameter("log_dir").value)
        if not logs:
            runtime = os.environ.get("XDG_RUNTIME_DIR") or "/dev/shm"
            logs = str(Path(runtime) / "so101-arm-owner-logs")
        lock = str(self.get_parameter("station_lock").value)
        if not lock:
            state_home = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
            lock = str(state_home / "exciting-science/station.lock")
        if not self._is_runtime_path(self._state_file) or not self._is_runtime_path(logs):
            raise RuntimeError("ArmSession state_file and log_dir must be RAM-backed under XDG_RUNTIME_DIR or /dev/shm")
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        command = [station_python, "-m", "workshop.station.arm_session",
                   "--config", config, "--state-file", str(self._state_file),
                   "--logdir", logs, "--lock", lock]
        child_env = os.environ.copy()
        for key in tuple(child_env):
            if (key.startswith(("PYTHON", "LD_", "ROS_", "AMENT_", "COLCON_", "RMW_",
                                "CONDA_", "VIRTUAL_ENV")) or
                    key in {"CMAKE_PREFIX_PATH", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"}):
                child_env.pop(key, None)
        child_env["PATH"] = os.pathsep.join((str(Path(station_python).parent), os.defpath))
        self.get_logger().info("Starting the configured ArmSession bus owner subprocess")
        self._owner_started = time.monotonic()
        self._process = subprocess.Popen(
            command, cwd=station_root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1, env=child_env)
        threading.Thread(target=self._read_responses, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError(f"ArmSession exited during startup ({self._process.returncode})")
            state = self._read_state()
            if (state and state.get("event") == "connected" and
                    float(state.get("monotonic", 0.0)) >= self._owner_started):
                self.get_logger().info("ArmSession connected; action server is ready")
                return
            time.sleep(0.05)
        self._process.terminate()
        raise RuntimeError("ArmSession state did not become fresh within 20 seconds")

    def _read_responses(self):
        for line in self._process.stdout:
            try:
                self._response_queue.put(json.loads(line))
            except json.JSONDecodeError:
                self.get_logger().error("ArmSession emitted invalid JSON on stdout")

    def _read_stderr(self):
        for line in self._process.stderr:
            self.get_logger().error("ArmSession: " + line.rstrip())

    def _read_state(self, *, allow_stale=False):
        try:
            state = json.loads(self._state_file.read_text())
            if (not allow_stale and
                    time.monotonic() - float(state["monotonic"]) > float(self.get_parameter("state_freshness_s").value)):
                return None
            return state
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            return None

    def _send(self, command):
        if self._process is None or self._process.poll() is not None:
            raise RuntimeError("ArmSession owner is not running")
        with self._write_lock:
            self._process.stdin.write(json.dumps(command, allow_nan=False) + "\n")
            self._process.stdin.flush()

    def _get_validity(self, positions, jaw_position, deadline):
        request = GetStateValidity.Request()
        request.group_name = str(self.get_parameter("planning_group").value)
        request.robot_state.joint_state.name = [*ARM_JOINTS, "gripper"]
        request.robot_state.joint_state.position = [*positions, jaw_position]
        future = self._state_validity.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not completed.wait(remaining):
            future.cancel()
            raise TimeoutError("MoveIt state-validity service timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt state-validity service returned no response")
        return response

    def _collision_check_path(self, trajectory, points, goal_id):
        state = self._read_state()
        if state is None:
            raise RuntimeError("fresh measured state is required before collision checking")
        start = {joint: math.radians(float(state["pose"][joint])) for joint in ARM_JOINTS}
        parsed_points = []
        for point in points:
            item = {"time": _seconds(point.time_from_start),
                    "positions": list(point.positions)}
            if point.velocities:
                item["velocities"] = list(point.velocities)
            if point.accelerations:
                item["accelerations"] = list(point.accelerations)
            parsed_points.append(item)
        helper = self._trajectory_helper
        max_jerk = float(self.get_parameter("max_jerk_rad_s3").value)
        if max_jerk <= 0 or max_jerk > helper.ROS_MAX_JERK_RAD_S3:
            raise ValueError(f"max_jerk_rad_s3 must be in (0, {helper.ROS_MAX_JERK_RAD_S3}]")
        prepared = helper.prepare_trajectory(
            trajectory.joint_names, parsed_points, start_radians=start,
            max_velocity=helper.ROS_MAX_VELOCITY_RAD_S,
            max_acceleration=helper.ROS_MAX_ACCELERATION_RAD_S2,
            max_jerk=max_jerk, period=0.05)
        dense = helper.dense_path_samples(prepared, max_joint_step_rad=0.01)
        jaw_opening = min(100.0, max(0.0, float(state["pose"]["gripper"])))
        jaw_position = math.radians(-10.0 + 1.1 * jaw_opening)
        deadline = time.monotonic() + float(self.get_parameter("collision_check_timeout_s").value)
        if not self._state_validity.wait_for_service(timeout_sec=min(2.0, max(0.0, deadline - time.monotonic()))):
            raise RuntimeError("MoveIt state-validity service is unavailable; no trajectory was sent")
        artifact = {
            "schema": 1, "goal_id": goal_id, "status": "collision_checking",
            "arm_id": state.get("arm_id"), "calibration_sha256": state.get("calibration_sha256"),
            "validated_monotonic": time.monotonic(), "planning_group": str(self.get_parameter("planning_group").value),
            "state_validity_service": str(self.get_parameter("state_validity_service").value),
            "start_state": {"raw": state.get("raw"), "pose": state.get("pose"),
                            "monotonic": state.get("monotonic")},
            "original_points": parsed_points,
            "effective_path_sha256": helper.path_digest(
                prepared, held_gripper_raw=state["raw"]["gripper"]),
            "effective_samples": [{"time": sample.time, "positions": list(sample.positions)}
                                  for sample in prepared.samples],
            "dense_collision_checks": [],
            "visual_only_jaw": {"opening_percent": jaw_opening, "joint_position_rad": jaw_position,
                                "raw": state["raw"]["gripper"],
                                "mapping": "normalized 0..100 percent -> URDF -10..100 degrees; unqualified"},
            "path_tolerance": "default only; non-default rejected",
            "goal_tolerance": "default only; station final tolerance is 2 degrees",
            "max_jerk_rad_s3": max_jerk,
        }
        for index, positions in enumerate(dense):
            with self._active_lock:
                if self._cancel_before_start:
                    raise _PreflightCancelled("cancelled during collision preflight")
            response = self._get_validity(positions, jaw_position, deadline)
            record = {"sample": index, "valid": bool(response.valid),
                      "positions": list(positions), "jaw_visual_only_rad": jaw_position}
            artifact["dense_collision_checks"].append(record)
            if not response.valid:
                contacts = [{"body_1": c.contact_body_1,
                             "body_2": c.contact_body_2}
                            for c in response.contacts[:8]]
                artifact["status"] = "collision_rejected"
                record["contacts"] = contacts
                raise _PathCollisionError(
                    f"effective trajectory collides at dense state {index}: {contacts}", artifact)
        artifact["status"] = "collision_validated_not_sent"
        return artifact

    def _write_artifact(self, artifact):
        configured = str(self.get_parameter("trajectory_log_dir").value)
        directory = Path(configured) if configured else Path.home() / ".local/share/exciting-science/station/ros-trajectory-logs"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"trajectory-{artifact['goal_id']}.json"
        temporary = path.with_suffix(".json.tmp")
        with temporary.open("w") as stream:
            json.dump(artifact, stream, sort_keys=True, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        dir_fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
        return path

    def _goal_request(self, goal_request):
        if self._external_state_only:
            self.get_logger().warning("Rejecting trajectory goal: external_state_only mode is read-only")
            return GoalResponse.REJECT
        trajectory = goal_request.trajectory
        if len(trajectory.joint_names) != len(ARM_JOINTS) or set(trajectory.joint_names) != set(ARM_JOINTS):
            return GoalResponse.REJECT
        if len(trajectory.points) < 2 or len(trajectory.points) > 1000:
            return GoalResponse.REJECT
        if self._has_nondefault_tolerance(goal_request):
            self.get_logger().warning("Rejecting trajectory goal: non-default path/goal tolerances are unsupported")
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active_goal is not None or self._owner_faulted:
                return GoalResponse.REJECT
            self._active_goal = "pending"
        return GoalResponse.ACCEPT

    @staticmethod
    def _has_nondefault_tolerance(goal_request):
        for tolerance in (*goal_request.path_tolerance, *goal_request.goal_tolerance):
            if any(abs(float(value)) > 1e-12 for value in
                   (tolerance.position, tolerance.velocity, tolerance.acceleration)):
                return True
        duration = goal_request.goal_time_tolerance
        return duration.sec != 0 or duration.nanosec != 0

    def _cancel_request(self, goal_handle):
        with self._active_lock:
            if self._active_goal is None:
                return CancelResponse.REJECT
            if isinstance(self._active_goal, str) and self._active_goal in ("pending", "validating"):
                self._cancel_before_start = True
                return CancelResponse.ACCEPT
        try:
            self._send({"command": "cancel_hold"})
            return CancelResponse.ACCEPT
        except (BrokenPipeError, OSError, RuntimeError):
            return CancelResponse.REJECT

    def _result(self, code, message):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _feedback(self, goal_handle, trajectory, start_monotonic):
        state = self._read_state()
        if state is None:
            return False
        elapsed = max(0.0, time.monotonic() - start_monotonic)
        names = trajectory.joint_names
        actual_values = [math.radians(float(state["pose"][name])) for name in names]
        desired = state.get("trajectory_desired")
        if state.get("trajectory_goal_id") and isinstance(desired, dict):
            desired_values = [float(desired[name]) for name in names]
        else:
            desired_values = self._desired(trajectory.points, names, elapsed)
        feedback = FollowJointTrajectory.Feedback()
        feedback.header = trajectory.header
        feedback.joint_names = list(names)
        feedback.actual = JointTrajectoryPoint(positions=actual_values, time_from_start=_duration(elapsed))
        feedback.desired = JointTrajectoryPoint(positions=desired_values, time_from_start=_duration(elapsed))
        feedback.error = JointTrajectoryPoint(
            positions=[d - a for d, a in zip(desired_values, actual_values)],
            time_from_start=_duration(elapsed))
        goal_handle.publish_feedback(feedback)
        return True

    @staticmethod
    def _desired(points, names, elapsed):
        if not points:
            return [0.0] * len(names)
        point_times = [_seconds(point.time_from_start) for point in points]
        if elapsed <= point_times[0]:
            return list(points[0].positions)
        for index in range(len(points) - 1):
            left, right = points[index], points[index + 1]
            t0, t1 = point_times[index], point_times[index + 1]
            if elapsed <= t1:
                ratio = min(1.0, max(0.0, (elapsed - t0) / (t1 - t0)))
                return [a + (b - a) * ratio for a, b in zip(left.positions, right.positions)]
        return list(points[-1].positions)

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        with self._active_lock:
            if self._cancel_before_start:
                self._cancel_before_start = False
                self._active_goal = None
                goal_handle.canceled()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL,
                                    "cancelled before trajectory submission")
            self._active_goal = "validating"
        command_points = []
        names = list(trajectory.joint_names)
        for point in trajectory.points:
            item = {"time": _seconds(point.time_from_start), "positions": list(point.positions)}
            if point.velocities:
                item["velocities"] = list(point.velocities)
            if point.accelerations:
                item["accelerations"] = list(point.accelerations)
            command_points.append(item)
        goal_id = uuid.uuid4().hex
        artifact = {"schema": 1, "goal_id": goal_id, "status": "rejected_before_send",
                    "original_points": command_points, "joint_names": names}
        try:
            artifact = self._collision_check_path(trajectory, trajectory.points, goal_id)
            with self._active_lock:
                cancelled_before_send = self._cancel_before_start
                if cancelled_before_send:
                    self._cancel_before_start = False
                    self._active_goal = None
            if cancelled_before_send:
                artifact["status"] = "cancelled_before_send"
                self._write_artifact(artifact)
                goal_handle.canceled()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL,
                                    "cancelled before collision-checked trajectory submission")
            self._write_artifact(artifact)
        except _PreflightCancelled as error:
            artifact["status"] = "cancelled_before_send"
            artifact["preflight_error"] = str(error)
            try:
                self._write_artifact(artifact)
            except OSError as log_error:
                self.get_logger().error(f"Could not save cancelled trajectory artifact: {log_error}")
            with self._active_lock:
                self._active_goal = None
                self._cancel_before_start = False
            goal_handle.canceled()
            return self._result(FollowJointTrajectory.Result.SUCCESSFUL, str(error))
        except Exception as error:
            artifact = getattr(error, "artifact", artifact)
            if artifact.get("status") != "collision_rejected":
                artifact["status"] = "rejected_before_send"
            artifact["preflight_error"] = str(error)
            try:
                self._write_artifact(artifact)
            except OSError as log_error:
                self.get_logger().error(f"Could not save rejected trajectory artifact: {log_error}")
            with self._active_lock:
                self._active_goal = None
                self._cancel_before_start = False
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.INVALID_GOAL,
                                f"trajectory preflight rejected before bus command: {error}")
        try:
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            with self._active_lock:
                if self._cancel_before_start:
                    self._cancel_before_start = False
                    self._active_goal = None
                    artifact["status"] = "cancelled_before_send"
                    cancelled_before_send = True
                else:
                    cancelled_before_send = False
                    self._active_goal = goal_handle
                    self._send({"command": "execute_trajectory", "goal_id": goal_id,
                                "joint_names": names, "points": command_points,
                                "path_sha256": artifact["effective_path_sha256"]})
            if cancelled_before_send:
                artifact["status"] = "cancelled_before_send"
                self._write_artifact(artifact)
                goal_handle.canceled()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL,
                                    "cancelled before collision-checked trajectory submission")
        except (BrokenPipeError, OSError, RuntimeError) as error:
            with self._active_lock:
                self._active_goal = None
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.INVALID_GOAL, str(error))
        started = time.monotonic()
        last_feedback = 0.0
        result_payload = None
        stale_stop_sent = False
        while result_payload is None:
            if self._process.poll() is not None:
                break
            now = time.monotonic()
            state = self._read_state(allow_stale=True)
            state_age = (now - float(state["monotonic"])) if state else math.inf
            preflighting = (state and state.get("trajectory_goal_id") == goal_id and
                            state.get("phase") == "preflight")
            if state_age > float(self.get_parameter("state_freshness_s").value):
                if preflighting and now - started < 30.0:
                    pass  # no actuator write occurs until the pre-motion audit finishes
                elif not stale_stop_sent:
                    try:
                        self._send({"command": "stop"})
                    except (BrokenPipeError, OSError, RuntimeError):
                        pass
                    stale_stop_sent = True
                    self._owner_faulted = True
                if stale_stop_sent and now - started > 5.0:
                    break
            if now - last_feedback >= 0.05:
                if (not preflighting and state_age <= float(self.get_parameter("state_freshness_s").value)
                        and not self._feedback(goal_handle, trajectory, started)):
                    try:
                        self._send({"command": "stop"})
                    except (BrokenPipeError, OSError, RuntimeError):
                        pass
                    stale_stop_sent = True
                    self._owner_faulted = True
                    break
                last_feedback = now
            try:
                result_payload = self._response_queue.get(timeout=0.01)
            except queue.Empty:
                continue
        with self._active_lock:
            cancelled = goal_handle.is_cancel_requested or (result_payload and result_payload.get("cancelled"))
            self._active_goal = None
        if cancelled:
            artifact["status"] = "cancelled"
            artifact["result"] = result_payload
            self._write_artifact(artifact)
            goal_handle.canceled()
            return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "cancelled; measured pose held")
        if result_payload and result_payload.get("ok"):
            artifact["status"] = "succeeded"
            artifact["result"] = result_payload
            self._write_artifact(artifact)
            goal_handle.succeed()
            return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory completed and settled")
        message = "ArmSession terminated or returned an error"
        error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        if result_payload:
            message = result_payload.get("error", result_payload.get("error_type", message))
            if result_payload.get("error_type") == "ArmPlanPreflightError":
                error_code = FollowJointTrajectory.Result.INVALID_GOAL
        artifact["status"] = "failed"
        artifact["error"] = message
        artifact["result"] = result_payload
        self._write_artifact(artifact)
        goal_handle.abort()
        return self._result(error_code, message)

    def destroy_node(self):
        self._closed = True
        if self._process is not None and self._process.poll() is None:
            try:
                self._send({"command": "exit"})
                self._process.wait(timeout=5.0)
            except (BrokenPipeError, OSError, RuntimeError, subprocess.TimeoutExpired):
                self._process.terminate()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryBridge()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"trajectory_server: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
