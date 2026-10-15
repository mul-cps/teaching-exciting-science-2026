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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .leader_reader import JOINTS as TELEOP_JOINTS, matched_pose, validate_pose_packet

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _arm_session_command(station_python, config, state_file, log_dir, lock, idle_timeout_s):
    timeout = float(idle_timeout_s)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("owner_idle_timeout_s must be finite and positive")
    return [str(station_python), "-m", "workshop.station.arm_session",
            "--config", str(config), "--state-file", str(state_file),
            "--logdir", str(log_dir), "--lock", str(lock),
            "--persistent-idle", "--idle-timeout", str(timeout)]


def _owner_state_is_ready(state, *, arm_id, calibration_sha256, owner_started):
    if not isinstance(state, dict) or state.get("event") not in {"connected", "sample"}:
        return False
    try:
        monotonic = float(state["monotonic"])
    except (KeyError, TypeError, ValueError):
        return False
    torque = state.get("torque")
    return (math.isfinite(monotonic) and monotonic >= owner_started and
            state.get("arm_id") == arm_id and
            state.get("calibration_sha256") == calibration_sha256 and
            isinstance(torque, dict) and torque and
            all(value == 0 for value in torque.values()))


def _load_measured_home(config_data, station_root, arm_id, calibration_sha256):
    """Load the station's identity-bound measured home and shared pose tolerance."""
    try:
        arm = config_data["arm"]
        home_path = Path(arm["home_file"])
        if not home_path.is_absolute():
            home_path = Path(station_root) / home_path
        data = json.loads(home_path.read_text())
        pose = data["position"]
        tolerance = float(arm["tolerance"])
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise ValueError("station config must provide a readable measured home and arm tolerance") from error
    if (data.get("schema") != 1 or data.get("kind") != "measured-home" or
            data.get("units") != "body degrees; gripper percent" or
            data.get("arm_id") != arm_id or
            data.get("calibration_sha256") != calibration_sha256):
        raise ValueError("measured-home identity, schema, or units do not match station arm config")
    if (not isinstance(pose, dict) or set(pose) != set(TELEOP_JOINTS) or
            any(type(pose[joint]) not in (int, float) or not math.isfinite(pose[joint])
                for joint in TELEOP_JOINTS) or
            not 0 <= pose["gripper"] <= 100 or not math.isfinite(tolerance) or tolerance <= 0):
        raise ValueError("measured home requires finite six-joint values and a positive finite tolerance")
    return {joint: float(pose[joint]) for joint in TELEOP_JOINTS}, tolerance


def _measured_home_error(state, *, arm_id, calibration_sha256, home_pose, tolerance):
    """Return why a fresh owner snapshot is not at saved home, or None."""
    if not isinstance(state, dict):
        return "resident ArmSession state is unavailable"
    if state.get("arm_id") != arm_id or state.get("calibration_sha256") != calibration_sha256:
        return "resident ArmSession identity does not match the configured follower and calibration"
    pose = state.get("pose")
    if (not isinstance(pose, dict) or set(pose) != set(TELEOP_JOINTS) or
            any(type(pose[joint]) not in (int, float) or not math.isfinite(pose[joint])
                for joint in TELEOP_JOINTS)):
        return "resident ArmSession state has no valid measured six-joint pose"
    outside = {joint: abs(float(pose[joint]) - float(home_pose[joint]))
               for joint in TELEOP_JOINTS
               if abs(float(pose[joint]) - float(home_pose[joint])) > tolerance}
    if outside:
        joints = ", ".join(f"{joint} ({error:.2f} > {tolerance:g})"
                            for joint, error in outside.items())
        return (f"measured follower pose is outside saved-home tolerance: {joints}; "
                "call /follower/home or align the arm manually before enabling teleoperation")
    return None


def calibration_ready_raw_legs(start_raw, target_raw):
    """Create bounded linear raw-count sublegs for the observation-pose move."""
    joints = set(TELEOP_JOINTS)
    for label, pose in (("start", start_raw), ("target", target_raw)):
        if not isinstance(pose, dict) or set(pose) != joints:
            raise ValueError(f"calibration-ready {label} raw pose must contain exactly six joints")
        if any(type(value) is not int or not 0 <= value <= 4095 for value in pose.values()):
            raise ValueError(f"calibration-ready {label} raw values must be integers in 0..4095")
    count = max(1, max(math.ceil(abs(target_raw[joint] - start_raw[joint]) / 100)
                       for joint in TELEOP_JOINTS))
    if count > 100:
        raise ValueError("calibration-ready route exceeds ArmSession's 100-leg limit")
    legs = []
    previous = start_raw
    for index in range(1, count + 1):
        target = {
            joint: int(round(start_raw[joint] +
                             (target_raw[joint] - start_raw[joint]) * index / count))
            for joint in TELEOP_JOINTS
        }
        if any(abs(target[joint] - previous[joint]) > 100 for joint in TELEOP_JOINTS):
            raise ValueError("calibration-ready subleg exceeds 100 raw counts")
        legs.append({"label": f"calibration_ready_{index:03d}_of_{count:03d}",
                     "target_raw": target})
        previous = target
    return legs


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
        self.declare_parameter("owner_idle_timeout_s", 300.0)
        self.declare_parameter("action_name", "/follower/arm_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("state_validity_service", "/follower/check_state_validity")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("state_freshness_s", 0.25)
        self.declare_parameter("collision_check_timeout_s", 30.0)
        self.declare_parameter("max_jerk_rad_s3", 2.0)
        self.declare_parameter("calibration_endpoint_mode", False)
        self.declare_parameter("teleop_pose_topic", "/leader/teleop_pose")
        self.declare_parameter("teleop_enable_service", "/follower/teleop_enable")
        self.declare_parameter("teleop_pose_stale_s", 0.15)
        self.declare_parameter("teleop_owner_timeout_s", 2.0)
        self.declare_parameter("home_service", "/follower/home")
        self.declare_parameter("home_timeout_s", 90.0)
        self.declare_parameter("calibration_ready_service", "/follower/calibration_ready")
        self.declare_parameter("calibration_ready_pose_file", "")
        self.declare_parameter("calibration_ready_timeout_s", 90.0)
        self._active_lock = threading.Lock()
        self._active_goal = None
        self._cancel_before_start = False
        self._write_lock = threading.Lock()
        self._action_group = ReentrantCallbackGroup()
        self._response_queue = queue.Queue()
        self._teleop_condition = threading.Condition()
        self._teleop_acknowledgements = {}
        self._teleop_state = "disabled"
        self._teleop_session = None
        self._teleop_origin = None
        self._teleop_follower_start = None
        self._teleop_last_source_sequence = -1
        self._teleop_owner_sequence = -1
        self._teleop_inflight_sequence = None
        self._teleop_inflight = False
        self._teleop_stop_sent = False
        self._teleop_step_done = threading.Event()
        self._teleop_step_done.set()
        self._teleop_latest = None
        self._teleop_latest_lock = threading.Lock()
        self._closed = False
        self._process = None
        self._owner_started = 0.0
        self._owner_faulted = False
        self._expected_arm_id = None
        self._expected_calibration_sha256 = None
        self._saved_home_pose = None
        self._saved_home_tolerance = None
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
        self._teleop_service_group = MutuallyExclusiveCallbackGroup()
        self._teleop_pose_subscription = self.create_subscription(
            String, str(self.get_parameter("teleop_pose_topic").value),
            self._leader_pose_received, 1)
        self._teleop_service = self.create_service(
            SetBool, str(self.get_parameter("teleop_enable_service").value),
            self._set_teleop_enabled, callback_group=self._teleop_service_group)
        self._home_service = self.create_service(
            Trigger, str(self.get_parameter("home_service").value),
            self._home, callback_group=self._teleop_service_group)
        self._calibration_ready_service = self.create_service(
            Trigger, str(self.get_parameter("calibration_ready_service").value),
            self._calibration_ready, callback_group=self._teleop_service_group)
        self.create_timer(1.0 / 125.0, self._teleop_tick)
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
        config_path = Path(config)
        if not config_path.is_absolute():
            config_path = station_root / config_path
        try:
            config_data = json.loads(config_path.read_text())
            expected_arm_id = str(config_data["arm"]["id"])
            expected_calibration_sha256 = str(config_data["arm"]["calibration"]["sha256"])
        except (OSError, json.JSONDecodeError, KeyError, TypeError) as error:
            raise RuntimeError("station_config must provide arm id and calibration SHA256") from error
        if (not expected_arm_id or len(expected_calibration_sha256) != 64 or
                any(character not in "0123456789abcdef" for character in expected_calibration_sha256)):
            raise RuntimeError("station_config has invalid arm identity or calibration SHA256")
        self._saved_home_pose, self._saved_home_tolerance = _load_measured_home(
            config_data, station_root, expected_arm_id, expected_calibration_sha256)
        self._expected_arm_id = expected_arm_id
        self._expected_calibration_sha256 = expected_calibration_sha256
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
        command = _arm_session_command(
            station_python, config, self._state_file, logs, lock,
            self.get_parameter("owner_idle_timeout_s").value)
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
            if _owner_state_is_ready(state, arm_id=expected_arm_id,
                                     calibration_sha256=expected_calibration_sha256,
                                     owner_started=self._owner_started):
                self.get_logger().info("ArmSession connected; action server is ready")
                return
            time.sleep(0.05)
        self._process.terminate()
        raise RuntimeError("ArmSession state did not become fresh within 20 seconds")

    def _read_responses(self):
        for line in self._process.stdout:
            try:
                response = json.loads(line)
                if not self._route_teleop_response(response):
                    self._response_queue.put(response)
            except json.JSONDecodeError:
                self.get_logger().error("ArmSession emitted invalid JSON on stdout")

    def _route_teleop_response(self, response):
        command = response.get("command")
        state = response.get("state")
        session_id = response.get("session_id")
        if command not in {"teleop_start", "teleop_step", "teleop_stop", "teleop_watchdog"}:
            return False
        with self._active_lock:
            session_matches = bool(session_id) and session_id == self._teleop_session
            current_state = self._teleop_state
            inflight_matches = (self._teleop_inflight and
                                (type(response.get("sequence")) is not int or
                                 response.get("sequence") == self._teleop_inflight_sequence))

        # Responses for an expired or superseded session are still owner replies, but
        # they must not mutate the current teleop state or the action response queue.
        if not session_matches:
            return True
        if command == "teleop_watchdog":
            with self._active_lock:
                if self._teleop_session != session_id:
                    return True
                self._active_goal = None
                self._teleop_state = "disabled"
                self._teleop_session = None
                self._teleop_origin = None
                self._teleop_follower_start = None
                self._teleop_inflight = False
                self._teleop_inflight_sequence = None
                self._teleop_stop_sent = False
            self._teleop_step_done.set()
            self.get_logger().warning("Leader stream expired; ArmSession watchdog held measured pose")
            with self._teleop_condition:
                self._teleop_acknowledgements["teleop_watchdog"] = response
                self._teleop_acknowledgements["teleop_stop"] = {
                    "ok": True, "state": state,
                    "message": "ArmSession watchdog held measured pose"}
                self._teleop_condition.notify_all()
            return True
        if command == "teleop_step":
            if not inflight_matches or current_state not in {"active", "stopping"}:
                return True
            with self._active_lock:
                if not self._teleop_inflight or self._teleop_session != session_id:
                    return True
                self._teleop_inflight = False
                self._teleop_inflight_sequence = None
                if not response.get("ok") and self._teleop_state == "active":
                    self._teleop_state = "stopping"
                    self._active_goal = "teleop_stopping"
            self._teleop_step_done.set()
            if not response.get("ok"):
                self.get_logger().error(
                    f"ArmSession rejected teleop step: {response.get('error', 'unknown error')}")
            return True
        if command == "teleop_start":
            if current_state != "starting":
                return True
            key = "teleop_start"
        else:
            if current_state != "stopping":
                return True
            key = "teleop_stop"
            if response.get("ok"):
                self._clear_teleop()
        with self._teleop_condition:
            self._teleop_acknowledgements[key] = response
            self._teleop_condition.notify_all()
        return True

    def _clear_teleop(self):
        with self._active_lock:
            if self._active_goal in {"teleop", "teleop_starting", "teleop_stopping"}:
                self._active_goal = None
            self._teleop_state = "disabled"
            self._teleop_session = None
            self._teleop_origin = None
            self._teleop_follower_start = None
            self._teleop_inflight = False
            self._teleop_inflight_sequence = None
            self._teleop_stop_sent = False

    def _wait_teleop_ack(self, key):
        deadline = time.monotonic() + float(self.get_parameter("teleop_owner_timeout_s").value)
        with self._teleop_condition:
            while key not in self._teleop_acknowledgements:
                if self._process is None or self._process.poll() is not None:
                    raise RuntimeError("ArmSession stopped before acknowledging teleop command")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"ArmSession did not acknowledge {key} before timeout")
                self._teleop_condition.wait(min(remaining, 0.05))
            return self._teleop_acknowledgements.pop(key)

    def _leader_pose_received(self, message):
        try:
            packet = json.loads(message.data)
            pose = validate_pose_packet(
                packet, time.monotonic(), float(self.get_parameter("teleop_pose_stale_s").value))
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Ignoring invalid or stale leader pose: {error}")
            return
        with self._teleop_latest_lock:
            current = self._teleop_latest
        if current is not None and packet["sequence"] <= current["sequence"]:
            with self._active_lock:
                inactive = self._teleop_state == "disabled"
            if not inactive:
                return
        with self._teleop_latest_lock:
            self._teleop_latest = {"stamp_s": float(packet["stamp_s"]),
                                   "sequence": int(packet["sequence"]), "pose": pose}

    def _set_teleop_enabled(self, request, response):
        if self._external_state_only or self._process is None:
            response.success = False
            response.message = "teleoperation requires the resident hardware owner"
            return response
        if request.data:
            with self._active_lock:
                if self._teleop_state == "active":
                    response.success = True
                    response.message = "teleoperation already enabled"
                    return response
                if self._active_goal is not None or self._owner_faulted:
                    response.success = False
                    response.message = "a trajectory or another owner operation is active"
                    return response
                self._teleop_state = "starting"
                self._active_goal = "teleop_starting"
            with self._teleop_latest_lock:
                latest = dict(self._teleop_latest) if self._teleop_latest else None
            if (latest is None or time.monotonic() - latest["stamp_s"] >
                    float(self.get_parameter("teleop_pose_stale_s").value)):
                self._clear_teleop()
                response.success = False
                response.message = "no fresh calibrated leader pose is available"
                return response
            follower_state = self._read_state()
            if follower_state is None or not isinstance(follower_state.get("pose"), dict):
                self._clear_teleop()
                response.success = False
                response.message = "follower state is not fresh"
                return response
            if self._saved_home_pose is None or self._expected_arm_id is None:
                self._clear_teleop()
                response.success = False
                response.message = "saved-home readiness check is unavailable; teleoperation remains disabled"
                return response
            home_error = _measured_home_error(
                follower_state, arm_id=self._expected_arm_id,
                calibration_sha256=self._expected_calibration_sha256,
                home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance)
            if home_error:
                self._clear_teleop()
                response.success = False
                response.message = home_error
                return response
            torque = follower_state.get("torque")
            if not isinstance(torque, dict) or set(torque) != set(TELEOP_JOINTS):
                torque_values = []
            else:
                torque_values = list(torque.values())
            if not torque_values or any(value not in (0, 1) for value in torque_values) or \
                    len(set(torque_values)) != 1:
                self._clear_teleop()
                response.success = False
                response.message = "follower torque state must be fresh and consistently verified on or off"
                return response
            session_id = uuid.uuid4().hex
            with self._active_lock:
                self._teleop_session = session_id
            with self._teleop_condition:
                self._teleop_acknowledgements.pop("teleop_start", None)
            command_sent = False
            acknowledged = False
            try:
                self._send({"command": "teleop_start", "session_id": session_id})
                command_sent = True
                acknowledgement = self._wait_teleop_ack("teleop_start")
                acknowledged = True
                if not acknowledgement.get("ok"):
                    raise RuntimeError(acknowledgement.get("error", "owner rejected teleop start"))
                state = acknowledgement.get("state")
                baseline = state.get("pose") if isinstance(state, dict) else None
                if not isinstance(baseline, dict) or set(baseline) != set(TELEOP_JOINTS):
                    raise RuntimeError("owner start acknowledgement did not include a full follower pose")
                if any(type(baseline[joint]) not in (int, float) or not math.isfinite(baseline[joint])
                       for joint in TELEOP_JOINTS):
                    raise RuntimeError("owner start acknowledgement contained an invalid follower pose")
                home_error = _measured_home_error(
                    state, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance)
                if home_error:
                    raise RuntimeError(home_error)
                with self._teleop_latest_lock:
                    start_reference = dict(self._teleop_latest) if self._teleop_latest else None
                if (start_reference is None or time.monotonic() - start_reference["stamp_s"] >
                        float(self.get_parameter("teleop_pose_stale_s").value)):
                    raise RuntimeError("leader pose became stale while waiting for owner start acknowledgement")
                with self._active_lock:
                    self._teleop_session = session_id
                    self._teleop_origin = start_reference["pose"]
                    self._teleop_follower_start = {joint: float(baseline[joint]) for joint in TELEOP_JOINTS}
                    self._teleop_last_source_sequence = start_reference["sequence"]
                    self._teleop_owner_sequence = -1
                    self._teleop_inflight = False
                    self._teleop_state = "active"
                    self._teleop_stop_sent = False
                    self._active_goal = "teleop"
                response.success = True
                response.message = "teleoperation enabled; ArmSession owner acknowledged"
                return response
            except (BrokenPipeError, OSError, RuntimeError, TimeoutError, ValueError) as error:
                if command_sent and (not acknowledged or acknowledgement.get("ok")):
                    # Start may have reached the resident even if its acknowledgement was
                    # delayed. Queue a matching stop behind it and keep action admission shut.
                    with self._active_lock:
                        self._teleop_session = session_id
                        self._teleop_state = "stopping"
                        self._active_goal = "teleop_stopping"
                        self._teleop_stop_sent = True
                    with self._teleop_condition:
                        self._teleop_acknowledgements.pop("teleop_stop", None)
                    try:
                        self._send({"command": "teleop_stop", "session_id": session_id})
                        stop_ack = self._wait_teleop_ack("teleop_stop")
                        if stop_ack.get("ok"):
                            self._clear_teleop()
                    except (BrokenPipeError, OSError, RuntimeError, TimeoutError):
                        self._owner_faulted = True
                else:
                    self._clear_teleop()
                response.success = False
                response.message = str(error)
                return response
        with self._active_lock:
            state = self._teleop_state
            session_id = self._teleop_session
            if state == "disabled":
                response.success = True
                response.message = "teleoperation already disabled"
                return response
            if state == "starting" or not session_id:
                response.success = False
                response.message = "teleoperation is still starting"
                return response
            should_send_stop = not self._teleop_stop_sent
            self._teleop_state = "stopping"
            self._active_goal = "teleop_stopping"
            if should_send_stop:
                self._teleop_stop_sent = True
        if should_send_stop:
            with self._teleop_condition:
                self._teleop_acknowledgements.pop("teleop_stop", None)
        try:
            if should_send_stop and not self._teleop_step_done.wait(
                    float(self.get_parameter("teleop_owner_timeout_s").value)):
                response.success = False
                response.message = "timed out waiting for the outstanding owner step; watchdog will hold"
                return response
            if should_send_stop:
                self._send({"command": "teleop_stop", "session_id": session_id})
            acknowledgement = self._wait_teleop_ack("teleop_stop")
            if not acknowledgement.get("ok"):
                raise RuntimeError(acknowledgement.get("error", "owner rejected teleop stop"))
            self._clear_teleop()
            response.success = True
            response.message = "teleoperation stopped; measured pose held"
        except (BrokenPipeError, OSError, RuntimeError, TimeoutError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _home(self, _request, response):
        """Run the existing measured-home operation through the resident owner."""
        process = self._process
        state = self._read_state()
        if (self._external_state_only or process is None or process.poll() is not None or
                self._owner_faulted):
            response.success = False
            response.message = "resident ArmSession owner is unavailable or faulted"
            return response
        if state is None:
            response.success = False
            response.message = "resident ArmSession state is not fresh"
            return response
        with self._active_lock:
            if self._active_goal is not None:
                response.success = False
                response.message = "a trajectory, teleoperation session, or owner operation is active"
                return response
            self._active_goal = "home"

        timeout_s = float(self.get_parameter("home_timeout_s").value)
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 90.0:
            response.success = False
            response.message = "home_timeout_s must be in (0, 90] seconds"
            with self._active_lock:
                if self._active_goal == "home":
                    self._active_goal = None
            return response
        deadline = time.monotonic() + timeout_s
        try:
            # With the exclusive service reservation, no other bridge command can
            # be in flight. Discard any stale unsolicited reply before sending home.
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            self._send({"command": "home"})
            while True:
                if process.poll() is not None:
                    raise RuntimeError("ArmSession owner stopped before home completed")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("ArmSession home did not complete within timeout")
                try:
                    result = self._response_queue.get(timeout=min(0.05, remaining))
                    break
                except queue.Empty:
                    continue
            if not isinstance(result, dict):
                raise RuntimeError("ArmSession returned a malformed home acknowledgement")
            home_state = result.get("state")
            if not result.get("ok"):
                raise RuntimeError(result.get("error", result.get("error_type", "ArmSession rejected home")))
            if not isinstance(home_state, dict) or home_state.get("event") != "home_complete":
                raise RuntimeError("ArmSession home acknowledgement did not confirm home_complete")
            response.success = True
            response.message = "measured home completed"
        except (BrokenPipeError, OSError, RuntimeError, TimeoutError) as error:
            response.success = False
            response.message = str(error)
        finally:
            with self._active_lock:
                if self._active_goal == "home":
                    self._active_goal = None
        return response

    def _calibration_ready(self, _request, response):
        """Move from the measured current pose to an explicit observation pose."""
        if not bool(self.get_parameter("calibration_endpoint_mode").value):
            response.success = False
            response.message = "calibration-ready motion requires calibration_endpoint_mode:=true"
            return response
        process = self._process
        if (self._external_state_only or process is None or process.poll() is not None or
                self._owner_faulted):
            response.success = False
            response.message = "resident ArmSession owner is unavailable or faulted"
            return response
        with self._active_lock:
            if self._active_goal is not None:
                response.success = False
                response.message = "a trajectory, teleoperation session, or owner operation is active"
                return response
            self._active_goal = "calibration_ready"

        try:
            pose_file_value = str(self.get_parameter("calibration_ready_pose_file").value).strip()
            if not pose_file_value:
                raise ValueError("calibration_ready_pose_file must name the measured observation pose JSON")
            pose_path = Path(pose_file_value)
            if not pose_path.is_absolute():
                raise ValueError("calibration_ready_pose_file must be an absolute path")
            try:
                observation = json.loads(pose_path.read_text())
            except (OSError, json.JSONDecodeError) as error:
                raise ValueError(f"could not read calibration-ready pose file: {error}") from error
            if not isinstance(observation, dict):
                raise ValueError("calibration-ready pose file must contain a JSON object")
            state = self._read_state()
            if state is None:
                raise RuntimeError("resident ArmSession state is not fresh")
            arm_id = state.get("arm_id")
            calibration_sha256 = state.get("calibration_sha256")
            if (not isinstance(arm_id, str) or not arm_id or
                    not isinstance(calibration_sha256, str) or len(calibration_sha256) != 64 or
                    any(character not in "0123456789abcdef" for character in calibration_sha256)):
                raise RuntimeError("fresh resident state has invalid arm or calibration identity")
            if (observation.get("arm_id") != arm_id or
                    observation.get("calibration_sha256") != calibration_sha256):
                raise ValueError("calibration-ready pose identity does not match fresh resident state")
            legs = calibration_ready_raw_legs(state.get("raw"), observation.get("raw"))

            timeout_s = float(self.get_parameter("calibration_ready_timeout_s").value)
            if not math.isfinite(timeout_s) or not 0 < timeout_s <= 90.0:
                raise ValueError("calibration_ready_timeout_s must be in (0, 90] seconds")
            deadline = time.monotonic() + timeout_s
            # This reservation excludes other bridge operations, so old queue
            # entries cannot be acknowledgements for this single owner request.
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            self._send({"command": "move_plan", "legs": legs})
            while True:
                if process.poll() is not None:
                    raise RuntimeError("ArmSession owner stopped before calibration-ready move completed")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("calibration-ready move did not complete within timeout")
                try:
                    result = self._response_queue.get(timeout=min(0.05, remaining))
                    break
                except queue.Empty:
                    continue
            if not isinstance(result, dict):
                raise RuntimeError("ArmSession returned a malformed calibration-ready acknowledgement")
            if not result.get("ok"):
                raise RuntimeError(result.get("error", result.get("error_type",
                                                                 "ArmSession rejected calibration-ready move")))
            final_state = result.get("state")
            if not isinstance(final_state, dict) or final_state.get("event") != "plan_complete":
                raise RuntimeError("ArmSession acknowledgement did not confirm plan_complete")
            response.success = True
            response.message = (
                f"measured observation pose reached in {len(legs)} raw sublegs; "
                "not MoveIt- or collision-qualified")
        except (OSError, ValueError, RuntimeError, TimeoutError, TypeError) as error:
            response.success = False
            response.message = str(error)
        finally:
            with self._active_lock:
                if self._active_goal == "calibration_ready":
                    self._active_goal = None
        return response

    def _teleop_tick(self):
        with self._active_lock:
            if self._teleop_state != "active" or self._teleop_inflight:
                return
            session_id = self._teleop_session
            origin = self._teleop_origin
            follower_start = self._teleop_follower_start
        with self._teleop_latest_lock:
            latest = dict(self._teleop_latest) if self._teleop_latest else None
        if latest is None:
            return
        age = time.monotonic() - latest["stamp_s"]
        if age > float(self.get_parameter("teleop_pose_stale_s").value):
            with self._active_lock:
                if self._teleop_state != "active":
                    return
                self._teleop_state = "stopping"
                self._active_goal = "teleop_stopping"
                self._teleop_stop_sent = True
                self._teleop_inflight = True
            try:
                self._send({"command": "teleop_stop", "session_id": session_id})
            except (BrokenPipeError, OSError, RuntimeError) as error:
                self.get_logger().error(f"Could not stop stale teleoperation session: {error}")
            return
        if latest["sequence"] <= self._teleop_last_source_sequence:
            return
        mapped = matched_pose(latest["pose"], origin, follower_start)
        # ArmSession clamps body targets to its exact calibrated raw bounds. Its gripper
        # input is normalized percent, so clamp that station-unit target here.
        mapped["gripper"] = max(0.0, min(100.0, mapped["gripper"]))
        sequence = self._teleop_owner_sequence + 1
        with self._active_lock:
            if self._teleop_state != "active" or self._teleop_inflight:
                return
            self._teleop_inflight = True
            self._teleop_inflight_sequence = sequence
            self._teleop_owner_sequence = sequence
            self._teleop_last_source_sequence = latest["sequence"]
            self._teleop_step_done.clear()
        try:
            self._send({"command": "teleop_step", "session_id": session_id,
                        "sequence": sequence, "pose": mapped})
        except (BrokenPipeError, OSError, RuntimeError) as error:
            with self._active_lock:
                self._teleop_inflight = False
                self._teleop_inflight_sequence = None
                self._teleop_state = "stopping"
                self._active_goal = "teleop_stopping"
            self._teleop_step_done.set()
            self.get_logger().error(f"Could not send teleop step: {error}")

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
                    command = {"command": "execute_trajectory", "goal_id": goal_id,
                               "joint_names": names, "points": command_points,
                               "path_sha256": artifact["effective_path_sha256"]}
                    if bool(self.get_parameter("calibration_endpoint_mode").value):
                        command["calibration_endpoint"] = True
                    self._send(command)
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
        if result_payload and result_payload.get("status") == "calibration_measured_endpoint_held":
            artifact["status"] = "calibration_measured_endpoint_held"
            artifact["target_reached"] = False
            artifact["result"] = result_payload
            self._write_artifact(artifact)
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                "calibration_measured_endpoint_held: measured endpoint held; plan target not reached")
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
