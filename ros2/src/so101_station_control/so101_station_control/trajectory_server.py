"""FollowJointTrajectory adapter that delegates all bus access to ArmSession."""
from __future__ import annotations

import json
import functools
import importlib.util
import math
import os
from pathlib import Path
import queue
import select
import subprocess
import sys
import threading
import time
import uuid

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from so101_station_workflow.srv import CommissionGripperStep
from trajectory_msgs.msg import JointTrajectoryPoint

from .leader_reader import JOINTS as TELEOP_JOINTS, matched_pose, validate_pose_packet
from .gripper_action import gap_to_percent, percent_to_gap, validate_jaw_calibration

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _arm_session_command(station_python, config, state_file, log_dir, lock, idle_timeout_s,
                         recover_sag=False):
    timeout = float(idle_timeout_s)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("owner_idle_timeout_s must be finite and positive")
    command = [str(station_python), "-m", "workshop.station.arm_session",
            "--config", str(config), "--state-file", str(state_file),
            "--logdir", str(log_dir), "--lock", str(lock),
            "--persistent-idle", "--idle-timeout", str(timeout)]
    if recover_sag:
        command.append("--recover-sag")
    return command


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


def _control_callback(callback):
    """Keep the stop barrier until every previously admitted callback exits."""
    @functools.wraps(callback)
    def tracked(self, *args, **kwargs):
        condition = getattr(self, "_stop_condition", None)
        if condition is None:  # lightweight offline harnesses
            return callback(self, *args, **kwargs)
        with condition:
            if self._stop_latched:
                if callback.__name__ in {"_goal_request", "_gripper_goal_request"}:
                    return GoalResponse.REJECT
                if callback.__name__ in {"_home", "_commission_gripper_step", "_calibration_ready", "_set_teleop_enabled"}:
                    response = args[1]
                    response.success, response.message = False, "owner stop remains latched"
                    return response
                if callback.__name__ == "_teleop_tick":
                    return None
                # Accepted action goals must reach a terminal state even if
                # their execute callback had not started when stop arrived.
                with self._active_lock:
                    self._active_goal = None
                args[0].abort()
                if callback.__name__ == "_execute_gripper":
                    return GripperCommand.Result()
                return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                    "owner stopped before trajectory submission; torque disabled")
            self._control_callbacks += 1
        try:
            return callback(self, *args, **kwargs)
        finally:
            with condition:
                self._control_callbacks -= 1
                condition.notify_all()
    return tracked


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
        self.declare_parameter("recover_sag", False)
        self.declare_parameter("action_name", "/follower/arm_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action_name", "/follower/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_jaw_calibration", "")
        self.declare_parameter("state_validity_service", "/follower/check_state_validity")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("state_freshness_s", 0.25)
        self.declare_parameter("collision_check_timeout_s", 30.0)
        self.declare_parameter("max_jerk_rad_s3", 2.0)
        self.declare_parameter("calibration_endpoint_mode", False)
        self.declare_parameter("teleop_pose_topic", "/leader/teleop_pose")
        self.declare_parameter("teleop_status_topic", "/follower/teleop_status")
        self.declare_parameter("teleop_enable_service", "/follower/teleop_enable")
        self.declare_parameter("teleop_pose_stale_s", 0.15)
        self.declare_parameter("teleop_owner_timeout_s", 2.0)
        self.declare_parameter("stop_service", "/follower/stop")
        self.declare_parameter("stop_timeout_s", 5.0)
        self.declare_parameter("home_service", "/follower/home")
        self.declare_parameter("home_timeout_s", 90.0)
        self.declare_parameter("calibration_ready_service", "/follower/calibration_ready")
        self.declare_parameter("calibration_ready_pose_file", "")
        self.declare_parameter("calibration_ready_timeout_s", 90.0)
        self._active_lock = threading.Lock()
        self._active_goal = None
        self._cancel_before_start = False
        self._write_lock = threading.Lock()
        self._stop_condition = threading.Condition()
        self._stop_lock = threading.Lock()
        self._stop_latched = False
        self._writer_faulted = False
        self._stop_pending = None
        self._stop_ack = None
        self._control_callbacks = 0
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
        self._teleop_stop_reason = None
        self._teleop_step_done = threading.Event()
        self._teleop_step_done.set()
        self._teleop_latest = None
        self._teleop_latest_lock = threading.Lock()
        self._teleop_last_receive_age_ms = None
        self._teleop_last_publish_delay_ms = None
        self._teleop_last_delivery_delay_ms = None
        self._teleop_stale_packet_count = 0
        self._teleop_sequence_gap_total = 0
        self._closed = False
        self._process = None
        self._owner_started = 0.0
        self._owner_faulted = False
        self._expected_arm_id = None
        self._expected_calibration_sha256 = None
        self._saved_home_pose = None
        self._saved_home_tolerance = None
        self._home_completed_owner_started = None
        self._state_file = self._resolve_state_file()
        self._gripper_jaw = None
        self._gripper_raw_limits = None
        self._gripper_goal_cancelled = False
        self._gripper_command_submitted = False
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
        self._gripper_action = ActionServer(
            self, GripperCommand,
            str(self.get_parameter("gripper_action_name").value),
            execute_callback=self._execute_gripper,
            goal_callback=self._gripper_goal_request,
            cancel_callback=self._gripper_cancel_request,
            callback_group=self._action_group)
        self._joint_state_pub = self.create_publisher(JointState, "/follower/joint_states", 10)
        self.create_timer(0.05, self._publish_joint_state)
        status_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                                reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._teleop_status_pub = self.create_publisher(
            String, str(self.get_parameter("teleop_status_topic").value), status_qos)
        self._publish_teleop_status("disabled", "bridge_startup")
        # Control traffic must not queue behind the default group, which also
        # services the reliable follower joint-state visualization publisher.
        self._teleop_control_group = MutuallyExclusiveCallbackGroup()
        self._teleop_service_group = MutuallyExclusiveCallbackGroup()
        self._teleop_pose_subscription = self.create_subscription(
            String, str(self.get_parameter("teleop_pose_topic").value),
            self._leader_pose_received,
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT),
            callback_group=self._teleop_control_group)
        self._teleop_service = self.create_service(
            SetBool, str(self.get_parameter("teleop_enable_service").value),
            self._set_teleop_enabled, callback_group=self._teleop_service_group)
        self._stop_group = MutuallyExclusiveCallbackGroup()
        self._stop_service = self.create_service(
            Trigger, str(self.get_parameter("stop_service").value),
            self._stop, callback_group=self._stop_group)
        self._home_service = self.create_service(
            Trigger, str(self.get_parameter("home_service").value),
            self._home, callback_group=self._teleop_service_group)
        self._commission_gripper_service = self.create_service(
            CommissionGripperStep, "/follower/commission_gripper_step",
            self._commission_gripper_step, callback_group=self._teleop_service_group)
        self._sorting_ready_service = self.create_service(
            Trigger, "/follower/sorting_ready", self._sorting_ready,
            callback_group=self._teleop_service_group)
        self._calibration_ready_service = self.create_service(
            Trigger, str(self.get_parameter("calibration_ready_service").value),
            self._calibration_ready, callback_group=self._teleop_service_group)
        self._teleop_tick_timer = self.create_timer(
            1.0 / 125.0, self._teleop_tick,
            callback_group=self._teleop_control_group)
        self.create_timer(1.0, self._publish_teleop_monitor)
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
        calibration_path = Path(config_data["arm"]["calibration"]["path"])
        if not calibration_path.is_absolute():
            calibration_path = station_root / calibration_path
        try:
            calibration_bytes = calibration_path.read_bytes()
            if __import__("hashlib").sha256(calibration_bytes).hexdigest() != expected_calibration_sha256:
                raise ValueError("station calibration file SHA256 does not match configured identity")
            raw_calibration = json.loads(calibration_bytes)
            gripper_cal = raw_calibration["gripper"]
            low, high = int(gripper_cal["range_min"]), int(gripper_cal["range_max"])
            if not 0 <= low < high <= 4095:
                raise ValueError("calibrated gripper raw bounds are invalid")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            raise RuntimeError("cannot load identity-matched gripper raw calibration") from error
        self._gripper_raw_limits = (low, high)
        jaw_path_value = str(self.get_parameter("gripper_jaw_calibration").value).strip()
        if jaw_path_value:
            try:
                self._gripper_jaw = validate_jaw_calibration(json.loads(Path(jaw_path_value).read_text()))
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                raise RuntimeError("configured jaw-gap calibration is invalid") from error
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
            self.get_parameter("owner_idle_timeout_s").value,
            self.get_parameter("recover_sag").value)
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
                if self._route_stop_response(response):
                    continue
                if not self._route_teleop_response(response):
                    self._response_queue.put(response)
            except json.JSONDecodeError:
                self.get_logger().error("ArmSession emitted invalid JSON on stdout")

    def _route_stop_response(self, response):
        if not response.get("stopped"):
            return False
        with self._stop_condition:
            if (response.get("request_id") == self._stop_pending and
                    self._stop_pending is not None):
                self._stop_ack = response
                self._stop_condition.notify_all()
        # The active worker still needs to terminate, but must never interpret
        # an owner stop as successful trajectory/plan completion.
        self._response_queue.put({**response, "ok": False, "cancelled": True,
                                  "error": "owner stopped; torque disabled"})
        return True

    def _write_owner_command(self, command, deadline):
        """Write one JSON line in atomic pipe chunks under the writer lock.

        A timeout before any bytes leaves framing intact. Once a line has
        started, any failure poisons the stream: no later command can complete
        that partial JSON and accidentally replay movement.
        """
        if getattr(self, "_writer_faulted", False):
            raise RuntimeError("previous owner write failed; command stream integrity is unknown")
        pending = (json.dumps(command, allow_nan=False) + "\n").encode("utf-8")
        if len(pending) > 1_048_576:
            raise RuntimeError("owner JSON command exceeds 1 MiB")
        stream = self._process.stdin
        if not hasattr(stream, "fileno"):
            raise RuntimeError("owner commands require a writable stdin file descriptor")
        descriptor = stream.fileno()
        chunk_size = os.fpathconf(descriptor, "PC_PIPE_BUF")
        if chunk_size <= 0:
            raise RuntimeError("owner command pipe has no atomic write bound")
        blocking = os.get_blocking(descriptor)
        offset = 0
        os.set_blocking(descriptor, False)
        try:
            while offset < len(pending):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("owner command pipe timed out")
                chunk = pending[offset:offset + chunk_size]
                try:
                    written = os.write(descriptor, chunk)
                    if written != len(chunk):
                        self._writer_faulted = True
                        raise RuntimeError("partial owner command write; command stream integrity is unknown")
                    offset += written
                except BlockingIOError:
                    select.select([], [descriptor], [], remaining)
        except BaseException:
            if offset:
                self._writer_faulted = True
            raise
        finally:
            os.set_blocking(descriptor, blocking)

    def _write_stop(self, command, deadline):
        self._write_owner_command(command, deadline)

    def _stop(self, _request, response):
        """Disarm the owner; success requires correlated fresh torque readback."""
        response.success = False
        timeout = float(self.get_parameter("stop_timeout_s").value)
        if not math.isfinite(timeout) or not 0 < timeout <= 10:
            response.message = "stop_timeout_s must be in (0, 10] seconds"
            return response
        deadline = time.monotonic() + timeout
        if not self._stop_lock.acquire(blocking=False):
            response.message = "another stop request is pending"
            return response
        try:
            if self._external_state_only or self._process is None or self._process.poll() is not None:
                raise RuntimeError("resident ArmSession owner is unavailable or read only")
            request_id = uuid.uuid4().hex
            # This lock is also the final admission check for every owner write.
            # Release it before taking _active_lock (execute uses the reverse order).
            self._stop_latched = True
            self._owner_faulted = True
            if not self._write_lock.acquire(timeout=max(0., deadline - time.monotonic())):
                raise TimeoutError("owner command writer did not quiesce; stop remains latched")
            try:
                with self._stop_condition:
                    self._stop_pending = request_id
                    self._stop_ack = None
                requested = time.monotonic()
                self._write_stop({"command": "stop", "request_id": request_id}, deadline)
            finally:
                self._write_lock.release()
            with self._active_lock:
                self._cancel_before_start = True
                self._gripper_goal_cancelled = True
                self._home_completed_owner_started = None
            self._clear_teleop()
            self._teleop_step_done.set()
            with self._teleop_condition:
                self._teleop_condition.notify_all()
            with self._stop_condition:
                while self._stop_ack is None or self._control_callbacks:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("stop acknowledgement or callback quiescence timed out; stop remains latched")
                    if self._process.poll() is not None:
                        raise RuntimeError("owner exited before stop confirmation")
                    self._stop_condition.wait(min(.02, remaining))
                ack = self._stop_ack
            with self._active_lock:
                if self._active_goal is not None:
                    raise RuntimeError("pending owner operation has not quiesced; stop remains latched")
            state = ack.get("state", ack.get("final_state"))
            if not ack.get("ok") or ack.get("stopped") is not True or not isinstance(state, dict):
                raise RuntimeError("owner did not confirm stop")
            stamp = state.get("monotonic")
            raw, torque = state.get("raw"), state.get("torque")
            if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                    not requested <= stamp <= time.monotonic() or
                    time.monotonic() - stamp > float(self.get_parameter("state_freshness_s").value) or
                    state.get("arm_id") != self._expected_arm_id or
                    state.get("calibration_sha256") != self._expected_calibration_sha256 or
                    not isinstance(raw, dict) or set(raw) != set(TELEOP_JOINTS) or
                    any(type(v) is not int or not 0 <= v <= 4095 for v in raw.values()) or
                    not isinstance(torque, dict) or set(torque) != set(TELEOP_JOINTS) or
                    any(type(v) is not int or v != 0 for v in torque.values()) or
                    self._process.poll() is not None):
                raise RuntimeError("stop lacks fresh identity-bound six-joint torque-off readback")
            with self._write_lock:
                self._stop_latched = False
                self._owner_faulted = False
                self._cancel_before_start = False
            response.success = True
            response.message = "owner stopped; all six servos verified torque off (disarmed)"
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as error:
            response.message = str(error)
        finally:
            with self._stop_condition:
                self._stop_pending = None
            self._stop_lock.release()
        return response

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
                self._teleop_stop_reason = None
            self._teleop_step_done.set()
            self.get_logger().warning("Leader stream expired; ArmSession watchdog held measured pose")
            self._publish_teleop_status("disabled", "arm_session_watchdog_hold",
                                        session_id=session_id)
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
                    self._teleop_stop_reason = "owner_rejected_step"
                    self._publish_teleop_status("stopping", self._teleop_stop_reason,
                                                error=response.get("error", "unknown"))
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
        reason = self._teleop_stop_reason or "teleoperation_disabled"
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
            self._teleop_stop_reason = None
        self._publish_teleop_status("disabled", reason)

    def _publish_teleop_status(self, state, reason, **details):
        status = {"schema": 1, "stamp_s": time.monotonic(),
                  "state": str(state), "reason": str(reason)}
        status.update(details)
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self._teleop_status_pub.publish(message)

    def _publish_teleop_monitor(self):
        with self._active_lock:
            if self._teleop_state != "active":
                return
            session_id = self._teleop_session
        with self._teleop_latest_lock:
            latest = dict(self._teleop_latest) if self._teleop_latest else None
            receive_age_ms = self._teleop_last_receive_age_ms
            publish_delay_ms = self._teleop_last_publish_delay_ms
            delivery_delay_ms = self._teleop_last_delivery_delay_ms
            stale_packet_count = self._teleop_stale_packet_count
            sequence_gap_total = self._teleop_sequence_gap_total
        source_age_ms = (max(0.0, time.monotonic() - latest["stamp_s"]) * 1000.0
                         if latest is not None else None)
        self._publish_teleop_status(
            "active", "stream_monitor", session_id=session_id,
            source_sequence=latest["sequence"] if latest is not None else None,
            source_age_ms=round(source_age_ms, 1) if source_age_ms is not None else None,
            last_receive_age_ms=receive_age_ms,
            last_source_to_publish_ms=publish_delay_ms,
            last_publish_to_callback_ms=delivery_delay_ms,
            stale_packet_count=stale_packet_count,
            source_sequence_gap_total=sequence_gap_total)

    def _wait_teleop_ack(self, key):
        deadline = time.monotonic() + float(self.get_parameter("teleop_owner_timeout_s").value)
        with self._teleop_condition:
            while key not in self._teleop_acknowledgements:
                if getattr(self, "_stop_latched", False):
                    return {"ok": False, "error": "owner stop requested; torque disabled"}
                if self._process is None or self._process.poll() is not None:
                    raise RuntimeError("ArmSession stopped before acknowledging teleop command")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"ArmSession did not acknowledge {key} before timeout")
                self._teleop_condition.wait(min(remaining, 0.05))
            return self._teleop_acknowledgements.pop(key)

    def _leader_pose_received(self, message):
        callback_stamp_s = time.monotonic()
        try:
            packet = json.loads(message.data)
            source_stamp_s = float(packet["stamp_s"])
            publish_stamp_s = float(packet.get("publish_stamp_s", source_stamp_s))
            receive_age_ms = round((callback_stamp_s - source_stamp_s) * 1000.0, 1)
            publish_delay_ms = round((publish_stamp_s - source_stamp_s) * 1000.0, 1)
            delivery_delay_ms = round((callback_stamp_s - publish_stamp_s) * 1000.0, 1)
            if not all(math.isfinite(value) for value in
                       (source_stamp_s, publish_stamp_s, receive_age_ms,
                        publish_delay_ms, delivery_delay_ms)):
                raise ValueError("non-finite leader diagnostic timestamp")
            with self._teleop_latest_lock:
                self._teleop_last_receive_age_ms = receive_age_ms
                self._teleop_last_publish_delay_ms = publish_delay_ms
                self._teleop_last_delivery_delay_ms = delivery_delay_ms
                if receive_age_ms > float(self.get_parameter("teleop_pose_stale_s").value) * 1000.0:
                    self._teleop_stale_packet_count += 1
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
            if current is not None:
                self._teleop_sequence_gap_total += max(
                    0, int(packet["sequence"]) - int(current["sequence"]) - 1)
            self._teleop_latest = {"stamp_s": float(packet["stamp_s"]),
                                   "sequence": int(packet["sequence"]), "pose": pose}

    @_control_callback
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
                    self._teleop_stop_reason = None
                    self._active_goal = "teleop"
                self._publish_teleop_status("active", "owner_acknowledged",
                                            session_id=session_id,
                                            source_sequence=start_reference["sequence"])
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
                self._teleop_stop_reason = "operator_disabled"
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

    @_control_callback
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
            self._home_completed_owner_started = self._owner_started
        except (BrokenPipeError, OSError, RuntimeError, TimeoutError) as error:
            self._home_completed_owner_started = None
            response.success = False
            response.message = str(error)
        finally:
            with self._active_lock:
                if self._active_goal == "home":
                    self._active_goal = None
        return response

    @_control_callback
    def _commission_gripper_step(self, request, response):
        """Raw bounded calibration step, serialized through the sole bus owner."""
        response.success = False
        if type(request.delta_counts) is not int or not 0 < abs(request.delta_counts) <= 16:
            response.message = "delta_counts must be a nonzero integer in [-16,16]"
            return response
        process = self._process
        if (self._external_state_only or process is None or process.poll() is not None or
                self._owner_faulted or self._read_state() is None):
            response.message = "resident ArmSession owner unavailable, faulted or stale"
            return response
        with self._active_lock:
            if self._active_goal is not None:
                response.message = "another owner operation is active"
                return response
            self._active_goal = "commission_gripper"
            self._home_completed_owner_started = None
        started = time.monotonic()
        try:
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            self._send({"command": "commission_gripper_step", "delta_counts": request.delta_counts})
            deadline = started + 4.0  # owner motion budget is fixed at 2 seconds
            while True:
                if process.poll() is not None:
                    raise RuntimeError("owner exited during component step")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("component result unknown; stop and inspect before any retry")
                try:
                    result = self._response_queue.get(timeout=min(.05, remaining))
                    break
                except queue.Empty:
                    continue
            if not isinstance(result, dict):
                raise RuntimeError("component acknowledgement is not an object")
            state = result.get("state")
            if (result.get("ok") is not True or result.get("stopped") or result.get("cancelled") or
                    not isinstance(state, dict) or state.get("event") != "commission_gripper_complete"):
                raise RuntimeError(result.get("error", "component operation did not complete"))
            torque, raw = state.get("torque"), state.get("raw")
            now = time.monotonic()
            freshness = min(.1, float(self.get_parameter("state_freshness_s").value))
            torque_stamp = state.get("torque_monotonic")
            if (state.get("arm_id") != self._expected_arm_id or
                    state.get("calibration_sha256") != self._expected_calibration_sha256 or
                    not max(started, now-freshness) <= state.get("monotonic", -1) <= now or
                    type(torque_stamp) not in (int, float) or
                    not max(started, now-freshness) <= torque_stamp <= now or
                    not isinstance(torque, dict) or set(torque) != set(TELEOP_JOINTS) or
                    any(type(v) is not int or v != 0 for v in torque.values()) or
                    not isinstance(raw, dict) or set(raw) != set(TELEOP_JOINTS) or
                    any(type(v) is not int or not 0 <= v <= 4095 for v in raw.values())):
                raise RuntimeError("component completion lacks fresh identity-bound all-off readback")
            response.success = True
            response.message = json.dumps({"state": "commission_gripper_complete", "raw": raw,
                                           "delta_counts": request.delta_counts, "torque": torque,
                                           "component_command": state.get("component_command"),
                                           "write_scope": state.get("write_scope"),
                                           "position_monotonic": state["monotonic"],
                                           "torque_monotonic": torque_stamp})
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as error:
            self._owner_faulted = True
            response.message = str(error)
            # Stop remains jaw-only in the owner even after the component method
            # has returned. Do not turn an unknown result into success or retry.
            try:
                self._send({"command": "stop", "request_id": uuid.uuid4().hex})
            except (OSError, RuntimeError, TimeoutError, ValueError):
                pass
        finally:
            with self._active_lock:
                if self._active_goal == "commission_gripper":
                    self._active_goal = None
        return response

    def _sorting_ready(self, _request, response):
        """Read-only admission check for sorting; this service never homes the arm."""
        process = self._process
        state = self._read_state()
        with self._active_lock:
            active = self._active_goal is not None
            teleop_state = self._teleop_state
        valid = (not self._external_state_only and process is not None and process.poll() is None and
                 not self._owner_faulted and not active and teleop_state == "disabled" and
                 self._gripper_jaw is not None and state is not None and
                 self._home_completed_owner_started == self._owner_started and
                 state.get("arm_id") == self._expected_arm_id and
                 state.get("calibration_sha256") == self._expected_calibration_sha256)
        response.success = bool(valid)
        response.message = ("sorting readiness confirmed from fresh identity-matched resident state; no motion commanded"
                            if valid else
                            "sorting not ready: require fresh matching owner, completed home in this session, "
                            "qualified jaw calibration, idle trajectory and disabled teleoperation")
        return response

    @_control_callback
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

    @_control_callback
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
                self._teleop_stop_reason = "leader_pose_stale"
            stale_limit = float(self.get_parameter("teleop_pose_stale_s").value)
            self.get_logger().error(
                f"Teleoperation stopping: leader pose stale ({age * 1000:.1f} ms > "
                f"{stale_limit * 1000:.1f} ms), source sequence {latest['sequence']}")
            self._publish_teleop_status("stopping", "leader_pose_stale",
                                        age_ms=round(age * 1000, 1),
                                        stale_limit_ms=round(stale_limit * 1000, 1),
                                        source_sequence=latest["sequence"],
                                        session_id=session_id)
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
                self._teleop_stop_reason = "owner_command_transport_error"
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
        # Leave most of the stop budget available for stop delivery/readback,
        # including when a trajectory exceeds the pipe's available capacity.
        stop_timeout = float(self.get_parameter("stop_timeout_s").value)
        timeout = min(.5, stop_timeout / 4) if math.isfinite(stop_timeout) and stop_timeout > 0 else .1
        deadline = time.monotonic() + timeout
        if not self._write_lock.acquire(timeout=timeout):
            raise TimeoutError("owner command writer did not become available")
        try:
            if getattr(self, "_stop_latched", False) and command.get("command") not in {"stop", "exit"}:
                raise RuntimeError("owner stop barrier rejects motion commands")
            self._write_owner_command(command, deadline)
        except (OSError, RuntimeError, ValueError):
            self._owner_faulted = True
            raise
        finally:
            self._write_lock.release()

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

    @_control_callback
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

    @_control_callback
    def _gripper_goal_request(self, goal):
        if (self._external_state_only or self._gripper_jaw is None or
                self._gripper_raw_limits is None or self._owner_faulted or
                self._process is None or self._process.poll() is not None):
            self.get_logger().warning(
                "Rejecting gripper goal: hardware owner or qualified jaw-gap calibration is unavailable")
            return GoalResponse.REJECT
        command = goal.command
        try:
            gap_to_percent(command.position, self._gripper_jaw)
        except (TypeError, ValueError):
            return GoalResponse.REJECT
        # ArmSession has no calibrated force limit. Nonzero effort must not be
        # silently interpreted as a force command.
        if not math.isfinite(float(command.max_effort)) or command.max_effort != 0:
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active_goal is not None or self._owner_faulted:
                return GoalResponse.REJECT
            self._active_goal = "gripper"
            self._gripper_goal_cancelled = False
            self._gripper_command_submitted = False
        return GoalResponse.ACCEPT

    def _gripper_cancel_request(self, _goal_handle):
        with self._active_lock:
            if self._active_goal != "gripper" or not getattr(_goal_handle, "is_active", True):
                return CancelResponse.REJECT
            self._gripper_goal_cancelled = True
            if not self._gripper_command_submitted:
                return CancelResponse.ACCEPT
            try:
                self._send({"command": "cancel_hold"})
                return CancelResponse.ACCEPT
            except (BrokenPipeError, OSError, RuntimeError):
                return CancelResponse.REJECT

    def _submit_gripper_plan(self, command):
        """Serialize cancel-before-submit against owner command admission."""
        with self._active_lock:
            if (self._active_goal != "gripper" or self._gripper_goal_cancelled or
                    self._gripper_command_submitted):
                return False
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            self._send(command)
            self._gripper_command_submitted = True
            return True

    @_control_callback
    def _execute_gripper(self, goal_handle):
        command = goal_handle.request.command
        result = GripperCommand.Result()
        command_sent = False
        command_complete = False
        stop_confirmed = False
        try:
            if self._external_state_only or self._gripper_jaw is None or self._gripper_raw_limits is None:
                raise RuntimeError("qualified jaw-gap calibration and resident hardware owner are required")
            if not math.isfinite(float(command.max_effort)) or command.max_effort != 0:
                raise ValueError("max_effort is unsupported; this servo has no calibrated force limit")
            state = self._read_state()
            if state is None or state.get("event") not in {"sample", "connected"}:
                raise RuntimeError("fresh resident state required for gripper command")
            if state.get("arm_id") != self._expected_arm_id or state.get("calibration_sha256") != self._expected_calibration_sha256:
                raise RuntimeError("resident state identity does not match configured calibration")
            pose, raw = state.get("pose"), state.get("raw")
            if not isinstance(pose, dict) or not isinstance(raw, dict) or set(raw) != set(TELEOP_JOINTS):
                raise RuntimeError("resident state lacks a complete six-joint measured pose")
            target_percent = gap_to_percent(command.position, self._gripper_jaw)
            low, high = self._gripper_raw_limits
            target_raw = int(round(low + target_percent * (high - low) / 100.0))
            # Match the resident's 650-count leg cap, 10-count arrival
            # tolerance plus four-count dwell allowance, six-second leg
            # deadline and 0.4-second dwell budget.
            # Use the full calibrated span because the owner measures the
            # actual start again when it admits this single command.
            max_chunks = max(1, math.ceil((high - low) / (650 - 10 - 4)))
            command_timeout_s = max_chunks * (6.0 + 0.4) + 2.0
            deadline = time.monotonic() + command_timeout_s
            submitted = self._submit_gripper_plan({"command": "move_gripper", "target_raw": target_raw})
            if not submitted:
                goal_handle.canceled()
                command_complete = True
                return result
            command_sent = True
            response = None
            cancel_deadline = None
            while time.monotonic() < deadline:
                with self._active_lock:
                    cancelled = self._gripper_goal_cancelled
                if cancelled:
                    cancel_deadline = cancel_deadline or (time.monotonic() + 2.0)
                    if time.monotonic() >= cancel_deadline:
                        raise TimeoutError("cancel_hold acknowledgement missing; owner remains reserved")
                if self._process.poll() is not None:
                    raise RuntimeError("resident owner stopped during gripper command")
                try:
                    candidate = self._response_queue.get(timeout=min(0.05, deadline-time.monotonic()))
                    with self._active_lock:
                        cancelled = self._gripper_goal_cancelled
                    if candidate.get("stopped"):
                        raise InterruptedError("owner stopped; gripper torque disabled")
                    if cancelled:
                        state = candidate.get("state") if isinstance(candidate, dict) else None
                        if not isinstance(state, dict) or state.get("event") != "trajectory_cancel_hold":
                            continue
                    response = candidate
                    break
                except queue.Empty:
                    continue
            if response is None:
                self._send({"command": "cancel_hold"})
                cancel_deadline = time.monotonic() + 2.0
                while time.monotonic() < cancel_deadline:
                    try:
                        stopped = self._response_queue.get(timeout=min(0.05, cancel_deadline-time.monotonic()))
                    except queue.Empty:
                        continue
                    stopped_state = stopped.get("state") if isinstance(stopped, dict) else None
                    if isinstance(stopped_state, dict) and stopped_state.get("event") == "trajectory_cancel_hold":
                        stop_confirmed = True
                        break
                if not stop_confirmed:
                    self._owner_faulted = True
                raise TimeoutError(f"resident gripper command did not complete within {command_timeout_s:.1f} seconds")
            if not response.get("ok"):
                raise RuntimeError(response.get("error", "resident rejected gripper command"))
            final_state = response.get("state")
            if not isinstance(final_state, dict) or final_state.get("event") != "plan_complete":
                if self._gripper_goal_cancelled and isinstance(final_state, dict) and final_state.get("event") == "trajectory_cancel_hold":
                    stop_confirmed = True
                    raise InterruptedError("gripper cancellation confirmed by resident hold")
                raise RuntimeError("resident acknowledgement did not confirm plan_complete")
            final_percent = float(final_state["pose"]["gripper"])
            result.position = percent_to_gap(final_percent, self._gripper_jaw)
            result.effort = 0.0  # Not measured in force units.
            tolerance_percent = 2.0
            result.reached_goal = abs(final_percent - target_percent) <= tolerance_percent
            legs = final_state.get("legs")
            contact_inferred = (isinstance(legs, list) and bool(legs) and
                                all(leg.get("label") == "gripper_command" for leg in legs) and
                                legs[-1].get("contact_inferred") is True)
            result.stalled = bool(contact_inferred)
            if not result.reached_goal and not contact_inferred:
                raise RuntimeError("jaw stopped outside requested calibrated gap; pickup is unverified")
            # Serialize terminal success against cancellation. If cancellation
            # wins after the plan reply, keep the owner reserved until its
            # measured hold acknowledgement arrives.
            with self._active_lock:
                cancelled = self._gripper_goal_cancelled
                if not cancelled:
                    command_complete = True
                    goal_handle.succeed()
            if cancelled:
                cancel_deadline = time.monotonic() + 2.0
                while time.monotonic() < cancel_deadline:
                    try:
                        held = self._response_queue.get(timeout=min(0.05, cancel_deadline-time.monotonic()))
                    except queue.Empty:
                        continue
                    held_state = held.get("state") if isinstance(held, dict) else None
                    if isinstance(held, dict) and held.get("ok") and isinstance(held_state, dict) and held_state.get("event") == "trajectory_cancel_hold":
                        stop_confirmed = True
                        raise InterruptedError("gripper cancellation confirmed by resident hold")
                raise TimeoutError("cancel_hold acknowledgement missing; owner remains reserved")
            return result
        except InterruptedError as error:
            goal_handle.canceled()
            result.stalled = False
            self.get_logger().warning(str(error))
            return result
        except (OSError, ValueError, RuntimeError, TimeoutError, TypeError, KeyError) as error:
            if command_sent and not command_complete and not stop_confirmed:
                self._owner_faulted = True
            result.stalled = False
            result.reached_goal = False
            self.get_logger().error(f"GripperCommand failed: {error}")
            goal_handle.abort()
            return result
        finally:
            with self._active_lock:
                if self._active_goal == "gripper" and (not self._owner_faulted or getattr(self, "_stop_latched", False)):
                    self._active_goal = None
                self._gripper_command_submitted = False

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

    @_control_callback
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
        if (result_payload and result_payload.get("stopped")) or getattr(self, "_stop_latched", False):
            artifact["status"] = "stopped_torque_off" if result_payload and result_payload.get("stopped") else "stop_requested"
            artifact["result"] = result_payload
            self._write_artifact(artifact)
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                "owner stop interrupted trajectory; torque-off confirmation is provided by the stop service")
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
        executor = MultiThreadedExecutor(num_threads=4)
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
