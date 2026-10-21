"""FollowJointTrajectory adapter that delegates all bus access to ArmSession."""
from __future__ import annotations

import copy
import json
import hashlib
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
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from so101_station_workflow.srv import CommissionGripperStep
from so101_station_operator.srv import OperatorMaintenance
from so101_station_control.operator_restart import OperatorRestartMixin
from trajectory_msgs.msg import JointTrajectoryPoint

from .leader_reader import JOINTS as TELEOP_JOINTS, matched_pose, validate_pose_packet
from .gripper_action import (gap_to_percent, percent_to_gap, validate_jaw_calibration,
                             joint_to_percent, percent_to_joint, validate_joint_calibration, joint_target_raw)

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _teleop_leader_readiness(latest, now, max_age):
    """Age only the cache admitted by validate_pose_packet; never read a device."""
    if latest is None:
        return False, "leader_pose_missing", None
    stamp = latest.get("stamp_s")
    if type(stamp) not in (int, float) or not math.isfinite(stamp):
        return False, "leader_pose_timestamp_invalid", None
    age = now - stamp
    if not math.isfinite(age):
        return False, "leader_pose_timestamp_invalid", None
    if not math.isfinite(max_age) or max_age <= 0:
        return False, "leader_freshness_config_invalid", age
    # Retain the packet validator's 20 ms allowance for local scheduling skew.
    if age < -0.02:
        return False, "leader_pose_future", age
    if age > max_age:
        return False, "leader_pose_stale", age
    return True, "fresh_calibrated_leader_pose", age


def _arm_bounds_observation(state):
    """Validate the owner's classification; never reproduce its raw-bound math."""
    value = state.get("arm_bounds") if isinstance(state, dict) else None
    if (not isinstance(value, dict) or value.get("schema") != "arm-bounds-observation-v1" or
            not isinstance(value.get("classification"), str) or
            value.get("classification") not in
            {"normal", "bounded_reentry_required", "unrecoverable", "unknown"} or
            type(value.get("position_monotonic")) not in (int, float) or
            not math.isfinite(value["position_monotonic"]) or
            value["position_monotonic"] != state.get("monotonic")):
        return None
    keys = ("admitted_min", "admitted_max", "firmware_min", "firmware_max")
    for key in keys:
        bounds = value.get(key)
        if (not isinstance(bounds, dict) or set(bounds) != set(TELEOP_JOINTS) or
                any(type(v) is not int or not 0 <= v <= 4095 for v in bounds.values())):
            return None
    if any(not value["firmware_min"][j] <= value["admitted_min"][j] <=
           value["admitted_max"][j] <= value["firmware_max"][j] for j in TELEOP_JOINTS):
        return None
    return {"schema": value["schema"], "classification": value["classification"],
            "position_monotonic": value["position_monotonic"],
            **{key: dict(value[key]) for key in keys}}


def _arm_session_command(station_python, config, state_file, log_dir, lock, idle_timeout_s,
                         recover_sag=False, observe_only=False):
    timeout = float(idle_timeout_s)
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("owner_idle_timeout_s must be finite and positive")
    command = [str(station_python), "-m", "workshop.station.arm_session",
            "--config", str(config), "--state-file", str(state_file),
            "--logdir", str(log_dir), "--lock", str(lock),
            "--persistent-idle", "--idle-timeout", str(timeout)]
    if observe_only:
        command.append("--observe-only")
    if recover_sag:
        command.append("--recover-sag")
    return command


def _load_owner_source_hashes(station_root):
    """Use the child's shared source-map contract once for this owner generation."""
    directory = Path(station_root) / "workshop/station"
    spec = importlib.util.spec_from_file_location(
        "so101_owner_source_contract", directory / "recovery_types.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.owner_source_sha256(directory)


def _load_recovery_cache_config(station_root, config, state_file):
    """Load the shared lightweight authority parser only during owner startup."""
    source = Path(station_root) / "workshop/station/recovery_runtime_config.py"
    spec = importlib.util.spec_from_file_location("so101_recovery_runtime_config", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.load_recovery_runtime_config(config, state_file)


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


def _observe_owner_error(state, *, arm_id, calibration_sha256, owner_started, source_sha256, now):
    """Validate a new passive generation independently of torque-off readiness."""
    if not isinstance(state, dict) or state.get("observe_only") is not True:
        return "new observe-only owner state missing"
    if (state.get("arm_id") != arm_id or state.get("calibration_sha256") != calibration_sha256 or
            state.get("owner_source_sha256") != source_sha256):
        return "owner identity, calibration or source pin mismatch"
    for key in ("monotonic", "torque_monotonic"):
        stamp = state.get(key)
        if type(stamp) not in (int, float) or not max(owner_started, now-.1) <= stamp <= now:
            return "new owner encoder/torque state is not fresh"
    for key, upper in (("raw", 4095), ("torque", 1)):
        values = state.get(key)
        if (not isinstance(values, dict) or set(values) != set(TELEOP_JOINTS) or
                any(type(v) is not int or not 0 <= v <= upper for v in values.values())):
            return "new owner lacks six-joint raw/torque readback"
    return None


def _fault_observer_error(state, *, arm_id, calibration_sha256, owner_started, source_sha256, now, allow_disarmed=False):
    """Only a fresh, pinned, all-off fault observer may exit for reconnect."""
    if (not isinstance(state, dict) or not isinstance(source_sha256, dict) or not source_sha256 or
            type(owner_started) not in (int, float) or not math.isfinite(owner_started) or owner_started <= 0):
        return "fault-observing owner state or pinned generation missing"
    if not isinstance(state.get("owner_session_id"), str) or not state["owner_session_id"]:
        return "live owner session identity missing"
    fault = state.get("fault_observation")
    if fault is None and not allow_disarmed:
        return "owner is alive without matching fault-observation identity"
    if fault is not None:
        if (not isinstance(fault, dict) or type(fault.get("schema")) is not int or fault.get("schema") != 1 or
                not isinstance(state.get("owner_session_id"), str) or
                fault.get("owner_session_id") != state["owner_session_id"]):
            return "owner is alive without matching fault-observation identity"
        entered = fault.get("entered_monotonic")
        if (type(entered) not in (int, float) or not math.isfinite(entered) or
                not owner_started <= entered <= now or
                any(type(state.get(key)) not in (int, float) or entered > state[key]
                    for key in ("monotonic", "torque_monotonic"))):
            return "fault observation does not belong to this owner generation"
    error = _observe_owner_error(dict(state, observe_only=True), arm_id=arm_id,
        calibration_sha256=calibration_sha256, owner_started=owner_started,
        source_sha256=source_sha256, now=now)
    if error:
        return error
    if any(state["torque"].values()):
        return "fault-observing reconnect requires all six servos off"
    return None


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


def prepare_with_controller_timing(helper, joint_names, points, *, start_radians, max_jerk,
                                   motion_profile=None, position_limits=None):
    """Retry controller limit failures up to 4x timing, retaining the 300s duration guard."""
    profile = motion_profile if motion_profile is not None else helper.resolve_ros_motion_profile({})
    retry_prefixes = ("velocity limit exceeded:", "acceleration limit exceeded:",
                      "jerk limit exceeded:")
    last_error = None
    for factor in (1.0, 1.2, 1.44, 1.728, 2.0, 2.4, 2.88, 3.0, 3.456, 4.0):
        effective_points = copy.deepcopy(points)
        if factor > 1.0:
            for point in effective_points:
                point["time"] *= factor
                for field, scale in (("velocities", factor), ("accelerations", factor ** 2)):
                    if point.get(field) is not None:
                        point[field] = [value / scale for value in point[field]]
        try:
            prepared = helper.prepare_trajectory(
                joint_names, effective_points, start_radians=start_radians,
                max_velocity=profile.max_velocity,
                max_acceleration=profile.max_acceleration,
                max_jerk=max_jerk, period=helper.ROS_TRAJECTORY_PERIOD_S,
                position_limits=position_limits)
        except ValueError as error:
            if not str(error).startswith(retry_prefixes):
                raise
            last_error = error
        else:
            return factor, effective_points, prepared
    raise ValueError(f"controller retiming failed within maximum factor 4.0: {last_error}") from last_error


def _control_callback(callback):
    """Keep the stop barrier until every previously admitted callback exits."""
    @functools.wraps(callback)
    def tracked(self, *args, **kwargs):
        condition = getattr(self, "_stop_condition", None)
        if condition is None:  # lightweight offline harnesses
            return callback(self, *args, **kwargs)
        with condition:
            resolving_maintenance = (callback.__name__ == "_operator_maintenance" and
                                     args and getattr(args[0], "operation", "") == "verify")
            if self._stop_latched and not resolving_maintenance:
                if callback.__name__ in {"_goal_request", "_gripper_goal_request", "_moveit_gripper_goal_request"}:
                    return GoalResponse.REJECT
                if callback.__name__ in {"_home", "_recover_camera", "_prepare", "_commission_gripper_step", "_commission_gripper_held_step", "_acknowledge_held_jaw", "_gripper_diagnostics", "_joint_diagnostics", "_calibration_ready", "_set_teleop_enabled", "_set_teleop_absolute_mode", "_enable_native_damping", "_native_damping_inventory", "_operator_maintenance", "_rearm_motion_profile"}:
                    response = args[1]
                    response.success, response.message = False, "owner stop remains latched"
                    if callback.__name__ == "_prepare":
                        response.message = json.dumps(dict(schema=1, kind="arm_prepare", state="rejected",
                            error="owner stop remains latched", command_delivery="not_sent",
                            phases=[], stop_verified=None, home_completed_this_session=False))
                    return response
                if callback.__name__ == "_teleop_tick":
                    return None
                # Accepted action goals must reach a terminal state even if
                # their execute callback had not started when stop arrived.
                with self._active_lock:
                    self._active_goal = None
                args[0].abort()
                if callback.__name__ in {"_execute_gripper", "_execute_moveit_gripper"}:
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


class TrajectoryBridge(OperatorRestartMixin, Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("so101_trajectory_bridge", parameter_overrides=parameter_overrides)
        self.declare_parameter("enable_hardware", False)
        self.declare_parameter("external_state_only", False)
        self.declare_parameter("publish_gripper_visual_proxy", True)
        self.declare_parameter("station_python", os.environ.get("SO101_STATION_PYTHON", ""))
        self.declare_parameter("station_root", os.environ.get("SO101_STATION_ROOT", os.getcwd()))
        self.declare_parameter("station_config", "")
        self.declare_parameter("joint_limit_contract", "")
        self.declare_parameter("planning_position_limits_json", "")
        self.declare_parameter("state_file", "")
        self.declare_parameter("log_dir", "")
        self.declare_parameter("trajectory_log_dir", "")
        self.declare_parameter("station_lock", "")
        self.declare_parameter("operator_service_units", [""])
        self.declare_parameter("operator_station_reset_units", [""])
        self.declare_parameter("operator_maintenance_journal", str(Path.home()/".local/share/exciting-science/station/operator-maintenance-pending.json"))
        self.declare_parameter("owner_idle_timeout_s", 3600.0)
        self.declare_parameter("recover_sag", False)
        self.declare_parameter("startup_observe_only", False)
        self.declare_parameter("action_name", "/follower/arm_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action_name", "/follower/gripper_controller/gripper_cmd")
        self.declare_parameter("gripper_jaw_calibration", "")
        self.declare_parameter("gripper_joint_calibration", "")
        self.declare_parameter("gripper_joint_limits_rad", [math.radians(-10.), math.radians(100.)])
        self.declare_parameter("moveit_gripper_action_name", "/follower/moveit_gripper_controller/gripper_cmd")
        self.declare_parameter("state_validity_service", "/follower/check_state_validity")
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("state_freshness_s", 0.25)
        self.declare_parameter("collision_check_timeout_s", 30.0)
        self.declare_parameter("max_jerk_rad_s3", 2.0)
        self.declare_parameter("calibration_endpoint_mode", False)
        self.declare_parameter("recovery_depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("recovery_camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("teleop_pose_topic", "/leader/teleop_pose")
        self.declare_parameter("teleop_status_topic", "/follower/teleop_status")
        self.declare_parameter("teleop_enable_service", "/follower/teleop_enable")
        self.declare_parameter("teleop_pose_stale_s", 0.15)
        self.declare_parameter("teleop_mode", "relative")
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
        self._operator_maintenance_token = None
        self._operator_maintenance_purpose = None
        self._operator_maintenance_deadline = None
        self._cancel_before_start = False
        self._write_lock = threading.Lock()
        self._stop_condition = threading.Condition()
        self._stop_lock = threading.Lock()
        self._stop_latched = False
        self._writer_faulted = False
        self._stop_pending = None
        self._stop_ack = None
        self._stop_requested = None
        self._control_callbacks = 0
        self._action_group = ReentrantCallbackGroup()
        self._response_queue = queue.Queue()
        self._teleop_condition = threading.Condition()
        self._teleop_acknowledgements = {}
        self._teleop_state = "disabled"
        self._teleop_mode = str(self.get_parameter("teleop_mode").value)
        if self._teleop_mode not in {"relative", "absolute"}:
            raise ValueError("teleop_mode must be relative or absolute")
        self._teleop_session = None
        self._teleop_origin = None
        self._teleop_follower_start = None
        self._teleop_last_source_sequence = -1
        self._teleop_owner_sequence = -1
        self._teleop_inflight_sequence = None
        self._teleop_inflight = False
        self._teleop_waiting_ack = None
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
        self._teleop_wakeup_lock = threading.Lock()
        self._teleop_ack_guard = None
        self._process = None
        self._owner_started = 0.0
        self._owner_readers = []
        self._owner_source_expected = None
        contract_path = str(self.get_parameter("joint_limit_contract").value)
        self._planning_contract = json.loads(Path(contract_path).read_text()) if contract_path else None
        limits_json = str(self.get_parameter("planning_position_limits_json").value)
        self._planning_limits = json.loads(limits_json) if limits_json else None
        self._planning_calibration = None
        self._reconnect_observe = False
        self._owner_fault_lock = threading.Lock()
        self._owner_fault_serial = 0
        self._held_jaw_fault = None
        self._clear_owner_fault("initialize")
        self._expected_arm_id = None
        self._expected_calibration_sha256 = None
        self._saved_home_pose = None
        self._saved_home_tolerance = None
        self._home_completed_owner_started = None
        self._prepare_reservation = None
        self._prepare_cohort = None
        self._prepare_phase = None
        self._prepare_config_path = None
        self._state_cache_lock = threading.Lock()
        self._cached_owner_state = None
        self._cached_owner_generation = None
        self._cached_owner_reason = "cached_state_missing"
        self._recovery_depth_cache = None
        self._recovery_camera_identity = None
        self._recovery_collector_context = None
        self._recovery_depth_subscriptions = []
        self._recovery_cache_config = None
        self._recovery_cache_error = None
        self._state_file = self._resolve_state_file()
        self._gripper_jaw = None
        self._gripper_joint = None
        self._gripper_raw_limits = None
        self._gripper_goal_cancelled = False
        self._gripper_command_submitted = False
        self._external_state_only = bool(self.get_parameter("external_state_only").value)
        self._trajectory_helper = None
        self._ros_motion_profile = None
        self._state_validity = None
        self._operator_restart_init()
        # Tail a separately owned resident without starting, locking, or commanding another owner.
        if not self._external_state_only:
            self._reconnect_observe = self.get_parameter("startup_observe_only").value is True
            if self._reconnect_observe:
                self._mark_owner_fault("startup_observe")
            else:
                self._clear_owner_fault("startup_active")
            self._start_owner(observe_only=self._reconnect_observe)
        if not self._external_state_only:
            self._operator_restart_load_pending()
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
        self._moveit_gripper_action = ActionServer(
            self, GripperCommand,
            str(self.get_parameter("moveit_gripper_action_name").value),
            execute_callback=self._execute_moveit_gripper,
            goal_callback=self._moveit_gripper_goal_request,
            cancel_callback=self._gripper_cancel_request,
            callback_group=self._action_group)
        self.create_service(Trigger, "/follower/gripper_calibration_status",
                            self._gripper_calibration_status, callback_group=self._action_group)
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
        self._teleop_mode_service = self.create_service(
            SetBool, "/follower/teleop_absolute_mode", self._set_teleop_absolute_mode,
            callback_group=self._teleop_service_group)
        self._stop_group = MutuallyExclusiveCallbackGroup()
        self._reconnect_service = self.create_service(
            Trigger, "/follower/reconnect", self._reconnect, callback_group=self._stop_group)
        self._reset_owner_service = self.create_service(
            Trigger, "/follower/reset_owner", self._reset_owner, callback_group=self._stop_group)
        self._stop_service = self.create_service(
            Trigger, str(self.get_parameter("stop_service").value),
            self._stop, callback_group=self._stop_group)
        self._damping_service = self.create_service(
            Trigger, "/follower/damping_enable", self._enable_native_damping,
            callback_group=self._teleop_service_group)
        self._maintenance_service = self.create_service(
            OperatorMaintenance, "/follower/operator_maintenance_enable", self._operator_maintenance,
            callback_group=self._teleop_service_group)
        self._damping_inventory_service = self.create_service(
            Trigger, "/follower/damping_inventory", self._native_damping_inventory,
            callback_group=self._teleop_service_group)
        self._rearm_service = self.create_service(
            Trigger, "/follower/rearm_motion_profile", self._rearm_motion_profile,
            callback_group=self._teleop_service_group)
        self._home_service = self.create_service(
            Trigger, str(self.get_parameter("home_service").value),
            self._home, callback_group=self._teleop_service_group)
        self._recover_camera_service = self.create_service(
            Trigger, "/follower/recover_camera", self._recover_camera,
            callback_group=self._teleop_service_group)
        self._prepare_service = self.create_service(
            Trigger, "/follower/prepare", self._prepare,
            callback_group=self._action_group)
        self._status_group = MutuallyExclusiveCallbackGroup()
        self._status_service = self.create_service(
            Trigger, "/follower/status", self._status, callback_group=self._status_group)
        self._commission_gripper_service = self.create_service(
            CommissionGripperStep, "/follower/commission_gripper_step",
            self._commission_gripper_step, callback_group=self._teleop_service_group)
        self._commission_gripper_held_service = self.create_service(
            CommissionGripperStep, "/follower/commission_gripper_held_step",
            self._commission_gripper_held_step, callback_group=self._teleop_service_group)
        self._held_jaw_ack_service = self.create_service(
            Trigger, "/follower/acknowledge_held_jaw", self._acknowledge_held_jaw,
            callback_group=self._teleop_service_group)
        self._gripper_diagnostics_service = self.create_service(
            Trigger, "/follower/gripper_diagnostics", self._gripper_diagnostics,
            callback_group=self._teleop_service_group)
        self._joint_diagnostics_service = self.create_service(
            Trigger, "/follower/joint_diagnostics", self._joint_diagnostics,
            callback_group=self._teleop_service_group)
        self._sorting_ready_service = self.create_service(
            Trigger, "/follower/sorting_ready", self._sorting_ready,
            callback_group=self._teleop_service_group)
        self._calibration_ready_service = self.create_service(
            Trigger, str(self.get_parameter("calibration_ready_service").value),
            self._calibration_ready, callback_group=self._teleop_service_group)
        # Owner replies only signal the executor; control stays in the same
        # mutually exclusive group as the timer and latest-pose subscription.
        self._teleop_ack_guard = self.create_guard_condition(
            self._teleop_tick, callback_group=self._teleop_control_group)
        self._teleop_tick_timer = self.create_timer(
            1.0 / 125.0, self._teleop_timer_tick,
            callback_group=self._teleop_control_group)
        self.create_timer(1.0, self._publish_teleop_monitor)
        self.create_timer(.5, self._expire_operator_modes)
        if not self._external_state_only:
            if self._trajectory_helper is None:
                self._trajectory_helper = self._load_trajectory_helper()
            self._state_validity = self.create_client(
                GetStateValidity, str(self.get_parameter("state_validity_service").value),
                callback_group=self._action_group)
        if self._external_state_only:
            self.get_logger().warning(
                f"READ ONLY: mirroring external state from {self._state_file}; trajectory goals are disabled; "
                "gripper joint is a visual-only normalized-percent-to-URDF-range estimate")
        elif self._gripper_joint is None:
            self.get_logger().warning(
                "Gripper radians are unqualified; joint feedback is visual-only and MoveIt gripper commands are rejected")
        else:
            self.get_logger().info("MoveIt gripper joint uses qualified radians; separate SI action uses metres")

        self._configure_recovery_depth_cache()

    def _publish_joint_state(self):
        state = self._read_state()
        if state is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(ARM_JOINTS)
        message.position = [math.radians(float(state["pose"][name])) for name in ARM_JOINTS]
        if self._gripper_joint is not None or bool(self.get_parameter("publish_gripper_visual_proxy").value):
            # Digital-twin convention: normalized opening percent maps linearly to the
            # canonical URDF joint range [-10, 100] degrees. This is visual only.
            opening = min(100.0, max(0.0, float(state["pose"]["gripper"])))
            message.name.append("gripper")
            calibration = getattr(self, "_gripper_joint", None)
            if calibration is not None:
                try:
                    angle = percent_to_joint(opening, calibration)
                except ValueError:
                    # Keep body feedback flowing without a made-up jaw angle.
                    message.name.pop()
                    self._joint_state_pub.publish(message)
                    return
            else:
                angle = math.radians(-10.0 + 1.1 * opening)
            message.position.append(angle)
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
        source_bytes = source.read_bytes()
        spec = importlib.util.spec_from_file_location("so101_ros_trajectory_shared", source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        exec(compile(source_bytes, str(source), "exec"), module.__dict__)
        self._trajectory_helper_sha256 = hashlib.sha256(source_bytes).hexdigest()
        return module

    def _start_owner(self, *, observe_only=False):
        if self._external_state_only:
            raise RuntimeError("external_state_only mode must never start an ArmSession owner")
        if not bool(self.get_parameter("enable_hardware").value):
            raise RuntimeError("hardware action bridge requires enable_hardware:=true")
        # Retire the old producer before spawning a separately scoped runtime.
        if not self._close_recovery_depth_cache():
            raise RuntimeError("old recovery depth producer did not retire before owner spawn")
        with self._state_cache_lock:
            self._recovery_collector_context = None
        self._recovery_cache_config = None
        self._recovery_cache_error = None
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
        if self._trajectory_helper is None:
            self._trajectory_helper = self._load_trajectory_helper()
        config_data["arm"] = self._trajectory_helper.resolve_joint_limit_settings(config_data["arm"], config_path)
        self._ros_motion_profile = self._trajectory_helper.resolve_ros_motion_profile(config_data["arm"])
        self._prepare_config_path = config_path
        option = config_data.get("arm", {}).get("camera_recovery")
        if isinstance(option, dict) and option.get("enabled") is True:
            try:
                self._recovery_cache_config = _load_recovery_cache_config(
                    station_root, config_data, self._state_file)
            except (OSError, RuntimeError, ValueError, KeyError, TypeError) as error:
                self._recovery_cache_error = str(error)
                self.get_logger().warning(f"Recovery depth producer disabled: {error}")
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
        self._planning_calibration = raw_calibration
        if self._planning_contract is not None:
            validated_limits = self._trajectory_helper.validate_joint_limit_contract(
                self._planning_contract, config_data["arm"], raw_calibration,
                _load_owner_source_hashes(station_root))
            if self._planning_limits is None or any(tuple(self._planning_limits[j]) != validated_limits[j] for j in ARM_JOINTS):
                raise ValueError("loaded planner and bridge joint limits differ; relaunch with the same contract")
            self._planning_limits = validated_limits
        jaw_path_value = str(self.get_parameter("gripper_jaw_calibration").value).strip()
        if jaw_path_value:
            try:
                self._gripper_jaw = validate_jaw_calibration(json.loads(Path(jaw_path_value).read_text()))
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                raise RuntimeError("configured jaw-gap calibration is invalid") from error
        joint_path = str(self.get_parameter("gripper_joint_calibration").value).strip()
        if joint_path:
            try:
                limits = list(self.get_parameter("gripper_joint_limits_rad").value)
                if len(limits) != 2 or not all(math.isfinite(v) for v in limits) or limits[0] >= limits[1]:
                    raise ValueError("invalid admitted gripper model bounds")
                joint = validate_joint_calibration(json.loads(Path(joint_path).read_text()), limits)
                if (joint["arm_id"] != expected_arm_id or joint["calibration_sha256"] != expected_calibration_sha256):
                    raise ValueError("joint-angle calibration identity does not match resident owner")
                self._gripper_joint = joint
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
                raise RuntimeError("configured gripper joint-angle calibration is invalid") from error
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
            self.get_parameter("recover_sag").value, observe_only=observe_only)
        self._owner_source_expected = _load_owner_source_hashes(station_root)
        if self._owner_source_expected["ros_trajectory"] != self._trajectory_helper_sha256:
            raise RuntimeError("trajectory helper changed between bridge loading and owner source pinning")
        child_env = os.environ.copy()
        for key in tuple(child_env):
            if (key.startswith(("PYTHON", "LD_", "ROS_", "AMENT_", "COLCON_", "RMW_",
                                "CONDA_", "VIRTUAL_ENV")) or
                    key in {"CMAKE_PREFIX_PATH", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"}):
                child_env.pop(key, None)
        child_env["PATH"] = os.pathsep.join((str(Path(station_python).parent), os.defpath))
        self.get_logger().info("Starting the configured ArmSession bus owner subprocess")
        self._owner_started = time.monotonic()
        if self._closed:
            raise RuntimeError("bridge closed before owner spawn")
        self._process = subprocess.Popen(
            command, cwd=station_root, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1, env=child_env)
        process = self._process
        self._owner_readers = [threading.Thread(target=callback, args=(process,), daemon=True)
                               for callback in (self._read_responses, self._read_stderr)]
        for reader in self._owner_readers:
            reader.start()
        return self._wait_for_owner_state(expected_arm_id, expected_calibration_sha256,
                                          observe_only=observe_only)

    def _wait_for_owner_state(self, expected_arm_id, expected_calibration_sha256, *, observe_only):
        """Wait for this generation without masking a separately delivered STOP."""
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self._closed:
                raise RuntimeError("bridge closed during owner startup")
            if self._process.poll() is not None:
                raise RuntimeError(f"ArmSession exited during startup ({self._process.returncode})")
            state = self._read_state()
            if observe_only:
                if (isinstance(state, dict) and state.get("observe_only") is False and
                        _observe_owner_error(dict(state, observe_only=True), arm_id=expected_arm_id,
                            calibration_sha256=expected_calibration_sha256,
                            owner_started=self._owner_started, source_sha256=self._owner_source_expected,
                            now=time.monotonic()) is None):
                    raise RuntimeError("passive owner received STOP during startup; explicit stop verification required")
                if _observe_owner_error(state, arm_id=expected_arm_id,
                        calibration_sha256=expected_calibration_sha256,
                        owner_started=self._owner_started, source_sha256=self._owner_source_expected,
                        now=time.monotonic()) is None:
                    return state
            elif _owner_state_is_ready(state, arm_id=expected_arm_id,
                                     calibration_sha256=expected_calibration_sha256,
                                     owner_started=self._owner_started):
                self.get_logger().info("ArmSession connected; action server is ready")
                return
            time.sleep(0.05)
        if not observe_only:
            self._process.terminate()
        raise RuntimeError("ArmSession state did not become fresh within 20 seconds")

    def _active_goal_is(self, *reservations):
        """Match named operations by string value and action/token objects by identity.

        rclpy goal-handle equality assumes the other object has goal_id; it
        cannot safely compare against this bridge's string reservations.
        Callers retain their existing _active_lock ownership.
        """
        active = self._active_goal
        return any(active is reservation or
                   (isinstance(active, str) and isinstance(reservation, str) and
                    active == reservation)
                   for reservation in reservations)

    def _read_responses(self, process=None):
        process = self._process if process is None else process
        generation = self._owner_started
        for line in process.stdout:
            if process is not self._process:
                return
            try:
                response = json.loads(line)
                # The existing 20 Hz state reader caches teleop telemetry; do
                # not copy every 125 Hz step acknowledgement into status.
                if isinstance(response, dict) and response.get("command") != "teleop_step":
                    self._cache_owner_state(response.get("state", response.get("final_state")),
                                            generation=generation, process=process)
                if isinstance(response, dict) and response.get("command") in {"joint_diagnostics", "native_damping_inventory", "rearm_motion_profile"}:
                    with self._active_lock:
                        expected = self._active_goal_is(response["command"])
                    if not expected:
                        self._mark_owner_fault("unexpected_" + response["command"] + "_reply")
                        continue
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
        if command.get("command") in {"stop", "exit"}:
            self.get_logger().info(
                f"Owner safety command: {command['command']} "
                f"request_id={command.get('request_id')} generation={self._owner_started} "
                f"caller={sys._getframe(2).f_code.co_name}")
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

    def _reset_owner(self, request, response):
        """Explicit controller reinitialization of a fresh, idle, all-off owner."""
        return self._reconnect(request, response, allow_disarmed=True)

    def _reconnect(self, _request, response, *, allow_disarmed=False):
        """Explicit dead/fault-observing owner replacement; passive startup only."""
        response.success = False
        if not self._stop_lock.acquire(blocking=False):
            response.message = "stop/reconnect lifecycle operation is active"
            return response
        old_barrier = self._stop_latched
        reserved = False
        try:
            if self._external_state_only or self._closed:
                raise RuntimeError("bridge cannot own a reconnect")
            retiring = self._process if self._process is not None and self._process.poll() is None else None
            if retiring is not None:
                generation = self._owner_started
                error = _fault_observer_error(self._read_state(), arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    owner_started=generation, source_sha256=self._owner_source_expected,
                    now=time.monotonic(), allow_disarmed=allow_disarmed)
                if error:
                    raise RuntimeError(error)
            with self._stop_condition:
                with self._active_lock:
                    if (self._active_goal_is("native_damping", "operator_maintenance", "operator_maintenance_unknown", "native_damping_inventory") and
                            not self._control_callbacks and self._process is not None and self._process.poll() is not None):
                        # Explicit reconnect retires only quiesced modes of this already-dead generation.
                        self._active_goal = None
                        if self._operator_external_job is None:
                            self._operator_maintenance_token = None
                        self._operator_maintenance_deadline = None
                        old_barrier = True
                        self._stop_latched = True
                        self._mark_owner_fault("dead_operator_generation_retired")
                    if self._active_goal is not None or self._control_callbacks or self._teleop_state != "disabled":
                        raise RuntimeError("owner operation or callback has not quiesced")
                    self._stop_latched = True
                    self._active_goal = "reconnect"
                    reserved = True
            self._mark_owner_fault("_reconnect")
            self._home_completed_owner_started = None
            if retiring is not None:
                generation = self._owner_started
                state = self._read_state()
                error = _fault_observer_error(state, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    owner_started=generation, source_sha256=self._owner_source_expected,
                    now=time.monotonic(), allow_disarmed=allow_disarmed)
                if error:
                    raise RuntimeError(error)
                if self._process is not retiring or self._owner_started != generation or self._closed:
                    raise RuntimeError("fault-observing owner changed before explicit exit")
                self._send({"command": "exit"})
                retiring.wait(timeout=8.)
            elif self._process is not None:
                self._process.wait(timeout=.5)
            for reader in self._owner_readers:
                reader.join(timeout=.5)
                if reader.is_alive():
                    raise RuntimeError("old owner reader has not quiesced")
            while True:
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break
            self._writer_faulted = False
            self._reconnect_observe = True
            if self._closed:
                raise RuntimeError("bridge closed before reconnect spawn")
            state = self._start_owner(observe_only=True)
            if self._closed:
                raise RuntimeError("bridge closed during reconnect startup")
            self._configure_recovery_depth_cache()
            if any(state["torque"].values()):
                response.message = "observe-only owner connected; torque is on, explicit stop required"
                return response
            response.success = True
            response.message = json.dumps({"state": "observe_only_connected", "raw": state["raw"],
                "torque": state["torque"], "owner_source_sha256": state["owner_source_sha256"],
                "monotonic": state["monotonic"], "torque_monotonic": state["torque_monotonic"],
                "fault_preserved": True, "explicit_stop_required": True})
        except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
            response.message = str(error)
        finally:
            if reserved:
                with self._active_lock:
                    if self._active_goal_is("reconnect"):
                        self._active_goal = None
                self._stop_latched = old_barrier
            self._stop_lock.release()
        return response

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
            if self._reconnect_observe:
                state = self._read_state()
                # A separately delivered STOP can already have promoted this
                # passive generation to normal/disarmed. Explicit STOP must
                # still be able to verify it; motion admission remains latched.
                if isinstance(state, dict) and type(state.get("observe_only")) is bool:
                    state = dict(state, observe_only=True)
                error = _observe_owner_error(state, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    owner_started=self._owner_started, source_sha256=self._owner_source_expected,
                    now=time.monotonic())
                if error is not None:
                    raise RuntimeError(error)
            request_id = uuid.uuid4().hex
            # This lock is also the final admission check for every owner write.
            # Release it before taking _active_lock (execute uses the reverse order).
            self._stop_latched = True
            self._mark_owner_fault("_stop")
            if not self._write_lock.acquire(timeout=max(0., deadline - time.monotonic())):
                raise TimeoutError("owner command writer did not quiesce; stop remains latched")
            try:
                with self._stop_condition:
                    self._stop_pending = request_id
                    self._stop_ack = None
                    requested = time.monotonic()
                    self._stop_requested = requested
                self._write_stop({"command": "stop", "request_id": request_id}, deadline)
            finally:
                self._write_lock.release()
            with self._active_lock:
                self._cancel_before_start = True
                self._gripper_goal_cancelled = True
                self._home_completed_owner_started = None
                if self._active_goal_is("native_damping", "operator_maintenance", "operator_maintenance_unknown"):
                    self._active_goal = "operator_maintenance_unknown" if self._operator_external_job is not None else None
                    if self._operator_external_job is None:
                        self._operator_maintenance_token = None
                    self._operator_maintenance_deadline = None
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
                if self._active_goal is not None and not (self._active_goal_is("operator_maintenance_unknown") and self._operator_external_job is not None):
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
                self._stop_latched = self._operator_external_job is not None
                if self._operator_external_job is None and state.get("fault_observation") is None:
                    self._clear_owner_fault("_stop")
                elif state.get("fault_observation") is not None:
                    self._mark_owner_fault("owner_fault_observation")
                    self._home_completed_owner_started = None
                self._reconnect_observe = False
                self._cancel_before_start = False
            self._cache_owner_state(state)
            response.success = True
            response.message = "owner stopped; all six servos verified torque off (disarmed)"
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as error:
            response.message = str(error)
        finally:
            with self._stop_condition:
                self._stop_pending = None
                self._stop_requested = None
            self._stop_lock.release()
        return response

    @_control_callback
    def _operator_maintenance(self, request, response):
        """Reserve an already-disarmed owner while camera services are restarted."""
        response.success = False
        try:
            state = self._read_state()
            process, generation = self._process, self._owner_started
            if request.operation == "verify":
                # Resolve only OS state; a later explicit verified STOP clears the fault barrier.
                if (not isinstance(state, dict) or state.get("arm_id") != self._expected_arm_id or
                        state.get("calibration_sha256") != self._expected_calibration_sha256 or
                        state.get("owner_source_sha256") != self._owner_source_expected or
                        state.get("torque") != {j: 0 for j in TELEOP_JOINTS} or
                        type(state.get("torque_monotonic")) not in (int, float) or
                        not 0 <= time.monotonic()-state["torque_monotonic"] <= .25):
                    raise RuntimeError("external job verification requires original fresh all-six OFF owner proof")
                proof = self._operator_restart_verify(request.token)
                response.success = True
                response.token = request.token
                response.message = json.dumps(proof)
                return response
            if request.operation == "restart":
                self._assert_owner_reservation("operator_maintenance", process, generation)
                self._prepare_owner_state(state, process, generation, torque=0, nonactuating_maintenance=True)
                job_id = self._operator_restart_begin(request.token, request.unit)
                response.success = True
                response.token = request.token
                response.message = json.dumps({"job_id": job_id, "state": "pending"})
                return response
            if request.operation and (request.operation != "station_reset" or not request.enable):
                raise RuntimeError("unknown operator maintenance operation")
            with self._active_lock:
                if request.enable:
                    if request.token:
                        raise RuntimeError("new maintenance reservation must not supply a token")
                    if self._active_goal is not None or self._teleop_state != "disabled":
                        raise RuntimeError("owner must be idle before maintenance")
                    self._prepare_owner_state(state, process, generation, torque=0, nonactuating_maintenance=True)
                    self._operator_maintenance_token = uuid.uuid4().hex
                    self._operator_maintenance_purpose = "station_reset" if request.operation == "station_reset" else "maintenance"
                    self._operator_maintenance_deadline = time.monotonic()+(120. if self._operator_maintenance_purpose == "station_reset" else 60.)
                    self._active_goal = "operator_maintenance"
                else:
                    if self._operator_external_job is not None:
                        raise RuntimeError("external systemd job outcome is unresolved; reservation cannot release")
                    if (not self._active_goal_is("operator_maintenance") or not request.token or
                            request.token != self._operator_maintenance_token):
                        raise RuntimeError("operator maintenance token does not own the reservation")
                    self._prepare_owner_state(state, process, generation, torque=0, nonactuating_maintenance=True)
                    self._active_goal = None
                response.token = self._operator_maintenance_token
                purpose = getattr(self, "_operator_maintenance_purpose", None)
                if not request.enable:
                    self._operator_maintenance_token = None
                    self._operator_maintenance_purpose = None
                    self._operator_maintenance_deadline = None
            response.success = True
            response.message = json.dumps({"state": "reserved" if request.enable else "released", "token": response.token, "purpose": purpose})
        except (RuntimeError, ValueError, TypeError, OSError) as error:
            response.message = str(error)
        return response

    def _expire_operator_modes(self):
        with self._state_cache_lock:
            state = self._cached_owner_state
            generation = self._cached_owner_generation
        now = time.monotonic()
        fresh_off = (isinstance(state, dict) and generation == self._owner_started and
                     self._process is not None and self._process.poll() is None and
                     state.get("arm_id") == self._expected_arm_id and
                     state.get("calibration_sha256") == self._expected_calibration_sha256 and
                     state.get("owner_source_sha256") == self._owner_source_expected and
                     state.get("observe_only") is False and
                     type(state.get("monotonic")) in (int, float) and 0 <= now-state["monotonic"] <= .25 and
                     type(state.get("torque_monotonic")) in (int, float) and 0 <= now-state["torque_monotonic"] <= .25 and
                     state.get("torque") == {j: 0 for j in TELEOP_JOINTS} and
                     state.get("native_damping_restore_pending") is False)
        with self._active_lock:
            if fresh_off and self._active_goal_is("native_damping") and state.get("native_damping") is None:
                self._active_goal = None
            if (self._active_goal_is("operator_maintenance") and self._operator_maintenance_deadline is not None and
                    now >= self._operator_maintenance_deadline):
                if (self._operator_external_job is not None or
                        getattr(self, "_operator_maintenance_purpose", None) == "station_reset"):
                    self._active_goal = "operator_maintenance_unknown"
                    self._stop_latched = True
                    self._mark_owner_fault("station_reset_acceptance_expired" if
                        getattr(self, "_operator_maintenance_purpose", None) == "station_reset" else
                        "external_restart_lease_expired")
                elif fresh_off:
                    self._active_goal = None
                    self._operator_maintenance_token = None
                    self._operator_maintenance_purpose = None
                    self._operator_maintenance_deadline = None

    @_control_callback
    def _rearm_motion_profile(self, _request, response):
        response.success = False
        reservation = "rearm_motion_profile"
        process, generation = self._process, self._owner_started
        reserved = delivery_possible = False
        try:
            state = self._read_state()
            if self._prepare_owner_state(state, process, generation, torque=0) != "normal" or state.get("native_damping_restore_pending") is not False:
                raise RuntimeError("profile rearm requires clean fresh normal all-six OFF")
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise RuntimeError("profile rearm requires an idle owner")
                self._active_goal = reservation
                reserved = True
            request_id = uuid.uuid4().hex
            delivery_possible = True
            reply = self._reserved_owner_reply({"command": reservation, "request_id": request_id},
                                               process, generation, time.monotonic()+5., reservation)
            if reply.get("ok") is not True or reply.get("command") != reservation or reply.get("request_id") != request_id:
                raise RuntimeError("profile rearm acknowledgement is not correlated")
            final = reply.get("state")
            if self._prepare_owner_state(final, process, generation, torque=0) != "normal" or final.get("motion_profile_rearm_required") is not False:
                raise RuntimeError("profile rearm did not confirm armed profile and all-six OFF")
            response.success = True
            response.message = "configured motion profile armed; all six joints remain torque OFF"
        except (RuntimeError, ValueError, TypeError, OSError, TimeoutError) as error:
            response.message = str(error)
            if delivery_possible:
                self._mark_owner_fault("profile_rearm_result_unknown")
        finally:
            if reserved:
                with self._active_lock:
                    if self._active_goal_is(reservation):
                        self._active_goal = None
        return response

    @_control_callback
    def _native_damping_inventory(self, _request, response):
        response.success = False
        reservation = "native_damping_inventory"
        process, generation = self._process, self._owner_started
        reserved = delivery_possible = False
        try:
            state = self._read_state()
            self._prepare_owner_state(state, process, generation)
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise RuntimeError("firmware inventory requires an idle owner")
                self._active_goal = reservation
                reserved = True
            delivery_possible = True
            result = self._reserved_owner_reply("native_damping_inventory", process, generation,
                                                time.monotonic()+2., reservation)
            if result.get("ok") is not True or result.get("command") != "native_damping_inventory":
                raise RuntimeError("native damping inventory was not acknowledged")
            final = result.get("state")
            self._prepare_owner_state(final, process, generation)
            if final.get("torque") != state.get("torque"):
                raise RuntimeError("torque changed during passive firmware inventory")
            response.success = True
            response.message = json.dumps({k: result[k] for k in ("firmware", "capability_enabled", "firmware_allowed", "physically_qualified")})
        except (RuntimeError, ValueError, TypeError, KeyError, OSError, TimeoutError) as error:
            response.message = str(error)
            if delivery_possible:
                self._mark_owner_fault("native_damping_inventory_result_unknown")
        finally:
            if reserved:
                with self._active_lock:
                    if self._active_goal_is(reservation):
                        self._active_goal = None
        return response

    @_control_callback
    def _enable_native_damping(self, _request, response):
        """Explicit manual 60-second native damping lease, or explicit renewal."""
        response.success = False
        process, generation = self._process, self._owner_started
        reserved = False
        try:
            state = self._read_state()
            if not isinstance(state, dict) or state.get("native_damping_supported") is not True:
                raise RuntimeError("native damping capability/firmware allowlist is not enabled")
            phase = state.get("native_damping")
            if phase is None and self._prepare_owner_state(state, process, generation, torque=0) != "normal":
                raise RuntimeError("native damping requires fresh normal six-joint torque OFF")
            with self._active_lock:
                renewing = self._active_goal_is("native_damping") and isinstance(phase, dict)
                if self._active_goal is not None and not renewing:
                    raise RuntimeError("another owner operation is active")
                if self._teleop_state != "disabled":
                    raise RuntimeError("teleoperation must be disabled before damping")
                self._active_goal = "native_damping"
                reserved = True
            self._assert_owner_reservation("native_damping", process, generation)
            if not renewing and self._prepare_owner_state(state, process, generation, torque=0) != "normal":
                raise RuntimeError("native damping requires fresh normal six-joint torque OFF")
            request_id = uuid.uuid4().hex
            command = "native_damping_renew" if renewing else "native_damping"
            reply = self._reserved_owner_reply({"command": command, "request_id": request_id},
                                               process, generation, time.monotonic()+3., "native_damping")
            final = reply.get("state") if isinstance(reply, dict) else None
            final_phase = final.get("native_damping") if isinstance(final, dict) else None
            now = time.monotonic()
            if (not isinstance(reply, dict) or reply.get("ok") is not True or reply.get("command") != command or
                    reply.get("request_id") != request_id or not isinstance(final_phase, dict) or
                    final_phase.get("phase") != "active" or final_phase.get("request_id") != request_id or
                    final.get("torque") != {j: 2 for j in TELEOP_JOINTS} or
                    final.get("arm_id") != self._expected_arm_id or
                    final.get("calibration_sha256") != self._expected_calibration_sha256 or
                    final.get("owner_source_sha256") != self._owner_source_expected or
                    type(final.get("monotonic")) not in (int, float) or not generation <= final["monotonic"] <= now or
                    now-final["monotonic"] > .25 or
                    type(final_phase.get("deadline_monotonic")) not in (int, float) or
                    not now < final_phase["deadline_monotonic"] <= now+60.):
                raise RuntimeError("native damping acknowledgement not correlated to fresh measured owner state")
            self._assert_owner_reservation("native_damping", process, generation)
            self._cache_owner_state(final)
            response.success = True
            response.message = json.dumps({"request_id": request_id, "lease_remaining_s": final_phase["deadline_monotonic"]-now,
                                           "torque": final["torque"], "native_damping": final_phase,
                                           "physically_qualified": False})
        except (RuntimeError, ValueError, TypeError, OSError, TimeoutError) as error:
            response.message = str(error)
            if reserved:
                self._mark_owner_fault("native_damping_result_unknown")
                try:
                    self._send({"command": "stop", "request_id": uuid.uuid4().hex})
                except (RuntimeError, OSError, TimeoutError):
                    pass
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
            reply_sequence = response.get("sequence")
            inflight_matches = (self._teleop_inflight and
                                (response.get("ok") is True or response.get("ok") is False) and
                                ((type(reply_sequence) is int and
                                  reply_sequence == self._teleop_inflight_sequence) or
                                 (response.get("ok") is False and type(reply_sequence) is not int)))

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
                self._teleop_waiting_ack = None
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
                if (not self._teleop_inflight or self._teleop_session != session_id or
                        (type(reply_sequence) is int and
                         reply_sequence != self._teleop_inflight_sequence)):
                    return True
                wakeup = (response.get("ok") is True and
                          self._teleop_state == "active" and
                          type(response.get("sequence")) is int and
                          response["sequence"] == self._teleop_inflight_sequence and
                          getattr(self, "_teleop_waiting_ack", None) ==
                          (session_id, response["sequence"]))
                self._teleop_waiting_ack = None
                self._teleop_inflight = False
                self._teleop_inflight_sequence = None
                if not response.get("ok") and self._teleop_state == "active":
                    self._teleop_state = "stopping"
                    self._active_goal = "teleop_stopping"
                    self._teleop_stop_reason = "owner_rejected_step"
                    self._publish_teleop_status("stopping", self._teleop_stop_reason,
                                                error=response.get("error", "unknown"))
                self._teleop_step_done.set()
            if wakeup:
                self._wake_teleop_executor()
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

    def _wake_teleop_executor(self):
        """Coalesce successful ACKs without executing control on the reader thread."""
        lock = getattr(self, "_teleop_wakeup_lock", None)
        if lock is None:  # offline harnesses do not own ROS entities
            return
        with lock:
            if self._closed or self._teleop_ack_guard is None:
                return
            try:
                self._teleop_ack_guard.trigger()
            except RuntimeError:
                # Context shutdown may invalidate the handle before destroy_node.
                # The periodic timer must never be replaced by an inline retry.
                return

    def _clear_teleop(self):
        reason = self._teleop_stop_reason or "teleoperation_disabled"
        with self._active_lock:
            if self._active_goal_is("teleop", "teleop_starting", "teleop_stopping"):
                self._active_goal = None
            self._teleop_state = "disabled"
            self._teleop_session = None
            self._teleop_origin = None
            self._teleop_follower_start = None
            self._teleop_inflight = False
            self._teleop_inflight_sequence = None
            self._teleop_waiting_ack = None
            self._teleop_stop_sent = False
            self._teleop_stop_reason = None
        self._publish_teleop_status("disabled", reason)

    def _publish_teleop_status(self, state, reason, **details):
        status = {"schema": 1, "stamp_s": time.monotonic(),
                  "state": str(state), "reason": str(reason),
                  "teleop_mode": getattr(self, "_teleop_mode", "relative")}
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

    def _teleop_start_error(self, state, process, generation, *, after=None, torque=None):
        """Admit a fresh normal current pose independently of sorting home proof."""
        try:
            classification = self._prepare_owner_state(state, process, generation, after=after, torque=torque)
            if classification != "normal":
                raise RuntimeError("teleoperation requires NORMAL current follower bounds")
            bounds = state["arm_bounds"]
            if any(not bounds["admitted_min"][joint] <= state["raw"][joint] <=
                       bounds["admitted_max"][joint] for joint in TELEOP_JOINTS):
                raise RuntimeError("teleoperation requires current raw pose inside admitted bounds")
            if len(set(state["torque"].values())) != 1:
                raise RuntimeError("follower torque state must be uniformly verified on or off")
            pose = state.get("pose")
            if (not isinstance(pose, dict) or set(pose) != set(TELEOP_JOINTS) or
                    any(type(pose[joint]) not in (int, float) or not math.isfinite(pose[joint])
                        for joint in TELEOP_JOINTS) or not 0 <= pose["gripper"] <= 100):
                raise RuntimeError("teleoperation requires a finite measured six-joint follower pose")
        except (RuntimeError, ValueError, KeyError, TypeError) as error:
            return str(error)
        return None

    @_control_callback
    def _set_teleop_absolute_mode(self, request, response):
        """Select mapping only while disabled; never send a joint command."""
        response.success = False
        if type(request.data) is not bool:
            response.message = "absolute mode selection requires a boolean"
            return response
        process, generation = self._process, self._owner_started
        state = self._read_state()
        with self._active_lock:
            if self._teleop_state != "disabled" or self._active_goal is not None:
                response.message = "disable teleoperation before changing mode; the arm will hold its measured pose"
                return response
            error = self._teleop_start_error(state, process, generation)
            if error:
                response.message = error
                return response
            self._teleop_mode = "absolute" if request.data else "relative"
            self._teleop_origin = None
            self._teleop_follower_start = None
        response.success = True
        response.message = "teleop mode selected: " + self._teleop_mode + "; enable captures a fresh current pose"
        return response

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
            if not _teleop_leader_readiness(latest, time.monotonic(),
                    float(self.get_parameter("teleop_pose_stale_s").value))[0]:
                self._clear_teleop()
                response.success = False
                response.message = "no fresh calibrated leader pose is available"
                return response
            process, generation = self._process, self._owner_started
            cohort = (self._expected_arm_id, self._expected_calibration_sha256,
                      dict(self._owner_source_expected) if isinstance(self._owner_source_expected, dict) else None)
            follower_state = self._read_state()
            start_error = self._teleop_start_error(follower_state, process, generation)
            if start_error:
                self._clear_teleop()
                response.success = False
                response.message = start_error
                return response
            session_id = uuid.uuid4().hex
            with self._active_lock:
                self._teleop_session = session_id
            with self._teleop_condition:
                self._teleop_acknowledgements.pop("teleop_start", None)
            command_sent = False
            acknowledged = False
            try:
                requested = time.monotonic()
                self._send({"command": "teleop_start", "session_id": session_id,
                            "mode": self._teleop_mode})
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
                start_error = self._teleop_start_error(state, process, generation,
                                                       after=requested, torque=1)
                if start_error:
                    raise RuntimeError(start_error)
                if cohort != (self._expected_arm_id, self._expected_calibration_sha256, self._owner_source_expected):
                    raise RuntimeError("owner identity/calibration/source cohort changed during teleop start")
                with self._teleop_latest_lock:
                    start_reference = dict(self._teleop_latest) if self._teleop_latest else None
                if not _teleop_leader_readiness(start_reference, time.monotonic(),
                        float(self.get_parameter("teleop_pose_stale_s").value))[0]:
                    raise RuntimeError("leader pose became stale while waiting for owner start acknowledgement")
                with self._active_lock:
                    self._teleop_session = session_id
                    self._teleop_origin = start_reference["pose"]
                    self._teleop_follower_start = {joint: float(baseline[joint]) for joint in TELEOP_JOINTS}
                    self._teleop_last_source_sequence = start_reference["sequence"]
                    self._teleop_owner_sequence = -1
                    self._teleop_inflight = False
                    self._teleop_waiting_ack = None
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
                        self._mark_owner_fault("_set_teleop_enabled")
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
            self._teleop_waiting_ack = None
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

    def _mark_owner_fault(self, reason, *, held_context=None):
        """Every new generic fault revokes any earlier jaw acknowledgment authority."""
        with self._owner_fault_lock:
            previously_faulted = self._owner_faulted
            self._owner_fault_serial += 1
            self._owner_faulted = True
            self._held_jaw_fault = None
            if held_context is not None:
                if reason != "verified_failed_held_jaw":
                    raise ValueError("only a verified held-jaw failure may install acknowledgment context")
                process, generation, frozen = held_context
                # A concurrent STOP or generic fault cannot acquire jaw-ACK authority.
                if (not previously_faulted and not self._closed and not self._stop_latched
                        and not self._cancel_before_start and self._process is process
                        and self._owner_started == generation):
                    self._held_jaw_fault = (self._owner_fault_serial, process, generation, frozen)

    def _clear_owner_fault(self, reason, *, expected_context=None):
        with self._owner_fault_lock:
            if expected_context is not None and (
                    not self._owner_faulted or self._held_jaw_fault is not expected_context
                    or expected_context[0] != self._owner_fault_serial or self._stop_latched
                    or self._cancel_before_start or self._closed):
                return False
            self._owner_fault_serial += 1
            self._owner_faulted = False
            self._held_jaw_fault = None
            return True

    def _held_jaw_ack_state(self, state, process, generation, context, *, after=None, acknowledged=False):
        """ACK-only validation. Ordinary _prepare_owner_state still rejects all faults."""
        with self._owner_fault_lock:
            if (not self._owner_faulted or self._held_jaw_fault is not context
                    or context[0] != self._owner_fault_serial):
                raise RuntimeError("held jaw fault context was replaced or revoked")
        frozen = json.loads(context[3])
        now = time.monotonic()
        if (self._closed or self._external_state_only or self._reconnect_observe or self._writer_faulted
                or self._stop_latched or self._cancel_before_start
                or self._process is not process or process.poll() is not None
                or context[1] is not process or context[2] != generation
                or self._owner_started != generation
                or self._expected_arm_id != frozen["arm_id"]
                or self._expected_calibration_sha256 != frozen["calibration_sha256"]
                or self._owner_source_expected != frozen["owner_source_sha256"]
                or list(self._gripper_raw_limits) != frozen["jaw_limits"]
                or not isinstance(state, dict) or state.get("observe_only") is not False
                or state.get("arm_id") != frozen["arm_id"]
                or state.get("calibration_sha256") != frozen["calibration_sha256"]
                or state.get("owner_source_sha256") != frozen["owner_source_sha256"]
                or state.get("owner_session_id") != frozen["hold"]["owner_session_id"]
                or state.get("component_restore_pending") is not False
                or state.get("component_command") != frozen["component"]
                or state.get("profile_registers") != frozen["profile"]
                or frozen["held_goals"] != frozen["hold"]["held_goals_raw"]
                or any(frozen["held_goals"][j] != frozen["component"]["preserved_body_goals"][j]
                       for j in ARM_JOINTS)
                or state.get("failed_endpoint_hold") != (None if acknowledged else frozen["hold"])):
            raise RuntimeError("fresh unchanged owner/failed-jaw preservation proof is required")
        lower = max(generation, now-min(.1, float(self.get_parameter("state_freshness_s").value)),
                    after if after is not None else generation)
        for key in ("monotonic", "torque_monotonic"):
            stamp = state.get(key)
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or not lower <= stamp <= now:
                raise RuntimeError("held jaw acknowledgment requires original fresh measurements")
        for key, high in (("raw",4095),("torque",1)):
            values = state.get(key)
            if (not isinstance(values,dict) or set(values) != set(TELEOP_JOINTS)
                    or any(type(v) is not int or not 0 <= v <= high for v in values.values())):
                raise RuntimeError("held jaw acknowledgment requires exact six-joint measurements")
        pose = state.get("pose")
        if (not isinstance(pose,dict) or any(type(pose.get(j)) not in (int,float)
                or not math.isfinite(pose[j]) for j in ARM_JOINTS)):
            raise RuntimeError("held jaw acknowledgment requires finite measured body pose")
        bounds = _arm_bounds_observation(state)
        hold = frozen["hold"]
        delta = frozen["command"]["delta_counts"]
        start = frozen["entry_raw"]["gripper"]
        target, held = hold["target_raw"]["gripper"], hold["held_raw"]["gripper"]
        direction = 1 if delta > 0 else -1
        if (bounds is None or bounds["classification"] != "normal" or any(v != 1 for v in state["torque"].values())
                or type(delta) is not int or not 0 < abs(delta) <= 16 or target-start != delta
                or abs(target-held) > 4 or not 1 <= direction*(held-start) <= abs(delta)+4
                or any(abs(state["raw"][j]-hold["body_anchor_raw"][j]) > 2 for j in ARM_JOINTS)
                or abs(state["raw"]["gripper"]-held) > 2
                or abs(state["raw"]["gripper"]-target) > 4
                or direction*(state["raw"]["gripper"]-start) < 1):
            raise RuntimeError("held jaw acknowledgment bounds, anchor, progress or torque guard failed")
        return frozen

    def _assert_owner_reservation(self, reservation, process, generation):
        with self._active_lock:
            own_prepare = (reservation is not None and reservation is self._prepare_reservation and
                           self._active_goal_is("prepare"))
            own_operation = (isinstance(reservation, str) and self._active_goal_is(reservation))
            if (not (own_prepare or own_operation) or self._closed or self._stop_latched or
                    self._cancel_before_start or self._process is not process or
                    self._owner_started != generation or process.poll() is not None):
                raise RuntimeError("owner reservation cancelled, stopped or replaced")
            if own_prepare and self._prepare_cohort != (
                    self._expected_arm_id, self._expected_calibration_sha256, self._owner_source_expected):
                raise RuntimeError("owner identity/calibration/source cohort changed during prepare")

    def _reserved_owner_reply(self, command, process, generation, deadline, reservation):
        """Exactly one existing owner command under the caller's reservation."""
        self._assert_owner_reservation(reservation, process, generation)
        while True:
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break
        self._assert_owner_reservation(reservation, process, generation)
        if reservation is self._prepare_reservation:
            # Conservative: any exception once submission starts has unknown delivery.
            self._prepare_delivery_possible = True
        payload = {"command": command} if isinstance(command, str) else command
        self._send(payload)
        while True:
            self._assert_owner_reservation(reservation, process, generation)
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError(payload["command"] + " result unknown; verified stop required")
            try:
                reply = self._response_queue.get(timeout=min(.05, remaining))
                self._assert_owner_reservation(reservation, process, generation)
                if payload["command"] == "joint_diagnostics" and (
                        not isinstance(reply, dict) or reply.get("command") != "joint_diagnostics"):
                    raise RuntimeError("joint diagnostic response kind mismatch; owner result unknown")
                return reply
            except queue.Empty:
                continue

    def _prepare_owner_state(self, state, process, generation, *, after=None, torque=None,
                             nonactuating_maintenance=False):
        now = time.monotonic()
        freshness = min(.1, float(self.get_parameter("state_freshness_s").value))
        if (self._closed or self._external_state_only or self._process is not process or
                process is None or process.poll() is not None or
                type(generation) not in (int, float) or not math.isfinite(generation) or generation <= 0 or
                self._owner_started != generation or self._reconnect_observe or self._owner_faulted or
                self._stop_latched or self._cancel_before_start or not isinstance(state, dict) or
                state.get("observe_only") is not False or state.get("fault_observation") is not None or
                state.get("arm_id") != self._expected_arm_id or
                state.get("calibration_sha256") != self._expected_calibration_sha256 or
                not isinstance(self._owner_source_expected, dict) or not self._owner_source_expected or
                state.get("owner_source_sha256") != self._owner_source_expected):
            raise RuntimeError("fresh matching non-observing owner without fault/stop is required")
        lower = max(generation, now-freshness, after if after is not None else generation)
        for key in ("monotonic", "torque_monotonic"):
            stamp = state.get(key)
            if type(stamp) not in (int, float) or not math.isfinite(stamp) or not lower <= stamp <= now:
                raise RuntimeError("prepare requires original fresh position/torque acquisition")
        for key, upper in (("raw", 4095), ("torque", 1)):
            values = state.get(key)
            if (not isinstance(values, dict) or set(values) != set(TELEOP_JOINTS) or
                    any(type(v) is not int or not 0 <= v <= upper for v in values.values())):
                raise RuntimeError("prepare requires exact six-joint measurements")
        if torque is not None and any(v != torque for v in state["torque"].values()):
            raise RuntimeError("prepare completion lacks verified torque-on hold")
        bounds = _arm_bounds_observation(state)
        # Perception-unit maintenance never commands the arm. A real but out-of-range
        # passive pose may reserve/release it only while every joint is verified OFF.
        if bounds is None or (bounds["classification"] not in {"normal", "bounded_reentry_required"} and
                not (nonactuating_maintenance and torque == 0 and bounds["classification"] == "unrecoverable")):
            raise RuntimeError("owner bounds classification is unknown or unrecoverable")
        return bounds["classification"]

    def _prepare_configuration(self):
        # Finite-operation admission only; status/ticks never read configuration.
        path = self._prepare_config_path
        if path is None:
            raise RuntimeError("prepare requires trusted station configuration")
        config = json.loads(Path(path).read_text())
        if not isinstance(config, dict):
            raise ValueError("prepare station configuration must be an object")
        arm = config.get("arm")
        if not isinstance(arm, dict) or not isinstance(arm.get("calibration"), dict):
            raise ValueError("prepare arm and calibration configuration must be objects")
        if (any(config.get(k) is not True for k in ("enabled", "commissioned")) or
                type(config.get("automatic_start")) is not bool or
                arm.get("id") != self._expected_arm_id or
                arm.get("calibration", {}).get("sha256") != self._expected_calibration_sha256):
            raise RuntimeError("prepare station configuration/identity is not commissioned and admitted")

    def _prepare_stop(self, process, generation):
        """Physical stop ACK only. Public stop alone clears quiescent barriers."""
        self._stop_latched = True
        self._mark_owner_fault("_prepare_stop")
        self._home_completed_owner_started = None
        timeout = float(self.get_parameter("stop_timeout_s").value)
        if not math.isfinite(timeout) or not 0 < timeout <= 10:
            raise RuntimeError("invalid stop cleanup deadline; stop remains latched")
        deadline = time.monotonic()+timeout
        owns_stop = self._stop_lock.acquire(blocking=False)
        request_id = None
        try:
            if owns_stop:
                if not self._write_lock.acquire(timeout=max(0., deadline-time.monotonic())):
                    raise TimeoutError("prepare stop writer unavailable; stop remains latched")
                try:
                    with self._stop_condition:
                        request_id = uuid.uuid4().hex
                        requested = time.monotonic()
                        self._stop_pending, self._stop_ack = request_id, None
                        self._stop_requested = requested
                    self._write_stop({"command": "stop", "request_id": request_id}, deadline)
                finally:
                    self._write_lock.release()
            with self._stop_condition:
                while True:
                    # A public stop may own the lock but still be waiting for
                    # the writer. Never borrow an old ACK before its new ID exists.
                    pending, ack, requested = self._stop_pending, self._stop_ack, self._stop_requested
                    if (pending is not None and requested is not None and ack is not None and
                            ack.get("request_id") == pending and
                            (not owns_stop or pending == request_id)):
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError("prepare stop acknowledgement unresolved; stop remains latched")
                    self._stop_condition.wait(min(.02, deadline-time.monotonic()))
            state = ack.get("state", ack.get("final_state"))
            now = time.monotonic()
            if (ack.get("ok") is not True or ack.get("stopped") is not True or
                    self._process is not process or self._owner_started != generation or
                    process.poll() is not None or not isinstance(state, dict) or
                    state.get("arm_id") != self._expected_arm_id or
                    state.get("calibration_sha256") != self._expected_calibration_sha256 or
                    state.get("owner_source_sha256") != self._owner_source_expected):
                raise RuntimeError("prepare stop lacks current identity/source-bound acknowledgement")
            for key in ("monotonic", "torque_monotonic"):
                stamp = state.get(key)
                if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                        not max(requested, now-min(.1, timeout)) <= stamp <= now):
                    raise RuntimeError("prepare stop lacks fresh actual acquisition")
            for key, upper in (("raw", 4095), ("torque", 0)):
                values = state.get(key)
                if (not isinstance(values, dict) or set(values) != set(TELEOP_JOINTS) or
                        any(type(v) is not int or not 0 <= v <= upper for v in values.values())):
                    raise RuntimeError("prepare stop lacks six-joint verified disarm")
            self._cache_owner_state(state, generation=generation, process=process)
            return True
        finally:
            if owns_stop:
                with self._stop_condition:
                    if self._stop_pending == request_id:
                        self._stop_pending = None
                        self._stop_requested = None
                self._stop_lock.release()

    @_control_callback
    def _prepare(self, _request, response):
        """One follower reservation, existing recovery/home commands, no retry."""
        response.success = False
        process, generation = self._process, self._owner_started
        reservation = object()
        reserved = False
        delivery_possible = False
        stop_verified = None
        phases = []
        try:
            self._prepare_configuration()
            initial = self._read_state()
            classification = self._prepare_owner_state(initial, process, generation)
            expected_raw = dict(initial["raw"])
            if (self._gripper_jaw is None or not isinstance(self._saved_home_pose, dict) or
                    set(self._saved_home_pose) != set(TELEOP_JOINTS) or
                    any(type(v) not in (int, float) or not math.isfinite(v)
                        for v in self._saved_home_pose.values()) or
                    type(self._saved_home_tolerance) not in (int, float) or
                    not math.isfinite(self._saved_home_tolerance) or self._saved_home_tolerance <= 0):
                raise RuntimeError("qualified jaw and measured home configuration are required")
            home_timeout = float(self.get_parameter("home_timeout_s").value)
            if not math.isfinite(home_timeout) or not 0 < home_timeout <= 90:
                raise ValueError("home_timeout_s must be in (0, 90] seconds")
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise RuntimeError("another follower operation is active")
                self._active_goal = "prepare"
                self._prepare_reservation = reservation
                self._prepare_cohort = (self._expected_arm_id, self._expected_calibration_sha256,
                                        dict(self._owner_source_expected))
                self._prepare_phase = "admission"
                self._prepare_delivery_possible = False
                reserved = True
            self._assert_owner_reservation(reservation, process, generation)
            admitted = self._read_state()
            self._prepare_owner_state(admitted, process, generation)
            if admitted["raw"] != expected_raw:
                raise RuntimeError("owner raw changed during prepare admission")
            if (classification == "normal" and
                    self._sorting_ready_value(admitted, reservation=reservation) and
                    _measured_home_error(admitted, arm_id=self._expected_arm_id,
                        calibration_sha256=self._expected_calibration_sha256,
                        home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance) is None):
                fresh = admitted
                result_state = "already_ready"
            else:
                self._home_completed_owner_started = None
                if classification == "bounded_reentry_required":
                    if self._recovery_cache_config is None:
                        raise RuntimeError("qualified camera recovery configuration is unavailable")
                    self._prepare_phase = "recover_camera"
                    began = time.monotonic()
                    reply = self._reserved_owner_reply("recover_camera", process, generation,
                                                       began+8., reservation)
                    completion = self._prepare_completion(reply, "camera_recovery_complete",
                                                         process, generation, began)
                    if self._home_completed_owner_started is not None:
                        raise RuntimeError("recovery cannot establish home provenance")
                    self._cache_owner_state(completion, generation=generation, process=process)
                    expected_raw = dict(completion["raw"])
                    phases.append("recover_camera")
                self._assert_owner_reservation(reservation, process, generation)
                pre_home = self._read_state()
                if self._prepare_owner_state(pre_home, process, generation) != "normal":
                    raise RuntimeError("measured home requires owner's normal bounds classification")
                if pre_home["raw"] != expected_raw:
                    raise RuntimeError("held owner raw changed before measured home")
                self._prepare_phase = "home"
                began = time.monotonic()
                reply = self._reserved_owner_reply("home", process, generation,
                                                   began+home_timeout, reservation)
                completion = self._prepare_completion(reply, "home_complete", process, generation, began)
                error = _measured_home_error(completion, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance)
                if error:
                    raise RuntimeError(error)
                self._home_completed_owner_started = generation
                self._cache_owner_state(completion, generation=generation, process=process)
                phases.append("home")
                self._prepare_phase = "readiness"
                self._assert_owner_reservation(reservation, process, generation)
                fresh = self._read_state()
                self._prepare_owner_state(fresh, process, generation, after=began, torque=1)
                if fresh["raw"] != completion["raw"]:
                    raise RuntimeError("measured home raw changed before final readiness")
                error = _measured_home_error(fresh, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance)
                if error:
                    raise RuntimeError(error)
                if not self._sorting_ready_value(fresh, reservation=reservation):
                    raise RuntimeError("prepared follower failed final sorting readiness")
                result_state = "ready"
            self._assert_owner_reservation(reservation, process, generation)
            response.success = True
            response.message = json.dumps(dict(schema=1, kind="arm_prepare", state=result_state,
                home_completed_this_session=True, phases=phases, owner_started_monotonic=generation,
                observation={key: fresh[key] for key in ("arm_id", "calibration_sha256",
                    "owner_source_sha256", "raw", "torque", "monotonic", "torque_monotonic", "arm_bounds")}))
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError, KeyError) as error:
            delivery_possible = reserved and self._prepare_delivery_possible
            if delivery_possible:
                self._prepare_phase = "stopping"
                try:
                    stop_verified = self._prepare_stop(process, generation)
                except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as stop_error:
                    stop_verified = False
                    error.add_note(str(stop_error))
            response.message = json.dumps(dict(schema=1, kind="arm_prepare", state="failed" if delivery_possible else "rejected",
                error=str(error), cleanup_errors=getattr(error, "__notes__", []), phases=phases,
                command_delivery="may_have_been_sent" if delivery_possible else "not_sent",
                stop_verified=stop_verified, home_completed_this_session=False))
        finally:
            if reserved:
                with self._active_lock:
                    if self._prepare_reservation is reservation:
                        if not response.success:
                            self._home_completed_owner_started = None
                        self._prepare_reservation = None
                        self._prepare_cohort = None
                        self._prepare_phase = None
                        self._active_goal = None
        return response

    def _prepare_completion(self, reply, event, process, generation, began):
        if (not isinstance(reply, dict) or reply.get("ok") is not True or
                reply.get("stopped") or reply.get("cancelled")):
            raise RuntimeError(reply.get("error", "owner preparation command failed")
                               if isinstance(reply, dict) else "malformed preparation acknowledgement")
        state = reply.get("state")
        if not isinstance(state, dict) or state.get("event") != event:
            raise RuntimeError("preparation completion event mismatch")
        if self._prepare_owner_state(state, process, generation, after=began, torque=1) != "normal":
            raise RuntimeError("preparation completion remains outside normal bounds")
        return state

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
                if self._active_goal_is("home"):
                    self._active_goal = None
            return response
        deadline = time.monotonic() + timeout_s
        try:
            if self._prepare_config_path is None:
                raise RuntimeError("home requires trusted station configuration")
            config_data = json.loads(Path(self._prepare_config_path).read_text())
            home_pose, home_tolerance = _load_measured_home(config_data,
                Path(str(self.get_parameter("station_root").value)),
                self._expected_arm_id, self._expected_calibration_sha256)
            result = self._reserved_owner_reply("home", process, self._owner_started,
                                                deadline, "home")
            if not isinstance(result, dict):
                raise RuntimeError("ArmSession returned a malformed home acknowledgement")
            home_state = result.get("state")
            if not result.get("ok"):
                raise RuntimeError(result.get("error", result.get("error_type", "ArmSession rejected home")))
            if not isinstance(home_state, dict) or home_state.get("event") != "home_complete":
                raise RuntimeError("ArmSession home acknowledgement did not confirm home_complete")
            mismatch = _measured_home_error(home_state, arm_id=self._expected_arm_id,
                calibration_sha256=self._expected_calibration_sha256,
                home_pose=home_pose, tolerance=home_tolerance)
            if mismatch is not None:
                raise RuntimeError("reloaded measured home completion mismatch: " + str(mismatch))
            response.success = True
            response.message = "measured home completed"
            self._saved_home_pose, self._saved_home_tolerance = home_pose, home_tolerance
            self._home_completed_owner_started = self._owner_started
            self._cache_owner_state(home_state)
        except (BrokenPipeError, OSError, RuntimeError, TimeoutError, ValueError, KeyError, TypeError) as error:
            self._home_completed_owner_started = None
            response.success = False
            response.message = str(error)
        finally:
            with self._active_lock:
                if self._active_goal_is("home"):
                    self._active_goal = None
        return response

    @_control_callback
    def _recover_camera(self, _request, response):
        """Request only the owner's qualified bounded camera re-entry operation."""
        response.success = False
        process, generation = self._process, self._owner_started
        state = self._read_state()
        if (self._external_state_only or self._closed or process is None or
                process.poll() is not None or self._owner_faulted or self._reconnect_observe):
            response.message = "resident ArmSession owner unavailable, faulted or awaiting verified stop"
            return response
        now = time.monotonic()
        freshness = min(.1, float(self.get_parameter("state_freshness_s").value))
        if (not isinstance(state, dict) or type(generation) not in (int, float) or
                not math.isfinite(generation) or generation <= 0 or
                state.get("observe_only") is not False or state.get("fault_observation") is not None or
                state.get("arm_id") != self._expected_arm_id or
                state.get("calibration_sha256") != self._expected_calibration_sha256 or
                not isinstance(self._owner_source_expected, dict) or
                state.get("owner_source_sha256") != self._owner_source_expected or
                any(type(state.get(key)) not in (int, float) or
                    not math.isfinite(state[key]) or
                    not max(generation, now-freshness) <= state[key] <= now
                    for key in ("monotonic", "torque_monotonic"))):
            response.message = "fresh identity-matched non-observing owner state is required"
            return response
        with self._active_lock:
            if (self._active_goal is not None or self._teleop_state != "disabled" or
                    self._process is not process or self._owner_started != generation):
                response.message = "another owner operation or generation is active"
                return response
            self._active_goal = "recover_camera"
            self._home_completed_owner_started = None
        started = time.monotonic()
        try:
            result = self._reserved_owner_reply("recover_camera", process, generation,
                                                started+8., "recover_camera")
            if not isinstance(result, dict):
                raise RuntimeError("camera recovery acknowledgement is not an object")
            if (result.get("ok") is not True or result.get("cancelled") or result.get("stopped")):
                raise RuntimeError(result.get("error", "camera recovery did not complete"))
            completion = result.get("state")
            now = time.monotonic()
            freshness = min(.1, float(self.get_parameter("state_freshness_s").value))
            if not isinstance(completion, dict):
                raise RuntimeError("camera recovery completion state is missing")
            raw, torque = completion.get("raw"), completion.get("torque")
            stamps = (completion.get("monotonic"), completion.get("torque_monotonic"))
            if (self._process is not process or self._owner_started != generation or
                    process.poll() is not None or self._stop_latched or self._closed or
                    completion.get("event") != "camera_recovery_complete" or
                    completion.get("observe_only") is not False or
                    completion.get("owner_source_sha256") != self._owner_source_expected or
                    completion.get("arm_id") != self._expected_arm_id or
                    completion.get("calibration_sha256") != self._expected_calibration_sha256 or
                    any(type(stamp) not in (int, float) or not math.isfinite(stamp) or
                        not max(started, now-freshness) <= stamp <= now for stamp in stamps) or
                    not isinstance(raw, dict) or set(raw) != set(TELEOP_JOINTS) or
                    any(type(value) is not int or not 0 <= value <= 4095 for value in raw.values()) or
                    not isinstance(torque, dict) or set(torque) != set(TELEOP_JOINTS) or
                    any(type(value) is not int or value != 1 for value in torque.values())):
                raise RuntimeError("camera recovery lacks fresh generation-bound torque-on held completion")
            self._cache_owner_state(completion, generation=generation, process=process)
            response.success = True
            response.message = json.dumps({"schema": 1, "state": "camera_recovery_complete",
                "home_completed_this_session": False, "raw": raw, "torque": torque})
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as error:
            self._mark_owner_fault("_recover_camera")
            response.message = str(error)
        finally:
            with self._active_lock:
                self._home_completed_owner_started = None
                if self._active_goal_is("recover_camera"):
                    self._active_goal = None
        return response

    def _cache_owner_state(self, state, *, generation=None, process=None):
        """Prune already-read telemetry to bounded fields, bound to its reader generation."""
        state = state if isinstance(state, dict) else {}
        generation = self._owner_started if generation is None else generation
        process = self._process if process is None else process
        def stamp(key):
            value = state.get(key)
            return value if type(value) in (int, float) and math.isfinite(value) else None
        position, torque_stamp = stamp("monotonic"), stamp("torque_monotonic")
        runtime_started = stamp("camera_recovery_runtime_started_monotonic")
        bounds = _arm_bounds_observation(state)
        reason = None
        if type(generation) not in (int, float) or not math.isfinite(generation) or generation <= 0:
            reason = "owner_generation_unconfirmed"
        elif position is None or position < generation:
            reason = "position_sample_precedes_owner_or_invalid"
        elif torque_stamp is None or torque_stamp < generation:
            reason = "torque_sample_precedes_owner_or_invalid"
        retained = {key: state.get(key) if isinstance(state.get(key), str) and
                    len(state[key]) <= 128 else None
                    for key in ("event", "arm_id", "calibration_sha256",
                                "timestamp_utc", "torque_timestamp_utc")}
        fault = state.get("fault_observation")
        retained["fault_observation"] = None if fault is None else {}
        if isinstance(fault, dict):
            retained["fault_observation"] = {key: fault[key] for key in
                ("owner_session_id", "command", "error", "error_type")
                if isinstance(fault.get(key), str) and len(fault[key]) <= 512}
            retained["fault_observation"].update(schema=fault.get("schema") if type(fault.get("schema")) is int else None,
                entered_monotonic=fault.get("entered_monotonic") if type(fault.get("entered_monotonic")) in (int, float) and math.isfinite(fault["entered_monotonic"]) else None)
        session_id = state.get("owner_session_id")
        retained["owner_session_id"] = session_id if isinstance(session_id, str) and len(session_id) <= 128 else None
        retained.update(arm_bounds=bounds, monotonic=position, torque_monotonic=torque_stamp,
                        observe_only=state.get("observe_only") if
                        type(state.get("observe_only")) is bool else None,
                        camera_recovery_runtime_started_monotonic=runtime_started)
        damping = state.get("native_damping")
        valid_damping = (isinstance(damping, dict) and set(damping) == {"phase", "request_id", "deadline_monotonic", "entered_monotonic", "torque_limit"} and
            damping.get("phase") == "active" and isinstance(damping.get("request_id"), str) and len(damping["request_id"]) == 32 and
            all(c in "0123456789abcdef" for c in damping["request_id"]) and
            all(type(damping.get(k)) in (int, float) and math.isfinite(damping[k]) for k in ("deadline_monotonic", "entered_monotonic")) and
            damping["entered_monotonic"] <= damping["deadline_monotonic"] <= damping["entered_monotonic"]+60. and
            type(damping.get("torque_limit")) is int and 1 <= damping["torque_limit"] <= 50)
        retained["native_damping"] = dict(damping) if valid_damping else None
        retained["native_damping_restore_pending"] = state.get("native_damping_restore_pending") is True
        retained["native_damping_supported"] = state.get("native_damping_supported") is True
        retained["motion_profile_rearm_required"] = state.get("motion_profile_rearm_required") is True
        nonce = state.get("camera_recovery_cache_generation")
        retained["camera_recovery_cache_generation"] = (nonce if isinstance(nonce, str) and
            len(nonce) == 32 and all(char in "0123456789abcdef" for char in nonce) else None)
        for key in ("raw", "pose", "torque"):
            values = state.get(key)
            valid = isinstance(values, dict) and set(values) == set(TELEOP_JOINTS)
            if valid and key == "pose":
                valid = all(type(value) in (int, float) and math.isfinite(value)
                            for value in values.values())
            elif valid:
                upper = 4095 if key == "raw" else 2 if valid_damping else 1
                valid = all(type(value) is int and 0 <= value <= upper for value in values.values())
            retained[key] = dict(values) if valid else None
        phase_keys = ("camera_recovery_phase", "camera_recovery_held_raw",
                      "camera_recovery_hold_stamp_monotonic",
                      "camera_recovery_wait_deadline_monotonic")
        phase = state.get("camera_recovery_phase")
        held = state.get("camera_recovery_held_raw")
        hold_stamp = stamp("camera_recovery_hold_stamp_monotonic")
        wait_deadline = stamp("camera_recovery_wait_deadline_monotonic")
        supported = all(key in state for key in phase_keys)
        inactive = (phase is None and held is None and
                    state.get(phase_keys[2]) is None and state.get(phase_keys[3]) is None)
        valid_wait = (phase == "stationary_recapture" and isinstance(held, dict) and
                      set(held) == set(TELEOP_JOINTS) and
                      all(type(value) is int and 0 <= value <= 4095 for value in held.values()) and
                      hold_stamp is not None and wait_deadline is not None and
                      type(generation) in (int, float) and math.isfinite(generation) and
                      generation <= hold_stamp <= wait_deadline)
        retained["camera_recovery_observation"] = {
            "supported": supported, "phase": phase if valid_wait else None,
            "held_raw": dict(held) if valid_wait else None,
            "hold_stamp_monotonic": hold_stamp if valid_wait else None,
            "wait_deadline_monotonic": wait_deadline if valid_wait else None,
            "reason": ("unsupported_by_owner" if not supported else
                       None if inactive or valid_wait else "invalid_owner_observation")}
        sources = state.get("owner_source_sha256")
        expected_sources = self._owner_source_expected
        # This trusted startup map is generated from OWNER_SOURCE_MODULES;
        # its exact registered keyset bounds the copy, independently of received input.
        retained["owner_source_sha256"] = (dict(sources) if
            isinstance(sources, dict) and isinstance(expected_sources, dict) and
            bool(expected_sources) and set(sources) == set(expected_sources) and
            all(isinstance(value, str) and len(value) == 64 for value in sources.values()) else None)
        collector = None
        with self._state_cache_lock:
            if self._owner_started == generation and self._process is process:
                self._cached_owner_state = retained if reason is None else None
                self._cached_owner_generation = generation
                self._cached_owner_reason = reason
                collector = self._recovery_depth_cache
        if (retained["fault_observation"] is not None and self._owner_started == generation and
                self._process is process and retained["owner_source_sha256"] == self._owner_source_expected):
            if not self._owner_faulted:
                self._mark_owner_fault("owner_fault_observation")
            self._home_completed_owner_started = None
        if collector is not None:
            now = time.monotonic()
            admitted = (reason is None and not self._closed and process is not None and
                process.poll() is None and retained["raw"] is not None and
                retained["torque"] is not None and type(retained["observe_only"]) is bool and
                position <= now and torque_stamp <= now and
                retained["arm_id"] == self._expected_arm_id and
                retained["calibration_sha256"] == self._expected_calibration_sha256 and
                retained["owner_source_sha256"] == expected_sources and
                self._recovery_runtime_context(retained, generation) ==
                (collector.cache_generation, collector.runtime_started_monotonic))
            if admitted:
                collector.observe_owner_sample(retained, (process, generation))
            else:
                collector.invalidate_generation(None)

    def _recovery_runtime_context(self, state, generation):
        """Read only the nonce authority already admitted from this owner snapshot."""
        if not isinstance(state, dict):
            return None
        nonce = state.get("camera_recovery_cache_generation")
        started = state.get("camera_recovery_runtime_started_monotonic")
        acquired = state.get("monotonic")
        if (not isinstance(nonce, str) or len(nonce) != 32 or
                any(char not in "0123456789abcdef" for char in nonce) or
                type(generation) not in (int, float) or not math.isfinite(generation) or generation <= 0 or
                type(started) not in (int, float) or not math.isfinite(started) or
                type(acquired) not in (int, float) or not math.isfinite(acquired) or
                not generation <= started <= acquired <= time.monotonic() or
                state.get("arm_id") != self._expected_arm_id or
                state.get("calibration_sha256") != self._expected_calibration_sha256 or
                state.get("owner_source_sha256") != self._owner_source_expected or
                state.get("raw") is None or state.get("torque") is None):
            return None
        return nonce, started

    def _close_recovery_depth_cache(self):
        identity = self._recovery_camera_identity
        if identity is not None:
            identity.set_collector(None)
        with self._state_cache_lock:
            collector, self._recovery_depth_cache = self._recovery_depth_cache, None
        if collector is not None:
            try:
                collector.close()
            except (OSError, RuntimeError) as error:
                self._recovery_cache_error = str(error)
                self.get_logger().warning(f"Recovery depth producer close failed: {error}")
                return False
        return True

    def _configure_recovery_depth_cache(self):
        """Start only a manifest-authorized observation producer, outside control ticks."""
        if (self._closed or self._external_state_only or self._recovery_cache_config is None or
                self._process is None or self._process.poll() is not None):
            return
        process, generation = self._process, self._owner_started
        with self._state_cache_lock:
            state, cached_generation = self._cached_owner_state, self._cached_owner_generation
            current = self._recovery_depth_cache
        context = self._recovery_runtime_context(state, generation)
        if context is None or cached_generation != generation:
            self._recovery_cache_error = "current owner recovery runtime nonce is unavailable"
            return
        if current is not None:
            if (current.cache_generation, current.runtime_started_monotonic) == context:
                return  # One collector owns this nonce for its whole owner lifetime.
            self._recovery_cache_error = "recovery runtime changed within an owner generation"
            current.invalidate_generation(None)
            return
        collector_context = (process, generation, *context)
        with self._state_cache_lock:
            if self._recovery_collector_context is not None:
                self._recovery_cache_error = "collector already created for this owner runtime"
                return
            self._recovery_collector_context = collector_context
        collector = None
        try:
            from .recovery_depth_cache import RecoveryDepthCache
            collector = RecoveryDepthCache(None, trusted_config=self._recovery_cache_config,
                expected_owner_sources=self._owner_source_expected, state_file=self._state_file,
                cache_generation=context[0], runtime_started_monotonic=context[1],
                depth_topic=str(self.get_parameter("recovery_depth_topic").value),
                camera_info_topic=str(self.get_parameter("recovery_camera_info_topic").value))
            collector.invalidate_generation((process, generation))
            if self._recovery_camera_identity is None:
                from .recovery_camera_identity import RecoveryCameraIdentity
                self._recovery_camera_identity = RecoveryCameraIdentity(self,
                    depth_topic=str(self.get_parameter("recovery_depth_topic").value),
                    camera_info_topic=str(self.get_parameter("recovery_camera_info_topic").value))
            self._ensure_recovery_depth_subscriptions()
            with self._state_cache_lock:
                attach = (not self._closed and self._process is process and
                          self._owner_started == generation)
                if attach:
                    self._recovery_depth_cache = collector
                state = self._cached_owner_state
                cached_generation = self._cached_owner_generation
            if not attach:
                collector.close()
                return
            self._recovery_camera_identity.set_collector(collector)
            if state is not None:
                self._cache_owner_state(state, generation=cached_generation, process=process)
        except (OSError, RuntimeError, ValueError, KeyError, TypeError, ImportError) as error:
            if collector is not None:
                with self._state_cache_lock:
                    if self._recovery_depth_cache is collector:
                        self._recovery_depth_cache = None
                collector.close()
            self._recovery_cache_error = str(error)
            self.get_logger().warning(f"Recovery depth producer disabled: {error}")

    def _ensure_recovery_depth_subscriptions(self):
        """One bounded subscription pair survives collector/owner replacement."""
        if len(self._recovery_depth_subscriptions) == 2:
            return
        if not self._recovery_depth_subscriptions:
            self._recovery_depth_group = MutuallyExclusiveCallbackGroup()
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        if not self._recovery_depth_subscriptions:
            self._recovery_depth_subscriptions.append(self.create_subscription(
                Image, str(self.get_parameter("recovery_depth_topic").value),
                self._recovery_depth_received, qos, callback_group=self._recovery_depth_group))
        if len(self._recovery_depth_subscriptions) == 1:
            self._recovery_depth_subscriptions.append(self.create_subscription(
                CameraInfo, str(self.get_parameter("recovery_camera_info_topic").value),
                self._recovery_info_received, qos, callback_group=self._recovery_depth_group))

    def _recovery_depth_received(self, message, message_info):
        collector, identity = self._recovery_depth_cache, self._recovery_camera_identity
        if not self._closed and collector is not None and identity is not None:
            generation = identity.accept('depth', message, message_info, collector=collector)
            if generation is not None:
                collector._on_depth(message, camera_generation=generation)

    def _recovery_info_received(self, message, message_info):
        collector, identity = self._recovery_depth_cache, self._recovery_camera_identity
        if not self._closed and collector is not None and identity is not None:
            generation = identity.accept('info', message, message_info, collector=collector)
            if generation is not None:
                collector._on_info(message, camera_generation=generation)

    def _status(self, _request, response):
        """Observe cached telemetry without I/O; this is not motion admission."""
        now = time.monotonic()
        with self._state_cache_lock:
            state = self._cached_owner_state
            cached_generation = self._cached_owner_generation
            cache_reason = self._cached_owner_reason
            collector = self._recovery_depth_cache
        with self._active_lock:
            process, generation = self._process, self._owner_started
            faulted, barrier = self._owner_faulted, self._stop_latched
            active = self._active_goal
            teleop_state = self._teleop_state
            completed = self._home_completed_owner_started
            observing = self._reconnect_observe
            expected_sources = self._owner_source_expected
        with self._teleop_latest_lock:
            leader_latest = dict(self._teleop_latest) if self._teleop_latest is not None else None
        leader_ready, leader_reason, leader_age = _teleop_leader_readiness(
            leader_latest, now, float(self.get_parameter("teleop_pose_stale_s").value))
        reasons = [cache_reason] if cache_reason else []
        connected = bool(not self._closed and process is not None and process.poll() is None)
        if not connected:
            reasons.append("owner_disconnected")
        state = state if isinstance(state, dict) else {}
        if not state:
            reasons.append("cached_state_missing")
        def age(key):
            stamp = state.get(key)
            if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp > now:
                reasons.append(key + "_missing_or_invalid")
                return None
            return now - stamp
        position_age, torque_age = age("monotonic"), age("torque_monotonic")
        generation_valid = (type(generation) in (int, float) and math.isfinite(generation) and
                            generation > 0 and cached_generation == generation)
        if not generation_valid:
            reasons.append("owner_generation_unconfirmed")
        identity = (bool(self._expected_arm_id) and bool(self._expected_calibration_sha256) and
                    state.get("arm_id") == self._expected_arm_id and
                    state.get("calibration_sha256") == self._expected_calibration_sha256)
        if not identity:
            reasons.append("owner_identity_unconfirmed")
        owner_fault = state.get("fault_observation")
        if owner_fault is not None:
            faulted = True
            reasons.append("owner_fault_observation_requires_explicit_reconnect")
        mode = state.get("observe_only")
        observe_only = True if observing else mode if type(mode) is bool else None
        if observe_only is None:
            reasons.append("observe_only_unknown")
        raw, torque = state.get("raw"), state.get("torque")
        measurements_valid = (isinstance(raw, dict) and set(raw) == set(TELEOP_JOINTS) and
            all(type(value) is int and 0 <= value <= 4095 for value in raw.values()) and
            isinstance(torque, dict) and set(torque) == set(TELEOP_JOINTS) and
            all(type(value) is int and value in ((0, 1, 2) if state.get("native_damping") is not None else (0, 1)) for value in torque.values()))
        if not measurements_valid:
            reasons.append("six_joint_measurements_unconfirmed")
        freshness = float(self.get_parameter("state_freshness_s").value)
        timely = (generation_valid and position_age is not None and torque_age is not None and
                  0 <= position_age <= freshness and 0 <= torque_age <= freshness and
                  state.get("monotonic", -1) >= generation and
                  state.get("torque_monotonic", -1) >= generation)
        if not timely:
            reasons.append("measurements_not_fresh")
        if self._process is not process or self._owner_started != generation:
            connected = False
            reasons.append("owner_changed_during_snapshot")
        actual_sources = state.get("owner_source_sha256")
        if not isinstance(actual_sources, dict):
            actual_sources = None
            reasons.append("owner_source_hashes_missing")
        source_matches = (isinstance(expected_sources, dict) and bool(expected_sources) and
                          actual_sources == expected_sources)
        if not source_matches:
            reasons.append("owner_source_hashes_unconfirmed")
        planning_limit_error = None
        if state and getattr(self, "_planning_calibration", None) is not None:
            try:
                self._planning_limits_for_state(state)
            except (ValueError, KeyError, TypeError) as error:
                planning_limit_error = str(error)
                reasons.append("planning_joint_limits_mismatch: " + planning_limit_error)
        home = bool(connected and generation_valid and identity and not faulted and not barrier and
                    source_matches and observe_only is False and completed is not None and completed == generation)
        trusted = generation_valid and identity and source_matches
        teleop_start_error = ("another owner operation is active" if active is not None else
                              self._teleop_start_error(state, process, generation))
        if teleop_start_error is None and not leader_ready:
            teleop_start_error = leader_reason
        payload = {"schema": 1, "kind": "follower_status", "sampled_monotonic": now,
            "connected": connected, "fresh": bool(connected and generation_valid and identity and
                                                    measurements_valid and timely and source_matches),
            "observe_only": observe_only, "owner_faulted": bool(faulted),
            "fault_observation": owner_fault,
            "recovery_action": "explicit_reconnect_then_stop" if owner_fault is not None else None,
            "stop_latched": bool(barrier), "home_completed_this_session": home,
            "active_operation": active if isinstance(active, str) else "trajectory" if active else None,
            "teleop_state": teleop_state, "teleop_mode": getattr(self, "_teleop_mode", "relative"),
            "owner_started_monotonic": generation if generation_valid else None,
            "teleop_start_ready": teleop_start_error is None,
            "teleop_start_reason": teleop_start_error or "fresh_current_pose",
            "teleop_leader_ready": leader_ready, "teleop_leader_reason": leader_reason,
            "teleop_leader_age_s": leader_age,
            "native_damping": state.get("native_damping") if trusted else None,
            "native_damping_supported": state.get("native_damping_supported") is True and trusted,
            "native_damping_restore_pending": state.get("native_damping_restore_pending") is True,
            "motion_profile_rearm_required": state.get("motion_profile_rearm_required") is True,
            "operator_maintenance_token": self._operator_maintenance_token,
            "operator_maintenance_deadline_monotonic": self._operator_maintenance_deadline,
            "operator_external_job": self._operator_external_job,
            "operator_restart_result": self._operator_restart_result,
            "operator_restart_units": sorted(self._operator_restart_units),
            "operator_station_reset_units": sorted(getattr(self, "_operator_station_reset_units", ())),
            "operator_maintenance_purpose": getattr(self, "_operator_maintenance_purpose", None),
            "position_age_s": position_age, "torque_age_s": torque_age,
            "position_monotonic": state.get("monotonic") if position_age is not None else None,
            "torque_monotonic": state.get("torque_monotonic") if torque_age is not None else None,
            "raw": raw if trusted and measurements_valid else None,
            "pose": state.get("pose") if trusted else None,
            "torque": torque if trusted and measurements_valid else None,
            "arm_id": state.get("arm_id"),
            "calibration_sha256": state.get("calibration_sha256"),
            "owner_source_sha256": actual_sources,
            "expected_owner_source_sha256": expected_sources if isinstance(expected_sources, dict) else None,
            "arm_bounds": state.get("arm_bounds") if trusted else None,
            "planning_joint_limits_ready": planning_limit_error is None and getattr(self, "_planning_contract", None) is not None,
            "planning_joint_limits_error": planning_limit_error,
            "prepare_phase": self._prepare_phase,
            "camera_recovery": (state.get("camera_recovery_observation") if trusted else
                {"supported": False, "phase": None, "held_raw": None,
                 "hold_stamp_monotonic": None, "wait_deadline_monotonic": None,
                 "reason": "owner_unconfirmed"}),
            "camera_identity": (self._recovery_camera_identity.snapshot()
                                if self._recovery_camera_identity is not None else None),
            "recovery_depth_cache": (collector.snapshot()
                if collector is not None else
                {"enabled": False, "reason": self._recovery_cache_error or "not_configured"}),
            "reasons": reasons}
        response.success = True
        response.message = json.dumps(payload, allow_nan=False)
        return response

    @_control_callback
    def _joint_diagnostics(self, _request, response):
        """Serialize a passive six-joint inventory through the resident bus owner."""
        response.success = False
        process, generation = self._process, self._owner_started
        cohort = (self._expected_arm_id, self._expected_calibration_sha256,
                  dict(self._owner_source_expected or {}))
        started = time.monotonic()
        try:
            with self._active_lock:
                if (self._active_goal is not None or self._teleop_state != "disabled" or
                        self._prepare_reservation is not None or self._closed or
                        self._stop_latched or self._cancel_before_start):
                    raise ValueError("joint diagnostics require an unreserved idle owner")
            state = self._read_state()
            try:
                classification = self._prepare_owner_state(state, process, generation)
            except RuntimeError as error:
                # A passive request rejected before delivery cannot poison another operation.
                raise ValueError(str(error)) from error
            if classification != "normal":
                raise ValueError("joint diagnostics require normal owner bounds")
            if (len(set(state["torque"].values())) != 1 or
                    state.get("trajectory_goal_id") is not None or state.get("phase") is not None or
                    state.get("failed_endpoint_hold") is not None or
                    state.get("camera_recovery_phase") is not None or
                    state.get("component_restore_pending") is not False):
                raise ValueError("joint diagnostics require idle uniform six-joint ON or OFF torque")
            owner_session_id = state.get("owner_session_id")
            if not isinstance(owner_session_id, str) or not owner_session_id:
                raise ValueError("joint diagnostic owner session is missing")
            with self._active_lock:
                if (self._active_goal is not None or self._teleop_state != "disabled" or
                        self._prepare_reservation is not None or self._closed or
                        self._stop_latched or self._cancel_before_start):
                    raise ValueError("joint diagnostics require an unreserved idle owner")
                self._active_goal = "joint_diagnostics"
            result = self._reserved_owner_reply("joint_diagnostics", process, generation,
                                                started+1., "joint_diagnostics")
            self._assert_owner_reservation("joint_diagnostics", process, generation)
            if cohort != (self._expected_arm_id, self._expected_calibration_sha256,
                          self._owner_source_expected):
                raise RuntimeError("joint diagnostic identity/source cohort replaced")
            diagnostic = result.get("diagnostic")
            if (not isinstance(diagnostic, dict) or
                    diagnostic.get("schema") != "passive_joint_diagnostics" or
                    diagnostic.get("diagnostic_only") is not True or
                    diagnostic.get("calibration_authority") is not False or
                    diagnostic.get("motion_authority") is not False or
                    diagnostic.get("owner_session_id") != owner_session_id or
                    (diagnostic.get("arm_id"), diagnostic.get("calibration_sha256"),
                     diagnostic.get("owner_source_sha256")) != cohort):
                raise RuntimeError("joint diagnostic acknowledgement lacks original owner cohort")
            payload = dict(diagnostic, owner_started_monotonic=generation)
            response.message = json.dumps(payload, allow_nan=False)
            if result.get("ok") is not True or diagnostic.get("available") is not True:
                return response
            final = result.get("state")
            if (self._prepare_owner_state(final, process, generation, after=started) != "normal" or
                    final.get("event") != "joint_diagnostics" or
                    final.get("owner_session_id") != owner_session_id or
                    final["torque"] != state["torque"]):
                raise RuntimeError("joint diagnostic final owner state or torque cohort changed")
            after = diagnostic.get("after")
            if (not isinstance(after, dict) or any(after.get(k) != final.get(k)
                    for k in ("raw", "torque", "monotonic", "torque_monotonic", "arm_bounds"))):
                raise RuntimeError("joint diagnostic final measurements do not match owner state")
            acquisition_start = diagnostic.get("acquisition_started_monotonic")
            acquisition_end = diagnostic.get("acquisition_ended_monotonic")
            if (any(type(v) not in (int, float) or not math.isfinite(v)
                    for v in (acquisition_start, acquisition_end)) or
                    not started <= acquisition_start <= acquisition_end <= time.monotonic() or
                    acquisition_end-acquisition_start > .08 or diagnostic.get("read_budget_s") != .08):
                raise RuntimeError("joint diagnostic read acquisition exceeded its 80 ms bound")
            expected = {"Goal_Position", "Phase", "Present_Position", "Torque_Enable", "P_Coefficient",
                "I_Coefficient", "D_Coefficient", "CW_Dead_Zone", "CCW_Dead_Zone",
                "Minimum_Startup_Force", "Torque_Limit", "Max_Torque_Limit", "Present_Load",
                "Present_Current", "Present_Voltage", "Present_Temperature", "Status", "Moving"}
            rows = diagnostic.get("registers")
            if not isinstance(rows, dict) or set(rows) != expected:
                raise RuntimeError("joint diagnostic inventory differs from fixed read whitelist")
            for name, row in rows.items():
                if (not isinstance(row, dict) or row.get("status") not in
                        {"available", "unsupported", "budget_exhausted", "read_error"} or
                        row.get("units") != "raw SDK integer; physical units unqualified"):
                    raise RuntimeError("joint diagnostic register observation is malformed")
                if row["status"] == "available":
                    values = row.get("values")
                    if (not isinstance(values, dict) or set(values) != set(TELEOP_JOINTS) or
                            any(type(v) is not int for v in values.values()) or
                            type(row.get("started_monotonic")) not in (int, float) or
                            type(row.get("ended_monotonic")) not in (int, float) or
                            not acquisition_start <= row["started_monotonic"] <=
                                row["ended_monotonic"] <= acquisition_end):
                        raise RuntimeError("joint diagnostic requires original six-joint raw register values")
                elif name in {"Present_Position", "Torque_Enable"} or not isinstance(row.get("error"), str):
                    raise RuntimeError("joint diagnostic required feedback is unavailable")
            if (rows["Present_Position"]["values"] != final["raw"] or
                    rows["Torque_Enable"]["values"] != final["torque"]):
                raise RuntimeError("joint diagnostic register/final feedback values differ")
            response.success = True
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError, KeyError) as error:
            if isinstance(error, (RuntimeError, OSError)):
                self._mark_owner_fault("_joint_diagnostics")
            response.success = False
            response.message = json.dumps(dict(schema="passive_joint_diagnostics",
                diagnostic_only=True, calibration_authority=False, motion_authority=False,
                available=False, error=str(error), error_type=type(error).__name__), allow_nan=False)
        finally:
            with self._active_lock:
                if self._active_goal_is("joint_diagnostics"):
                    self._active_goal = None
        return response

    @_control_callback
    def _gripper_diagnostics(self, _request, response):
        """Finite, diagnostic-only read through the existing sole-owner reservation."""
        response.success = False
        process, generation = self._process, self._owner_started
        cohort = (self._expected_arm_id, self._expected_calibration_sha256,
                  dict(self._owner_source_expected or {}))
        started = time.monotonic()

        def observation(value, *, after, max_age=.1):
            now = time.monotonic()
            freshness = min(max_age, float(self.get_parameter("state_freshness_s").value)) if max_age == .1 else max_age
            if not isinstance(value, dict):
                raise ValueError("jaw diagnostic six-joint observation is missing")
            for key in ("monotonic", "torque_monotonic"):
                stamp = value.get(key)
                if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                        not max(generation, after, now-freshness) <= stamp <= now):
                    raise ValueError("jaw diagnostic original acquisition is stale")
            for key, upper in (("raw", 4095), ("torque", 0)):
                values = value.get(key)
                if (not isinstance(values, dict) or set(values) != set(TELEOP_JOINTS) or
                        any(type(v) is not int or not 0 <= v <= upper for v in values.values())):
                    raise ValueError("jaw diagnostic requires exact raw six-joint all-off observations")

        try:
            if (self._external_state_only or self._closed or process is None or
                    process.poll() is not None or self._owner_faulted or self._reconnect_observe or
                    type(generation) not in (int, float) or not math.isfinite(generation) or
                    generation <= 0 or not cohort[2]):
                raise ValueError("initialized non-observing owner is unavailable")
            state = self._read_state()
            if (not isinstance(state, dict) or state.get("observe_only") is not False or
                    (state.get("arm_id"), state.get("calibration_sha256"),
                     state.get("owner_source_sha256")) != cohort):
                raise ValueError("jaw diagnostic owner identity/source mismatch")
            observation(state, after=generation)
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise ValueError("jaw diagnostics require idle owner")
                self._active_goal = "jaw_diagnostics"
            result = self._reserved_owner_reply("jaw_diagnostics", process, generation,
                                                started+1., "jaw_diagnostics")
            self._assert_owner_reservation("jaw_diagnostics", process, generation)
            if cohort != (self._expected_arm_id, self._expected_calibration_sha256,
                          self._owner_source_expected):
                raise RuntimeError("jaw diagnostic identity/source cohort replaced")
            diagnostic = result.get("diagnostic") if isinstance(result, dict) else None
            if (not isinstance(diagnostic, dict) or
                    diagnostic.get("schema") != "jaw-register-diagnostics-v1" or
                    diagnostic.get("diagnostic_only") is not True or
                    diagnostic.get("calibration_authority") is not False or
                    diagnostic.get("motion_authority") is not False or
                    diagnostic.get("motor_id") != 6 or
                    (diagnostic.get("arm_id"), diagnostic.get("calibration_sha256"),
                     diagnostic.get("owner_source_sha256")) != cohort):
                raise RuntimeError("jaw diagnostic acknowledgement lacks original owner cohort")
            payload = dict(diagnostic, owner_started_monotonic=generation)
            if result.get("ok") is not True or diagnostic.get("available") is not True:
                response.message = json.dumps(payload, allow_nan=False)
                return response
            for key in ("started_monotonic", "ended_monotonic"):
                stamp = diagnostic.get(key)
                if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                        not started <= stamp <= time.monotonic()):
                    raise RuntimeError("jaw diagnostic acquisition clock is invalid")
            if diagnostic["ended_monotonic"] < diagnostic["started_monotonic"]:
                raise RuntimeError("jaw diagnostic clock order changed")
            observation(diagnostic.get("after"), after=started)
            observation(diagnostic.get("before"), after=started, max_age=.5)
            expected = (("CW_Dead_Zone", 26, 1), ("CCW_Dead_Zone", 27, 1),
                ("P_Coefficient", 21, 1), ("D_Coefficient", 22, 1),
                ("I_Coefficient", 23, 1), ("Minimum_Startup_Force", 24, 2),
                ("Max_Torque_Limit", 16, 2), ("Torque_Limit", 48, 2),
                ("Present_Load", 60, 2), ("Present_Current", 69, 2),
                ("Status", 65, 1), ("Moving", 66, 1))
            rows = diagnostic.get("registers")
            if (not isinstance(rows, list) or len(rows) != len(expected) or
                    any(not isinstance(row, dict) or row.get("register") != name or
                        type(row.get("motor_id")) is not int or row["motor_id"] != 6 or
                        type(row.get("address")) is not int or row["address"] != address or
                        type(row.get("width_bytes")) is not int or row["width_bytes"] != width or
                        type(row.get("value")) is not int or
                        not (-1023 if name == "Present_Load" else 0) <= row["value"] <=
                            (1023 if name == "Present_Load" else 2**(8*width)-1) or
                        row.get("units") != "raw register value" or
                        type(row.get("started_monotonic")) not in (int, float) or
                        type(row.get("ended_monotonic")) not in (int, float) or
                        not diagnostic["started_monotonic"] <= row["started_monotonic"] <=
                            row["ended_monotonic"] <= diagnostic["ended_monotonic"]
                        for row, (name, address, width) in zip(rows, expected)) or rows[-1]["value"] != 0):
                raise RuntimeError("jaw diagnostic fixed ID 6 inventory is invalid")
            # The initial acquisition may be older than final feedback. Keep its
            # original clocks and require exact equality, never a drift tolerance.
            before, after = diagnostic.get("before"), diagnostic["after"]
            if (not isinstance(before, dict) or before.get("raw") != after["raw"] or
                    before.get("torque") != after["torque"]):
                raise RuntimeError("jaw diagnostic six-joint snapshot changed")
            response.message = json.dumps(payload, allow_nan=False)
            response.success = True
        except (OSError, RuntimeError, TimeoutError, TypeError, ValueError) as error:
            response.success = False
            # Pure read failures do not authorize any cleanup/profile/goal write.
            # Unknown delivery poisons admission until a separately explicit stop.
            if isinstance(error, (RuntimeError, OSError)):
                self._mark_owner_fault("_gripper_diagnostics")
            response.message = json.dumps(dict(schema="jaw-register-diagnostics-v1",
                diagnostic_only=True, calibration_authority=False, motion_authority=False,
                available=False, error=str(error), error_type=type(error).__name__,
                owner_started_monotonic=(generation if type(generation) in (int, float)
                    and math.isfinite(generation) else None)), allow_nan=False)
        finally:
            with self._active_lock:
                if self._active_goal_is("jaw_diagnostics"):
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
            self._mark_owner_fault("_commission_gripper_step")
            response.message = str(error)
            # Stop remains jaw-only in the owner even after the component method
            # has returned. Do not turn an unknown result into success or retry.
            try:
                self._send({"command": "stop", "request_id": uuid.uuid4().hex})
            except (OSError, RuntimeError, TimeoutError, ValueError):
                pass
        finally:
            with self._active_lock:
                if self._active_goal_is("commission_gripper"):
                    self._active_goal = None
        return response

    @_control_callback
    def _commission_gripper_held_step(self, request, response):
        """Explicit raw jaw calibration; preserve the already-held body."""
        response.success = False
        reservation = "commission_gripper_held"
        process, generation = self._process, self._owner_started
        delivery_possible = terminal_verified = False
        try:
            if self.get_parameter("calibration_endpoint_mode").value is not True:
                raise ValueError("held jaw requires calibration_endpoint_mode:=true")
            delta = request.delta_counts
            if type(delta) is not int or not 0 < abs(delta) <= 16:
                raise ValueError("delta_counts must be a nonzero integer in [-16,16]")
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise RuntimeError("owner must be idle with teleoperation disabled")
                self._active_goal = reservation
            self._assert_owner_reservation(reservation, process, generation)
            state = self._read_state()
            if self._prepare_owner_state(state, process, generation, torque=1) != "normal":
                raise RuntimeError("held jaw requires normal admitted six-joint state")
            bounds = _arm_bounds_observation(state)
            target = state["raw"]["gripper"]+delta
            if not bounds["admitted_min"]["gripper"] <= target <= bounds["admitted_max"]["gripper"]:
                raise ValueError("held jaw target exceeds admitted bounds")
            low, high = self._gripper_raw_limits
            if type(low) is not int or type(high) is not int or not 0 <= low < high <= 4095:
                raise ValueError("identity-matched raw jaw calibration is unavailable")
            timeout_s = float(self.get_parameter("collision_check_timeout_s").value)
            if not math.isfinite(timeout_s) or not 0 < timeout_s <= 30:
                raise ValueError("held jaw collision preflight timeout must be finite in (0,30]")
            if any(type(state["pose"][j]) not in (int,float) or not math.isfinite(state["pose"][j]) for j in ARM_JOINTS):
                raise ValueError("held jaw requires finite measured body radians")
            deadline = time.monotonic()+timeout_s
            if not self._state_validity.wait_for_service(timeout_sec=min(2., max(0.,deadline-time.monotonic()))):
                raise RuntimeError("MoveIt state validity is unavailable")
            positions = [math.radians(state["pose"][j]) for j in ARM_JOINTS]
            for raw in range(state["raw"]["gripper"], target+(1 if delta>0 else -1), 1 if delta>0 else -1):
                self._assert_owner_reservation(reservation, process, generation)
                jaw_angle = math.radians(-10.+110.*(raw-low)/(high-low))
                checked = self._get_validity(positions, jaw_angle, deadline)
                if time.monotonic() > deadline:
                    raise TimeoutError("held jaw collision response exceeded preflight deadline")
                if checked.valid is not True:
                    raise RuntimeError("held jaw collision sample rejected before dispatch")
            fresh = self._read_state()
            if (self._prepare_owner_state(fresh, process, generation, torque=1) != "normal" or
                    fresh["raw"] != state["raw"] or
                    fresh.get("owner_session_id") != state.get("owner_session_id") or
                    not isinstance(state.get("owner_session_id"), str) or not state["owner_session_id"]):
                raise RuntimeError("held jaw start changed during collision preflight")
            command = dict(command="commission_gripper_held_step", delta_counts=delta,
                expected_start_raw=dict(fresh["raw"]), owner_session_id=fresh["owner_session_id"],
                command_id="held-jaw-"+uuid.uuid4().hex)
            self._home_completed_owner_started = None
            dispatched = time.monotonic()
            if dispatched > deadline:
                raise TimeoutError("held jaw collision preflight expired before owner dispatch")
            delivery_possible = True
            reply = self._reserved_owner_reply(command, process, generation,
                dispatched+4., reservation)
            if not isinstance(reply, dict):
                raise RuntimeError("held jaw result is not a typed object")
            final = reply.get("state")
            if self._prepare_owner_state(final, process, generation, after=dispatched, torque=1) != "normal":
                raise RuntimeError("held jaw final state is not normal")
            component = final.get("component_command", {})
            failed = reply.get("status") == "failed_held_jaw"
            entry_ram, restored_ram = component.get("entry_jaw_ram"), component.get("restored_jaw_ram")
            body_goals = component.get("preserved_body_goals")
            if (not isinstance(entry_ram, dict) or set(entry_ram) != {"Acceleration","Goal_Velocity","Torque_Limit"} or
                    any(type(v) is not int or not 0 <= v <= (255 if k == "Acceleration" else 65535)
                        for k,v in entry_ram.items()) or entry_ram != restored_ram or
                    not isinstance(body_goals, dict) or set(body_goals) != set(ARM_JOINTS) or
                    any(type(v) is not int or not bounds["admitted_min"][j] <= v <= bounds["admitted_max"][j]
                        for j,v in body_goals.items())):
                raise RuntimeError("held jaw entry profile/body goals proof invalid")
            if (reply.get("qualified") is not False or
                    component.get("mode") != "held_body_raw_jaw" or
                    component.get("command_id") != command["command_id"] or
                    component.get("delta_counts") != delta or component.get("target_raw") != target or
                    type(component.get("endpoint_tolerance_counts")) is not int or
                    component["endpoint_tolerance_counts"] != min(2, abs(delta)-1) or
                    component.get("body_profile_unchanged") is not True or
                    component.get("entry_jaw_ram") != component.get("restored_jaw_ram") or
                    final.get("owner_session_id") != command["owner_session_id"] or
                    any(abs(final["raw"][j]-state["raw"][j]) > 2 for j in ARM_JOINTS) or
                    type(component.get("goal_monotonic")) not in (int, float) or
                    not max(dispatched,time.monotonic()-.1) <= component["goal_monotonic"] <= time.monotonic()):
                raise RuntimeError("held jaw completion lacks matching fresh preservation proof")
            if failed:
                hold = reply.get("hold", {})
                if (reply.get("ok") is not False or reply.get("target_reached") is not False or
                        final.get("event") not in {"commission_gripper_held_failed", "read_state"} or
                        hold != final.get("failed_endpoint_hold") or
                        hold.get("schema") != "failed-held-jaw-v1" or
                        hold.get("goal_id") != command["command_id"] or
                        hold.get("owner_session_id") != command["owner_session_id"] or
                        hold.get("body_anchor_raw") != state["raw"] or
                        hold.get("qualified") is not False or hold.get("target_reached") is not False or
                        hold.get("lifetime") != "while_healthy_until_explicit_stop_or_transport_loss" or
                        not isinstance(hold.get("target_raw"), dict) or
                        hold["target_raw"] != {**body_goals, "gripper":target} or
                        not isinstance(hold.get("held_goals_raw"),dict) or
                        set(hold["held_goals_raw"]) != set(TELEOP_JOINTS) or
                        any(type(v) is not int or not bounds["admitted_min"][j] <= v <= bounds["admitted_max"][j]
                            for j,v in hold["held_goals_raw"].items()) or
                        any(hold["held_goals_raw"][j] != body_goals[j] for j in ARM_JOINTS) or
                        type(hold.get("held_goal_monotonic")) not in (int,float) or
                        hold["held_goal_monotonic"] != component["goal_monotonic"] or
                        not isinstance(hold.get("held_raw"), dict) or set(hold["held_raw"]) != set(TELEOP_JOINTS) or
                        any(type(v) is not int or not bounds["admitted_min"][j] <= v <= bounds["admitted_max"][j] or
                            abs(v-final["raw"][j])>4 for j,v in hold["held_raw"].items()) or
                        any(type(hold.get(k)) not in (int,float) or not math.isfinite(hold[k]) or
                            not dispatched <= hold[k] <= final[clock] for k,clock in
                            (("entry_position_monotonic","monotonic"),("entry_torque_monotonic","torque_monotonic")))):
                    raise RuntimeError("held jaw failure proof is invalid")
            elif (reply.get("ok") is not True or final.get("event") != "commission_gripper_held_complete" or
                    component.get("status") != "complete" or
                    abs(final["raw"]["gripper"]-target)>component["endpoint_tolerance_counts"]):
                raise RuntimeError("held jaw did not strictly arrive")
            message = json.dumps(dict(qualified=False, qualification="SUPERVISED_RAW_JAW_NOT_SI_CALIBRATED",
                collision_samples_checked=abs(delta)+1, result=reply), allow_nan=False)
            self._prepare_owner_state(final, process, generation, after=dispatched, torque=1)
            terminal_verified = True
            self._cache_owner_state(final, process=process, generation=generation)
            if failed:
                frozen = json.dumps(dict(command=command, entry_raw=state["raw"],
                    hold=hold, held_goals=dict(hold["held_goals_raw"]),
                    component=component, profile=final.get("profile_registers"),
                    arm_id=self._expected_arm_id, calibration_sha256=self._expected_calibration_sha256,
                    owner_source_sha256=self._owner_source_expected,
                    jaw_limits=list(self._gripper_raw_limits)), sort_keys=True, allow_nan=False)
                self._mark_owner_fault("verified_failed_held_jaw", held_context=(process,generation,frozen))
            response.message, response.success = message, not failed
        except (OSError, ValueError, RuntimeError, TimeoutError, TypeError, KeyError) as error:
            response.message = str(error)
            if delivery_possible and not terminal_verified:
                self._mark_owner_fault("_commission_gripper_held_step")
                self._home_completed_owner_started = None
                try:
                    self._prepare_stop(process, generation)
                except (OSError, ValueError, RuntimeError, TimeoutError):
                    pass
        finally:
            with self._active_lock:
                if self._active_goal_is(reservation):
                    self._active_goal = None
        return response

    @_control_callback
    def _acknowledge_held_jaw(self, request, response):
        response.success = False
        reservation = "acknowledge_held_jaw_"+uuid.uuid4().hex
        delivery_possible = terminal_verified = False
        process, generation = self._process, self._owner_started
        try:
            if self.get_parameter("calibration_endpoint_mode").value is not True:
                raise RuntimeError("held jaw acknowledgment requires calibration endpoint mode")
            with self._active_lock:
                if self._active_goal is not None or self._teleop_state != "disabled":
                    raise RuntimeError("held jaw acknowledgment requires idle owner and disabled teleop")
                with self._owner_fault_lock:
                    context = self._held_jaw_fault
                if context is None:
                    raise RuntimeError("no verified scoped held jaw failure is available")
                self._active_goal = reservation
            self._assert_owner_reservation(reservation, process, generation)
            state = self._read_state()
            frozen = self._held_jaw_ack_state(state, process, generation, context)
            timeout = float(self.get_parameter("collision_check_timeout_s").value)
            if not math.isfinite(timeout) or not 0 < timeout <= 30:
                raise ValueError("invalid acknowledgment collision timeout")
            deadline = time.monotonic()+timeout
            if not self._state_validity.wait_for_service(timeout_sec=min(2.,timeout)):
                raise RuntimeError("current-state collision service is unavailable")
            low, high = self._gripper_raw_limits
            if type(low) is not int or type(high) is not int or not 0 <= low < high <= 4095:
                raise RuntimeError("identity-matched jaw calibration is unavailable")
            positions = [math.radians(state["pose"][j]) for j in ARM_JOINTS]
            if not all(math.isfinite(v) for v in positions):
                raise ValueError("current body pose is not finite")
            jaw = math.radians(-10.+110.*(state["raw"]["gripper"]-low)/(high-low))
            checked = self._get_validity(positions, jaw, deadline)
            if time.monotonic() > deadline or checked.valid is not True:
                raise RuntimeError("current held pose collision check rejected or expired")
            self._assert_owner_reservation(reservation, process, generation)
            fresh = self._read_state()
            self._held_jaw_ack_state(fresh, process, generation, context)
            if fresh["raw"] != state["raw"]:
                raise RuntimeError("held pose changed during acknowledgment collision check")
            command = dict(command="acknowledge_held_jaw",
                owner_session_id=frozen["hold"]["owner_session_id"],
                failed_goal_id=frozen["hold"]["goal_id"], expected_held_raw=frozen["hold"]["held_raw"])
            dispatched = time.monotonic()
            delivery_possible = True
            reply = self._reserved_owner_reply(command, process, generation, dispatched+2., reservation)
            if (isinstance(reply,dict) and reply.get("ok") is False
                    and reply.get("command") == "acknowledge_held_jaw"
                    and reply.get("error_type") == "ArmPlanPreflightError"):
                self._held_jaw_ack_state(self._read_state(), process, generation, context)
                terminal_verified = True
                response.message = json.dumps(dict(acknowledged=False, qualified=False,
                    target_reached=False, error=reply.get("error")), allow_nan=False)
                return response
            if not isinstance(reply,dict):raise RuntimeError("held jaw acknowledgment result unknown")
            final, proof = reply.get("state"), reply.get("acknowledgment")
            self._held_jaw_ack_state(final, process, generation, context, after=dispatched, acknowledged=True)
            if not isinstance(proof,dict) or not isinstance(proof.get("original_failure"),dict):
                raise RuntimeError("held jaw acknowledgment failure history is malformed")
            original = proof["original_failure"]
            goals = frozen["held_goals"]
            if (reply.get("ok") is not True or reply.get("status") != "held_jaw_acknowledged"
                    or reply.get("qualified") is not False or reply.get("target_reached") is not False
                    or not isinstance(proof,dict) or proof.get("schema") != "held-jaw-ack-v1"
                    or proof.get("acknowledged") is not True or proof.get("qualified") is not False
                    or proof.get("target_reached") is not False or type(proof.get("hardware_write_count")) is not int
                    or proof["hardware_write_count"] != 0 or final.get("event") != "held_jaw_acknowledged"
                    or final.get("held_jaw_acknowledgment") != proof
                    or proof.get("owner_session_id") != command["owner_session_id"]
                    or proof.get("failed_goal_id") != command["failed_goal_id"]
                    or proof.get("held_raw") != command["expected_held_raw"]
                    or proof.get("calibration_sha256") != frozen["calibration_sha256"]
                    or proof.get("owner_source_sha256") != frozen["owner_source_sha256"]
                    or proof.get("native_profile") != frozen["profile"] or proof.get("observed_goals") != goals
                    or original.get("hold") != frozen["hold"]
                    or original.get("original_start_raw") != frozen["entry_raw"]
                    or original.get("component_command") != frozen["component"]
                    or original.get("delta_counts") != frozen["command"]["delta_counts"]
                    or original.get("native_profile") != frozen["profile"]
                    or original.get("held_goals") != goals
                    or original.get("preserved_body_goals") != frozen["component"]["preserved_body_goals"]
                    or original.get("calibration_sha256") != frozen["calibration_sha256"]
                    or original.get("owner_source_sha256") != frozen["owner_source_sha256"]
                    or type(proof.get("stable_samples")) is not int or proof["stable_samples"] < 5):
                raise RuntimeError("held jaw acknowledgment terminal proof differs")
            for key in ("started_monotonic","completed_monotonic","stable_started_monotonic","stable_ended_monotonic"):
                value = proof.get(key)
                if type(value) not in (int,float) or not math.isfinite(value) or not dispatched <= value <= time.monotonic():
                    raise RuntimeError("held jaw acknowledgment clocks are invalid")
            if (proof["stable_ended_monotonic"]-proof["stable_started_monotonic"] < .3
                    or not proof["started_monotonic"] <= proof["stable_started_monotonic"]
                    <= proof["stable_ended_monotonic"] <= proof["completed_monotonic"]
                    or final["monotonic"] != proof["stable_ended_monotonic"]):
                raise RuntimeError("held jaw acknowledgment lacks original stable window")
            minima, maxima = proof.get("stable_raw_min"), proof.get("stable_raw_max")
            if (not isinstance(minima,dict) or not isinstance(maxima,dict)
                    or set(minima) != set(TELEOP_JOINTS) or set(maxima) != set(TELEOP_JOINTS)
                    or any(type(minima[j]) is not int or type(maxima[j]) is not int
                        or not 0 <= minima[j] <= final["raw"][j] <= maxima[j] <= 4095
                        or maxima[j]-minima[j] > 2 for j in TELEOP_JOINTS)):
                raise RuntimeError("held jaw acknowledgment stable encoder range is invalid")
            self._assert_owner_reservation(reservation, process, generation)
            with self._active_lock:
                self._held_jaw_ack_state(final, process, generation, context, after=dispatched, acknowledged=True)
                if (not self._active_goal_is(reservation) or self._stop_latched or self._cancel_before_start
                        or self._closed or not self._clear_owner_fault("held_jaw_acknowledged", expected_context=context)):
                    raise RuntimeError("held jaw acknowledgment lost its fault reservation")
            terminal_verified = True
            self._cache_owner_state(final, process=process, generation=generation)
            response.success = True
            response.message = json.dumps(dict(acknowledged=True, qualified=False,
                target_reached=False, result=reply), allow_nan=False)
        except (OSError,ValueError,RuntimeError,TimeoutError,TypeError,KeyError) as error:
            if delivery_possible and not terminal_verified:
                self._mark_owner_fault("held_jaw_acknowledgment_unknown")
            response.message = json.dumps(dict(acknowledged=False,qualified=False,
                target_reached=False,error=str(error)), allow_nan=False)
        finally:
            with self._active_lock:
                if self._active_goal_is(reservation):self._active_goal = None
        return response

    def _sorting_ready_value(self, state, *, reservation=None):
        process = self._process
        with self._active_lock:
            own_prepare = (reservation is not None and reservation is self._prepare_reservation and
                           self._active_goal_is("prepare"))
            active = self._active_goal is not None and not own_prepare
            teleop_state = self._teleop_state
        try:
            if self._prepare_owner_state(state, process, self._owner_started, torque=1) != "normal":
                return False
            if _measured_home_error(state, arm_id=self._expected_arm_id,
                    calibration_sha256=self._expected_calibration_sha256,
                    home_pose=self._saved_home_pose, tolerance=self._saved_home_tolerance) is not None:
                return False
        except (RuntimeError, TypeError, ValueError, KeyError):
            return False
        return bool(not self._external_state_only and not self._closed and
            process is not None and process.poll() is None and not self._owner_faulted and
            not self._stop_latched and not self._reconnect_observe and not active and
            teleop_state == "disabled" and self._gripper_jaw is not None and state is not None and
            self._home_completed_owner_started is not None and
            self._home_completed_owner_started == self._owner_started and
            state.get("arm_id") == self._expected_arm_id and
            state.get("calibration_sha256") == self._expected_calibration_sha256)

    def _sorting_ready(self, _request, response):
        """Read-only admission; an external caller cannot exempt preparation."""
        response.success = self._sorting_ready_value(self._read_state())
        response.message = ("sorting readiness confirmed from fresh identity-matched resident state; no motion commanded"
                            if response.success else
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

        generation = self._owner_started
        delivery_possible = False
        response.success = False
        try:
            pose_path = Path(str(self.get_parameter("calibration_ready_pose_file").value).strip())
            if not pose_path.is_absolute():
                raise ValueError("calibration_ready_pose_file must be an absolute planned-target path")
            observation = json.loads(pose_path.read_text())
            helper = self._trajectory_helper
            required = {"schema", "kind", "qualified", "qualification", "arm_id",
                        "calibration_sha256", "owner_source_sha256", "owner_started_monotonic",
                        "start_raw", "raw"}
            if (not isinstance(observation, dict) or set(observation) != required or
                    observation.get("schema") != "so101-supervised-calibration-target-v1" or
                    observation.get("kind") != "planned_observation" or
                    observation.get("qualified") is not False or
                    observation.get("qualification") != helper.SUPERVISED_QUALIFICATION or
                    type(observation.get("owner_started_monotonic")) not in (int, float) or
                    observation["owner_started_monotonic"] != generation):
                raise ValueError("explicit identity-bound supervised planned observation required")
            state = self._read_state()
            if self._prepare_owner_state(state, process, generation, torque=0) != "normal":
                raise RuntimeError("supervised commissioning requires normal admitted start")
            for key in ("arm_id", "calibration_sha256", "owner_source_sha256"):
                if observation[key] != state[key]:
                    raise ValueError("planned observation identity/source mismatch: " + key)
            if observation['start_raw'] != state['raw']:
                raise ValueError("planned observation frozen start changed")
            bounds = _arm_bounds_observation(state)
            start = {joint: math.radians(float(state['pose'][joint])) for joint in ARM_JOINTS}
            points, prepared = helper.prepare_supervised_calibration(
                observation['start_raw'], observation['raw'], start,
                bounds['admitted_min'], bounds['admitted_max'])
            digest = helper.path_digest(prepared, held_gripper_raw=state['raw']['gripper'])
            timeout_s = float(self.get_parameter("calibration_ready_timeout_s").value)
            if not math.isfinite(timeout_s) or not 0 < timeout_s <= 90 or prepared.duration+4 >= timeout_s:
                raise ValueError("calibration-ready timeout must fit the slow path and settlement, at most 90s")
            self._home_completed_owner_started = None
            command = {"command": "execute_trajectory", "goal_id": "supervised-calibration-"+uuid.uuid4().hex,
                       "joint_names": list(ARM_JOINTS), "points": points, "path_sha256": digest,
                       "supervised_commissioning": True, "expected_start_raw": observation['start_raw'],
                       "ros_motion_profile": self._ros_motion_profile.name}
            dispatched = time.monotonic()
            delivery_possible = True
            result = self._reserved_owner_reply(command, process, generation,
                time.monotonic()+timeout_s, "calibration_ready")
            if not isinstance(result, dict) or result.get('ok') is not True:
                raise RuntimeError("supervised owner trajectory failed: " + str(result))
            final_state = result.get('state')
            self._prepare_owner_state(final_state, process, generation, after=dispatched, torque=1)
            trajectory = result.get('trajectory', {})
            if (final_state.get('event') != 'trajectory_complete' or
                    trajectory.get('goal_id') != command['goal_id'] or
                    trajectory.get('effective_path_sha256') != digest or
                    trajectory.get('qualification') != helper.SUPERVISED_QUALIFICATION or
                    trajectory.get('qualified') is not False):
                raise RuntimeError("supervised completion lacks matching path and qualification")
            response.message = json.dumps({"qualification": helper.SUPERVISED_QUALIFICATION,
                "qualified": False, "effective_path_sha256": digest,
                "state": final_state, "trajectory": trajectory}, allow_nan=False)
            response.success = True
        except (OSError, ValueError, RuntimeError, TimeoutError, TypeError, KeyError) as error:
            stop_verified = None
            if delivery_possible:
                try:
                    stop_verified = self._prepare_stop(process, generation)
                except (OSError, ValueError, RuntimeError, TimeoutError, TypeError) as stop_error:
                    error.add_note("supervised stop cleanup: " + str(stop_error))
            response.success = False
            response.message = json.dumps({"qualified": False, "error": str(error),
                "command_delivery": "may_have_been_sent" if delivery_possible else "not_sent",
                "stop_verified": stop_verified}, allow_nan=False)
        finally:
            with self._active_lock:
                if self._active_goal_is("calibration_ready"):
                    self._active_goal = None
        return response

    def _teleop_timer_tick(self):
        self._teleop_tick(wait_for_ack=True)

    @_control_callback
    def _teleop_tick(self, *, wait_for_ack=False):
        with self._active_lock:
            if (getattr(self, "_closed", False) or self._teleop_state != "active" or
                    self._owner_faulted or self._stop_latched):
                return
            if self._teleop_inflight:
                # Only a blocked periodic timer requests an ACK wakeup. A queued
                # guard callback must not register a wait for a newer command.
                if wait_for_ack:
                    self._teleop_waiting_ack = (self._teleop_session,
                                                self._teleop_inflight_sequence)
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
                if (getattr(self, "_closed", False) or
                        self._teleop_state != "active" or self._teleop_inflight or
                        self._teleop_session != session_id):
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
        mapped = (dict(latest["pose"]) if self._teleop_mode == "absolute" else
                  matched_pose(latest["pose"], origin, follower_start))
        # ArmSession clamps body targets to its exact calibrated raw bounds. Its gripper
        # input is normalized percent, so clamp that station-unit target here.
        mapped["gripper"] = max(0.0, min(100.0, mapped["gripper"]))
        with self._active_lock:
            if (getattr(self, "_closed", False) or
                    self._teleop_state != "active" or self._teleop_inflight or
                    self._teleop_session != session_id or
                    latest["sequence"] <= self._teleop_last_source_sequence):
                return
            sequence = self._teleop_owner_sequence + 1
            self._teleop_waiting_ack = None
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

    def _read_stderr(self, process=None):
        process = self._process if process is None else process
        for line in process.stderr:
            if process is not self._process:
                return
            self.get_logger().error("ArmSession: " + line.rstrip())

    def _read_state(self, *, allow_stale=False):
        try:
            generation, process = self._owner_started, self._process
            state = json.loads(self._state_file.read_text())
            self._cache_owner_state(state, generation=generation, process=process)
            if (not allow_stale and
                    time.monotonic() - float(state["monotonic"]) > float(self.get_parameter("state_freshness_s").value)):
                return None
            return state
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            with self._state_cache_lock:
                if self._owner_started == generation and self._process is process:
                    self._cached_owner_state = None
                    self._cached_owner_generation = generation
                    self._cached_owner_reason = "state_read_failed"
                    collector = self._recovery_depth_cache
                else:
                    collector = None
            if collector is not None:
                collector.invalidate_generation(None)
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
            if getattr(self, "_closed", False) and command.get("command") != "exit":
                raise RuntimeError("bridge destruction rejects owner commands")
            with self._state_cache_lock:
                fault_observing = (isinstance(self._cached_owner_state, dict) and
                                   self._cached_owner_state.get("fault_observation") is not None)
            if fault_observing and command.get("command") not in {"read_state", "stop", "exit"}:
                raise RuntimeError("fault-observing owner requires explicit reconnect before commands")
            if getattr(self, "_stop_latched", False) and command.get("command") not in {"stop", "exit"}:
                raise RuntimeError("owner stop barrier rejects motion commands")
            self._write_owner_command(command, deadline)
        except (OSError, RuntimeError, ValueError):
            self._mark_owner_fault("_send")
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

    def _planning_limits_for_state(self, state):
        contract = getattr(self, "_planning_contract", None)
        if contract is not None:
            self._trajectory_helper.check_joint_limit_state(contract, state)
            return self._planning_limits
        # A portable model cannot authorize a hardware range it does not model.
        bounds = _arm_bounds_observation(state)
        if bounds is None or getattr(self, "_planning_calibration", None) is None:
            raise ValueError("planning joint limits lack calibrated owner evidence; export joint_limit_contract")
        actual = self._trajectory_helper.encoder_position_limits(self._planning_calibration,
            bounds["admitted_min"], bounds["admitted_max"])
        fallback = self._planning_limits or self._trajectory_helper.POSITION_LIMITS_RAD
        if any(any(abs(a-b) > 1e-12 for a, b in zip(actual[j], fallback[j])) for j in ARM_JOINTS):
            raise ValueError("portable model differs from owner limits; export joint_limit_contract from observe-only owner and relaunch")
        if bounds["classification"] != "normal":
            raise ValueError("resident state requires bounded recovery before planning")
        return actual

    def _collision_check_path(self, trajectory, points, goal_id):
        state = self._read_state()
        if state is None:
            raise RuntimeError("fresh measured state is required before collision checking")
        position_limits = self._planning_limits_for_state(state)
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
        retiming_factor, command_points, prepared = prepare_with_controller_timing(
            helper, trajectory.joint_names, parsed_points, start_radians=start,
            max_jerk=max_jerk, motion_profile=self._ros_motion_profile,
            position_limits=position_limits)
        timing_method, collinear_residual = "uniform_controller_timing", None
        candidate = helper.prepare_collinear_quintic(
            trajectory.joint_names, parsed_points, start_radians=start,
            max_velocity=self._ros_motion_profile.max_velocity,
            max_acceleration=self._ros_motion_profile.max_acceleration,
            max_jerk=max_jerk, period=helper.ROS_TRAJECTORY_PERIOD_S,
            position_limits=position_limits)
        if candidate is not None and candidate[1].duration < prepared.duration:
            command_points, prepared, collinear_residual = candidate
            retiming_factor = prepared.duration / parsed_points[-1]["time"]
            timing_method = "monotone_collinear_rest_to_rest_quintic"
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
            "joint_names": list(trajectory.joint_names),
            "retiming_factor": retiming_factor, "command_points": command_points,
            "timing_method": timing_method, "collinear_residual_rad": collinear_residual,
            "original_point_count": len(parsed_points), "command_point_count": len(command_points),
            "ros_motion_profile": self._ros_motion_profile.name,
            "max_velocity_rad_s": self._ros_motion_profile.max_velocity,
            "max_acceleration_rad_s2": self._ros_motion_profile.max_acceleration,
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
            "sample_period_s": helper.ROS_TRAJECTORY_PERIOD_S,
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

    def _validate_failed_endpoint_hold(self, reply, *, goal_id, process, generation,
                                       owner_session_id, after):
        if (not isinstance(reply, dict) or reply.get("status") != "failed_endpoint_held" or
                reply.get("ok") is not False or reply.get("target_reached") is not False or
                reply.get("qualified") is not False or reply.get("goal_id") != goal_id):
            raise RuntimeError("malformed failed endpoint hold terminal")
        state, hold = reply.get("state"), reply.get("hold")
        if (not isinstance(state, dict) or not isinstance(hold, dict) or
                hold.get("schema") != "failed-endpoint-hold-v1" or
                hold.get("goal_id") != goal_id or
                not isinstance(owner_session_id, str) or not owner_session_id or
                hold.get("owner_session_id") != owner_session_id or
                state.get("owner_session_id") != owner_session_id or
                state.get("event") not in {"trajectory_failed_endpoint_held", "read_state"} or
                state.get("failed_endpoint_hold") != hold or
                hold.get("lifetime") != "while_healthy_until_explicit_stop_or_transport_loss" or
                hold.get("qualified") is not False or hold.get("target_reached") is not False or
                not isinstance(hold.get("held_raw"), dict)):

            raise RuntimeError("failed endpoint hold session/proof mismatch")
        if self._prepare_owner_state(state, process, generation, after=after, torque=1) != "normal":
            raise RuntimeError("failed endpoint hold is outside normal admitted bounds")
        for entry_key, observation_key in (("entry_position_monotonic", "monotonic"),
                                            ("entry_torque_monotonic", "torque_monotonic")):
            stamp = hold.get(entry_key)
            if (type(stamp) not in (int, float) or not math.isfinite(stamp) or
                    not after <= stamp <= state[observation_key]):
                raise RuntimeError("failed endpoint hold entry clock mismatch")
        if hold["entry_position_monotonic"] > hold["entry_torque_monotonic"]:
            raise RuntimeError("failed endpoint hold entry clock order mismatch")
        target, errors = hold.get("target_raw"), hold.get("goal_error_rad")
        bounds = _arm_bounds_observation(state)
        anchor = hold["held_raw"]
        if (set(anchor) != set(TELEOP_JOINTS) or
                any(type(v) is not int or not bounds["admitted_min"][j] <= v <=
                    bounds["admitted_max"][j] or abs(v-state["raw"][j]) > 4
                    for j, v in anchor.items())):
            raise RuntimeError("failed endpoint hold original anchor mismatch")
        if (not isinstance(target, dict) or set(target) != set(TELEOP_JOINTS) or
                any(type(v) is not int or not bounds["admitted_min"][j] <= v <=
                    bounds["admitted_max"][j] for j, v in target.items()) or
                not isinstance(errors, dict) or set(errors) != set(ARM_JOINTS) or
                any(type(v) not in (int, float) or not math.isfinite(v) or
                    not 0 <= v <= math.radians(10) for v in errors.values()) or
                type(hold.get("settled_samples")) is not int or
                not 0 <= hold["settled_samples"] < 3):
            raise RuntimeError("failed endpoint hold target/error metadata invalid")
        json.dumps(reply, allow_nan=False)
        return state

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

    def _gripper_calibration_status(self, _request, response):
        """Read-only distinction between qualified metric and angular interfaces."""
        metric = self._gripper_jaw is not None
        angular = getattr(self, "_gripper_joint", None) is not None
        response.success = metric and angular
        response.message = json.dumps(dict(
            schema="so101-gripper-calibration-status-v1", si_gap_calibrated=metric,
            moveit_joint_calibrated=angular, si_position_units="metres",
            moveit_position_units="radians", visual_proxy_only=not angular,
            reason=("qualified gap and joint-angle calibration loaded" if response.success else
                    "MoveIt gripper requires qualified gap and identity-matched joint-angle calibration; no motion commanded")))
        return response

    def _moveit_gripper_gap(self, radians):
        if self._gripper_joint is None or self._gripper_jaw is None:
            raise ValueError("MoveIt gripper requires qualified joint-angle and SI jaw-gap calibrations")
        return percent_to_gap(joint_to_percent(radians, self._gripper_joint), self._gripper_jaw)

    @_control_callback
    def _moveit_gripper_goal_request(self, goal):
        try:
            adapted = GripperCommand.Goal()
            adapted.command.position = self._moveit_gripper_gap(goal.command.position)
            adapted.command.max_effort = goal.command.max_effort
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Rejecting MoveIt gripper goal: {error}")
            return GoalResponse.REJECT
        return self._gripper_goal_request(adapted)

    @_control_callback
    def _execute_moveit_gripper(self, goal_handle):
        return self._execute_gripper(goal_handle, angular=True)

    def _gripper_collision_binding(self, target_raw):
        """Check every reachable raw jaw sample against the full collision scene."""
        process, generation = self._process, self._owner_started
        self._assert_owner_reservation("gripper", process, generation)
        if self._teleop_state != "disabled":
            raise RuntimeError("MoveIt gripper requires disabled teleoperation")
        state = self._read_state()
        if self._prepare_owner_state(state, process, generation, torque=1) != "normal":
            raise RuntimeError("MoveIt gripper requires normal admitted six-joint state")
        bounds = _arm_bounds_observation(state)
        if bounds is None or not bounds["admitted_min"]["gripper"] <= target_raw <= bounds["admitted_max"]["gripper"]:
            raise ValueError("MoveIt gripper target exceeds admitted raw bounds")
        timeout = float(self.get_parameter("collision_check_timeout_s").value)
        if not math.isfinite(timeout) or not 0 < timeout <= 30:
            raise ValueError("gripper collision timeout must be finite in (0,30]")
        deadline = time.monotonic()+timeout
        if not self._state_validity.wait_for_service(timeout_sec=min(2., timeout)):
            raise RuntimeError("MoveIt state validity is unavailable")
        low, high = self._gripper_raw_limits
        body = [math.radians(state["pose"][j]) for j in ARM_JOINTS]
        start = state["raw"]["gripper"]
        step = 1 if target_raw >= start else -1
        for raw in range(start, target_raw+step, step):
            self._assert_owner_reservation("gripper", process, generation)
            with self._active_lock:
                if self._gripper_goal_cancelled:
                    raise InterruptedError("MoveIt gripper cancelled before dispatch")
            request = GetStateValidity.Request()
            # Empty group checks the complete robot, including both fingers.
            request.group_name = ""
            request.robot_state.joint_state.name = [*ARM_JOINTS, "gripper"]
            request.robot_state.joint_state.position = [*body,
                percent_to_joint(100.*(raw-low)/(high-low), self._gripper_joint)]
            future = self._state_validity.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            while not completed.wait(.01):
                with self._active_lock:
                    cancelled = self._gripper_goal_cancelled
                if cancelled or time.monotonic() >= deadline:
                    future.cancel()
                    if cancelled:
                        raise InterruptedError("MoveIt gripper cancelled during collision preflight")
                    raise TimeoutError("MoveIt gripper collision preflight timed out")
            checked = future.result()
            if time.monotonic() >= deadline:
                raise TimeoutError("MoveIt gripper collision preflight expired")
            if checked is None or checked.valid is not True:
                raise RuntimeError("MoveIt gripper collision sample rejected before dispatch")
        fresh = self._read_state()
        if (self._prepare_owner_state(fresh, process, generation, torque=1) != "normal" or
                fresh["raw"] != state["raw"] or
                not isinstance(state.get("owner_session_id"), str) or not state["owner_session_id"] or
                fresh.get("owner_session_id") != state["owner_session_id"]):
            raise RuntimeError("MoveIt gripper start changed during collision preflight")
        return dict(expected_start_raw=dict(state["raw"]), owner_session_id=state["owner_session_id"])

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
            if not self._active_goal_is("gripper") or not getattr(_goal_handle, "is_active", True):
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
            if (not self._active_goal_is("gripper") or self._gripper_goal_cancelled or
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

    def _gripper_feedback_state(self):
        """Read the already-admitted cache; feedback never reads a snapshot file."""
        now = time.monotonic()
        with self._state_cache_lock:
            state, generation = self._cached_owner_state, self._cached_owner_generation
        if (not isinstance(state, dict) or type(generation) not in (int, float) or
                not math.isfinite(generation) or generation <= 0 or generation != self._owner_started or
                self._closed or self._stop_latched or self._owner_faulted or
                self._process is None or self._process.poll() is not None or
                state.get("arm_id") != self._expected_arm_id or
                state.get("calibration_sha256") != self._expected_calibration_sha256 or
                not isinstance(self._owner_source_expected, dict) or not self._owner_source_expected or
                state.get("owner_source_sha256") != self._owner_source_expected or
                state.get("observe_only") is not False or not isinstance(state.get("pose"), dict)):
            return None
        age_limit = min(.1, float(self.get_parameter("state_freshness_s").value))
        if any(type(state.get(k)) not in (int, float) or not math.isfinite(state[k]) or
               not generation <= state[k] <= now or now-state[k] > age_limit
               for k in ("monotonic", "torque_monotonic")):
            return None
        return state

    @_control_callback
    def _execute_gripper(self, goal_handle, *, angular=False):
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
            target_percent = (joint_to_percent(command.position, self._gripper_joint) if angular else
                              gap_to_percent(command.position, self._gripper_jaw))
            low, high = self._gripper_raw_limits
            if angular:
                bounds = _arm_bounds_observation(state)
                if bounds is None:
                    raise ValueError("MoveIt gripper requires admitted raw bounds")
                target_raw = joint_target_raw(command.position, self._gripper_joint, (low, high),
                    (bounds["admitted_min"]["gripper"], bounds["admitted_max"]["gripper"]))
            else:
                target_raw = int(round(low + target_percent * (high - low) / 100.0))
            # Match the resident's 650-count leg cap, 10-count arrival
            # tolerance plus four-count dwell allowance, six-second leg
            # deadline and 0.4-second dwell budget.
            # Use the full calibrated span because the owner measures the
            # actual start again when it admits this single command.
            max_chunks = max(1, math.ceil((high - low) / (650 - 10 - 4)))
            command_timeout_s = max_chunks * (6.0 + 0.4) + 2.0
            deadline = time.monotonic() + command_timeout_s
            binding = self._gripper_collision_binding(target_raw) if angular else {}
            deadline = time.monotonic() + command_timeout_s
            submitted = self._submit_gripper_plan({"command": "move_gripper", "target_raw": target_raw, **binding})
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
                measured = self._gripper_feedback_state()
                if measured is not None:
                    feedback = GripperCommand.Feedback()
                    feedback.position = (percent_to_joint(measured["pose"]["gripper"], self._gripper_joint) if angular else
                                         percent_to_gap(measured["pose"]["gripper"], self._gripper_jaw))
                    goal_handle.publish_feedback(feedback)
                try:
                    candidate = self._response_queue.get(timeout=max(0., min(0.05, deadline-time.monotonic())))
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
                    self._mark_owner_fault("_execute_gripper")
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
            result.position = (percent_to_joint(final_percent, self._gripper_joint) if angular else
                               percent_to_gap(final_percent, self._gripper_jaw))
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
                self._mark_owner_fault("_execute_gripper")
            result.stalled = False
            result.reached_goal = False
            self.get_logger().error(f"GripperCommand failed: {error}")
            goal_handle.abort()
            return result
        finally:
            with self._active_lock:
                if self._active_goal_is("gripper") and (not self._owner_faulted or getattr(self, "_stop_latched", False)):
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
            if isinstance(self._active_goal, str) and self._active_goal_is("native_damping", "operator_maintenance", "operator_maintenance_unknown", "native_damping_inventory", "rearm_motion_profile"):
                return CancelResponse.REJECT
            if isinstance(self._active_goal, str) and self._active_goal_is("pending", "validating"):
                self._cancel_before_start = True
                return CancelResponse.ACCEPT
            if self._active_goal is not goal_handle:
                return CancelResponse.REJECT
            try:
                # Admission and dispatch share the lock with operator-mode reservation.
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
            artifact["controller_action_goal_uuid"] = [int(value) for value in goal_handle.goal_id.uuid]
            command_points = artifact["command_points"]
            if artifact["retiming_factor"] != 1.0 or len(command_points) != len(trajectory.points):
                trajectory = copy.deepcopy(trajectory)
                trajectory.points = []
                previous_ns = -1
                for effective in command_points:
                    timestamp_ns = round(effective["time"] * 1e9)
                    if timestamp_ns < 0 or timestamp_ns <= previous_ns:
                        raise ValueError("retimed feedback timestamps must remain nonnegative and strictly increasing")
                    previous_ns = timestamp_ns
                    sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
                    point = JointTrajectoryPoint(
                        positions=list(effective["positions"]),
                        time_from_start=Duration(sec=sec, nanosec=nanosec))
                    if "velocities" in effective:
                        point.velocities = list(effective["velocities"])
                    if "accelerations" in effective:
                        point.accelerations = list(effective["accelerations"])
                    trajectory.points.append(point)
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
        owner_process, owner_generation = self._process, self._owner_started
        dispatch_state = self._read_state()
        owner_session_id = dispatch_state.get("owner_session_id") if dispatch_state else None
        dispatched_at = time.monotonic()
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
                               "path_sha256": artifact["effective_path_sha256"],
                               "ros_motion_profile": artifact["ros_motion_profile"]}
                    if self._planning_contract is not None:
                        command["joint_limit_contract"] = self._planning_contract
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
                    self._mark_owner_fault("_execute")
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
                    self._mark_owner_fault("_execute")
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
        if (result_payload and (result_payload.get("status") == "failed_endpoint_held" or
                result_payload.get("hold") is not None or
                (isinstance(result_payload.get("state"), dict) and
                 result_payload["state"].get("failed_endpoint_hold") is not None))):
            try:
                held_state = self._validate_failed_endpoint_hold(result_payload,
                    goal_id=goal_id, process=owner_process, generation=owner_generation,
                    owner_session_id=owner_session_id, after=dispatched_at)
                self._mark_owner_fault("_execute")
                self._home_completed_owner_started = None
                artifact.update(status="failed_endpoint_held", target_reached=False,
                                qualified=False, result=result_payload)
                self._write_artifact(artifact)
                if (self._process is not owner_process or self._owner_started != owner_generation or
                        any(not 0 <= time.monotonic()-held_state[key] <= .1
                            for key in ("monotonic", "torque_monotonic"))):
                    raise RuntimeError("failed endpoint hold proof became stale")
                self._cache_owner_state(held_state, generation=owner_generation, process=owner_process)
                message = "failed_endpoint_held: target not reached; measured pose held; verified STOP required"
            except (OSError, ValueError, TypeError, KeyError, RuntimeError, TimeoutError) as error:
                self._mark_owner_fault("_execute")
                self._home_completed_owner_started = None
                try:
                    self._prepare_stop(owner_process, owner_generation)
                except (OSError, ValueError, TypeError, RuntimeError, TimeoutError) as stop_error:
                    error.add_note("failed hold proof cleanup unresolved: " + str(stop_error))
                message = "failed endpoint hold unavailable: " + str(error)
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, message)
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
            message = "calibration_measured_endpoint_held: measured endpoint held; plan target not reached"
            errors = result_payload.get("goal_error_rad")
            if isinstance(errors, dict):
                valid_errors = [(joint, value) for joint, value in errors.items()
                                if joint in ARM_JOINTS and type(value) in (int, float)
                                and value >= 0 and math.isfinite(value)]
                if valid_errors:
                    joint, error = max(valid_errors, key=lambda item: item[1])
                    message += f"; {joint} endpoint error {math.degrees(error):.2f}deg"
            return self._result(
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, message)
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
        # Signal first so an in-progress passive startup stops waiting. The same
        # lifecycle lock prevents a replacement from surviving node destruction.
        with self._teleop_wakeup_lock:
            self._closed = True
        self._close_recovery_depth_cache()
        if self._recovery_camera_identity is not None:
            self._recovery_camera_identity.close()
        with self._stop_lock:
            if self._process is not None and self._process.poll() is None:
                process = self._process
                try:
                    self._send({"command": "exit"})
                    process.wait(timeout=5.0)
                except (BrokenPipeError, OSError, RuntimeError, subprocess.TimeoutExpired):
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
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
