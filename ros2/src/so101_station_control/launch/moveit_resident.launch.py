"""Real planning stack plus the ArmSession action bridge; no ros2_control mock."""
import importlib.util
import json
import hashlib
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

from so101_station_control.model_offsets import apply_model_offsets, model_position_limits
from so101_station_gpu.gpu_renderer import rviz_preload_value

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _independent_joint_limits(limits):
    """Detach YAML aliases before applying any per-joint dynamics or positions."""
    return {joint: dict(values) for joint, values in limits.items()}


def _launch_nodes(context):
    station_root = Path(LaunchConfiguration("station_root").perform(context)).resolve()
    config_value = LaunchConfiguration("station_config").perform(context)
    arm_settings = {}
    if config_value:
        config_path = Path(config_value)
        if not config_path.is_absolute():
            config_path = station_root / config_path
        arm_settings = json.loads(config_path.read_text())["arm"]
    source = station_root / "workshop/station/ros_trajectory.py"
    spec = importlib.util.spec_from_file_location("so101_launch_motion_profiles", source)
    helper = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = helper
    exec(compile(source.read_bytes(), str(source), "exec"), helper.__dict__)
    if arm_settings:
        arm_settings = helper.resolve_joint_limit_settings(arm_settings, config_path)
    profile = helper.resolve_ros_motion_profile(arm_settings)
    description_share = get_package_share_directory("so101_description")
    config_share = get_package_share_directory("so101_moveit_config")
    xacro_path = os.path.join(description_share, "urdf", "so101_arm.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": "follower", "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .joint_limits(file_path=os.path.join(config_share, "config", "joint_limits.yaml"))
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )
    planning = moveit_config.joint_limits["robot_description_planning"]
    planning["joint_limits"] = _independent_joint_limits(planning["joint_limits"])
    for joint in ARM_JOINTS:
        moveit_config.joint_limits["robot_description_planning"]["joint_limits"][joint].update({
            "has_velocity_limits": True, "max_velocity": profile.max_velocity,
            "has_acceleration_limits": True, "max_acceleration": profile.max_acceleration,
            "has_jerk_limits": True, "max_jerk": profile.max_jerk,
        })
    # Anchor the marker at the configured TCP to avoid a rotating 98 mm offset.
    semantic = ET.fromstring(
        moveit_config.robot_description_semantic["robot_description_semantic"])
    end_effector = semantic.find("end_effector[@name='ee']")
    if (end_effector is None or
            end_effector.get("parent_link") not in {"gripper_link", "gripper_frame_link"}):
        raise RuntimeError("Expected end effector ee on gripper_link or gripper_frame_link")
    end_effector.set("parent_link", "gripper_frame_link")
    moveit_config.robot_description_semantic["robot_description_semantic"] = ET.tostring(
        semantic, encoding="unicode")
    offset_path = os.path.join(get_package_share_directory("so101_station_control"),
                               "config", "model_offsets.yaml")
    with open(offset_path, encoding="utf-8") as stream:
        model_offsets = yaml.safe_load(stream)
    moveit_config.robot_description["robot_description"] = apply_model_offsets(
        moveit_config.robot_description["robot_description"],
        model_offsets)
    contract_path = LaunchConfiguration("joint_limit_contract").perform(context)
    contract = None
    if contract_path:
        contract = json.loads(Path(contract_path).read_text())
        calibration_path = Path(arm_settings["calibration"]["path"])
        if not calibration_path.is_absolute():
            calibration_path = station_root / calibration_path
        data = calibration_path.read_bytes()
        if hashlib.sha256(data).hexdigest() != arm_settings["calibration"]["sha256"]:
            raise ValueError("planning calibration hash mismatch")
        source_spec = importlib.util.spec_from_file_location("so101_limit_sources", station_root / "workshop/station/recovery_types.py")
        source_helper = importlib.util.module_from_spec(source_spec)
        sys.modules[source_spec.name] = source_helper
        source_spec.loader.exec_module(source_helper)
        limits = helper.validate_joint_limit_contract(contract, arm_settings, json.loads(data),
            source_helper.owner_source_sha256(station_root / "workshop/station"))
        moveit_config.robot_description["robot_description"] = model_position_limits(
            moveit_config.robot_description["robot_description"], limits)
    else:
        limits = model_position_limits(moveit_config.robot_description["robot_description"])
    for joint, (low, high) in limits.items():
        moveit_config.joint_limits["robot_description_planning"]["joint_limits"][joint].update({
            "has_position_limits": True, "min_position": low, "max_position": high})
    # Keep the host DRM ABI override local to RViz. Its entrypoint probes GLX
    # with this environment before executing the renderer.
    rviz_environment = {"LD_PRELOAD": rviz_preload_value()}
    parameters = moveit_config.to_dict()
    parameters["moveit_controller_manager"] = "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    parameters["moveit_simple_controller_manager"] = {
        "controller_names": ["arm_trajectory_controller", "moveit_gripper_controller"],
        "arm_trajectory_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": ARM_JOINTS,
        },
    }
    parameters["moveit_simple_controller_manager"]["moveit_gripper_controller"] = {
        "type": "GripperCommand", "action_ns": "gripper_cmd", "default": True,
        "joints": ["gripper"], "command_joint": "gripper", "allow_failure": False,
    }
    parameters["moveit_manage_controllers"] = False
    # Include bounded bridge preflight and at most 4x controller retiming.
    # The owner's independent 100 ms feedback watchdog remains unchanged.
    parameters.setdefault("trajectory_execution", {}).update({
        "allowed_execution_duration_scaling": 5.0,
        "allowed_goal_duration_margin": 35.0,
    })

    bridge_parameters = {
        "joint_limit_contract": contract_path,
        "planning_position_limits_json": json.dumps(limits),
        "operator_service_units": [LaunchConfiguration(key).perform(context) for key in
                                   ("operator_camera_unit", "operator_depth_unit", "operator_tags_unit")],
        "operator_station_reset_units": [LaunchConfiguration(key).perform(context) for key in
                                   ("operator_movegroup_unit", "operator_objects_unit", "operator_scene_unit", "operator_leader_readonly_unit")],
        "operator_maintenance_journal": LaunchConfiguration("operator_maintenance_journal"),
        "enable_hardware": LaunchConfiguration("enable_hardware"),
        "external_state_only": LaunchConfiguration("external_state_only"),
        "startup_observe_only": ParameterValue(
            LaunchConfiguration("startup_observe_only"), value_type=bool),
        "station_python": LaunchConfiguration("station_python"),
        "station_root": LaunchConfiguration("station_root"),
        "station_config": LaunchConfiguration("station_config"),
        "state_file": LaunchConfiguration("state_file"),
        "log_dir": LaunchConfiguration("log_dir"),
        "trajectory_log_dir": LaunchConfiguration("trajectory_log_dir"),
        "station_lock": LaunchConfiguration("station_lock"),
        "recovery_depth_topic": LaunchConfiguration("recovery_depth_topic"),
        "recovery_camera_info_topic": LaunchConfiguration("recovery_camera_info_topic"),
        "gripper_action_name": LaunchConfiguration("gripper_action_name"),
        "gripper_jaw_calibration": LaunchConfiguration("gripper_jaw_calibration"),
        "gripper_joint_calibration": LaunchConfiguration("gripper_joint_calibration"),
        "gripper_joint_limits_rad": [float(ET.fromstring(
            moveit_config.robot_description["robot_description"]).find("joint[@name='gripper']/limit").get(k))
            for k in ("lower", "upper")],
        "state_validity_service": LaunchConfiguration("state_validity_service"),
        "planning_group": LaunchConfiguration("planning_group"),
        "collision_check_timeout_s": LaunchConfiguration("collision_check_timeout_s"),
        "max_jerk_rad_s3": LaunchConfiguration("max_jerk_rad_s3"),
        "action_name": "/follower/arm_trajectory_controller/follow_joint_trajectory",
    }
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="follower", name="robot_state_publisher",
             condition=IfCondition(LaunchConfiguration("use_robot_state_publisher")),
             parameters=[moveit_config.robot_description],
             remappings=[("joint_states", "/follower/joint_states")], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             namespace="follower", name="move_group", parameters=[parameters],
             condition=IfCondition(LaunchConfiguration("use_move_group")),
             remappings=[("joint_states", "/follower/joint_states")], output="screen"),
        Node(package="so101_station_gpu", executable="rviz_gpu_guard",
             namespace="follower", name="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")),
             additional_env=rviz_environment,
             arguments=["-d", LaunchConfiguration("rviz_config")],
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         moveit_config.joint_limits,
                         # 5DOF RViz position-drag mode; set orientation via joint
                         # controls. MoveGroup keeps strict kinematics.
                         {"default_planning_pipeline": "ompl",
                          "robot_description_kinematics.manipulator.rotation_scale": 0.0,
                          "robot_description_kinematics.manipulator.approximate_solution_position_threshold": 0.002,
                          "robot_description_planning.default_velocity_scaling_factor": 1.0,
                          "robot_description_planning.default_acceleration_scaling_factor": 1.0}],
             remappings=[("joint_states", "/follower/joint_states")], output="screen"),
        Node(package="so101_station_control", executable="trajectory_server",
             namespace="follower", name="trajectory_server",
             condition=IfCondition(LaunchConfiguration("use_trajectory_server")),
             parameters=[bridge_parameters], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value="true") for name in
          ("use_robot_state_publisher", "use_move_group", "use_trajectory_server")),
        *(DeclareLaunchArgument("operator_"+key+"_unit", default_value="") for key in
          ("movegroup", "objects", "scene", "leader_readonly")),
        DeclareLaunchArgument("leader", default_value="true",
                              description="Start the read-only calibrated leader publisher; does not enable follower teleop"),
        DeclareLaunchArgument("leader_port", default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00"),
        DeclareLaunchArgument("leader_calibration_dir", default_value="/home/bjoern/git/soarm101-highschool-demos/.calibration"),
        DeclareLaunchArgument("enable_hardware", default_value="false",
                              description="Explicitly enable the station Python bus owner"),
        DeclareLaunchArgument("startup_observe_only", default_value="true",
                              description="Start with read-only measured encoders; explicit Reset and Start restore motion admission"),
        DeclareLaunchArgument("external_state_only", default_value="false",
                              description="Read-only mirror of an existing owner state JSON; no station child and no accepted trajectory goals"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz with the station live scene configuration"),
        DeclareLaunchArgument("rviz_config",
                              default_value=os.path.join(get_package_share_directory("so101_station_perception"), "config", "station_live.rviz"),
                              description="RViz configuration for robot state and perception markers"),
        DeclareLaunchArgument("joint_limit_contract", default_value="",
                              description="Read-only calibrated owner limit export; regenerate after source/config/calibration change"),
        DeclareLaunchArgument("station_python", default_value="",
                              description="Absolute stationvenv Python executable"),
        DeclareLaunchArgument("operator_camera_unit", default_value=""),
        DeclareLaunchArgument("operator_depth_unit", default_value=""),
        DeclareLaunchArgument("operator_tags_unit", default_value=""),
        DeclareLaunchArgument("operator_maintenance_journal", default_value=str(Path.home()/".local/share/exciting-science/station/operator-maintenance-pending.json")),
        DeclareLaunchArgument("station_root", default_value=os.getcwd(),
                              description="Checkout containing workshop.station.arm_session"),
        DeclareLaunchArgument("station_config", default_value="",
                              description="Station config with exact arm identity and calibration"),
        DeclareLaunchArgument("state_file", default_value="",
                              description="Live state JSON; defaults to XDG_RUNTIME_DIR or /dev/shm"),
        DeclareLaunchArgument("log_dir", default_value="",
                              description="RAM-backed ArmSession runtime log directory (XDG_RUNTIME_DIR or /dev/shm)"),
        DeclareLaunchArgument("trajectory_log_dir", default_value="",
                              description="Persistent preflighted path and post-execution result artifacts"),
        DeclareLaunchArgument("station_lock", default_value="",
                              description="Canonical station lock path"),
        DeclareLaunchArgument("recovery_depth_topic",
                              default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("recovery_camera_info_topic",
                              default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("gripper_action_name",
                              default_value="/follower/gripper_controller/gripper_cmd",
                              description="GripperCommand action exposed by the resident bridge"),
        DeclareLaunchArgument("gripper_jaw_calibration", default_value="",
                              description="Path to qualified so101-jaw-gap-calibration-v1 JSON"),
        DeclareLaunchArgument("gripper_joint_calibration", default_value="",
                              description="Qualified identity-matched so101-gripper-joint-calibration-v1 JSON; radians, never gap metres"),
        DeclareLaunchArgument("state_validity_service", default_value="/follower/check_state_validity",
                              description="MoveIt state-validity service used to check the exact effective path"),
        DeclareLaunchArgument("planning_group", default_value="manipulator",
                              description="MoveIt group used for full-state collision validation"),
        DeclareLaunchArgument("collision_check_timeout_s", default_value="30.0",
                              description="Bounded total timeout for pre-motion state-validity calls"),
        DeclareLaunchArgument("max_jerk_rad_s3", default_value="2.0",
                              description="Software jerk admission limit; values above the 2 rad/s^3 station cap are rejected"),
        OpaqueFunction(function=_launch_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("so101_station_control"),
                                                       "launch", "leader_teleop.launch.py")),
            condition=IfCondition(LaunchConfiguration("leader")),
            launch_arguments={
                "station_python": LaunchConfiguration("station_python"),
                "station_root": LaunchConfiguration("station_root"),
                "leader_port": LaunchConfiguration("leader_port"),
                "calibration_dir": LaunchConfiguration("leader_calibration_dir"),
            }.items()),
    ])
