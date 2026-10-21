"""Real planning stack plus the ArmSession action bridge; no ros2_control mock."""
import os
import xml.etree.ElementTree as ET
import yaml

from so101_station_control.model_offsets import apply_model_offsets
from so101_station_gpu.gpu_renderer import rviz_preload_value

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def generate_launch_description():
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
    # Keep the host DRM ABI override local to RViz. Its entrypoint probes GLX
    # with this environment before executing the renderer.
    rviz_environment = {"LD_PRELOAD": rviz_preload_value()}
    parameters = moveit_config.to_dict()
    parameters["moveit_controller_manager"] = "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    parameters["moveit_simple_controller_manager"] = {
        "controller_names": ["arm_trajectory_controller"],
        "arm_trajectory_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": ARM_JOINTS,
        },
    }
    parameters["moveit_manage_controllers"] = False
    # Include bounded bridge preflight and at most 4x controller retiming.
    # The owner's independent 100 ms feedback watchdog remains unchanged.
    parameters.setdefault("trajectory_execution", {}).update({
        "allowed_execution_duration_scaling": 5.0,
        "allowed_goal_duration_margin": 35.0,
    })

    bridge_parameters = {
        "enable_hardware": LaunchConfiguration("enable_hardware"),
        "external_state_only": LaunchConfiguration("external_state_only"),
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
        "state_validity_service": LaunchConfiguration("state_validity_service"),
        "planning_group": LaunchConfiguration("planning_group"),
        "collision_check_timeout_s": LaunchConfiguration("collision_check_timeout_s"),
        "max_jerk_rad_s3": LaunchConfiguration("max_jerk_rad_s3"),
        "action_name": "/follower/arm_trajectory_controller/follow_joint_trajectory",
    }
    return LaunchDescription([
        DeclareLaunchArgument("enable_hardware", default_value="false",
                              description="Explicitly enable the station Python bus owner"),
        DeclareLaunchArgument("external_state_only", default_value="false",
                              description="Read-only mirror of an existing owner state JSON; no station child and no accepted trajectory goals"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz with the station live scene configuration"),
        DeclareLaunchArgument("rviz_config",
                              default_value=os.path.join(get_package_share_directory("so101_station_perception"), "config", "station_live.rviz"),
                              description="RViz configuration for robot state and perception markers"),
        DeclareLaunchArgument("station_python", default_value="",
                              description="Absolute stationvenv Python executable"),
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
        DeclareLaunchArgument("state_validity_service", default_value="/follower/check_state_validity",
                              description="MoveIt state-validity service used to check the exact effective path"),
        DeclareLaunchArgument("planning_group", default_value="manipulator",
                              description="MoveIt group used for full-state collision validation"),
        DeclareLaunchArgument("collision_check_timeout_s", default_value="30.0",
                              description="Bounded total timeout for pre-motion state-validity calls"),
        DeclareLaunchArgument("max_jerk_rad_s3", default_value="2.0",
                              description="Software jerk admission limit; values above the 2 rad/s^3 station cap are rejected"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="follower", name="robot_state_publisher",
             parameters=[moveit_config.robot_description],
             remappings=[("joint_states", "/follower/joint_states")], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             namespace="follower", name="move_group", parameters=[parameters],
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
                          "robot_description_planning.default_velocity_scaling_factor": 0.4,
                          "robot_description_planning.default_acceleration_scaling_factor": 0.4}],
             remappings=[("joint_states", "/follower/joint_states")], output="screen"),
        Node(package="so101_station_control", executable="trajectory_server",
             namespace="follower", name="trajectory_server",
             parameters=[bridge_parameters], output="screen"),
    ])
