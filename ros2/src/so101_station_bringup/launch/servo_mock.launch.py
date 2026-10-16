import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


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
        .joint_limits(file_path=os.path.join(config_share, "config", "joint_limits.yaml"))
        .to_moveit_configs()
    )

    servo_config = get_package_share_directory("so101_station_bringup") + "/config/servo.yaml"
    with open(servo_config, encoding="utf-8") as stream:
        servo_params = yaml.safe_load(stream)

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_bringup"), "launch", "follower_moveit_demo.launch.py"]
            )
        ),
        launch_arguments={
            "hardware_type": "mock",
            "namespace": "follower",
            "joint_config_file": "",
            "use_cameras": "false",
            "use_rviz": "false",
        }.items(),
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("so101_station_bringup"),
                "launch", "guarded_moveit_rviz.launch.py",
            ])
        ),
        launch_arguments={"namespace": "follower", "variant": "follower"}.items(),
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            {"planning_group_name": "manipulator", "update_period": 0.05},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([moveit_launch, rviz_launch, servo_node])
