"""Launch MoveIt RViz through the station's GPU-only preflight."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from so101_station_gpu.gpu_renderer import rviz_preload_value


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    variant = LaunchConfiguration("variant")
    rviz_config = os.path.join(
        get_package_share_directory("so101_moveit_config"), "config", "moveit.rviz"
    )
    xacro_path = os.path.join(
        get_package_share_directory("so101_description"), "urdf", "so101_arm.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("so101_arm", package_name="so101_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={"variant": variant, "use_ros2_control": "false"},
        )
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits()
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="follower"),
        DeclareLaunchArgument("variant", default_value="follower"),
        Node(
            package="so101_station_gpu",
            executable="rviz_gpu_guard",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[moveit_config.to_dict()],
            remappings=[("joint_states", [namespace, "/joint_states"])],
            additional_env={"LD_PRELOAD": rviz_preload_value()},
        ),
    ])
