"""Optional proposal bridge only. Does not start Intel, a camera or robot control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("backend_service", default_value="/intel_grasp/plan_grasps"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        Node(package="so101_station_scene", executable="intel_grasp_adapter",
             condition=IfCondition(LaunchConfiguration("enabled")),
             parameters=[{"enabled": ParameterValue(LaunchConfiguration("enabled"), value_type=bool),
                          "backend_service": LaunchConfiguration("backend_service"),
                          "frame_id": LaunchConfiguration("frame_id")}], output="screen"),
    ])
