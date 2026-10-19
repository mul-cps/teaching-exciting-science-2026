"""Opt-in, GPU-only synthetic camera; no physical robot/camera control."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from so101_station_gpu.gpu_renderer import rviz_preload_value


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID") == "0":
        raise ValueError("synthetic camera requires a nonzero ROS_DOMAIN_ID")
    enabled = LaunchConfiguration("enabled")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("pinch_artifact", default_value=""),
        DeclareLaunchArgument("publish_annotation", default_value="false"),
        Node(package="so101_station_perception", executable="virtual_gripper_camera",
            condition=IfCondition(enabled), output="screen", parameters=[{"enabled": True,
                "pinch_artifact": ParameterValue(LaunchConfiguration("pinch_artifact"), value_type=str),
                "publish_annotation": ParameterValue(LaunchConfiguration("publish_annotation"), value_type=bool)}],
            additional_env={"LD_PRELOAD": rviz_preload_value(),
                            "LIBGL_ALWAYS_SOFTWARE": "false", "GALLIUM_DRIVER": "radeonsi",
                            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "76"),
                            "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", "1")}),
    ])
