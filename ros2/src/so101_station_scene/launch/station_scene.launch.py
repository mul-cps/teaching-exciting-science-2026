"""Run tracked scene updates and plan-only grasp-candidate checks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("so101_station_scene")
    return LaunchDescription([
        DeclareLaunchArgument("validate_grasps", default_value="true"),
        Node(package="so101_station_scene", executable="scene_updater",
             name="so101_scene_updater",
             parameters=[os.path.join(share, "config", "scene_updater.yaml")],
             output="screen"),
        Node(package="so101_station_scene", executable="grasp_validator",
             name="so101_grasp_validator",
             condition=IfCondition(LaunchConfiguration("validate_grasps")),
             output="screen"),
    ])
