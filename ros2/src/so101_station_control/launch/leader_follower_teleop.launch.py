"""Bring up the single resident follower owner and read-only ROS leader publisher.

Starting this launch does not enable teleoperation. The operator must request
the follower teleop service after both arms report ready; stale sessions need
an explicit new enable request.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("so101_station_control")
    control_launch = os.path.join(share, "launch", "moveit_resident.launch.py")
    leader_launch = os.path.join(share, "launch", "leader_teleop.launch.py")
    station_python = LaunchConfiguration("station_python")
    station_root = LaunchConfiguration("station_root")

    return LaunchDescription([
        DeclareLaunchArgument("enable_hardware", default_value="false",
                              description="Explicitly start the one resident follower ArmSession owner"),
        DeclareLaunchArgument("station_python",
                              default_value="/home/bjoern/.local/share/exciting-science/station/venv/bin/python",
                              description="Station venv Python shared by the leader reader and follower owner"),
        DeclareLaunchArgument("station_root",
                              default_value="/home/bjoern/git/teaching-exciting-science-2026",
                              description="Checkout containing ArmSession and station helpers"),
        DeclareLaunchArgument("station_config",
                              default_value="/home/bjoern/.local/share/exciting-science/station/station.json",
                              description="Follower station identity and calibration configuration"),
        DeclareLaunchArgument("leader_port",
                              default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00",
                              description="Read-only leader USB identity"),
        DeclareLaunchArgument("calibration_dir",
                              default_value="/home/bjoern/git/soarm101-highschool-demos/.calibration",
                              description="Directory containing hs_rig01_leader.json and follower calibration"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Start RViz with the follower MoveIt scene"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                "enable_hardware": LaunchConfiguration("enable_hardware"),
                "external_state_only": "false",
                "leader": "false",  # This wrapper includes the leader once below.
                "station_python": station_python,
                "station_root": station_root,
                "station_config": LaunchConfiguration("station_config"),
                "rviz": LaunchConfiguration("rviz"),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(leader_launch),
            launch_arguments={
                "station_python": station_python,
                "station_root": station_root,
                "leader_port": LaunchConfiguration("leader_port"),
                "calibration_dir": LaunchConfiguration("calibration_dir"),
            }.items()),
    ])
