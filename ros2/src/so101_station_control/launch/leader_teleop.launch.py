"""Start the read-only leader state publisher. Follower teleop remains separately disabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("station_python",
                              default_value="/home/bjoern/.local/share/exciting-science/station/venv/bin/python",
                              description="LeRobot station environment Python executable"),
        DeclareLaunchArgument("station_root", default_value="/home/bjoern/git/teaching-exciting-science-2026",
                              description="Checkout used as the reader working directory"),
        DeclareLaunchArgument("leader_port",
                              default_value="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00",
                              description="Exact read-only SO-ARM101 leader USB identity"),
        DeclareLaunchArgument("calibration_dir",
                              default_value="/home/bjoern/git/soarm101-highschool-demos/.calibration",
                              description="Saved hs_rig01_leader.json calibration directory"),
        Node(package="so101_station_control", executable="leader_teleop", output="screen",
             parameters=[{
                 "station_python": LaunchConfiguration("station_python"),
                 "station_root": LaunchConfiguration("station_root"),
                 "leader_port": LaunchConfiguration("leader_port"),
                 "calibration_dir": LaunchConfiguration("calibration_dir"),
             }]),
    ])
