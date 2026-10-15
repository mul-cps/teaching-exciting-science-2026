"""Measured robot, MoveIt, RealSense and diagnostic calibration in one scene."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    home = Path.home()
    station = home / '.local/share/exciting-science/station'
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/dev/shm')) / 'exciting-science'
    defaults = {
        'external_state_only': 'false',
        'enable_hardware': 'true',
        'station_root': os.getcwd(),
        'station_python': str(station / 'venv/bin/python'),
        'station_config': str(station / 'station.json'),
        'state_file': str(runtime / 'arm-state.json'),
        'log_dir': str(station / 'evidence/ros-control'),
        'station_lock': '',
        'rviz': 'true',
        'use_camera': 'true',
        'camera_arm_calibration': '',
        'camera_target_calibration': '',
        'evidence_folder': '',
    }
    def include(package, filename, args, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            launch_arguments=args.items(), **kwargs)

    robot_keys = ['external_state_only', 'enable_hardware', 'station_root',
                  'station_python', 'station_config', 'state_file', 'log_dir',
                  'station_lock', 'rviz']
    robot = include('so101_station_control', 'moveit_resident.launch.py',
                    {k: LaunchConfiguration(k) for k in robot_keys})
    perception = include('so101_station_perception', 'perception.launch.py', {
        'camera_arm_calibration': LaunchConfiguration('camera_arm_calibration'),
        'camera_target_calibration': LaunchConfiguration('camera_target_calibration'),
        'resident_state_file': LaunchConfiguration('state_file'),
        'evidence_folder': LaunchConfiguration('evidence_folder'),
        'rviz': 'false',
    }, condition=IfCondition(LaunchConfiguration('use_camera')))
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        robot, perception,
    ])
