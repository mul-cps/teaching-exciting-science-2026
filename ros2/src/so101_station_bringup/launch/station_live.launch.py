"""Measured robot, MoveIt, RealSense and diagnostic calibration in one scene."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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
        'log_dir': str(runtime / 'arm-owner-logs'),
        'trajectory_log_dir': str(station / 'evidence/ros-control'),
        'station_lock': '',
        'rviz': 'true',
        'use_camera': 'true',
        'use_objects': 'true',
        'use_map': 'true',
        'use_planning_scene': 'true',
        'track_charuco_world': 'true',
        'base_registration_file': str(station / 'calibration/robot_base_registration.json') if (station / 'calibration/robot_base_registration.json').is_file() else '',
        'attach_candidate': 'true',
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
                  'station_lock', 'rviz', 'trajectory_log_dir']
    robot = include('so101_station_control', 'moveit_resident.launch.py',
                    {k: LaunchConfiguration(k) for k in robot_keys})
    perception = include('so101_station_perception', 'perception.launch.py', {
        'track_charuco_world': LaunchConfiguration('track_charuco_world'),
        'base_registration_file': LaunchConfiguration('base_registration_file'),
        'camera_arm_calibration': LaunchConfiguration('camera_arm_calibration'),
        'attach_candidate': LaunchConfiguration('attach_candidate'),
        'camera_target_calibration': LaunchConfiguration('camera_target_calibration'),
        'resident_state_file': LaunchConfiguration('state_file'),
        'evidence_folder': LaunchConfiguration('evidence_folder'),
        'rviz': 'false',
    }, condition=IfCondition(LaunchConfiguration('use_camera')))
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        robot, perception,
        include('so101_station_objects', 'objects.launch.py', {},
                condition=IfCondition(LaunchConfiguration('use_objects'))),
        include('so101_station_scene', 'station_scene.launch.py', {},
                condition=IfCondition(LaunchConfiguration('use_planning_scene'))),
        Node(package='so101_station_map', executable='surface_voxel_map',
             name='so101_surface_voxel_map', output='screen',
             condition=IfCondition(LaunchConfiguration('use_map'))),
    ])
