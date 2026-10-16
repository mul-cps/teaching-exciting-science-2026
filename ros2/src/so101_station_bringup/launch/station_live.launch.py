"""Measured robot, MoveIt, RealSense and diagnostic calibration in one scene."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        'leader_port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00',
        'calibration_dir': str(home / 'git/soarm101-highschool-demos/.calibration'),
        'state_file': str(runtime / 'arm-state.json'),
        'log_dir': str(runtime / 'arm-owner-logs'),
        'trajectory_log_dir': str(station / 'evidence/ros-control'),
        'gripper_action_name': '/follower/gripper_controller/gripper_cmd',
        'gripper_jaw_calibration': '',
        'station_lock': '',
        'rviz': 'true',
        'use_camera': 'true',
        'color_profile': '640,480,15',
        'far_depth_m': '1.2',
        'use_objects': 'true',
        'use_map': 'true',
        'use_planning_scene': 'true',
        'use_station_api': 'true',
        'sorting_config': str(station / 'sorting.json'),
        'sorting_evidence_dir': str(station / 'evidence/sorting'),
        'ev3_api_port': '18766',
        'use_ev3_tunnel': 'false',
        'ev3_remote': 'robot@192.168.0.1',
        'ev3_ssh_key': str(station / 'ssh/id_ev3'),
        'use_web_twin': 'true',
        'web_twin_host': '0.0.0.0',
        'web_twin_port': '8099',
        'web_twin_python': os.environ.get(
            'SO101_TWIN_VIEWER_PYTHON', str(station / 'venv/bin/python')),
        'track_charuco_world': 'true',
        'base_registration_file': str(station / 'calibration/robot_base_registration.json') if (station / 'calibration/robot_base_registration.json').is_file() else '',
        'attach_candidate': 'true',
        'camera_arm_calibration': '',
        'camera_target_calibration': '',
        'evidence_folder': '',
        # Object observations cover the full valid RGB-D frame by default.
        # Keep support fitting and pickup admission independently configurable.
        'max_object_height_m': '0.025',
        'observation_roi_enabled': 'false',
        'world_roi_x_min': '-0.03', 'world_roi_x_max': '0.21',
        'world_roi_y_min': '-0.03', 'world_roi_y_max': '0.16',
        'world_roi_z_min': '-0.05', 'world_roi_z_max': '0.06',
        'pickup_roi_x_min': '-0.03', 'pickup_roi_x_max': '0.21',
        'pickup_roi_y_min': '-0.03', 'pickup_roi_y_max': '0.16',
        'pickup_roi_z_min': '-0.05', 'pickup_roi_z_max': '0.06',
        'observation_roi_x_min': '-1.0', 'observation_roi_x_max': '1.0',
        'observation_roi_y_min': '-1.0', 'observation_roi_y_max': '1.0',
        'observation_roi_z_min': '-1.0', 'observation_roi_z_max': '1.0',
    }
    def include(package, filename, args, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            launch_arguments=args.items(), **kwargs)

    robot_keys = ['external_state_only', 'enable_hardware', 'station_root',
                  'station_python', 'station_config', 'state_file', 'log_dir',
                  'station_lock', 'rviz', 'trajectory_log_dir',
                  'gripper_action_name', 'gripper_jaw_calibration']
    robot = include('so101_station_control', 'moveit_resident.launch.py',
                    {k: LaunchConfiguration(k) for k in robot_keys})
    perception = include('so101_station_perception', 'perception.launch.py', {
        'color_profile': LaunchConfiguration('color_profile'),
        'far_depth_m': LaunchConfiguration('far_depth_m'),
        'track_charuco_world': LaunchConfiguration('track_charuco_world'),
        'base_registration_file': LaunchConfiguration('base_registration_file'),
        'camera_arm_calibration': LaunchConfiguration('camera_arm_calibration'),
        'attach_candidate': LaunchConfiguration('attach_candidate'),
        'camera_target_calibration': LaunchConfiguration('camera_target_calibration'),
        'resident_state_file': LaunchConfiguration('state_file'),
        'evidence_folder': LaunchConfiguration('evidence_folder'),
        'rviz': 'false',
    }, condition=IfCondition(LaunchConfiguration('use_camera')))
    leader = include('so101_station_control', 'leader_teleop.launch.py', {
        'station_python': LaunchConfiguration('station_python'),
        'station_root': LaunchConfiguration('station_root'),
        'leader_port': LaunchConfiguration('leader_port'),
        'calibration_dir': LaunchConfiguration('calibration_dir'),
    })
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        robot, leader, perception,
        ExecuteProcess(cmd=['ssh', '-N', '-T', '-o', 'ExitOnForwardFailure=yes',
                            '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                            '-o', 'IdentitiesOnly=yes', '-i', LaunchConfiguration('ev3_ssh_key'),
                            '-L', ['127.0.0.1:', LaunchConfiguration('ev3_api_port'), ':127.0.0.1:8766'],
                            LaunchConfiguration('ev3_remote')], output='screen',
                       condition=IfCondition(LaunchConfiguration('use_ev3_tunnel'))),
        Node(package='so101_station_workflow', executable='station_api',
             name='station_workflow', output='screen',
             additional_env={'SO101_STATION_ROOT': LaunchConfiguration('station_root')},
             parameters=[{'sorting_config': LaunchConfiguration('sorting_config'),
                          'evidence_dir': LaunchConfiguration('sorting_evidence_dir'),
                          'ev3_port': ParameterValue(
                              LaunchConfiguration('ev3_api_port'), value_type=int)}],
             condition=IfCondition(LaunchConfiguration('use_station_api'))),
        include('so101_station_bringup', 'ros_web_twin.launch.py', {
                    'station_root': LaunchConfiguration('station_root'),
                    'host': LaunchConfiguration('web_twin_host'),
                    'port': LaunchConfiguration('web_twin_port'),
                    'viewer_python': LaunchConfiguration('web_twin_python'),
                },
                condition=IfCondition(LaunchConfiguration('use_web_twin'))),
        include('so101_station_objects', 'objects.launch.py', {
                    'max_observation_depth_m': LaunchConfiguration('far_depth_m'),
                    **{key: LaunchConfiguration(key) for key in (
                        'max_object_height_m', 'observation_roi_enabled',
                        'world_roi_x_min', 'world_roi_x_max',
                        'world_roi_y_min', 'world_roi_y_max',
                        'world_roi_z_min', 'world_roi_z_max',
                        'pickup_roi_x_min', 'pickup_roi_x_max',
                        'pickup_roi_y_min', 'pickup_roi_y_max',
                        'pickup_roi_z_min', 'pickup_roi_z_max',
                        'observation_roi_x_min', 'observation_roi_x_max',
                        'observation_roi_y_min', 'observation_roi_y_max',
                        'observation_roi_z_min', 'observation_roi_z_max')}},
                condition=IfCondition(LaunchConfiguration('use_objects'))),
        include('so101_station_scene', 'station_scene.launch.py', {},
                condition=IfCondition(LaunchConfiguration('use_planning_scene'))),
        Node(package='so101_station_map', executable='surface_voxel_map',
             name='so101_surface_voxel_map', output='screen',
             condition=IfCondition(LaunchConfiguration('use_map'))),
    ])
