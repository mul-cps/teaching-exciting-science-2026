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
        'recovery_depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'recovery_camera_info_topic': '/camera/color/camera_info',
        'color_profile': '1920,1080,30',
        'depth_profile': '1280,720,30',
        'enable_sync': 'false',
        'camera_wait_timeout_s': '10.0',
        'camera_reconnect_timeout_s': '2',
        'camera_repair_prefix': str(Path.cwd()/'ros2/.cache/realsense-cpu-texture-bounds/install'),
        'camera_sdk_prefix': os.environ.get('CONDA_PREFIX', str(home/'miniforge3/envs/so101-jazzy')),
        'far_depth_m': '1.2',
        'marker_allow_derived_source': 'false',
        'marker_derived_source_cohort': '',
        'marker_depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'objects_allow_derived_source': 'false',
        'objects_derived_source_cohort': '',
        'objects_depth_topic': '/camera/aligned_depth_to_color/image_raw',
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
    supervisor_defaults={"perception_supervisor_enabled":"false","perception_supervisor_allow_private_availability_repair":"false",
        **{"perception_supervisor_"+key:("243322071783" if key=="expected_serial" else "") for key in ('expected_source_cohort', 'expected_serial', 'journal_directory', 'recovery_artifact_directory', 'confirmed_adoption_acceptance_artifact', 'confirmed_adoption_acceptance_sha256', 'confirmed_adoption_completed_artifact', 'confirmed_adoption_completed_sha256')}}
    defaults.update(supervisor_defaults)
    def include(package, filename, args, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare(package), 'launch', filename])),
            launch_arguments=args.items(), **kwargs)

    robot_keys = ['external_state_only', 'enable_hardware', 'station_root',
                  'station_python', 'station_config', 'state_file', 'log_dir',
                  'station_lock', 'rviz', 'trajectory_log_dir',
                  'gripper_action_name', 'gripper_jaw_calibration',
                  'recovery_depth_topic', 'recovery_camera_info_topic']
    robot = include('so101_station_control', 'moveit_resident.launch.py',
                    {k: LaunchConfiguration(k) for k in robot_keys})
    perception = include('so101_station_perception', 'perception.launch.py', {
        **{key:LaunchConfiguration(key) for key in supervisor_defaults},
        **{key:LaunchConfiguration(key) for key in ('color_profile','depth_profile','enable_sync','camera_wait_timeout_s','camera_reconnect_timeout_s','camera_repair_prefix','camera_sdk_prefix')},
        **{key: LaunchConfiguration(key) for key in ('marker_allow_derived_source', 'marker_derived_source_cohort', 'marker_depth_topic')},
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
        # Camera repair preparation must fail before constructing owner actions.
        # This orders launch validation; it does not assert camera stream readiness.
        perception, robot, leader,
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
                    'allow_derived_source': LaunchConfiguration('objects_allow_derived_source'),
                    'derived_source_cohort': LaunchConfiguration('objects_derived_source_cohort'),
                    'depth_topic': LaunchConfiguration('objects_depth_topic'),
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
