"""Default-off explicit source recovery. No camera/actuator process control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('artifact_directory', default_value=''),
        DeclareLaunchArgument('expected_source_cohort', default_value=''),
        DeclareLaunchArgument('expected_serial', default_value='243322071783'),
        Node(package='so101_station_perception', executable='perception_recovery',
             condition=IfCondition(LaunchConfiguration('enabled')), output='screen',
             parameters=[{'enabled': ParameterValue(LaunchConfiguration('enabled'),value_type=bool),
                'artifact_directory': LaunchConfiguration('artifact_directory'),
                'expected_source_cohort': LaunchConfiguration('expected_source_cohort'),
                'expected_serial': LaunchConfiguration('expected_serial')}]),
    ])
