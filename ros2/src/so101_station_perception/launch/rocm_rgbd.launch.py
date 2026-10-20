"""No node, worker, camera or GPU context unless explicitly enabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cyclonedds_uri', default_value=EnvironmentVariable(
            'CYCLONEDDS_URI', default_value=PathJoinSubstitution([
                FindPackageShare('so101_station_perception'), 'config', 'cyclonedds_rgbd.xml']))),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('allow_explicit_reset', default_value='false'),
        DeclareLaunchArgument('configuration_file', default_value=''),
        DeclareLaunchArgument('compatibility_library', default_value=''),
        DeclareLaunchArgument('worker_python', default_value='/home/bjoern/miniforge3/bin/python'),
        Node(package='so101_station_perception', executable='rocm_rgbd', output='screen',
             additional_env={'CYCLONEDDS_URI': LaunchConfiguration('cyclonedds_uri')},
             condition=IfCondition(LaunchConfiguration('enabled')), parameters=[{
                 'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                 'allow_explicit_reset': ParameterValue(LaunchConfiguration('allow_explicit_reset'), value_type=bool),
                 'configuration_file': LaunchConfiguration('configuration_file'),
                 'compatibility_library': LaunchConfiguration('compatibility_library'),
                 'worker_python': LaunchConfiguration('worker_python')}]),
    ])
