"""No node, worker, camera or GPU context unless explicitly enabled."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('configuration_file', default_value=''),
        DeclareLaunchArgument('compatibility_library', default_value=''),
        DeclareLaunchArgument('worker_python', default_value='/home/bjoern/miniforge3/bin/python'),
        Node(package='so101_station_perception', executable='rocm_rgbd', output='screen',
             condition=IfCondition(LaunchConfiguration('enabled')), parameters=[{
                 'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                 'configuration_file': LaunchConfiguration('configuration_file'),
                 'compatibility_library': LaunchConfiguration('compatibility_library'),
                 'worker_python': LaunchConfiguration('worker_python')}]),
    ])
