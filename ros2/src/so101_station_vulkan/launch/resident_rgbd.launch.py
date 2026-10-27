"""Experimental acquisition; raw Vulkan requires cold SDK/source selfchecks."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='auto', choices=['auto', 'vulkan', 'cpu']),
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('profile', default_value='720p', choices=['vga', '720p', '1080p']),
        ComposableNodeContainer(
            name='resident_rgbd_container', namespace='',
            package='rclcpp_components', executable='component_container',
            output='screen', composable_node_descriptions=[
                ComposableNode(
                    package='so101_station_vulkan',
                    plugin='so101_station_vulkan::CameraComponent',
                    name='resident_camera',
                    parameters=[{'backend': LaunchConfiguration('backend'),
                                 'profile': ParameterValue(LaunchConfiguration('profile'), value_type=str),
                                 'serial': ParameterValue(LaunchConfiguration('serial'), value_type=str)}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='so101_station_vulkan',
                    plugin='so101_station_vulkan::ObjectsComponent',
                    name='resident_measurements',
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ])])
