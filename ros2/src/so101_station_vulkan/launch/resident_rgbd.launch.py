"""Experimental composed acquisition; strict raw-camera Vulkan is gated closed."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='auto', choices=['auto', 'vulkan', 'cpu']),
        DeclareLaunchArgument('serial', default_value=''),
        ComposableNodeContainer(
            name='resident_rgbd_container', namespace='',
            package='rclcpp_components', executable='component_container',
            output='screen', composable_node_descriptions=[
                ComposableNode(
                    package='so101_station_vulkan',
                    plugin='so101_station_vulkan::CameraComponent',
                    name='resident_camera',
                    parameters=[{'backend': LaunchConfiguration('backend'),
                                 'serial': LaunchConfiguration('serial')}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='so101_station_vulkan',
                    plugin='so101_station_vulkan::ObjectsComponent',
                    name='resident_measurements',
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ])])
