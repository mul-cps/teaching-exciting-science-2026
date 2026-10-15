from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("board_frame", default_value="charuco_board"),
        Node(package="so101_station_objects", executable="colored_objects",
             name="so101_colored_objects", output="screen", parameters=[{
                 "color_topic": LaunchConfiguration("color_topic"),
                 "depth_topic": LaunchConfiguration("depth_topic"),
                 "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                 "board_frame": LaunchConfiguration("board_frame"),
             }]),
    ])
