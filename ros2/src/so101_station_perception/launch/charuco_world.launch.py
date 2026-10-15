from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("optical_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("camera_link_frame", default_value="camera_link"),
        DeclareLaunchArgument("board_frame", default_value="charuco_board"),
        DeclareLaunchArgument("initial_base_from_camera_file", default_value=""),
        DeclareLaunchArgument("initial_camera_from_board_file", default_value=""),
        Node(package="so101_station_perception", executable="charuco_world",
             name="so101_charuco_world", output="screen", parameters=[{
                 "color_topic": LaunchConfiguration("color_topic"),
                 "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                 "optical_frame": LaunchConfiguration("optical_frame"),
                 "camera_link_frame": LaunchConfiguration("camera_link_frame"),
                 "board_frame": LaunchConfiguration("board_frame"),
                 "initial_base_from_camera_file": LaunchConfiguration("initial_base_from_camera_file"),
                 "initial_camera_from_board_file": LaunchConfiguration("initial_camera_from_board_file"),
             }]),
    ])
