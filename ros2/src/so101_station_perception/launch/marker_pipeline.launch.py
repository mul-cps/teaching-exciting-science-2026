from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = {
        "color_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/color/camera_info",
        "optical_frame": "camera_color_optical_frame",
        "board_frame": "charuco_board",
        "max_pair_delta_s": "0.04",
        "max_age_s": "0.5",
        "max_processing_hz": "5.0",
        "base_registration_file": "",
        "initial_base_from_camera_file": "",
        "initial_camera_from_board_file": "",
    }
    declarations = [DeclareLaunchArgument(key, default_value=value)
                    for key, value in names.items()]
    parameters = [{key: LaunchConfiguration(key)} for key in names]
    return LaunchDescription(declarations + [
        Node(package="so101_station_perception", executable="marker_pipeline",
             name="so101_marker_pipeline", output="screen", parameters=parameters),
    ])
