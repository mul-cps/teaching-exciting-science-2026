"""Launch provisional marker alignment only; existing marker pipeline supplies RGB-D."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from so101_station_perception.derived_lineage import configure_ros_domain


def generate_launch_description():
    configure_ros_domain(os.environ)
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise ValueError("live tag alignment requires ROS_LOCALHOST_ONLY=1")
    return LaunchDescription([
        DeclareLaunchArgument("bindings_file"),
        DeclareLaunchArgument("max_age_s", default_value="0.5"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
        Node(package="so101_station_perception", executable="live_tag_alignment", output="screen",
             parameters=[{"bindings_file": LaunchConfiguration("bindings_file"),
                          "max_age_s": ParameterValue(LaunchConfiguration("max_age_s"), value_type=float),
                          "camera_frame": LaunchConfiguration("camera_frame")}],
             additional_env={"ROS_DOMAIN_ID": os.environ["ROS_DOMAIN_ID"], "ROS_LOCALHOST_ONLY": "1",
                             "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"})
    ])
