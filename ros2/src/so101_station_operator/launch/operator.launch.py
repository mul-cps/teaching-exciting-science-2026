from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"station_config": "", "station_root": "", "enable_service_restarts": "false",
                "camera_unit": "", "depth_unit": "", "tags_unit": ""}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package="so101_station_operator", executable="operator_api", output="screen",
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=bool if name == "enable_service_restarts" else str)
                          for name in defaults}]),
    ])
