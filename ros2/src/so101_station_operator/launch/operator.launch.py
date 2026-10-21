from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"station_config": "", "station_root": "", "enable_service_restarts": "false",
                "camera_unit": "", "depth_unit": "", "tags_unit": "",
                "movegroup_unit": "", "objects_unit": "", "scene_unit": "", "leader_readonly_unit": "",
                "leader_required": "false", "alignment_reseed_service": ""}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package="so101_station_operator", executable="operator_api", output="screen",
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=bool if name in {"enable_service_restarts", "leader_required"} else str)
                          for name in defaults}]),
    ])
