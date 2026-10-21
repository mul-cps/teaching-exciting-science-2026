from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {"station_config": "", "station_root": "", "enable_service_restarts": "false",
                "camera_unit": "", "depth_unit": "", "tags_unit": "",
                "movegroup_unit": "", "objects_unit": "", "scene_unit": "", "leader_readonly_unit": "",
                "leader_required": "false", "alignment_reseed_service": "",
                "recovery_profile": "derived_rocm", "native_publisher_nodes_json": "",
                "native_mask_backend": "vulkan", "native_pair_delta_s": "0.04",
                "rgb_image_topic": "/camera/color/image_raw",
                "depth_image_topic": "/so101_perception/rocm/aligned_depth",
                "rgb_topic": "/camera/color/camera_info",
                "depth_topic": "/so101_perception/rocm/aligned_camera_info",
                "tags_topic": "/so101_perception/tag_detections",
                "objects_topic": "/so101/objects", "objects_status_topic": "/so101/grasp_status",
                "scene_status_topic": "/so101_scene/status", "joint_states_topic": "/follower/joint_states",
                "movegroup_scene_topic": "/follower/monitored_planning_scene"}
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package="so101_station_operator", executable="operator_api", output="screen",
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=bool if name in {"enable_service_restarts", "leader_required"} else float if name == "native_pair_delta_s" else str)
                          for name in defaults}]),
    ])
