from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    roi_defaults = {
        "world_roi_x_min": "-0.03", "world_roi_x_max": "0.21",
        "world_roi_y_min": "-0.03", "world_roi_y_max": "0.16",
        "world_roi_z_min": "-0.05", "world_roi_z_max": "0.06",
        "pickup_roi_x_min": "-0.03", "pickup_roi_x_max": "0.21",
        "pickup_roi_y_min": "-0.03", "pickup_roi_y_max": "0.16",
        "pickup_roi_z_min": "-0.05", "pickup_roi_z_max": "0.06",
        "observation_roi_x_min": "-1.0", "observation_roi_x_max": "1.0",
        "observation_roi_y_min": "-1.0", "observation_roi_y_max": "1.0",
        "observation_roi_z_min": "-1.0", "observation_roi_z_max": "1.0",
    }
    return LaunchDescription([
        DeclareLaunchArgument("allow_derived_source", default_value="false"),
        DeclareLaunchArgument("derived_source_cohort", default_value=""),
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("board_frame", default_value="charuco_board"),
        DeclareLaunchArgument("max_observation_depth_m", default_value="1.2",
                              description="Far optical-depth limit in metres for colored observations."),
        DeclareLaunchArgument("max_object_height_m", default_value="0.025",
                              description="Maximum support-plane height for grasp candidate admission."),
        DeclareLaunchArgument("observation_roi_enabled", default_value="false",
                              description="Optionally bound observations in board XYZ; defaults to the full valid RGB-D image."),
        *(DeclareLaunchArgument(name, default_value=value)
          for name, value in roi_defaults.items()),
        Node(package="so101_station_objects", executable="colored_objects",
             name="so101_colored_objects", output="screen", parameters=[{
                 "allow_derived_source": ParameterValue(LaunchConfiguration("allow_derived_source"), value_type=bool),
                 "derived_source_cohort": ParameterValue(LaunchConfiguration("derived_source_cohort"), value_type=str),
                 "color_topic": LaunchConfiguration("color_topic"),
                 "depth_topic": LaunchConfiguration("depth_topic"),
                 "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                 "board_frame": LaunchConfiguration("board_frame"),
                 "max_observation_depth_m": ParameterValue(
                     LaunchConfiguration("max_observation_depth_m"), value_type=float),
                 "max_object_height_m": LaunchConfiguration("max_object_height_m"),
                 "observation_roi_enabled": LaunchConfiguration("observation_roi_enabled"),
                 **{name: LaunchConfiguration(name) for name in roi_defaults},
             }]),
    ])
