from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import math

from so101_station_perception.calibration import load_calibration_frames


def _launch_nodes(context):
    serial = LaunchConfiguration("serial_no").perform(context)
    if serial != "243322071783":
        raise ValueError("Rig01 perception is pinned to D435i serial 243322071783")
    target_path = LaunchConfiguration("camera_target_calibration").perform(context)
    arm_path = LaunchConfiguration("camera_arm_calibration").perform(context)
    qualified_value = LaunchConfiguration("use_qualified_frames").perform(context).lower()
    if qualified_value not in ("true", "false"):
        raise ValueError("use_qualified_frames must be true or false")
    qualified_mode = qualified_value == "true"
    attach_candidate = LaunchConfiguration("attach_candidate").perform(context).lower() == "true"
    base_frame = LaunchConfiguration("base_frame").perform(context)
    optical_frame = LaunchConfiguration("optical_frame").perform(context)
    transforms, driver_tf = load_calibration_frames(
        target_path, arm_path, qualified_mode=qualified_mode,
        base_frame=base_frame, optical_frame=optical_frame)
    depth_scale = float(LaunchConfiguration("depth_scale_m").perform(context))
    if not math.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError("depth_scale_m must be positive and finite")
    output_folder = LaunchConfiguration("evidence_folder").perform(context)
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower()
    if rviz_enabled not in ("true", "false"):
        raise ValueError("rviz must be true or false")
    arm_transform = next((item for item in transforms if item.get("kind") == "arm"), None)
    unqualified_arm = arm_transform is not None and arm_transform["qualified"] is not True
    resident_state_file = LaunchConfiguration("resident_state_file").perform(context)
    if output_folder:
        output_path = Path(output_folder)
        if output_path.exists() and (not output_path.is_dir() or any(output_path.iterdir())):
            raise ValueError("evidence_folder must be a new or empty directory")
        if not resident_state_file or not Path(resident_state_file).is_file():
            raise ValueError("evidence sink requires an existing resident_state_file")
    camera_config = PathJoinSubstitution([
        FindPackageShare("so101_station_perception"), "config", "realsense.yaml"])
    realsense_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("so101_station_perception"), "config", "station_live.rviz"])
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "",
            "serial_no": "_" + serial,
            # Keep the full vendor camera/depth/color TF in both modes. The
            # qualified base transform is attached at camera_link by composing
            # the official camera_link-to-optical edge.
            "config_file": camera_config,
            "publish_tf": "true",
            "enable_color": "true",
            "rgb_camera.color_profile": "640,480,15",
            "rgb_camera.color_format": "RGB8",
            "enable_depth": "true",
            "depth_module.depth_profile": "640,480,15",
            "depth_module.depth_format": "Z16",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "pointcloud.stream_filter": "2",
            "pointcloud.stream_index_filter": "0",
            "pointcloud.ordered_pc": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
        }.items(),
    )]
    if transforms:
        actions.append(Node(
            package="so101_station_perception", executable="calibration_tf_publisher",
            name="so101_calibration_tf", output="screen",
            parameters=[{
                "camera_target_calibration": target_path,
                "camera_arm_calibration": arm_path,
                "qualified_mode": qualified_mode,
                "attach_candidate": attach_candidate,
                "base_frame": base_frame,
                "optical_frame": optical_frame,
                "camera_link_frame": "camera_link",
            }],
        ))
    if LaunchConfiguration("publish_tag_observations").perform(context).lower() == "true":
        actions.append(Node(
            package="so101_station_perception", executable="tag_observations",
            name="so101_tag_observations", output="screen",
            parameters=[{
                "color_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "depth_scale_m": float(LaunchConfiguration("depth_scale_m").perform(context)),
                "diagnostic_frame": "so101_diag_camera_optical" if unqualified_arm and not attach_candidate else "",
            }],
        ))
    if unqualified_arm and not attach_candidate:
        actions.append(Node(
            package="so101_station_perception", executable="diagnostic_pointcloud",
            name="so101_diagnostic_pointcloud", output="screen",
            parameters=[{
                "input_topic": "/camera/depth/color/points",
                "output_topic": "/so101_perception/diagnostic/points",
                "color_optical_frame": optical_frame,
                "diagnostic_frame": "so101_diag_camera_optical",
                "max_publish_hz": 5.0,
            }],
        ))
    if output_folder:
        actions.append(Node(
            package="so101_station_perception", executable="rgbd_evidence_sink",
            name="so101_rgbd_evidence_sink", output="screen",
            parameters=[{
                "output_folder": output_folder,
                "resident_state_file": resident_state_file,
                "camera_serial": serial,
                "depth_scale_m": depth_scale,
                "color_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
            }],
        ))
    if rviz_enabled == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="so101_station_live_rviz",
            arguments=["-d", rviz_config], output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="243322071783"),
        DeclareLaunchArgument("camera_target_calibration", default_value=""),
        DeclareLaunchArgument("camera_arm_calibration", default_value=""),
        DeclareLaunchArgument("use_qualified_frames", default_value="false"),
        DeclareLaunchArgument("attach_candidate", default_value="false", description="Explicitly attach the current unqualified calibration estimate to the native camera tree; status remains provisional"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("optical_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("publish_tag_observations", default_value="true"),
        DeclareLaunchArgument("depth_scale_m", default_value="0.001"),
        DeclareLaunchArgument("evidence_folder", default_value=""),
        DeclareLaunchArgument("resident_state_file", default_value=""),
        OpaqueFunction(function=_launch_nodes),
    ])
