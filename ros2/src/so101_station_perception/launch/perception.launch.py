from ament_index_python.packages import get_package_prefix
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import math
import os

from so101_station_perception.calibration import load_calibration_frames
from so101_station_perception.camera_startup import camera_parameters, validate_repair


def _launch_nodes(context):
    serial = LaunchConfiguration("serial_no").perform(context)
    if serial != "243322071783":
        raise ValueError("Rig01 perception is pinned to D435i serial 243322071783")
    live_world = LaunchConfiguration("track_charuco_world").perform(context).lower() == "true"
    derived = LaunchConfiguration("marker_allow_derived_source").perform(context).lower()
    if derived not in ("true", "false"):
        raise ValueError("marker_allow_derived_source must be true or false")
    if derived == "true" and (not live_world or LaunchConfiguration("publish_tag_observations").perform(context).lower() != "true"):
        raise ValueError("derived aligned admission requires the shared live marker pipeline")
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
    far_depth = float(LaunchConfiguration("far_depth_m").perform(context))
    if not math.isfinite(far_depth) or far_depth <= 0.1:
        raise ValueError("far_depth_m must be finite and greater than 0.1")
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
    camera_settings = camera_parameters(*(LaunchConfiguration(key).perform(context) for key in
        ("color_profile", "depth_profile", "enable_sync", "far_depth_m", "camera_wait_timeout_s", "camera_reconnect_timeout_s")))
    sdk_prefix = Path(LaunchConfiguration("camera_sdk_prefix").perform(context)).expanduser().resolve()
    # The vendor Node resolves its executable through this same ament index.
    # Refuse an overlay executable different from the one traced by the guard.
    camera_enabled = LaunchConfiguration('use_camera').perform(context).lower()
    if camera_enabled not in ('true','false'):
        raise ValueError('use_camera must be true or false')
    repair = {"camera_environment": {"LD_PRELOAD": ""}}
    if camera_enabled == 'true':
        if Path(get_package_prefix("realsense2_camera")).resolve() != sdk_prefix:
            raise ValueError("camera package prefix differs from loader-validated SDK prefix")
        repair = validate_repair(LaunchConfiguration("camera_repair_prefix").perform(context),
                                 sdk_prefix, environment=context.environment)
    realsense_launch = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("so101_station_perception"), "config", "station_live.rviz"])
    # Only the vendor camera child receives this verified preload. Scoped launch
    # environment restoration keeps marker, arm and RViz processes untouched.
    camera_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "",
            "serial_no": "_" + serial,
            # Keep the full vendor camera/depth/color TF in both modes. The
            # qualified base transform is attached at camera_link by composing
            # the official camera_link-to-optical edge.
            # rs_launch applies YAML after these arguments. Keep this complete
            # explicit profile authoritative; static YAML would override a
            # requested color_profile with its conservative 640-wide default.
            "config_file": "''",
            "publish_tf": "true",
            "enable_color": "true",
            **camera_settings,
            "rgb_camera.color_format": "RGB8",
            "enable_depth": "true",
            "depth_module.depth_format": "Z16",
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
    )
    actions = [GroupAction(scoped=True, actions=[
        SetEnvironmentVariable("LD_PRELOAD", repair["camera_environment"]["LD_PRELOAD"]), camera_action])] if camera_enabled == 'true' else []
    if live_world:
        actions.append(Node(
            package="so101_station_perception", executable=("marker_pipeline" if LaunchConfiguration("publish_tag_observations").perform(context).lower() == "true" else "charuco_world"),
            additional_env={"CYCLONEDDS_URI": LaunchConfiguration("cyclonedds_uri")},
            name="so101_charuco_world", output="screen", parameters=[{
                "initial_base_from_camera_file": arm_path,
                "base_registration_file": LaunchConfiguration("base_registration_file").perform(context),
                "initial_camera_from_board_file": target_path,
                "optical_frame": optical_frame,
                **({"allow_derived_source": LaunchConfiguration("marker_allow_derived_source").perform(context).lower()=="true",
                    "derived_source_cohort": LaunchConfiguration("marker_derived_source_cohort").perform(context),
                    "depth_topic": LaunchConfiguration("marker_depth_topic").perform(context)}
                   if LaunchConfiguration("publish_tag_observations").perform(context).lower()=="true" else {}),
            }]))
    if transforms and not live_world and LaunchConfiguration('use_calibration_tf').perform(context).lower() == 'true':
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
    if not live_world and LaunchConfiguration("publish_tag_observations").perform(context).lower() == "true":
        actions.append(Node(
            package="so101_station_perception", executable="tag_observations",
            name="so101_tag_observations", output="screen",
            parameters=[{
                "color_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "depth_scale_m": float(LaunchConfiguration("depth_scale_m").perform(context)),
                "diagnostic_frame": "so101_diag_camera_optical" if unqualified_arm and not attach_candidate and not live_world else "",
            }],
        ))
    if unqualified_arm and not attach_candidate and not live_world and LaunchConfiguration('use_diagnostic_pointcloud').perform(context).lower() == 'true':
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
            package="so101_station_gpu", executable="rviz_gpu_guard", name="so101_station_live_rviz",
            arguments=["-d", rviz_config, "-f", "charuco_board" if live_world else base_frame], output="screen",
        ))
    return actions


def generate_launch_description():
    supervisor_keys=('expected_source_cohort', 'expected_serial', 'journal_directory', 'recovery_artifact_directory', 'confirmed_adoption_acceptance_artifact', 'confirmed_adoption_acceptance_sha256', 'confirmed_adoption_completed_artifact', 'confirmed_adoption_completed_sha256')
    supervisor=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("so101_station_perception"),"launch","perception_supervisor.launch.py"])),
        launch_arguments={"enabled":LaunchConfiguration("perception_supervisor_enabled"),
            "allow_private_availability_repair":LaunchConfiguration("perception_supervisor_allow_private_availability_repair"),
            **{key:LaunchConfiguration("perception_supervisor_"+key) for key in supervisor_keys}}.items(),
        condition=IfCondition(LaunchConfiguration("perception_supervisor_enabled")))
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value="true") for name in
          ("use_camera", "use_calibration_tf", "use_diagnostic_pointcloud")),
        DeclareLaunchArgument("cyclonedds_uri", default_value=EnvironmentVariable(
            "CYCLONEDDS_URI", default_value=PathJoinSubstitution([
                FindPackageShare("so101_station_perception"), "config", "cyclonedds_rgbd.xml"]))),
        DeclareLaunchArgument("perception_supervisor_enabled",default_value="false"),
        DeclareLaunchArgument("perception_supervisor_allow_private_availability_repair",default_value="false"),
        *(DeclareLaunchArgument("perception_supervisor_"+key,default_value="243322071783" if key=="expected_serial" else "") for key in supervisor_keys),
        supervisor,
        DeclareLaunchArgument("marker_allow_derived_source", default_value="false"),
        DeclareLaunchArgument("marker_derived_source_cohort", default_value=""),
        DeclareLaunchArgument("marker_depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("base_registration_file", default_value=""),
        DeclareLaunchArgument("track_charuco_world", default_value="false"),
        DeclareLaunchArgument("serial_no", default_value="243322071783"),
        DeclareLaunchArgument("far_depth_m", default_value="1.2",
                              description="Discard depth beyond the workcell before pointcloud generation"),
        DeclareLaunchArgument("color_profile", default_value="1920,1080,30"),
        DeclareLaunchArgument("depth_profile", default_value="1280,720,30"),
        DeclareLaunchArgument("enable_sync", default_value="false"),
        DeclareLaunchArgument("camera_wait_timeout_s", default_value="10.0"),
        DeclareLaunchArgument("camera_reconnect_timeout_s", default_value="2"),
        DeclareLaunchArgument("camera_repair_prefix", default_value=str(Path.cwd()/"ros2/.cache/realsense-cpu-texture-bounds/install")),
        DeclareLaunchArgument("camera_sdk_prefix", default_value=os.environ.get("CONDA_PREFIX", str(Path.home()/"miniforge3/envs/so101-jazzy"))),
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
