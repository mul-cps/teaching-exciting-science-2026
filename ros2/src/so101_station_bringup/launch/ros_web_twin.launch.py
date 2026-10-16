"""Passive web twin: bridge the exact same ROS state streams used by RViz."""
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root_default = os.environ.get("SO101_STATION_ROOT", os.getcwd())
    root = Path(root_default).resolve()
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    state_file = runtime / "exciting-science" / "ros-twin-state.json"
    placement_directory = runtime / "exciting-science" / "placement-camera"
    viewer_python = os.environ.get("SO101_TWIN_VIEWER_PYTHON",
                                  str(Path.home() / ".local/share/exciting-science/station/venv/bin/python"))
    env = os.environ.copy()
    env["PYTHONPATH"] = str(root) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    viewer_env = {key: value for key, value in os.environ.items() if key not in {
        "LD_LIBRARY_PATH", "LD_PRELOAD", "PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE",
        "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH", "ROS_DISTRO",
        "ROS_VERSION", "ROS_PYTHON_VERSION", "ROS_ETC_DIR", "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV", "CONDA_SHLVL", "CONDA_EXE", "CONDA_PYTHON_EXE",
    }}
    viewer_env["PYTHONPATH"] = str(root)
    bridge = ExecuteProcess(
        cmd=[sys.executable, "-m", "workshop.digital_twin.ros_state_node",
             "--output", str(state_file), "--root-frame", LaunchConfiguration("world_frame"),
             "--placement-directory", str(placement_directory),
             "--color-topic", LaunchConfiguration("placement_color_topic"),
             "--depth-topic", LaunchConfiguration("placement_depth_topic"),
             "--camera-info-topic", LaunchConfiguration("placement_camera_info_topic"),
             "--placement-selection-timeout", LaunchConfiguration("placement_selection_timeout_s")],
        cwd=str(root), env=env, output="screen", name="so101_ros_twin_state")
    return LaunchDescription([
        DeclareLaunchArgument("world_frame", default_value="charuco_board",
                              description="Common ROS world frame also shown in RViz"),
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8099"),
        DeclareLaunchArgument("viewer_python", default_value=viewer_python,
                              description="Python with MuJoCo and Viser installed"),
        DeclareLaunchArgument("placement_color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("placement_depth_topic",
                              default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("placement_camera_info_topic",
                              default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("placement_selection_timeout_s", default_value="30.0"),
        # Consume the declared override without shell evaluation.
        ExecuteProcess(cmd=[LaunchConfiguration("viewer_python"), "-m", "workshop.digital_twin",
                            "--ros-state-file", str(state_file), "--host",
                            LaunchConfiguration("host"), "--port", LaunchConfiguration("port")],
                       cwd=str(root), env=viewer_env, output="screen", name="so101_ros_web_twin",
                       respawn=False),
        bridge,
    ])
