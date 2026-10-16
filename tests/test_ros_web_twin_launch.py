"""Launch-action evaluation for the source-rooted passive web twin."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
from launch.actions import ExecuteProcess
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration


ROOT = Path(__file__).resolve().parents[1]
WEB_TWIN_LAUNCH = ROOT / "ros2/src/so101_station_bringup/launch/ros_web_twin.launch.py"
STATION_LIVE_LAUNCH = ROOT / "ros2/src/so101_station_bringup/launch/station_live.launch.py"


def _load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _evaluate_process(action, launch_configurations):
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations)
    process = action.process_description
    process.prepare(context, action)
    return process.final_cwd, process.final_env, process.final_cmd


def test_web_twin_processes_use_explicit_station_root_from_any_cwd(tmp_path, monkeypatch):
    station_root = tmp_path / "station-source"
    wrong_cwd = tmp_path / "unrelated-current-directory"
    station_root.mkdir()
    wrong_cwd.mkdir()
    monkeypatch.chdir(wrong_cwd)
    description = _load_launch(WEB_TWIN_LAUNCH, "ros_web_twin_launch").generate_launch_description()
    config = {"station_root": str(station_root), "world_frame": "charuco_board",
              "host": "127.0.0.1", "port": "8102",
              "viewer_python": "/station/viewer/python",
              "placement_color_topic": "/camera/color/image_raw",
              "placement_depth_topic": "/camera/aligned_depth_to_color/image_raw",
              "placement_camera_info_topic": "/camera/color/camera_info",
              "placement_selection_timeout_s": "30.0"}
    processes = [action for action in description.entities if isinstance(action, ExecuteProcess)]
    assert len(processes) == 2

    for action in processes:
        cwd, env, command = _evaluate_process(action, config)
        assert cwd == str(station_root)
        assert env["SO101_STATION_ROOT"] == str(station_root)
        assert env["PYTHONPATH"].split(":", 1)[0] == str(station_root)
        assert command


def test_station_live_forwards_its_station_root_to_web_twin_include(tmp_path):
    from launch.actions import IncludeLaunchDescription

    description = _load_launch(STATION_LIVE_LAUNCH, "station_live_launch").generate_launch_description()
    includes = [action for action in description.entities
                if isinstance(action, IncludeLaunchDescription)]
    web_twin = next(action for action in includes
                    if {key for key, _ in action.launch_arguments}
                    == {"station_root", "host", "port", "viewer_python"})
    arguments = dict(web_twin.launch_arguments)
    assert "station_root" in arguments
    assert isinstance(arguments["station_root"], LaunchConfiguration)
