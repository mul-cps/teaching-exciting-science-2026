"""Describe launch actions only; these checks never start a camera or arm."""
from pathlib import Path
import importlib.util

import pytest

pytest.importorskip("launch.launch_context")
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

ROOT = Path(__file__).resolve().parents[1]


def load_launch(relative):
    spec = importlib.util.spec_from_file_location("camera_profile_launch", ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def defaults(description):
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    return context


def arguments(action, context):
    return {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
            for name, value in action.launch_arguments}


def test_perception_allows_wider_color_without_changing_depth_or_default():
    module = load_launch("ros2/src/so101_station_perception/launch/perception.launch.py")
    context = defaults(module.generate_launch_description())
    assert context.launch_configurations["color_profile"] == "640,480,15"
    context.launch_configurations["color_profile"] = "848,480,15"
    actions = module._launch_nodes(context)
    camera = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    config = arguments(camera, context)
    assert config["rgb_camera.color_profile"] == "848,480,15"
    # Vendor rs_launch applies its YAML after launch arguments. A static
    # 640-wide profile there silently overrides the explicit wider request.
    assert config.get("config_file", "''") == "''"
    assert config["depth_module.depth_profile"] == "640,480,15"
    assert config["serial_no"] == "_243322071783"
    assert config["align_depth.enable"] == "true"
    assert config["clip_distance"] == "1.2"


def test_station_forwards_explicit_color_profile_to_its_single_perception_launch():
    module = load_launch("ros2/src/so101_station_bringup/launch/station_live.launch.py")
    description = module.generate_launch_description()
    context = defaults(description)
    assert context.launch_configurations["color_profile"] == "640,480,15"
    context.launch_configurations["color_profile"] = "848,480,15"
    matches = [arguments(action, context) for action in description.entities
               if isinstance(action, IncludeLaunchDescription)
               and "color_profile" in dict(action.launch_arguments)]
    assert len(matches) == 1
    assert matches[0]["color_profile"] == "848,480,15"


@pytest.mark.parametrize("limit", ["nan", "inf", "0", "-1", "0.1"])
def test_perception_rejects_invalid_far_depth(limit):
    module = load_launch("ros2/src/so101_station_perception/launch/perception.launch.py")
    context = defaults(module.generate_launch_description())
    context.launch_configurations["far_depth_m"] = limit
    with pytest.raises(ValueError, match="far_depth_m"):
        module._launch_nodes(context)

def test_shared_far_depth_reaches_camera_and_detector():
    module = load_launch("ros2/src/so101_station_bringup/launch/station_live.launch.py")
    description = module.generate_launch_description()
    context = defaults(description)
    context.launch_configurations["far_depth_m"] = "1.4"
    configs = [arguments(a, context) for a in description.entities
               if isinstance(a, IncludeLaunchDescription)]
    assert next(c for c in configs if "far_depth_m" in c)["far_depth_m"] == "1.4"
    assert next(c for c in configs if "max_observation_depth_m" in c)["max_observation_depth_m"] == "1.4"
