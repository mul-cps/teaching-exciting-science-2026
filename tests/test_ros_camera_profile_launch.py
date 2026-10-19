"""Describe launch actions only; these checks never start a camera or arm."""
from pathlib import Path
import sys
import importlib.util

import pytest

pytest.importorskip("launch.launch_context")
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2/src/so101_station_perception"))


def load_launch(relative):
    spec = importlib.util.spec_from_file_location("camera_profile_launch", ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module,"validate_repair"):
        module.validate_repair=lambda *a,**kw:{"camera_environment":{"LD_PRELOAD":"/fixture/verified/librealsense2_camera.so"}}
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


def test_perception_profiles_are_authoritative_over_conservative_yaml():
    module = load_launch("ros2/src/so101_station_perception/launch/perception.launch.py")
    context = defaults(module.generate_launch_description())
    assert context.launch_configurations["color_profile"] == "1920,1080,30"
    context.launch_configurations["color_profile"] = "848,480,15"
    actions = module._launch_nodes(context)
    group = next(action for action in actions if isinstance(action, GroupAction))
    camera = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
    config = arguments(camera, context)
    assert config["rgb_camera.color_profile"] == "848,480,15"
    # Vendor rs_launch applies its YAML after launch arguments. A static
    # 640-wide profile there silently overrides the explicit wider request.
    assert config.get("config_file", "''") == "''"
    assert config["depth_module.depth_profile"] == "1280,720,30"
    assert config["serial_no"] == "_243322071783"
    assert config["align_depth.enable"] == "true"
    assert config["clip_distance"] == "1.2"
    # Retain other station settings while the requested profiles/sync win.
    import yaml
    baseline = yaml.safe_load((ROOT / "ros2/config/realsense.yaml").read_text())
    for key, value in baseline.items():
        expected = {"rgb_camera.color_profile":"848,480,15", "depth_module.depth_profile":"1280,720,30", "enable_sync":"false"}.get(key, (
            str(value).lower() if isinstance(value, bool) else str(value)))
        assert config[key] == expected, key


def test_station_forwards_explicit_camera_knobs_to_single_perception_launch():
    module = load_launch("ros2/src/so101_station_bringup/launch/station_live.launch.py")
    description = module.generate_launch_description()
    context = defaults(description)
    assert context.launch_configurations["color_profile"] == "1920,1080,30"
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


def test_cold_defaults_persist_max_profiles_and_independent_depth_sync_knobs():
    module=load_launch('ros2/src/so101_station_bringup/launch/station_live.launch.py')
    context=defaults(module.generate_launch_description())
    assert context.launch_configurations['color_profile']=='1920,1080,30'
    assert context.launch_configurations['depth_profile']=='1280,720,30'
    assert context.launch_configurations['enable_sync']=='false'
    assert context.launch_configurations['camera_wait_timeout_s']=='10.0'
    assert context.launch_configurations['camera_reconnect_timeout_s']=='2'


def test_camera_preload_environment_is_scoped_and_restored_before_other_nodes(monkeypatch):
    module=load_launch('ros2/src/so101_station_perception/launch/perception.launch.py')
    context=defaults(module.generate_launch_description())
    context.launch_configurations['track_charuco_world']='true'
    context.launch_configurations['rviz']='true'
    monkeypatch.setenv('LD_PRELOAD','/parent/unrelated.so')
    actions=module._launch_nodes(context)
    group=next(a for a in actions if isinstance(a,GroupAction))
    children=group.get_sub_entities()
    camera=next(a for a in children if isinstance(a,IncludeLaunchDescription))
    # Execute only context/environment actions. Never visit camera or ROS nodes.
    for action in children:
        if action is camera:assert context.environment['LD_PRELOAD']=='/fixture/verified/librealsense2_camera.so'
        else:action.execute(context)
    assert context.environment['LD_PRELOAD']=='/parent/unrelated.so'
    assert sum(isinstance(a,SetEnvironmentVariable) for a in children)==1
    assert all(not isinstance(a,SetEnvironmentVariable) for a in actions if a is not group)


def test_missing_repair_fails_during_preparation_before_camera_action(tmp_path):
    module=load_launch('ros2/src/so101_station_perception/launch/perception.launch.py')
    from so101_station_perception.camera_startup import validate_repair
    module.validate_repair=validate_repair
    context=defaults(module.generate_launch_description())
    context.launch_configurations['camera_repair_prefix']=str(tmp_path/'missing/install')
    with pytest.raises(ValueError,match='repair|provenance'):module._launch_nodes(context)


def test_camera_overlay_executable_cannot_bypass_validated_sdk_identity():
    module=load_launch('ros2/src/so101_station_perception/launch/perception.launch.py')
    module.get_package_prefix=lambda name:'/unverified/overlay'
    context=defaults(module.generate_launch_description())
    with pytest.raises(ValueError,match='package prefix'):module._launch_nodes(context)


def test_explicit_depth_sync_and_timeout_arguments_reach_vendor_camera():
    module=load_launch('ros2/src/so101_station_perception/launch/perception.launch.py')
    context=defaults(module.generate_launch_description())
    context.launch_configurations.update(depth_profile='640,480,30',enable_sync='true',camera_wait_timeout_s='12',camera_reconnect_timeout_s='3')
    group=next(a for a in module._launch_nodes(context) if isinstance(a,GroupAction))
    camera=next(a for a in group.get_sub_entities() if isinstance(a,IncludeLaunchDescription))
    config=arguments(camera,context)
    assert config['depth_module.depth_profile']=='640,480,30' and config['enable_sync']=='true'
    assert config['wait_for_device_timeout']=='12.0' and config['reconnect_timeout']=='3.0'


def test_camera_preparation_precedes_owner_and_disabled_camera_keeps_bypass():
    module=load_launch('ros2/src/so101_station_bringup/launch/station_live.launch.py')
    description=module.generate_launch_description()
    includes=[a for a in description.entities if isinstance(a,IncludeLaunchDescription)]
    camera=next(a for a in includes if 'camera_repair_prefix' in dict(a.launch_arguments))
    owner=next(a for a in includes if 'station_lock' in dict(a.launch_arguments))
    assert description.entities.index(camera)<description.entities.index(owner)
    context=defaults(description);context.launch_configurations['use_camera']='false'
    assert camera.condition.evaluate(context) is False
