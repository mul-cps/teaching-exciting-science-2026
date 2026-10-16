"""Resolve actual object-detector Node parameters without launching a process."""
from pathlib import Path
import importlib.util

import pytest

pytest.importorskip('launch.launch_context')
pytest.importorskip('launch_ros.actions')
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('override,expected', [(None,1.2),('1.4',1.4)])
def test_detector_node_receives_typed_default_and_overridden_far_depth(override,expected):
    path = ROOT/'ros2/src/so101_station_objects/launch/objects.launch.py'
    spec = importlib.util.spec_from_file_location('objects_depth_launch',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context,action.default_value)
    assert context.launch_configurations['max_observation_depth_m'] == '1.2'
    if override is not None:
        context.launch_configurations['max_observation_depth_m'] = override
    nodes = [action for action in description.entities if isinstance(action,Node)]
    assert len(nodes) == 1
    # launch_ros stores the actual normalized Node parameter payload privately;
    # evaluate it with the same utility used during Node substitution expansion.
    evaluated = evaluate_parameters(context,nodes[0]._Node__parameters)
    parameters = {key:value for group in evaluated for key,value in group.items()}
    assert parameters['max_observation_depth_m'] == expected
    assert type(parameters['max_observation_depth_m']) is float
