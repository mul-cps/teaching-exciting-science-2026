"""Evaluate launch parameter types only; no node/process/service is started."""
import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('launch.launch_context')
pytest.importorskip('launch_ros.actions')
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('overrides', [
    {},
    {'enabled': 'true', 'artifact_directory': '/tmp/recovery artifacts',
     'expected_source_cohort': '/tmp/source.json', 'expected_serial': '243322071783'},
    {'enabled': 'true', 'artifact_directory': '123',
     'expected_source_cohort': 'false', 'expected_serial': '000123'},
])
def test_recovery_launch_preserves_all_three_strings_and_typed_enabled(overrides):
    path = ROOT / 'ros2/src/so101_station_perception/launch/perception_recovery.launch.py'
    spec = importlib.util.spec_from_file_location('recovery_launch_types', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(overrides)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    values = {key: value for group in evaluate_parameters(context, node._Node__parameters)
              for key, value in group.items()}
    for key in ('artifact_directory', 'expected_source_cohort', 'expected_serial'):
        assert type(values[key]) is str
        assert values[key] == context.launch_configurations[key]
    assert type(values['enabled']) is bool
    assert values['enabled'] is (context.launch_configurations['enabled'] == 'true')
    assert node.condition.evaluate(context) is values['enabled']
