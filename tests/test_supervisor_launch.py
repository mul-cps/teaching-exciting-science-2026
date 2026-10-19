import importlib.util
from pathlib import Path
import os
import pytest
assert os.environ.get('ROS_DOMAIN_ID')=='177','Use fixed177 supervisor test launcher'


def test_launch_preserves_paths_and_leading_zero_serial_strings_without_node_execution():
    from launch import LaunchContext
    from launch_ros.utilities import normalize_parameters,evaluate_parameters
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('supervisor_launch',root/'ros2/src/so101_station_perception/launch/perception_supervisor.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description()
    from launch_ros.actions import Node
    node=next(a for a in description.entities if isinstance(a,Node))
    context=LaunchContext()
    values={'enabled':'true','expected_serial':'001234567890','expected_source_cohort':'null','journal_directory':'00123','recovery_artifact_directory':'false'}
    context.launch_configurations.update(values)
    params=evaluate_parameters(context,normalize_parameters(node._Node__parameters))
    merged={k:v for item in params for k,v in item.items()}
    assert merged['enabled'] is True
    for key,value in values.items():
        if key!='enabled':assert merged[key]==value and type(merged[key]) is str
