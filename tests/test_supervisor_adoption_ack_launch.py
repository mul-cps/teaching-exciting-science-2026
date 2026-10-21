"""Launch evaluation only, never executes a Node or includes a camera."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import normalize_parameters,evaluate_parameters
from so101_station_perception.adoption_ack import PARAMETERS
ROOT=Path(__file__).resolve().parents[1]


def module(path):
    spec=importlib.util.spec_from_file_location('launch_test',ROOT/path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def test_leaf_ack_inputs_are_typed_strings_and_default_off():
    desc=module('ros2/src/so101_station_perception/launch/perception_supervisor.launch.py').generate_launch_description()
    ctx=LaunchContext();ctx.launch_configurations.update(enabled='true',expected_serial='00123')
    for key,value in zip(PARAMETERS,('null','00123','false','1e3')):ctx.launch_configurations[key]=value
    node=next(a for a in desc.entities if isinstance(a,Node));params=evaluate_parameters(ctx,normalize_parameters(node._Node__parameters))
    merged={k:v for p in params for k,v in p.items()}
    for key in PARAMETERS:assert merged[key]==ctx.launch_configurations[key] and type(merged[key]) is str
    enabled=next(a for a in desc.entities if isinstance(a,DeclareLaunchArgument) and a.name=='enabled')
    assert ''.join(x.text for x in enabled.default_value)=='false'


def test_perception_and_station_have_real_default_off_leaf_forwarding():
    # Inspect actual launch objects and argument mappings; don't execute OpaqueFunction camera preflight.
    for path in ('ros2/src/so101_station_perception/launch/perception.launch.py','ros2/src/so101_station_bringup/launch/station_live.launch.py'):
        desc=module(path).generate_launch_description();args={a.name:a for a in desc.entities if isinstance(a,DeclareLaunchArgument)}
        assert ''.join(x.text for x in args['perception_supervisor_enabled'].default_value)=='false'
        for key in PARAMETERS:assert 'perception_supervisor_'+key in args
        ctx=LaunchContext()
        for argument in args.values():argument.execute(ctx)
        for key in PARAMETERS:ctx.launch_configurations['perception_supervisor_'+key]='00123'
        includes=[a for a in desc.entities if isinstance(a,IncludeLaunchDescription)]
        matches=[a for a in includes if any(k in ('confirmed_adoption_acceptance_artifact','perception_supervisor_confirmed_adoption_acceptance_artifact') for k,v in a.launch_arguments)]
        assert len(matches)==1
        pairs=dict(matches[0].launch_arguments)
        for key in PARAMETERS:
            value=pairs.get(key,pairs.get('perception_supervisor_'+key));assert value.perform(ctx)=='00123'
