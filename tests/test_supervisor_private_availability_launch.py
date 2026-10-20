"""Actual launch parameter evaluation; no launch execution."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
from test_supervisor_adoption_ack_launch import module
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import normalize_parameters,evaluate_parameters


def test_private_repair_default_false_and_explicit_bool_across_three_layers():
    desc=module('ros2/src/so101_station_perception/launch/perception_supervisor.launch.py').generate_launch_description()
    node=next(a for a in desc.entities if isinstance(a,Node));ctx=LaunchContext();ctx.launch_configurations['enabled']='true'
    for value,expected in (('false',False),('true',True)):
        desc=module('ros2/src/so101_station_perception/launch/perception_supervisor.launch.py').generate_launch_description()
        node=next(a for a in desc.entities if isinstance(a,Node))
        ctx.launch_configurations['allow_private_availability_repair']=value
        params=evaluate_parameters(ctx,normalize_parameters(node._Node__parameters));merged={k:v for p in params for k,v in p.items()}
        assert merged['allow_private_availability_repair'] is expected
    for path in ('ros2/src/so101_station_perception/launch/perception.launch.py','ros2/src/so101_station_bringup/launch/station_live.launch.py'):
        desc=module(path).generate_launch_description();name='perception_supervisor_allow_private_availability_repair'
        arg=next(a for a in desc.entities if isinstance(a,DeclareLaunchArgument) and a.name==name)
        assert ''.join(v.text for v in arg.default_value)=='false'
        ctx.launch_configurations[name]='true'
        matches=[a for a in desc.entities if isinstance(a,IncludeLaunchDescription) and any(k in (name,'allow_private_availability_repair') for k,v in a.launch_arguments)]
        assert len(matches)==1
        assert next(v for k,v in matches[0].launch_arguments if k in (name,'allow_private_availability_repair')).perform(ctx)=='true'
