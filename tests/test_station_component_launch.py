"""Inspect actual launch descriptions only; never start ROS nodes or devices."""
import importlib.util
from pathlib import Path
import pytest
pytest.importorskip('launch.launch_context')
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
ROOT=Path(__file__).resolve().parents[1]


def load(relative):
    spec=importlib.util.spec_from_file_location('station_role_launch',ROOT/relative)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def context(description, **overrides):
    ctx=LaunchContext()
    for action in description.entities:
        if isinstance(action,DeclareLaunchArgument):ctx.launch_configurations[action.name]=perform_substitutions(ctx,action.default_value)
    ctx.launch_configurations.update(overrides);return ctx


def arguments(action,ctx):
    return {key:perform_substitutions(ctx,normalize_to_list_of_substitutions(value)) for key,value in action.launch_arguments}


def test_default_integrated_actions_and_explicit_role_enum():
    mod=load('ros2/src/so101_station_bringup/launch/station_live.launch.py');desc=mod.generate_launch_description();ctx=context(desc)
    assert ctx.launch_configurations['component']=='integrated'
    includes=[a for a in desc.entities if isinstance(a,IncludeLaunchDescription)]
    robot=next(a for a in includes if 'station_lock' in dict(a.launch_arguments))
    assert robot.condition.evaluate(ctx)
    for role in ('owner','movegroup','rviz'):
        ctx.launch_configurations['component']=role
        assert robot.condition.evaluate(ctx)
        args=arguments(robot,ctx)
        assert args['use_trajectory_server']==str(role=='owner').lower()
        assert args['use_robot_state_publisher']==str(role=='owner').lower()
        assert args['use_move_group']==str(role=='movegroup').lower()
        assert args['rviz']==str(role=='rviz').lower()
        assert args['leader']=='false'
    ctx.launch_configurations['component']='__import__("os")'
    with pytest.raises(ValueError):robot.condition.evaluate(ctx)


def test_camera_tags_and_other_units_do_not_create_second_owner_or_camera():
    mod=load('ros2/src/so101_station_bringup/launch/station_live.launch.py');desc=mod.generate_launch_description()
    for role in ('camera','tags','objects','scene','leader','apis'):
        ctx=context(desc,component=role)
        enabled=[a for a in desc.entities if isinstance(a,IncludeLaunchDescription) and (a.condition is None or a.condition.evaluate(ctx))]
        assert not any('station_lock' in dict(a.launch_arguments) for a in enabled)
        cameras=[a for a in enabled if 'camera_repair_prefix' in dict(a.launch_arguments)]
        assert len(cameras)==int(role in ('camera','tags'))
        if cameras:
            args=arguments(cameras[0],ctx)
            assert args['use_camera']==str(role=='camera').lower()
            assert args['publish_tag_observations']==str(role=='tags').lower()
            assert args['track_charuco_world']==str(role=='tags').lower()
            assert args['use_calibration_tf']==str(role=='tags').lower()


def test_exact_reset_unit_slots_reach_api_and_bridge_without_owner_slot():
    mod=load('ros2/src/so101_station_bringup/launch/station_live.launch.py');desc=mod.generate_launch_description()
    ctx=context(desc,**{'operator_'+key+'_unit':'so101-fixture-'+key+'.service' for key in ('movegroup','objects','scene','leader_readonly')})
    includes=[a for a in desc.entities if isinstance(a,IncludeLaunchDescription)]
    api=next(a for a in includes if 'enable_service_restarts' in dict(a.launch_arguments))
    bridge=next(a for a in includes if 'station_lock' in dict(a.launch_arguments))
    for key in ('movegroup','objects','scene','leader_readonly'):
        assert arguments(api,ctx)[key+'_unit']=='so101-fixture-'+key+'.service'
        assert arguments(bridge,ctx)['operator_'+key+'_unit']=='so101-fixture-'+key+'.service'


def test_api_component_respects_existing_tunnel_flag():
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    mod=load('ros2/src/so101_station_bringup/launch/station_live.launch.py');desc=mod.generate_launch_description()
    ctx=context(desc,component='apis',use_ev3_tunnel='false')
    ssh=next(a for a in desc.entities if isinstance(a,ExecuteProcess) and not isinstance(a,Node))
    assert not ssh.condition.evaluate(ctx)
    ctx.launch_configurations['use_ev3_tunnel']='true'
    assert ssh.condition.evaluate(ctx)
