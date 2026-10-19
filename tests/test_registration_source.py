"""Runtime source pins are derived, not handwritten/qualified by capture."""
import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin.registration_monitor import make_source_cohort,StationaryHistory


def fixture():
    cal=b'{"actual":"calibration"}';sha=hashlib.sha256(cal).hexdigest()
    config=json.dumps({'arm':{'id':'rig','calibration':{'path':'/actual/cal.json','sha256':sha}}}).encode()
    matrix=np.eye(4);matrix[:3,3]=[.03,.2,.0024]
    base=json.dumps({'parent_frame':'charuco_board','child_frame':'base_link','transform_direction':'world_from_base','world_from_base':matrix.tolist(),'qualified':False}).encode()
    owner={'arm_id':'rig','owner_session_id':'actual-generation','calibration_sha256':sha,
           'owner_source_sha256':{'arm':'a'*64,'arm_session':'b'*64},'monotonic':10.,
           'raw':dict.fromkeys(StationaryHistory.JOINTS,2000),'torque':dict.fromkeys(StationaryHistory.JOINTS,1),'violations':[],
           'arm_bounds':{'classification':'normal','admitted_min':dict.fromkeys(StationaryHistory.JOINTS,1000),'admitted_max':dict.fromkeys(StationaryHistory.JOINTS,3000)}}
    kwargs=dict(station_config_path='/actual/station.json',station_config_bytes=config,
                calibration_path='/actual/cal.json',calibration_bytes=cal,
                base_registration_path='/actual/base.json',base_registration_bytes=base,world_from_base=matrix)
    return owner,kwargs


def build(owner=None,**changes):
    original,kwargs=fixture();kwargs.update(changes)
    return make_source_cohort('<robot name="current"><link name="base_link"/></robot>',owner or original,
                              {'serial_number':'actualserial','firmware_version':'actualfirmware'},'01'*16,**kwargs)


def test_pins_current_model_full_source_hashes_and_actual_nonidentity_base():
    result=build();assert result['model_sha256']==hashlib.sha256(b'<robot name="current"><link name="base_link"/></robot>').hexdigest()
    assert result['world_from_base'][0][3]==.03 and result['world_from_base'][2][3]==.0024
    assert result['camera_serial']=='actualserial' and len(result['owner_source_sha256'])==2
    assert result['fit_eligibility_at_capture']=={'eligible':True,'reason':None}
    assert result['qualified'] is False


def test_source_pins_can_record_failed_hold_but_never_admit_it_to_fit():
    owner,_=fixture();owner['failed_endpoint_hold']={'target_reached':False}
    result=build(owner);assert result['fit_eligibility_at_capture']['eligible'] is False
    assert 'failed endpoint hold' in result['fit_eligibility_at_capture']['reason']


def test_model_hash_is_not_a_specific_model_or_camera_literal():
    owner,kwargs=fixture()
    a=make_source_cohort('<robot name="new"><link name="base_link"/></robot>',owner,
                         {'serial_number':'changedserial','firmware_version':'fw'},'02'*16,**kwargs)
    assert a['model_sha256']!=build()['model_sha256'] and a['camera_serial']=='changedserial'


def test_changed_base_calibration_or_config_is_rejected():
    with pytest.raises(ValueError,match='differs'):build(world_from_base=np.eye(4))
    with pytest.raises(ValueError,match='identity'):build(calibration_bytes=b'changed')
    with pytest.raises(ValueError,match='absolute'):build(station_config_path='relative')


def test_capture_helper_has_no_actuator_calls_or_background_timer_and_installs_cli():
    root=Path(__file__).resolve().parents[1]
    source=(root/'ros2/src/so101_station_perception/so101_station_perception/registration_source.py').read_text()
    tree=ast.parse(source);attributes={node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
    assert not {'create_timer','create_publisher','create_action_client','send_goal_async','set_parameters','sendTransform'}&attributes
    clients=[node for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='create_client']
    assert len(clients)==1 and isinstance(clients[0].args[0],ast.Name) and clients[0].args[0].id=='DeviceInfo'
    assert 'Time(nanoseconds=ns)' in source and 'world_from_base=world_base' in source
    subscriptions=[node for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='create_subscription']
    assert not any(isinstance(node.args[2],ast.Attribute) and node.args[2].attr=='append' for node in subscriptions)
    assert 'registration_capture_sources' in (root/'ros2/src/so101_station_perception/CMakeLists.txt').read_text()


def test_native_identity_bracket_can_pin_failed_state_without_fit_admission():
    pytest.importorskip('rclpy')
    import sys,types
    from datetime import datetime,timezone
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'ros2/src/so101_station_perception'))
    private=types.ModuleType('so101_station_perception_tag_tracking');private.__path__=[str(root/'workshop/digital_twin')]
    sys.modules.setdefault(private.__name__,private)
    from so101_station_perception.registration_source import select_identity_bracket,same_gid
    from workshop.digital_twin.registration_monitor import owner_bracket
    owner,_=fixture();owner['failed_endpoint_hold']={'target_reached':False};joints=dict.fromkeys(StationaryHistory.JOINTS,0.)
    def item(t):
        return {'state':owner|{'schema':1,'event':'sample','timestamp_utc':datetime.fromtimestamp(t,timezone.utc).isoformat(),
                              'monotonic':t-100,'pose':joints},'utc_before':t+.01,'utc_after':t+.011,'mono_before':t-99.99,'mono_after':t-99.989}
    reads=[item(101.),item(101.05)];cohort={key:owner[key] for key in ('owner_session_id','arm_id','calibration_sha256','owner_source_sha256')}
    a,b=select_identity_bracket(reads,101.02,joints,cohort,now=101.1)
    assert a is reads[0] and b is reads[1]
    with pytest.raises(ValueError,match='failed endpoint hold'):owner_bracket(reads,101.02,joints,cohort,now=101.1)
    assert same_gid(b'\1'*16,b'\1'*16+b'\0'*8)
    assert not same_gid(b'\1'*16,b'\2'*16)


def test_native_device_info_is_bound_to_exact_publisher_generation():
    pytest.importorskip('rclpy')
    import sys,types
    from collections import deque
    root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'ros2/src/so101_station_perception'))
    private=types.ModuleType('so101_station_perception_tag_tracking');private.__path__=[str(root/'workshop/digital_twin')]
    sys.modules.setdefault(private.__name__,private)
    from so101_station_perception.registration_source import SourceCapture
    old=b'\1'*16;new=b'\2'*16
    endpoint=lambda gid:types.SimpleNamespace(endpoint_gid=gid,node_name='camera',node_namespace='/camera')
    fake=types.SimpleNamespace(device_request_gid=old,source_invalid_reason=None,
        options=types.SimpleNamespace(cloud_topic='/cloud'),endpoint=endpoint(old),clouds=deque([(object(),old)]),
        get_publishers_info_by_topic=lambda topic:[endpoint(old+b'\0'*8)])
    SourceCapture._validate_generation(fake,old)
    fake.get_publishers_info_by_topic=lambda topic:[endpoint(new)]
    with pytest.raises(ValueError,match='generation changed'):SourceCapture._validate_generation(fake,new)
    assert not fake.clouds
    fake.get_publishers_info_by_topic=lambda topic:[endpoint(old)]
    with pytest.raises(ValueError,match='generation changed'):SourceCapture._validate_generation(fake,old)
    fake.source_invalid_reason=None;fake.clouds.append((object(),old))
    SourceCapture._cloud(fake,object(),{'publisher_gid':new})
    assert not fake.clouds and fake.source_invalid_reason
    SourceCapture._cloud(fake,object(),{'publisher_gid':old})
    assert not fake.clouds
    fake.source_invalid_reason=None;fake.clouds.append((object(),old))
    SourceCapture._cloud(fake,object(),{'publisher_gid':b''})
    assert not fake.clouds and fake.source_invalid_reason
