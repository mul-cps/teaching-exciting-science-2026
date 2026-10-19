"""Native message/API fakes, without ROS startup, SDK, GPU or hardware."""
from collections import deque
import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ros2/src/so101_station_perception'),str(ROOT/'tests')]
private=types.ModuleType('so101_station_perception_tag_tracking');private.__path__=[str(ROOT/'workshop/digital_twin')]
sys.modules.setdefault(private.__name__,private)
from test_derived_registration import fixture,endpoints
from so101_station_perception.derived_lineage import LineageBuffer
from so101_station_perception.registration_source import SourceCapture
from so101_station_perception.registration_monitor import RegistrationMonitor


def test_device_info_uses_actual_native_camera_with_all_four_request_gids(monkeypatch):
    import so101_station_perception.registration_source as module
    monkeypatch.setattr(module.time,'monotonic',lambda:100.12)
    statement=fixture();buffer=LineageBuffer();buffer.add(json.dumps(statement),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    graph=endpoints(statement);services=[]
    node=NS(derived=buffer,clouds=deque([('cloud',bytes.fromhex('5'*32))]),source_invalid_reason=None,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_120_000_000)),get_publishers_info_by_topic=graph.__getitem__,
        device_request_source=None,device_client=None,device_future=None)
    node._derived_generation=lambda gid=None:SourceCapture._derived_generation(node,gid)
    def client(service):
        services.append(service);node.device_client=NS(service_is_ready=lambda:True,call_async=lambda request:NS(done=lambda:False))
    node._open_device_client=client
    SourceCapture._device_request(node)
    assert services==['/actual_camera/device_info']
    assert node.device_request_source==statement['identity']['device_info_request_source']
    assert node.device_request_gid==bytes.fromhex(statement['source']['depth_publisher_gid'])
    # Same node name, replaced CameraInfo publisher during async DeviceInfo.
    old=graph['/camera/color/camera_info'][0]
    graph['/camera/color/camera_info']=[NS(node_name=old.node_name,node_namespace=old.node_namespace,endpoint_gid=b'\xff'*16)]
    with pytest.raises(ValueError,match='generation changed'):SourceCapture._device_request(node)
    assert node.source_invalid_reason and not node.clouds


def test_monitor_requires_independent_explicit_opt_in_for_derived_artifact(tmp_path):
    import numpy as np
    from so101_station_perception_tag_tracking.registration_monitor import StationaryHistory
    value=fixture();pin=LineageBuffer();pin.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    cohort=dict(schema='so101-registration-source-v1',model_sha256='a'*64,owner_session_id='actual',arm_id='actual',
        calibration_sha256='b'*64,owner_source_sha256={'actual':'c'*64},camera_serial='actual_serial',cloud_publisher_gid='5'*32,
        station_config='/actual/config',calibration_path='/actual/calibration',station_config_sha256='d'*64,
        base_registration_path='/actual/base',base_registration_sha256='e'*64,world_from_base=np.eye(4).tolist(),
        derived_source=dict(schema=value['schema'],qualified=False,local_provenance_not_cryptographic=True,pin=pin.pin))
    path=tmp_path/'source.json';path.write_text(json.dumps(cohort));statuses=[]
    node=NS(allow_derived=False,cohort_path=str(path),marker_path='',description=None,
        joints=StationaryHistory(),clouds=deque(),infos=deque(),boards=deque(),derived_infos={'depth':deque(),'color':deque()},
        _publish=lambda state,**kw:statuses.append((state,kw)),_source_files=lambda:None)
    RegistrationMonitor._load_cohort(node)
    assert node.cohort is None and 'explicit' in statuses[-1][1]['reason']
    node.allow_derived=True;RegistrationMonitor._load_cohort(node)
    assert node.cohort==cohort and node.derived.pin==pin.pin
    # Direct source artifact remains accepted without derived opt-in or subscriptions.
    del cohort['derived_source'];path.write_text(json.dumps(cohort));node.allow_derived=False
    RegistrationMonitor._load_cohort(node);assert node.derived is None and node.cohort==cohort


def test_newest_cloud_missing_exact_status_does_not_fall_back_to_older(monkeypatch):
    import so101_station_perception.registration_monitor as module
    monkeypatch.setattr(module.time,'time',lambda:10.12)
    def cloud(ns):return NS(header=NS(stamp=NS(sec=10,nanosec=ns),frame_id='depth'))
    old,new=cloud(0),cloud(1_000_000);calls=[]
    node=NS(derived=object(),get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_120_000_000)),
        clouds=deque([(old,'5'*32),(new,'5'*32)]),cohort={'cloud_publisher_gid':'5'*32},
        joints=NS(select=lambda stamp,now:(stamp,{})),boards=deque([dict(state='tracking',observation_valid=True,plane_source='aligned_depth',stamp=10.001)]))
    def missing(message,gid):calls.append(message);raise ValueError('no exact per-cohort lineage')
    node._derived_cohort=missing
    with pytest.raises(ValueError,match='no exact'):RegistrationMonitor._cohort(node,{'reads':[]})
    assert calls==[new]


def test_complete_derived_job_publishes_selected_statement_original_receipt_and_gid(monkeypatch):
    import numpy as np
    import so101_station_perception.registration_monitor as module
    from so101_station_perception_tag_tracking.registration_monitor import StationaryHistory
    captured=[];pose=np.eye(4);raw={j:2000 for j in StationaryHistory.JOINTS};statement=fixture()
    cloud=NS(header=NS(stamp=NS(sec=10,nanosec=0),frame_id='depth'))
    info=NS(k=[200.,0.,159.5,0.,200.,119.5,0.,0.,1.],width=320,height=240)
    board={'stamp':10.,'charuco_corners':10,'reprojection_rms_px':.2};binding={'raw':raw}
    cache=NS(links=('body',),all_links={'body'},model_sha256='model',base_floor_z=0.,provenance={},
        at=lambda poses:(np.ones((100,3)),np.ones((100,3)),np.repeat('body',100)))
    node=NS(cache=cache,mount=None,marker=None,base='base',world='world',enabled=False,
        cohort={'world_from_base':pose.tolist(),'cloud_publisher_gid':'5'*32,'camera_serial':'actual_serial',
                'station_config_sha256':'config','base_registration_sha256':'seed'},cohort_sha='cohort',source_sha256={},mount_sha=None,
        reference_camera=None,fit_budget=.2,derived=NS(pin={'pinned':'unchanged'}),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_120_000_000)),
        _cohort=lambda job:(cloud,info,{j:0. for j in StationaryHistory.JOINTS},10.,board,binding),
        _derived_cohort=lambda *args:(statement,info,100.111,'6'*32),_source_files=lambda:None,
        _tf=lambda *args:pose,result=NS(publish=captured.append),status=NS(publish=lambda message:None))
    node._publish=lambda state,**kw:RegistrationMonitor._publish(node,state,**kw)
    monkeypatch.setattr(module,'visible_samples',lambda p,n,l,*args:(p,n,l))
    monkeypatch.setattr(module,'sampled_cloud',lambda *args:(np.ones((100,3)),None,'absent'))
    monkeypatch.setattr(module,'fit_registration',lambda *args,**kw:{'candidate_delta':pose.tolist(),'state':'proposal_unqualified'})
    monkeypatch.setattr(module,'owner_bracket',lambda *args,**kw:{'raw':raw})
    node._read_owner=lambda:{'state':{'timestamp_utc':'1970-01-01T00:00:10.120000+00:00'},'utc_after':10.12}
    monkeypatch.setattr(module.time,'time',lambda:10.12)
    RegistrationMonitor._run(node,{'reads':[{}]*12})
    lineage=json.loads(captured[0].data)['derived_source']['lineage']
    assert lineage['statement']==statement
    assert lineage['statement']['original']['color_stamp_ns']==10_001_000_000
    assert lineage['received_monotonic']==100.111 and lineage['publisher_gid']=='6'*32


def test_all_diagnostic_entrypoints_reject_normalized_zero_before_ros_start(monkeypatch,tmp_path):
    import importlib.util
    import so101_station_perception.registration_monitor as monitor
    import so101_station_perception.registration_source as source
    path=ROOT/'ros2/src/so101_station_perception/launch/periodic_registration.launch.py'
    spec=importlib.util.spec_from_file_location('periodic_derived_launch',path);launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    calls=[];monkeypatch.setattr(monitor.rclpy,'init',lambda *a,**kw:calls.append('init'))
    for value in ('00','+0','233'):
        monkeypatch.setenv('ROS_DOMAIN_ID',value)
        for invoke in (lambda:monitor.main([]),lambda:source.main(['--output',str(tmp_path/'unused.json')]),launch.generate_launch_description):
            with pytest.raises(ValueError):invoke()
    assert not calls and not list(tmp_path.iterdir())
