"""Native ROS callbacks with fakes, without initialization/DDS/hardware."""
import hashlib
import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2/src/so101_station_perception'))
private=types.ModuleType('so101_station_perception_tag_tracking')
private.__path__=[str(ROOT/'workshop/digital_twin')]
sys.modules.setdefault(private.__name__,private)
from so101_station_perception.registration_monitor import RegistrationMonitor, bounded_bytes, read_owner
from so101_station_perception_tag_tracking.registration_monitor import StationaryHistory
from std_srvs.srv import Trigger, SetBool


def test_bounded_owner_read_records_real_clock_bracket(tmp_path):
    p=tmp_path/'owner.json';p.write_text('{"schema":1}')
    result=read_owner(p)
    assert result['mono_before']<=result['mono_after']
    assert result['utc_before']<=result['utc_after']
    assert result['state']=={'schema':1}
    p.write_bytes(b'x'*131073)
    with pytest.raises(ValueError):bounded_bytes(p)


def test_busy_request_has_no_queue_and_disable_cancels_collection():
    node=NS(job={'started':0.},enabled=True,source_invalid=False)
    ok,why=RegistrationMonitor._start(node)
    assert ok is False and 'busy' in why
    response=SetBool.Response()
    RegistrationMonitor._enable(node,NS(data=False),response)
    assert response.success and node.job is None and not node.enabled


def test_changed_description_invalidates_history_and_pending_job():
    history=StationaryHistory();history.samples.append((1.,{}))
    node=NS(description='old',pending_description=None,cache='old',job={'started':0.},joints=history,
            _publish=lambda *a,**kw:None)
    RegistrationMonitor._description(node,NS(data='new'))
    assert node.cache is None and node.job is None and not history.samples
    assert node.pending_description=='new'


def test_explicit_trigger_reports_unavailable_without_starting_job():
    node=NS(_start=lambda:(False,'model/source cohort unavailable'))
    response=RegistrationMonitor._trigger(node,None,Trigger.Response())
    assert not response.success


def test_collection_has_exact_twelve_bounded_reads_then_one_run(monkeypatch):
    import so101_station_perception.registration_monitor as module
    calls=[]
    monkeypatch.setattr(module,'read_owner',lambda p:calls.append(p) or {'state':{}})
    monkeypatch.setattr(module.time,'monotonic',lambda:1.)
    job={'started':.5,'reads':[]}
    node=NS(source_invalid=False,pending_description=None,cohort={},job=job,owner_path='owner',_run=lambda job:calls.append('fit'),
            _publish=lambda *a,**kw:None)
    node._read_owner=lambda:module.read_owner('owner')
    for _ in range(12):RegistrationMonitor._tick(node)
    assert calls==['owner']*12+['fit'] and node.job is None


def test_unpaired_cloud_never_runs_fit():
    from collections import deque
    node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),clouds=deque(),
            _cohort=RegistrationMonitor._cohort)
    # Calling the selector alone only checks UTC mismatch; no fitter is reachable.
    with pytest.raises(ValueError):RegistrationMonitor._cohort(node,{'reads':[]})


@pytest.mark.parametrize('info',[{'publisher_gid':b'\x01'*16},NS(publisher_gid=b'\x01'*16),{}])
def test_cloud_publisher_identity_handles_native_dict_and_missing_gid(info):
    from collections import deque
    node=NS(job={},clouds=deque(maxlen=8),source_invalid=False,cohort=None)
    RegistrationMonitor._cloud(node,'cloud',info)
    assert node.clouds[0][1]==('01'*16 if info!={} else '')


def test_non_object_board_json_never_enters_cohort_buffer():
    from collections import deque
    node=NS(boards=deque(maxlen=8))
    for payload in ('[]','null','42','invalid'):
        RegistrationMonitor._board(node,NS(data=payload))
    assert not node.boards


def test_changed_base_seed_hash_invalidates_cache_and_stationary_history(tmp_path):
    p=tmp_path/'source';p.write_text('{}')
    sha=hashlib.sha256(p.read_bytes()).hexdigest()
    node=NS(cohort={'station_config':str(p),'station_config_sha256':sha,
                    'calibration_path':str(p),'calibration_sha256':sha,
                    'base_registration_path':str(p),'base_registration_sha256':'changed'},
            cache='cached',joints=StationaryHistory(),_invalidate=lambda reason:invalidate())
    def invalidate():
        node.cache=None;node.joints.samples.clear()
    node.joints.samples.append((1.,{}))
    with pytest.raises(ValueError,match='base_registration_path'):
        RegistrationMonitor._source_files(node)
    assert node.cache is None and not node.joints.samples


def test_complete_pure_job_publishes_original_stamp_pins_and_ground_metric(monkeypatch):
    import numpy as np
    import so101_station_perception.registration_monitor as module
    captured=[];lookups=[]
    pose=np.eye(4);raw={j:2000 for j in StationaryHistory.JOINTS}
    cloud=NS(header=NS(stamp=NS(sec=10,nanosec=0),frame_id='depth_optical'))
    info=NS(k=[200.,0.,159.5,0.,200.,119.5,0.,0.,1.],width=320,height=240)
    binding={'raw':raw};board={'stamp':10.,'charuco_corners':10,'reprojection_rms_px':.2}
    cache=NS(links=('body',),all_links={'body'},model_sha256='model',base_floor_z=0.,provenance={},
             at=lambda poses:(np.ones((100,3)),np.ones((100,3)),np.repeat('body',100)))
    node=NS(cache=cache,mount=None,marker=None,owner_path='unused',base='base',world='world',enabled=False,
            cohort={'world_from_base':pose.tolist(),'cloud_publisher_gid':'gid','camera_serial':'serial',
                    'station_config_sha256':'config','base_registration_sha256':'seed'},
            cohort_sha='cohort',source_sha256={'node':'node','pure':'pure'},mount_sha=None,
            reference_camera=None,fit_budget=.2,joints=StationaryHistory(),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
            _cohort=lambda job:(cloud,info,{j:0. for j in StationaryHistory.JOINTS},10.,board,binding),
            _source_files=lambda:None,
            _tf=lambda target,source,stamp:lookups.append((target,source,stamp)) or pose,
            status=NS(publish=lambda message:None),result=NS(publish=captured.append))
    node._publish=lambda state,**kw:RegistrationMonitor._publish(node,state,**kw)
    monkeypatch.setattr(module,'visible_samples',lambda p,n,l,*args:(p,n,l))
    monkeypatch.setattr(module,'sampled_cloud',lambda *args:(np.ones((100,3)),None,'absent'))
    monkeypatch.setattr(module,'fit_registration',lambda *args,**kw:{'candidate_delta':pose.tolist(),'state':'proposal_unqualified'})
    monkeypatch.setattr(module,'read_owner',lambda path:{'state':{'timestamp_utc':'1970-01-01T00:00:10.100000+00:00'},'utc_after':10.1})
    monkeypatch.setattr(module,'owner_bracket',lambda *args,**kw:{'raw':raw})
    monkeypatch.setattr(module.time,'time',lambda:10.1)
    node._read_owner=lambda:module.read_owner('unused')
    RegistrationMonitor._run(node,{'reads':[{}]*12})
    payload=json.loads(captured[0].data)
    assert payload['capture_stamp_ns']==10_000_000_000
    assert payload['qualified'] is False and payload['applied'] is False
    assert payload['base_registration_sha256']=='seed'
    assert payload['ground_constraint']['candidate_ground_violation'] is False
    assert payload['attached_marker']['state']=='unavailable_missing_mount_or_observation'
    assert all(stamp==10_000_000_000 for _,_,stamp in lookups)


def test_new_camera_publisher_invalidates_old_cloud_generation_until_reset():
    from collections import deque
    states=[]
    node=NS(job={},source_invalid=False,cache='cache',reference_camera='old',marker='old',
            cohort={'cloud_publisher_gid':'01'*16},clouds=deque(),infos=deque(['old']),boards=deque(['old']),
            joints=StationaryHistory(),_publish=lambda state,**kw:states.append(state))
    node._invalidate=lambda reason:RegistrationMonitor._invalidate(node,reason)
    RegistrationMonitor._cloud(node,'old-valid',{'publisher_gid':b'\x01'*16})
    assert len(node.clouds)==1
    RegistrationMonitor._cloud(node,'new-source',{'publisher_gid':b'\x02'*16})
    assert node.source_invalid and node.job is None and node.cache is None
    assert not node.clouds and not node.infos and not node.boards and node.marker is None
    RegistrationMonitor._cloud(node,'old-source-again',{'publisher_gid':b'\x01'*16})
    assert not node.clouds and states==['unavailable_source_changed']
    assert RegistrationMonitor._start(node)[0] is False


def test_owner_restart_invalidates_job_even_when_code_identity_and_raw_unchanged(monkeypatch):
    import so101_station_perception.registration_monitor as module
    invalid=[]
    node=NS(owner_path='owner',cohort={'owner_session_id':'old'},_invalidate=invalid.append)
    monkeypatch.setattr(module,'read_owner',lambda path:{'state':{'owner_session_id':'new','raw':{'joint':2000}}})
    with pytest.raises(ValueError,match='owner generation changed'):
        RegistrationMonitor._read_owner(node)
    assert len(invalid)==1
