"""CPU-only coordinator control-plane faults, never physical services."""
from copy import deepcopy
import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest
pytest.importorskip('rclpy')
from so101_station_perception import perception_recovery as module
from so101_station_perception.recovery_core import Budget,ConsumerProgress
from so101_station_perception.derived_lineage import LineageBuffer,statement_pin
from test_derived_registration import fixture,endpoints
from test_perception_recovery_core import proof


def rpc_fake(monkeypatch,*,ready=True,done=False,late=False):
    clock=[100.];events=[];budget=Budget(100.)
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    future=NS(done=lambda:done,cancel=lambda:events.append('cancel'))
    def result():
        if late:clock[0]+=2.01
        return NS(success=True,message='{}')
    future.result=result
    def call(req):events.append('dispatch');return future
    client=NS(service_is_ready=lambda:ready,call_async=call,remove_pending_request=lambda f:events.append('remove'))
    node=NS(budget=budget,cancel=NS(wait=lambda t:clock.__setitem__(0,clock[0]+t)),rpc_clients={'fixed':client},
        _guard=lambda:budget.check(clock[0]))
    return node,events


@pytest.mark.parametrize('kind',['unavailable','unknown','late'])
def test_rpc_bound_no_blind_retry_or_late_adoption(monkeypatch,kind):
    node,events=rpc_fake(monkeypatch,ready=kind!='unavailable',done=kind=='late',late=kind=='late')
    with pytest.raises(ValueError):module.PerceptionRecovery._rpc(node,'fixed',object())
    assert events.count('dispatch')==(0 if kind=='unavailable' else 1)
    if kind!='unavailable':assert events[-2:]==['remove','cancel']


def test_post_parse_serialization_cannot_renew_rpc_budget(monkeypatch):
    clock=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    node=NS(rpc_deadline=102.,_guard=lambda:None)
    node._rpc_guard=lambda:module.PerceptionRecovery._rpc_guard(node)
    parse=module.parse_lineage
    def slow(raw):value=parse(raw);clock[0]=102.01;return value
    monkeypatch.setattr(module,'parse_lineage',slow)
    with pytest.raises(ValueError):module.PerceptionRecovery._json_response(node,NS(message='{}'))


def test_partial_rebind_does_not_rollback_or_repeat_reset():
    calls=[];pin=statement_pin(fixture());names=module.CONSUMERS
    def rpc(endpoint,req):
        calls.append(endpoint)
        if endpoint.endswith('/derived_source_status'):
            return NS(success=False,message=json.dumps({'schema':'so101-derived-admission-status-v1','qualified':False,'execution_enabled':False,'consumer_epoch':'a'*32}))
        if endpoint.endswith('/set_parameters'):
            return NS(results=[NS(successful=not endpoint.startswith(names[1]))])
        if endpoint.endswith('/get_parameters'):return NS(values=[NS(type=4,string_value='/fixed/new.json')])
        return NS(success=True)
    node=NS(_rpc=rpc,_json_response=lambda r:json.loads(r.message),_phase=lambda p:None,
        _artifact_unchanged=lambda:None,_guard=lambda:None,_rpc_guard=lambda:None,
        artifact_path=Path('/fixed/new.json'),progress={},pin=pin,native=NS(start_ns=9_000_000_000),
        lock=threading.RLock(),result={'configured_consumers':[],'rebound_consumers':[]})
    with pytest.raises(ValueError):module.PerceptionRecovery._rebind(node)
    assert calls.count(names[0]+'/reset_derived_source')==1
    assert names[1]+'/reset_derived_source' not in calls
    assert len(node.progress)==1
    assert node.result['rebound_consumers']==[names[0]]


def test_current_lineage_clear_during_final_json_revokes_ready(monkeypatch):
    v=fixture();v2=deepcopy(v);v2['sequence']=2
    for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):v2['original'][k]+=20_000_000
    for k in ('cloud_stamp_ns','aligned_stamp_ns'):v2['outputs'][k]+=20_000_000
    ledger=LineageBuffer()
    for value in (v,v2):ledger.add(json.dumps(value),'6'*32,now_ns=10_120_000_000,now_mono=100.12)
    progress={}
    for name in module.CONSUMERS:
        p=ConsumerProgress('b'*32,statement_pin(v),start_ns=9_000_000_000)
        p.observe(proof(v),v,now_ns=10_110_000_000,now_mono=100.11)
        p.observe(proof(v2),v2,now_ns=10_120_000_000,now_mono=100.12);progress[name]=p
    monkeypatch.setattr(module.time,'monotonic',lambda:100.12)
    graph=endpoints(v)
    node=NS(lock=threading.RLock(),_active=lambda:False,result={'operation_success':True,'state':'completed'},
        _artifact_unchanged=lambda:None,ledger=ledger,lineage_error=None,progress=progress,
        baseline_pin=statement_pin(v),get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_120_000_000)),
        get_publishers_info_by_topic=graph.__getitem__)
    node.expected_generation=v['producer_generation'];node.result['accepted_reset_id']='f'*32
    node._producer_status=lambda **kw:dict(state='ready',producer_generation=v['producer_generation'],
        reset=dict(accepted_reset_id='f'*32,new_producer_generation=v['producer_generation'],ready=True))
    node._statement=lambda *args,**kw:module.PerceptionRecovery._statement(node,*args,**kw)
    node._proofs_ready=lambda **kw:module.PerceptionRecovery._proofs_ready(node,**kw)
    original=module.bounded_json
    def clear(value):
        raw=original(value)
        if value.get('ready_now'):ledger.entries.clear()
        return raw
    monkeypatch.setattr(module,'bounded_json',clear)
    response=module.PerceptionRecovery._status(node,None,NS())
    value=json.loads(response.message)
    assert value['operation_success'] is True and value['ready_now'] is False


def test_hung_job_retains_node_context_and_reports_bounded_cleanup():
    waits=[];errors=[]
    node=NS(closed=False,cancel=threading.Event(),thread=NS(join=lambda **kw:waits.append(kw)),
        _active=lambda:True,get_logger=lambda:NS(error=errors.append))
    assert module.PerceptionRecovery.close(node) is False
    assert node.closed and node.cancel.is_set() and waits==[{'timeout':2.}]
    assert errors


def test_domain_zero_guard_and_no_unsafe_resources(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','00');started=[]
    monkeypatch.setattr(module.rclpy,'init',lambda **kw:started.append(True))
    with pytest.raises(ValueError):module.main()
    assert not started
    source=Path(module.__file__).read_text();tree=ast.parse(source)
    imports=[ast.unparse(n) for n in ast.walk(tree) if isinstance(n,(ast.Import,ast.ImportFrom))]
    assert not any(word in x for x in imports for word in ('subprocess','pyrealsense2','torch','serial','control_msgs'))
    assert not any(isinstance(n,ast.Attribute) and n.attr in ('kill','system','popen') for n in ast.walk(tree))
    assert 'from sensor_msgs.msg import CameraInfo' in source
    assert 'Image' not in source and 'PointCloud2' not in source
    assert 'use_global_arguments=False' in source and 'executor=executor' in source
    assert 'daemon=False' in source and 'rclpy.spin(' not in source
    launch=Path(module.__file__).parents[1]/'launch/perception_recovery.launch.py'
    assert "DeclareLaunchArgument('enabled', default_value='false')" in launch.read_text()


def test_mutated_reviewed_file_fails_before_any_reset(tmp_path):
    path=tmp_path/'baseline.json';path.write_text('{}')
    node=NS(baseline_path=path,baseline_sha='0'*64,_guard=lambda:None)
    with pytest.raises((ValueError,KeyError)):
        module.PerceptionRecovery._check_baseline(node)


def test_current_producer_fault_revokes_completed_source_proof():
    value=fixture()
    node=NS(expected_generation=value['producer_generation'],result={'accepted_reset_id':'f'*32},
        _producer_status=lambda **kw:dict(state='unavailable',producer_generation=value['producer_generation'],
            reset={'accepted_reset_id':'f'*32,'new_producer_generation':value['producer_generation'],'ready':False}),
        progress={})
    with pytest.raises(ValueError,match='current producer'):
        module.PerceptionRecovery._proofs_ready(node,job=False)


def test_unhealthy_native_preflight_fails_without_producer_reset(monkeypatch):
    clock=[100.];budget=Budget(100.);rpc=[]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    graph={topic:[NS(node_namespace='/',node_name='camera',endpoint_gid=bytes([i+1])*16)]
           for i,topic in enumerate(module.TOPICS.values())}
    node=NS(lock=threading.RLock(),_guard=lambda:budget.check(clock[0]),budget=budget,
        _check_baseline=lambda:None,native=NS(graph=None,select=lambda *a,**kw:False),
        native_error='future native clock',get_publishers_info_by_topic=graph.__getitem__,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),
        _pause=lambda *a:clock.__setitem__(0,clock[0]+.05),
        _rpc=lambda *a:rpc.append(a),result={})
    module.PerceptionRecovery._run(node)
    assert node.result['state']=='failed' and not node.result['operation_success']
    assert 'camera-only supervisor' in node.result['reason'] and not rpc
