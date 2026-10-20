"""Five independent final-gate RED regressions, preserved for private path."""
import os
assert os.environ.get("ROS_DOMAIN_ID")=="177" and os.environ.get("ROS_LOCALHOST_ONLY")=="1"
from types import SimpleNamespace as NS
import pytest
from test_supervisor_consumer_reconciliation_ros import reconciler
from test_supervisor_private_availability_ros import begin,proof
from so101_station_perception import perception_supervisor as m


def native(node):
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=stamp,receipt=receipt) for _ in range(2)) for stamp,receipt in ((10_170_000_000,100.17),(10_190_000_000,100.19))],selected=((100.19,10_190_000_000),)*2,request=100.15,received=100.16)
    node.repair_bindings={c:(proof(node),100.19) for c in m.CONSUMERS};node.repair_coordinator_receipt=100.19
    status,gid,_=node.producer;node.producer=(status,gid,100.19)


def test_repair_deadline_checks_mono_after_ros_clock_access(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);clock[0]=101.9
    def slow_clock():
        clock[0]=102.01
        return NS(now=lambda:NS(nanoseconds=12_010_000_000))
    monkeypatch.setattr(node,'get_clock',slow_clock)
    with pytest.raises(ValueError):node._repair_deadline()


def test_original_heartbeat_still_current_after_native_final_access(reconciler,monkeypatch):
    node,clock=reconciler;status=begin(node,monkeypatch);clock[0]=100.2;native(node)
    gid='9'*32;node.producer=(status,gid,100.0);node.heartbeat_gid=gid
    monkeypatch.setattr(node,'get_publishers_info_by_topic',lambda topic:[NS(node_namespace='/',node_name='so101_rocm_rgbd',endpoint_gid=bytes.fromhex(gid))])
    monkeypatch.setattr(node,'_producer_now',m.PerceptionSupervisor._producer_now.__get__(node))
    original=node._native_final
    def delayed_native(**kwargs):
        clock[0]=100.31
        return original(**kwargs)
    monkeypatch.setattr(node,'_native_final',delayed_native)
    with pytest.raises((ValueError,m.HealthNotCurrent)):node._repair_final()


def test_original_terminal_coordinator_receipt_checked_after_final_access(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);clock[0]=100.2;native(node);calls=[]
    monkeypatch.setattr(node,'_private_native_proof',lambda:True)
    monkeypatch.setattr(node,'_take',lambda endpoint:(True,dict(schema='so101-perception-recovery-v1',state='completed',ready_now=False),100.0) if endpoint.endswith('/recovery_status') else None)
    original=node._native_final
    def delayed_native(**kwargs):
        clock[0]=100.26
        return original(**kwargs)
    monkeypatch.setattr(node,'_native_final',delayed_native)
    monkeypatch.setattr(node,'_request',lambda endpoint,*a,**kw:calls.append(endpoint) if endpoint.endswith('/start_recovery') else None)
    with pytest.raises((ValueError,m.HealthNotCurrent)):node._dispatch_private_repair()
    assert not calls


def test_request_clamp_cannot_follow_last_native_lease_gate(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);clock[0]=100.2;native(node);calls=[]
    node.policy.reserve_private_repair(node.repair_intent['repair_id'],100.2,1000.2)
    endpoint=m.COORDINATOR+'/start_recovery'
    node.rpc_clients[endpoint]=NS(service_is_ready=lambda:True,remove_pending_request=lambda _:None,call_async=lambda _:calls.append(clock[0]) or NS(add_done_callback=lambda _:None,cancel=lambda:None))
    def slow_clamp():clock[0]=100.5;return 102.
    monkeypatch.setattr(node,'_repair_deadline',slow_clamp)
    with pytest.raises(ValueError):node._request(endpoint,2.,native_authority=lambda:102.)
    assert not calls


def test_private_device_info_cannot_start_after_service_ready_expired_entry(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);clock[0]=100.2;native(node);calls=[]
    graph=dict(node=node.pin['source']['camera_node'],endpoints={k:dict(node=node.pin['source']['camera_node'],gid=node.pin['source'][k+'_publisher_gid']) for k in m.TOPICS})
    monkeypatch.setattr(m,'native_graph',lambda lookup:graph)
    pairs=node.native_witness['pairs'];node.native=NS(select=lambda *a,**kw:True,pairs=pairs)
    node.native_infos={kind:[(object(),graph['endpoints'][kind+'_info']['gid'],100.19,10_190_000_000)] for kind in ('depth','color')}
    node.buffer=NS(lookup_transform=lambda *a:object());monkeypatch.setattr(m,'extrinsic_matrix',lambda value:object())
    monkeypatch.setattr(m,'validate_camera',lambda *a:None)
    def ready():clock[0]=102.01;return True
    node.device_client=NS(service_is_ready=ready,remove_pending_request=lambda _:None,call_async=lambda _:calls.append(clock[0]) or NS(add_done_callback=lambda _:None,cancel=lambda:None))
    with pytest.raises(ValueError):node._private_native_proof()
    assert not calls


def test_private_start_rejects_original_ros_header_expiry_after_readiness_access(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);clock[0]=100.2;native(node);calls=[];ros=[10_640_000_000]
    node.policy.reserve_private_repair(node.repair_intent['repair_id'],100.2,1000.2)
    monkeypatch.setattr(node,'get_clock',lambda:NS(now=lambda:NS(nanoseconds=ros[0])))
    def ready():clock[0]=100.26;ros[0]=10_700_000_000;return True
    endpoint=m.COORDINATOR+'/start_recovery'
    node.rpc_clients[endpoint]=NS(service_is_ready=ready,remove_pending_request=lambda _:None,
        call_async=lambda _:calls.append('start') or NS(add_done_callback=lambda _:None,cancel=lambda:None))
    with pytest.raises(ValueError,match='native pairs expired/future'):
        node._request(endpoint,2.,native_authority=node._repair_final)
    assert not calls and not node.rpc
