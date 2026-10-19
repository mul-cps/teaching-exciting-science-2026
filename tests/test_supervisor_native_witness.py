"""Independent-shape scalar race reproduction; no ROS initialization/publishers."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177'
from types import SimpleNamespace as NS
import pytest
from test_derived_registration import fixture
from so101_station_perception import perception_supervisor as m
from so101_station_perception.derived_lineage import statement_pin,native_snapshot
from so101_station_perception.supervisor_policy import SupervisorPolicy


def test_selected_info_cannot_expire_during_last_pre_dispatch_access(monkeypatch):
    value=fixture();pin=statement_pin(value);clock=[100.2];results=[0];starts=[]
    graph=native_snapshot(value)
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0],time=lambda:1000.+clock[0]))
    monkeypatch.setattr(m,'native_graph',lambda _:graph)
    monkeypatch.setattr(m,'validate_camera',lambda *a:None)
    monkeypatch.setattr(m,'extrinsic_matrix',lambda _:None)
    def result():
        results[0]+=1
        if results[0]==2:clock[0]+=.11
        return NS(serial_number=pin['source']['serial_number'])
    policy=SupervisorPolicy();policy.enabled=True;policy.state='verifying_native';policy.fault={'fault_id':'a'*32}
    node=NS(pin=pin,baseline={'camera_evidence':{'lineage':{'statement':value}}},native_started=100.,
        get_publishers_info_by_topic=lambda _:None,_graph=lambda:None,
        native=NS(select=lambda *a,**k:True,pairs=[tuple(dict(stamp=value['original'][kind+'_info_stamp_ns']-offset,receipt=100.) for kind in ('depth','color')) for offset in (1_000_000,0)]),
        native_infos={kind:[(None,None,100.,value['original'][kind+'_info_stamp_ns'])] for kind in ('depth','color')},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)),buffer=NS(lookup_transform=lambda *a:None),
        identity_rpc={'request':100.,'received':100.1,'graph':graph,'future':NS(done=lambda:True,result=result)},
        serial=pin['source']['serial_number'],policy=policy,_outage_still_current=lambda:None,
        _take=lambda _: (True,{'state':'idle'},clock[0]),_baseline_unchanged=lambda:None,_persist=lambda:None,
        _request=lambda *a:starts.append(a))
    node._native_proof=lambda:m.PerceptionSupervisor._native_proof(node)
    node._history_final=lambda:102.
    node._native_final=lambda:m.PerceptionSupervisor._native_final(node)
    try:m.PerceptionSupervisor._dispatch(node)
    except ValueError:pass
    assert starts==[], 'expired original selectedInfo proof dispatched coordinator'


def test_service_readiness_stall_cannot_dispatch_expired_original_witness(monkeypatch):
    clock=[100.2];calls=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    witness=dict(started=100.,pairs=[tuple(dict(stamp=10_000_000_000,receipt=100.) for _ in range(2)) for _ in range(2)],
        selected=((100.,10_000_000_000),)*2,request=100.,received=100.1)
    def readiness():clock[0]+=.11;return True
    client=NS(service_is_ready=readiness,call_async=lambda _:calls.append('start'))
    node=NS(rpc={},rpc_clients={m.COORDINATOR+'/start_recovery':client},native_witness=witness,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)))
    node._history_final=lambda:102.
    node._native_final=lambda:m.PerceptionSupervisor._native_final(node)
    with pytest.raises(ValueError):m.PerceptionSupervisor._request(node,m.COORDINATOR+'/start_recovery',2.)
    assert not calls and not node.rpc


@pytest.mark.parametrize('field,value',[('started',97.),('request',99.),('received',101.)])
def test_final_original_deadline_and_identity_leases_checked(field,value,monkeypatch):
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:100.2))
    witness=dict(started=100.,pairs=[tuple(dict(stamp=10_000_000_000,receipt=100.) for _ in range(2)) for _ in range(2)],
        selected=((100.,10_000_000_000),)*2,request=100.,received=100.1)
    witness[field]=value
    node=NS(native_witness=witness,get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)))
    node._history_final=lambda:102.
    with pytest.raises(ValueError):m.PerceptionSupervisor._native_final(node)
