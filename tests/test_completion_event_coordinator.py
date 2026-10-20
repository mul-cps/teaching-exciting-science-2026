"""Actual coordinator methods: opt-in evidence cannot change legacy authority."""
from copy import deepcopy
import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from so101_station_perception import perception_recovery as m
from test_completion_event_protocol import event,CONSUMERS


def test_default_off_status_exact_legacy_keys_and_no_capture(monkeypatch):
    legacy=dict(schema='so101-perception-recovery-v1',state='idle',operation_success=False,ready_now=False,qualified=False,execution_enabled=False)
    n=NS(lock=threading.RLock(),result=deepcopy(legacy),_active=lambda:False)
    result=m.PerceptionRecovery._status(n,None,NS())
    assert result.success is True and json.loads(result.message)==legacy


def test_opted_in_status_exposes_history_but_rechecks_current(monkeypatch):
    e=event();clock=[100.41]
    n=NS(lock=threading.RLock(),result=dict(state='completed',operation_success=True,ready_now=True,
        qualified=False,execution_enabled=False),expose_completion_evidence=True,_completion_event=e,
        _active=lambda:False,_artifact_unchanged=lambda:None,producer=({},None,100.3),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_410_000_000)),
        _proofs_ready=lambda **kw:True)
    n._evidence_status=lambda result:m.PerceptionRecovery._evidence_status(n,result)
    n._readiness_snapshot=lambda ready,reason:m.PerceptionRecovery._readiness_snapshot(n,ready,reason)
    n.progress={c:NS(current=deepcopy(v['witnesses'][-1]['proof'])) for c,v in e['consumers'].items()}
    n.ledger=NS(entries=[])
    # The snapshot's exact matched lineages are retained with original receipts.
    from test_derived_registration import fixture
    v=fixture();v['identity']['expires_monotonic']=101.
    v['health']['source_expiry_monotonic']=100.55
    records=[]
    for i,c in enumerate(CONSUMERS):
        value=deepcopy(v);p=n.progress[c].current
        value['sequence']=p['sequence']
        for k in ('depth_stamp_ns','color_stamp_ns'):value['original'][k]=p[k]
        records.append((value,None,100.3))
    n.ledger.entries=records
    n._statement=lambda *a,**kw:v
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    response=m.PerceptionRecovery._status(n,None,NS())
    value=json.loads(response.message)
    assert value['completion_event']==e and value['current_readiness']['ready'] is True
    n._proofs_ready=lambda **kw:(_ for _ in ()).throw(ValueError('original receipt expired'))
    clock[0]=100.7
    expired=json.loads(m.PerceptionRecovery._status(n,None,NS()).message)
    assert expired['operation_success'] and not expired['ready_now']
    assert expired['completion_event']==e and not expired['current_readiness']['ready']


def test_extension_cap_failure_keeps_legacy_success(monkeypatch):
    assert hasattr(m.PerceptionRecovery,'_evidence_status'),'evidence status not implemented'
    legacy=dict(schema='so101-perception-recovery-v1',state='completed',operation_success=True,ready_now=False,
        qualified=False,execution_enabled=False,padding='x'*16100)
    raw=m.bounded_json(legacy)
    n=NS(expose_completion_evidence=True,_readiness_snapshot=lambda *a:dict(large='x'*1000))
    out=m.PerceptionRecovery._evidence_status(n,deepcopy(legacy))
    assert m.bounded_json(out)==raw and 'completion_event' not in out


def test_readonly_exposure_leaf_launch_and_no_supervisor_forwarding():
    root=Path(__file__).resolve().parents[1]
    source=(root/'ros2/src/so101_station_perception/launch/perception_recovery.launch.py').read_text()
    assert "DeclareLaunchArgument('expose_completion_evidence', default_value='false')" in source
    assert "'expose_completion_evidence': ParameterValue(LaunchConfiguration('expose_completion_evidence'),value_type=bool)" in source
    assert 'expose_completion_evidence' not in (root/'ros2/src/so101_station_perception/so101_station_perception/perception_supervisor.py').read_text()


def test_event_gate_original_expiry_after_clock_access(monkeypatch):
    e=event();clock=[100.49]
    def ros():clock[0]=100.51;return NS(nanoseconds=10_490_000_000)
    n=NS(closed=False,cancel=NS(is_set=lambda:False),budget=NS(deadline=139.),
        get_clock=lambda:NS(now=ros),_evidence_error=None)
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    assert m.PerceptionRecovery._event_commit_gate(n,e) is None
    assert 'original clock gate' in n._evidence_error


def test_missing_evidence_duplicate_cannot_reconstruct_and_rolling_pair_aligned(monkeypatch):
    from collections import deque
    from test_derived_registration import fixture
    from test_perception_recovery_core import proof
    clock=[100.15];ns=[10_150_000_000]
    n=NS(_event_high={},_event_witnesses={},_evidence_error=None,
        _rpc_trace=(100.12,100.13,100.14,100.4),ledger=NS(entries=[]),
        progress={CONSUMERS[0]:NS(witnesses=deque(maxlen=2))},get_clock=lambda:NS(now=lambda:NS(nanoseconds=ns[0])))
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    v=fixture();p=proof(v);n.progress[CONSUMERS[0]].witnesses.append(p)
    # Missing selected original statement is negative metadata, no exception or reconstruction.
    m.PerceptionRecovery._capture_event_witness(n,CONSUMERS[0],p,v)
    n.ledger.entries=[(v,bytes.fromhex('6'*32),100.11)]
    m.PerceptionRecovery._capture_event_witness(n,CONSUMERS[0],p,v)
    assert not n._event_witnesses[CONSUMERS[0]]
    for index in (1,2):
        value=deepcopy(v);value['sequence']=index+1
        for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][k]+=index*100_000_000
        value['health']['source_receipts_monotonic']=dict.fromkeys(value['health']['source_receipts_monotonic'],100.1+index*.1)
        value['health']['source_expiry_monotonic']=100.35+index*.1
        item=proof(value);item['checked_ros_ns']+=index*100_000_000;item['original_receipt_expiry_monotonic']=100.3+index*.1
        clock[0]=100.15+index*.1;ns[0]=10_150_000_000+index*100_000_000
        n._rpc_trace=(100.12+index*.1,100.13+index*.1,100.14+index*.1,100.4+index*.1)
        n.progress[CONSUMERS[0]].witnesses.append(item)
        n.ledger.entries.append((value,bytes.fromhex('6'*32),100.11+index*.1))
        m.PerceptionRecovery._capture_event_witness(n,CONSUMERS[0],item,value)
    assert [x['proof']['sequence'] for x in n._event_witnesses[CONSUMERS[0]]]==[2,3]


def test_status_materialization_crossing_original_limits_never_ready(monkeypatch):
    # Exercise actual opt-in status after a metadata serialization stall.
    clock=[100.1];original=m.bounded_json;calls=[0]
    def serialize(value):
        raw=original(value)
        if 'current_readiness' in value:clock[0]=100.3
        return raw
    monkeypatch.setattr(m,'bounded_json',serialize);monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    n=NS(lock=threading.RLock(),result=dict(state='completed',operation_success=True,ready_now=True),
        expose_completion_evidence=True,_active=lambda:False,_artifact_unchanged=lambda:None,_proofs_ready=lambda **kw:True,
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_100_000_000)))
    def snapshot(ready,reason):
        return dict(schema='so101-current-source-readiness-v1',ready=ready,reason=reason,checked_monotonic=clock[0],checked_ros_ns=10_100_000_000,
            original_receipt_valid_until_monotonic=100.2,original_identity_valid_until_monotonic=100.2,
            original_local_lineage_valid_until_monotonic=100.2,effective_valid_until_monotonic=100.2,
            original_header_valid_until_ros_ns=10_500_000_000,qualified=False,execution_enabled=False)
    n._readiness_snapshot=snapshot;n._evidence_status=lambda result:m.PerceptionRecovery._evidence_status(n,result)
    response=m.PerceptionRecovery._status(n,None,NS())
    assert json.loads(response.message)['operation_success'] and not json.loads(response.message)['ready_now']


def test_legacy_status_implementation_unchanged_and_default_off_has_no_new_capture():
    import subprocess
    root=Path(__file__).resolve().parents[1]
    rel='ros2/src/so101_station_perception/so101_station_perception/perception_recovery.py'
    old=ast.parse(subprocess.check_output(['git','show','049bb3cecc8f8e26ab82aec9defffefe55320044:'+rel],cwd=root,text=True))
    new=ast.parse((root/rel).read_text())
    def method(tree,name):
        return next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
    a=method(old,'_status');b=deepcopy(method(new,'_legacy_status'));b.name='_status'
    assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
    # The default dispatch branch returns legacy before importing/calling new evidence.
    dispatch=method(new,'_status').body[0]
    assert isinstance(dispatch,ast.If) and isinstance(dispatch.body[0],ast.Return)
    assert '_legacy_status' in ast.unparse(dispatch.body[0])


def test_exact_legacy_status_16384_cap_extension_falls_back_without_truncation():
    legacy=dict(state='completed',operation_success=True,ready_now=False,padding='')
    legacy['padding']='x'*(16384-len(m.bounded_json(legacy)))
    assert len(m.bounded_json(legacy))==16384
    n=NS(_readiness_snapshot=lambda *a:dict(extra='bounded new metadata'))
    assert m.PerceptionRecovery._evidence_status(n,deepcopy(legacy))==legacy
    legacy['padding']+='x'
    with pytest.raises(ValueError,match='exceeds bound'):m.bounded_json(legacy)


@pytest.mark.parametrize('kind',['heartbeat','header','job','local','producer_reference'])
def test_opt_in_final_legacy_original_limits_after_last_clock_access(monkeypatch,kind):
    from test_derived_registration import fixture
    e=event();v=fixture();p=e['consumers'][CONSUMERS[0]]['witnesses'][-1]['proof']
    v['sequence']=p['sequence']
    for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):
        v['original'][key]=e['consumers'][CONSUMERS[0]]['witnesses'][-1]['original'][key]
    v['health']['source_expiry_monotonic']=100.55
    local_receipt=100.1 if kind=='local' else 100.31
    clock=[100.3 if kind=='local' else 100.4]
    n=NS(ledger=NS(entries=[(v,None,local_receipt)]),producer=({},'a'*32,100.2),
        budget=NS(started=99.,deadline=139.),expected_generation='a'*32,result={'job_id':'b'*32},
        progress={c:NS(current=deepcopy(x['witnesses'][-1]['proof'])) for c,x in e['consumers'].items()},
        closed=False,cancel=NS(is_set=lambda:False),native=NS(start_ns=9_000_000_000))
    def ros():
        if kind=='heartbeat':clock[0]=100.46
        elif kind=='job':clock[0]=139.01
        elif kind=='local':clock[0]=100.36
        elif kind=='producer_reference':n.producer=(dict(n.producer[0]),n.producer[1],n.producer[2])
        return NS(nanoseconds=10_710_000_000 if kind=='header' else 10_400_000_000)
    n.get_clock=lambda:NS(now=ros)
    monkeypatch.setattr(m.time,'monotonic',lambda:clock[0])
    with pytest.raises(ValueError,match='legacy (original|final)'):
        m.PerceptionRecovery._completion_final(n,None)


def test_current_readiness_includes_original_producer_heartbeat(monkeypatch):
    from test_derived_registration import fixture
    v=fixture();v['sequence']=2
    for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):v['original'][k]+=200_000_000
    v['health']['source_expiry_monotonic']=100.55
    p=event()['consumers'][CONSUMERS[0]]['witnesses'][-1]['proof']
    n=NS(ledger=NS(entries=[(v,None,100.31)]),progress={CONSUMERS[0]:NS(current=p)},producer=({},None,100.1),
        _active=lambda:False,get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_400_000_000)))
    monkeypatch.setattr(m.time,'monotonic',lambda:100.4)
    r=m.PerceptionRecovery._readiness_snapshot(n,True,None)
    assert r['original_receipt_valid_until_monotonic']==100.35 and not r['ready']
