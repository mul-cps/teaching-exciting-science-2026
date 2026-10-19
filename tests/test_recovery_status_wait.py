"""Temporal heartbeat absence never hides structural or clock contradictions."""
from types import SimpleNamespace as NS
import threading
import pytest
pytest.importorskip('rclpy')
from so101_station_perception import perception_recovery as module


def node(monkeypatch, *, receipt=100., endpoint_gid=b'a'*16):
    monkeypatch.setattr(module.time,'monotonic',lambda:100.3)
    return NS(_guard=lambda:None,lock=threading.RLock(),
        producer=({'producer_generation':'f'*32},(b'a'*16).hex(),receipt),
        get_publishers_info_by_topic=lambda _: [NS(node_namespace='/',node_name='so101_rocm_rgbd',endpoint_gid=endpoint_gid)])


def test_expired_status_has_typed_temporal_result_not_readiness(monkeypatch):
    fake=node(monkeypatch)
    with pytest.raises(module.ProducerStatusPending):module.PerceptionRecovery._producer_status(fake)


@pytest.mark.parametrize('kind',['gid','ambiguous','future','missing','malformed'])
def test_real_status_contradictions_are_not_temporal_pending(monkeypatch,kind):
    fake=node(monkeypatch,receipt=101. if kind=='future' else 100.,endpoint_gid=b'b'*16 if kind=='gid' else b'a'*16)
    if kind=='ambiguous':fake.get_publishers_info_by_topic=lambda _:[]
    if kind in ('missing','malformed'):fake.producer=None
    with pytest.raises(ValueError) as caught:module.PerceptionRecovery._producer_status(fake)
    assert not isinstance(caught.value,module.ProducerStatusPending)


def test_exact_status_receipt_boundary_remains_fresh(monkeypatch):
    fake=node(monkeypatch,receipt=100.05)
    assert module.PerceptionRecovery._producer_status(fake)=={'producer_generation':'f'*32}


@pytest.mark.parametrize('kind',['heartbeat_never_returns','unexpected_generation'])
def test_wait_keeps_original_job_budget_and_never_rebinds_on_unknown_state(monkeypatch,kind):
    from so101_station_perception.recovery_core import Budget
    clock=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    budget=Budget(clock[0]);calls=[];seen=[0]
    def guard():budget.check(clock[0])
    def status():
        seen[0]+=1
        if seen[0]==1:return {'producer_generation':'a'*32}
        if kind=='unexpected_generation':return {'producer_generation':'c'*32}
        raise module.ProducerStatusPending('awaiting current heartbeat')
    def rpc(endpoint,request):
        calls.append(endpoint)
        return NS(success=True,message='')
    fake=NS(budget=budget,lock=threading.RLock(),_guard=guard,_check_baseline=lambda:None,
        native=NS(graph=None,pairs=[],select=lambda *a,**k:True),native_error=None,lineage_error=None,
        get_publishers_info_by_topic=lambda _: [NS(node_namespace='/',node_name='camera',endpoint_gid=bytes([len(calls)+1])*16)],
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
        _phase=lambda _:None,_producer_status=status,_rpc=rpc,_rpc_guard=guard,
        _json_response=lambda _:dict(accepted=True,ready=False,accepted_reset_id='d'*32,new_producer_generation='b'*32),
        result={},_capture=lambda:pytest.fail('must not capture or rebind'))
    # Native graph itself is independently tested; this case isolates the wait.
    monkeypatch.setattr(module,'native_graph',lambda *a,**k:{})
    def pause(*a):clock[0]+=.25;guard()
    fake._pause=pause
    module.PerceptionRecovery._run(fake)
    assert fake.result['state']=='failed' and fake.result['operation_success'] is False
    assert calls==[module.PRODUCER+'/reset_source']
    assert ('absolute recovery40s' if kind=='heartbeat_never_returns' else 'unexpected producer generation') in fake.result['reason']
    if kind=='heartbeat_never_returns':assert budget.deadline==140. and 140.<clock[0]<=140.25


@pytest.mark.parametrize('when',['before_status','during_status'])
def test_pending_heartbeat_does_not_mask_already_latched_lineage_fault(monkeypatch,when):
    from so101_station_perception.recovery_core import Budget
    clock=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    budget=Budget(100.);seen=[];pauses=[]
    def status():
        seen.append(True)
        if len(seen)==1:return {'producer_generation':'a'*32}
        if when=='during_status':fake.ledger.invalid_reason='immutable source generation changed'
        raise module.ProducerStatusPending('old heartbeat expired')
    def phase(value):
        if value=='wait_producer_generation' and when=='before_status':fake.ledger.invalid_reason='immutable source generation changed'
    def pause(*args):
        pauses.append(True)
        raise RuntimeError('must not pause after known immutable fault')
    fake=NS(budget=budget,lock=threading.RLock(),_guard=lambda:budget.check(clock[0]),
        _check_baseline=lambda:None,native=NS(graph=None,pairs=[],select=lambda *a,**k:True),
        native_error=None,lineage_error='immutable source generation changed',
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),get_publishers_info_by_topic=lambda _:[],
        _phase=phase,_producer_status=status,_rpc=lambda *a:NS(success=True,message=''),_rpc_guard=lambda:None,
        _json_response=lambda _:dict(accepted=True,ready=False,accepted_reset_id='d'*32,new_producer_generation='b'*32),
        _pause=pause,result={})
    monkeypatch.setattr(module,'native_graph',lambda *a,**k:{})
    module.PerceptionRecovery._run(fake)
    assert fake.result['state']=='failed' and 'immutable source generation changed' in fake.result['reason']
    assert len(seen)==(1 if when=='before_status' else 2) and not pauses
