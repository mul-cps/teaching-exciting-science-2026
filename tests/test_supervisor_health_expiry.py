"""Isolated CPU source behavior; no ROS initialization or service calls."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177','Use fixed isolated177 launcher'
from copy import deepcopy
import json
from types import SimpleNamespace as NS,MethodType
import pytest
from test_derived_registration import fixture
from test_source_binding_health import binding_fixture
from so101_station_perception import perception_supervisor as m
from so101_station_perception.derived_lineage import statement_pin,LineageBuffer
from so101_station_perception.supervisor_policy import SupervisorPolicy


def fake_node(monkeypatch,receipt=100.11):
    value=fixture();pin=statement_pin(value);clock=[100.3];requests=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0],time=lambda:clock[0]+900.))
    policy=SupervisorPolicy();policy.clocks(100.,1000.);policy.enable(100.);policy.stable_since=100.
    proof=binding_fixture(value,checked=100.11,checked_ns=10_110_000_000)
    proof['consumer_epoch']='b'*32
    ledger=LineageBuffer(pin);ledger.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.11)
    node=NS(policy=policy,pre_outage=m.PreOutageHistory(),closed=False,reset_pending=False,pin=pin,epochs={c:'b'*32 for c in m.CONSUMERS},rpc={},health_progress={},health_error=None,
        ledger=ledger,health_cache={},health_next_request={},health_request_starts={},health_rpc_clocks={},
        health_coverage_end=None,health_candidate_end=None,health_gap_reason=None,health_rejections={},health_guard_consumes=0,health_guard_wakeups=0,
        producer=({},None,100.3),_producer_now=lambda:dict(state='ready',fault=None,producer_generation=pin['producer_generation']),
        _graph=lambda:None,_take=lambda _: (True,proof,receipt),_request=lambda *a:requests.append(a),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(10_000_000_000+(clock[0]-100.)*1e9))),_fail=lambda error:policy.fail(str(error)))
    for name in ('_history_context','_history_final','_current_health','_health_queries','_retained_health_statement','_reject_health','_apply_current_health','_health_wakeup'):
        setattr(node,name,MethodType(getattr(m.PerceptionSupervisor,name),node))
    return node,proof,clock,requests


def test_known_past_consumer_response_expiry_inhibits_and_requeries_without_latching(monkeypatch):
    node,_,clock,requests=fake_node(monkeypatch);clock[0]=100.4
    m.PerceptionSupervisor._step(node)
    assert node.policy.latched is None and node.policy.enabled and node.policy.state=='warming'
    assert node.policy.stable_since is None and node.policy.job is None and not node.policy.attempts
    assert len(requests)==2 and len(node.health_rejections)==2 and not node.health_cache


@pytest.mark.parametrize('mutation',[lambda p:p.update(schema='bad'),lambda p:p.update(consumer_epoch='d'*32),
    lambda p:p.update(source_pin_sha256='0'*64),lambda p:p.update(checked_ros_ns=11_000_000_000),
    lambda p:p.update(original_header_expiry_ros_ns=11_000_000_000),lambda p:p.update(original_receipt_expiry_monotonic=float('nan'))])
def test_stale_malformed_context_or_future_proof_remains_fatal(monkeypatch,mutation):
    node,proof,_,_=fake_node(monkeypatch);node.epochs={c:'b'*32 for c in m.CONSUMERS};mutation(proof)
    m.PerceptionSupervisor._step(node)
    assert node.policy.latched and not node.policy.enabled and not node.policy.attempts


def test_true_future_response_clock_remains_fatal(monkeypatch):
    node,_,_,_=fake_node(monkeypatch,receipt=100.31)
    m.PerceptionSupervisor._step(node);assert node.policy.latched and not node.policy.enabled


def test_guard_consumes_without_any_rpc_start_or_chain(monkeypatch):
    node,_,clock,requests=fake_node(monkeypatch);clock[0]=100.4
    for _ in range(20):node._health_wakeup()
    assert not requests and node.health_guard_wakeups==20 and node.health_guard_consumes==40
    assert node.policy.state=='warming' and node.policy.job is None and not node.policy.latched


def test_timer_queries_ratecapped_without_catchup_and_one_pending(monkeypatch):
    node,_,clock,requests=fake_node(monkeypatch)
    node._health_queries();assert len(requests)==2
    for offset in (.001,.01,.19,.199):clock[0]=100.3+offset;node._health_queries()
    assert len(requests)==2
    clock[0]=100.501;node._health_queries();assert len(requests)==4
    node.rpc={c+'/derived_source_health':object() for c in m.CONSUMERS}
    clock[0]=120.;node._health_queries();assert len(requests)==4 and len(node.rpc)==2
    node.rpc.clear();node._health_queries();assert len(requests)==6


def test_short_original_coverage_gaps_never_accumulate_five_seconds(monkeypatch):
    node,_,clock,_=fake_node(monkeypatch)
    for index in range(40):
        clock[0]=100.3+index*.2;node.health_candidate_end=clock[0]+.09
        node._apply_current_health(True)
        assert node.policy.state=='warming' and node.policy.stable_since==clock[0]
    assert not node.policy.attempts and node.health_gap_reason=='original health coverage gap'


def test_real_contiguous_original_coverage_can_accumulate_five_seconds(monkeypatch):
    node,_,clock,_=fake_node(monkeypatch)
    for index in range(27):
        clock[0]=100.3+index*.2;node.health_candidate_end=clock[0]+.25;node._apply_current_health(True)
    assert node.policy.state=='armed' and not node.policy.attempts


def test_clock_cross_during_materialization_drops_original_without_renewing(monkeypatch):
    node,proof,clock,_=fake_node(monkeypatch,receipt=100.29);original=deepcopy(proof)
    observe=m.SourceBindingProgress.observe
    def slow(self,*args,**kwargs):
        result=observe(self,*args,**kwargs);clock[0]+=.1;return result
    monkeypatch.setattr(m.SourceBindingProgress,'observe',slow)
    m.PerceptionSupervisor._step(node)
    assert not node.policy.latched and node.policy.state=='warming' and node.policy.stable_since is None
    assert proof==original and node.policy.job is None and not node.policy.attempts
    assert node.health_rejections


def test_expiry_between_validation_and_final_decision_resets_stability(monkeypatch):
    node,_,clock,_=fake_node(monkeypatch)
    node.health_coverage_end=100.35;node.health_candidate_end=100.35;clock[0]=100.351
    node._apply_current_health(True)
    assert node.policy.state=='warming' and node.policy.stable_since is None
    assert node.health_coverage_end is None and node.health_gap_reason=='original health expired before decision'


@pytest.mark.parametrize('change',[lambda v:v['calibration'].update(sha256='0'*64),
    lambda v:v['health']['source_receipts_monotonic'].update(color=101.),
    lambda v:v['identity'].update(serial_number='unexpected'),lambda v:v['health'].update(state='unknown')])
def test_stale_cached_statement_contradiction_is_not_hidden_by_expiry(monkeypatch,change):
    node,_,_,_=fake_node(monkeypatch);change(node.ledger.entries[-1][0])
    m.PerceptionSupervisor._step(node)
    assert node.policy.latched and not node.policy.enabled and not node.policy.attempts


def test_timer_reanchors_after_callback_without_catchup_or_guard_reset():
    calls=[]
    node=NS(closed=False,_step=lambda:calls.append('step'),publisher=NS(publish=lambda _:calls.append('status')),
        _snapshot=lambda:{},timer=NS(reset=lambda:calls.append('timer_reset')))
    m.PerceptionSupervisor._tick(node)
    assert calls==['step','status','timer_reset']


def test_health_future_cleanup_never_cancels_mutating_coordinator_job():
    calls=[];endpoint=m.COORDINATOR+'/start_recovery';owned={'future':object()}
    rpc={endpoint:owned};clients={}
    for consumer in m.CONSUMERS:
        name=consumer+'/derived_source_health'
        rpc[name]={'future':NS(cancel=lambda n=name:calls.append(('cancel',n)))}
        clients[name]=NS(remove_pending_request=lambda _,n=name:calls.append(('remove',n)))
    node=NS(rpc=rpc,rpc_clients=clients,health_cache={'old':object()},health_progress={'old':object()},health_coverage_end=1.)
    m.PerceptionSupervisor._clear_health_queries(node)
    assert node.rpc=={endpoint:owned} and len(calls)==4
    assert node.health_cache=={} and node.health_progress=={} and node.health_coverage_end is None


def test_absolute_timer_microjitter_skip_is_avoided_by_end_anchored_schedule(monkeypatch):
    node,_,clock,requests=fake_node(monkeypatch)
    clock[0]=100.0001;node._health_queries();assert len(requests)==2
    clock[0]=100.2;node._health_queries();assert len(requests)==2  # Before exact request+.2 cap.
    clock[0]=100.0004+.2;node._health_queries();assert len(requests)==4  # Bounded callback-end+.2.


def test_duplicate_original_proof_never_renews_cached_response_receipt(monkeypatch):
    node,proof,clock,_=fake_node(monkeypatch,receipt=100.11);clock[0]=100.12;proof.update(checked_ros_ns=10_110_000_000,checked_monotonic=100.11);node.producer=({},None,100.11)
    node._current_health(start_requests=False)
    assert all(v[1]==100.11 for v in node.health_cache.values())
    clock[0]=100.2;node._take=lambda _: (True,deepcopy(proof),100.19)
    node._current_health(start_requests=False)
    assert all(v[1]==100.11 for v in node.health_cache.values())
