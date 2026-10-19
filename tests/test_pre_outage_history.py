"""Volatile historical eligibility: never current health/readiness or motion."""
from copy import deepcopy
import pytest
from so101_station_perception.pre_outage_history import PreOutageHistory
from so101_station_perception.supervisor_policy import SupervisorPolicy
from test_perception_supervisor_ros import supervisor

CONTEXT=dict(pin_sha256='a'*64,consumer_epochs={'board':'b'*32,'objects':'c'*32})

def qualified():
    h=PreOutageHistory();h.observe(CONTEXT,now=100.,ns=10_000_000_000,coverage_end=100.1,armed=True);return h

def fault(**changes):
    return dict(fault_id='e'*32,first_observed_monotonic=100.3,first_observed_ros_ns=10_300_000_000,**changes)

def test_delayed_typed_notice_after_original_health_gap_is_once_eligible_without_current_readiness():
    h=qualified();p=SupervisorPolicy();p.enable(95.)
    for now in (95.,96.,97.,98.,99.,100.):p.healthy(now,True)
    assert p.state=='armed'
    p.healthy(100.11,False);assert p.state=='warming' and p.stable_since is None
    h.claim(CONTEXT,fault(),now=100.4,ns=10_400_000_000)
    p.outage(fault(),100.4,history_verified=True)
    assert p.state=='verifying_native' and p.job is None and not p.attempts
    assert h.status(now=100.4)['pre_outage_history_available'] is False
    assert h.check_active(CONTEXT,now=100.5,ns=10_500_000_000)==102.1
    with pytest.raises(ValueError):h.claim(CONTEXT,fault(),now=100.6,ns=10_600_000_000)


def test_typed_notice_before_health_guard_gap_has_same_original_deadline():
    h=qualified();assert h.claim(CONTEXT,fault(),now=100.4,ns=10_400_000_000)==102.1


@pytest.mark.parametrize('now,ns,first,first_ns',[(102.11,12_110_000_000,102.,12_000_000_000),
    (100.4,10_400_000_000,99.99,9_990_000_000),(100.4,10_400_000_000,100.3,9_999_999_999),
    (100.4,10_400_000_000,100.5,10_300_000_000)])
def test_late_old_or_future_typed_fault_never_claims(now,ns,first,first_ns):
    h=qualified();f=fault();f.update(first_observed_monotonic=first,first_observed_ros_ns=first_ns)
    with pytest.raises(ValueError):h.claim(CONTEXT,f,now=now,ns=ns)
    assert h.active is None


def test_short_healthy_blip_cannot_extend_original_deadline_until_full_requalification():
    h=qualified()
    h.observe(CONTEXT,now=101.,ns=11_000_000_000,coverage_end=101.25,armed=False)
    assert h.token['expires_monotonic']==102.1
    h.observe(CONTEXT,now=106.,ns=16_000_000_000,coverage_end=106.2,armed=True)
    assert h.token['expires_monotonic']==108.2


@pytest.mark.parametrize('change',[lambda c:c.update(pin_sha256='f'*64),lambda c:c['consumer_epochs'].update(board='d'*32)])
def test_exact_source_and_epoch_context_changes_reject_history(change):
    h=qualified();context=deepcopy(CONTEXT);change(context)
    with pytest.raises(ValueError):h.claim(context,fault(),now=100.4,ns=10_400_000_000)


@pytest.mark.parametrize('now,ns',[(99.9,10_100_000_000),(100.1,9_999_999_999),(100.1,12_100_000_001)])
def test_original_clockpair_rollback_or_existing_two_second_drift_bound_rejects(now,ns):
    with pytest.raises(ValueError):qualified().validate(CONTEXT,now=now,ns=ns)


def test_clear_is_volatile_no_journal_authority_and_prevents_claim():
    h=qualified();h.clear()
    with pytest.raises(ValueError):h.claim(CONTEXT,fault(),now=100.4,ns=10_400_000_000)
    assert h.status(now=100.4)==dict(pre_outage_history_available=False,pre_outage_history=None,claimed_pre_outage_expiry_monotonic=None)
    assert 'pre_outage' not in SupervisorPolicy().journal()


def test_real_node_expiry_before_notification_enters_native_verification_without_resurrecting_health(monkeypatch):
    from types import MethodType
    from test_supervisor_health_expiry import fake_node
    from so101_station_perception import perception_supervisor as m
    node,_,clock,_=fake_node(monkeypatch)
    node.policy.stable_since=95.;node.health_coverage_end=100.31;node.health_candidate_end=100.4
    node._apply_current_health(True);assert node.policy.state=='armed'
    clock[0]=100.41;node._apply_current_health(False)
    assert node.policy.state=='warming' and node.health_coverage_end is None
    f=fault();f.update(first_observed_monotonic=100.42,first_observed_ros_ns=10_420_000_000,
        code='private_source_stream_lost',producer_generation=node.pin['producer_generation'],
        producer_node_sha256=node.pin['implementation']['producer_node_sha256'],automatic_recovery_class='transient_private_availability')
    clock[0]=100.5;node._producer_now=lambda:dict(state='unavailable',producer_generation=node.pin['producer_generation'],fault=f)
    node._baseline_unchanged=lambda:None;node._persist=lambda:None;node._clear_health_queries=lambda:None
    node._begin_outage=MethodType(m.PerceptionSupervisor._begin_outage,node)
    m.PerceptionSupervisor._step(node)
    assert node.policy.state=='verifying_native' and node.health_coverage_end is None and not node.policy.attempts
    assert node.pre_outage.active['expires_monotonic']==102.4 and node.native_started==100.5


def test_clipped_history_deadline_crossed_during_service_readiness_never_starts_rpc(monkeypatch):
    from types import SimpleNamespace as NS
    from so101_station_perception import perception_supervisor as m
    clock=[102.09];calls=[];h=qualified();h.claim(CONTEXT,fault(),now=100.4,ns=10_400_000_000)
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    def readiness():clock[0]=102.11;return True
    node=NS(rpc={},rpc_clients={m.COORDINATOR+'/start_recovery':NS(service_is_ready=readiness,call_async=lambda _:calls.append('start'))})
    node._native_final=lambda:h.check_active(CONTEXT,now=clock[0],ns=int(10_000_000_000+(clock[0]-100.)*1e9))
    with pytest.raises(ValueError):m.PerceptionSupervisor._request(node,m.COORDINATOR+'/start_recovery',2.)
    assert not calls and not node.rpc


def test_original_history_deadline_clips_later_native_preflight_start(monkeypatch):
    from types import SimpleNamespace as NS
    from so101_station_perception import perception_supervisor as m
    h=qualified();h.claim(CONTEXT,fault(),now=100.4,ns=10_400_000_000)
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:102.11))
    ns=12_110_000_000
    witness=dict(started=101.9,pairs=[tuple(dict(stamp=ns-1_000_000,receipt=102.1) for _ in range(2)) for _ in range(2)],
        selected=((102.1,ns-1_000_000),)*2,request=102.,received=102.05)
    node=NS(native_witness=witness,get_clock=lambda:NS(now=lambda:NS(nanoseconds=ns)),
        _history_final=lambda:h.check_active(CONTEXT,now=102.11,ns=ns))
    with pytest.raises(ValueError,match='original pre-outage eligibility expired'):m.PerceptionSupervisor._native_final(node)


def test_expired_history_and_delayed_notice_never_starts_new_outage(monkeypatch):
    from test_supervisor_health_expiry import fake_node
    from so101_station_perception import perception_supervisor as m
    node,_,clock,_=fake_node(monkeypatch);node.policy.stable_since=95.;node.health_coverage_end=100.31;node.health_candidate_end=100.4
    node._apply_current_health(True);node._apply_current_health(False)
    clock[0]=102.41;called=[]
    node._producer_now=lambda:dict(state='unavailable');node._begin_outage=lambda _:called.append('outage')
    m.PerceptionSupervisor._step(node)
    assert node.policy.state=='warming' and not called and not node.policy.attempts


def test_disable_reset_and_rejected_ingress_clear_node_history(supervisor,monkeypatch):
    from so101_station_perception import perception_supervisor as m
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    node,_=supervisor;node.epochs={c:'b'*32 for c in m.CONSUMERS}
    def seed():
        now=m.time.monotonic();node.pre_outage.observe(node._history_context(),now=now,ns=node.get_clock().now().nanoseconds,coverage_end=now+.1,armed=True)
    seed();node._disable(None,Trigger.Response());assert node.pre_outage.token is None
    seed();monkeypatch.setattr(node,'_request',lambda *a:None)
    node._reset_fault(None,Trigger.Response());assert node.pre_outage.token is None
    seed();node._lineage(String(data='invalid json'),None);assert node.pre_outage.token is None


def test_history_validation_stall_cannot_hide_native_two_second_deadline(monkeypatch):
    from types import SimpleNamespace as NS
    from so101_station_perception import perception_supervisor as m
    clock=[101.95];calls=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    def history():
        clock[0]+=.1
        assert clock[0]<102.2  # original historical eligibility still live
        return 102.2
    witness=dict(started=100.,pairs=[tuple(dict(stamp=11_900_000_000,receipt=101.9) for _ in range(2)) for _ in range(2)],
        selected=((101.9,11_900_000_000),)*2,request=101.8,received=101.85)
    def start(_):
        calls.append(clock[0]);return NS(add_done_callback=lambda _:None)
    client=NS(service_is_ready=lambda:True,call_async=start)
    node=NS(native_witness=witness,_history_final=history,rpc={},
        rpc_clients={m.COORDINATOR+'/start_recovery':client},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=int((clock[0]-90.)*1e9))))
    node._native_final=lambda:m.PerceptionSupervisor._native_final(node)
    try:m.PerceptionSupervisor._request(node,m.COORDINATOR+'/start_recovery',2.)
    except ValueError:pass
    assert not calls, 'RPC started beyond ORIGINAL native2s deadline while history still live: '+str(calls)


def test_context_hash_stall_cannot_mint_from_expired_current_health(monkeypatch):
    from test_supervisor_health_expiry import fake_node
    node,_,clock,_=fake_node(monkeypatch)
    node.policy.stable_since=95.;node.health_coverage_end=100.31;node.health_candidate_end=100.35
    original=node._history_context
    def slow():
        result=original();clock[0]+=.1;return result
    node._history_context=slow
    node._apply_current_health(True)
    assert node.pre_outage.token is None and node.policy.state=='warming' and node.health_coverage_end is None
