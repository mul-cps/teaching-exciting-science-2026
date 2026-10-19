"""Pure clocks/outage/quota tests: never ROS, SDK, GPU or motor authority."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
from so101_station_perception.supervisor_policy import SupervisorPolicy,transient_fault


def armed():
    p=SupervisorPolicy();p.clocks(100.,1000.);p.enable(100.);p.healthy(100.,True);p.clocks(105.,1005.);p.healthy(105.,True)
    assert p.state=='armed';return p


def fault(i=1):return dict(fault_id=f'{i:032x}')


def test_defaultoff_and_current_stable_window_not_historical_ready():
    p=SupervisorPolicy();p.healthy(1.,True);assert p.state=='disabled'
    p.enable(1.);p.healthy(1.,True);p.healthy(5.99,True);assert p.state=='warming'
    p.healthy(6.,False);p.healthy(7.,True);p.healthy(11.99,True);assert p.state=='warming'
    p.healthy(12.,True);assert p.state=='armed'


def test_durable_reservation_is_pending_before_acceptance_and_disable_survives_completion():
    p=armed();p.outage(fault(),105.);p.reserve(105.,1005.)
    assert p.journal()['inflight']['state']=='dispatch_unknown' and list(p.attempts)==[1005.]
    p.accepted('a'*32,145.,105.);p.disable(106.);p.completed(107.)
    assert not p.enabled and p.state=='disabled'
    with pytest.raises(ValueError):p.outage(fault(2),108.)


def test_unknown_restart_latches_and_cannot_reset_away_inflight():
    p=armed();p.outage(fault(),105.);p.reserve(105.,1005.)
    restored=SupervisorPolicy(p.journal());assert restored.latched and restored.job
    with pytest.raises(ValueError):restored.enable(106.)
    with pytest.raises(ValueError):restored.reset_fault(106.,terminal_proven=True)


def test_manualreset_disable_never_rewinds_quota_or_outage():
    p=armed();p.outage(fault(),105.);p.reserve(105.,1005.);p.job=None;p.fail('known terminal failed')
    p.reset_fault(106.,terminal_proven=True);assert list(p.attempts)==[1005.] and fault()['fault_id'] in p.handled
    p.enable(106.);p.healthy(106.,True);p.clocks(136.,1036.);p.healthy(136.,True)
    with pytest.raises(ValueError):p.outage(fault(),136.)


def test_quota_three_attempts_and_cooldown_retain_after_reset():
    p=armed()
    for i,t in enumerate((105.,140.,175.),1):
        if i>1:p.clocks(t,900.+t);p.healthy(t,True)
        p.outage(fault(i),t);p.reserve(t,900.+t);p.accepted(f'{i:032x}',t+40.,t);p.completed(t+1.)
        p.healthy(t+1.,True)
    p.clocks(210.,1110.);p.healthy(210.,True)
    assert p.latched and len(p.attempts)==3
    p.reset_fault(211.,terminal_proven=True);assert len(p.attempts)==3


@pytest.mark.parametrize('now,utc',[(104.,1006.),(106.,1004.),(106.,1010.)])
def test_clockrollback_or_wall_monotonic_jump_refuses(now,utc):
    p=armed()
    with pytest.raises(ValueError):p.clocks(now,utc)


def test_typed_fault_requires_exact_code_generation_hash_and_original_clocks():
    f=dict(code='private_source_stream_lost',fault_id='a'*32,producer_generation='b'*32,producer_node_sha256='c'*64,
           first_observed_monotonic=100.,first_observed_ros_ns=10,automatic_recovery_class='transient_private_availability')
    s=dict(state='unavailable',fault=f,producer_generation='b'*32)
    assert transient_fault(s,'b'*32,'c'*64,now_mono=101.,now_ns=20)==f
    for key,value in [('code','unknown_nonrecoverable'),('producer_node_sha256','d'*64),('first_observed_monotonic',102.),('producer_generation','e'*32)]:
        old=f[key];f[key]=value
        with pytest.raises(ValueError):transient_fault(s,'b'*32,'c'*64,now_mono=101.,now_ns=20)
        f[key]=old


def test_oversized_persisted_history_rejected_before_deque_truncation():
    p=SupervisorPolicy();p.clocks(100.,1000.);history=p.journal();history['handled']=['a'*32]*129
    with pytest.raises(ValueError):SupervisorPolicy(history)


def test_disable_during_native_verification_prevents_dispatch():
    p=armed();p.outage(fault(),105.);p.disable(105.1)
    with pytest.raises(ValueError):p.reserve(105.2,1005.2)
    assert not p.attempts and p.job is None
