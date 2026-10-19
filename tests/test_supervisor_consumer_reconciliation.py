"""Explicit operator reconciliation policy; no fake faults/history or devices."""
from copy import deepcopy
import pytest
from so101_station_perception.supervisor_policy import SupervisorPolicy

RID='a'*32


def test_disabled_operator_reservation_uses_same_quota_without_forging_fault():
    p=SupervisorPolicy();p.clocks(100.,1000.);p.handled.append('b'*32)
    p.begin_reconciliation(RID,100.)
    assert not p.enabled and p.fault is None and p.state=='operator_preflight'
    p.reserve_reconciliation(RID,100.1,1000.1)
    assert p.job==dict(state='dispatch_unknown',reserved_monotonic=100.1,origin='operator_reconciliation',reconciliation_id=RID)
    assert list(p.attempts)==[1000.1] and list(p.handled)==['b'*32] and p.fault is None
    restored=SupervisorPolicy(p.journal())
    assert restored.latched and restored.job==p.job and list(restored.attempts)==[1000.1]
    p.accepted('c'*32,140.,100.2);p.completed(101.)
    assert p.state=='disabled' and not p.enabled and p.job is None and list(p.handled)==['b'*32]


@pytest.mark.parametrize('mutate',[lambda p:p.enable(100.),lambda p:p.fail('unresolved'),
    lambda p:setattr(p,'job',dict(state='dispatch_unknown')),lambda p:setattr(p,'state','verifying_native')])
def test_operator_entry_rejects_enabled_latched_or_nonterminal_policy(mutate):
    p=SupervisorPolicy();mutate(p);before=deepcopy(p.journal())
    with pytest.raises(ValueError):p.begin_reconciliation(RID,100.)
    assert p.journal()==before


@pytest.mark.parametrize('count,delta',[(3,31.),(1,29.)])
def test_manual_reservation_cannot_reset_hourly_budget_or_cooldown(count,delta):
    p=SupervisorPolicy();p.clocks(100.,1000.);p.attempts.extend([1000.]*count)
    p.begin_reconciliation(RID,100.)
    with pytest.raises(ValueError):p.reserve_reconciliation(RID,100.+delta,1000.+delta)
    assert list(p.attempts)==[1000.]*count and p.job is None


@pytest.mark.parametrize('change',[lambda j:j.pop('reconciliation_id'),lambda j:j.update(origin='unknown'),
    lambda j:j.update(reconciliation_id='bad'),lambda j:j.update(outage_id='f'*32),lambda j:j.update(extra=True)])
def test_manual_journal_requires_strict_discriminated_identity(change):
    p=SupervisorPolicy();p.clocks(100.,1000.);p.begin_reconciliation(RID,100.);p.reserve_reconciliation(RID,100.1,1000.1)
    value=p.journal();change(value['inflight'])
    with pytest.raises(ValueError):SupervisorPolicy(value)


def test_failed_unreserved_operator_preflight_explicit_terminal_reset_clears_only_pending_identity():
    policy=SupervisorPolicy();policy.clocks(100.,1000.)
    policy.begin_reconciliation('a'*32,100.)
    policy.fail('preflight unavailable')
    policy.reset_fault(100.1,terminal_proven=True)
    assert policy.reconciliation_id is None and not policy.attempts and not policy.handled
    policy.begin_reconciliation('b'*32,100.2)
    assert policy.reconciliation_id=='b'*32 and policy.fault is None
