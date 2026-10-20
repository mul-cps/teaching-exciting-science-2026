"""Native ROS types/fake node only; no executors, devices, or service calls."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
from std_srvs.srv import Trigger
from test_supervisor_late_local_adoption import late
from test_supervisor_consumer_reconciliation_ros import reconciler
from so101_station_perception import perception_supervisor as m


def test_explicit_enable_finalizes_own_success_before_failed_live_source_check(late,monkeypatch):
    node,clock,value=late;node._late_historical(value)
    monkeypatch.setattr(node,'_late_final',lambda:node._late_deadline())
    node.late_ledger=object();node._late_commit();before=node.journal.path.read_bytes();seen=[]
    def unavailable():
        assert not node.journal.late_marker_path.exists() and not node.late_transaction_open
        seen.append('source');raise ValueError('live source unavailable')
    monkeypatch.setattr(node,'_baseline_unchanged',unavailable)
    response=node._enable(None,Trigger.Response())
    assert not response.success and seen==['source'] and not node.policy.enabled
    assert node.journal.late_marker is None and not node.journal.late_marker_path.exists()
    assert len(node.policy.attempts)==1 and node.policy.job is None


def test_enable_ack_failure_stays_disabled_and_preserves_journal(late,monkeypatch):
    node,clock,value=late;node._late_historical(value)
    monkeypatch.setattr(node,'_late_final',lambda:node._late_deadline())
    node.late_ledger=object();node._late_commit();before=node.journal.path.read_bytes()
    from so101_station_perception import adoption_ack
    monkeypatch.setattr(adoption_ack,'acknowledge_runtime',lambda *args:(_ for _ in ()).throw(OSError('readback failed')))
    monkeypatch.setattr(node,'_baseline_unchanged',lambda:(_ for _ in ()).throw(AssertionError('must not reach live source')))
    response=node._enable(None,Trigger.Response())
    assert not response.success and not node.policy.enabled and node.policy.latched
    assert node.journal.path.read_bytes()==before and node.journal.late_marker_path.exists()
    assert node.late_transaction_open and len(node.policy.attempts)==1
