"""CPU native API/clock fakes; no source, SDK, GPU, arm or real recovery job."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177'
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import pytest
from std_srvs.srv import Trigger
from test_derived_registration import fixture
import rclpy
from so101_station_perception import perception_supervisor as m
from so101_station_perception.supervisor_policy import SupervisorPolicy

@pytest.fixture
def reconciler(tmp_path,monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    value=fixture();path=tmp_path/'source.json';journal=tmp_path/'journal';journal.mkdir()
    from so101_station_perception.derived_lineage import statement_pin
    path.write_text(json.dumps(dict(schema='so101-camera-source-v1',qualified=False,execution_enabled=False,
        metadata_only=True,independent_pixels_observed=False,derived_source=dict(schema=value['schema'],qualified=False,pin=statement_pin(value)),
        camera_evidence={'lineage':{'statement':value}})))
    rclpy.init(args=['--ros-args','-p','enabled:=true','-p','expected_source_cohort:='+str(path),
        '-p','expected_serial:='+value['source']['serial_number'],'-p','journal_directory:='+str(journal),
        '-p','recovery_artifact_directory:='+str(tmp_path)])
    node=m.PerceptionSupervisor();clock=[100.]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0],time=lambda:1000.+clock[0]-100.))
    monkeypatch.setattr(node,'get_clock',lambda:NS(now=lambda:NS(nanoseconds=int(10_000_000_000+(clock[0]-100.)*1e9))))
    node.policy=SupervisorPolicy();node.policy.clocks(100.,1000.)
    monkeypatch.setattr(node,'_baseline_unchanged',lambda:None);monkeypatch.setattr(node,'_graph',lambda:None)
    monkeypatch.setattr(node,'_statement',lambda:value)
    monkeypatch.setattr(node,'_producer_now',lambda:dict(state='ready',fault=None,producer_generation=node.pin['producer_generation']))
    try:yield node,clock
    finally:
        # Close while this fixture's module-local clock is STILL installed.
        try:node.close()
        finally:node.destroy_node();rclpy.shutdown()


def admit(node):
    response=node._reconcile_consumers(None,Trigger.Response())
    assert response.success,response.message
    result=json.loads(response.message);assert result['accepted'] and not result['enabled'] and not result['ready_now']
    return result


def test_explicit_request_is_not_typed_fault_or_history_and_does_not_enable(reconciler):
    node,clock=reconciler;result=admit(node)
    assert node.policy.state=='operator_preflight' and node.policy.fault is None and not node.policy.enabled
    assert node.pre_outage.token is None and node.pre_outage.active is None and not node.policy.attempts and not node.rpc
    assert node.native.start_mono==100. and node.native.start_ns==10_000_000_000
    assert node.identity_rpc is None and result['deadline_monotonic']==102.
    assert not node._reconcile_consumers(None,Trigger.Response()).success
    assert not node._enable(None,Trigger.Response()).success


@pytest.mark.parametrize('change',[lambda n:n.policy.enable(100.),lambda n:n.policy.fail('unresolved'),
    lambda n:setattr(n,'reset_pending',True),lambda n:setattr(n.policy,'job',dict(state='dispatch_unknown')),
    lambda n:n.rpc.update({m.COORDINATOR+'/start_recovery':{}})])
def test_busy_enabled_latched_unknown_rejects_without_quota_or_epoch_adoption(reconciler,change):
    node,_=reconciler;change(node);epochs=deepcopy(node.epochs);before=list(node.policy.attempts)
    assert not node._reconcile_consumers(None,Trigger.Response()).success
    assert node.reconciliation is None and node.epochs==epochs and list(node.policy.attempts)==before
    # Cleanup's existing future loop must not inspect deliberately malformed fake records.
    node.rpc.clear()


@pytest.mark.parametrize('stage',['graph','baseline','materialization'])
def test_original_two_seconds_includes_admission_access_and_serialization(reconciler,monkeypatch,stage):
    node,clock=reconciler
    if stage=='materialization':
        original=m.bounded_json
        def slow(value):
            text=original(value)
            if value.get('accepted') is True:clock[0]=102.01
            return text
        monkeypatch.setattr(m,'bounded_json',slow)
    else:monkeypatch.setattr(node,'_'+('graph' if stage=='graph' else 'baseline_unchanged'),lambda:clock.__setitem__(0,102.01))
    assert not node._reconcile_consumers(None,Trigger.Response()).success
    assert node.reconciliation is None and node.policy.state=='disabled' and node.policy.job is None and not node.policy.attempts


def test_own_operator_reservation_is_durable_before_only_fixed_start_rpc(reconciler,monkeypatch):
    node,clock=reconciler;result=admit(node);clock[0]=100.1;calls=[]
    monkeypatch.setattr(node,'_native_proof',lambda **kwargs:bool(kwargs['authority']()))
    monkeypatch.setattr(node,'_native_final',lambda **kwargs:kwargs['authority']())
    monkeypatch.setattr(node,'_take',lambda _:(True,dict(schema='so101-perception-recovery-v1',state='idle'),clock[0]))
    def request(endpoint,timeout,**kwargs):
        stored=json.loads(node.journal.path.read_text())
        assert stored['policy']['inflight']['origin']=='operator_reconciliation'
        assert stored['policy']['inflight']['reconciliation_id']==result['reconciliation_id']
        assert len(stored['policy']['attempts'])==1 and kwargs['native_authority']()==102.
        calls.append(endpoint)
    monkeypatch.setattr(node,'_request',request);node._step()
    assert calls==[m.COORDINATOR+'/start_recovery'] and node.policy.job['origin']=='operator_reconciliation'
    assert node.policy.fault is None and not node.policy.enabled


def test_postreservation_fs_expiry_preserves_charge_unknown_and_no_rpc(reconciler,monkeypatch):
    node,clock=reconciler;admit(node);clock[0]=100.1;calls=[]
    monkeypatch.setattr(node,'_native_proof',lambda **kw:bool(kw['authority']()))
    monkeypatch.setattr(node,'_take',lambda _:(True,dict(schema='so101-perception-recovery-v1',state='idle'),clock[0]))
    persist=node._persist
    def slow():persist();clock[0]=102.01
    monkeypatch.setattr(node,'_persist',slow);monkeypatch.setattr(node,'_request',lambda *a,**k:calls.append(a))
    node._step()
    assert node.policy.latched and not calls and len(node.policy.attempts)==1
    assert node.policy.job['origin']=='operator_reconciliation' and node.policy.job['state']=='dispatch_unknown'
    stored=json.loads(node.journal.path.read_text());assert len(stored['policy']['attempts'])==1 and stored['policy']['inflight']


@pytest.mark.parametrize('identity_started',[99.99,100.])
def test_manual_native_final_rejects_cached_preentry_identity(reconciler,identity_started):
    node,clock=reconciler;admit(node);clock[0]=100.1
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=10_050_000_000,receipt=100.05) for _ in range(2)) for _ in range(2)],
        selected=((100.05,10_050_000_000),)*2,request=identity_started,received=100.01)
    with pytest.raises(ValueError,match='predates request'):node._native_final(authority=node._preflight_deadline)


def test_changed_full_pin_is_not_consumer_reconciliation(reconciler):
    node,clock=reconciler;admit(node);node.pin['outputs']['aligned_publisher_gid']='f'*32
    with pytest.raises(ValueError,match='context/state changed'):node._preflight_deadline()
    assert not node.policy.attempts


def test_manual_final_service_readiness_cannot_cross_original_entry_deadline(reconciler):
    node,clock=reconciler;result=admit(node)
    node.policy.reserve_reconciliation(result['reconciliation_id'],100.1,1000.1)
    clock[0]=101.99;calls=[]
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=11_900_000_000,receipt=101.9) for _ in range(2)) for _ in range(2)],
        selected=((101.9,11_900_000_000),)*2,request=101.8,received=101.85)
    def readiness():clock[0]=102.01;return True
    node.rpc_clients[m.COORDINATOR+'/start_recovery']=NS(service_is_ready=readiness,call_async=lambda _:calls.append('start'))
    with pytest.raises(ValueError):node._request(m.COORDINATOR+'/start_recovery',2.,native_authority=node._preflight_deadline)
    assert not calls and not node.rpc and len(node.policy.attempts)==1


def test_disable_cancels_only_unreserved_operator_preflight(reconciler):
    node,_=reconciler;admit(node)
    response=node._disable(None,Trigger.Response())
    assert response.success and node.reconciliation is None and node.native is None
    assert node.policy.state=='disabled' and node.policy.reconciliation_id is None
    assert not node.policy.attempts and node.policy.job is None
    assert node.reconciliation_result['state']=='cancelled_before_reservation'


def test_disable_never_cancels_owned_reserved_operator_job(reconciler):
    node,_=reconciler;result=admit(node)
    node.policy.reserve_reconciliation(result['reconciliation_id'],100.,1000.)
    node._persist();context=node.reconciliation
    response=node._disable(None,Trigger.Response())
    assert response.success and json.loads(response.message)['accepted_job_continues']
    assert node.reconciliation is context and node.policy.job['origin']=='operator_reconciliation'
    assert len(node.policy.attempts)==1 and not node.policy.enabled
