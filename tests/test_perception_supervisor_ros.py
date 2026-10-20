"""Native CPU fake nodes ONLY. Guard isolated domain BEFORE every ROS init."""
import os
from pathlib import Path
import json
import sys
from types import SimpleNamespace as NS
import time
import pytest
assert os.environ.get('ROS_DOMAIN_ID')=='177','Use tools/test_perception_supervisor.sh (fixed isolated domain177)'
pytest.importorskip('rclpy')
import rclpy
from std_srvs.srv import Trigger
from so101_station_perception import perception_supervisor as module
from test_derived_registration import fixture
from so101_station_perception.derived_lineage import statement_pin
from test_rocm_source_reset import reset_node
from test_ros_rocm_rgbd import module as producer_module


@pytest.fixture(autouse=True)
def isolated_before_ros_init():
    assert os.environ.get('ROS_DOMAIN_ID')=='177'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'


@pytest.fixture
def supervisor(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='177'
    value=fixture();path=tmp_path/'source.json'
    path.write_text(json.dumps(dict(schema='so101-camera-source-v1',qualified=False,execution_enabled=False,
        metadata_only=True,independent_pixels_observed=False,derived_source=dict(schema=value['schema'],qualified=False,pin=statement_pin(value)),
        camera_evidence={'lineage':{'statement':value}})))
    journal=tmp_path/'journal';journal.mkdir()
    rclpy.init(args=['--ros-args','-p','enabled:=true','-p','expected_source_cohort:='+str(path),
                    '-p','expected_serial:='+value['source']['serial_number'],'-p','journal_directory:='+str(journal),
                    '-p','recovery_artifact_directory:='+str(tmp_path)])
    node=None
    try:node=module.PerceptionSupervisor();yield node,value
    finally:
        if node is not None:node.close();node.destroy_node()
        rclpy.shutdown()


def test_native_defaults_disabled_and_enable_does_not_trigger_reset(supervisor):
    node,value=supervisor
    assert node.policy.state=='disabled' and not node.policy.enabled
    response=node._status(None,Trigger.Response());state=json.loads(response.message)
    assert response.success and state['qualified'] is False and state['execution_enabled'] is False
    assert node.policy.job is None and not node.rpc
    assert set(node.rpc_clients)=={module.COORDINATOR+'/start_recovery',module.COORDINATOR+'/recovery_status',*(c+'/derived_source_health' for c in module.CONSUMERS),*(c+'/derived_source_status' for c in module.CONSUMERS)}
    assert not any('reset_source' in k or 'follower' in k for k in node.rpc_clients)


def test_native_enable_needs_actual_source_and_no_arbitrary_paths(supervisor):
    node,_=supervisor
    response=node._enable(None,Trigger.Response())
    assert not response.success and node.policy.latched and node.policy.job is None
    assert node.journal.path.exists()
    assert not node.rpc


def test_disable_cannot_claim_cancellation_or_reenable_after_completion(supervisor):
    node,_=supervisor
    node.policy.enabled=True;node.policy.state='recovering'
    node.policy.job=dict(state='accepted',job_id='a'*32,deadline=time.monotonic()+40.)
    response=node._disable(None,Trigger.Response());status=json.loads(response.message)
    assert response.success and status['accepted_job_continues'] is True and not node.policy.enabled
    node.policy.completed(time.monotonic());assert node.policy.state=='disabled' and not node.policy.enabled


def test_unknown_pending_job_cannot_api_reset_or_start_again(supervisor):
    node,_=supervisor;node.policy.job=dict(state='dispatch_unknown');node._fail('completion unknown')
    assert not node._reset_fault(None,Trigger.Response()).success
    assert not node._enable(None,Trigger.Response()).success
    assert node.policy.job=={'state':'dispatch_unknown'}


@pytest.mark.parametrize('code,message',[('private_source_receipt_gap','source stream gap'),('private_source_stream_lost','source stream lost before publication')])
def test_producer_two_typed_faults_latch_identity_until_explicit_reset(reset_node,code,message):
    node,_,_=reset_node
    error=producer_module.TypedSourceReceiptGap(code,message)
    node._invalidate(error);first=dict(node.fault)
    assert first['code']==code and first['automatic_recovery_class']=='transient_private_availability'
    node._invalidate(RuntimeError('different later fault'))
    assert node.fault==first and node.invalid==message
    node._reset_source(Trigger.Request(),Trigger.Response());node._perform_reset()
    assert node.invalid is None and node.fault is None


def test_unknown_producer_failure_is_never_auto_recoverable(reset_node):
    node,_,_=reset_node
    node._invalidate(ValueError('source stream gap'))  # Same text is not typed authority.
    assert node.fault['code']=='unknown_nonrecoverable'
    assert node.fault['automatic_recovery_class']=='nonrecoverable'


def test_auto_context_may_rotate_private_generation_only():
    from copy import deepcopy
    pin=statement_pin(fixture());other=deepcopy(pin);other['producer_generation']='a'*32
    module.same_auto_context(pin,other)
    for section,key,value in [('outputs','aligned_publisher_gid','e'*32),('implementation','producer_node_sha256','f'*64),('source','depth_publisher_gid','f'*32)]:
        changed=deepcopy(other);old=changed[section][key];changed[section][key]=('0' if old[0]!='0' else '1')*len(old)
        with pytest.raises(ValueError):module.same_auto_context(pin,changed)


def test_durable_reservation_failure_never_dispatches_start(monkeypatch):
    from so101_station_perception.supervisor_policy import SupervisorPolicy
    from types import SimpleNamespace as NS
    p=SupervisorPolicy();p.clocks(time.monotonic(),time.time());p.enabled=True;p.state='verifying_native';p.fault={'fault_id':'a'*32}
    requests=[]
    fake=NS(policy=p,_outage_still_current=lambda:None,_native_proof=lambda:True,_take=lambda _: (True,{'state':'idle'},time.monotonic()),
        _baseline_unchanged=lambda:None,_persist=lambda: (_ for _ in ()).throw(OSError('journal write failure')),
        _request=lambda *a:requests.append(a))
    with pytest.raises(OSError):module.PerceptionSupervisor._dispatch(fake)
    assert requests==[] and p.job['state']=='dispatch_unknown' and len(p.attempts)==1


def test_late_or_unknown_completion_never_adopted(supervisor,monkeypatch):
    node,_=supervisor;node.policy.job=dict(state='accepted',job_id='a'*32,deadline=time.monotonic()-.001)
    node.policy.enabled=True
    monkeypatch.setattr(node,'_adopt_completed',lambda *_:pytest.fail('late result adopted'))
    node._step()
    assert node.policy.latched and node.policy.job['job_id']=='a'*32
    assert not node._reset_fault(None,Trigger.Response()).success


def test_partial_or_unknown_generation_artifact_is_not_adopted(supervisor):
    node,_=supervisor;old=node.pin.copy();node.policy.job=dict(state='accepted',job_id='a'*32)
    node.expected_generation='b'*32;node.accepted_reset_id='c'*32
    value=dict(operation_success=True,state='completed',producer_generation='d'*32,accepted_reset_id='c'*32)
    with pytest.raises(ValueError):node._adopt_completed(value)
    assert node.pin==old and node.adopted is None


def test_reset_requires_actual_terminal_confirmation_and_preserves_quota(supervisor,monkeypatch):
    node,_=supervisor;node.policy.fail('known failed');node.policy.attempts.append(time.time())
    monkeypatch.setattr(node,'_request',lambda *a:None)
    reply=node._reset_fault(None,Trigger.Response());assert reply.success and node.policy.latched and node.reset_pending
    monkeypatch.setattr(node,'_take',lambda _: (True,{'schema':'so101-perception-recovery-v1','state':'native_preflight'},time.monotonic()))
    node._step();assert node.policy.latched and not node.reset_pending and len(node.policy.attempts)==1
    node._reset_fault(None,Trigger.Response())
    monkeypatch.setattr(node,'_take',lambda _: (True,{'schema':'so101-perception-recovery-v1','state':'completed'},time.monotonic()))
    node._step();assert node.policy.latched is None and not node.policy.enabled and len(node.policy.attempts)==1


def test_expired_rpc_record_cleanup_survives_already_removed_future(supervisor):
    from rclpy.task import Future
    node,_=supervisor;endpoint=module.COORDINATOR+'/recovery_status';future=Future()
    node.rpc[endpoint]=dict(future=future,deadline=time.monotonic()-.001)
    node.rpc_clients[endpoint]=NS(remove_pending_request=lambda _:(_ for _ in ()).throw(KeyError('already removed')))
    with pytest.raises(TimeoutError):node._take(endpoint)
    assert endpoint not in node.rpc and future.cancelled()


def test_outage_changed_before_dispatch_cannot_call_coordinator(supervisor,monkeypatch):
    node,_=supervisor;node.policy.enabled=True;node.policy.state='verifying_native'
    original=dict(code='private_source_stream_lost',fault_id='a'*32,producer_generation=node.pin['producer_generation'],
        producer_node_sha256=node.pin['implementation']['producer_node_sha256'],first_observed_monotonic=time.monotonic(),
        first_observed_ros_ns=node.get_clock().now().nanoseconds,automatic_recovery_class='transient_private_availability')
    node.policy.fault=original
    monkeypatch.setattr(node,'_producer_now',lambda:dict(state='unavailable',producer_generation='b'*32,fault=dict(original,producer_generation='b'*32)))
    monkeypatch.setattr(node,'_native_proof',lambda:pytest.fail('changed outage reached native proof'))
    with pytest.raises(ValueError):node._dispatch()
    assert not node.rpc and node.policy.job is None


def test_pending_final_proof_after_reservation_latches_without_rpc_or_infinite_wait(supervisor,monkeypatch):
    node,_=supervisor;node.policy.enabled=True;node.policy.state='verifying_native'
    node.policy.fault={'fault_id':'a'*32};node.native_started=time.monotonic();calls=[]
    monkeypatch.setattr(node,'_outage_still_current',lambda:None)
    proofs=[0]
    def native():
        proofs[0]+=1
        if proofs[0]>1:raise module.NativeProofPending('final native proof missing')
        return True
    monkeypatch.setattr(node,'_native_proof',native)
    monkeypatch.setattr(node,'_take',lambda _: (True,{'state':'idle'},time.monotonic()))
    monkeypatch.setattr(node,'_request',lambda *a:calls.append(a))
    node._step()
    assert node.policy.latched and node.policy.state=='latched' and not calls
    assert len(node.policy.attempts)==1 and node.policy.job['state']=='dispatch_unknown'
    node._step();assert not calls and node.policy.latched


def test_source_binding_metadata_cannot_substitute_coordinator_image_admission(supervisor,monkeypatch):
    from copy import deepcopy
    from test_source_binding_health import binding_fixture
    node,value=supervisor;node.policy.job=dict(state='accepted',job_id='a'*32)
    node.expected_generation='b'*32;node.accepted_reset_id='c'*32
    pin=deepcopy(node.pin);pin['producer_generation']='b'*32
    proof=binding_fixture(value);proof.update(producer_generation='b'*32,source_pin_sha256=module.pin_digest(pin))
    artifact=dict(derived_source=dict(pin=pin));digest='e'*64
    monkeypatch.setattr(node,'_baseline_unchanged',lambda:None)
    monkeypatch.setattr(module,'read_artifact',lambda _:(artifact,digest))
    result=dict(operation_success=True,state='completed',producer_generation='b'*32,accepted_reset_id='c'*32,
        artifact=str(node.recovery_directory/('a'*32+'.json')),artifact_sha256=digest,source_pin_sha256=module.pin_digest(pin),
        consumers={c:dict(latest=proof) for c in module.CONSUMERS})
    with pytest.raises(ValueError,match='actual IMAGE admission'):node._adopt_completed(result)
    assert node.adopted is None and node.pin['producer_generation']!=pin['producer_generation']
