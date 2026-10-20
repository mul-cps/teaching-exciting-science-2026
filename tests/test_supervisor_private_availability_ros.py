"""Native fake APIs and virtual clocks, no actual reset/device/motion."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
from test_supervisor_consumer_reconciliation_ros import reconciler
from so101_station_perception import perception_supervisor as m
from so101_station_perception.admission_proof import pin_digest


def fault(node):
    return dict(schema='so101-derived-rocm-rgbd-v1',qualified=False,execution_enabled=False,state='unavailable',
        producer_generation=node.pin['producer_generation'],reset=dict(state='ready'),fault=dict(code='private_source_stream_lost',
        automatic_recovery_class='transient_private_availability',fault_id='d'*32,producer_generation=node.pin['producer_generation'],
        producer_node_sha256=node.pin['implementation']['producer_node_sha256'],first_observed_monotonic=1.,first_observed_ros_ns=1))


def begin(node,monkeypatch):
    node.allow_private_repair=True;node.epochs={c:'a'*32 for c in m.CONSUMERS};node.policy.enable(100.)
    status=fault(node);node.producer=(status,'9'*32,100.);monkeypatch.setattr(node,'_producer_now',lambda:status)
    node._begin_private_repair(status);return status


def proof(node):
    return dict(schema='so101-derived-source-binding-health-v1',state='unavailable',binding_invalidated=False,
        image_admitted=False,processing_progress=False,qualified=False,execution_enabled=False,
        consumer_epoch='a'*32,producer_generation=node.pin['producer_generation'],source_pin_sha256=pin_digest(node.pin),
        camera_info_validation='independent_native_info',identity_validation='producer_attested_device_info_no_new_rpc',
        optical_tf_validation='local_exact_depth_timestamp')


def test_old_resident_fault_new_entry_not_renewed_fault_or_forged_history(reconciler,monkeypatch):
    node,clock=reconciler;status=begin(node,monkeypatch)
    assert node.native.start_mono==100. and node.repair_intent['deadline_monotonic']==102.
    assert node.policy.fault['first_observed_monotonic']==1. and node.pre_outage.token is None and node.pre_outage.active is None
    assert node.reconciliation is None and node.late_intent is None and list(node.policy.handled)==['d'*32]
    assert not node.policy.attempts and node.policy.state=='private_repair_preflight'
    stored=m.bounded_read(node.journal.path);assert stored['policy']['handled']==['d'*32]
    assert node._snapshot()['private_availability_repair']['entry_monotonic']==100.


@pytest.mark.parametrize('change',[lambda n:setattr(n,'allow_private_repair',False),lambda n:n.epochs.clear(),
    lambda n:setattr(n,'reset_pending',True),lambda n:setattr(n,'late_transaction_open',True)])
def test_optin_bootstrap_and_pending_operation_gates(reconciler,monkeypatch,change):
    node,clock=reconciler;node.allow_private_repair=True;node.epochs={c:'a'*32 for c in m.CONSUMERS};node.policy.enable(100.);change(node)
    with pytest.raises(ValueError):node._begin_private_repair(fault(node))
    assert not node.policy.attempts and not node.policy.handled


@pytest.mark.parametrize('change',[lambda s:s['fault'].update(code='unknown'),lambda s:s['fault'].update(first_observed_monotonic=101.),
    lambda s:s['fault'].update(producer_node_sha256='0'*64),lambda s:s.update(producer_generation='0'*32)])
def test_typed_fault_provenance_not_reason_text(reconciler,monkeypatch,change):
    node,clock=reconciler;node.allow_private_repair=True;node.epochs={c:'a'*32 for c in m.CONSUMERS};node.policy.enable(100.)
    status=fault(node);change(status)
    with pytest.raises(ValueError):node._begin_private_repair(status)
    assert not node.policy.handled and not node.policy.attempts


def test_disable_before_reservation_cancels_only_volatile_intent_preserves_dedupe(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch)
    from std_srvs.srv import Trigger
    assert node._disable(None,Trigger.Response()).success
    assert node.repair_intent is None and not node.policy.enabled and list(node.policy.handled)==['d'*32] and not node.policy.attempts


def test_final_start_rpc_uses_original_entry_budget_after_readiness(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);calls=[]
    node.policy.reserve_private_repair(node.repair_intent['repair_id'],100.1,1000.1)
    future=NS(add_done_callback=lambda _:None)
    def ready():clock[0]=101.9;return True
    endpoint=m.COORDINATOR+'/start_recovery'
    node.rpc_clients[endpoint]=NS(service_is_ready=ready,call_async=lambda _:calls.append(clock[0]) or future)
    monkeypatch.setattr(node,'_native_final',lambda **kw:node._repair_deadline())
    monkeypatch.setattr(node,'_repair_final',lambda:node._repair_deadline())
    node._request(endpoint,2.,native_authority=node._repair_deadline)
    assert calls==[101.9] and node.rpc[endpoint]['deadline']==102.
    node.rpc.clear()


@pytest.mark.parametrize('action',['expired','disable','context'])
def test_no_start_after_readiness_stall_or_state_change(reconciler,monkeypatch,action):
    node,clock=reconciler;begin(node,monkeypatch);calls=[]
    def ready():
        if action=='expired':clock[0]=102.01
        elif action=='disable':node.policy.disable(100.)
        else:node.epochs[m.CONSUMERS[0]]='b'*32
        return True
    endpoint=m.COORDINATOR+'/start_recovery';node.rpc_clients[endpoint]=NS(service_is_ready=ready,call_async=lambda _:calls.append('start'))
    monkeypatch.setattr(node,'_native_final',lambda **kw:node._repair_deadline())
    monkeypatch.setattr(node,'_repair_final',lambda:node._repair_deadline())
    with pytest.raises(ValueError):node._request(endpoint,2.,native_authority=node._repair_deadline)
    assert not calls and not node.rpc


def test_reservation_fs_stall_charged_latched_without_dispatch(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch);calls=[]
    node.repair_bindings={c:(proof(node),100.) for c in m.CONSUMERS}
    monkeypatch.setattr(node,'_private_native_proof',lambda:True)
    monkeypatch.setattr(node,'_native_final',lambda **kw:node._repair_deadline())
    monkeypatch.setattr(node,'_repair_final',lambda:node._repair_deadline())
    monkeypatch.setattr(node,'_take',lambda ep:(True,dict(schema='so101-perception-recovery-v1',state='completed',ready_now=False),clock[0]) if ep.endswith('recovery_status') else None)
    monkeypatch.setattr(node,'_request',lambda *a,**kw:calls.append(a))
    original=node._persist
    def persist():original();clock[0]=102.01
    monkeypatch.setattr(node,'_persist',persist)
    node._step()
    assert node.policy.latched and len(node.policy.attempts)==1 and node.policy.job['state']=='dispatch_unknown'
    assert not any(a[0].endswith('start_recovery') for a in calls)


def test_original_default_paths_ast_unchanged():
    # Frozen reviewed Task A, independent of Git object reachability and
    # ast.dump's Python3.12/3.13 empty-field rendering differences.
    import ast,hashlib,json
    def canonical(n):
        if isinstance(n,ast.AST):return [type(n).__name__,{k:canonical(v) for k,v in ast.iter_fields(n) if v is not None and v!=[]}]
        if isinstance(n,list):return [canonical(v) for v in n]
        return n
    path='ros2/src/so101_station_perception/so101_station_perception/perception_supervisor.py'
    expected={'_preflight_deadline': 'd11e65c6378a6c6fd72f3ac52d90df08032b70e714d17a4aceed8b1c03404250', '_reconciliation_current': 'ddbd097a64c7f95abd85ccf8614c0c3956bc812679e4ff5a5bc66d4010450dfb', '_native_proof': '741af97964f367e14fb717549b0bca6426749886bf652bc69cd554124815ac47', '_late_native_proof': '586af2c472831d9ae486504c9158e6f4d3028f79fc5db7e4ba472e1e68fdfe65', '_begin_outage': '1478fb52d9b5318e98db8be3adfb5f6c049ec57bceda24942f819b1ba59751d2', '_dispatch': '3cc3888a3d42cfe9cc985288ed2f88483c4c155c15b9491363a2169fd9c5e679', '_dispatch_reconciliation': 'ced44c68c23d5558406cc72b01da68e7700ec1c41e82b8abb30b36f119241e8c', '_late_commit': '0835116434cb813a5abf5475210a7e92514ec6b9799970fe2b25705a75a8b219'}
    cls=next(n for n in ast.parse(open(path).read()).body if isinstance(n,ast.ClassDef) and n.name=='PerceptionSupervisor')
    methods={n.name:n for n in cls.body if isinstance(n,ast.FunctionDef)}
    for name,digest in expected.items():
        assert hashlib.sha256(json.dumps(canonical(methods[name]),sort_keys=True).encode()).hexdigest()==digest,name


def test_default_off_unarmed_fault_never_begins_or_reserves(reconciler,monkeypatch):
    node,clock=reconciler;node.epochs={c:'a'*32 for c in m.CONSUMERS};node.policy.enable(100.)
    monkeypatch.setattr(node,'_producer_now',lambda:fault(node));node._step()
    assert not node.allow_private_repair and node.repair_intent is None and not node.policy.attempts and not node.policy.handled


def test_pending_private_intent_cannot_be_reenabled_or_fault_reset(reconciler,monkeypatch):
    from std_srvs.srv import Trigger
    node,clock=reconciler;begin(node,monkeypatch)
    assert not node._enable(None,Trigger.Response()).success
    assert not node._reset_fault(None,Trigger.Response()).success
    assert node.policy.state=='private_repair_preflight' and not node.policy.attempts


def test_accepted_private_job_monitor_uses_original_coordinator40s_not_entry_extension(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch)
    node.policy.reserve_private_repair(node.repair_intent['repair_id'],100.1,1000.1);node.policy.accepted('e'*32,140.,100.2)
    clock[0]=104.;calls=[];future=NS(add_done_callback=lambda _:None)
    endpoint=m.COORDINATOR+'/recovery_status';node.rpc_clients[endpoint]=NS(service_is_ready=lambda:True,call_async=lambda _:calls.append('status') or future)
    node._request(endpoint,2.)
    assert calls==['status'] and node.rpc[endpoint]['deadline']==106.
    node.rpc.clear()


@pytest.mark.parametrize('flag',[1,'true',None])
def test_malformed_runtime_flag_cannot_take_default_off_escape(flag,reconciler):
    node,clock=reconciler;node.allow_private_repair=flag;node.epochs={c:'a'*32 for c in m.CONSUMERS};node.policy.enable(100.)
    with pytest.raises(ValueError):node._begin_private_repair(fault(node))
    assert not node.policy.handled and node.repair_intent is None


def test_inplace_implementation_context_change_rejected_before_reset(reconciler,monkeypatch):
    node,clock=reconciler;begin(node,monkeypatch)
    node.pin['implementation']['producer_node_sha256']='0'*64
    with pytest.raises(ValueError):node._repair_current()
    assert not node.policy.attempts and not node.policy.job
