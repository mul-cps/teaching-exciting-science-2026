"""Native CPU API/transaction fakes, no executor or actual recovery/reset calls."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177'
from copy import deepcopy
import json
import pytest
from std_srvs.srv import Trigger
from test_supervisor_consumer_reconciliation_ros import reconciler
from so101_station_perception import perception_supervisor as m
from so101_station_perception.derived_lineage import statement_pin
from so101_station_perception.admission_proof import pin_digest
from so101_station_perception.late_adoption import bounded_read,original_descendant


@pytest.fixture
def late(reconciler,monkeypatch):
    node,clock=reconciler
    node.policy.job=dict(state='accepted',reserved_monotonic=90.,origin='operator_reconciliation',reconciliation_id='c'*32,job_id='d'*32,deadline=99.)
    node.policy.attempts.append(990.);node.policy.fail('historical local failure')
    node.epochs={c:'a'*32 for c in m.CONSUMERS};node._persist()
    monkeypatch.setattr(node,'_late_coordinator_identity',lambda:None)
    response=node._adopt_completed_job(None,Trigger.Response());assert response.success,response.message
    pin=deepcopy(node.pin);pin['producer_generation']='b'*32
    digest='e'*64;value=dict(schema='so101-perception-recovery-v1',state='completed',operation_success=True,ready_now=False,
        qualified=False,execution_enabled=False,job_id='d'*32,deadline_monotonic=99.,accepted_reset_id='f'*32,
        producer_generation='b'*32,artifact=str(node.recovery_directory/('d'*32+'.json')),artifact_sha256=digest,
        source_pin_sha256=pin_digest(pin),consumers={})
    for consumer in m.CONSUMERS:
        witnesses=[]
        for sequence in (1,2):
            stamp=8_000_000_000+sequence*200_000_000
            witnesses.append(dict(schema='so101-derived-admission-status-v1',state='ready',qualified=False,execution_enabled=False,
                consumer_epoch='1'*32,producer_generation='b'*32,source_pin_sha256=pin_digest(pin),sequence=sequence,
                depth_stamp_ns=stamp,color_stamp_ns=stamp,checked_ros_ns=stamp+10_000_000,original_header_expiry_ros_ns=stamp+500_000_000,
                original_receipt_expiry_monotonic=98.+sequence*.2))
        value['consumers'][consumer]=dict(latest=deepcopy(witnesses[-1]),witnesses=witnesses)
    monkeypatch.setattr(m,'read_artifact',lambda _:({'derived_source':{'pin':pin}},digest))
    return node,clock,value


def test_historical_attestation_is_not_current_or_automatic_authority(late):
    node,clock,value=late;node._late_historical(value)
    assert node.late_completion['pin']['producer_generation']=='b'*32
    assert node.pin['producer_generation']=='a'*32 and node.policy.job['deadline']==99. and not node.policy.enabled
    assert node.pre_outage.token is None and node.late_intent['deadline_monotonic']==102.
    assert len(node.policy.attempts)==1


@pytest.mark.parametrize('change',[lambda v:v.update(job_id='0'*32),lambda v:v.update(state='failed'),
    lambda v:v.update(deadline_monotonic=140.),lambda v:v.update(artifact_sha256='0'*64),
    lambda v:v['consumers'][m.CONSUMERS[0]]['witnesses'][0].update(schema='so101-derived-source-binding-health-v1'),
    lambda v:v['consumers'][m.CONSUMERS[0]]['witnesses'][0].update(checked_ros_ns=20_000_000_000)])
def test_unknown_interleaved_artifact_or_invalid_historical_timing_rejected_without_mutation(late,change):
    node,_,value=late;change(value)
    with pytest.raises(ValueError):node._late_historical(value)
    assert node.late_completion is None and node.policy.job['job_id']=='d'*32 and len(node.policy.attempts)==1


def test_local_commit_remains_disabled_retains_quota_and_original_job_marker(late,monkeypatch):
    node,clock,value=late;node._late_historical(value);checks=[]
    monkeypatch.setattr(node,'_late_final',lambda:checks.append(clock[0]) or node._late_deadline())
    node.late_ledger=object();node._late_commit()
    assert len(checks)==4 and node.late_result['state']=='completed' and node.late_result['operation_success'] is True
    assert not node.policy.enabled and node.policy.state=='disabled' and node.policy.job is None and node.policy.latched is None
    assert len(node.policy.attempts)==1 and not node.policy.handled and node.pre_outage.token is None
    stored=bounded_read(node.journal.path);marker=node.journal.late_marker
    assert stored['policy']['inflight'] is None and stored['adopted']['job_id']=='d'*32
    assert marker['phase']=='committed_local_success' and marker['owned_job']['deadline']==99.
    assert marker['original_journal']['policy']['inflight']['job_id']=='d'*32
    assert marker['original_journal']['policy']['attempts']==stored['policy']['attempts']


@pytest.mark.parametrize('stage',[3,4])
def test_observed_postwrite_expiry_restores_owned_job_with_paired_current_clocks_and_never_reports_success(late,monkeypatch,stage):
    node,clock,value=late;node._late_historical(value);calls=[0]
    def final():
        calls[0]+=1
        if calls[0]==stage:clock[0]=102.01
        node._late_deadline()
    monkeypatch.setattr(node,'_late_final',final);node.late_ledger=object()
    try:node._late_commit()
    except Exception as error:node._late_failure(error)
    assert node.late_result['state']=='failed' and node.late_result['operation_success'] is False
    assert node.policy.latched and not node.policy.enabled and node.policy.job['job_id']=='d'*32
    actual=bounded_read(node.journal.path)
    assert original_descendant(node.journal.late_marker,actual)
    assert actual['policy']['last_mono']==102.01 and actual['policy']['last_utc']==1002.01
    assert len(actual['policy']['attempts'])==1 and node.journal.late_marker_path.exists()


def test_postreplace_readback_failure_preserves_marker_and_compensation_failure_is_unknown(late,monkeypatch):
    node,clock,value=late;node._late_historical(value)
    monkeypatch.setattr(node,'_late_final',lambda:node._late_deadline());write=node.journal.write
    def broken(policy,**context):
        if policy['inflight'] is None:
            write(policy,**context);raise OSError('candidate replaced but readback unknown')
        raise OSError('compensation filesystem unavailable')
    monkeypatch.setattr(node.journal,'write',broken)
    try:node._late_commit()
    except Exception as error:node._late_failure(error)
    assert node.journal.late_marker_conflict and node.late_result['state']=='failed' and node.policy.latched
    assert node.policy.job['job_id']=='d'*32 and len(node.policy.attempts)==1
    assert node.journal.late_marker['owned_job']['job_id']=='d'*32 and node.journal.late_marker_path.exists()
    assert not node._adopt_completed_job(None,Trigger.Response()).success
    assert len(bounded_read(node.journal.path)['policy']['attempts'])==1


def test_consume_guard_can_finalize_locally_but_never_starts_rpc(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,_,value=late;node._late_historical(value);calls=[]
    node.native_witness={};node.late_progress={c:NS(complete=True) for c in m.CONSUMERS}
    monkeypatch.setattr(node,'_late_collect',lambda start:calls.append(('consume',start)))
    monkeypatch.setattr(node,'_late_commit',lambda:calls.append('local_commit'))
    monkeypatch.setattr(node,'_request',lambda *a,**k:pytest.fail('completion guard started RPC'))
    node._late_guard();assert calls==[('consume',False),'local_commit']


@pytest.mark.parametrize('old',['identity','acquisition'])
def test_operator_native_proof_cannot_reuse_preentry_identity_or_acquisition(late,old):
    node,clock,_=late;clock[0]=100.1
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=10_050_000_000,receipt=100.05) for _ in range(2)) for _ in range(2)],
        selected=((100.05,10_050_000_000),)*2,request=100.01,received=100.02)
    if old=='identity':node.native_witness['request']=99.99
    else:node.native_witness['pairs'][0][0]['stamp']=9_999_000_000
    with pytest.raises(ValueError,match='predates request'):node._native_final(authority=node._late_deadline)
    assert len(node.policy.attempts)==1 and node.policy.job['deadline']==99.


def test_unchanged_entry_budget_includes_acceptance_materialization(late,monkeypatch):
    node,clock,_=late;node.late_intent=None;before=bounded_read(node.journal.path)
    encode=m.bounded_json
    def slow(value):
        text=encode(value)
        if value.get('accepted') is True:clock[0]=102.01
        return text
    monkeypatch.setattr(m,'bounded_json',slow)
    response=node._adopt_completed_job(None,Trigger.Response())
    assert not response.success and node.late_intent is None
    assert bounded_read(node.journal.path)==before and len(node.policy.attempts)==1


def test_prospective_pin_is_volatile_until_final_commit_and_context_change_rejects(late):
    node,_,value=late;old=deepcopy(node.pin);old_epochs=deepcopy(node.epochs)
    node._late_historical(value)
    assert node.pin==old and node.epochs==old_epochs
    node.pin['calibration']['sha256']='0'*64
    with pytest.raises(ValueError,match='context changed'):node._late_deadline()
    assert len(node.policy.attempts)==1 and node.policy.job['job_id']=='d'*32


def test_static_late_adoption_guard_has_no_rpc_native_or_reset_starter():
    import ast
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    tree=ast.parse((root/'ros2/src/so101_station_perception/so101_station_perception/perception_supervisor.py').read_text())
    methods={n.name:ast.unparse(n) for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    for name in ('_late_guard','_late_commit','_late_final','_late_collect'):
        assert 'start_recovery' not in methods[name] and 'reset_source' not in methods[name]
    assert '_request(' not in methods['_late_guard'] and '_native_proof(' not in methods['_late_guard']
    assert '_request(' not in methods['_late_commit'] and '_native_proof(' not in methods['_late_final']
    assert 'if start_requests' in methods['_late_collect']


def test_unmatched_future_image_cannot_hide_as_ordinary_missing_lineage(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,clock,value=late;node._late_historical(value);clock[0]=100.1
    pin=node.late_completion['pin']
    proof=dict(schema='so101-derived-admission-status-v1',qualified=False,execution_enabled=False,state='ready',consumer_epoch='1'*32,
        source_pin_sha256=pin_digest(pin),producer_generation='b'*32,sequence=999,depth_stamp_ns=10_050_000_000,color_stamp_ns=10_050_000_000,
        checked_ros_ns=11_000_000_000,original_header_expiry_ros_ns=10_550_000_000,original_receipt_expiry_monotonic=100.3)
    monkeypatch.setattr(node,'_late_lineages',lambda:NS(entries=[]))
    monkeypatch.setattr(node,'_take',lambda endpoint:None if endpoint==m.COORDINATOR+'/recovery_status' else (True,proof,100.09))
    with pytest.raises(ValueError,match='timing schema/future'):node._late_collect(False)
    assert node.policy.job['job_id']=='d'*32 and len(node.policy.attempts)==1


@pytest.mark.parametrize('mutation',['expiry','cancel'])
def test_readonly_request_rechecks_original_entry_after_service_ready(late,mutation):
    from types import SimpleNamespace as NS
    node,clock,_=late;calls=[];endpoint=m.CONSUMERS[0]+'/derived_source_status'
    def ready():
        if mutation=='expiry':clock[0]=102.01
        else:node.late_intent=None
        return True
    node.rpc_clients[endpoint]=NS(service_is_ready=ready,call_async=lambda _:calls.append('RPC'))
    with pytest.raises(ValueError):node._request(endpoint,2.)
    assert not calls and endpoint not in node.rpc and len(node.policy.attempts)==1


def test_late_readonly_response_deadline_is_original_entry_not_rebased_duration(late):
    from types import SimpleNamespace as NS
    node,clock,_=late;endpoint=m.COORDINATOR+'/recovery_status'
    def ready():clock[0]=100.1;return True
    future=NS(add_done_callback=lambda _:None)
    node.rpc_clients[endpoint]=NS(service_is_ready=ready,call_async=lambda _:future)
    node._request(endpoint,2.)
    assert node.rpc[endpoint]['started']==100.1 and node.rpc[endpoint]['deadline']==102.
    node.rpc.clear()  # Tiny fake future has no teardown cancel methods.


def test_device_identity_request_cannot_start_after_service_ready_consumes_entry_budget(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,clock,_=late;_,calls,_=buffered_native(node,clock,monkeypatch)
    def ready():clock[0]=102.01;return True
    node.device_client=NS(service_is_ready=ready,call_async=lambda _:calls.append('DeviceInfo'))
    with pytest.raises(ValueError,match='entry2s'):node._late_native_proof()
    assert not calls and node.identity_rpc is None


def current_gates(node,clock,monkeypatch):
    """Actual final validators, synthetic native/IMAGE lineage; no payload/RPC."""
    from collections import deque
    from types import SimpleNamespace as NS
    from test_derived_registration import fixture,endpoints
    from so101_station_perception.recovery_core import ConsumerProgress
    from types import MethodType
    clock[0]=100.12
    first=fixture();first['producer_generation']='b'*32
    second=deepcopy(first);second['sequence']=2
    for value,offset,receipt in ((first,30_000_000,100.03),(second,100_000_000,100.1)):
        for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):value['original'][key]+=offset
        value['outputs']['cloud_stamp_ns']=value['outputs']['aligned_stamp_ns']=value['original']['depth_stamp_ns']
        value['health'].update(source_receipts_monotonic=dict.fromkeys(m.TOPICS,receipt),source_expiry_monotonic=receipt+.25,
            observation_ros_ns=max(value['original'][k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')))
    pin=node.late_completion['pin'];rows=[(json.dumps(v),'6'*32,r) for v,r in ((first,100.04),(second,100.11))]
    node.raw_lineages=deque(rows,maxlen=8)
    node.late_progress={};node.late_receipts={};node.late_coord_receipt=100.11
    for consumer in m.CONSUMERS:
        progress=ConsumerProgress(node.epochs[consumer],pin,start_ns=10_000_000_000)
        for value,now,ns in ((first,100.05,10_050_000_000),(second,100.12,10_120_000_000)):
            proof=dict(schema='so101-derived-admission-status-v1',state='ready',qualified=False,execution_enabled=False,
                consumer_epoch='1'*32,producer_generation='b'*32,source_pin_sha256=pin_digest(pin),sequence=value['sequence'],
                depth_stamp_ns=value['original']['depth_stamp_ns'],color_stamp_ns=value['original']['color_stamp_ns'],checked_ros_ns=ns,
                original_receipt_expiry_monotonic=value['health']['source_expiry_monotonic'],original_header_expiry_ros_ns=value['original']['depth_stamp_ns']+500_000_000)
            progress.observe(proof,value,now_ns=ns,now_mono=now)
        node.late_progress[consumer]=progress;node.late_receipts[consumer]=100.11
    node.native_witness=dict(started=100.,pairs=[tuple(dict(stamp=ns,receipt=r) for _ in range(2)) for ns,r in ((10_040_000_000,100.04),(10_100_000_000,100.1))],
        selected=((100.1,10_100_000_000),)*2,request=100.01,received=100.02)
    gid='9'*32;node.producer=(dict(state='ready',fault=None,producer_generation='b'*32),gid,100.11);node.heartbeat_gid=gid
    graph=endpoints(second)
    graph[m.PRODUCER_STATUS]=[NS(node_namespace='/',node_name=m.PRODUCER[1:],endpoint_gid=bytes.fromhex(gid))]
    monkeypatch.setattr(node,'get_publishers_info_by_topic',graph.__getitem__)
    monkeypatch.setattr(node,'_producer_now',MethodType(m.PerceptionSupervisor._producer_now,node))
    monkeypatch.setattr(node,'_graph',MethodType(m.PerceptionSupervisor._graph,node))
    monkeypatch.setattr(node,'get_node_names_and_namespaces',lambda:[(m.COORDINATOR[1:],'/')])
    monkeypatch.setattr(node,'get_service_names_and_types_by_node',lambda *a:[(m.COORDINATOR+'/recovery_status',['std_srvs/srv/Trigger'])])
    monkeypatch.setattr(node,'_late_coordinator_identity',MethodType(m.PerceptionSupervisor._late_coordinator_identity,node))
    return first,second


def test_actual_final_validation_rechecks_original_leases_after_materialization_stall(late,monkeypatch):
    node,clock,value=late;node._late_historical(value);current_gates(node,clock,monkeypatch)
    node._late_final()  # Positive actual full validation at100.12, not a mocked guard.
    progress=node.late_progress[m.CONSUMERS[0]];check=progress.check
    def slow(*a,**kw):
        result=check(*a,**kw);clock[0]=100.42;return result
    monkeypatch.setattr(progress,'check',slow)
    with pytest.raises(ValueError):node._late_final()
    assert node.policy.job['job_id']=='d'*32 and node.pin['producer_generation']=='a'*32 and len(node.policy.attempts)==1


def test_actual_final_gate_and_cold_local_commit_metadata_benchmark(late,monkeypatch):
    import time,statistics
    from pathlib import Path
    from so101_station_perception.supervisor_policy import SupervisorPolicy
    node,clock,value=late;node._late_historical(value);current_gates(node,clock,monkeypatch)
    def measured(call,count):
        samples=[]
        for index in range(count+3):
            began=time.perf_counter_ns();call();elapsed=(time.perf_counter_ns()-began)/1000.
            if index>=3:samples.append(elapsed)
        samples.sort();return dict(samples=count,mean_us=statistics.mean(samples),p95_us=samples[int(.95*(len(samples)-1))],max_us=max(samples))
    result=dict(scope='actual final native/2IMAGE/prospectiveLineage/heartbeat validators; fixed graph and artifact-read fake; coldcommit realtempFS marker+journal/fsync/readback; no actual RPC/pixels/SDK/GPU')
    result['whole_current_final_gate']=measured(node._late_final,100)
    original=bounded_read(node.journal.path);old_pin=deepcopy(node.pin);old_epochs=deepcopy(node.epochs)
    def commit():
        node.policy=SupervisorPolicy(original['policy']);node.pin=deepcopy(old_pin);node.epochs=deepcopy(old_epochs)
        node.journal.write(original['policy'],consumer_epochs=old_epochs,adopted=original['adopted'])
        node.journal.late_marker=None;node.late_transaction_open=False;node.late_commit_started=False;node.late_intent=None
        assert node._adopt_completed_job(None,Trigger.Response()).success
        # Fixed fakeclock entry now100.12; refresh synthetic proof entry offsets.
        node.late_intent['started_monotonic']=100.;node.late_intent['started_ros_ns']=10_000_000_000
        node.late_intent['deadline_monotonic']=102.;node.late_intent['last_ros_ns']=10_000_000_000
        node._late_historical(value);current_gates(node,clock,monkeypatch);node._late_commit()
    result['cold_commit_including_synthetic_setup']=measured(commit,20)
    assert not node.policy.enabled and node.policy.job is None and len(node.policy.attempts)==1
    if os.environ.get('LATE_ADOPTION_BENCHMARK_OUTPUT'):
        Path(os.environ['LATE_ADOPTION_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')


def test_guard_waits_for_already_requested_images_instead_of_committing_cached_old_pair(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,_,value=late;node._late_historical(value);calls=[]
    node.native_witness={};node.late_progress={c:NS(complete=True) for c in m.CONSUMERS}
    node.rpc[m.CONSUMERS[0]+'/derived_source_status']=object()
    monkeypatch.setattr(node,'_late_collect',lambda _:None)
    monkeypatch.setattr(node,'_late_commit',lambda:calls.append('commit'))
    node._late_guard()
    node.rpc.clear()  # Deliberately tiny fake pending record has no teardown methods.
    assert not calls


def test_unresolved_marker_cannot_be_cleared_by_generic_reset_or_enable(late,monkeypatch):
    node,_,_=late;node.late_intent=None;node.policy.job=None
    node.journal.late_marker_conflict=True;node.late_transaction_open=True
    monkeypatch.setattr(node,'_request',lambda *a,**k:pytest.fail('unresolved marker started generic RPC'))
    assert not node._reset_fault(None,Trigger.Response()).success
    assert not node._enable(None,Trigger.Response()).success
    assert node.policy.latched and not node.policy.enabled and len(node.policy.attempts)==1


def test_completed_image_callbacks_before_native_readiness_do_not_starve_final_guard(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,_,value=late;node._late_historical(value);calls=[]
    node.late_progress={c:NS(complete=True) for c in m.CONSUMERS};node.native_witness=None
    def collect(start):
        calls.append('RPCpair' if start else 'consume')
        if start:
            for c in m.CONSUMERS:node.rpc[c+'/derived_source_status']=object()
        else:node.rpc.clear()
    def native(**kw):node.native_witness={};return True
    monkeypatch.setattr(node,'_late_collect',collect);monkeypatch.setattr(node,'_late_native_proof',native)
    monkeypatch.setattr(node,'_late_commit',lambda:calls.append('local_commit'))
    node._late_guard();assert calls==['consume']
    node._late_step();assert calls==['consume','RPCpair']
    node._late_guard();assert calls==['consume','RPCpair','consume','local_commit']


def buffered_native(node,clock,monkeypatch):
    from types import SimpleNamespace as NS
    from sensor_msgs.msg import CameraInfo
    from geometry_msgs.msg import TransformStamped
    from so101_station_perception.derived_lineage import native_snapshot
    from so101_station_perception.recovery_core import NativePairs
    value=node.baseline['camera_evidence']['lineage']['statement'];graph=native_snapshot(value);calls=[];transforms=[]
    node.native=NativePairs(start_ns=10_000_000_000,start_mono=100.)
    node.native_infos={k:[] for k in ('depth','color')}
    for ns,receipt in ((10_030_000_000,100.03),(10_060_000_000,100.06)):
        for kind in ('depth','color'):
            msg=CameraInfo();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
            msg.header.frame_id=value['original'][kind+'_frame'];msg.width,msg.height=value['calibration'][kind+'_size']
            fx,fy,cx,cy=value['calibration'][kind+'_K'];msg.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];msg.d=[0.]*5;msg.distortion_model='plumb_bob'
            gid=graph['endpoints'][kind+'_info']['gid']
            node.native.add(kind,msg,gid,now_ns=ns+1,now_mono=receipt);node.native_infos[kind].append((msg,gid,receipt,ns))
    def lookup(target,source,when):
        transforms.append(when.nanoseconds);tf=TransformStamped();tf.transform.rotation.w=1.;tf.transform.translation.x=.015;return tf
    node.buffer=NS(lookup_transform=lookup)
    monkeypatch.setattr(m,'native_graph',lambda _:graph)
    future=NS(add_done_callback=lambda _:None,cancel=lambda:None)
    node.device_client=NS(service_is_ready=lambda:True,call_async=lambda _:calls.append('DeviceInfo') or future,remove_pending_request=lambda _:None)
    clock[0]=100.12
    return graph,calls,transforms


def test_two_buffered_native_acquisitions_are_usable_in_one_existing_control_callback(late,monkeypatch):
    node,clock,_=late;_,calls,transforms=buffered_native(node,clock,monkeypatch)
    method=getattr(node,'_late_native_proof',lambda:node._native_proof(authority=node._late_deadline))
    assert method() is False  # One independent identity RPC pending; no adoption.
    assert len(node.native.pairs)==2 and calls==['DeviceInfo']
    assert [p[0]['receipt'] for p in node.native.pairs]==[100.03,100.06]
    assert transforms==[10_030_000_000,10_060_000_000]


@pytest.mark.parametrize('which',[0,1])
@pytest.mark.parametrize('fault',['K','TF','missing'])
def test_both_buffered_native_acquisitions_validate_before_identity_rpc(late,monkeypatch,which,fault):
    node,clock,_=late;_,calls,_=buffered_native(node,clock,monkeypatch)
    if fault=='K':node.native_infos['depth'][which][0].k[0]+=10.
    if fault=='missing':node.native_infos['depth'].pop(which)
    if fault=='TF':
        original=node.buffer.lookup_transform;stamp=(10_030_000_000,10_060_000_000)[which]
        def wrong(target,source,when):
            tf=original(target,source,when)
            if when.nanoseconds==stamp:tf.transform.translation.x+=.01
            return tf
        node.buffer.lookup_transform=wrong
    with pytest.raises(ValueError):node._late_native_proof()
    assert calls==[] and node.identity_rpc is None


def test_late_native_pair_refresh_never_renews_identity_request(late,monkeypatch):
    from types import SimpleNamespace as NS
    node,clock,_=late;graph,calls,_=buffered_native(node,clock,monkeypatch)
    assert node._late_native_proof() is False
    original=node.identity_rpc;original['received']=100.13
    original['future']=NS(done=lambda:True,result=lambda:NS(serial_number=node.serial),cancel=lambda:None)
    # New actual acquisitions, first unique receipts; the existing independent
    # DeviceInfo request remains the ORIGINAL request, never restarted/refreshed.
    for kind in ('depth','color'):
        for ns,receipt in ((10_150_000_000,100.15),(10_180_000_000,100.18)):
            msg=deepcopy(node.native_infos[kind][-1][0]);msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
            gid=graph['endpoints'][kind+'_info']['gid'];node.native.add(kind,msg,gid,now_ns=ns+1,now_mono=receipt)
            node.native_infos[kind].append((msg,gid,receipt,ns))
    clock[0]=100.2
    assert node._late_native_proof() is True
    assert node.identity_rpc is original and original['request']==100.12 and calls==['DeviceInfo']
    assert [p[0]['receipt'] for p in node.native_witness['pairs']]==[100.15,100.18]


def test_buffered_native_whole_control_gate_metadata_benchmark(late,monkeypatch):
    from types import SimpleNamespace as NS
    from pathlib import Path
    import statistics,time
    node,clock,_=late;_,calls,transforms=buffered_native(node,clock,monkeypatch)
    assert not node._late_native_proof();rpc=node.identity_rpc;rpc['received']=100.121;clock[0]=100.13
    rpc['future']=NS(done=lambda:True,result=lambda:NS(serial_number=node.serial),cancel=lambda:None)
    samples=[]
    for _ in range(200):
        began=time.perf_counter_ns();assert node._late_native_proof();samples.append((time.perf_counter_ns()-began)/1000)
    samples.sort();assert calls==['DeviceInfo'] and len(node.native.pairs)==2
    result=dict(scope='whole late-only buffered native selection/BOTH actual CameraInfo KD/exactTF/native graph/fixed identity-result/final originallease validator; fake graph+TF+completedfuture, no DDS/RPC/SDK',
        samples=200,mean_us=statistics.mean(samples),p95_us=samples[189],max_us=max(samples),pair_bound=16,actual_TF_checks_per_call=2)
    if os.environ.get('LATE_NATIVE_BENCHMARK_OUTPUT'):Path(os.environ['LATE_NATIVE_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')
