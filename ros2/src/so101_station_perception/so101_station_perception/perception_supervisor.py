"""Default-off metadata-only automatic client of the sole recovery coordinator."""
from collections import deque
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from realsense2_camera_msgs.srv import DeviceInfo
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException

from .derived_lineage import (TOPICS,LINEAGE_TOPIC,LineageBuffer,canonical_gid,configure_ros_domain,
    finite,hex_value,parse_lineage,require,statement_pin,validate_graph,validate_statement,validate_camera)
from .derived_aligned import received_gid
from .admission_proof import pin_digest
from .recovery_core import NativePairs,native_graph,bounded_json
from .perception_recovery import read_artifact,PRODUCER,PRODUCER_STATUS,CONSUMERS
from .rocm_rgbd import extrinsic_matrix
from .supervisor_policy import SupervisorPolicy,transient_fault
from .supervisor_journal import SupervisorJournal
from .pre_outage_history import PreOutageHistory
from .supervisor_health import HealthNotCurrent,statement_coverage
from .source_binding_health import SourceBindingProgress,binding_context,binding_coverage

COORDINATOR='/so101_perception_recovery'
TERMINAL=frozenset(('idle','completed','failed'))


class NativeProofPending(ValueError):
    """Expected absent native witnesses/service; never a contradiction."""


def same_auto_context(reference,candidate):
    """Only private producer generation can rotate automatically; ALL GIDs stay."""
    a,b=deepcopy(reference),deepcopy(candidate)
    a.pop('producer_generation');b.pop('producer_generation')
    require(a==b,'automatic source/code/calibration/graph context changed')


class PerceptionSupervisor(Node):
    def __init__(self):
        super().__init__('so101_perception_supervisor')
        self.journal=None;self.closed=False
        readonly=ParameterDescriptor(read_only=True)
        for name,value in (('enabled',False),('expected_source_cohort',''),('expected_serial','243322071783'),
                           ('journal_directory',''),('recovery_artifact_directory','')):
            self.declare_parameter(name,value,readonly)
        require(self.get_parameter('enabled').value is True,'explicit enabled=true required')
        self.baseline_path=Path(self.get_parameter('expected_source_cohort').value).resolve()
        require(bool(self.get_parameter('expected_source_cohort').value),'reviewed source artifact required')
        self.baseline,self.baseline_sha=read_artifact(self.baseline_path)
        self.original_pin=deepcopy(self.baseline['derived_source']['pin']);self.pin=deepcopy(self.original_pin)
        self.serial=self.get_parameter('expected_serial').value
        require(self.pin['source']['serial_number']==self.serial,'configured serial differs from reviewed source')
        directory=self.get_parameter('recovery_artifact_directory').value
        require(bool(directory) and Path(directory).is_dir(),'trusted recovery artifact directory required')
        self.recovery_directory=Path(directory).resolve()
        require(bool(self.get_parameter('journal_directory').value),'trusted journal directory required')
        self.journal=SupervisorJournal(self.get_parameter('journal_directory').value,self.baseline_sha)
        loaded=self.journal.loaded
        try:
            self.policy=SupervisorPolicy(loaded.get('policy'))
            self.epochs=dict(loaded.get('consumer_epochs',{}));self.adopted=loaded.get('adopted')
            if self.adopted:
                path=self.recovery_directory/(self.adopted['job_id']+'.json')
                artifact,digest=read_artifact(path)
                require(digest==self.adopted['artifact_sha256'],'persisted attributable artifact changed')
                restored=artifact['derived_source']['pin'];same_auto_context(self.original_pin,restored)
                require(restored['producer_generation']==self.adopted['producer_generation'],'persisted generation differs')
                self.pin=restored
            self.policy.clocks(time.monotonic(),time.time())
            self.ledger=LineageBuffer(self.pin);self.raw_lineages=deque(maxlen=8)
            self.producer=None;self.heartbeat_gid=None;self.native=None;self.native_started=None
            self.native_infos={k:deque(maxlen=4) for k in ('depth','color')}
            self.identity=None;self.identity_rpc=None;self.expected_generation=None;self.accepted_reset_id=None
            self.dispatch_ros_ns=None;self.health_error='supervisor disabled';self.health_progress={}
            self.rpc={};self.reset_pending=False;self.pre_outage=PreOutageHistory()
            self.reconciliation=None;self.reconciliation_result=None
            self.health_cache={};self.health_next_request={};self.health_request_starts={};self.health_rpc_clocks={}
            self.health_coverage_end=None;self.health_candidate_end=None;self.health_gap_reason=None
            self.health_rejections={};self.health_guard_consumes=0;self.health_guard_wakeups=0
            self.health_refresh_callbacks=0
            self.health_guard=self.create_guard_condition(self._health_wakeup)
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.rpc_clients={name:self.create_client(Trigger,name) for name in
                (COORDINATOR+'/start_recovery',COORDINATOR+'/recovery_status',
                 *(consumer+'/derived_source_health' for consumer in CONSUMERS))}
            self.device_client=self.create_client(DeviceInfo,self.pin['source']['camera_node']+'/device_info')
            self.subscriptions_=[self.create_subscription(String,PRODUCER_STATUS,self._producer,qos_profile_sensor_data),
                self.create_subscription(String,LINEAGE_TOPIC,self._lineage,qos_profile_sensor_data)]
            for kind in ('depth','color'):
                self.subscriptions_.append(self.create_subscription(CameraInfo,TOPICS[kind+'_info'],self._info(kind),qos_profile_sensor_data))
            for name,callback in (('enable',self._enable),('disable',self._disable),('status',self._status),('reset_fault',self._reset_fault),('reconcile_consumers',self._reconcile_consumers)):
                self.create_service(Trigger,'~/'+name,callback)
            self.publisher=self.create_publisher(String,'~/status',1)
            self.timer=self.create_timer(.2,self._tick)
            self.health_timer=self.create_timer(.1,self._health_refresh)
            self._persist()
        except BaseException:
            self.journal.close();raise

    def _persist(self):
        self.policy.clocks(time.monotonic(),time.time())
        self.journal.write(self.policy.journal(),consumer_epochs=self.epochs,adopted=self.adopted)

    def _fail(self,error):
        self.pre_outage.clear();self.reset_pending=False
        if self.reconciliation is not None:self.reconciliation_result=dict(self.reconciliation_result or {},state='failed',reason=str(error)[:512])
        self.reconciliation=None
        self.policy.fail(str(error));self.health_error=str(error)[:512]
        try:self._persist()
        except Exception as failure:self.health_error+='; journal failure: '+str(failure)[:160]

    def _baseline_unchanged(self):
        _,digest=read_artifact(self.baseline_path)
        require(digest==self.baseline_sha,'reviewed baseline bytes changed')

    def _producer(self,message,info):
        receipt=time.monotonic()
        try:
            value=parse_lineage(message.data)
            require(value.get('schema')=='so101-derived-rocm-rgbd-v1' and value.get('qualified') is False and
                value.get('execution_enabled') is False and hex_value(value.get('producer_generation'),32),'invalid producer status')
            gid=canonical_gid(received_gid(info))
            if self.heartbeat_gid is not None:require(gid==self.heartbeat_gid,'producer heartbeat generation changed')
            self.producer=(value,gid,receipt)
        except (ValueError,TypeError,KeyError) as error:
            self.producer=None
            if self.policy.enabled or self.policy.job is not None:self._fail(error)

    def _lineage(self,message,info):
        receipt=time.monotonic()
        try:
            value=parse_lineage(message.data);same_auto_context(self.original_pin,statement_pin(value))
            gid=canonical_gid(received_gid(info));require(gid==self.pin['outputs']['status_publisher_gid'],'lineage actual generation changed')
            if self.policy.job is None:require(value['producer_generation']==self.pin['producer_generation'],'unexpected external producer generation')
            elif self.expected_generation is not None:
                require(value['producer_generation'] in (self.pin['producer_generation'],self.expected_generation),'interleaved unexpected producer generation')
            validate_statement(value,now_ns=self.get_clock().now().nanoseconds,now_mono=receipt,receipt_mono=receipt)
            self.raw_lineages.append((message.data,gid,receipt))
            if value['producer_generation']==self.pin['producer_generation']:
                if self.ledger.add(message.data,gid,now_ns=self.get_clock().now().nanoseconds,now_mono=receipt):
                    v,g,_=self.ledger.entries[-1];self.ledger.entries[-1]=(v,g,receipt)
        except (ValueError,TypeError,KeyError) as error:
            self.pre_outage.clear();self.ledger.entries.clear();self.health_error=str(error)[:512]
            if any(word in str(error) for word in ('context changed','generation','external')):self._fail(error)

    def _info(self,kind):
        def receive(message,info):
            if self.native is None:return
            try:
                gid=canonical_gid(received_gid(info));require(gid==self.pin['source'][kind+'_info_publisher_gid'],'native Info generation changed')
                cal=self.pin['calibration'];expected=cal[kind+'_K']
                require([message.width,message.height]==cal[kind+'_size'] and
                    message.header.frame_id==self.baseline['camera_evidence']['lineage']['statement']['original'][kind+'_frame'] and
                    np.array_equal(np.asarray([message.k[i] for i in (0,4,2,5)],np.float32),np.asarray(expected,np.float32)),'native Info calibration/profile changed')
                stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec;receipt=time.monotonic()
                if self.native.add(kind,message,gid,now_ns=self.get_clock().now().nanoseconds,now_mono=receipt):
                    self.native_infos[kind].append((message,gid,receipt,stamp))
            except (ValueError,TypeError,KeyError,AttributeError,IndexError) as error:self._fail(error)
        return receive

    def _graph(self):
        template=self.baseline['camera_evidence']['lineage']['statement']
        snapshot=validate_graph(template,self.get_publishers_info_by_topic)
        require(snapshot['node']==self.pin['source']['camera_node'],'native source node changed')
        return snapshot

    def _producer_now(self):
        require(self.producer is not None,'producer heartbeat missing')
        value,gid,receipt=self.producer
        endpoints=self.get_publishers_info_by_topic(PRODUCER_STATUS)
        require(len(endpoints)==1 and endpoints[0].node_namespace.rstrip('/')+'/'+endpoints[0].node_name==PRODUCER and
            canonical_gid(endpoints[0].endpoint_gid)==gid,'producer heartbeat actual endpoint changed')
        if self.heartbeat_gid is None:self.heartbeat_gid=gid
        require(gid==self.heartbeat_gid,'producer heartbeat endpoint changed')
        now=time.monotonic();require(finite(receipt) and 0<=receipt<=now,'producer heartbeat clock future/invalid')
        if now-receipt>.250:raise HealthNotCurrent('producer heartbeat expired',response_done_monotonic=receipt,check_monotonic=now)
        return value

    def _request(self,name,timeout,*,native_authority=None):
        if name in self.rpc:return
        client=self.rpc_clients[name]
        if not client.service_is_ready():
            if name in (c+'/derived_source_health' for c in CONSUMERS):raise HealthNotCurrent('consumer metadata service unavailable')
            raise ValueError('metadata service unavailable: '+name)
        started=time.monotonic()
        if name==COORDINATOR+'/start_recovery':
            if native_authority is None:self._native_final()
            else:self._native_final(authority=native_authority)
        if name in (c+'/derived_source_health' for c in CONSUMERS):
            if not PerceptionSupervisor._health_eligible(self):return False
            if self._producer_now().get('state')!='ready':return False
        future=client.call_async(Trigger.Request())
        record=dict(future=future,started=started,deadline=started+timeout,received=None,request_ros_ns=self.get_clock().now().nanoseconds)
        def completed(_,r=record):
            r.update(received=time.monotonic(),done_ros_ns=self.get_clock().now().nanoseconds)
            if name in (c+'/derived_source_health' for c in CONSUMERS):self.health_guard.trigger()
        future.add_done_callback(completed)
        self.rpc[name]=record

    def _take(self,name):
        record=self.rpc.get(name)
        if record is None:return None
        future=record['future'];now=time.monotonic()
        if now>record['deadline']:
            try:self.rpc_clients[name].remove_pending_request(future)
            except (KeyError,RuntimeError):pass
            future.cancel();del self.rpc[name]
            if name in (c+'/derived_source_health' for c in CONSUMERS):raise HealthNotCurrent('consumer metadata RPC expired',request_started_monotonic=record['started'],check_monotonic=now)
            raise TimeoutError('metadata RPC expired/unknown: '+name)
        if not future.done():return None
        response=future.result();require(response is not None and record['received'] is not None and
            record['started']<=record['received']<=now<=record['deadline'],'metadata response clock invalid/late')
        value=parse_lineage(response.message)
        require(time.monotonic()<=record['deadline'],'metadata materialization expired')
        if name in (c+'/derived_source_health' for c in CONSUMERS):
            self.health_rpc_clocks[name]=dict(request_started_monotonic=record['started'],response_done_monotonic=record['received'],
                take_monotonic=time.monotonic(),request_ros_ns=record.get('request_ros_ns'),done_ros_ns=record.get('done_ros_ns'),take_ros_ns=self.get_clock().now().nanoseconds,proof={
                    k:(v if type(v) in (int,float) and finite(v) or type(v) is str and len(v)<=64 else '<invalid>')
                    for k in ('checked_ros_ns','depth_stamp_ns','color_stamp_ns','original_receipt_expiry_monotonic',
                        'original_header_expiry_ros_ns','checked_monotonic','original_identity_expiry_monotonic','effective_health_expiry_monotonic','sequence','consumer_epoch','producer_generation','source_pin_sha256')
                    for v in (value.get(k),)})
        del self.rpc[name]
        return response.success,value,record['received']

    def _statement(self,proof=None):
        require(bool(self.ledger.entries) and not self.ledger.invalid_reason,'current lineage unavailable')
        latest=self.ledger.entries[-1]
        validate_statement(latest[0],now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic(),receipt_mono=latest[2])
        self._graph()
        if proof is None:return latest[0]
        matches=[r for r in self.ledger.entries if r[0]['sequence']==proof.get('sequence') and
            all(r[0]['original'][k]==proof.get(k) for k in ('depth_stamp_ns','color_stamp_ns'))]
        require(bool(matches),'exact consumer lineage not retained')
        value,gid,receipt=matches[-1]
        validate_statement(value,now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic(),receipt_mono=receipt)
        return value

    def _health_eligible(self):
        return not self.closed and self.policy.enabled and not self.policy.latched and not self.reset_pending and self.policy.job is None and self.policy.state in ('warming','armed')

    def _health_refresh(self):
        if self.closed:return
        self.health_refresh_callbacks+=1
        try:
            if self._health_eligible():self._health_queries()
        except HealthNotCurrent as error:
            self.health_error=str(error)[:512]
        except Exception as error:self._fail(error)
        finally:
            # Read-only starter has its own end-anchored cadence. Never reset
            # the control timer or start RPCs from completion callbacks.
            if not self.closed:self.health_timer.reset()

    def _health_queries(self):
        # Only the100ms read-only timer calls this. Recheck eligibility before
        # EACH actual admission, including after a slow previous call_async.
        for consumer in CONSUMERS:
            if not PerceptionSupervisor._health_eligible(self):return
            status=self._producer_now()
            if status.get('state')!='ready':return
            endpoint=consumer+'/derived_source_health'
            if endpoint not in self.rpc and time.monotonic()>=self.health_next_request.get(endpoint,0.):
                if self._request(endpoint,2.) is False:return
                self.health_next_request[endpoint]=time.monotonic()+.1
                self.health_request_starts[endpoint]=self.health_request_starts.get(endpoint,0)+1

    def _clear_health_queries(self):
        # Read-only health futures cannot cross an explicitly owned reset.
        for consumer in CONSUMERS:
            endpoint=consumer+'/derived_source_health';record=self.rpc.pop(endpoint,None)
            if record is not None:
                try:self.rpc_clients[endpoint].remove_pending_request(record['future'])
                except (KeyError,RuntimeError):pass
                record['future'].cancel()
        self.health_cache.clear();self.health_progress.clear();self.health_coverage_end=None

    def _retained_health_statement(self,proof=None):
        require(not self.ledger.invalid_reason,'cached lineage invalidated: '+str(self.ledger.invalid_reason))
        if not self.ledger.entries:raise HealthNotCurrent('current lineage unavailable')
        if proof is None:return self.ledger.entries[-1]
        matches=[r for r in self.ledger.entries if r[0]['sequence']==proof.get('sequence') and
            all(r[0]['original'][k]==proof.get(k) for k in ('depth_stamp_ns','color_stamp_ns'))]
        if not matches:raise HealthNotCurrent('exact consumer lineage no longer retained')
        return matches[-1]

    def _reject_health(self,consumer,error):
        self.health_cache.pop(consumer,None);self.health_progress.pop(consumer,None)
        self.health_gap_reason=str(error)[:160]
        endpoint=consumer+'/derived_source_health'
        self.health_rejections[consumer]=dict(reason=str(error)[:160],
            rpc=self.health_rpc_clocks.get(endpoint,{}),original_clocks=getattr(error,'clocks',{}))

    def _current_health(self,start_requests=True):
        status=self._producer_now();require(status.get('state')=='ready' and status.get('fault') is None and
            status['producer_generation']==self.pin['producer_generation'],'current producer not healthy')
        # Validation/consumption only; requests belong to the100ms timer.
        self._graph();ready=True;ends=[]
        for consumer in CONSUMERS:
            endpoint=consumer+'/derived_source_health'
            try:
                result=self._take(endpoint)
                if result is not None:
                    success,proof,receipt=result
                    binding_context(proof,self.pin,self.epochs.get(consumer))
                    require(success is (proof['state']=='ready'),'consumer success/schema state contradiction')
                    if not start_requests:self.health_guard_consumes+=1
                    if not success:
                        self._reject_health(consumer,HealthNotCurrent('consumer reports unavailable'));ready=False;continue
                    # Context/future/expiry relationships are checked before any
                    # old proof is classified merely unavailable.
                    value,_,lineage_receipt=self._retained_health_statement(proof)
                    now=time.monotonic();ns=self.get_clock().now().nanoseconds
                    binding_coverage(proof,value,self.pin,receipt,lineage_receipt=lineage_receipt,now=now,ns=ns)
                    statement_coverage(value,self.pin,lineage_receipt,now=now,ns=ns)
                    progress=self.health_progress.get(consumer)
                    if progress is None:progress=SourceBindingProgress(self.pin);self.health_progress[consumer]=progress
                    progress.observe(proof,value,now_ns=ns,now_mono=now,response_receipt=receipt,lineage_receipt=lineage_receipt)
                    self.epochs[consumer]=proof['consumer_epoch']
                    previous=self.health_cache.get(consumer)
                    if previous is None or proof['sequence']!=previous[0]['sequence']:
                        self.health_cache[consumer]=(proof,receipt)
                    # Repeated RPC delivery never renews the first receipt of
                    # an unchanged original consumer acquisition.
                cached=self.health_cache.get(consumer)
                if cached is None:ready=False;continue
                proof,receipt=cached;value,_,lineage_receipt=self._retained_health_statement(proof)
                now=time.monotonic();ns=self.get_clock().now().nanoseconds
                binding_context(proof,self.pin,self.epochs.get(consumer))
                ends.append(binding_coverage(proof,value,self.pin,receipt,lineage_receipt=lineage_receipt,now=now,ns=ns))
                ends.append(statement_coverage(value,self.pin,lineage_receipt,now=now,ns=ns))
                self.health_progress[consumer].check(value,now_ns=ns,now_mono=now)
                ready=ready and self.health_progress[consumer].complete
            except HealthNotCurrent as error:
                self._reject_health(consumer,error);ready=False
        latest,_,receipt=self._retained_health_statement()
        now=time.monotonic();ns=self.get_clock().now().nanoseconds
        ends.append(statement_coverage(latest,self.pin,receipt,now=now,ns=ns))
        ends.append(self.producer[2]+.250)
        self.health_candidate_end=min(ends) if ready else None
        return ready

    def _history_context(self):
        require(set(self.epochs)==set(CONSUMERS) and all(hex_value(v,32) for v in self.epochs.values()),
            'both source-binding epochs required for pre-outage history')
        return dict(pin_sha256=pin_digest(self.pin),consumer_epochs=deepcopy(self.epochs))

    def _history_final(self):
        # Diagnostic eligibility only: current source health can remain false.
        return self.pre_outage.check_active(self._history_context(),now=time.monotonic(),ns=self.get_clock().now().nanoseconds)

    def _apply_current_health(self,valid):
        now=time.monotonic();end=self.health_candidate_end
        if not valid or end is None or now>=end:
            if valid and end is not None and now>=end:self.health_gap_reason='original health expired before decision'
            self.health_coverage_end=None;self.policy.healthy(now,False);return
        if self.health_coverage_end is None or now>self.health_coverage_end:
            self.health_gap_reason='original health coverage gap'
            self.policy.healthy(now,False)
        self.health_coverage_end=end;self.policy.healthy(now,True)
        if self.policy.state=='armed':
            context=self._history_context()
            ns=self.get_clock().now().nanoseconds;now=time.monotonic()
            if now>=end:
                self.health_gap_reason='original health expired before history mint'
                self.health_coverage_end=None;self.policy.healthy(now,False);return
            self.pre_outage.observe(context,now=now,ns=ns,coverage_end=end,armed=True)

    def _health_wakeup(self):
        self.health_guard_wakeups+=1
        if self.closed or not self.policy.enabled or self.policy.latched or self.policy.job is not None or self.policy.state not in ('warming','armed'):return
        try:
            self.policy.clocks(time.monotonic(),time.time())
            if self.pre_outage.token is not None or self.pre_outage.active is not None:
                self.pre_outage.validate(self._history_context(),now=time.monotonic(),ns=self.get_clock().now().nanoseconds)
            # The timer retains sole ownership of typed source-outage dispatch.
            if self._producer_now().get('state')=='unavailable':
                self.health_candidate_end=None;self._apply_current_health(False);return
            valid=self._current_health(start_requests=False)
            self.health_error=None if valid else 'waiting for current source bindings'
            self._apply_current_health(valid)
        except HealthNotCurrent as error:
            self.health_error=str(error)[:512];self.health_gap_reason=str(error)[:160]
            self.health_coverage_end=None;self.policy.healthy(time.monotonic(),False)
        except Exception as error:self._fail(error)

    def _preflight_deadline(self):
        context=self.reconciliation
        if context is None:return self._history_final()  # Automatic authority unchanged.
        require(not self.policy.enabled and not self.policy.latched and not self.reset_pending and
            pin_digest(self.pin)==context['pin_sha256'] and self.baseline_sha==context['baseline_sha256'],
            'operator reconciliation context/state changed')
        if self.policy.job is not None:
            require(self.policy.job.get('origin')=='operator_reconciliation' and
                self.policy.job.get('reconciliation_id')==context['reconciliation_id'],'operator job identity changed')
        ns=self.get_clock().now().nanoseconds;now=time.monotonic()
        require(context['started_monotonic']<=now<=context['deadline_monotonic'] and ns>=context['started_ros_ns'] and
            abs((ns-context['started_ros_ns'])/1e9-(now-context['started_monotonic']))<=2.,
            'original operator reconciliation2s expired/clock changed')
        return context['deadline_monotonic']

    def _reconciliation_current(self):
        self._baseline_unchanged();self._graph();self._statement()
        status=self._producer_now()
        require(status.get('state')=='ready' and status.get('fault') is None and
            status.get('producer_generation')==self.pin['producer_generation'],'same healthy producer required for consumer reconciliation')
        self._preflight_deadline()

    def _native_proof(self,*,authority=None):
        now=time.monotonic();require(now<=min(self.native_started+2.,self._history_final() if authority is None else authority()),'postfault native preflight2s/original history expired')
        self._graph();graph=native_graph(self.get_publishers_info_by_topic)
        require(all(graph['endpoints'][k]['gid']==self.pin['source'][k+'_publisher_gid'] for k in TOPICS),'native source replacement forbids automatic recovery')
        if not self.native.select(graph,now_ns=self.get_clock().now().nanoseconds,now_mono=now):
            raise NativeProofPending('two postfault advancing native pairs unavailable')
        pair=self.native.pairs[-1];records=[]
        for i,kind in enumerate(('depth','color')):
            record=next((r for r in reversed(self.native_infos[kind]) if r[3]==pair[i]['stamp']),None)
            require(record is not None and 0<=now-record[2]<=.250,'native selected Info receipt expired/future');records.append(record)
        value=deepcopy(self.baseline['camera_evidence']['lineage']['statement'])
        for i,kind in enumerate(('depth','color')):value['original'][kind+'_info_stamp_ns']=records[i][3]
        tf=self.buffer.lookup_transform(value['original']['color_frame'],value['original']['depth_frame'],Time(nanoseconds=records[0][3]))
        validate_camera(value,records[0][0],records[1][0],extrinsic_matrix(tf))
        if self.identity_rpc is None:
            if not self.device_client.service_is_ready():raise NativeProofPending('native DeviceInfo unavailable')
            request=time.monotonic();future=self.device_client.call_async(DeviceInfo.Request())
            self.identity_rpc=dict(future=future,request=request,graph=deepcopy(graph),received=None)
            future.add_done_callback(lambda _,r=self.identity_rpc:r.update(received=time.monotonic()))
            return False
        rpc=self.identity_rpc
        require(native_graph(self.get_publishers_info_by_topic)==rpc['graph'],'native graph changed during identity RPC')
        require(time.monotonic()-rpc['request']<=1.,'original independent identity lease expired')
        if not rpc['future'].done():
            require(time.monotonic()-rpc['request']<=.4,'independent DeviceInfo timeout');return False
        require(rpc['received'] is not None and rpc['request']<=rpc['received']<=rpc['request']+.4,'DeviceInfo completion late/unknown')
        response=rpc['future'].result();require(response is not None and response.serial_number==self.serial,'independent camera serial differs')
        require(native_graph(self.get_publishers_info_by_topic)==rpc['graph'],'native graph changed after identity response')
        self.native_witness=dict(started=self.native_started,pairs=deepcopy(tuple(self.native.pairs)),
            selected=tuple((r[2],r[3]) for r in records),request=rpc['request'],received=rpc['received'])
        if authority is None:self._native_final()
        else:self._native_final(authority=authority)
        return True

    def _native_final(self,*,authority=None):
        # Scalars from the ORIGINAL proof, checked after all graph/TF/result
        # accesses and again after service readiness immediately before RPC.
        witness=self.native_witness;history_expiry=self._history_final() if authority is None else authority()
        ns=self.get_clock().now().nanoseconds;now=time.monotonic()
        require(now<=history_expiry,'original pre-outage eligibility expired before dispatch')
        require(0<=now-witness['started']<=2.,'postfault native preflight2s expired')
        if authority is not None:
            context=self.reconciliation
            require(witness['started']==context['started_monotonic'] and witness['request']>context['started_monotonic'] and
                witness['received']>context['started_monotonic'] and all(r['receipt']>=context['started_monotonic'] and
                r['stamp']>context['started_ros_ns'] for pair in witness['pairs'] for r in pair),
                'operator native/identity proof predates request')
        require(len(witness['pairs'])==2 and all(0<=now-r['receipt']<=.250 and
            0<=ns-r['stamp']<=500_000_000 for pair in witness['pairs'] for r in pair),
            'original postfault native pairs expired/future')
        require(len(witness['selected'])==2 and all(0<=now-receipt<=.250 and 0<=ns-stamp<=500_000_000 for receipt,stamp in witness['selected']),
            'original selected native Info expired/future')
        require(0<=witness['request']<=witness['received']<=now<=witness['request']+1. and
            witness['received']-witness['request']<=.4,'original independent identity lease expired/future')

    def _begin_outage(self,status):
        fault=transient_fault(status,self.pin['producer_generation'],self.pin['implementation']['producer_node_sha256'],
                              now_mono=time.monotonic(),now_ns=self.get_clock().now().nanoseconds)
        self._baseline_unchanged();self._graph()
        require(not self.ledger.invalid_reason,'source lineage invalidated before outage')
        self.pre_outage.claim(self._history_context(),fault,now=time.monotonic(),ns=self.get_clock().now().nanoseconds)
        self.policy.outage(fault,time.monotonic(),history_verified=True);self._clear_health_queries();self.native_started=time.monotonic()
        self.native=NativePairs(start_ns=self.get_clock().now().nanoseconds,start_mono=self.native_started)
        self.native_infos={k:deque(maxlen=4) for k in ('depth','color')};self.identity_rpc=None
        self._persist()

    def _outage_still_current(self):
        fault=transient_fault(self._producer_now(),self.pin['producer_generation'],
            self.pin['implementation']['producer_node_sha256'],now_mono=time.monotonic(),now_ns=self.get_clock().now().nanoseconds)
        require(fault==self.policy.fault,'observed outage identity changed before dispatch')

    def _dispatch(self):
        self._outage_still_current()
        if not self._native_proof():return
        endpoint=COORDINATOR+'/recovery_status';result=self._take(endpoint)
        if result is None:
            if endpoint not in self.rpc:self._request(endpoint,2.)
            return
        success,value,receipt=result
        require(success and value.get('state') in TERMINAL and 0<=time.monotonic()-receipt<=.250,'coordinator already active/unknown')
        self._baseline_unchanged();self.policy.reserve(time.monotonic(),time.time());self._persist()
        require(self._native_proof(),'native proof unavailable after durable reservation')
        self._outage_still_current()
        self._native_final()
        self.dispatch_ros_ns=self.get_clock().now().nanoseconds
        self.expected_generation=None;self.accepted_reset_id=None
        self._request(COORDINATOR+'/start_recovery',2.)

    def _dispatch_reconciliation(self):
        self._reconciliation_current()
        if not self._native_proof(authority=self._preflight_deadline):return
        endpoint=COORDINATOR+'/recovery_status';result=self._take(endpoint)
        if result is None:
            if endpoint not in self.rpc:self._request(endpoint,2.)
            return
        success,value,receipt=result
        require(success and value.get('schema')=='so101-perception-recovery-v1' and value.get('state') in TERMINAL and
            0<=time.monotonic()-receipt<=.250,'coordinator not terminal/current for reconciliation')
        self._reconciliation_current()
        self.policy.reserve_reconciliation(self.reconciliation['reconciliation_id'],time.monotonic(),time.time());self._persist()
        require(self._native_proof(authority=self._preflight_deadline),'operator native proof unavailable after reservation')
        self._reconciliation_current();self._native_final(authority=self._preflight_deadline)
        self.dispatch_ros_ns=self.get_clock().now().nanoseconds
        self.expected_generation=None;self.accepted_reset_id=None
        self._request(COORDINATOR+'/start_recovery',2.,native_authority=self._preflight_deadline)

    def _adopt_completed(self,value):
        require(value.get('operation_success') is True and value.get('state')=='completed','coordinator did not complete successfully')
        require(hex_value(value.get('producer_generation'),32) and value['producer_generation']!=self.pin['producer_generation'] and
            value['producer_generation']==self.expected_generation and value.get('accepted_reset_id')==self.accepted_reset_id,'completed reset identity/generation changed')
        path=self.recovery_directory/(self.policy.job['job_id']+'.json')
        require(Path(value.get('artifact','')).resolve()==path,'coordinator artifact path differs from owned job')
        self._baseline_unchanged();artifact,digest=read_artifact(path)
        require(digest==value.get('artifact_sha256'),'owned job artifact hash changed')
        pin=artifact['derived_source']['pin'];same_auto_context(self.original_pin,pin)
        require(pin['producer_generation']==self.expected_generation and pin_digest(pin)==value.get('source_pin_sha256'),'owned artifact generation/pin differs')
        require(set(value.get('consumers',{}))==set(CONSUMERS),'both attributable consumer epochs required')
        epochs={}
        for consumer in CONSUMERS:
            proof=value['consumers'][consumer]['latest'];epoch=proof.get('consumer_epoch')
            require(proof.get('schema')=='so101-derived-admission-status-v1' and proof.get('state')=='ready' and
                proof.get('qualified') is False and proof.get('execution_enabled') is False,
                'coordinator completion requires actual IMAGE admission proof; source health cannot substitute')
            require(hex_value(epoch,32) and epoch!=self.epochs.get(consumer) and
                proof.get('producer_generation')==self.expected_generation and proof.get('source_pin_sha256')==pin_digest(pin),
                'owned completed consumer epoch/generation/pin invalid')
            epochs[consumer]=epoch
        self._graph();ledger=LineageBuffer(pin)
        for raw,gid,receipt in self.raw_lineages:
            v=parse_lineage(raw)
            if v['producer_generation']!=pin['producer_generation']:continue
            now=time.monotonic();now_ns=self.get_clock().now().nanoseconds
            # Expired retained history is not an admission witness. Never renew
            # original receipts while preparing the prospective generation.
            if now-receipt>.250 or any(now_ns-v['original'][k]>500_000_000 for k in
                    ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')):continue
            validate_statement(v,now_ns=now_ns,now_mono=now,receipt_mono=receipt)
            if ledger.add(raw,gid,now_ns=now_ns,now_mono=now):
                a,b,_=ledger.entries[-1];ledger.entries[-1]=(a,b,receipt)
        require(bool(ledger.entries),'no current attributable generation lineage')
        self.pre_outage.clear();self.pin=deepcopy(pin);self.epochs=epochs;self.ledger=ledger;self.health_progress={};self.health_cache={};self.health_coverage_end=None
        self.adopted=dict(job_id=self.policy.job['job_id'],artifact_sha256=digest,producer_generation=pin['producer_generation'])
        if self.reconciliation is not None:
            self.reconciliation_result=dict(self.reconciliation_result,state='completed',job_id=self.policy.job['job_id'],
                artifact_sha256=digest,producer_generation=pin['producer_generation'])
        self.policy.completed(time.monotonic());self.reconciliation=None;self.native=None;self._persist()

    def _monitor_job(self):
        job=self.policy.job
        if job['state']=='dispatch_unknown':
            result=self._take(COORDINATOR+'/start_recovery')
            if result is None:return
            success,value,_=result;require(success and value.get('accepted') is True,'recovery start refused/unknown')
            self.policy.accepted(value.get('job_id'),value.get('deadline_monotonic'),time.monotonic());self._persist()
        require(time.monotonic()<=self.policy.job['deadline'],'owned coordinator absolute deadline expired/unknown')
        endpoint=COORDINATOR+'/recovery_status';result=self._take(endpoint)
        if result is None:
            if endpoint not in self.rpc:self._request(endpoint,min(2.,self.policy.job['deadline']-time.monotonic()))
            return
        success,value,_=result;require(success and value.get('job_id')==self.policy.job['job_id'],'interleaved/unattributed coordinator job')
        generation=value.get('producer_generation');reset=value.get('accepted_reset_id')
        if generation is not None:
            require(hex_value(generation,32) and generation!=self.pin['producer_generation'] and hex_value(reset,32),'invalid reserved reset generation')
            require(self.expected_generation in (None,generation) and self.accepted_reset_id in (None,reset),'reserved reset identity changed')
            self.expected_generation=generation;self.accepted_reset_id=reset
        if value.get('state')=='failed':
            self.policy.job=None;raise RuntimeError('owned coordinator job failed; no retry: '+str(value.get('reason'))[:256])
        if value.get('state')=='completed':self._adopt_completed(value)

    def _tick(self):
        if self.closed:return
        try:self._step()
        finally:
            self.publisher.publish(String(data=bounded_json(self._snapshot())))
            # Anchor the next timer callback AFTER this bounded callback. An
            # absolute200ms timer plus exact200ms start cap can otherwise skip
            # alternate ticks on microsecond scheduling jitter. No catchup.
            if not self.closed:self.timer.reset()

    def _step(self):
        if self.closed:return
        try:
            self.policy.clocks(time.monotonic(),time.time())
            if self.pre_outage.token is not None or self.pre_outage.active is not None:
                self.pre_outage.validate(self._history_context(),now=time.monotonic(),ns=self.get_clock().now().nanoseconds)
            if self.reset_pending:self._finish_reset();return
            if self.policy.latched:return
            if self.policy.job is not None:self._monitor_job();return
            if getattr(self,'reconciliation',None) is not None:
                try:self._dispatch_reconciliation()
                except (NativeProofPending,TransformException) as error:
                    if self.policy.job is None and time.monotonic()<=self._preflight_deadline():
                        self.health_error=str(error)[:512];return
                    raise
                return
            if not self.policy.enabled:return
            if self.policy.state=='verifying_native':
                try:self._dispatch()
                except (NativeProofPending,TransformException) as error:
                    if self.policy.job is None and time.monotonic()<=min(self.native_started+2.,self._history_final()):
                        self.health_error=str(error)[:512];return
                    raise
                return
            status=self._producer_now()
            if status.get('state')=='unavailable':
                if self.policy.state=='armed' or self.pre_outage.status(now=time.monotonic())['pre_outage_history_available']:self._begin_outage(status)
                else:self.health_error='unavailable producer is not a new armed outage';self.policy.healthy(time.monotonic(),False)
                return
            try:valid=self._current_health();self.health_error=None if valid else 'waiting for two current source bindings'
            except HealthNotCurrent as error:
                self.health_error=str(error)[:512];self.health_gap_reason=str(error)[:160];valid=False
            self._apply_current_health(valid)
        except HealthNotCurrent as error:
            if self.policy.job is None and self.policy.state in ('warming','armed'):
                self.health_error=str(error)[:512];self.health_gap_reason=str(error)[:160]
                self.health_coverage_end=None;self.policy.healthy(time.monotonic(),False)
            else:self._fail(error)
        except Exception as error:self._fail(error)

    def _reconcile_consumers(self,request,response):
        # ENTRY clocks include admission, FS/graph access and reply materialization.
        started=time.monotonic();start_ns=self.get_clock().now().nanoseconds;admitted=False
        identity=uuid.uuid4().hex
        try:
            require(not self.closed and not self.reset_pending and self.reconciliation is None and
                not self.policy.enabled and self.policy.state=='disabled' and not self.policy.latched and self.policy.job is None and
                COORDINATOR+'/start_recovery' not in self.rpc,
                'consumer reconciliation requires disabled terminal supervisor')
            self.policy.clocks(time.monotonic(),time.time())
            self._baseline_unchanged();self._graph();self._statement()
            status=self._producer_now()
            require(status.get('state')=='ready' and status.get('fault') is None and status.get('producer_generation')==self.pin['producer_generation'],
                'same healthy producer required for consumer reconciliation')
            context=dict(reconciliation_id=identity,started_monotonic=started,started_ros_ns=start_ns,
                deadline_monotonic=started+2.,pin_sha256=pin_digest(self.pin),baseline_sha256=self.baseline_sha)
            message=bounded_json(dict(accepted=True,reconciliation_id=identity,deadline_monotonic=started+2.,
                state='operator_preflight',ready_now=False,operation_success=False,enabled=False,qualified=False,execution_enabled=False))
            require(time.monotonic()<=started+2.,'operator reconciliation expired before admission')
            self.policy.begin_reconciliation(identity,time.monotonic());admitted=True
            self.pre_outage.clear();self._clear_health_queries();self.reconciliation=context
            self.reconciliation_result=dict(reconciliation_id=identity,state='operator_preflight',deadline_monotonic=started+2.)
            self.native_started=started;self.native=NativePairs(start_ns=start_ns,start_mono=started)
            self.native_infos={k:deque(maxlen=4) for k in ('depth','color')};self.identity_rpc=None
            self._preflight_deadline();response.message=message;response.success=True
        except Exception as error:
            if admitted:self._fail(error)
            response.success=False;response.message=bounded_json(dict(accepted=False,reason=str(error)[:512],
                ready_now=False,operation_success=False,enabled=self.policy.enabled,qualified=False,execution_enabled=False))
        return response

    def _enable(self,request,response):
        if self.reconciliation is not None:
            response.success=False;response.message=bounded_json(dict(self._snapshot(),reason='operator reconciliation pending'));return response
        try:
            self.pre_outage.clear();self._baseline_unchanged();self._graph();self._producer_now()
            self.policy.clocks(time.monotonic(),time.time());self.policy.enable(time.monotonic());self._persist()
            response.success=True
        except Exception as error:self._fail(error);response.success=False
        response.message=bounded_json(self._snapshot());return response

    def _disable(self,request,response):
        if self.policy.job is None:
            if self.reconciliation is not None:self.reconciliation_result=dict(self.reconciliation_result,state='cancelled_before_reservation')
            self.reconciliation=None;self.native=None
        self.pre_outage.clear();self.policy.disable(time.monotonic())
        try:self._persist();response.success=True
        except Exception as error:self._fail(error);response.success=False
        response.message=bounded_json(dict(self._snapshot(),accepted_job_continues=self.policy.job is not None));return response

    def _finish_reset(self):
        result=self._take(COORDINATOR+'/recovery_status')
        if result is None:return
        success,value,receipt=result
        require(success and value.get('schema')=='so101-perception-recovery-v1' and value.get('state') in TERMINAL and
            0<=time.monotonic()-receipt<=.250,'coordinator terminal state unproven')
        require(self.policy.job is None,'active or unknown job cannot reset latch')
        self.policy.reset_fault(time.monotonic(),terminal_proven=True);self.reset_pending=False;self._persist()

    def _reset_fault(self,request,response):
        self.pre_outage.clear()
        # API acceptance requests a bounded terminal-state check; never clears
        # an unknown job, rewinds quota or implicitly enables supervision.
        try:
            require(self.policy.job is None and self.reconciliation is None and not self.reset_pending,'active/unknown job or reconciliation/reset check pending')
            self._request(COORDINATOR+'/recovery_status',2.);self.reset_pending=True
            response.success=True
        except Exception as error:response.success=False;self.health_error=str(error)[:512]
        response.message=bounded_json(dict(self._snapshot(),reset_check_accepted=response.success));return response

    def _snapshot(self):
        return dict(self.policy.status(),**self.pre_outage.status(now=time.monotonic()),operator_reconciliation=self.reconciliation_result,reset_check_pending=self.reset_pending,health_reason=self.health_error,current_producer_generation=self.pin['producer_generation'],
                    current_pin_sha256=pin_digest(self.pin),health_observation=dict(health_refresh_period_s=.1,control_period_s=.2,refresh_callbacks=self.health_refresh_callbacks,coverage_end_monotonic=self.health_coverage_end,
                        gap_reason=self.health_gap_reason,request_starts=self.health_request_starts,guard_consumes=self.health_guard_consumes,
                        guard_wakeups=self.health_guard_wakeups,rpc_clocks=self.health_rpc_clocks,rejected=self.health_rejections),health_scope='continuous source binding only',image_admitted=False,processing_progress=False,scope='source-binding health only; never image progress, motion or physical qualification')

    def _status(self,request,response):
        response.success=True;response.message=bounded_json(self._snapshot());return response

    def close(self):
        self.closed=True;self.health_timer.cancel();self.pre_outage.clear()
        for name,record in self.rpc.items():
            try:self.rpc_clients[name].remove_pending_request(record['future'])
            except (KeyError,RuntimeError):pass
            record['future'].cancel()
        if self.identity_rpc is not None:
            try:self.device_client.remove_pending_request(self.identity_rpc['future'])
            except (KeyError,RuntimeError):pass
            self.identity_rpc['future'].cancel()
        if self.policy.job is not None:self.policy.fail('supervisor closed with accepted/unknown coordinator job; no adoption')
        try:self._persist()
        finally:self.journal.close()


def main(args=None):
    configure_ros_domain(os.environ);rclpy.init(args=args);node=None
    try:node=PerceptionSupervisor();rclpy.spin(node)
    finally:
        try:
            if node is not None:
                try:node.close()
                finally:node.destroy_node()
        finally:rclpy.shutdown()
