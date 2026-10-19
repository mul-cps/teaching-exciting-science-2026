"""Default-off metadata-only automatic client of the sole recovery coordinator."""
from collections import deque
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import time

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
from .recovery_core import NativePairs,ConsumerProgress,native_graph,bounded_json
from .perception_recovery import read_artifact,PRODUCER,PRODUCER_STATUS,CONSUMERS
from .rocm_rgbd import extrinsic_matrix
from .supervisor_policy import SupervisorPolicy,transient_fault
from .supervisor_journal import SupervisorJournal

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
            self.rpc={};self.reset_pending=False
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.rpc_clients={name:self.create_client(Trigger,name) for name in
                (COORDINATOR+'/start_recovery',COORDINATOR+'/recovery_status',
                 *(consumer+'/derived_source_status' for consumer in CONSUMERS))}
            self.device_client=self.create_client(DeviceInfo,self.pin['source']['camera_node']+'/device_info')
            self.subscriptions_=[self.create_subscription(String,PRODUCER_STATUS,self._producer,qos_profile_sensor_data),
                self.create_subscription(String,LINEAGE_TOPIC,self._lineage,qos_profile_sensor_data)]
            for kind in ('depth','color'):
                self.subscriptions_.append(self.create_subscription(CameraInfo,TOPICS[kind+'_info'],self._info(kind),qos_profile_sensor_data))
            for name,callback in (('enable',self._enable),('disable',self._disable),('status',self._status),('reset_fault',self._reset_fault)):
                self.create_service(Trigger,'~/'+name,callback)
            self.publisher=self.create_publisher(String,'~/status',1)
            self.timer=self.create_timer(.2,self._tick)
            self._persist()
        except BaseException:
            self.journal.close();raise

    def _persist(self):
        self.policy.clocks(time.monotonic(),time.time())
        self.journal.write(self.policy.journal(),consumer_epochs=self.epochs,adopted=self.adopted)

    def _fail(self,error):
        self.reset_pending=False
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
            self.ledger.entries.clear();self.health_error=str(error)[:512]
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
        require(0<=time.monotonic()-receipt<=.250,'producer heartbeat expired/future')
        return value

    def _request(self,name,timeout):
        if name in self.rpc:return
        client=self.rpc_clients[name]
        require(client.service_is_ready(),'metadata service unavailable: '+name)
        started=time.monotonic()
        if name==COORDINATOR+'/start_recovery':self._native_final()
        future=client.call_async(Trigger.Request())
        record=dict(future=future,started=started,deadline=started+timeout,received=None)
        future.add_done_callback(lambda _,r=record:r.update(received=time.monotonic()))
        self.rpc[name]=record

    def _take(self,name):
        record=self.rpc.get(name)
        if record is None:return None
        future=record['future'];now=time.monotonic()
        if now>record['deadline']:
            try:self.rpc_clients[name].remove_pending_request(future)
            except (KeyError,RuntimeError):pass
            future.cancel();del self.rpc[name]
            raise TimeoutError('metadata RPC expired/unknown: '+name)
        if not future.done():return None
        response=future.result();require(response is not None and record['received'] is not None and
            record['started']<=record['received']<=now<=record['deadline'],'metadata response clock invalid/late')
        value=parse_lineage(response.message)
        require(time.monotonic()<=record['deadline'],'metadata materialization expired')
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

    def _current_health(self):
        status=self._producer_now();require(status.get('state')=='ready' and status.get('fault') is None and
            status['producer_generation']==self.pin['producer_generation'],'current producer not healthy')
        self._statement();ready=True
        for consumer in CONSUMERS:
            endpoint=consumer+'/derived_source_status'
            result=self._take(endpoint)
            if result is None:
                if endpoint not in self.rpc:self._request(endpoint,2.)
                ready=False;continue
            success,proof,receipt=result
            require(0<=time.monotonic()-receipt<=.250,'consumer RPC receipt expired/future')
            require(proof.get('source_pin_sha256')==pin_digest(self.pin) and
                proof.get('producer_generation')==self.pin['producer_generation'],'consumer source pin/generation changed')
            epoch=proof.get('consumer_epoch');require(hex_value(epoch,32) and epoch!='0'*32,'consumer epoch missing')
            if consumer in self.epochs:require(self.epochs[consumer]==epoch,'unexpected external consumer epoch')
            if not success or proof.get('state')!='ready':ready=False;self.health_progress.pop(consumer,None);continue
            value=self._statement(proof)
            progress=self.health_progress.get(consumer)
            if progress is None:progress=ConsumerProgress('0'*32,self.pin,start_ns=0);self.health_progress[consumer]=progress
            progress.observe(proof,value,now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
            self.epochs[consumer]=epoch
            ready=ready and progress.complete
            self._request(endpoint,2.)
        return ready

    def _native_proof(self):
        now=time.monotonic();require(now<=self.native_started+2.,'postfault native preflight2s expired')
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
        self._native_final()
        return True

    def _native_final(self):
        # Scalars from the ORIGINAL proof, checked after all graph/TF/result
        # accesses and again after service readiness immediately before RPC.
        witness=self.native_witness;now=time.monotonic();ns=self.get_clock().now().nanoseconds
        require(0<=now-witness['started']<=2.,'postfault native preflight2s expired')
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
        self.policy.outage(fault,time.monotonic());self.native_started=time.monotonic()
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
        self.pin=deepcopy(pin);self.epochs=epochs;self.ledger=ledger;self.health_progress={}
        self.adopted=dict(job_id=self.policy.job['job_id'],artifact_sha256=digest,producer_generation=pin['producer_generation'])
        self.policy.completed(time.monotonic());self.native=None;self._persist()

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

    def _step(self):
        if self.closed:return
        try:
            self.policy.clocks(time.monotonic(),time.time())
            if self.reset_pending:self._finish_reset();return
            if self.policy.latched:return
            if self.policy.job is not None:self._monitor_job();return
            if not self.policy.enabled:return
            if self.policy.state=='verifying_native':
                try:self._dispatch()
                except (NativeProofPending,TransformException) as error:
                    if self.policy.job is None and time.monotonic()<=self.native_started+2.:
                        self.health_error=str(error)[:512];return
                    raise
                return
            status=self._producer_now()
            if status.get('state')=='unavailable':
                if self.policy.state=='armed':self._begin_outage(status)
                else:self.health_error='unavailable producer is not a new armed outage';self.policy.healthy(time.monotonic(),False)
                return
            try:valid=self._current_health();self.health_error=None if valid else 'waiting for two current consumer admissions'
            except (ValueError,TimeoutError,TransformException) as error:
                if any(word in str(error) for word in ('changed','unexpected','generation','interleaved','clock invalid','future')):raise
                self.health_error=str(error)[:512];valid=False
            self.policy.healthy(time.monotonic(),valid)
        except Exception as error:self._fail(error)

    def _enable(self,request,response):
        try:
            self._baseline_unchanged();self._graph();self._producer_now()
            self.policy.clocks(time.monotonic(),time.time());self.policy.enable(time.monotonic());self._persist()
            response.success=True
        except Exception as error:self._fail(error);response.success=False
        response.message=bounded_json(self._snapshot());return response

    def _disable(self,request,response):
        self.policy.disable(time.monotonic())
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
        # API acceptance requests a bounded terminal-state check; never clears
        # an unknown job, rewinds quota or implicitly enables supervision.
        try:
            require(self.policy.job is None and not self.reset_pending,'active/unknown job or reset check pending')
            self._request(COORDINATOR+'/recovery_status',2.);self.reset_pending=True
            response.success=True
        except Exception as error:response.success=False;self.health_error=str(error)[:512]
        response.message=bounded_json(dict(self._snapshot(),reset_check_accepted=response.success));return response

    def _snapshot(self):
        return dict(self.policy.status(),reset_check_pending=self.reset_pending,health_reason=self.health_error,current_producer_generation=self.pin['producer_generation'],
                    current_pin_sha256=pin_digest(self.pin),scope='perception metadata admission only; never motion or physical qualification')

    def _status(self,request,response):
        response.success=True;response.message=bounded_json(self._snapshot());return response

    def close(self):
        self.closed=True
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
