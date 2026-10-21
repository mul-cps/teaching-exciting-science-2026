"""Optional ROS adapter for exact, pinned derived aligned-image cohorts.

Only metadata and references enter the collector. Direct camera subscriptions
are owned by callers and are unchanged when this adapter is not constructed.
"""
import json
import struct
from pathlib import Path
from threading import RLock
import time
import uuid

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .aligned_cohort import AlignedCohorts
from .derived_lineage import (TOPICS, ALIGNED_TOPIC, LINEAGE_TOPIC, SCHEMA, require,
    finite, statement_pin, validate_statement, validate_graph, validate_camera)
from .diagnostic_cloud import quaternion_matrix
from .admission_proof import AdmissionProof,pin_digest


def received_gid(info):
    return info.get("publisher_gid", b"") if isinstance(info,dict) else getattr(info,"publisher_gid",b"")


class DerivedAligned:
    def __init__(self,node,buffer,callback,*,callback_group=None,health_callback_group=None,period_s=0.,metadata_offer=None):
        if metadata_offer is not None and not callable(metadata_offer):
            raise TypeError('metadata_offer must be callable or None')
        self.metadata_offer=metadata_offer
        self.node=node;self.buffer=buffer;self.callback=callback
        self.lock=RLock();self.closed=False;self.fault=None
        self.period=max(.2,float(period_s)) if period_s else 0.
        self.last_take=0.;self.core=None
        require(node.get_parameter('depth_topic').value==ALIGNED_TOPIC,'explicit private aligned depth topic required')
        require(node.get_parameter('color_topic').value==TOPICS['color'],'native RGB topic required')
        require(node.get_parameter('camera_info_topic').value==TOPICS['color_info'],'native color CameraInfo required')
        self.reload()
        self.subscriptions=[]
        for kind,topic,typ in [('color',TOPICS['color'],Image),('aligned',ALIGNED_TOPIC,Image),
                              ('color_info',TOPICS['color_info'],CameraInfo),('depth_info',TOPICS['depth_info'],CameraInfo)]:
            self.subscriptions.append(node.create_subscription(typ,topic,self._receiver(kind),
                qos_profile_sensor_data,callback_group=callback_group))
        self.subscriptions.append(node.create_subscription(String,LINEAGE_TOPIC,self._lineage,
            qos_profile_sensor_data,callback_group=callback_group))
        self.reset_service=node.create_service(Trigger,'~/reset_derived_source',self._reset)
        self.status_service=node.create_service(Trigger,'~/derived_source_status',self._admission_status)
        self.health_service=node.create_service(Trigger,'~/derived_source_health',self._source_health,
                                                callback_group=health_callback_group)
        if self.metadata_offer is not None:
            self.timer=node.create_timer(1./30.,self.drain,callback_group=callback_group)
        else:
            self.timer=node.create_timer(.01,self.drain) if self.period else None

    def reload(self):
        path=str(self.node.get_parameter('derived_source_cohort').value)
        require(bool(path),'explicit derived_source_cohort required')
        with Path(path).open('rb') as stream:raw=stream.read(131073)
        require(len(raw)<=131072,'source artifact exceeds bounded size')
        source=json.loads(raw)['derived_source']
        require(source['schema']==SCHEMA and source['qualified'] is False,'diagnostic derived source artifact required')
        core=AlignedCohorts(source['pin'])
        with self.lock:
            if self.core is not None:self.core.close()
            self.core=core;self.fault=None;self.last_take=0.
            self.consumer_epoch=uuid.uuid4().hex
            self.source_pin_sha256=pin_digest(core.ledger.pin)
            self.last_proof=None
        self._invalidate_offers()

    def _invalidate_offers(self):
        if self.metadata_offer is not None:self.metadata_offer(None)

    def _reset(self,request,response):
        try:
            self.reload();response.success=True;response.message='Explicit source pins reloaded; waiting for new exact cohort'
        except (ValueError,KeyError,TypeError,OSError) as error:
            with self.lock:
                if self.core:self.core.close()
                self.fault=str(error)
            self._invalidate_offers()
            response.success=False;response.message=str(error)
        return response

    def _receiver(self,kind):
        def receive(message,info):
            self._intake(kind,message,received_gid(info))
        return receive

    def _lineage(self,message,info):
        self._intake('lineage',message,received_gid(info))

    def _intake(self,kind,message,gid):
        token=None;invalidated=False
        with self.lock:
            if self.closed:return
            try:
                clocks={'now_ns':self.node.get_clock().now().nanoseconds,'now_mono':time.monotonic()}
                if kind=='lineage':self.core.lineage(message.data,gid,**clocks)
                else:
                    if kind in ('depth_info','color_info'):self._validate_received_info(kind,message)
                    self.core.receive(kind,message,gid,**clocks)
                if not self.period and self.metadata_offer is None:token=self.core.take(**clocks)
            except (ValueError,KeyError,TypeError,AttributeError) as error:
                self.core.clear_observations();self.fault=str(error);invalidated=True
        if invalidated:self._invalidate_offers()
        # Marker callback only offers metadata to its existing latest scheduler.
        if token is not None:self.callback(token['color'],token['aligned'],token['color_info'],token)

    def drain(self):
        token=None;invalidated=False
        with self.lock:
            if self.closed:return
            if self.metadata_offer is None and time.monotonic()-self.last_take<self.period:return
            try:token=self.core.take(now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic())
            except (ValueError,KeyError,TypeError) as error:
                self.fault=str(error);invalidated=True
            if token is not None:self.last_take=time.monotonic()
        if invalidated:self._invalidate_offers()
        if token is not None:
            if self.metadata_offer is not None:self.metadata_offer(token)
            else:self.callback(token['color'],token['aligned'],token['color_info'],token)

    def token_is_current(self,token):
        """Epoch/pin fence only; never a replacement for check(token)."""
        with self.lock:
            return (not self.closed and not self.core.ledger.invalid_reason
                    and token.get('epoch') is self.core.epoch
                    and token['statement']['producer_generation']==self.core.ledger.pin['producer_generation'])

    def pop_fault(self):
        with self.lock:
            result=self.fault;self.fault=None;return result

    def check(self,token):
        # Exact timestamp lookup, nonblocking: unavailable TF rejects this cohort.
        value=token['statement'];stamp=token['aligned'].header.stamp
        try:
            tf=self.buffer.lookup_transform(value['original']['color_frame'],value['original']['depth_frame'],Time.from_msg(stamp))
            t,q=tf.transform.translation,tf.transform.rotation
            matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w);matrix[:3,3]=[t.x,t.y,t.z]
            with self.lock:
                self.core.check(token,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic(),
                    lookup=self.node.get_publishers_info_by_topic,color_from_depth=matrix)
                # Graph work may consume the remaining lease; never refresh it.
                from .derived_lineage import validate_statement
                validate_statement(value,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic(),receipt_mono=token['lineage_receipt'])
                proof=AdmissionProof.from_token(token)
                proof.check_leases(self.core,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic())
                self.last_proof=proof
        except Exception as error:
            # TF exceptions vary across ROS releases; all failures are unavailable.
            raise ValueError('derived aligned admission unavailable: '+str(error)) from error
        return True

    def _check_proof(self,proof):
        proof.check_leases(self.core,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic())
        original=proof.statement['original']
        tf=self.buffer.lookup_transform(original['color_frame'],original['depth_frame'],Time(nanoseconds=original['depth_stamp_ns']))
        t,q=tf.transform.translation,tf.transform.rotation
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w);matrix[:3,3]=[t.x,t.y,t.z]
        proof.check(self.core,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic(),
                    lookup=self.node.get_publishers_info_by_topic,color_from_depth=matrix)
        proof.check_leases(self.core,now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic())

    def _admission_status(self,request,response):
        # Cold RPC only. Retains scalar metadata and two small Info snapshots,
        # never another RGB/depth payload or renewed acquisition receipt.
        with self.lock:
            result={'schema':'so101-derived-admission-status-v1','qualified':False,
                    'execution_enabled':False,'state':'awaiting',
                    'consumer_epoch':self.consumer_epoch,
                    'source_pin_sha256':self.source_pin_sha256,
                    'producer_generation':self.core.ledger.pin['producer_generation']}
            proof=self.last_proof
            try:
                require(not self.closed and proof is not None,'waiting for checked source admission')
                self._check_proof(proof)
                result=proof.status(consumer_epoch=self.consumer_epoch,source_pin_sha256=self.source_pin_sha256,
                                    now_ns=self.node.get_clock().now().nanoseconds)
                message=json.dumps(result,allow_nan=False)
                self._check_proof(proof)
                response.success=True;response.message=message
            except Exception as error:
                result.update(state='unavailable',reason=str(error)[:512])
                response.success=False;response.message=json.dumps(result,allow_nan=False)
        return response

    def _validate_received_info(self,kind,message):
        # Info-only immutable metadata guard, including duplicate acquisitions.
        # The core still owns all high-water marks, receipts and buffer bounds.
        self.core._open()
        camera='depth' if kind=='depth_info' else 'color'
        try:
            pin=self.core.ledger.pin;cal=pin['calibration']
            expected=cal[camera+'_K']
            require(len(expected)==4 and len(message.k)==9
                    and len(message.d)==len(cal[camera+'_D'])<=8,'bounded native CameraInfo arrays required')
            k=[float(x) for x in message.k];d=[float(x) for x in message.d]
            frame=pin['outputs']['cloud_frame' if camera=='depth' else 'aligned_frame']
            valid=(message.header.frame_id==frame and [message.width,message.height]==cal[camera+'_size']
                and len(k)==9 and all(finite(x) and abs(x)<=100000 for x in k)
                and [k[1],k[3],*k[6:]]==[0.,0.,0.,0.,1.]
                and [struct.pack('f',x) for x in (k[0],k[4],k[2],k[5])]
                    ==[struct.pack('f',x) for x in expected]
                and len(d)==len(cal[camera+'_D']) and all(finite(x) and x==0 for x in d))
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError):valid=False
        if not valid:
            self.core.clear_observations()
            self.core.ledger.invalidate('native CameraInfo immutable calibration/profile changed; explicit repin/reset required')

    def _check_source_binding(self,entry,core,epoch):
        require(not self.closed and self.core is core and self.consumer_epoch==epoch,
                'source health predates close/reset')
        core._open()
        require(bool(core.ledger.entries) and core.ledger.entries[-1] is entry,
                'current newest source lineage unavailable/changed')
        value,gid,receipt=entry
        require(finite(receipt) and 0<=time.monotonic()-receipt<=.250,
                'original consumer lineage receipt expired/future')
        if statement_pin(value)!=core.ledger.pin:
            core.clear_observations();core.ledger.invalidate('source health immutable pin changed; explicit repin/reset required')
        validate_statement(value,now_ns=self.node.get_clock().now().nanoseconds,
                           now_mono=time.monotonic(),receipt_mono=receipt)
        records={}
        for kind in ('depth_info','color_info'):
            stamp=value['original'][kind+'_stamp_ns']
            record=next((item for item in reversed(core.streams[kind]) if item[3]==stamp),None)
            require(record is not None,'exact native CameraInfo unavailable; no nearest/fallback')
            require(finite(record[2]) and 0<=time.monotonic()-record[2]<=.250,
                    'original native CameraInfo receipt expired/future')
            records[kind]=record
        # TF lookup is exact and nonblocking. Missing TF is unavailable, not a contradiction.
        original=value['original']
        tf=self.buffer.lookup_transform(original['color_frame'],original['depth_frame'],Time(nanoseconds=original['depth_stamp_ns']))
        t,q=tf.transform.translation,tf.transform.rotation
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w);matrix[:3,3]=[t.x,t.y,t.z]
        try:
            for kind,record in records.items():
                camera='depth' if kind=='depth_info' else 'color'
                require(len(record[0].k)==9 and len(record[0].d)==len(value['calibration'][camera+'_D'])<=8,
                        'bounded exact native CameraInfo arrays required')
                core.ledger.observe_gid(kind,record[1])
            validate_graph(value,self.node.get_publishers_info_by_topic,lineage_gid=gid)
            validate_camera(value,records['depth_info'][0],records['color_info'][0],matrix)
        except (ValueError,TypeError,KeyError,AttributeError):
            core.clear_observations();core.ledger.invalidate('source health graph/calibration changed; explicit repin/reset required')
        require(not self.closed and self.core is core and self.consumer_epoch==epoch
                and bool(core.ledger.entries) and core.ledger.entries[-1] is entry,
                'current source health changed during access')
        now_ns,now_mono=self.node.get_clock().now().nanoseconds,time.monotonic()
        validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=receipt)
        require(finite(receipt) and 0<=now_mono-receipt<=.250,'original consumer lineage receipt expired/future')
        for record in records.values():
            require(finite(record[2]) and 0<=now_mono-record[2]<=.250,'original native CameraInfo receipt expired/future')
        return records

    def _info_diagnostics(self):
        # Caller holds lock. Scalar snapshots only: no messages retained/copied.
        core=self.core
        entry=core.ledger.entries[-1] if core.ledger.entries else None
        value=entry[0] if entry else {}
        original=value.get('original',{})
        result={'consumer_epoch':self.consumer_epoch,'latest_sequence':value.get('sequence'),
                'original_header_stamps_ns':{key:original.get(key) for key in
                    ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')},
                'streams':{}}
        for kind in ('depth_info','color_info'):
            target=original.get(kind+'_stamp_ns');records=core.streams[kind]
            stamps=[record[3] for record in records]
            receipts=[record[2] for record in records]
            if not stamps:classification='empty'
            elif target is None:classification='target_unavailable'
            elif target in stamps:classification='present'
            elif target>max(stamps):classification='not_yet'
            elif target<min(stamps):classification='older_than_oldest'
            else:classification='interior_missing'
            result['streams'][kind]={'target_stamp_ns':target,'classification':classification,
                'retained_count':len(records),'oldest_stamp_ns':min(stamps) if stamps else None,
                'newest_stamp_ns':max(stamps) if stamps else None,
                'first_unique_receipt_min_monotonic':min(receipts) if receipts else None,
                'first_unique_receipt_max_monotonic':max(receipts) if receipts else None}
        return result

    def _source_health(self,request,response):
        # Source binding is deliberately independent of selected images/analysis.
        # No token is taken or admitted and no image/progress proof is updated.
        with self.lock:
            core,epoch=self.core,self.consumer_epoch
            result={'schema':'so101-derived-source-binding-health-v1','state':'unavailable',
                'image_admitted':False,'processing_progress':False,'qualified':False,'execution_enabled':False,
                'binding_invalidated':bool(core.ledger.invalid_reason),
                'consumer_epoch':epoch,'source_pin_sha256':self.source_pin_sha256,
                'producer_generation':core.ledger.pin['producer_generation'],
                'camera_info_validation':'independent_native_info',
                'identity_validation':'producer_attested_device_info_no_new_rpc',
                'optical_tf_validation':'local_exact_depth_timestamp',
                'scope':'current source binding only; not image admission, analyzer progress or physical qualification'}
            try:
                require(bool(core.ledger.entries),'current newest source lineage unavailable')
                entry=core.ledger.entries[-1]
                records=self._check_source_binding(entry,core,epoch)
                value,_,receipt=entry;original=value['original']
                deadlines={'consumer_lineage':receipt+.250}
                deadlines.update({'source_'+k:r+.250 for k,r in value['health']['source_receipts_monotonic'].items()})
                deadlines.update({'consumer_'+k:r[2]+.250 for k,r in records.items()})
                receipt_expiry=min(deadlines.values());identity_expiry=value['identity']['expires_monotonic']
                headers={k:original[k] for k in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns')}
                result.update(state='ready',sequence=value['sequence'],depth_stamp_ns=original['depth_stamp_ns'],
                    color_stamp_ns=original['color_stamp_ns'],receipt_deadlines_monotonic=deadlines,
                    original_receipt_expiry_monotonic=receipt_expiry,
                    original_identity_expiry_monotonic=identity_expiry,
                    effective_health_expiry_monotonic=min(receipt_expiry,identity_expiry),
                    original_header_stamps_ns=headers,original_header_expiry_ros_ns=min(headers.values())+500_000_000,
                    checked_ros_ns=self.node.get_clock().now().nanoseconds,checked_monotonic=time.monotonic())
                message=json.dumps(result,allow_nan=False)
                self._check_source_binding(entry,core,epoch)
                response.success=True;response.message=message
            except Exception as error:
                result.update(state='unavailable',reason=str(error)[:512],
                    binding_invalidated=bool(core.ledger.invalid_reason or self.core is not core or self.consumer_epoch!=epoch))
                result['native_info_diagnostics']=self._info_diagnostics()
                response.success=False;response.message=json.dumps(result,allow_nan=False)
        return response

    def close(self):
        with self.lock:
            self.closed=True;self.core.close();self.last_proof=None
        if self.timer:self.timer.cancel()
        self._invalidate_offers()
