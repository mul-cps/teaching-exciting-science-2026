"""Optional ROS adapter for exact, pinned derived aligned-image cohorts.

Only metadata and references enter the collector. Direct camera subscriptions
are owned by callers and are unchanged when this adapter is not constructed.
"""
import json
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
from .derived_lineage import TOPICS, ALIGNED_TOPIC, LINEAGE_TOPIC, SCHEMA, require
from .diagnostic_cloud import quaternion_matrix
from .admission_proof import AdmissionProof,pin_digest


def received_gid(info):
    return info.get("publisher_gid", b"") if isinstance(info,dict) else getattr(info,"publisher_gid",b"")


class DerivedAligned:
    def __init__(self,node,buffer,callback,*,callback_group=None,period_s=0.):
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

    def _reset(self,request,response):
        try:
            self.reload();response.success=True;response.message='Explicit source pins reloaded; waiting for new exact cohort'
        except (ValueError,KeyError,TypeError,OSError) as error:
            with self.lock:
                if self.core:self.core.close()
                self.fault=str(error)
            response.success=False;response.message=str(error)
        return response

    def _receiver(self,kind):
        def receive(message,info):
            self._intake(kind,message,received_gid(info))
        return receive

    def _lineage(self,message,info):
        self._intake('lineage',message,received_gid(info))

    def _intake(self,kind,message,gid):
        token=None
        with self.lock:
            if self.closed:return
            try:
                clocks={'now_ns':self.node.get_clock().now().nanoseconds,'now_mono':time.monotonic()}
                if kind=='lineage':self.core.lineage(message.data,gid,**clocks)
                else:self.core.receive(kind,message,gid,**clocks)
                if not self.period:token=self.core.take(**clocks)
            except (ValueError,KeyError,TypeError,AttributeError) as error:
                self.core.clear_observations();self.fault=str(error)
        # Marker callback only offers metadata to its existing latest scheduler.
        if token is not None:self.callback(token['color'],token['aligned'],token['color_info'],token)

    def drain(self):
        token=None
        with self.lock:
            if self.closed or time.monotonic()-self.last_take<self.period:return
            try:token=self.core.take(now_ns=self.node.get_clock().now().nanoseconds,now_mono=time.monotonic())
            except (ValueError,KeyError,TypeError) as error:self.fault=str(error)
            if token is not None:self.last_take=time.monotonic()
        if token is not None:self.callback(token['color'],token['aligned'],token['color_info'],token)

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

    def close(self):
        with self.lock:
            self.closed=True;self.core.close();self.last_proof=None
        if self.timer:self.timer.cancel()
