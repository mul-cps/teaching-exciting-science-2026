"""Explicit metadata-only derived camera pin capture; no physical qualification."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from realsense2_camera_msgs.srv import DeviceInfo
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer,TransformListener,TransformException

from . import camera_source_pin,derived_lineage,diagnostic_cloud
from .camera_source_pin import CameraPins,artifact,publish_exclusive
from .derived_lineage import TOPICS,LINEAGE_TOPIC,configure_ros_domain,require,validate_graph,validate_statement
from .derived_aligned import received_gid
from .diagnostic_cloud import quaternion_matrix


class CameraSourceCapture(Node):
    def __init__(self,*,serial='243322071783',context=None,use_global_arguments=True):
        super().__init__('camera_capture_sources',context=context,use_global_arguments=use_global_arguments)
        self.serial=serial;self.core=CameraPins();self.last_error='two advancing original source pairs unavailable'
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.client=None;self.future=None;self.rpc_snapshot=None;self.rpc_started=None;self.identity=None;self.graph_bound=False
        self.source_hashes={Path(module.__file__).name:hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
            for module in (sys.modules[__name__],camera_source_pin,derived_lineage,diagnostic_cloud)}
        self.create_subscription(String,LINEAGE_TOPIC,self._lineage,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,TOPICS['depth_info'],self._depth_info,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,TOPICS['color_info'],self._color_info,qos_profile_sensor_data)

    def _clocks(self):return {'now_ns':self.get_clock().now().nanoseconds,'now_mono':time.monotonic()}

    def _fault(self,error):
        self.last_error=str(error);self.core.ledger.entries.clear()

    def _lineage(self,message,info):
        try:self.core.lineage(message.data,received_gid(info),**self._clocks())
        except (ValueError,TypeError,KeyError) as error:self._fault(error)

    def _info(self,kind,message,info):
        try:self.core.info(kind,message,received_gid(info),**self._clocks())
        except (ValueError,TypeError,KeyError) as error:self._fault(error)

    def _depth_info(self,message,info):self._info('depth',message,info)
    def _color_info(self,message,info):self._info('color',message,info)

    def _graph(self,value,gid):
        missing=False
        def lookup(topic):
            nonlocal missing
            endpoints=self.get_publishers_info_by_topic(topic)
            missing=missing or not endpoints
            return endpoints
        try:snapshot=validate_graph(value,lookup,lineage_gid=gid)
        except ValueError as error:
            # An initial undiscovered endpoint is not a publisher replacement.
            # Once bound, loss also invalidates; observed wrong/ambiguous GIDs
            # always latch even before the independent RPC is ready.
            if self.graph_bound or not missing:self.core.invalidate(str(error))
            raise
        self.graph_bound=True
        if self.rpc_snapshot is not None and snapshot!=self.rpc_snapshot:
            self.core.invalidate('native camera generation changed during independent DeviceInfo')
        return snapshot

    def _device(self):
        if not self.core.ledger.entries:return
        value,gid,receipt=self.core.ledger.entries[-1]
        validate_statement(value,receipt_mono=receipt,**self._clocks())
        snapshot=self._graph(value,gid)
        if self.client is None:self.client=self.create_client(DeviceInfo,snapshot['node']+'/device_info')
        if self.future is None:
            if not self.client.service_is_ready():raise ValueError('native DeviceInfo service not ready')
            # Bind exactly the endpoint generation present at the actual RPC.
            self.rpc_snapshot=deepcopy(snapshot);self.rpc_started=time.monotonic()
            self.future=self.client.call_async(DeviceInfo.Request())
        if self.identity is None and self.future.done():
            self._graph(value,gid)
            response=self.future.result()
            require(response is not None,'native DeviceInfo returned no identity')
            require(response.serial_number==self.serial==value['source']['serial_number'],'independent native DeviceInfo serial differs from explicit expected/derived serial')
            self.identity={'serial_number':response.serial_number,'source_snapshot':deepcopy(self.rpc_snapshot),
                'request_started_monotonic':self.rpc_started,'confirmed_monotonic':time.monotonic(),
                'confirmed_ros_ns':self.get_clock().now().nanoseconds,'service':snapshot['node']+'/device_info',
                'response':dict(message_to_ordereddict(response))}
            self._graph(value,gid)

    def _matrix(self,selected):
        original=selected['statement']['original']
        tf=self.buffer.lookup_transform(original['color_frame'],original['depth_frame'],
            Time(nanoseconds=original['depth_stamp_ns']))
        t,q=tf.transform.translation,tf.transform.rotation
        matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w);matrix[:3,3]=[t.x,t.y,t.z]
        return matrix

    def verify(self,selected,deadline):
        require(time.monotonic()<=deadline,'camera source capture deadline expired')
        matrix=self._matrix(selected)
        self.core.verify(selected,lookup=self.get_publishers_info_by_topic,color_from_depth=matrix,identity=self.identity,**self._clocks())
        # Graph/TF work cannot refresh original leases.
        self.core.check_leases(selected,self.identity,**self._clocks())
        require(time.monotonic()<=deadline,'camera source capture deadline expired')

    def capture(self,*,timeout=4.,output=None,executor=None):
        require(type(timeout) in (int,float) and math.isfinite(timeout) and 0<timeout<=4.,'timeout must be finite in (0,4] seconds')
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.core.ledger.invalid_reason:raise ValueError(self.core.ledger.invalid_reason)
            wait=min(.005,max(0.,deadline-time.monotonic()))
            if executor is None:rclpy.spin_once(self,timeout_sec=wait)
            else:executor.spin_once(timeout_sec=wait)
            try:
                self._device()
                if self.identity is None:continue
                selected=self.core.select(**self._clocks());self.verify(selected,deadline)
                result=artifact(selected,self.identity,source_hashes=self.source_hashes)
                if output is not None:
                    path=Path(output);evidence=path.with_name(path.stem+'.evidence.json')
                    # Complete evidence is linked first, artifact last; neither
                    # can overwrite an existing operator path or contain partial JSON.
                    publish_exclusive({evidence:{'schema':'so101-camera-source-evidence-v1','qualified':False,**result['camera_evidence']},
                                       path:result},check=lambda:self.verify(selected,deadline))
                else:
                    json.dumps(result,allow_nan=False);self.verify(selected,deadline)
                return result
            except (ValueError,TypeError,KeyError,TransformException) as error:
                self.last_error=str(error)
                if self.core.ledger.invalid_reason or self.identity is not None and time.monotonic()>=self.rpc_started+1.:raise
        raise ValueError('camera source capture timed out: '+self.last_error)


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--timeout',type=float,default=4.)
    parser.add_argument('--serial',default='243322071783')
    options=parser.parse_args(args)
    require(math.isfinite(options.timeout) and 0<options.timeout<=4.,'timeout must be finite in (0,4] seconds')
    require(0<len(options.serial)<=128,'explicit expected camera serial required')
    configure_ros_domain(os.environ)
    output=Path(options.output).expanduser().resolve();sidecar=output.with_name(output.stem+'.evidence.json')
    if output.exists() or sidecar.exists():raise FileExistsError('camera source artifact/evidence already exists')
    rclpy.init(args=[]);node=None
    try:
        node=CameraSourceCapture(serial=options.serial);result=node.capture(timeout=options.timeout,output=output)
        print(json.dumps({'state':'camera_source_pins_prepared','schema':result['schema'],'qualified':False,
            'output':str(output),'evidence':str(sidecar),'camera_serial':options.serial,
            'sequence':result['camera_evidence']['lineage']['statement']['sequence']}))
    except BaseException as error:
        failed=output.with_name(output.stem+'.failed.json')
        try:publish_exclusive({failed:{'schema':'so101-camera-source-failure-v1','qualified':False,'reason':str(error)}},check=lambda:None)
        except (OSError,ValueError):pass
        raise
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
