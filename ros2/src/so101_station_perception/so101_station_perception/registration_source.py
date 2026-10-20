"""One-shot runtime source pins: subscriptions, DeviceInfo, and read-only files."""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo,JointState,PointCloud2
from std_msgs.msg import String
from realsense2_camera_msgs.srv import DeviceInfo
from rosidl_runtime_py.convert import message_to_ordereddict

from .registration_monitor import bounded_bytes,read_owner,stamp_ns,transform
from . import derived_lineage as lineage_module
from .derived_lineage import (LineageBuffer,TOPICS,CLOUD_TOPIC,LINEAGE_TOPIC,SCHEMA,
    validate_graph,validate_cloud,validate_camera,configure_ros_domain)
from so101_station_perception_tag_tracking.registration_monitor import (
    StationaryHistory,owner_acquisition,owner_eligibility,make_source_cohort)


def gid_bytes(info):
    value=info.get('publisher_gid',b'') if isinstance(info,dict) else getattr(info,'publisher_gid',b'')
    return bytes(value)


def same_gid(a,b):
    """Accommodate only zero padding of the native RMW GID storage, not identity."""
    a,b=bytes(a),bytes(b)
    return bool(a and b) and (a==b or
        (min(len(a),len(b))>=16 and max(len(a),len(b))<=24 and
         a[:min(len(a),len(b))]==b[:min(len(a),len(b))] and
         not any((a if len(a)>len(b) else b)[min(len(a),len(b)):])) )


def select_identity_bracket(reads,stamp,joints,cohort,*,now):
    """Pin identities even when a measured failure state correctly forbids fitting."""
    valid=[];reason='owner acquisition bracket unavailable'
    for item in reads:
        try:valid.append((*owner_acquisition(item,joints,cohort,now=now),item))
        except (KeyError,TypeError,ValueError,OverflowError) as error:reason=str(error)
    before=[v for v in valid if v[0]<=stamp];after=[v for v in valid if v[0]>=stamp]
    if not before or not after:raise ValueError(reason)
    a,b=max(before,key=lambda v:v[0]),min(after,key=lambda v:v[0])
    if stamp-a[0]>.1 or b[0]-stamp>.1 or a[1]['raw']!=b[1]['raw']:
        raise ValueError('actual owner exposure bracket moving/unpaired')
    return a[2],b[2]


class SourceCapture(Node):
    def __init__(self,options):
        super().__init__('registration_capture_sources')
        self.options=options;self.clouds=deque(maxlen=12);self.infos=deque(maxlen=24)
        self.boards=deque(maxlen=16);self.history=StationaryHistory();self.description=None;self.model_changes=0
        self.device_future=None;self.device_client=None;self.device_service=None;self.endpoint=None
        self.device_request_gid=None;self.source_invalid_reason=None
        self.derived=LineageBuffer() if getattr(options,'allow_derived_source',False) else None
        self.derived_infos={kind:deque(maxlen=24) for kind in ('depth','color')} if self.derived else None
        self.device_request_source=None
        self.derived_validator_sha256=hashlib.sha256(Path(lineage_module.__file__).read_bytes()).hexdigest() if self.derived else None
        if self.derived and (options.cloud_topic!=CLOUD_TOPIC or options.camera_info_topic!=TOPICS['depth_info'] or options.lineage_topic!=LINEAGE_TOPIC):
            raise ValueError('explicit derived source requires canonical private cloud/lineage and native depth CameraInfo topics')
        from tf2_ros import Buffer,TransformListener
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(PointCloud2,options.cloud_topic,self._cloud,qos_profile_sensor_data)
        if self.derived:
            self.create_subscription(CameraInfo,options.camera_info_topic,self._depth_info,qos_profile_sensor_data)
            self.create_subscription(CameraInfo,TOPICS['color_info'],self._color_info,qos_profile_sensor_data)
            self.create_subscription(String,options.lineage_topic,self._lineage,qos_profile_sensor_data)
        else:
            self.create_subscription(CameraInfo,options.camera_info_topic,lambda message:self.infos.append(message),qos_profile_sensor_data)
        self.create_subscription(JointState,'/follower/joint_states',self._joint,qos_profile_sensor_data)
        self.create_subscription(String,'/so101_perception/charuco_world/status',self._board,10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/follower/robot_description',self._description,qos)

    def _description(self,message):
        if len(message.data)<=2_000_000 and message.data!=self.description:
            self.description=message.data;self.model_changes+=1

    def _lineage(self,message,info):
        try:
            self.derived.add(message.data,gid_bytes(info),now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        except ValueError:
            if self.derived.invalid_reason:
                self.source_invalid_reason=self.derived.invalid_reason;self.clouds.clear()

    def _native_info(self,kind,message,info):
        try:
            gid=gid_bytes(info);self.derived.observe_gid(kind+'_info',gid)
            self.derived_infos[kind].append((message,gid))
            if kind=='depth':self.infos.append(message)
        except ValueError as error:
            self.source_invalid_reason=str(error);self.clouds.clear()

    def _depth_info(self,message,info):self._native_info('depth',message,info)
    def _color_info(self,message,info):self._native_info('color',message,info)

    def _derived_pair(self,cloud,gid):
        value,status_gid,receipt=self.derived.match(stamp_ns(cloud),now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        validate_cloud(value,cloud,gid)
        native={}
        for kind in ('depth','color'):
            matches=[message for message,info_gid in self.derived_infos[kind]
                     if stamp_ns(message)==value['original'][kind+'_info_stamp_ns'] and
                     same_gid(info_gid,bytes.fromhex(value['source'][kind+'_info_publisher_gid']))]
            if not matches:raise ValueError('exact derived native CameraInfo cohort unavailable')
            native[kind]=matches[-1]
        return value,status_gid,receipt,native

    def _derived_generation(self,selected_gid=None):
        if self.source_invalid_reason:raise ValueError(self.source_invalid_reason)
        if not self.derived.entries:raise ValueError('fresh derived per-cohort lineage unavailable')
        value,status_gid,receipt=self.derived.entries[-1]
        self.derived.match(value['original']['depth_stamp_ns'],now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        try:
            snapshot=validate_graph(value,self.get_publishers_info_by_topic,cloud_gid=selected_gid,lineage_gid=status_gid)
            if self.device_request_source is not None and snapshot!=self.device_request_source:
                raise ValueError('native camera generation changed during DeviceInfo')
        except ValueError as error:
            self.source_invalid_reason=str(error);self.clouds.clear();raise
        self.derived.match(value['original']['depth_stamp_ns'],now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        return snapshot

    def _cloud(self,message,info):
        gid=gid_bytes(info)
        if getattr(self,'derived',None) is not None:
            try:self.derived.observe_gid('cloud',gid)
            except ValueError as error:self.source_invalid_reason=str(error);self.clouds.clear();return
            if self.source_invalid_reason is None:self.clouds.append((message,gid))
            return
        if self.device_request_gid is not None and not same_gid(self.device_request_gid,gid):
            self.source_invalid_reason='camera publisher generation changed after DeviceInfo binding'
            self.clouds.clear()
        if self.source_invalid_reason is None:self.clouds.append((message,gid))

    def _joint(self,message):
        if len(message.name)==len(message.position) and len(set(message.name))==len(message.name):
            self.history.add(stamp_ns(message)*1e-9,dict(zip(message.name,message.position)),
                             now=self.get_clock().now().nanoseconds*1e-9)

    def _board(self,message):
        try:
            if len(message.data)>16384:return
            value=json.loads(message.data)
            if isinstance(value,dict):self.boards.append(value)
        except (TypeError,ValueError):pass

    def _validate_generation(self,selected_gid=None):
        if getattr(self,'derived',None) is not None:return self._derived_generation(selected_gid)
        if self.source_invalid_reason:raise ValueError(self.source_invalid_reason)
        endpoints=self.get_publishers_info_by_topic(self.options.cloud_topic)
        if (len(endpoints)!=1 or self.device_request_gid is None or
                not same_gid(endpoints[0].endpoint_gid,self.device_request_gid) or
                endpoints[0].node_name!=self.endpoint.node_name or
                endpoints[0].node_namespace!=self.endpoint.node_namespace or
                (selected_gid is not None and not same_gid(selected_gid,self.device_request_gid))):
            self.source_invalid_reason='camera publisher generation changed during DeviceInfo capture'
            self.clouds.clear()
            raise ValueError(self.source_invalid_reason)

    def _open_device_client(self,service):
        self.device_client=self.create_client(DeviceInfo,service)

    def _device_request(self):
        if self.derived is not None:
            if not self.derived.entries or not self.clouds:return
            snapshot=self._derived_generation(self.clouds[-1][1])
            if self.device_client is None:
                self.device_request_source=snapshot
                self.endpoint=self.get_publishers_info_by_topic(TOPICS['depth'])[0]
                self.device_request_gid=bytes(self.endpoint.endpoint_gid)
                self.device_service=snapshot['node']+'/device_info'
                self._open_device_client(self.device_service)
            if self.device_future is None and self.device_client.service_is_ready():
                self.device_future=self.device_client.call_async(DeviceInfo.Request())
            return
        if self.device_request_gid is not None:self._validate_generation()
        if self.device_future is not None or not self.clouds:return
        endpoints=self.get_publishers_info_by_topic(self.options.cloud_topic)
        if len(endpoints)!=1:return
        endpoint=endpoints[0]
        if not same_gid(endpoint.endpoint_gid,self.clouds[-1][1]):return
        node='/'.join((endpoint.node_namespace.rstrip('/'),endpoint.node_name)).rstrip('/')
        service=node+'/device_info'
        if self.device_client is None:
            self.device_service=service;self.endpoint=endpoint
            self.device_request_gid=bytes(endpoint.endpoint_gid)
            self._open_device_client(service)
        if self.device_service!=service:raise ValueError('camera publisher changed while preparing DeviceInfo')
        if self.device_client.service_is_ready():self.device_future=self.device_client.call_async(DeviceInfo.Request())

    def capture(self):
        options=self.options
        config_raw=bounded_bytes(options.station_config);config=json.loads(config_raw)
        calibration_path=str(Path(config['arm']['calibration']['path']).expanduser().resolve())
        calibration_raw=bounded_bytes(calibration_path);base_raw=bounded_bytes(options.base_registration)
        initial=read_owner(options.owner_state_file);identity=initial['state']
        cohort={k:identity[k] for k in ('owner_session_id','arm_id','calibration_sha256','owner_source_sha256')}
        reads=deque(maxlen=12);started=time.monotonic();next_read=started+.4
        deadline=started+min(4.,max(1.2,options.timeout))
        def sample_owner():
            nonlocal next_read
            now=time.monotonic()
            if next_read<=now<deadline:
                reads.append(read_owner(options.owner_state_file));next_read=now+.05
        while time.monotonic()-started<1.15:
            self._device_request();sample_owner()
            rclpy.spin_once(self,timeout_sec=.008)
        while (self.description is None or self.device_future is None or not self.device_future.done()) and time.monotonic()<deadline:
            self._device_request();sample_owner();rclpy.spin_once(self,timeout_sec=.008)
        if self.description is None:raise ValueError('actual robot description unavailable')
        if self.device_future is None or not self.device_future.done():raise ValueError('DeviceInfo for actual unique cloud publisher unavailable')
        self._validate_generation()
        response=self.device_future.result()
        self._validate_generation()
        if response is None:raise ValueError('DeviceInfo returned no camera identity')
        device=dict(message_to_ordereddict(response));reason='no synchronized current source cohort'
        while time.monotonic()<deadline:
            sample_owner()
            if self.source_invalid_reason:raise ValueError(self.source_invalid_reason)
            candidates=tuple(self.clouds)[-1:] if self.derived is not None else reversed(self.clouds)
            for cloud,gid in candidates:
                try:
                    now=self.get_clock().now().nanoseconds*1e-9;stamp=stamp_ns(cloud)*1e-9
                    if abs(now-time.time())>.05:raise ValueError('ROS and owner UTC clock domains differ')
                    if self.model_changes!=1:raise ValueError('model changed during source capture; recapture required')
                    joint_stamp,joints=self.history.select(stamp,now=now)
                    if self.derived is not None:
                        lineage,status_gid,lineage_receipt,native=self._derived_pair(cloud,gid)
                        info=native['depth']
                    else:info=min(self.infos,key=lambda m:abs(stamp_ns(m)*1e-9-stamp))
                    if info.header.frame_id!=cloud.header.frame_id or abs(stamp_ns(info)*1e-9-stamp)>.05 or stamp_ns(info)*1e-9>now:
                        raise ValueError('original cloud and native CameraInfo not paired')
                    k=np.asarray(info.k,dtype=float).reshape(3,3)
                    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0 or not 1<=info.width<=1280 or not 1<=info.height<=720:
                        raise ValueError('finite supported native CameraInfo required')
                    before,after=select_identity_bracket(reads,stamp,joints,cohort,now=time.time())
                    frame=cloud.header.frame_id;ns=stamp_ns(cloud);tf_deadline=min(deadline,time.monotonic()+.3)
                    while not self.buffer.can_transform('charuco_board','base_link',Time(nanoseconds=ns)) or not self.buffer.can_transform('base_link',frame,Time(nanoseconds=ns)):
                        if time.monotonic()>=tf_deadline:raise ValueError('exact cloud-stamp TF unavailable; never substituting latest')
                        sample_owner();rclpy.spin_once(self,timeout_sec=.008)
                    world_base=transform(self.buffer.lookup_transform('charuco_board','base_link',Time(nanoseconds=ns)))
                    base_camera=self.buffer.lookup_transform('base_link',frame,Time(nanoseconds=ns))
                    self._validate_generation(gid)
                    if self.derived is not None:
                        lineage,status_gid,lineage_receipt,native=self._derived_pair(cloud,gid)
                        color_from_depth=transform(self.buffer.lookup_transform(lineage['original']['color_frame'],frame,Time(nanoseconds=ns)))
                        validate_camera(lineage,native['depth'],native['color'],color_from_depth)
                        if device.get('serial_number')!=lineage['source']['serial_number']:
                            raise ValueError('actual camera DeviceInfo differs from derived lineage serial')
                    final=read_owner(options.owner_state_file)
                    for key,value in cohort.items():
                        if final['state'].get(key)!=value:raise ValueError('owner identity/generation changed during capture')
                    if final['state']['raw']!=before['state']['raw']:raise ValueError('owner moved during source capture')
                    for path,raw in ((options.station_config,config_raw),(calibration_path,calibration_raw),(options.base_registration,base_raw)):
                        if bounded_bytes(path)!=raw:raise ValueError('source file changed during capture')
                    result=make_source_cohort(self.description,before['state'],device,gid.hex(),
                        station_config_path=options.station_config,station_config_bytes=config_raw,
                        calibration_path=calibration_path,calibration_bytes=calibration_raw,
                        base_registration_path=options.base_registration,base_registration_bytes=base_raw,
                        world_from_base=world_base)
                    faults=[fault for fault in (owner_eligibility(before['state']),owner_eligibility(after['state']),owner_eligibility(final['state'])) if fault]
                    board=min(self.boards,key=lambda b:abs(float(b.get('stamp',-1e30))-stamp)) if self.boards else None
                    if (board is None or board.get('state')!='tracking' or board.get('observation_valid') is not True or
                            board.get('plane_source')!='aligned_depth' or not 0<=now-float(board.get('stamp',-1e30))<=.5 or abs(float(board['stamp'])-stamp)>.05):
                        faults.append('fresh paired depth-backed board unavailable')
                    faults=list(dict.fromkeys(faults));result['fit_eligibility_at_capture']={'eligible':not faults,'reason':faults[0] if faults else None,'reasons':faults}
                    result['capture_stamp_ns']=ns
                    evidence={'schema':'so101-registration-source-evidence-v1','qualified':False,
                        'capture_stamp_ns':ns,'joint_stamp':joint_stamp,'cloud_frame':frame,'device_info':device,
                        'device_info_service':self.device_service,'device_info_publisher_gid':self.device_request_gid.hex(),'cloud_publisher_node':{'name':self.endpoint.node_name,'namespace':self.endpoint.node_namespace},
                        'original_cloud':{'publisher_gid':gid.hex(),'width':cloud.width,'height':cloud.height,'point_step':cloud.point_step,'row_step':cloud.row_step,'data_bytes':len(cloud.data)},
                        'camera_info':dict(message_to_ordereddict(info)),'owner_before':before,'owner_after':after,'owner_final':final,
                        'owner_reads':len(reads),'board':board,'base_from_camera':dict(message_to_ordereddict(base_camera)),
                        'world_from_base':world_base.tolist(),'duration_s':time.monotonic()-started,
                        'source_capture_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        'fit_eligibility_at_capture':result['fit_eligibility_at_capture']}
                    if self.derived is not None:
                        self.derived.match(ns,now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
                        result['derived_source']={'schema':SCHEMA,'qualified':False,'local_provenance_not_cryptographic':True,'pin':self.derived.pin}
                        evidence['derived_validator_sha256']=self.derived_validator_sha256
                        evidence['derived_lineage']={'statement':lineage,'received_monotonic':lineage_receipt,'publisher_gid':status_gid,
                            'native_device_info_request_source':self.device_request_source}
                        endpoint=self.get_publishers_info_by_topic(options.cloud_topic)[0]
                        evidence['cloud_publisher_node']={'name':endpoint.node_name,'namespace':endpoint.node_namespace}
                    return result,evidence
                except (ValueError,KeyError,TypeError) as error:
                    if self.source_invalid_reason:raise ValueError(self.source_invalid_reason) from error
                    reason=str(error)
            sample_owner();rclpy.spin_once(self,timeout_sec=.008)
        raise ValueError(reason)


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    home=Path.home()/'.local/share/exciting-science/station'
    parser.add_argument('--output',required=True)
    parser.add_argument('--station-config',default=str(home/'station.json'))
    parser.add_argument('--base-registration',default=str(home/'calibration/robot_base_registration.json'))
    parser.add_argument('--owner-state-file',default='/dev/shm/so101-sorting-20261016/rviz-arm-state.json')
    parser.add_argument('--cloud-topic',default='/camera/depth/color/points')
    parser.add_argument('--camera-info-topic',default='/camera/depth/camera_info')
    parser.add_argument('--timeout',type=float,default=4.)
    parser.add_argument('--allow-derived-source',action='store_true',default=False)
    parser.add_argument('--lineage-topic',default=LINEAGE_TOPIC)
    options=parser.parse_args(args)
    for key in ('output','station_config','base_registration','owner_state_file'):
        setattr(options,key,str(Path(getattr(options,key)).expanduser().resolve()))
    configure_ros_domain(os.environ)
    rclpy.init();node=None
    try:
        node=SourceCapture(options);cohort,evidence=node.capture();output=Path(options.output)
        output.parent.mkdir(parents=True,exist_ok=True)
        evidence_path=output.with_name(output.stem+'.evidence.json')
        with evidence_path.open('x') as stream:json.dump(evidence,stream,indent=2,allow_nan=False)
        with output.open('x') as stream:json.dump(cohort,stream,indent=2,allow_nan=False)
        print(json.dumps({'state':'source_pins_prepared','output':str(output),'evidence':str(evidence_path),
                          'model_sha256':cohort['model_sha256'],'camera_serial':cohort['camera_serial'],
                          'cloud_publisher_gid':cohort['cloud_publisher_gid'],
                          'fit_eligibility_at_capture':cohort['fit_eligibility_at_capture']},allow_nan=False))
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
