"""Opt-in stationary registration proposals; no TF, parameter or motor writes."""
from collections import deque
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from ament_index_python.packages import get_package_share_directory

from so101_station_perception_tag_tracking.registration_monitor import (
    Deadline, MeshSamples, StationaryHistory, fit_registration, marker_residual,
    owner_bracket, owner_eligibility, rigid, visible_samples, validate_base_pin, ground_metrics)
import so101_station_perception_tag_tracking.registration_monitor as pure_monitor
from so101_station_perception_tag_tracking.point_cloud_samples import sampled_cloud
from .diagnostic_cloud import quaternion_matrix
from . import derived_lineage as lineage_module
from .derived_lineage import (LineageBuffer,TOPICS,CLOUD_TOPIC,LINEAGE_TOPIC,SCHEMA,
    canonical_gid,validate_graph,validate_cloud,validate_camera,configure_ros_domain)


def bounded_bytes(path, limit=131072):
    with Path(path).open('rb') as stream:
        raw=stream.read(limit+1)
    if len(raw)>limit: raise ValueError('source file exceeds bounded read')
    return raw


def read_owner(path):
    mono_before,utc_before=time.monotonic(),time.time()
    state=json.loads(bounded_bytes(path))
    utc_after,mono_after=time.time(),time.monotonic()
    return {'state':state,'utc_before':utc_before,'utc_after':utc_after,
            'mono_before':mono_before,'mono_after':mono_after}


def stamp_ns(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


def transform(message):
    t,q=message.transform.translation,message.transform.rotation
    matrix=np.eye(4);matrix[:3,:3]=quaternion_matrix(q.x,q.y,q.z,q.w)
    matrix[:3,3]=[t.x,t.y,t.z]
    return rigid(matrix)


class RegistrationMonitor(Node):
    def __init__(self):
        super().__init__('periodic_registration')
        defaults={'enabled':False,'period_s':30.,'source_cohort':'','owner_state_file':'',
                  'marker_mount':'','max_samples':4000,'fit_budget_s':.2,
                  'cloud_topic':'/camera/depth/color/points','camera_info_topic':'/camera/depth/camera_info',
                  'joint_states_topic':'/follower/joint_states','robot_description_topic':'/follower/robot_description',
                  'board_status_topic':'/so101_perception/charuco_world/status',
                  'marker_pose_topic':'/so101_perception/associated_marker_pose',
                  'base_frame':'base_link','world_frame':'charuco_board',
                  'allow_derived_source':False,'lineage_topic':LINEAGE_TOPIC}
        for key,value in defaults.items():self.declare_parameter(key,value)
        p=lambda key:self.get_parameter(key).value
        self.enabled=bool(p('enabled'));self.period=max(30.,float(p('period_s')))
        self.samples=min(4000,max(100,int(p('max_samples'))))
        self.fit_budget=min(.2,max(.001,float(p('fit_budget_s'))))
        self.base,self.world=str(p('base_frame')),str(p('world_frame'))
        self.owner_path=str(p('owner_state_file'));self.cohort_path=str(p('source_cohort'))
        self.marker_path=str(p('marker_mount'))
        self.allow_derived=p('allow_derived_source') is True;self.derived=None
        self.derived_infos={kind:deque(maxlen=16) for kind in ('depth','color')} if self.allow_derived else None
        self.source_sha256={'node':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                            'pure':hashlib.sha256(Path(pure_monitor.__file__).read_bytes()).hexdigest()}
        if self.allow_derived:self.source_sha256['derived_validator']=hashlib.sha256(Path(lineage_module.__file__).read_bytes()).hexdigest()
        self.source_invalid=False;self.reference_camera=None;self.cohort=None;self.mount=None;self.cache=None;self.description=None;self.pending_description=None
        self.joints=StationaryHistory();self.clouds=deque(maxlen=8);self.infos=deque(maxlen=12)
        self.boards=deque(maxlen=8);self.marker=None;self.job=None;self.last_job=-math.inf
        self.buffer=Buffer(cache_time=Duration(seconds=3.));self.listener=TransformListener(self.buffer,self)
        self.status=self.create_publisher(String,'/so101/registration_diagnostic/status',10)
        self.result=self.create_publisher(String,'/so101/registration_diagnostic/result',10)
        self.create_service(Trigger,'/so101/registration_diagnostic/trigger',self._trigger)
        self.create_service(SetBool,'/so101/registration_diagnostic/enable',self._enable)
        self.create_service(Trigger,'/so101/registration_diagnostic/reset',self._reset)
        self.create_subscription(PointCloud2,str(p('cloud_topic')),self._cloud,qos_profile_sensor_data)
        if self.allow_derived:
            if str(p('cloud_topic'))!=CLOUD_TOPIC or str(p('camera_info_topic'))!=TOPICS['depth_info'] or str(p('lineage_topic'))!=LINEAGE_TOPIC:
                raise ValueError('derived source requires canonical private cloud/lineage and native depth CameraInfo topics')
            self.create_subscription(CameraInfo,str(p('camera_info_topic')),self._depth_info,qos_profile_sensor_data)
            self.create_subscription(CameraInfo,TOPICS['color_info'],self._color_info,qos_profile_sensor_data)
            self.create_subscription(String,LINEAGE_TOPIC,self._lineage,qos_profile_sensor_data)
        else:
            self.create_subscription(CameraInfo,str(p('camera_info_topic')),lambda message:self.infos.append(message),qos_profile_sensor_data)
        self.create_subscription(JointState,str(p('joint_states_topic')),self._joints,qos_profile_sensor_data)
        self.create_subscription(String,str(p('board_status_topic')),self._board,10)
        self.create_subscription(String,str(p('marker_pose_topic')),self._marker,10)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,str(p('robot_description_topic')),self._description,qos)
        self.timer=self.create_timer(.05,self._tick)
        self._load_cohort()

    def _publish(self,state,**extra):
        message=String();message.data=json.dumps({'schema':'so101-registration-diagnostic-v1',
            'state':state,'qualified':False,'applied':False,'enabled':self.enabled,
            'published_at':self.get_clock().now().nanoseconds*1e-9,**extra},allow_nan=False,sort_keys=True)
        self.status.publish(message)
        return message

    def _load_cohort(self):
        self.source_invalid=False;self.job=None;self.joints.samples.clear();self.cache=None;self.reference_camera=None
        self.clouds.clear();self.infos.clear();self.boards.clear();self.marker=None
        self.derived=None
        if getattr(self,'derived_infos',None) is not None:
            for messages in self.derived_infos.values():messages.clear()
        try:
            raw=bounded_bytes(self.cohort_path);cohort=json.loads(raw)
            keys=('model_sha256','owner_session_id','arm_id','calibration_sha256','owner_source_sha256',
                  'camera_serial','cloud_publisher_gid','station_config','calibration_path',
                  'station_config_sha256','base_registration_path','base_registration_sha256','world_from_base')
            if not isinstance(cohort,dict) or cohort.get('schema')!='so101-registration-source-v1' or any(not cohort.get(k) for k in keys):
                raise ValueError('incomplete pinned source cohort')
            if not isinstance(cohort['owner_session_id'],str):raise ValueError('nonempty owner_session_id string required')
            rigid(cohort['world_from_base'])
            derived=cohort.get('derived_source')
            if 'derived_source' in cohort:
                if not getattr(self,'allow_derived',False):raise ValueError('derived source requires explicit allow_derived_source=true')
                if (not isinstance(derived,dict) or derived.get('schema')!=SCHEMA or derived.get('qualified') is not False or
                        derived.get('local_provenance_not_cryptographic') is not True):raise ValueError('invalid derived source pin')
                pin=derived['pin']
                if (pin['outputs']['cloud_publisher_gid']!=cohort['cloud_publisher_gid'] or pin['source']['serial_number']!=cohort['camera_serial']):
                    raise ValueError('derived source pin identity differs from common cohort')
                self.derived=LineageBuffer(pin)
            else:self.derived=None
            self.cohort=cohort;self.cohort_sha=hashlib.sha256(raw).hexdigest()
            mount_raw=bounded_bytes(self.marker_path) if self.marker_path else None
            self.mount=json.loads(mount_raw) if mount_raw is not None else None
            if self.mount is not None and not isinstance(self.mount,dict):raise ValueError('marker mount must be an object')
            self.mount_sha=hashlib.sha256(mount_raw).hexdigest() if mount_raw is not None else None
            self._source_files()
            if self.description is not None:self.pending_description=self.description
            self._publish('waiting_for_model_and_stationary_cohort')
        except (OSError,ValueError,TypeError,KeyError) as error:
            self.derived=None;self.cohort=None;self._publish('unavailable_source_cohort',reason=str(error))

    def _source_files(self):
        for path,key in (('station_config','station_config_sha256'),('calibration_path','calibration_sha256'),
                         ('base_registration_path','base_registration_sha256')):
            if hashlib.sha256(bounded_bytes(self.cohort[path])).hexdigest()!=self.cohort[key]:
                self._invalidate('pinned '+path+' changed; explicit repin/reset required')
                raise ValueError('pinned '+path+' changed; explicit repin/reset required')

    def _description(self,message):
        if message.data!=self.description:
            self.description=message.data;self.pending_description=message.data
            self.cache=None;self.job=None;self.joints.samples.clear()
            self._publish('model_changed_history_reset')

    def _invalidate(self,reason):
        already_invalid=self.source_invalid
        self.source_invalid=True;self.job=None;self.cache=None;self.reference_camera=None
        self.joints.samples.clear();self.clouds.clear();self.infos.clear();self.boards.clear();self.marker=None
        if getattr(self,'derived',None) is not None:self.derived.entries.clear()
        if getattr(self,'derived_infos',None) is not None:
            for messages in self.derived_infos.values():messages.clear()
        if not already_invalid:self._publish('unavailable_source_changed',reason=reason)

    def _lineage(self,message,info):
        if self.source_invalid or self.derived is None:return
        gid=info.get('publisher_gid',b'') if isinstance(info,dict) else getattr(info,'publisher_gid',b'')
        try:self.derived.add(message.data,gid,now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        except ValueError as error:
            if self.derived.invalid_reason:self._invalidate(str(error))

    def _native_info(self,kind,message,info):
        if self.source_invalid or self.derived is None:return
        gid=info.get('publisher_gid',b'') if isinstance(info,dict) else getattr(info,'publisher_gid',b'')
        try:
            self.derived.observe_gid(kind+'_info',gid)
            self.derived_infos[kind].append((message,canonical_gid(gid)))
            if kind=='depth':self.infos.append(message)
        except ValueError as error:self._invalidate(str(error))

    def _depth_info(self,message,info):self._native_info('depth',message,info)
    def _color_info(self,message,info):self._native_info('color',message,info)

    def _derived_cohort(self,cloud,gid):
        value,status_gid,receipt=self.derived.match(stamp_ns(cloud),now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        try:validate_graph(value,self.get_publishers_info_by_topic,cloud_gid=gid,lineage_gid=status_gid)
        except ValueError as error:self._invalidate(str(error));raise
        try:validate_cloud(value,cloud,gid)
        except ValueError as error:self._invalidate(str(error));raise
        native={}
        for kind in ('depth','color'):
            matches=[message for message,info_gid in self.derived_infos[kind]
                     if stamp_ns(message)==value['original'][kind+'_info_stamp_ns'] and info_gid==value['source'][kind+'_info_publisher_gid']]
            if not matches:raise ValueError('exact native CameraInfo for derived cohort unavailable')
            native[kind]=matches[-1]
        pose=self._tf(value['original']['color_frame'],value['original']['depth_frame'],stamp_ns(cloud))
        try:validate_camera(value,native['depth'],native['color'],pose)
        except ValueError as error:self._invalidate(str(error));raise
        self.derived.match(stamp_ns(cloud),now_ns=self.get_clock().now().nanoseconds,now_mono=time.monotonic())
        return value,native['depth'],receipt,status_gid

    def _cloud(self,message,info):
        if self.source_invalid:return
        if getattr(self,'derived',None) is not None:
            raw=info.get('publisher_gid',b'') if isinstance(info,dict) else getattr(info,'publisher_gid',b'')
            try:self.derived.observe_gid('cloud',raw)
            except ValueError as error:self._invalidate(str(error));return
            if self.job is not None:self.clouds.append((message,canonical_gid(raw)))
            return
        gid=info.get('publisher_gid',b'') if isinstance(info,dict) else getattr(info,'publisher_gid',b'')
        gid=bytes(gid).hex()
        if self.cohort is not None and gid!=self.cohort['cloud_publisher_gid']:
            self._invalidate('camera publisher changed or GID unavailable; explicit repin/reset required')
            return
        if self.job is not None:self.clouds.append((message,gid))

    def _joints(self,message):
        if len(message.name)==len(message.position) and len(set(message.name))==len(message.name):
            self.joints.add(stamp_ns(message)*1e-9,dict(zip(message.name,message.position)),
                            now=self.get_clock().now().nanoseconds*1e-9)

    def _board(self,message):
        try:
            if len(message.data)>16384:raise ValueError('board status exceeds cap')
            value=json.loads(message.data)
            if isinstance(value,dict):self.boards.append(value)
        except (TypeError,ValueError):pass

    def _marker(self,message):
        try:
            if len(message.data)>16384:raise ValueError('marker status exceeds cap')
            self.marker=json.loads(message.data)
        except (TypeError,ValueError):self.marker=None

    def _enable(self,request,response):
        self.enabled=bool(request.data)
        if not self.enabled:self.job=None
        response.success=True;response.message='enabled' if self.enabled else 'disabled; pending collection cancelled'
        return response

    def _reset(self,request,response):
        self._load_cohort();response.success=self.cohort is not None
        response.message='source cohort reloaded' if response.success else 'source cohort unavailable'
        return response

    def _start(self):
        if self.source_invalid:return False,'source invalidated; explicit repin/reset required'
        if self.job is not None:return False,'busy; no queued request'
        if time.monotonic()-self.last_job<self.period:return False,'minimum period/cooldown; no queued request'
        if self.cache is None or self.cohort is None:return False,'model/source cohort unavailable'
        self._source_files()
        self.last_job=time.monotonic();self.clouds.clear()
        if getattr(self,'derived',None) is not None:self.derived.entries.clear()
        self.job={'started':self.last_job,'reads':[]}
        self._publish('collecting_stationary_cohort')
        return True,'collection accepted; result published on result/status'

    def _trigger(self,request,response):
        try:response.success,response.message=self._start()
        except (OSError,ValueError) as error:response.success=False;response.message=str(error)
        return response

    def _read_owner(self):
        item=read_owner(self.owner_path)
        state=item.get('state')
        if not isinstance(state,dict) or state.get('owner_session_id')!=self.cohort['owner_session_id']:
            self._invalidate('owner generation changed or unavailable; explicit repin/reset required')
            raise ValueError('owner generation changed or unavailable')
        fault=owner_eligibility(state)
        if fault is not None:raise ValueError(fault)
        return item

    def _tick(self):
        if not self.source_invalid and self.pending_description is not None and self.job is None and self.cohort is not None:
            description=self.pending_description;self.pending_description=None
            try:
                if hashlib.sha256(description.encode()).hexdigest()!=self.cohort['model_sha256']:
                    raise ValueError('model hash differs from pinned cohort; repin/reset required')
                self.cache=MeshSamples(description,get_package_share_directory,samples=self.samples)
                self._publish('ready',model=self.cache.provenance)
            except (OSError,ValueError,KeyError) as error:
                self.cache=None;self._publish('unavailable_model',reason=str(error))
        if self.job is None:
            if self.enabled and time.monotonic()-self.last_job>=self.period:
                try:
                    accepted,reason=self._start()
                    if not accepted:self.last_job=time.monotonic();self._publish('unavailable',reason=reason)
                except (OSError,ValueError) as error:
                    self.last_job=time.monotonic();self._publish('unavailable',reason=str(error))
            return
        job=self.job
        try:
            elapsed=time.monotonic()-job['started']
            if elapsed>.7:raise ValueError('cohort collection wall budget exceeded')
            if len(job['reads'])<12:job['reads'].append(self._read_owner())
            if len(job['reads'])<12:return
            self.job=None
            self._run(job)
        except (OSError,ValueError,KeyError,TypeError,TransformException) as error:
            self.job=None;self._publish('unavailable',reason=str(error))

    def _tf(self,target,source,stamp):
        return transform(self.buffer.lookup_transform(target,source,Time(nanoseconds=stamp)))

    def _cohort(self,job):
        now=self.get_clock().now().nanoseconds*1e-9
        if abs(now-time.time())>.05:raise ValueError('ROS and owner UTC clock domains differ')
        rejection='no paired cloud/board/camera/joint/owner cohort'
        candidates=tuple(self.clouds)[-1:] if getattr(self,'derived',None) is not None else reversed(self.clouds)
        for cloud,gid in candidates:
            try:
                stamp=stamp_ns(cloud)*1e-9
                if gid!=self.cohort['cloud_publisher_gid']:raise ValueError('camera cloud publisher changed')
                joint_stamp,joints=self.joints.select(stamp,now=now)
                board=min(self.boards,key=lambda b:abs(float(b.get('stamp',-math.inf))-stamp))
                if (board.get('state')!='tracking' or board.get('observation_valid') is not True
                        or board.get('plane_source')!='aligned_depth' or not 0<=now-float(board['stamp'])<=.5
                        or abs(float(board['stamp'])-stamp)>.05):raise ValueError('fresh paired depth-backed board unavailable')
                if getattr(self,'derived',None) is not None:lineage,info,_,_=self._derived_cohort(cloud,gid)
                else:info=min(self.infos,key=lambda m:abs(stamp_ns(m)*1e-9-stamp))
                k=np.asarray(info.k,dtype=float).reshape(3,3)
                if (abs(stamp_ns(info)*1e-9-stamp)>.05 or stamp_ns(info)*1e-9>now
                        or info.header.frame_id!=cloud.header.frame_id or not np.isfinite(k).all()
                        or min(k[0,0],k[1,1])<=0 or not 1<=info.width<=1280 or not 1<=info.height<=720):
                    raise ValueError('paired cloud CameraInfo invalid')
                binding=owner_bracket(job['reads'],stamp,joints,self.cohort,now=now)
                return cloud,info,joints,joint_stamp,board,binding
            except (ValueError,KeyError,TypeError) as error:rejection=str(error)
        raise ValueError(rejection)

    def _run(self,job):
        cloud,info,joints,joint_stamp,board,binding=self._cohort(job)
        ns=stamp_ns(cloud);stamp=ns*1e-9;frame=cloud.header.frame_id
        budget=Deadline(self.fit_budget)
        base_from_camera=self._tf(self.base,frame,ns)
        world_from_base=self._tf(self.world,self.base,ns)
        try:validate_base_pin(world_from_base,self.cohort['world_from_base'])
        except ValueError:
            self._invalidate('world_from_base changed; explicit repin/reset required');raise
        world_from_camera=self._tf(self.world,frame,ns)
        link_poses={link:self._tf(self.base,link,ns) for link in self.cache.links}
        marker_link=self.mount.get('link') if self.mount else None
        if marker_link in self.cache.all_links and marker_link not in link_poses:
            link_poses[marker_link]=self._tf(self.base,marker_link,ns)
        points,normals,labels=self.cache.at(link_poses)
        points,normals,labels=visible_samples(points,normals,labels,np.linalg.inv(base_from_camera),info.k,info.width,info.height)
        observed=sampled_cloud(cloud,4000)[0]
        observed=observed@base_from_camera[:3,:3].T+base_from_camera[:3,3]
        report=fit_registration(points,normals,labels,observed,deadline=budget)
        self._source_files()
        after=self._read_owner()
        after_stamp=datetime.fromisoformat(after['state']['timestamp_utc'].replace('Z','+00:00')).timestamp()
        post=owner_bracket([after],after_stamp,joints,self.cohort,now=time.time())
        if after['utc_after']-after_stamp>.1 or post['raw']!=binding['raw']:
            raise ValueError('owner changed during fit or final owner sample stale')
        now=self.get_clock().now().nanoseconds*1e-9
        if not 0<=now-stamp<=.5:raise ValueError('capture too old at fit completion')
        selected_lineage=None
        if getattr(self,'derived',None) is not None:
            value,_,receipt,status_gid=self._derived_cohort(cloud,self.cohort['cloud_publisher_gid'])
            selected_lineage={'statement':value,'received_monotonic':receipt,'publisher_gid':status_gid}
        marker_link=self.mount.get('link') if self.mount else None
        marker_pose=(np.linalg.inv(base_from_camera)@link_poses[marker_link]) if marker_link in link_poses else np.eye(4)
        attached=marker_residual(self.mount,self.marker,marker_pose,stamp,self.cache.model_sha256,camera_frame=frame)
        if marker_link is not None and marker_link not in link_poses:attached={'state':'unavailable_mount_link_missing','qualified':False}
        if self.reference_camera is None:self.reference_camera=world_from_camera.copy()
        drift=np.linalg.inv(self.reference_camera)@world_from_camera
        from scipy.spatial.transform import Rotation
        board_drift={'reference':'first admitted cohort after reset',
                     'translation_m':float(np.linalg.norm(drift[:3,3])),
                     'rotation_deg':float(np.linalg.norm(Rotation.from_matrix(drift[:3,:3]).as_rotvec())*180/math.pi)}
        candidate=report['candidate_delta']
        world_candidate=(world_from_base@np.asarray(candidate)).tolist() if candidate is not None else None
        payload={'capture_stamp_ns':ns,'capture_age_s':now-stamp,'cloud_frame':frame,
                 'cloud_publisher_gid':self.cohort['cloud_publisher_gid'],'camera_serial':self.cohort['camera_serial'],
                 'source_cohort_sha256':self.cohort_sha,'monitor_source_sha256':self.source_sha256,
                 'station_config_sha256':self.cohort['station_config_sha256'],
                 'base_registration_sha256':self.cohort['base_registration_sha256'],
                 'model':self.cache.provenance,'owner':binding,
                 'joint_stamp':joint_stamp,'fixed_joint_positions':joints,
                 'world_from_base_initial':world_from_base.tolist(),'world_from_base_candidate':world_candidate,
                 'ground_constraint':ground_metrics(world_from_base,world_candidate,cad_floor_z=self.cache.base_floor_z),
                 'world_from_camera':world_from_camera.tolist(),'board_camera_drift':board_drift,'board_observation_stamp':board['stamp'],
                 'board':{'charuco_corners':board.get('charuco_corners'),'reprojection_rms_px':board.get('reprojection_rms_px')},
                 'attached_marker':attached,'marker_mount_sha256':self.mount_sha,'fit':report,
                 'budget':{'collection_reads':len(job['reads']),'fit_seconds':self.fit_budget},
                 'interpretation':'stationary single-pose proposal; no transform/model/qzero writes; holdout validation required'}
        if getattr(self,'derived',None) is not None:
            payload['derived_source']={'qualified':False,'local_provenance_not_cryptographic':True,'pin':self.derived.pin,'lineage':selected_lineage}
        self.result.publish(self._publish(report['state'],**payload))


def main(args=None):
    configure_ros_domain(os.environ)
    rclpy.init(args=args);node=None
    try:
        node=RegistrationMonitor();rclpy.spin(node)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
