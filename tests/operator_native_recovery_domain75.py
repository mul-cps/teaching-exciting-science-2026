#!/usr/bin/env python3
"""Real DDS + fake owner/manager only; never imports hardware or calls systemd.

Run from an existing built ROS overlay with ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1.
"""
import importlib.machinery
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
import uuid

if os.environ.get('ROS_DOMAIN_ID')!='75' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('requires isolated ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1')

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
from vision_msgs.msg import Detection3DArray
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import String
from std_srvs.srv import Trigger
from so101_station_operator.srv import OperatorMaintenance

ROOT=Path(__file__).resolve().parents[1]
mod=importlib.machinery.SourceFileLoader('native_api',str(ROOT/'ros2/src/so101_station_operator/scripts/operator_api')).load_module()
TOPICS={key:'/native_test/'+key for key in ('rgb_image','depth_image','rgb','depth','tags','objects','objects_status','scene_status','joint_states','movegroup')}
COMPONENTS={'camera':('rgb_image','depth_image','rgb','depth'),'tags':('tags',),
            'objects':('objects','objects_status'),'scene':('scene_status',),'movegroup':('movegroup',),'owner':('joint_states',)}
NODES={key:'/native_fake_'+component for component,keys in COMPONENTS.items() for key in keys}
UNITS={key:'so101-fake-native-'+key+'.service' for key in COMPONENTS if key!='owner'}


class Component(Node):
    def __init__(self, fixture, component):
        super().__init__('native_fake_'+component)
        self.fixture,self.component=fixture,component
        self.instance=uuid.uuid4().hex
        self.pubs={}
        self.restart()
        self.create_timer(.04,self.publish)
        if component=='movegroup':
            self.create_service(GetPlanningScene,'/follower/get_planning_scene',self.scene)

    def restart(self):
        for publisher in self.pubs.values():self.destroy_publisher(publisher)
        self.instance=uuid.uuid4().hex
        self.pubs={key:self.create_publisher(Image if key.endswith('_image') else CameraInfo if key in ('rgb','depth') else
            JointState if key=='joint_states' else Detection3DArray if key=='objects' else PlanningScene if key=='movegroup' else String,TOPICS[key],10)
            for key in COMPONENTS[self.component]}

    def scene(self,req,res):
        res.scene.robot_state.joint_state.name=sorted(mod.JOINTS)
        res.scene.robot_state.joint_state.position=[0.]*6
        if self.fixture.expire_during_moveit and self.fixture.api.native_proof is not None:
            self.fixture.paused=True
            time.sleep(1.1)
        return res

    def publish(self):
        with self.fixture.lock:
            if self.fixture.components.get(self.component) is not self:return
            if self.fixture.paused:return
            if self.component=='camera':self.fixture.stamp=self.get_clock().now().to_msg()
            stamp=self.fixture.stamp
            if stamp is None:return
            ns=stamp.sec*10**9+stamp.nanosec
            for key,pub in self.pubs.items():
                if key.endswith('_image'):
                    msg=Image();msg.width=msg.height=2;msg.encoding='16UC1' if key=='depth_image' else 'rgb8'
                    msg.step=4 if key=='depth_image' else 6;msg.data=b'0'*(msg.step*2)
                elif key in ('rgb','depth'):
                    msg=CameraInfo();msg.width=msg.height=2;msg.k=[1.,0.,1.,0.,1.,1.,0.,0.,1.]
                elif key=='joint_states':
                    msg=JointState();msg.name=sorted(mod.JOINTS);msg.position=[0.]*6
                elif key=='objects':msg=Detection3DArray()
                elif key=='movegroup':msg=PlanningScene()
                else:
                    if key=='tags':data=dict(schema='so101-tag-detections-v1',stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec),observations=[])
                    elif key=='objects_status':data=dict(schema=1,state='tracking',stamp=ns/1e9,frame_id='charuco_board',color_mask_backend={'selected':'vulkan'})
                    else:data=dict(state='empty',stamp=ns/1e9,accepted=0,rejected=0,instance_id=self.instance,scene_busy=False,pending_count=0,scene_fault=False,live_ids=[],applied_world_ids=[])
                    msg=String(data=json.dumps(data));pub.publish(msg);continue
                if hasattr(msg,'header'):
                    msg.header.stamp=stamp;msg.header.frame_id='charuco_board' if key=='objects' else '' if key=='joint_states' else 'optical'
                pub.publish(msg)


class Owner(Node):
    def __init__(self, fixture):
        super().__init__('native_fake_manager')
        self.fixture=fixture;self.calls=[];self.unknown=False
        self.state=dict(kind='follower_status',fresh=True,connected=True,observe_only=False,owner_faulted=False,
            active_operation=None,torque=dict.fromkeys(mod.JOINTS,0),arm_id='FAKE',calibration_sha256='a'*64,
            owner_started_monotonic=1.,owner_source_sha256={'fake':'b'*64},operator_external_job=None,
            operator_restart_units=[UNITS[k] for k in ('camera','tags')],
            operator_station_reset_units=[UNITS[k] for k in ('movegroup','objects','scene')])
        group=ReentrantCallbackGroup()
        for name in ('status','stop','reset_owner','reconnect'):
            self.create_service(Trigger,'/follower/'+name,lambda req,res,name=name:self.trigger(name,res),callback_group=group)
        self.create_service(OperatorMaintenance,'/follower/operator_maintenance_enable',self.maintenance,callback_group=group)

    def trigger(self,name,res):
        if name!='status':self.calls.append(name)
        if name=='stop':self.state.update(active_operation=None,operator_maintenance_token=None)
        elif name=='reset_owner':self.state['owner_started_monotonic']+=1
        res.success=True;res.message=json.dumps(self.state) if name=='status' else 'fake '+name
        return res

    def maintenance(self,req,res):
        self.calls.append(('maintenance',req.operation,req.unit,req.enable))
        if req.enable and not req.token:
            self.state.update(active_operation='operator_maintenance',operator_maintenance_token='a'*32)
            value={'token':'a'*32}
        elif req.enable:
            assert req.token==self.state['operator_maintenance_token']
            component=next(k for k,v in UNITS.items() if v==req.unit)
            self.fixture.replace(component)
            job=uuid.uuid4().hex
            self.state['operator_restart_result']=dict(job_id=job,kind='restart_outcome_unknown' if self.unknown else 'restart_unit_terminal',success=not self.unknown)
            value={'job_id':job}
        else:
            assert req.token==self.state['operator_maintenance_token']
            self.state.update(active_operation=None,operator_maintenance_token=None);value={}
        res.success=True;res.message=json.dumps(value);return res


class Fixture:
    def __init__(self):
        self.lock=threading.Lock();self.stamp=None;self.paused=False;self.expire_during_moveit=False
        self.executor=MultiThreadedExecutor(num_threads=8)
        self.components={}
        for key in COMPONENTS:self.replace(key)
        self.owner=Owner(self);self.executor.add_node(self.owner)
        self.api=mod.OperatorAPI();self.executor.add_node(self.api)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        end=time.monotonic()+5.
        while time.monotonic()<end:
            try:
                if self.api.native_graph() and self.api.service_clients['maintenance'].service_is_ready():break
            except RuntimeError:pass
            time.sleep(.05)
        else:raise RuntimeError('fake graph did not discover')
        time.sleep(.15)
        def diagnostic():
            for _ in range(12):
                if self.closed.wait(5.):return
                p=self.api.native_proof
                if p is None:
                    print('FAKE75 graph',getattr(self.api,'native_graph_reason',None),flush=True)
                else:
                    with p.lock:
                        print('FAKE75 proof',self.api.reset_progress,{k:len(v) for k,v in p.samples.items()},p.blocked,flush=True)
        self.closed=threading.Event()
        threading.Thread(target=diagnostic,daemon=True).start()

    def replace(self,key):
        with self.lock:
            previous=self.components.get(key)
            if previous:
                previous.restart()
                return
            node=Component(self,key);self.components[key]=node;self.executor.add_node(node)

    def close(self):
        self.closed.set()
        self.executor.shutdown(timeout_sec=5.);self.thread.join(5.)
        for node in [self.api,self.owner,*self.components.values()]:node.destroy_node()


class NativeDDS(unittest.TestCase):
    def setUp(self):self.f=Fixture()
    def tearDown(self):self.f.close()

    def command(self,name):return self.f.api.command(name,None,SimpleNamespace())

    def test_full_reset_and_moveit_and_absent_camera_recovery(self):
        # An absent old endpoint is legitimate input to a requested restart.
        with self.f.lock:
            old=self.f.components['camera']
            for publisher in old.pubs.values():old.destroy_publisher(publisher)
            old.pubs={}
        time.sleep(.15)
        result=self.command('emergency_reset');self.assertTrue(result.success,result.message)
        self.assertTrue(self.f.api.reset_progress['initialized_all_off'])
        self.assertFalse(self.f.api.reset_progress['pick_place_ready'])
        self.assertEqual(self.f.owner.state['torque'],dict.fromkeys(mod.JOINTS,0))
        self.assertFalse(self.f.api.service_clients['recovery_start'].service_is_ready())
        calls=list(self.f.owner.calls)
        self.assertEqual(sum(isinstance(c,tuple) and c[2]==UNITS['camera'] for c in calls),1)
        result=self.command('restart_moveit');self.assertTrue(result.success,result.message)
        self.assertNotIn('home',self.f.owner.calls)

    def test_unknown_manager_job_retains_barrier(self):
        self.f.owner.unknown=True
        result=self.command('restart_moveit')
        self.assertFalse(result.success);self.assertIn('unknown',result.message)
        self.assertEqual(self.f.owner.state['active_operation'],'operator_maintenance')
        self.assertFalse(any(isinstance(c,tuple) and c[3] is False for c in self.f.owner.calls))

    def test_fresh_to_stale_to_fresh_and_final_rpc_expiry(self):
        self.assertTrue(self.command('emergency_reset').success)
        # The last proof cannot be renewed by polling; use real DDS messages.
        gids=self.f.api.native_graph();now=time.monotonic()
        proof=mod.NativeResetProof(gids,self.f.api.get_clock().now().nanoseconds,now,'vulkan',.04)
        self.f.api.native_proof=proof;self.f.api.activate_reset_images()
        time.sleep(.35)
        self.assertTrue(proof.ready(time.monotonic(),self.f.api.get_clock().now().nanoseconds))
        self.f.paused=True;time.sleep(1.1)
        self.assertFalse(proof.ready(time.monotonic(),self.f.api.get_clock().now().nanoseconds))
        self.f.paused=False;time.sleep(.35)
        self.assertTrue(proof.ready(time.monotonic(),self.f.api.get_clock().now().nanoseconds))
        self.f.api.retire_reset_images();self.f.api.native_proof=None
        self.f.expire_during_moveit=True
        result=self.command('restart_moveit')
        self.assertFalse(result.success);self.assertEqual(self.f.owner.state['active_operation'],'operator_maintenance')


if __name__=='__main__':
    parameters={'recovery_profile':'native_ros','enable_service_restarts':True,'native_publisher_nodes_json':json.dumps(NODES),
                'camera_unit':UNITS['camera'],'depth_unit':UNITS['camera'],'tags_unit':UNITS['tags'],
                **{key+'_unit':UNITS[key] for key in ('movegroup','objects','scene')},
                **{key+'_topic':value for key,value in TOPICS.items() if key!='movegroup'},
                'movegroup_scene_topic':TOPICS['movegroup']}
    with tempfile.NamedTemporaryFile(mode='w',suffix='.json') as config:
        json.dump({'/so101_operator_api':{'ros__parameters':parameters}},config);config.flush()
        rclpy.init(args=['--ros-args','--params-file',config.name])
        try:unittest.main()
        finally:rclpy.shutdown()
