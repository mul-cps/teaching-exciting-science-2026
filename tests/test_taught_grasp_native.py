"""Synthetic ROS transport fixtures for the real taught grasp CLI transaction.

No hardware or commissioned skill is used; FK and actuators are explicit fakes.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading

import pytest
rclpy=pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2/src/so101_station_control'))
from geometry_msgs.msg import Pose,PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectoryPoint
from workshop.blockly_recording import Recorder
import test_ros_grasp_cli_native as base

SOURCE={'fixture':'a'*64}
SOURCE_HASH=hashlib.sha256(json.dumps(SOURCE,sort_keys=True,separators=(',',':')).encode()).hexdigest()
IDENTITY={'arm_id':'synthetic-arm','calibration_sha256':'b'*64,'source_sha256':SOURCE_HASH,'owner_generation':1.}


def manifest(tmp_path,files,model):
    def entry(path): return {'path':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    angular=tmp_path/'joint.json';angular.write_text(json.dumps({'schema':'so101-gripper-joint-calibration-v1','qualified':True,'arm_id':IDENTITY['arm_id'],'calibration_sha256':IDENTITY['calibration_sha256'],'closed_joint_rad':0.,'open_joint_rad':1.,'closed_percent':0.,'open_percent':100.}))
    config=json.loads(files['execution.json'].read_text())
    artifacts={key:entry((files['execution.json'].parent/Path(config[key])).resolve()) for key in ('jaw_calibration','tcp_calibration','camera_arm_calibration')}
    artifacts['execution_config']=entry(files['execution.json']);artifacts['joint_calibration']=entry(angular)
    identity={**IDENTITY,'jaw_calibration_sha256':artifacts['jaw_calibration']['sha256'],'joint_calibration_sha256':artifacts['joint_calibration']['sha256']}
    phases={}
    for phase in ('approach','lift','transfer','return'):
        recorder=Recorder(phase,identity,origin='physical')
        recorder.add([0.]*5,0.,0.);recorder.add([0.]*5,0.,.1)
        path=tmp_path/(phase+'.stroke');path.write_text(json.dumps(recorder.finish()));phases[phase]=entry(path)
    contact=base.PoseTarget(base.FRAME_ID,base.BASE_OBJECT,(1.,0.,0.,0.))
    targets=dict(base.staged_targets(contact,place=base.PoseTarget(base.FRAME_ID,base.PLACE_POSE[:3],base.PLACE_POSE[3:]),tip_from_grasp=base._tcp_calibration()['tip_from_grasp']))
    checkpoints={'pickup':[*base.BASE_OBJECT,1.,0.,0.,0.],
                 'drop':list(base.PLACE_POSE),'approach_start':[*targets['pregrasp'].position,*targets['pregrasp'].orientation_xyzw],
                 'return':[.45,.05,.26,0.,0.,0.,1.]}
    data={'schema':'so101-taught-pick-place-v1','name':'fixture','origin':'physical','qualified':True,
          'qualification':{'instructor':'synthetic regression only','evidence_sha256':'e'*64},
          'binding':{k:v for k,v in identity.items() if k!='owner_generation'},
          'fixture':{'id':'synthetic-only','provenance_sha256':'f'*64},
          'frame_id':base.FRAME_ID,'tip_link':base.TIP_LINK,'checkpoints':checkpoints,'artifacts':artifacts,'phases':phases}
    data['binding']['effective_model_sha256']=hashlib.sha256(model.encode()).hexdigest()
    path=tmp_path/'fixture.json';path.write_text(json.dumps(data))
    return path,data,targets


def start(monkeypatch,tmp_path,*,failure=None):
    files,model=base._write_calibrations(tmp_path)
    path,data,targets=manifest(tmp_path,files,model)
    def owner_status(bridge,request,response):
        generation=2. if failure=='restart' and bridge._test_fixture.arm_goal_count>=1 else 1.
        response.success=True
        payload={'schema':1,'kind':'follower_status','sampled_monotonic':time.monotonic(),
            'fresh':True,'position_age_s':0.,'torque_age_s':0.,'owner_source_sha256':SOURCE,
            'owner_started_monotonic':generation,'arm_id':IDENTITY['arm_id'],'calibration_sha256':IDENTITY['calibration_sha256'],
            'jaw_calibration_sha256':data['artifacts']['jaw_calibration']['sha256'],
            'joint_calibration_sha256':data['artifacts']['joint_calibration']['sha256']}
        if failure=='jaw_missing':payload['jaw_calibration_sha256']=None
        if failure=='joint_malformed':payload['joint_calibration_sha256']='unknown'
        if failure in {'jaw_changed','joint_changed'} and bridge._test_fixture.arm_goal_count>=1:
            payload[('jaw' if failure=='jaw_changed' else 'joint')+'_calibration_sha256']='0'*64
        response.message=json.dumps(payload)
        return response
    monkeypatch.setattr(base.TrajectoryBridge,'_status',owner_status)
    # The sole measured stream is this synthetic fixture, not a second visual-proxy publisher.
    monkeypatch.setattr(base.TrajectoryBridge,'_publish_joint_state',lambda bridge:None)
    original_status=base.TrajectoryBridge._gripper_calibration_status
    def calibration(bridge,req,res):
        res.success=True;res.message=json.dumps({'schema':'so101-gripper-calibration-status-v1','si_gap_calibrated':True,'moveit_joint_calibrated':True,'visual_proxy_only':False,'si_position_units':'metres','moveit_position_units':'radians',
            'jaw_calibration_sha256':data['artifacts']['jaw_calibration']['sha256'],
            'joint_calibration_sha256':data['artifacts']['joint_calibration']['sha256']})
        return res
    monkeypatch.setattr(base.TrajectoryBridge,'_gripper_calibration_status',calibration)
    def plan(fixture,request,response):
        fixture.plans.append({'joint_goal':True})
        response.motion_plan_response.error_code.val=0 if failure=='no_path' else 1
        trajectory=response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names=list(base.ARM_JOINTS);point=JointTrajectoryPoint();point.positions=[0.]*5;point.time_from_start.sec=1;trajectory.points=[point]
        return response
    monkeypatch.setattr(base._Fixture,'_plan',plan)
    def fk(fixture,request,response):
        index=fixture.arm_goal_count
        target=[targets['pregrasp'],targets['contact'],targets['lift'],targets['place'],base.PoseTarget(base.FRAME_ID,(.45,.05,.26),(0.,0.,0.,1.))][min(index,4)]
        stamped=PoseStamped();stamped.header=request.header
        stamped.pose.position.x,stamped.pose.position.y,stamped.pose.position.z=target.position
        stamped.pose.orientation.x,stamped.pose.orientation.y,stamped.pose.orientation.z,stamped.pose.orientation.w=target.orientation_xyzw
        response.error_code.val=1;response.fk_link_names=list(request.fk_link_names);response.pose_stamped=[stamped]
        return response
    monkeypatch.setattr(base._Fixture,'_fk',fk)
    original_execute=base._Fixture._execute_arm
    def execute(fixture,handle):
        # Exercise healthy owner refresh across multiple 300ms identity windows.
        if fixture.arm_goal_count==0:
            fixture.first_goal_started.set()
            time.sleep(.55)
        return original_execute(fixture,handle)
    monkeypatch.setattr(base._Fixture,'_execute_arm',execute)
    original_publish=base._Fixture._publish_live_observations
    def publish(fixture):
        if failure=='lift' and fixture.phase in {'lift','lift_observed'}: fixture.phase='initial'
        if failure=='release' and fixture.phase in {'released','released_observed'}: fixture.phase='initial'
        original_publish(fixture)
    monkeypatch.setattr(base._Fixture,'_publish_live_observations',publish)
    original_create=base._create_bridge
    def create_fixture_bridge(monkeypatch,directory):
        path=original_create(monkeypatch,directory)
        original_start=base.TrajectoryBridge._start_owner
        def start_owner(bridge,**kwargs): original_start(bridge)
        monkeypatch.setattr(base.TrajectoryBridge,'_start_owner',start_owner)
        return path
    monkeypatch.setattr(base,'_create_bridge',create_fixture_bridge)
    fixture,bridge,executor,stop,thread=base._start_fixture(monkeypatch,tmp_path,cancel_on_lift=failure=='cancel')
    fixture.set_robot_description(model)
    fixture.first_goal_started=threading.Event()
    if failure=='no_contact':
        original_send=bridge._send
        def no_contact(command):
            if command.get('command')=='move_gripper' and command.get('target_raw')==500:
                fixture.gripper_commands.append(command)
                bridge._response_queue.put({'ok':True,'state':{'event':'plan_complete','pose':{'gripper':0.},'legs':[{'label':'gripper_command','contact_inferred':False}]}})
            else: original_send(command)
        bridge._send=no_contact
    if failure=='late_ack':
        def delayed_goal(goal):
            fixture.first_goal_started.set();time.sleep(5.5)
            return base.GoalResponse.ACCEPT
        fixture.arm_server.register_goal_callback(delayed_goal)
    output=tmp_path/'result.json'
    command=[sys.executable,str(base.ROOT/'tools/ros_plan_grasp.py'),'--execute','--track-id',base.TRACK_ID,'--frame',base.FRAME_ID,'--tip-link',base.TIP_LINK,'--place-pose',*(str(v) for v in base.PLACE_POSE),
             '--execution-config',str(files['execution.json']),'--taught-skill',str(path),'--trajectory-action',base.TEST_ARM_ACTION,'--timeout','35','--output',str(output)]
    fixture.lifetime_w=None
    pass_fds=()
    if failure in {'parent_before','parent_during','late_ack'}:
        read_fd,write_fd=os.pipe();fixture.lifetime_w=write_fd
        command.extend(['--taught-lifetime-fd',str(read_fd)]);pass_fds=(read_fd,)
        if failure=='parent_before':os.close(write_fd);fixture.lifetime_w=None
    process=subprocess.Popen(command,cwd=base.ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=os.environ.copy(),pass_fds=pass_fds)
    if pass_fds:os.close(read_fd)
    return fixture,bridge,executor,stop,thread,process,output


@pytest.mark.parametrize('failure',[None,'no_path','no_contact','lift','release','restart','cancel','parent_before','parent_during','late_ack','jaw_changed','joint_changed','jaw_missing','joint_malformed'])
def test_taught_existing_transaction(monkeypatch,tmp_path,failure):
    fixture,bridge,executor,stop,thread,process,output=start(monkeypatch,tmp_path,failure=failure)
    try:
        if failure=='cancel':
            assert fixture.lift_goal_started.wait(20)
            process.send_signal(signal.SIGINT)
        if failure in {'parent_during','late_ack'}:
            assert fixture.first_goal_started.wait(20)
            os.close(fixture.lifetime_w);fixture.lifetime_w=None
        stdout,_=base._communicate_cli(process,fixture,timeout=45)
        if failure in {'parent_before','jaw_missing','joint_malformed'}:
            assert process.returncode!=0 and fixture.arm_goal_count==0
            if failure=='jaw_missing':assert 'reteach required' in stdout
            if failure=='joint_malformed':assert 'calibration digest malformed' in stdout
            return
        assert output.exists(),stdout
        result=json.loads(output.read_text())
        if failure is None:
            assert process.returncode==0,stdout+'\n'+str(result)
            assert result['state']=='pick_place_complete'
            assert all(result[field] is True for field in ('pickup_verified','lift_verified','release_verified'))
            assert len(fixture.plans)==1 # only explicit pregrasp is planned
            assert fixture.arm_goal_count==5
            assert result['segments'][-1]['name']=='post_release_taught_return'
        else:
            assert process.returncode!=0,stdout
            assert result['state']!='pick_place_complete'
            if failure in {'no_path','no_contact','lift','restart','cancel','parent_during','late_ack','jaw_changed','joint_changed','jaw_missing','joint_malformed'}:
                assert not any(item['action']=='detach' for item in fixture.scene_requests)
            if failure in {'no_path','jaw_missing','joint_malformed'}: assert fixture.arm_goal_count==0
            if failure=='lift': assert fixture.arm_goal_count==3 and not result['lift_verified']
            if failure=='release': assert result['lift_verified'] and not result['release_verified']
    finally:
        if process.poll() is None: process.kill();process.wait(timeout=3)
        if fixture.lifetime_w is not None:os.close(fixture.lifetime_w)
        base._cleanup(fixture,bridge,executor,stop,thread)
