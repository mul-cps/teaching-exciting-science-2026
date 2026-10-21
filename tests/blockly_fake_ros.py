"""Standalone fake ROS integration: run in ROS_DOMAIN_ID=75 only, no hardware."""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from unittest.mock import patch

if os.environ.get('ROS_DOMAIN_ID')!='75': raise SystemExit('fake ROS checks require isolated ROS_DOMAIN_ID=75')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from tools.run_blockly_station import Application

context=Context(); rclpy.init(context=context)
node=Node('blockly_fake_station',context=context); group=ReentrantCallbackGroup()
state={'workflow':{'state':'idle','job_id':None,'result':None},
       'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True},
       'ev3':{'status':{'color':'Color.RED'}}}
counts={'arm':0,'jaw':0,'stop':0}; points=[]; publishing=[True]; slow=[False]
def trigger(endpoint,request,response):
    response.success=True
    if endpoint=='/follower/status': data={'schema':1,'kind':'follower_status','position_age_s':.01,'torque_age_s':.01,'fresh':True,'sampled_monotonic':time.monotonic(),'arm_id':'fake','calibration_sha256':'a'*64,'owner_started_monotonic':123.,'owner_source_sha256':{'owner':'b'*64},'owner_faulted':False,'stop_latched':False,'active_operation':None,'teleop_state':'disabled','torque':dict.fromkeys(['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'],0)}
    elif endpoint=='/station/status': data=state
    elif endpoint=='/follower/gripper_calibration_status': data={'si_gap_calibrated':True,'moveit_joint_calibrated':True}
    elif endpoint=='/station/stop': counts['stop']+=1; state['stop']['request_id']='stop-'+str(counts['stop']); data={'cancel_requested':True,'request_id':state['stop']['request_id']}
    elif endpoint=='/station/ev3/eject':
        state['workflow']={'state':'idle','job_id':'fake-eject','result':{'done':True}}; data={'job_id':'fake-eject'}
    else: data={'acknowledged':True}
    response.message=json.dumps(data); return response
services=[]
for endpoint in ('/follower/status','/station/status','/station/stop','/follower/gripper_calibration_status','/station/ev3/eject','/station/operator/moveit'):
    services.append(node.create_service(Trigger,endpoint,lambda request,response,e=endpoint:trigger(e,request,response),callback_group=group))

def arm_execute(handle):
    counts['arm']+=1; points.append(handle.request.trajectory.points)
    end=time.monotonic()+(.8 if slow[0] else .03)
    while time.monotonic()<end:
        if handle.is_cancel_requested:
            handle.canceled(); return FollowJointTrajectory.Result(error_code=0)
        time.sleep(.01)
    handle.succeed(); return FollowJointTrajectory.Result(error_code=0)
def jaw_execute(handle):
    counts['jaw']+=1; handle.succeed(); return GripperCommand.Result(position=handle.request.command.position,reached_goal=True,stalled=False)
arm=ActionServer(node,FollowJointTrajectory,'/follower/arm_trajectory_controller/follow_joint_trajectory',arm_execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
jaw=ActionServer(node,GripperCommand,'/follower/gripper_controller/gripper_cmd',jaw_execute,callback_group=group)
pub=node.create_publisher(JointState,'/follower/joint_states',10)
compact=node.create_publisher(String,'/station/ev3/operator_status',10)
operator=node.create_publisher(String,'/station/operator/status',10)
def publish():
    if not publishing[0]: return
    message=JointState(); message.header.stamp=node.get_clock().now().to_msg()
    message.name=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']; message.position=[0.]*6
    pub.publish(message); compact.publish(String(data=json.dumps({'fresh':True,'connected':True,'session':'fake-session','color':'Color.RED'})))
    operator.publish(String(data=json.dumps({'rgb':'fresh','depth':'fresh','tags':'fresh'})))
node.create_timer(.05,publish,callback_group=group)
executor=MultiThreadedExecutor(num_threads=3,context=context); executor.add_node(node)
def spin():
    from rclpy.executors import ExternalShutdownException, ShutdownException
    try: executor.spin()
    except (ExternalShutdownException,ShutdownException): pass
thread=threading.Thread(target=spin,daemon=True); thread.start()
app=None
try:
    with tempfile.TemporaryDirectory() as directory:
        # Fixture owns no physical station: isolate its local program lock as well as ROS domain.
        with patch('tools.run_blockly_station.Path', wraps=Path) as fixture_path:
            fixture_path.home.return_value=Path(directory)
            app=Application(directory,physical=True,instructor=True)
        deadline=time.monotonic()+5
        while True:
            try: app.backend.sample(); break
            except RuntimeError:
                if time.monotonic()>deadline: raise
                time.sleep(.05)
        app.record_start('transfer'); time.sleep(.65); recorded=app.record_stop()
        assert recorded['samples']>=4,recorded
        reply=app.run([{'op':'ev3','command':'eject'},{'op':'replay','name':'transfer','speed':.5},{'op':'jaw','gap_mm':30}])
        deadline=time.monotonic()+6
        while app.status()['job']['state']=='running' and time.monotonic()<deadline: time.sleep(.03)
        assert app.status()['job']['state']=='complete',app.status()
        assert counts['arm']==1 and counts['jaw']==1,counts
        assert points[0][-1].time_from_start.sec+points[0][-1].time_from_start.nanosec*1e-9>.8
        original_duration=points[0][-1].time_from_start.sec+points[0][-1].time_from_start.nanosec*1e-9
        app.thread.join(timeout=2)
        app.run([{'op':'replay','name':'transfer','speed':.5,'timing':'skip_stationary_pauses'}])
        deadline=time.monotonic()+6
        while app.status()['job']['state']=='running' and time.monotonic()<deadline: time.sleep(.03)
        assert app.status()['job']['state']=='complete',app.status()
        assert len(points[1])==4
        compressed_duration=points[1][-1].time_from_start.sec+points[1][-1].time_from_start.nanosec*1e-9
        assert compressed_duration<original_duration
        assert app.backend.color()=='red'
        assert app.operator('moveit')['acknowledged'] is True
        publishing[0]=False; time.sleep(.4)
        try: app.backend.sample(); raise AssertionError('stale state accepted')
        except RuntimeError: pass
        publishing[0]=True; time.sleep(.15); slow[0]=True
        app.run([{'op':'replay','name':'transfer','speed':.5}]); time.sleep(.15); app.stop()
        deadline=time.monotonic()+3
        while app.status()['job']['state']=='running' and time.monotonic()<deadline: time.sleep(.03)
        assert app.status()['job']['state']=='failed'; assert counts['stop']>=1
        print(json.dumps({'fake_ros':'passed','recorded_samples':recorded['samples'],'counts':counts,'stale_rejected':True,'cancel_stopped':True,'optional_timing_shortened':True}))
finally:
    if app: app.close()
    for timer in tuple(node.timers): timer.cancel()
    executor.remove_node(node); time.sleep(.1)
    executor.shutdown(timeout_sec=2); thread.join(timeout=2); node.destroy_node(); context.shutdown()
