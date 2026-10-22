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
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from so101_station_workflow.srv import RunCycles
from tools.run_blockly_station import Application
from workshop.blockly_ros import OPERATOR_COMMANDS

context=Context(); rclpy.init(context=context)
node=Node('blockly_fake_station',context=context); group=ReentrantCallbackGroup()
state={'workflow':{'state':'idle','job_id':None,'result':None},
       'stop':{'pending':False,'ev3_stopped':True,'arm_disarmed':True},
       'ev3':{'status':{'color':'Color.RED'}}}
counts={'arm':0,'jaw':0,'stop':0,'sort':0}; points=[]; publishing=[True]; slow=[False]
calibration={'si':True,'angle':True}; calibration_replies=[]; jaw_positions=[]; operator_calls=[]; ev3_fault_reset_calls=[]
def trigger(endpoint,request,response):
    response.success=True
    if endpoint=='/follower/status': data={'schema':1,'kind':'follower_status','position_age_s':.01,'torque_age_s':.01,'fresh':True,'sampled_monotonic':time.monotonic(),'arm_id':'fake','calibration_sha256':'a'*64,'owner_started_monotonic':123.,'owner_source_sha256':{'owner':'b'*64},'owner_faulted':False,'stop_latched':False,'active_operation':None,'teleop_state':'disabled','torque':dict.fromkeys(['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'],0)}
    elif endpoint=='/station/status': data=state
    elif endpoint=='/follower/gripper_calibration_status':
        si,angle=calibration['si'],calibration['angle']
        response.success=si and angle
        data={'schema':'so101-gripper-calibration-status-v1','si_gap_calibrated':si,'moveit_joint_calibrated':angle,'si_position_units':'metres','moveit_position_units':'radians','visual_proxy_only':not angle,'reason':'qualified gap and joint-angle calibration loaded' if si and angle else 'calibration incomplete'}
        calibration_replies.append((si,angle,response.success))
    elif endpoint=='/station/stop': counts['stop']+=1; state['stop']['request_id']='stop-'+str(counts['stop']); data={'cancel_requested':True,'request_id':state['stop']['request_id']}
    elif endpoint=='/station/reset_fault':
        ev3_fault_reset_calls.append(endpoint); data={'acknowledged':True}
    elif endpoint.startswith('/station/operator/'):
        operator_calls.append(endpoint.removeprefix('/station/operator/')); data={'acknowledged':True}
    elif endpoint=='/station/ev3/eject':
        state['workflow']={'state':'idle','job_id':'fake-eject','result':{'done':True}}; data={'job_id':'fake-eject'}
    else: data={'acknowledged':True}
    response.message=json.dumps(data); return response
services=[]
for endpoint in ('/follower/status','/station/status','/station/stop','/follower/gripper_calibration_status',
                 '/station/ev3/eject','/station/reset_fault',*('/station/operator/'+command for command in OPERATOR_COMMANDS)):
    services.append(node.create_service(Trigger,endpoint,lambda request,response,e=endpoint:trigger(e,request,response),callback_group=group))


def run_cycle(request,response):
    assert request.cycles==1
    counts['sort']+=1; job='fake-sort-'+str(counts['sort'])
    state['workflow']={'state':'sorting','job_id':job,'fault':None,'result':None}
    def finish():
        time.sleep(.15)
        state['workflow']={'state':'idle','job_id':job,'fault':None,'result':{'completed':1,'colors':{'red':1}}}
    threading.Thread(target=finish,daemon=True).start()
    response.success=True;response.message=json.dumps({'job_id':job});return response
services.append(node.create_service(RunCycles,'/station/run_cycles',run_cycle,callback_group=group))

def arm_execute(handle):
    counts['arm']+=1; points.append(handle.request.trajectory.points)
    end=time.monotonic()+(.8 if slow[0] else .03)
    while time.monotonic()<end:
        if handle.is_cancel_requested:
            handle.canceled(); return FollowJointTrajectory.Result(error_code=0)
        time.sleep(.01)
    handle.succeed(); return FollowJointTrajectory.Result(error_code=0)
def jaw_execute(handle):
    counts['jaw']+=1; jaw_positions.append(handle.request.command.position)
    handle.succeed(); return GripperCommand.Result(position=handle.request.command.position,reached_goal=True,stalled=False)
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
        def await_calibration(si,angle):
            deadline=time.monotonic()+5
            pause=threading.Event()
            while True:
                status=app.backend.status()
                snapshot=status['streams'].get('gripper_calibration',{})
                data=snapshot.get('data',{})
                if (0<=snapshot.get('age_s',99)<=2
                        and data.get('si_gap_calibrated') is si
                        and data.get('moveit_joint_calibrated') is angle
                        and status['capabilities']['jaw'] is si):
                    return
                if time.monotonic()>=deadline:
                    raise AssertionError(('calibration observation deadline',si,angle,status))
                pause.wait(.02)
        calibration.update(si=True,angle=False)
        await_calibration(True,False)
        app.run([{'op':'jaw','gap_mm':30}])
        deadline=time.monotonic()+6
        while app.status()['job']['state']=='running' and time.monotonic()<deadline:
            threading.Event().wait(.02)
        assert app.status()['job']['state']=='complete',app.status()
        app.thread.join(timeout=2)
        assert not app.thread.is_alive()
        assert counts['jaw']==1 and len(jaw_positions)==1,(counts,jaw_positions)
        assert math.isclose(jaw_positions[0],.03,rel_tol=0.,abs_tol=1e-12),jaw_positions
        assert (True,False,False) in calibration_replies,calibration_replies
        si_only_outcome={'native_flags':[True,False,False],'jaw_goals':counts['jaw'],'position_metres':jaw_positions[0]}
        calibration.update(si=False,angle=False)
        await_calibration(False,False)
        try:
            app.backend.command({'op':'jaw','gap_mm':30},threading.Event(),time.monotonic()+5)
        except RuntimeError as error:
            assert str(error)=='qualified SI jaw calibration is missing',str(error)
        else:
            raise AssertionError('uncalibrated SI jaw accepted')
        assert counts['jaw']==1 and len(jaw_positions)==1,(counts,jaw_positions)
        assert (False,False,False) in calibration_replies,calibration_replies
        calibration.update(si=True,angle=True)
        await_calibration(True,True)
        counts['jaw']=0; jaw_positions.clear()
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
        app.thread.join(timeout=2)
        app.run([{'op':'sort_cycle'}])
        deadline=time.monotonic()+6
        while app.status()['job']['state']=='running' and time.monotonic()<deadline: time.sleep(.03)
        assert app.status()['job']['state']=='complete',app.status()
        assert app.status()['job']['result']['sort_cycles']==[{'completed':1,'colors':{'red':1}}]
        assert counts['sort']==1
        assert app.backend.color()=='red'
        assert app.operator('moveit')['acknowledged'] is True
        def await_program():
            deadline=time.monotonic()+6
            pause=threading.Event()
            while app.status()['job']['state']=='running' and time.monotonic()<deadline:
                pause.wait(.02)
            assert app.status()['job']['state']=='complete',app.status()
            app.thread.join(timeout=2)
            assert not app.thread.is_alive()
        app.thread.join(timeout=2)
        operator_begin=len(operator_calls); arm_before=counts['arm']; jaw_before=counts['jaw']
        app.run([{'op':'arm','command':'teleop_relative'},
                 {'op':'record_stroke','name':'block_taught','seconds':1},
                 {'op':'arm','command':'moveit'}])
        await_program()
        assert operator_calls[operator_begin:]==['teleop_relative','teleop','moveit'],operator_calls
        block_recording=app.recordings['block_taught']
        block_samples=len(block_recording['points'])
        assert block_samples>=8,block_samples
        saved_path=Path(directory)/'block_taught.json'
        assert saved_path.is_file()
        assert json.loads(saved_path.read_text())==block_recording
        assert counts['arm']==arm_before and counts['jaw']==jaw_before,counts
        assert block_recording['jaw_radians']==0.
        app.run([{'op':'replay','name':'block_taught','speed':.5}])
        await_program()
        assert counts['arm']==arm_before+1 and counts['jaw']==jaw_before,counts
        assert len(points)==counts['arm']
        assert all(isinstance(point,JointTrajectoryPoint) for point in points[-1])
        assert len(points[-1])==block_samples
        assert len(points[-1][0].positions)==5
        operator_begin=len(operator_calls)
        app.run([{'op':'arm','command':'teleop_absolute'},
                 {'op':'arm','command':'moveit'}, {'op':'arm','command':'home'}])
        await_program()
        assert operator_calls[operator_begin:]==['teleop_absolute','teleop','moveit','start'],operator_calls
        operator_begin=len(operator_calls); arm_before=counts['arm']; jaw_before=counts['jaw']
        app.run([{'op':'station','command':command} for command in OPERATOR_COMMANDS])
        await_program()
        assert operator_calls[operator_begin:]==list(OPERATOR_COMMANDS),operator_calls
        app.recovery_required=True; app.record_error='previous fake recording failed'
        operator_begin=len(operator_calls)
        app.run([{'op':'station','command':'reinitialize'}])
        await_program()
        assert operator_calls[operator_begin:]==['emergency_reset'],operator_calls
        assert app.recovery_required is False and app.record_error is None
        app.recovery_required=True; app.record_error='previous fake recording failed'
        operator_begin=len(operator_calls)
        app.run([{'op':'station','command':'verify_recovery'}])
        await_program()
        assert len(operator_calls)==operator_begin
        assert app.recovery_required is False and app.record_error is None
        assert counts['arm']==arm_before and counts['jaw']==jaw_before,counts
        app.run([{'op':'ev3','command':'reset_fault'}])
        await_program()
        assert ev3_fault_reset_calls==['/station/reset_fault']
        publishing[0]=False; time.sleep(.4)
        try: app.backend.sample(); raise AssertionError('stale state accepted')
        except RuntimeError: pass
        publishing[0]=True; time.sleep(.15); slow[0]=True
        app.run([{'op':'replay','name':'transfer','speed':.5}]); time.sleep(.15); app.stop()
        deadline=time.monotonic()+3
        while app.status()['job']['state']=='running' and time.monotonic()<deadline: time.sleep(.03)
        assert app.status()['job']['state']=='failed'; assert counts['stop']>=1
        print(json.dumps({'fake_ros':'passed','recorded_samples':recorded['samples'],'counts':counts,'stale_rejected':True,'cancel_stopped':True,'optional_timing_shortened':True,'native_cycle_exact_terminal':True,'si_only':si_only_outcome,'uncalibrated_si_rejected_without_goal':True,'typed_arm_controls':True,'program_recording':True,'program_recording_replay':True,'program_recorded_samples':block_samples,'typed_station_controls':True,'station_reinitialize_verified':True,'station_verify_recovery_verified':True,'typed_ev3_global_fault_reset':True}))
finally:
    if app: app.close()
    for timer in tuple(node.timers): timer.cancel()
    executor.remove_node(node); time.sleep(.1)
    executor.shutdown(timeout_sec=2); thread.join(timeout=2); node.destroy_node(); context.shutdown()
