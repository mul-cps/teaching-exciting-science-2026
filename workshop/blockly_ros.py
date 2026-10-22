"""ROS clients for Blockly. Reuses the sole resident owner and guarded APIs."""
from types import MappingProxyType
import hashlib
import json
import math
import threading
import time
from .blockly_recording import JOINTS, replay_points
from .blockly_program import STATION

OPERATOR_COMMANDS=('emergency_reset','reset','start','base_release','base_confirm','teleop',
 'teleop_relative','teleop_absolute','moveit','damping','zero_torque','restart_moveit',
 'restart_camera','restart_depth','restart_tags','restart_all','verify_restart')
ARM_SERVICES=MappingProxyType({'home':('start',), 'moveit':('moveit',),
 'teleop_relative':('teleop_relative','teleop'),
 'teleop_absolute':('teleop_absolute','teleop')})
EV3_SERVICES={key:'/station/'+value for key,value in {
 'connect':'ev3/connect','home':'ev3/home','left_end':'ev3/left_end',
 'right_end':'ev3/right_end','eject':'ev3/eject','wake':'wake','sleep':'sleep',
 'reset_fault':'reset_fault'}.items()}


def terminal_job(snapshot, job_id):
    """Service acceptance is not completion; require the exact workflow job."""
    workflow=snapshot.get('workflow') if isinstance(snapshot,dict) else None
    if not isinstance(workflow,dict): raise RuntimeError('native workflow status is unavailable')
    if workflow.get('job_id') != job_id: raise RuntimeError('station job identity changed or is unavailable')
    if workflow.get('state')=='fault' or workflow.get('fault') is not None: raise RuntimeError('station job failed: '+str(workflow.get('fault')))
    return workflow.get('state')=='idle' and workflow.get('result') is not None


def validate_sort_result(result):
    """Native run_sorting counts only independently verified complete cycles."""
    if not isinstance(result,dict) or len(result)!=2 or set(result)!={'completed','colors'} or type(result.get('completed')) is not int or result['completed']!=1:
        raise RuntimeError('sorting workflow did not verify exactly one completed cycle')
    colors=result.get('colors')
    if (not isinstance(colors,dict) or not 1<=len(colors)<=4 or set(colors)-{'red','blue','green','yellow'}
            or any(type(value) is not int or value<0 for value in colors.values()) or sum(colors.values())!=1):
        raise RuntimeError('sorting workflow returned invalid one-cycle color counts')
    return {'completed':1,'colors':dict(colors)}


def parse_gripper_calibration_status(result):
    """Validate the bounded read-only native status, including combined success."""
    if result is None or not isinstance(result.message,str) or len(result.message)>8192:
        raise RuntimeError('invalid gripper calibration status')
    def unique_fields(pairs):
        data={}
        for key,value in pairs:
            if key in data: raise ValueError('duplicate calibration status field')
            data[key]=value
        return data
    try: data=json.loads(result.message,object_pairs_hook=unique_fields)
    except (ValueError,TypeError,RecursionError) as error: raise RuntimeError('invalid gripper calibration JSON') from error
    if (not isinstance(data,dict) or data.get('schema')!='so101-gripper-calibration-status-v1'
            or any(type(data.get(key)) is not bool for key in
                   ('si_gap_calibrated','moveit_joint_calibrated','visual_proxy_only'))
            or data.get('si_position_units')!='metres' or data.get('moveit_position_units')!='radians'
            or type(result.success) is not bool
            or data.get('visual_proxy_only') is not (not data.get('moveit_joint_calibrated'))
            or result.success is not (data.get('si_gap_calibrated') and data.get('moveit_joint_calibrated'))
            or ('reason' in data and (not isinstance(data['reason'],str) or len(data['reason'])>1000))):
        raise RuntimeError('invalid gripper calibration status schema')
    return data


class ROSBackend:
    physical=True
    def __init__(self, recordings):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        from rclpy.action import ActionClient
        self.ros=rclpy; self.Trigger=Trigger; self.FJT=FollowJointTrajectory; self.Gripper=GripperCommand
        rclpy.init(); self.node=Node('blockly_station_clients'); group=ReentrantCallbackGroup()
        self.executor=SingleThreadedExecutor(); self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self._spin,daemon=True)
        self.lock=threading.Lock(); self.latest=None; self.cache={}; self.clients={}; self.recordings=recordings
        self.dispatch_lock=threading.Lock(); self.action_lock=threading.Lock(); self.active_handle=None; self.pending_cancel=None; self.pending_mutations=set(); self.stop_lock=threading.Lock()
        self.node.create_subscription(JointState,'/follower/joint_states',self._joints,10,callback_group=group)
        for topic in ('/station/operator/status','/follower/teleop_status','/station/ev3/operator_status'):
            self.node.create_subscription(String,topic,lambda msg,t=topic:self._status_message(t,msg),10,callback_group=group)
        self.arm=ActionClient(self.node,FollowJointTrajectory,'/follower/arm_trajectory_controller/follow_joint_trajectory',callback_group=group)
        self.jaw=ActionClient(self.node,GripperCommand,'/follower/gripper_controller/gripper_cmd',callback_group=group)
        self.identity_client=self.node.create_client(Trigger,'/follower/status',callback_group=group)
        self.identity_pending=None; self.identity_pending_at=0.; self.owner_cache=None; self.calibration_pending=None; self.calibration_pending_at=0.
        self.calibration_client=self.node.create_client(Trigger,'/follower/gripper_calibration_status',callback_group=group)
        self.node.create_timer(1.,self._refresh_calibration,callback_group=group)
        self.node.create_timer(.05,self._refresh_owner,callback_group=group)
        self.thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException, ShutdownException
        try: self.executor.spin()
        except (ExternalShutdownException,ShutdownException): pass

    def _status_message(self,topic,msg):
        if len(msg.data)>32768: return
        try: payload=json.loads(msg.data)
        except (ValueError,TypeError): return
        if not isinstance(payload,dict): return
        with self.lock: self.cache[topic]=(time.monotonic(),payload)

    def _joints(self,msg):
        if len(msg.name)>16 or len(msg.position)!=len(msg.name) or any(not isinstance(n,str) or len(n)>64 for n in msg.name): return
        if len(set(msg.name))!=len(msg.name): return
        values=dict(zip(msg.name,msg.position))
        if not set(JOINTS+('gripper',)).issubset(values): return
        positions=[values[j] for j in JOINTS]
        if any(not math.isfinite(v) for v in positions+[values['gripper']]): return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.node.get_clock().now().nanoseconds*1e-9-stamp
        if not 0<=age<=.25: return
        with self.lock:
            if self.latest and stamp<=self.latest['stamp']: return
            self.latest={'positions':positions,'jaw':values['gripper'],'stamp':stamp,'received':time.monotonic()}

    def _refresh_calibration(self):
        if self.calibration_pending is not None and not self.calibration_pending.done():
            if time.monotonic()-self.calibration_pending_at<1.: return
            self.calibration_client.remove_pending_request(self.calibration_pending)
            self.calibration_pending=None
        if not self.calibration_client.service_is_ready(): return
        future=self.calibration_client.call_async(self.Trigger.Request()); self.calibration_pending=future; self.calibration_pending_at=time.monotonic()
        requested_at=self.calibration_pending_at
        def received(reply):
            try:
                if reply is not self.calibration_pending or reply.cancelled(): return
                if not 0<=time.monotonic()-requested_at<=1.: return
                data=parse_gripper_calibration_status(reply.result())
                with self.lock:
                    if reply is self.calibration_pending and not reply.cancelled() and 0<=time.monotonic()-requested_at<=1.:
                        self.cache['gripper_calibration']=(time.monotonic(),data)
            except (ValueError,TypeError,RuntimeError,KeyError): return
        future.add_done_callback(received)

    def _refresh_owner(self):
        if self.identity_pending is not None and not self.identity_pending.done():
            if time.monotonic()-self.identity_pending_at<.5: return
            self.identity_client.remove_pending_request(self.identity_pending)
            self.identity_pending=None
        if not self.identity_client.service_is_ready(): return
        future=self.identity_client.call_async(self.Trigger.Request()); self.identity_pending=future; self.identity_pending_at=time.monotonic()
        def received(reply):
            try:
                result=reply.result()
                if result is None or not result.success or len(result.message)>32768: return
                data=json.loads(result.message)
                self._identity(data)
                with self.lock: self.owner_cache=(time.monotonic(),data)
            except (ValueError,TypeError,RuntimeError,KeyError): return
        future.add_done_callback(received)

    @staticmethod
    def _identity(status):
        if status.get('schema')!=1 or status.get('kind')!='follower_status': raise RuntimeError('unsupported owner status schema')
        sampled=status.get('sampled_monotonic')
        if type(sampled) not in (int,float) or not math.isfinite(sampled) or not 0<=time.monotonic()-sampled<=.3:
            raise RuntimeError('owner status source snapshot is stale or future')
        if status.get('fresh') is not True: raise RuntimeError('fresh identity-matched follower state required')
        elapsed=time.monotonic()-sampled
        for field in ('position_age_s','torque_age_s'):
            age=status.get(field)
            if type(age) not in (int,float) or not math.isfinite(age) or age<0 or elapsed+age>.3:
                raise RuntimeError('owner measurements are stale')
        source=status.get('owner_source_sha256')
        generation=status.get('owner_started_monotonic')
        if not isinstance(source,dict) or not source or type(generation) not in (int,float) or not math.isfinite(generation):
            raise RuntimeError('owner generation/source identity unavailable')
        return {'arm_id':status['arm_id'],'calibration_sha256':status['calibration_sha256'],
                'owner_generation':generation,'source_sha256':hashlib.sha256(json.dumps(source,sort_keys=True,separators=(',',':')).encode()).hexdigest()}

    def sample(self):
        with self.lock:
            sample=dict(self.latest) if self.latest else None
            owner=self.owner_cache
        if sample is None or time.monotonic()-sample['received']>.25: raise RuntimeError('follower joint feedback stale')
        if owner is None or time.monotonic()-owner[0]>.5: raise RuntimeError('owner identity/status stale')
        sample['identity']=self._identity(owner[1])
        return sample

    def _wait(self,future,cancel,deadline):
        while True:
            if cancel.is_set(): raise InterruptedError('program cancelled; command delivery may be unknown')
            if time.monotonic()>=deadline: raise TimeoutError('ROS response deadline; command delivery may be unknown')
            if future.done(): break
            cancel.wait(.02)
        result=future.result()
        if result is None: raise RuntimeError('ROS call returned no result')
        return result

    def _track(self,future):
        with self.lock: self.pending_mutations.add(future)
        def done(_):
            try: future.result()
            except Exception: pass
            with self.lock: self.pending_mutations.discard(future)
        future.add_done_callback(done)
        return future

    def trigger(self,endpoint,cancel=None,deadline=None):
        return self._service(endpoint,self.Trigger,self.Trigger.Request(),cancel,deadline)

    def _service(self,endpoint,service_type,request,cancel=None,deadline=None):
        cancel=cancel or threading.Event(); deadline=deadline or time.monotonic()+5
        with self.lock:
            client=self.clients.get(endpoint)
            if client is None:
                client=self.node.create_client(service_type,endpoint); self.clients[endpoint]=client
        while not client.service_is_ready():
            if cancel.is_set() or time.monotonic()>=deadline: raise RuntimeError(endpoint+' unavailable')
            cancel.wait(.02)
        mutation=endpoint not in {'/follower/status','/station/status','/follower/gripper_calibration_status','/station/stop'}
        with self.dispatch_lock:
            with self.lock:
                if mutation and self.pending_mutations: raise RuntimeError('previous mutating request/goal is unresolved')
            if cancel.is_set() or time.monotonic()>=deadline: raise InterruptedError('call cancelled before dispatch')
            future=client.call_async(request)
        if mutation: self._track(future)
        try: result=self._wait(future,cancel,deadline)
        except BaseException:
            if not mutation: client.remove_pending_request(future)
            raise
        if mutation:
            with self.lock: self.pending_mutations.discard(future)
        if endpoint=='/follower/gripper_calibration_status': return parse_gripper_calibration_status(result)
        if not result.success: raise RuntimeError(endpoint+': '+result.message[:1000])
        try: return json.loads(result.message)
        except ValueError: return {'message':result.message[:1000]}

    def identity(self):
        return self._identity(self.trigger('/follower/status'))

    def color(self):
        with self.lock: compact=self.cache.get('/station/ev3/operator_status')
        if (not compact or time.monotonic()-compact[0]>1.5 or compact[1].get('fresh') is not True
                or compact[1].get('connected') is not True or not compact[1].get('session')):
            raise RuntimeError('color feedback is stale or EV3 session unavailable; no branch chosen')
        color=str(compact[1].get('color')).split('.')[-1].lower()
        return color if color in {'red','blue','green','yellow'} else 'unknown'

    def log(self,message): self.node.get_logger().info(message)

    def _action(self,client,goal,cancel,deadline):
        while not client.server_is_ready():
            if cancel.is_set() or time.monotonic()>=deadline: raise RuntimeError('guarded action unavailable')
            cancel.wait(.02)
        if cancel.is_set() or time.monotonic()>=deadline: raise InterruptedError('action cancelled before dispatch')
        marker=object()
        with self.lock: self.pending_mutations.add(marker)
        try:
            with self.dispatch_lock:
                with self.lock:
                    if self.pending_mutations-{marker}: raise RuntimeError('previous mutating request/goal is unresolved')
                if cancel.is_set() or time.monotonic()>=deadline: raise InterruptedError('action cancelled before dispatch')
                future=client.send_goal_async(goal)
        except BaseException:
            with self.lock: self.pending_mutations.discard(marker)
            raise
        uncertain=threading.Event(); late_lock=threading.Lock(); late_processed=[False]
        def late_reply(reply):
            if uncertain.is_set():
                with late_lock:
                    if late_processed[0]: return
                    late_processed[0]=True
                try:
                    late=reply.result()
                    if late is not None and late.accepted:
                        with self.action_lock: self.active_handle=late
                        terminal=self._track(late.get_result_async())
                        terminal.add_done_callback(lambda _:self._clear_handle(late))
                        self._cancel_handle(late)
                except Exception: pass
                finally:
                    with self.lock: self.pending_mutations.discard(marker)
        future.add_done_callback(late_reply)
        try: handle=self._wait(future,cancel,deadline)
        except BaseException:
            uncertain.set()
            if future.done(): late_reply(future)
            raise
        if not handle.accepted:
            with self.lock: self.pending_mutations.discard(marker)
            raise RuntimeError('controller rejected goal; inspect RViz diagnostics')
        with self.action_lock: self.active_handle=handle
        try:
            terminal=self._track(handle.get_result_async())
            terminal.add_done_callback(lambda _:self._clear_handle(handle))
            with self.lock: self.pending_mutations.discard(marker)
            wrapped=self._wait(terminal,cancel,deadline)
            if wrapped.status!=4: raise RuntimeError(f'action did not succeed (status {wrapped.status})')
            return wrapped.result
        finally:
            with self.action_lock:
                # Keep a handle on uncertain delivery so STOP can still cancel it.
                if 'wrapped' in locals(): self.active_handle=None

    def _cancel_handle(self,handle):
        with self.action_lock:
            pending=getattr(self,'pending_cancel',None)
            if pending and pending[0] is handle and not pending[1].done(): return
            future=handle.cancel_goal_async()
            self.pending_cancel=(handle,future)
            self._track(future)

    def _clear_handle(self,handle):
        with self.action_lock:
            if self.active_handle is handle: self.active_handle=None
            pending=self.pending_cancel
            if pending and pending[0] is handle:
                # A terminal action cannot execute later; discard an unanswered cancellation RPC.
                if not pending[1].done(): pending[1].cancel()
                self.pending_cancel=None

    def _wait_job(self,job,cancel,deadline):
        if not isinstance(job,str) or not 1<=len(job)<=128: raise RuntimeError('native workflow acknowledgement lacks exact job ID')
        while True:
            if cancel.is_set(): raise InterruptedError('cancelled during native workflow')
            if time.monotonic()>=deadline: raise TimeoutError('native workflow deadline expired')
            snapshot=self.trigger('/station/status',cancel,min(deadline,time.monotonic()+3))
            if terminal_job(snapshot,job): return snapshot['workflow']['result']
            if cancel.wait(.1): raise InterruptedError('cancelled during native workflow')

    def command(self,row,cancel,deadline):
        if row['op']=='record_stroke': raise ValueError('record_stroke requires Application dispatch')
        if row['op']=='station':
            command=row['command']
            if not isinstance(command,str) or command not in STATION: raise ValueError('station command not allowed')
            if cancel.is_set(): raise InterruptedError('program cancelled before station command')
            if time.monotonic()>=deadline: raise TimeoutError('station command deadline expired')
            if command=='verify_recovery': return self.recover(cancel=cancel,deadline=deadline)
            reply=self.trigger('/station/operator/'+('emergency_reset' if command=='reinitialize' else command),cancel,deadline)
            if command=='reinitialize': return self.recover(cancel=cancel,deadline=deadline)
            return reply
        if row['op']=='arm':
            if cancel.is_set(): raise InterruptedError('program cancelled before arm command')
            if time.monotonic()>=deadline: raise TimeoutError('arm command deadline expired')
            if row['command']=='torque_off': return self.stop()
            for command in ARM_SERVICES[row['command']]:
                self.trigger('/station/operator/'+command,cancel,deadline)
            return
        if row['op']=='sort_cycle':
            from so101_station_workflow.srv import RunCycles
            request=RunCycles.Request(); request.cycles=1
            reply=self._service('/station/run_cycles',RunCycles,request,cancel,min(deadline,time.monotonic()+8))
            if not isinstance(reply,dict): raise RuntimeError('invalid sorting workflow acknowledgement')
            return validate_sort_result(self._wait_job(reply.get('job_id'),cancel,deadline))
        if row['op']=='ev3':
            reply=self.trigger(EV3_SERVICES[row['command']],cancel,min(deadline,time.monotonic()+8))
            job=reply.get('job_id')
            if row['command'] in {'home','left_end','right_end','eject'} and (not isinstance(job,str) or not 1<=len(job)<=128):
                raise RuntimeError('EV3 motion acknowledgement lacks exact workflow job ID')
            if job:
                if not isinstance(job,str) or len(job)>128: raise RuntimeError('invalid EV3 workflow job ID')
                self._wait_job(job,cancel,deadline)
            return
        if row['op']=='jaw':
            status=self.trigger('/follower/gripper_calibration_status',cancel,min(deadline,time.monotonic()+3))
            if status.get('si_gap_calibrated') is not True: raise RuntimeError('qualified SI jaw calibration is missing')
            goal=self.Gripper.Goal(); goal.command.position=row['gap_mm']/1000.; goal.command.max_effort=0.
            result=self._action(self.jaw,goal,cancel,deadline)
            if not result.reached_goal or result.stalled: raise RuntimeError('jaw did not reach requested gap')
            return
        if row['op']=='replay':
            from trajectory_msgs.msg import JointTrajectoryPoint
            from builtin_interfaces.msg import Duration
            recording=self.recordings.get(row['name'])
            if recording is None: raise RuntimeError('recording not found')
            identity=self.identity()
            sample=self.sample()
            points=replay_points(recording,identity,row['speed'],timing=row.get('timing','original'),
                                 start_radians=dict(zip(JOINTS,sample['positions'])))
            if sample['identity']!=recording['identity']: raise RuntimeError('owner cohort changed during replay admission')
            if max(abs(a-b) for a,b in zip(sample['positions'],points[0]['positions']))>math.radians(2):
                raise RuntimeError('move to recorded start under instructor supervision before replay')
            if abs(sample['jaw']-recording['jaw_radians'])>.015: raise RuntimeError('jaw differs from recorded fixed-jaw stroke')
            goal=self.FJT.Goal(); goal.trajectory.joint_names=list(JOINTS)
            for row in points:
                ns=round(row['time']*1e9); sec,nanosec=divmod(ns,1000000000)
                goal.trajectory.points.append(JointTrajectoryPoint(positions=row['positions'],time_from_start=Duration(sec=sec,nanosec=nanosec)))
            result=self._action(self.arm,goal,cancel,deadline)
            if result.error_code!=0: raise RuntimeError('trajectory failed: '+result.error_string)
            return
        raise ValueError('unsupported physical command')

    def stop(self):
        with self.stop_lock: return self._stop()

    def _stop(self):
        with self.action_lock: handle=self.active_handle
        cancel_error=None
        if handle is not None:
            # An action transport failure must never skip the independent station STOP.
            try: self._cancel_handle(handle)
            except Exception as error: cancel_error=str(error)[:256]
        deadline=time.monotonic()+15
        reply=self.trigger('/station/stop',deadline=deadline)
        request_id=reply.get('request_id')
        if not isinstance(request_id,str) or not request_id: raise RuntimeError('station STOP API lacks correlation; update station API before physical Blockly')
        while True:
            status=self.trigger('/station/status',deadline=deadline).get('stop',{})
            if status.get('request_id')!=request_id:
                raise RuntimeError('STOP correlation changed')
            if status.get('pending') is False and status.get('ev3_stopped') is True and status.get('arm_disarmed') is True:
                if cancel_error: status=dict(status,action_cancel_error=cancel_error)
                return status
            if time.monotonic()>=deadline: raise RuntimeError('STOP terminal acknowledgements missing')
            time.sleep(.05)

    def recover(self,*,cancel=None,deadline=None):
        def check():
            if cancel is not None and cancel.is_set(): raise InterruptedError('program recovery cancelled')
            if deadline is not None and time.monotonic()>=deadline: raise TimeoutError('program recovery deadline expired')
        check()
        context={'cancel':cancel,'deadline':deadline} if cancel is not None or deadline is not None else {}
        with self.action_lock:
            if self.active_handle is not None: raise RuntimeError('previous action terminal result is unresolved')
        with self.lock:
            if self.pending_mutations: raise RuntimeError('mutating ROS request/goal delivery still unresolved; keep station stopped')
        self.stop()
        check()
        status=self.trigger('/follower/status',**context)
        self._identity(status)
        station=self.trigger('/station/status',**context)
        workflow=station.get('workflow')
        if not isinstance(workflow,dict) or workflow.get('state')!='idle' or workflow.get('fault') is not None:
            raise RuntimeError('native workflow is active/faulted; inspect and use RViz Reset inspected fault before recovery')
        stop=station.get('stop',{})
        torque=status.get('torque')
        if (not isinstance(torque,dict) or set(torque)!=set(JOINTS+('gripper',))
                or any(type(value) is not int or value!=0 for value in torque.values())):
            raise RuntimeError('fresh measured torque OFF required for all six joints')
        if (status.get('owner_faulted') is not False or status.get('stop_latched') is not False
                or status.get('active_operation') is not None or status.get('teleop_state')!='disabled'
                or status.get('operator_external_job') is not None or status.get('operator_maintenance_token') is not None
                or stop.get('pending') is not False or stop.get('ev3_stopped') is not True
                or stop.get('arm_disarmed') is not True):
            raise RuntimeError('explicit station recovery and fresh STOP acknowledgements required')
        check()
        return {'recovered':True}

    def instructor(self,command,*,cancel=None):
        if command not in OPERATOR_COMMANDS: raise ValueError('operator command not allowed')
        return self.trigger('/station/operator/'+command,deadline=time.monotonic()+125,**({'cancel':cancel} if cancel is not None else {}))

    def status(self):
        now=time.monotonic()
        with self.lock: snapshots={topic:{'age_s':now-stamp,'data':data} for topic,(stamp,data) in self.cache.items()}
        jaw=snapshots.get('gripper_calibration',{})
        return {'physical':True,'streams':snapshots,'capabilities':{'jaw':bool(0<=jaw.get('age_s',99)<=2 and jaw.get('data',{}).get('si_gap_calibrated') is True)}}

    def close(self):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            with self.lock: pending=bool(self.pending_mutations)
            if not pending: break
            time.sleep(.02)
        for timer in tuple(self.node.timers): timer.cancel()
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=2); self.thread.join(timeout=2); self.node.destroy_node(); self.ros.shutdown()
