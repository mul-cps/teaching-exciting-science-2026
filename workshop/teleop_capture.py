"""Local paired leader/follower capture; --run explicitly enables matched-pose teleop."""
import argparse
import gc
import hashlib
import json
import math
from pathlib import Path
import signal
import time
from workshop.choreography import JOINTS, Recorder

RATE = dict(zip(JOINTS, (60, 60, 60, 60, 90, 100)))

def normalized_position(raw, calibration, gripper):
    lo, hi = calibration['range_min'], calibration['range_max']
    return (raw-lo)*100/(hi-lo) if gripper else (raw-(lo+hi)/2)*360/4095

def bounded_goal(leader, leader_start, follower_start, previous, dt, bounds, rates):
    goal, limited = {}, False
    for k in previous:
        requested = follower_start[k] + leader[k] - leader_start[k]
        bounded = max(bounds[k][0], min(bounds[k][1], requested))
        goal[k] = max(previous[k]-rates[k]*dt, min(previous[k]+rates[k]*dt, bounded))
        limited |= abs(goal[k]-requested)>1e-6
    return goal, limited

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true')
    p.add_argument('--leader-port',required=True)
    p.add_argument('--follower-port',required=True)
    p.add_argument('--calibration-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--camera',default='/dev/video1')
    p.add_argument('--hz',type=int,default=125)
    p.add_argument('--duration',type=float,default=120)
    p.add_argument('--matched-pose',action='store_true',help='Confirm the arms were physically matched before capture')
    p.add_argument('--preflight',action='store_true',help='Identify/read both arms and camera; do not enable torque')
    a=p.parse_args()
    if not a.run and not a.preflight: p.error('Select --preflight or --run')
    if a.run and not a.preflight and not a.matched_pose: p.error('--run requires --matched-pose: physically match leader and follower first')
    if a.leader_port==a.follower_port or not 100<=a.hz<=200 or not math.isfinite(a.duration) or not 0<a.duration<=300: p.error('Invalid ports, frequency or duration')
    a.output.mkdir(parents=True,exist_ok=False)
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader.so_leader import SOLeader
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
    from lerobot.utils.robot_utils import precise_sleep
    import cv2
    follower=SOFollower(SOFollowerRobotConfig(port=a.follower_port,id='hs_rig01_follower',calibration_dir=a.calibration_dir,use_degrees=True))
    leader=SOLeader(SOLeaderTeleopConfig(port=a.leader_port,id='hs_rig01_leader',calibration_dir=a.calibration_dir,use_degrees=True))
    buses={'leader':leader.bus,'follower':follower.bus}
    calibrations={name:json.loads((a.calibration_dir/f'hs_rig01_{name}.json').read_text()) for name in buses}
    camera=writer=recorder=None; trace=[]; was_gc=gc.isenabled(); status='failed'; failure=None
    def interrupt(signum,frame): raise KeyboardInterrupt('Stop requested')
    signal.signal(signal.SIGTERM,interrupt)
    def read(name):
        begin=time.monotonic(); raw=buses[name].sync_read('Present_Position',normalize=False,num_retry=2); end=time.monotonic()
        pos={k:normalized_position(raw[k],calibrations[name][k],k=='gripper') for k in JOINTS}
        return raw,pos,(begin+end)/2,end-begin
    try:
        camera=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')); camera.set(cv2.CAP_PROP_FRAME_WIDTH,640); camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480); camera.set(cv2.CAP_PROP_FPS,25)
        for _ in range(10): ok,frame=camera.read()
        if not ok: raise RuntimeError('Camera preflight failed')
        cv2.imwrite(str(a.output/'before.jpg'),frame)
        metadata={'schema':1,'mapping':'matched physical pose offsets; body joint changes 1:1 degrees, gripper percent; full calibrated follower bounds','hz':a.hz,'duration_limit_s':a.duration,'units':'body degrees; gripper percent','ports':{'leader':a.leader_port,'follower':a.follower_port},'rate_limits':RATE,'calibration_sha256':{n:hashlib.sha256((a.calibration_dir/f'hs_rig01_{n}.json').read_bytes()).hexdigest() for n in buses},'register_units':'telemetry is SDK-decoded register values, no current-to-torque conversion'}
        for name,bus in buses.items():
            bus.connect()
            if not bus.is_calibrated: raise RuntimeError(f'{name} calibration mismatch')
            bus.disable_torque(num_retry=2)
            if any(bus.read('Torque_Enable',k,num_retry=2) for k in JOINTS): raise RuntimeError(f'{name} torque-off verification failed')
            metadata[name]={'models':{k:bus.ping(k,num_retry=2,raise_on_error=True) for k in JOINTS},'acceleration':{k:bus.read('Acceleration',k,num_retry=2) for k in JOINTS},'goal_velocity':{k:bus.read('Goal_Velocity',k,num_retry=2) for k in JOINTS}}
        lr,ls,_,_=read('leader'); _,fs,_,_=read('follower')
        initial_goal=dict(fs)
        metadata['initial_mapped_leader']=initial_goal
        metadata['matched_pose_offsets']={k:fs[k]-ls[k] for k in JOINTS}
        metadata['leader_start']=ls; metadata['follower_start']=fs
        bounds={}
        for k in JOINTS:
            c=calibrations['follower'][k]
            lo=normalized_position(c['range_min'],c,k=='gripper'); hi=normalized_position(c['range_max'],c,k=='gripper')
            bounds[k]=(lo,hi)
            if not bounds[k][0]<=fs[k]<=bounds[k][1]: raise RuntimeError(f'{k} outside calibrated range')
        metadata['bounds']=bounds
        (a.output/'metadata.json').write_text(json.dumps(metadata,indent=2))
        print('Both calibrated six-motor buses and webcam verified; torque OFF',flush=True)
        print('Matched pose offsets (follower minus leader):', {k:round(fs[k]-ls[k],2) for k in JOINTS},flush=True)
        if a.preflight: status='preflight'; return
        h,w=frame.shape[:2]; writer=cv2.VideoWriter(str(a.output/'dance.avi'),cv2.VideoWriter_fourcc(*'MJPG'),25,(w,h))
        if not writer.isOpened(): raise RuntimeError('Video writer failed')
        recorder=Recorder(camera,writer,a.output)
        gc.collect(); gc.disable()
        follower.bus.sync_write('Goal_Position',fs,num_retry=2)
        follower.bus.sync_write('Torque_Enable',{k:1 for k in JOINTS},num_retry=2)
        start=last=time.monotonic(); tick=0; previous=fs
        print('RECORDING NOW: move leader; matched-pose mapping, full follower range, slow/faster sweeps, gentle reversals. Stop by Ctrl-C or stop request.',flush=True)
        while True:
            deadline=start+tick/a.hz
            precise_sleep(max(0,deadline-time.monotonic()),spin_threshold=.0003,sleep_margin=.0003)
            now=time.monotonic(); elapsed=now-start
            if elapsed>=a.duration: status='complete'; break
            if now-deadline>.05 or now-last>.05: raise RuntimeError('Control gap >50 ms')
            if recorder.error: raise RuntimeError(f'Camera/recording failed: {recorder.error}')
            lr,lp,lt,lcost=read('leader')
            mapped={k:fs[k]+lp[k]-ls[k] for k in JOINTS}
            zero={k:0.0 for k in JOINTS}
            goal,limited=bounded_goal(mapped,zero,zero,previous,min(now-last,1/a.hz),bounds,RATE)
            wb=time.monotonic(); follower.bus.sync_write('Goal_Position',goal,num_retry=2); we=time.monotonic()
            fr,fp,ft,fcost=read('follower')
            sample={'t':elapsed,'phase':'teleop','leader':lp,'leader_raw':lr,'leader_mapped':mapped,'goal':goal,'actual':fp,'follower_raw':fr,'leader_t':lt-start,'follower_t':ft-start,'command_t':(wb+we)/2-start,'limited':limited,'leader_read_ms':lcost*1000,'follower_read_ms':fcost*1000,'bus_ms':(time.monotonic()-now)*1000,'deadline_late_ms':(now-deadline)*1000}
            # Stagger telemetry: one extra register packet each 5th tick, ~6 Hz/register.
            if tick%5==0:
                reg=('Present_Velocity','Present_Current','Present_Load','Present_Voltage')[(tick//5)%4]
                tb=time.monotonic(); values=follower.bus.sync_read(reg,num_retry=2)
                sample['telemetry']={'register':reg,'values':values,'t':(tb+time.monotonic())/2-start}
            sample['cycle_ms']=(time.monotonic()-now)*1000
            trace.append(sample); recorder.samples.put(sample)
            if sample['cycle_ms']>50: raise RuntimeError('Motor transaction >50 ms')
            if max(abs(fp[k]-goal[k]) for k in JOINTS)>10: raise RuntimeError('Follower tracking error >10 degrees/percent')
            previous=goal; last=now; tick=max(tick+1,int((time.monotonic()-start)*a.hz)+1)
            if tick%625==0: print(f'{elapsed:.1f}s recorded',flush=True)
    except KeyboardInterrupt:
        status='stopped'; print('Recording stopped by request',flush=True)
    except BaseException as exc:
        failure=str(exc); raise
    finally:
        cleanup={}
        for name,bus in buses.items():
            if bus.is_connected:
                try:
                    bus.disable_torque(num_retry=2)
                    cleanup[name]=all(bus.read('Torque_Enable',k,num_retry=2)==0 for k in JOINTS)
                except Exception as exc: cleanup[name]=str(exc)
                finally:
                    try: bus.disconnect(disable_torque=False)
                    except Exception as exc: cleanup[name]=f'Disconnect failed: {exc}'
        try:
            if recorder: recorder.stop()
            else:
                if writer is not None: writer.release()
                if camera is not None: camera.release()
        finally:
            (a.output/'trace.json').write_text(json.dumps(trace))
            (a.output/'result.json').write_text(json.dumps({'status':status,'failure':failure,'samples':len(trace),'torque_off_readback':cleanup},indent=2))
            if was_gc: gc.enable()
        print(f'Capture {status}: {len(trace)} paired samples; torque-off readback {cleanup}',flush=True)
        if any(v is not True for v in cleanup.values()): raise RuntimeError('Torque-off verification incomplete; check power cutoff')

if __name__=='__main__': main()
