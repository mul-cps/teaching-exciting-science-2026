"""Replay local captured commands at original timestamps, without spline retiming."""
import argparse
import bisect
import gc
import hashlib
import json
import math
from pathlib import Path
import signal
import time
from workshop.choreography import JOINTS, Recorder
from workshop.teleop_capture import normalized_position


def prepare_samples(rows):
    origin=rows[0]['command_t']
    return [{'t':r['command_t']-origin,'goal':dict(r['goal'])} for r in rows]


def validate_samples(samples,bounds,rates,nominal_dt=0.0):
    if len(samples)<2 or samples[0]['t']!=0: raise ValueError('Invalid replay start')
    previous=None
    for s in samples:
        if not math.isfinite(s['t']) or set(s['goal'])!=set(JOINTS): raise ValueError('Invalid replay sample')
        if any(not math.isfinite(s['goal'][k]) or not bounds[k][0]-1e-6<=s['goal'][k]<=bounds[k][1]+1e-6 for k in JOINTS):
            raise ValueError('Replay outside calibrated range')
        if previous:
            dt=s['t']-previous['t']
            if dt<=0 or dt>.05: raise ValueError('Replay timing invalid or gap >50 ms')
            if any(abs(s['goal'][k]-previous['goal'][k])>rates[k]*max(dt,nominal_dt)*1.01+1e-6 for k in JOINTS):
                raise ValueError('Replay exceeds recorded rate limits')
        previous=s


def reverse_return(samples):
    # Omit the original initial hold once returning has reached the starting goal.
    first_motion=next((i for i,s in enumerate(samples) if s['goal']!=samples[0]['goal']),len(samples)-1)
    first=max(0,first_motion-1)
    end=samples[-1]['t']
    return [{'t':end-s['t'],'goal':dict(s['goal'])} for s in reversed(samples[first:])]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path)
    p.add_argument('--run',action='store_true')
    p.add_argument('--port',required=True)
    p.add_argument('--calibration-dir',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--camera',default='/dev/video1')
    from workshop.safe_home import DEFAULT_HOME,load_home,make_home_move,home_at,validate_home_start
    p.add_argument('--home',type=Path,default=DEFAULT_HOME,help='Measured home; required before replay')
    a=p.parse_args()
    metadata=json.loads((a.source/'metadata.json').read_text())
    result=json.loads((a.source/'result.json').read_text())
    if result['status']!='complete': p.error('Source capture was not completed')
    source_bytes=(a.source/'trace.json').read_bytes(); samples=prepare_samples(json.loads(source_bytes))
    calibration_path=a.calibration_dir/'hs_rig01_follower.json'
    digest=hashlib.sha256(calibration_path.read_bytes()).hexdigest()
    if digest!=metadata['calibration_sha256']['follower']: p.error('Calibration hash mismatch')
    calibration=json.loads(calibration_path.read_text())
    bounds={k:(normalized_position(calibration[k]['range_min'],calibration[k],k=='gripper'),normalized_position(calibration[k]['range_max'],calibration[k],k=='gripper')) for k in JOINTS}
    validate_samples(samples,bounds,metadata['rate_limits'],1/metadata['hz'])
    returning=reverse_return(samples)
    validate_samples(returning,bounds,metadata['rate_limits'],1/metadata['hz'])
    print(f'Validated {len(samples)} commands, {samples[-1]["t"]:.3f}s forward; reverse return {returning[-1]["t"]:.3f}s',flush=True)
    if not a.run: return
    a.output.mkdir(parents=True,exist_ok=False)
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.utils.robot_utils import precise_sleep
    import cv2
    arm=SOFollower(SOFollowerRobotConfig(port=a.port,id='hs_rig01_follower',calibration_dir=a.calibration_dir,use_degrees=True)); bus=arm.bus
    camera=writer=recorder=None; trace=[]; original_acceleration=None; original_velocity=None
    was_gc=gc.isenabled(); status='failed'; failure=None; cleanup={}
    def interrupt(signum,frame):raise KeyboardInterrupt('Stop requested')
    signal.signal(signal.SIGTERM,interrupt)
    try:
        camera=cv2.VideoCapture(a.camera,cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG')); camera.set(cv2.CAP_PROP_FRAME_WIDTH,640); camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480); camera.set(cv2.CAP_PROP_FPS,25)
        for _ in range(10):ok,frame=camera.read()
        if not ok:raise RuntimeError('Camera preflight failed')
        cv2.imwrite(str(a.output/'before.jpg'),frame)
        h,w=frame.shape[:2];writer=cv2.VideoWriter(str(a.output/'dance.avi'),cv2.VideoWriter_fourcc(*'MJPG'),25,(w,h))
        if not writer.isOpened():raise RuntimeError('Video writer failed')
        bus.connect()
        if not bus.is_calibrated:raise RuntimeError('Hardware calibration mismatch')
        bus.disable_torque(num_retry=2)
        if any(bus.read('Torque_Enable',k,num_retry=2) for k in JOINTS):raise RuntimeError('Torque off preflight failed')
        current=bus.sync_read('Present_Position',num_retry=2)
        home=load_home(a.home,'hs_rig01_follower',digest)
        validate_home_start(samples[0]['goal'],home)
        home_move=make_home_move(current,home,bounds,metadata['hz'])
        home_path=[{'t':min(i/metadata['hz'],home_move['duration']),
                    'goal':home_at(home_move,min(i/metadata['hz'],home_move['duration']))[0]}
                   for i in range(math.ceil(home_move['duration']*metadata['hz'])+1)]
        phases=[('home',home_path),('forward',samples)]
        original_acceleration={k:bus.read('Acceleration',k,num_retry=2) for k in JOINTS}
        original_velocity={k:bus.read('Goal_Velocity',k,num_retry=2) for k in JOINTS}
        acceleration=metadata['follower']['acceleration'];velocity=metadata['follower']['goal_velocity']
        replay_meta={'schema':1,'mode':'original command timestamps, zero-order hold; no smoothing or retiming','source_trace_sha256':hashlib.sha256(source_bytes).hexdigest(),'calibration_sha256':digest,'acceleration':acceleration,'goal_velocity':velocity,'original_acceleration':original_acceleration,'original_velocity':original_velocity,'start':current,'home_move':home_move,'home_sha256':hashlib.sha256(a.home.read_bytes()).hexdigest(),'phases':[{'name':name,'samples':path} for name,path in phases]}
        (a.output/'replay-plan.json').write_text(json.dumps(replay_meta))
        bus.sync_write('Acceleration',acceleration,num_retry=2);bus.sync_write('Goal_Velocity',velocity,num_retry=2)
        for reg,target in [('Acceleration',acceleration),('Goal_Velocity',velocity)]:
            if any(bus.read(reg,k,num_retry=2)!=target[k] for k in JOINTS):raise RuntimeError(f'{reg} readback failed')
        recorder=Recorder(camera,writer,a.output)
        gc.collect();gc.disable()
        bus.sync_write('Goal_Position',current,num_retry=2);bus.sync_write('Torque_Enable',dict.fromkeys(JOINTS,1),num_retry=2)
        routine_start=time.monotonic()
        for phase,path in phases:
            actual=bus.sync_read('Present_Position',num_retry=2)
            if max(abs(actual[k]-path[0]['goal'][k]) for k in JOINTS)>(1 if phase=='forward' else 3):raise RuntimeError('Phase starting pose mismatch')
            print(f'{phase.upper()} starting: {path[-1]["t"]:.1f}s, acceleration {acceleration}',flush=True)
            times=[s['t'] for s in path]; index=0; start=last=time.monotonic(); last_report=-5
            while index<len(path):
                deadline=start+path[index]['t'];precise_sleep(max(0,deadline-time.monotonic()),spin_threshold=.0003,sleep_margin=.0003)
                now=time.monotonic()
                if now-deadline>.05 or now-last>.05:raise RuntimeError('Control deadline/gap >50 ms')
                if recorder.error:raise RuntimeError(f'Recorder failed: {recorder.error}')
                index=max(index,bisect.bisect_right(times,now-start)-1)
                goal=path[index]['goal']; wb=time.monotonic();bus.sync_write('Goal_Position',goal,num_retry=2);we=time.monotonic()
                rb=time.monotonic();actual=bus.sync_read('Present_Position',num_retry=2);re=time.monotonic()
                sample={'t':now-routine_start,'phase':phase,'source_t':path[index]['t'],'goal':goal,'actual':actual,'command_t':(wb+we)/2-routine_start,'follower_t':(rb+re)/2-routine_start,'bus_ms':(re-now)*1000,'deadline_late_ms':(now-deadline)*1000}
                trace.append(sample);recorder.samples.put(sample)
                if sample['bus_ms']>50:raise RuntimeError('Motor bus transaction >50 ms')
                if max(abs(actual[k]-goal[k]) for k in JOINTS)>10:raise RuntimeError('Tracking error >10 degrees/percent')
                if now-start-last_report>=5:print(f'{phase} {now-start:.1f}s',flush=True);last_report=now-start
                last=now;index+=1
        status='complete';print('Original-timing replay complete',flush=True)
    except KeyboardInterrupt:status='stopped'
    except BaseException as exc:failure=str(exc);raise
    finally:
        if bus.is_connected:
            try:
                bus.disable_torque(num_retry=2)
                cleanup['torque_off']=all(bus.read('Torque_Enable',k,num_retry=2)==0 for k in JOINTS)
                for reg,target in [('Acceleration',original_acceleration),('Goal_Velocity',original_velocity)]:
                    if target is not None:
                        bus.sync_write(reg,target,num_retry=2)
                        cleanup[reg+'_restored']=all(bus.read(reg,k,num_retry=2)==target[k] for k in JOINTS)
            except Exception as exc:cleanup['error']=str(exc)
            finally:bus.disconnect(disable_torque=False)
        try:
            if recorder:recorder.stop()
            else:
                if writer is not None:writer.release()
                if camera is not None:camera.release()
        finally:
            (a.output/'trace.json').write_text(json.dumps(trace));(a.output/'result.json').write_text(json.dumps({'status':status,'failure':failure,'samples':len(trace),'cleanup':cleanup},indent=2))
            if was_gc:gc.enable()
        print(f'Replay {status}; cleanup {cleanup}',flush=True)
        if any(v is not True for v in cleanup.values()):raise RuntimeError('Cleanup readback incomplete')

if __name__=='__main__':main()
