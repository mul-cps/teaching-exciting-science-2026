"""Small base-only system probe with C3 position profiles and native settings logged."""
import argparse
import gc
import hashlib
import json
import math
from pathlib import Path
import signal
import time
from workshop.choreography import JOINTS, Recorder
from workshop.teleop_capture import normalized_position


def sweep_profile(t,distance,speed):
    ramp=max(.8,speed*.085)
    if distance<speed*ramp:raise ValueError('Sweep too short for acceleration ramp')
    duration=distance/speed+ramp
    if t<=0:return (0,0,0,0)
    if t>=duration:return (distance,0,0,0)
    def rising(local):
        u=local/ramp
        return (speed*ramp*(2.5*u**4-3*u**5+u**6),
                speed*(10*u**3-15*u**4+6*u**5),
                speed/ramp*(30*u**2-60*u**3+30*u**4),
                speed/ramp**2*(60*u-180*u**2+120*u**3))
    if t<ramp:return rising(t)
    if t>duration-ramp:
        q,v,a,j=rising(duration-t);return distance-q,v,-a,j
    return speed*ramp/2+speed*(t-ramp),speed,0,0


def make_stages():
    stages=[];begin=0.;position=0.
    def hold(name,duration,accel):
        nonlocal begin
        stages.append(dict(name=name,begin=begin,duration=duration,from_angle=position,to_angle=position,speed=0,acceleration_register=accel));begin+=duration
    def move(name,target,speed,accel):
        nonlocal begin,position
        duration=abs(target-position)/speed+max(.8,speed*.085)
        stages.append(dict(name=name,begin=begin,duration=duration,from_angle=position,to_angle=target,speed=speed,acceleration_register=accel));begin+=duration;position=target
    hold('baseline-hold',4,254);move('approach',-12,6,254)
    for speed,accel in [(1,254),(3,254),(8,254),(12,254),(3,50)]:
        label=f'speed-{speed}-accel-{accel}'
        move(label+'-positive',12,speed,accel);hold(label+'-hold-positive',1.5,accel)
        move(label+'-negative',-12,speed,accel);hold(label+'-hold-negative',1.5,accel)
    move('return-center',0,6,254);hold('final-hold',5,254)
    return stages


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');p.add_argument('--port',required=True);p.add_argument('--calibration-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--camera',default='/dev/video1');a=p.parse_args()
    stages=make_stages();duration=stages[-1]['begin']+stages[-1]['duration'];print(f'Base-only ±12 degree probe, {duration:.2f}s, 125 Hz',flush=True)
    if not a.run:return
    a.output.mkdir(parents=True,exist_ok=False)
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.utils.robot_utils import precise_sleep
    import cv2
    arm=SOFollower(SOFollowerRobotConfig(port=a.port,id='hs_rig01_follower',calibration_dir=a.calibration_dir,use_degrees=True));bus=arm.bus
    camera=writer=recorder=None;trace=[];original_acceleration=None;status='failed';failure=None;cleanup={};was_gc=gc.isenabled()
    def interrupt(signum,frame):raise KeyboardInterrupt('Stop requested')
    signal.signal(signal.SIGTERM,interrupt)
    try:
        camera=cv2.VideoCapture(a.camera,cv2.CAP_V4L2);camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'));camera.set(cv2.CAP_PROP_FRAME_WIDTH,640);camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480);camera.set(cv2.CAP_PROP_FPS,25)
        for _ in range(10):ok,frame=camera.read()
        if not ok:raise RuntimeError('Camera preflight failed')
        cv2.imwrite(str(a.output/'before.jpg'),frame);h,w=frame.shape[:2];writer=cv2.VideoWriter(str(a.output/'dance.avi'),cv2.VideoWriter_fourcc(*'MJPG'),25,(w,h))
        if not writer.isOpened():raise RuntimeError('Video writer failed')
        bus.connect()
        if not bus.is_calibrated:raise RuntimeError('Hardware calibration mismatch')
        bus.disable_torque(num_retry=2)
        if any(bus.read('Torque_Enable',k,num_retry=2) for k in JOINTS):raise RuntimeError('Torque-off preflight failed')
        start=bus.sync_read('Present_Position',num_retry=2)
        calibration_path=a.calibration_dir/'hs_rig01_follower.json';cal=json.loads(calibration_path.read_text())
        for k in JOINTS:
            lo=normalized_position(cal[k]['range_min'],cal[k],k=='gripper');hi=normalized_position(cal[k]['range_max'],cal[k],k=='gripper')
            margin=12 if k=='shoulder_pan' else 0
            if start[k]-margin<lo or start[k]+margin>hi:raise RuntimeError(f'{k} probe outside calibration')
        original_acceleration={k:bus.read('Acceleration',k,num_retry=2) for k in JOINTS}
        velocity={k:bus.read('Goal_Velocity',k,num_retry=2) for k in JOINTS}
        if velocity['shoulder_pan']!=0:raise RuntimeError('Unexpected base native velocity setting')
        metadata={'schema':1,'probe':'base only; other targets fixed at measured start; quintic velocity ramps, C3 position','start':start,'hz':125,'duration_s':duration,'stages':stages,'original_acceleration':original_acceleration,'goal_velocity':velocity,'calibration_sha256':hashlib.sha256(calibration_path.read_bytes()).hexdigest(),'max_base_velocity':12,'max_base_acceleration':24,'max_base_jerk':100}
        (a.output/'probe-plan.json').write_text(json.dumps(metadata,indent=2))
        with (a.output/'planned-profiles.jsonl').open('w') as planned:
            for stage in stages:
                direction=1 if stage['to_angle']>=stage['from_angle'] else -1
                for tick in range(math.ceil(stage['duration']*125)+1):
                    local=min(tick/125,stage['duration'])
                    q,v,acc,j=sweep_profile(local,abs(stage['to_angle']-stage['from_angle']),stage['speed']) if stage['speed'] else (0,0,0,0)
                    if abs(v)>12+1e-6 or abs(acc)>24+1e-6 or abs(j)>100+1e-6:raise RuntimeError('Offline probe profile exceeds limits')
                    planned.write(json.dumps({'t':stage['begin']+local,'phase':stage['name'],'offset':stage['from_angle']+direction*q,'velocity':direction*v,'acceleration':direction*acc,'jerk':direction*j})+'\n')
        recorder=Recorder(camera,writer,a.output);gc.collect();gc.disable()
        bus.sync_write('Goal_Position',start,num_retry=2);bus.sync_write('Torque_Enable',dict.fromkeys(JOINTS,1),num_retry=2)
        routine_start=last=time.monotonic();tick=0;stage_index=0;previous_phase=None
        while True:
            deadline=routine_start+tick/125;precise_sleep(max(0,deadline-time.monotonic()),spin_threshold=.0003,sleep_margin=.0003)
            now=time.monotonic();elapsed=now-routine_start
            if now-deadline>.05 or now-last>.05:raise RuntimeError('Control deadline/gap >50 ms')
            if recorder.error:raise RuntimeError(f'Recorder failed: {recorder.error}')
            while stage_index<len(stages)-1 and elapsed>=stages[stage_index]['begin']+stages[stage_index]['duration']:stage_index+=1
            stage=stages[stage_index];local=min(elapsed-stage['begin'],stage['duration'])
            if stage['name']!=previous_phase:
                bus.write('Acceleration','shoulder_pan',stage['acceleration_register'],num_retry=2)
                if bus.read('Acceleration','shoulder_pan',num_retry=2)!=stage['acceleration_register']:raise RuntimeError('Acceleration readback failed')
                previous_phase=stage['name'];print(f'{elapsed:.1f}s: {previous_phase}',flush=True)
            direction=1 if stage['to_angle']>=stage['from_angle'] else -1
            q,v,acc,j=sweep_profile(local,abs(stage['to_angle']-stage['from_angle']),stage['speed']) if stage['speed'] else (0,0,0,0)
            goal=dict(start);goal['shoulder_pan']+=stage['from_angle']+direction*q
            wb=time.monotonic();bus.sync_write('Goal_Position',goal,num_retry=2);we=time.monotonic();rb=time.monotonic();raw=bus.sync_read('Present_Position',normalize=False,num_retry=2);re=time.monotonic();actual={k:normalized_position(raw[k],cal[k],k=='gripper') for k in JOINTS}
            sample={'t':elapsed,'phase':stage['name'],'goal':goal,'actual':actual,'raw':raw,'command_t':(wb+we)/2-routine_start,'follower_t':(rb+re)/2-routine_start,'velocity_command':direction*v,'acceleration_command':direction*acc,'jerk_command':direction*j,'native_acceleration':stage['acceleration_register'],'bus_ms':(re-now)*1000,'deadline_late_ms':(now-deadline)*1000}
            if tick%5==0:
                reg=('Present_Velocity','Present_Current','Present_Load','Present_Voltage')[(tick//5)%4];tb=time.monotonic();values=bus.sync_read(reg,num_retry=2)
                sample['telemetry']={'register':reg,'values':values,'t':(tb+time.monotonic())/2-routine_start}
            sample['cycle_ms']=(time.monotonic()-now)*1000;trace.append(sample);recorder.samples.put(sample)
            if sample['cycle_ms']>50:raise RuntimeError('Motor transaction >50 ms')
            if abs(actual['shoulder_pan']-goal['shoulder_pan'])>3 or max(abs(actual[k]-goal[k]) for k in JOINTS)>5:raise RuntimeError('Probe tracking bound exceeded')
            if elapsed>=duration:status='complete';break
            last=now;tick=max(tick+1,int((time.monotonic()-routine_start)*125)+1)
        print('Probe complete; returned to measured center',flush=True)
    except KeyboardInterrupt:status='stopped'
    except BaseException as exc:failure=str(exc);raise
    finally:
        if bus.is_connected:
            try:
                bus.disable_torque(num_retry=2);cleanup['torque_off']=all(bus.read('Torque_Enable',k,num_retry=2)==0 for k in JOINTS)
                if original_acceleration is not None:
                    bus.sync_write('Acceleration',original_acceleration,num_retry=2);cleanup['acceleration_restored']=all(bus.read('Acceleration',k,num_retry=2)==original_acceleration[k] for k in JOINTS)
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
        print(f'Probe {status}; cleanup {cleanup}',flush=True)
        if any(v is not True for v in cleanup.values()):raise RuntimeError('Cleanup readback incomplete')

if __name__=='__main__':main()
