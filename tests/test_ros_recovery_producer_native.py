"""Actual ROS transports, simulated camera/device responses and synthetic bus only."""
import json
import os
from pathlib import Path
import subprocess
import socket
import sys
from types import SimpleNamespace
import threading
import time
from collections import deque

import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from realsense2_camera_msgs.srv import DeviceInfo
from std_srvs.srv import Trigger
from test_ros_stop_service import TrajectoryBridge,TELEOP_JOINTS
from so101_station_control.trajectory_server import _load_owner_source_hashes,_load_recovery_cache_config
from so101_station_control import recovery_depth_cache


@pytest.mark.parametrize('scenario', [
    'full_recovery_required', 'expired_acquisition_aborts', 'stop_during_wait',
    'publisher_loss_during_wait', 'acquisition_expiry_during_wait',
    'drift_during_wait', 'torque_loss_during_wait',
    'delayed_full_recovery_required'])
def test_native_ros_depth_reaches_actual_station_owner(tmp_path,monkeypatch,scenario):
    prepare_transaction = scenario == 'arm_prepare_required'
    expire_before_request = scenario == 'expired_acquisition_aborts'
    wait_negative = scenario.endswith('_during_wait')
    transport_delay = .080 if scenario == 'delayed_full_recovery_required' else 0.
    assert os.environ.get('ROS_DOMAIN_ID')=='75'
    python=os.environ.get('SO101_RECOVERY_TEST_PYTHON')
    if not python or not Path(python).is_file(): pytest.skip('actual station Python required')
    root=Path(__file__).resolve().parents[1]
    folder=tmp_path/'external-owner';folder.mkdir();state_file=folder/'state.json'
    env=dict(os.environ,PYTHONPATH=os.pathsep.join((str(root),str(root/'tests'))))
    prepare="from pathlib import Path;import json,sys;from test_recovery_runtime import fixture;p=Path(sys.argv[1]);c,*_=fixture(p);(p/'config.json').write_text(json.dumps(c))"
    subprocess.run([python,'-c',prepare,str(folder)],cwd=root,env=env,check=True,capture_output=True,text=True)
    config=json.loads((folder/'config.json').read_text())
    if prepare_transaction:
        config.update(enabled=True,commissioned=True,automatic_start=False)
        (folder/'bridge-config.json').write_text(json.dumps(config))
    dependency_provenance={}
    for name,relative in [('rclpy','rclpy-publisher-gid-7.1.12/install/share/rclpy_gid_overlay/provenance.json'),
                          ('rmw_cyclonedds','rmw-cyclonedds-gid-2.2.4/install/share/rmw_gid_overlay/provenance.json')]:
        marker=root/'ros2/.cache'/relative
        if marker.is_file():dependency_provenance[name]=json.loads(marker.read_text())
    api_process=None;ev3_listener=None;station_job=None
    processes=[];publications=[];callbacks=[];camera_frames=[]
    terminal=dict(success=None,message="no recovery request completed")
    intervention = None
    original_write=recovery_depth_cache.RecoveryDepthCache._write
    def write(collector,record,depth,generation,epoch):
        began=time.monotonic();result=original_write(collector,record,depth,generation,epoch)
        publications.append(dict(record,serialize_started_monotonic=began,published_monotonic=time.monotonic()))
        return result
    monkeypatch.setattr(recovery_depth_cache.RecoveryDepthCache,'_write',write)
    def start(node,**kwargs):
        assert not kwargs
        node._owner_started=time.monotonic();node._owner_source_expected=_load_owner_source_hashes(root)
        node._recovery_cache_config=_load_recovery_cache_config(root,config,state_file)
        wait_fault = {'drift_during_wait': 'drift', 'torque_loss_during_wait': 'torque'}.get(scenario)
        extra = ['--wait-fault', wait_fault] if wait_fault else []
        process=subprocess.Popen([python,str(root/'tests/fake_recovery_owner.py'),'--external-depth', *extra,
            '--fixture-root',str(folder),'--state-file',str(state_file),'--logdir',str(folder/'logs'),
            '--lock',str(folder/'owner.lock')],cwd=root,env=env,stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        processes.append(process);node._process=process
        node._owner_readers=[threading.Thread(target=callback,args=(process,),daemon=True)
                            for callback in (node._read_responses,node._read_stderr)]
        for reader in node._owner_readers:reader.start()
        until=time.monotonic()+10
        while time.monotonic()<until:
            assert process.poll() is None,'fake owner exited'
            sample=node._read_state()
            if sample:
                node._expected_arm_id=sample['arm_id'];node._expected_calibration_sha256=sample['calibration_sha256']
                if sample.get('event')=='sample':
                    if prepare_transaction:
                        from so101_station_control.trajectory_server import _load_measured_home
                        from so101_station_control.gripper_action import validate_jaw_calibration
                        node._prepare_config_path=folder/'bridge-config.json'
                        node._saved_home_pose,node._saved_home_tolerance=_load_measured_home(
                            config,root,node._expected_arm_id,node._expected_calibration_sha256)
                        node._gripper_jaw=validate_jaw_calibration(dict(
                            schema='so101-jaw-gap-calibration-v1',qualified=True,
                            closed_gap_m=.004,open_gap_m=.075,closed_percent=0.,open_percent=100.))
                    return
            time.sleep(.005)
        raise TimeoutError('actual owner initialization')
    monkeypatch.setattr(TrajectoryBridge,'_start_owner',start)
    original_depth=TrajectoryBridge._recovery_depth_received
    def received(node,message,message_info):
        callbacks.append(dict(header_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                              callback_monotonic=time.monotonic()))
        return original_depth(node,message,message_info)
    monkeypatch.setattr(TrajectoryBridge,'_recovery_depth_received',received)
    rclpy.init()
    native_libraries=sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
        if 'librmw_cyclonedds_cpp' in line or '_rclpy_pybind11' in line})
    camera=Node('camera');node=None;executor=None;thread=None;commands=[];operation_commands=[]
    camera.declare_parameter('serial_no','_synthetic-qualified')
    def device(request,response):response.serial_number='synthetic-qualified';return response
    service=camera.create_service(DeviceInfo,'/camera/device_info',device)
    depth_pub=camera.create_publisher(Image,'/camera/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
    info_pub=camera.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
    due_frames = deque(maxlen=2)
    def deliver(image, info, captured):
        info_pub.publish(info);depth_pub.publish(image)
        camera_frames.append(dict(header_ns=image.header.stamp.sec*10**9+image.header.stamp.nanosec,
                                  captured_monotonic=captured,published_monotonic=time.monotonic()))
    def publish():
        stamp=camera.get_clock().now().to_msg()
        image=Image();image.header.stamp=stamp;image.header.frame_id='camera_optical_frame'
        image.height=image.width=100;image.encoding='32FC1';image.step=400
        image.data=bytes.fromhex('0000803f')*10000
        info=CameraInfo();info.header=image.header;info.height=info.width=100
        info.k=[100.,0.,49.5,0.,100.,49.5,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
        captured=time.monotonic()
        if transport_delay:
            due_frames.append((captured+transport_delay,image,info,captured))
        else:
            deliver(image,info,captured)
    def flush():
        while due_frames and due_frames[0][0] <= time.monotonic():
            _,image,info,captured=due_frames.popleft()
            deliver(image,info,captured)
    timer=camera.create_timer(1/15,publish)
    delivery_timer = camera.create_timer(.005,flush) if transport_delay else None
    try:
        for dependency in dependency_provenance.values():
            assert any(path.startswith(dependency['prefix']+'/') for path in native_libraries), native_libraries
        node=TrajectoryBridge(parameter_overrides=[Parameter('station_root',value=str(root)),Parameter('state_file',value=str(state_file))])
        original_send=node._send
        def send(command):commands.append(dict(command));return original_send(command)
        node._send=send
        original_write_stop = node._write_stop
        def write_stop(command, deadline):
            commands.append(dict(command))
            return original_write_stop(command, deadline)
        node._write_stop = write_stop
        executor=MultiThreadedExecutor(num_threads=5);executor.add_node(node);executor.add_node(camera)
        thread=threading.Thread(target=executor.spin);thread.start()
        until=time.monotonic()+4
        while len(publications)<3 and time.monotonic()<until:time.sleep(.005)
        assert len(publications)>=3,dict(identity=node._recovery_camera_identity.snapshot(),producer=node._recovery_depth_cache.snapshot())
        # Test-only observation of the real background provider avoids issuing
        # a request during its one-time prewarm. It grants no motion authority.
        readiness_path=folder/'fake-provider-readiness.json'
        until=time.monotonic()+4;readiness={};prepared=False
        while time.monotonic()<until:
            try:
                readiness=json.loads(readiness_path.read_text())
                now=time.monotonic()
                prepared=(0<=now-readiness['observed_monotonic']<.050 and
                    all(type(readiness.get(key)) in (int,float) and
                        0<=now-readiness[key]<.100 for key in
                        ('scene_stamp_monotonic','acquisition_arm_stamp_monotonic')))
                if prepared and scenario == 'publisher_loss_during_wait':
                    identity = node._recovery_camera_identity.snapshot()
                    request = identity.get('request_started_monotonic')
                    confirmation = identity.get('confirmed_monotonic')
                    prepared = (identity.get('valid') is True and type(request) in (int,float) and type(confirmation) in (int,float) and
                        0 <= now-request < .250 and
                        readiness['scene_stamp_monotonic'] > confirmation)
                if prepared:break
            except (OSError,ValueError,KeyError,TypeError):pass
            time.sleep(.002)
        terminal['provider_before_request']=readiness
        assert prepared,readiness
        if expire_before_request:
            timer.cancel()
            time.sleep(.300)  # Deliberate simulated camera outage, beyond original250ms expiry.
        if prepare_transaction:
            # Actual resident API/job machinery; the EV3 destination is an
            # isolated local listener, never the real station/tunnel port.
            ev3_listener=socket.socket();ev3_listener.bind(('127.0.0.1',0))
            ev3_listener.listen();ev3_listener.setblocking(False)
            api_env=dict(os.environ,PYTHONPATH=os.pathsep.join((str(root),os.environ.get('PYTHONPATH',''))),XDG_STATE_HOME=str(folder/'api-state'))
            api_process=subprocess.Popen([sys.executable,'-c',
                'from pathlib import Path;from tools import ros_station_api as api;'
                'assert Path(api.__file__).resolve()==Path.cwd()/"tools/ros_station_api.py";api.main()',
                '--ros-args','-p','evidence_dir:='+str(folder/'station-jobs'),
                '-p','ev3_port:='+str(ev3_listener.getsockname()[1])],
                cwd=root,env=api_env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        endpoint='/station/arm/prepare' if prepare_transaction else '/follower/recover_camera'
        client=camera.create_client(Trigger,endpoint);assert client.wait_for_service(timeout_sec=8 if prepare_transaction else 2)
        began=time.monotonic();future=client.call_async(Trigger.Request());until=began+(105 if prepare_transaction else 10)
        if wait_negative:
            wait_path = folder/'fake-wait-phase.json'
            wait_observation = None
            wait_until = time.monotonic()+4
            while time.monotonic()<wait_until:
                try:
                    observed = json.loads(wait_path.read_text())
                    if observed.get('first_entry'):
                        wait_observation = observed
                        break
                except (OSError, ValueError):
                    pass
                time.sleep(.001)
            assert wait_observation is not None, 'actual owner never entered stationary recapture'
            intervention = dict(scenario=scenario, observed_monotonic=time.monotonic(),
                                wait=wait_observation)
            if scenario == 'stop_during_wait':
                assert wait_observation['state'] == 'entered', wait_observation
                stop_client = camera.create_client(Trigger, '/follower/stop')
                assert stop_client.wait_for_service(timeout_sec=1)
                stop_future = stop_client.call_async(Trigger.Request())
                stop_until = time.monotonic()+3
                while not stop_future.done() and time.monotonic()<stop_until:
                    time.sleep(.002)
                assert stop_future.done()
                stopped = stop_future.result()
                intervention['stop_success'] = stopped.success
                intervention['stop_message'] = stopped.message
                assert stopped.success, stopped.message
            elif scenario == 'publisher_loss_during_wait':
                assert wait_observation['state'] == 'entered', wait_observation
                timer.cancel()
                assert camera.destroy_publisher(depth_pub)
            elif scenario == 'acquisition_expiry_during_wait':
                assert wait_observation['state'] == 'entered', wait_observation
                timer.cancel()
        while not future.done() and time.monotonic()<until:time.sleep(.005)
        assert future.done();result=future.result()
        if prepare_transaction:
            assert result.success,result.message
            receipt=json.loads(result.message)
            status_client=camera.create_client(Trigger,'/station/status')
            assert status_client.wait_for_service(timeout_sec=2)
            while time.monotonic()<until:
                status_future=status_client.call_async(Trigger.Request())
                response_until=min(until,time.monotonic()+2)
                while not status_future.done() and time.monotonic()<response_until:time.sleep(.005)
                assert status_future.done()
                status=json.loads(status_future.result().message)
                assert status['ev3']['connected'] is False
                station_job=status['workflow']
                assert station_job['job_id']==receipt['job_id']
                if station_job['state'] in {'idle','fault'}:break
                time.sleep(.025)
            assert station_job['state']=='idle',station_job
            assert any(event.get('operation')=='arm_prepare' and event.get('event')=='complete'
                       for event in station_job['events'])
            result=SimpleNamespace(success=True,message=json.dumps(station_job['result']))
            with pytest.raises(BlockingIOError):ev3_listener.accept()
        elapsed=time.monotonic()-began
        # A stationary-only cohort may safely hold during movement. Preserve deadlines.
        assert elapsed<(100 if prepare_transaction else 8)
        if scenario == 'stop_during_wait':
            assert len(commands) == 2 and commands[0] == {'command':'recover_camera'}
            assert commands[1]['command'] == 'stop' and set(commands[1]) == {'command','request_id'}
        elif prepare_transaction:
            assert commands==[{'command':'recover_camera'},{'command':'home'}]
        else:
            assert commands==[{'command':'recover_camera'}]
        operation_commands=list(commands)
        assert (node._home_completed_owner_started is not None) is prepare_transaction
        if prepare_transaction:
            assert node._sorting_ready(Trigger.Request(),Trigger.Response()).success
            value=json.loads(result.message)
            assert value["phases"]==["recover_camera","home"] and value["state"]=="ready"
            assert value["observation"]["raw"]==dict.fromkeys(TELEOP_JOINTS,2000)
        terminal=dict(success=result.success,message=result.message,elapsed_s=elapsed,provider_before_request=readiness,
                      intervention=intervention,station_job=station_job,
                      producer=node._recovery_depth_cache.snapshot(),identity=node._recovery_camera_identity.snapshot())
    finally:
        terminal['intervention'] = intervention
        if node is not None:
            terminal['cached_owner_before_cleanup']=node._cached_owner_state
            terminal['owner_faulted_before_cleanup']=node._owner_faulted
            terminal['stop_latched_before_cleanup']=node._stop_latched
            terminal['home_completed_before_cleanup']=node._home_completed_owner_started is not None
            terminal['final_producer'] = (node._recovery_depth_cache.snapshot()
                if node._recovery_depth_cache is not None else None)
            terminal['final_identity'] = (node._recovery_camera_identity.snapshot()
                if node._recovery_camera_identity is not None else None)
        if api_process is not None:
            api_process.terminate()
            try:api_output=api_process.communicate(timeout=15)[0]
            except subprocess.TimeoutExpired:
                api_process.kill();api_output=api_process.communicate()[0]
                terminal['api_shutdown_timeout']=True
            terminal['api_process_output']=api_output
        if ev3_listener is not None:ev3_listener.close()
        if executor:executor.shutdown()
        if thread:thread.join(2)
        if node:node.destroy_node()
        camera.destroy_node()
        for process in processes:
            if process.poll() is None:process.terminate();process.wait(timeout=3)
        rclpy.shutdown()
        evidence=os.environ.get('SO101_PRODUCER_TEST_EVIDENCE')
        audit_path=folder/'fake-owner-write-audit.json'
        if evidence:
            if scenario != 'full_recovery_required':
                suffix = 'expired' if expire_before_request else scenario
                path=Path(evidence);evidence=str(path.with_name(path.stem+'-'+suffix+path.suffix))
            audit_value=json.loads(audit_path.read_text()) if audit_path.exists() else None
            Path(evidence).write_text(json.dumps(dict(schema=1,hardware_access=False,
                physical_qualification=False,scenario=scenario,
                camera_identity_source="simulated device responses over actual ROS services",
                geometry_scope="100x100 synthetic qualified fixture; 640x480 worker performance is separate",
                camera_hz=15,camera_shape=[100,100],child_python=python,
                camera_transport_delay_s=transport_delay,camera_pending_capacity=2,
                camera_frames=camera_frames,
                owner_source_sha256=_load_owner_source_hashes(root),
                ros_dependency_provenance=dependency_provenance,ros_backend=os.environ.get("RMW_IMPLEMENTATION"),
                loaded_native_libraries=native_libraries,
                owner_commands=operation_commands,cleanup_commands=commands[len(operation_commands):],terminal=terminal,callbacks=callbacks,publications=publications,
                actual_bus=audit_value),indent=2)+'\n')
    audit=json.loads((folder/'fake-owner-write-audit.json').read_text())
    entered = None
    for index, event in enumerate(audit['wait_observations']):
        assert event['event_index'] == index
        if event['state'] == 'entered':
            assert entered is None
            entered = event
        else:
            assert entered is not None
            assert event['goal_writes'] == entered['goal_writes'], 'Goal write inside stationary wait'
            entered = None
    assert entered is None, 'stationary wait never reported its terminal state'
    if expire_before_request:
        assert not terminal['success'] and audit['goal_history']==[],terminal
        assert audit['final_raw']['elbow_flex']==2993
    elif wait_negative:
        assert terminal['success'] is False, terminal
        waits = audit['wait_observations']
        first = next(event for event in waits if event['state']=='entered')
        assert first['held_raw']['elbow_flex'] == 2991
        assert first['goal_writes'] == 2
        if scenario == 'publisher_loss_during_wait':
            # DDS loss discovery is asynchronous. An already acquired fresh
            # post-hold cohort can admit one step before absence is observed.
            assert 2 <= len(audit['goal_history']) <= 3
            failed = next(event for event in waits if event['state']=='failed')
            assert len(audit['goal_history']) == failed['goal_writes']
            assert audit['final_raw']['elbow_flex'] == failed['held_raw']['elbow_flex']
            revoked = next(event for event in audit['health_observations']
                           if event['during_wait'] and not event['available'])
            assert all(event['finished_monotonic'] <= revoked['started_monotonic']
                       for event in audit['goal_write_observations'])
            for write in audit['goal_write_observations'][first['goal_writes']:]:
                proof = next(event for event in reversed(audit['proof_observations'])
                             if event['check_finished_monotonic'] <= write['started_monotonic']
                             and 'error' not in event)
                assert proof['scene_stamp_monotonic'] > first['hold_stamp_monotonic']
                assert write['finished_monotonic'] <= proof['scene_valid_until_monotonic']
                assert any(record['depth_stamp_monotonic'] == proof['scene_stamp_monotonic']
                           and record['acquisition_arm_raw'] == first['held_raw']
                           for record in publications)
        else:
            assert len(audit['goal_history']) == 2
            assert all(event['goal_writes']==first['goal_writes'] for event in waits)
            assert audit['final_raw']['elbow_flex'] == first['held_raw']['elbow_flex']
        assert terminal['message']
    else:
        assert terminal['success'],terminal
        assert audit['goal_history'],'actual planning/proof did not dispatch a goal'
        if transport_delay:
            assert camera_frames
            assert all(frame['published_monotonic']-frame['captured_monotonic'] >= transport_delay
                       for frame in camera_frames)
    assert audit['eeprom_writes']==[]
    assert terminal['home_completed_before_cleanup'] is prepare_transaction
    if terminal['success']:
        assert audit['goal_history'][-1]['elbow_flex']==(2000 if prepare_transaction else 2973)
        assert not terminal['owner_faulted_before_cleanup']
    elif scenario == 'stop_during_wait':
        assert terminal['message'] and terminal['intervention']['stop_success']
        assert not terminal['stop_latched_before_cleanup']
        assert not terminal['owner_faulted_before_cleanup']
    else:
        assert terminal['message'] and terminal['owner_faulted_before_cleanup']
    previous=dict.fromkeys(TELEOP_JOINTS,2000);previous['elbow_flex']=2993
    for goal in audit['goal_history']:
        assert 0<=previous['elbow_flex']-goal['elbow_flex']<=(100 if prepare_transaction and previous['elbow_flex']<=2973 else 2)
        assert all(goal[joint]==2000 for joint in TELEOP_JOINTS if joint!='elbow_flex')
        previous=goal
    assert all(value==0 for value in audit['final_torque'].values())
