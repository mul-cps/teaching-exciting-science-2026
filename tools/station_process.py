#!/usr/bin/env python3
"""Own one integrated station launch. Startup never enables motion or homes."""
import argparse
import ctypes
import fcntl
import hashlib
import json
import math
import re
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def process_identity(pid, *, include_zombie=False):
    try:
        # comm may contain spaces/parentheses; fields after its final ')' start at 3.
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z' and not include_zombie:
            return None
        return {'pid': pid, 'starttime': fields[19], 'pgrp': int(fields[2]),
                'session': int(fields[3])}
    except (OSError, ValueError, IndexError):
        return None


def same_process(record):
    current = process_identity(record['pid'])
    return current == record and record['pid'] == record['pgrp'] == record['session']


def duplicate_owners():
    matches = []
    ports = {str(port.resolve()) for port in (
        Path('/dev/serial/by-id')/f'usb-1a86_USB_Single_Serial_{serial}-if00'
        for serial in ('5AB9068906', '5AB9068765')) if port.exists()}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            words = (entry/'cmdline').read_bytes().decode(errors='replace').split('\0')
        except OSError:
            continue
        names = [Path(word).name for word in words if word]
        if any(name in {'trajectory_server', 'leader_teleop', 'station_api',
                        'ros_station_api.py', 'realsense2_camera_node', 'move_group', 'station_live.launch.py'} for name in names) or \
                'workshop.station.arm_session' in words:
            matches.append(f"{entry.name}: {' '.join(words).strip()[:240]}")
            continue
        if ports:
            try:
                if any(os.readlink(fd) in ports for fd in (entry/'fd').iterdir()):
                    matches.append(f"{entry.name}: identified arm serial port already open")
            except OSError:
                pass
    return matches


STOP_STEPS = ((signal.SIGINT, 16), (signal.SIGTERM, 5), (signal.SIGKILL, 2))
RPC_STEPS = ((signal.SIGTERM, .5), (signal.SIGKILL, 1.))


def session_members(session):
    """Read live identities, including descendant process groups in this session."""
    members = []
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            identity = process_identity(int(entry.name))
            if identity is not None and identity['session'] == session:
                members.append(identity)
    return members


def refresh_members(record):
    """Extend ownership only while an exact previously verified member survives."""
    known = record.get('members', [record['process']])
    observed = {member['pid']: process_identity(member['pid'], include_zombie=True) for member in known}
    escaped = [current for member in known if (current := observed[member['pid']]) is not None
               and current['starttime'] == member['starttime']
               and current['session'] != record['process']['session']]
    if escaped:
        record['unverified_members'] = escaped
        raise RuntimeError('Managed descendants changed session; retained ownership record')
    anchored = any(observed[member['pid']] == member for member in known)
    current = session_members(record['process']['session'])
    if current and not anchored:
        record['unverified_members'] = current
        raise RuntimeError('Managed session contains unverifiable survivors; retained ownership record')
    record['members'] = current
    return current


def open_pidfd(pid):
    if hasattr(os, 'pidfd_open'):
        return os.pidfd_open(pid)
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.pidfd_open
    function.argtypes, function.restype = [ctypes.c_int, ctypes.c_uint], ctypes.c_int
    descriptor = function(pid, 0)
    if descriptor < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return descriptor


def send_pidfd(descriptor, sig):
    if hasattr(signal, 'pidfd_send_signal'):
        return signal.pidfd_send_signal(descriptor, sig)
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.pidfd_send_signal
    function.argtypes, function.restype = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint], ctypes.c_int
    if function(descriptor, sig, None, 0) < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def signal_member(identity, sig):
    """A pidfd pins the signaled task across the final identity check/PID reuse."""
    try:
        descriptor = open_pidfd(identity['pid'])
    except ProcessLookupError:
        return
    try:
        if process_identity(identity['pid']) != identity:
            return
        try:
            send_pidfd(descriptor, sig)
        except ProcessLookupError:
            pass
    finally:
        os.close(descriptor)


def cleanup_session(record, steps, *, path=None, child=None):
    """Bounded escalation follows verified descendants after the leader exits."""
    for sig, seconds in steps:
        members = refresh_members(record)
        if path is not None:
            save(path, record)
        if not members:
            return
        for member in members:
            signal_member(member, sig)
        until = time.monotonic()+seconds
        while True:
            # Snapshot while an exited-but-unreaped leader still pins its
            # identity, then reap only after newly forked descendants are saved.
            members = refresh_members(record)
            if path is not None:
                save(path, record)
            if child is not None:
                child.poll()
            else:
                try:
                    os.waitpid(record['process']['pid'], os.WNOHANG)
                except ChildProcessError:
                    pass
            if not members:
                return
            remaining = until-time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(.05, remaining))
    if refresh_members(record):
        raise RuntimeError('Managed session did not exit; retained ownership record')


def retain_rpc_record(record):
    descriptor, name = tempfile.mkstemp(prefix='station-rpc-survivors-', suffix='.json')
    os.close(descriptor)
    save(Path(name), record)
    return name


def run(command, *, timeout=15, **kwargs):
    # subprocess.run(timeout) kills only the wrapper, leaving its ROS CLI child.
    capture = kwargs.pop('capture_output', False)
    if capture:
        if 'stdout' in kwargs or 'stderr' in kwargs:
            raise ValueError('capture_output conflicts with stdout/stderr')
        kwargs.update(stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    input_value = kwargs.pop('input', None)
    if input_value is not None:
        if 'stdin' in kwargs:
            raise ValueError('input conflicts with stdin')
        kwargs['stdin'] = subprocess.PIPE
    child = subprocess.Popen(command, start_new_session=True, **kwargs)
    identity = process_identity(child.pid, include_zombie=True)
    if identity is None:
        record = {'process': {'pid': child.pid, 'session': child.pid, 'pgrp': child.pid,
                               'starttime': 'unavailable'}, 'members': [], 'command': command,
                  'unverified_members': session_members(child.pid)}
        name = retain_rpc_record(record)
        raise RuntimeError('RPC identity unavailable; ownership retained in '+name)
    record = {'process': identity, 'members': [identity], 'command': command}
    deadline = time.monotonic()+timeout
    try:
        while True:
            refresh_members(record)
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise subprocess.TimeoutExpired(command, timeout)
            try:
                output, errors = child.communicate(input=input_value, timeout=min(.05, remaining))
                break
            except subprocess.TimeoutExpired:
                input_value = None
        # A wrapper's successful exit does not release its surviving children.
        if session_members(identity['session']):
            cleanup_session(record, RPC_STEPS, child=child)
        result = subprocess.CompletedProcess(command, child.returncode, output, errors)
        result.check_returncode()
        return result
    except BaseException as original:
        try:
            cleanup_session(record, RPC_STEPS, child=child)
            child.wait(timeout=.1)
        except BaseException as cleanup_error:
            name = retain_rpc_record(record)
            raise RuntimeError('RPC cleanup incomplete; ownership retained in '+name+': '+str(cleanup_error)) from original
        raise


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2)+'\n')
    temporary.replace(path)


def wrapper(args):
    if args.wrapper:
        return [str(Path(args.wrapper).expanduser()), args.wrapper_command or 'run']
    if args.profile == 'fwmini4':
        return [str(Path.home()/'.local/bin/station-fwmini4'), 'ros-vulkan']
    return [str(ROOT/'ros2/conda/so101-conda.sh'), 'run']


COMPONENT_OVERLAY_FILES = {
    'ros2/src/so101_station_bringup/launch/station_live.launch.py':('so101_station_bringup','launch/station_live.launch.py'),
    'ros2/src/so101_station_control/launch/moveit_resident.launch.py':('so101_station_control','launch/moveit_resident.launch.py'),
    'ros2/src/so101_station_perception/launch/perception.launch.py':('so101_station_perception','launch/perception.launch.py'),
    'ros2/src/so101_station_operator/launch/operator.launch.py':('so101_station_operator','launch/operator.launch.py'),
    'ros2/src/so101_station_workflow/scripts/station_api':('so101_station_workflow','station_api'),
    'ros2/src/so101_station_operator/scripts/operator_api':('so101_station_operator','operator_api'),
    'ros2/src/so101_station_control/so101_station_control/operator_restart.py':('so101_station_control','operator_restart'),
    'ros2/src/so101_station_control/so101_station_control/trajectory_server.py':('so101_station_control','trajectory_server'),
}


def verify_component_overlay(ros):
    # Read installed origins only. Never import a node module or construct nodes.
    program = """import importlib.util,json
from pathlib import Path
from ament_index_python.packages import get_package_prefix
files=json.loads(__import__('sys').argv[1]);result={}
for source,(package,relative) in files.items():
    if relative.startswith('launch/'):
        path=Path(get_package_prefix(package))/'share'/package/relative
    elif relative in ('operator_api','station_api'):
        path=Path(get_package_prefix(package))/'lib'/package/relative
    else:
        path=Path(importlib.util.find_spec(package+'.'+relative).origin)
    result[source]=str(path)
print(json.dumps(result))
"""
    reply=run(ros+['python','-c',program,json.dumps(COMPONENT_OVERLAY_FILES)],timeout=15,capture_output=True,text=True)
    if len(reply.stdout)>16384:raise RuntimeError('Installed overlay origin report exceeds bound')
    origins=json.loads(reply.stdout)
    if not isinstance(origins,dict) or set(origins)!=set(COMPONENT_OVERLAY_FILES):
        raise RuntimeError('Installed component overlay origin set is incomplete')
    for source,origin in origins.items():
        path=Path(origin)
        if not path.is_absolute() or not path.is_file() or path.stat().st_size>2*1024*1024 or hashlib.sha256(path.read_bytes()).digest()!=hashlib.sha256((ROOT/source).read_bytes()).digest():
            raise RuntimeError('Installed component overlay is stale: '+source+'; rebuild before startup')


def prerequisites(args, ros):
    if not Path(ros[0]).is_file() or not os.access(ros[0], os.X_OK):
        raise RuntimeError('ROS wrapper unavailable: '+ros[0])
    if not args.python.is_file():
        raise RuntimeError('Station Python missing: '+str(args.python))
    config = json.loads(args.config.read_text())
    arm = config['arm']
    if arm['usb']['serial'] != '5AB9068906':
        raise RuntimeError('Expected identified follower serial 5AB9068906')
    calibration = Path(arm['calibration']['path'])
    if not calibration.is_absolute():
        calibration = args.config.parent/calibration
    if hashlib.sha256(calibration.read_bytes()).hexdigest() != arm['calibration']['sha256']:
        raise RuntimeError('Follower calibration hash mismatch')
    for serial in ('5AB9068906', '5AB9068765'):
        port = Path('/dev/serial/by-id')/f'usb-1a86_USB_Single_Serial_{serial}-if00'
        if not port.exists():
            raise RuntimeError('Required identified arm is absent: '+str(port))
    if not (args.calibration_dir/'hs_rig01_leader.json').is_file():
        raise RuntimeError('Saved leader calibration missing')
    repair = ROOT/'ros2/.cache/realsense-cpu-texture-bounds/install'
    if not repair.is_dir():
        raise RuntimeError('Camera repair missing; run ros2/conda/so101-conda.sh realsense-cpu-build')
    result = run(ros+['ros2', 'launch', str(ROOT/'ros2/src/so101_station_bringup/launch/station_live.launch.py'),
                     '--show-args'], timeout=45, capture_output=True, text=True)
    if any(f"'{name}'" not in result.stdout for name in ('joint_limit_contract', 'use_station_api', 'use_operator_panel_api')):
        raise RuntimeError('Launch overlay is stale; build this checkout with ros2/conda/so101-conda.sh build')
    installed = run(ros+['ros2', 'launch', 'so101_station_bringup', 'station_live.launch.py', '--show-args'],
                    timeout=45, capture_output=True, text=True)
    if any(f"'{name}'" not in installed.stdout for name in ('joint_limit_contract', 'use_station_api', 'use_operator_panel_api')):
        raise RuntimeError('Installed launch overlay is stale; run ros2/conda/so101-conda.sh build before hardware startup')
    run(ros+['ros2', 'pkg', 'prefix', 'so101_station_workflow'], capture_output=True, text=True)
    run(ros+['ros2', 'pkg', 'prefix', 'so101_station_operator'], capture_output=True, text=True)
    if getattr(args,'deployment','integrated')=='user-units':
        verify_component_overlay(ros)
    if not args.existing_tunnel:
        with socket.socket() as listener:
            try:
                listener.bind(('127.0.0.1', 18766))
            except OSError as error:
                raise RuntimeError('Port 18766 already occupied; inspect and use --existing-tunnel for the verified forward') from error
    # Read-only remote prerequisite: routine startup never installs/restarts the daemon.
    run(['ssh', '-o', 'ConnectTimeout=5', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
         '-o', 'IdentitiesOnly=yes', '-i', str(Path.home()/'.local/share/exciting-science/station/ssh/id_ev3'),
         'robot@192.168.0.1', 'systemctl --user is-active cps-ev3-api.service'],
        timeout=10, capture_output=True, text=True)


def stop_record(path, record):
    if record.get('deployment')=='user-units':
        return stop_units(path,record)
    if not same_process(record['process']) and not any(
            member.get('session') == record['process'].get('session') and
            process_identity(member['pid']) == member for member in record.get('members', [])):
        # Never send ROS STOP or signals based on a stale/reused process ID.
        raise RuntimeError('Managed launch identity no longer matches; inspect surviving station processes manually')
    try:
        refresh_members(record)
    finally:
        save(path, record)
    rpc_cleanup_errors = list(record.get('rpc_cleanup_errors', []))
    for service in ('/station/stop', '/follower/stop'):
        try:
            result = run(record['ros']+['ros2', 'service', 'call', service,
                                        'std_srvs/srv/Trigger', '{}'], timeout=8,
                         capture_output=True, text=True, env={**os.environ, **record['environment']})
            print(result.stdout, end='')
            if 'success=True' not in result.stdout and 'success: true' not in result.stdout.lower():
                print(service+': no successful terminal acknowledgement; inspect physical state', file=sys.stderr)
        except RuntimeError as error:
            # An uncertain CLI cleanup cannot prevent the independent follower
            # STOP or verified managed-session escalation. Preserve its record.
            rpc_cleanup_errors.append({'service': service, 'error': str(error)})
            print(service+': STOP RPC cleanup incomplete: '+str(error), file=sys.stderr)
        except (subprocess.SubprocessError, OSError) as error:
            print(service+': STOP acknowledgement unavailable: '+str(error), file=sys.stderr)
    record['rpc_cleanup_errors'] = rpc_cleanup_errors
    # Persist descendant identities before the leader can exit on SIGINT.
    # Workflow shutdown also performs coordinated STOP/disarm; retain its grace period.
    try:
        refresh_members(record)
        save(path, record)
        cleanup_session(record, STOP_STEPS, path=path)
    except BaseException:
        save(path, record)
        raise
    if rpc_cleanup_errors:
        record['managed_cleanup_complete'] = True
        save(path, record)
        raise RuntimeError('STOP RPC cleanup incomplete; managed session stopped, ownership record retained: '+
                           '; '.join(item['service']+': '+item['error'] for item in rpc_cleanup_errors))
    path.unlink(missing_ok=True)


UNIT_PROPERTIES = ('Id','LoadState','ActiveState','SubState','Job','InvocationID','StateChangeTimestampMonotonic',
                   'MainPID','ControlGroup','FragmentPath','DropInPaths','UnitFileState','Restart')


def unit_tools():
    try:
        from tools import install_station_user_units
    except ModuleNotFoundError:
        import install_station_user_units
    return install_station_user_units


def manager_identity():
    result = run(['busctl','--user','call','org.freedesktop.DBus','/org/freedesktop/DBus',
                  'org.freedesktop.DBus','GetNameOwner','s','org.freedesktop.systemd1'],
                 timeout=3,capture_output=True,text=True)
    match = re.fullmatch(r's "(:[0-9]+\.[0-9]+)"\s*', result.stdout)
    if not match:
        raise RuntimeError('Exact user systemd manager identity unavailable')
    bus = run(['busctl','--user','call','org.freedesktop.DBus','/org/freedesktop/DBus',
               'org.freedesktop.DBus','GetId'],timeout=3,capture_output=True,text=True)
    bus_id = re.fullmatch(r's "([0-9a-f]{32})"\s*',bus.stdout)
    if not bus_id:raise RuntimeError('Exact user bus identity unavailable')
    return {'uid':os.getuid(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'dbus_owner':match[1],'dbus_id':bus_id[1]}


def read_managed_unit(manifest,role):
    tools = unit_tools()
    unit = tools.UNITS[role]
    path = tools.unit_directory()/unit
    if path.is_symlink() or not path.is_file() or path.read_text()!=tools.unit_text(manifest,role):
        raise RuntimeError('Managed unit fragment differs from reviewed manifest: '+unit)
    result = run(['systemctl','--user','show','--no-pager','--property='+','.join(UNIT_PROPERTIES),'--',unit],
                 timeout=3,capture_output=True,text=True)
    row = dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if (set(row)!=set(UNIT_PROPERTIES) or row['Id']!=unit or row['LoadState']!='loaded' or
            row['FragmentPath']!=str(path) or row['DropInPaths'] or row['UnitFileState'] not in ('static','disabled') or
            row['Restart']!='no' or not row['StateChangeTimestampMonotonic'].isdigit() or
            not row['MainPID'].isdigit() or not re.fullmatch(r'(?:[0-9a-f]{32})?',row['InvocationID'])):
        raise RuntimeError('Exact disabled unit manager/fragment state unavailable: '+unit)
    row['Job'] = row['Job'] or '0'
    return row


def unit_binding(row):
    pid = int(row['MainPID'])
    identity = process_identity(pid) if pid>0 else None
    group = row['ControlGroup']
    if (row['ActiveState']!='active' or row['SubState']!='running' or row['Job']!='0' or identity is None or
            not re.fullmatch('[0-9a-f]{32}',row['InvocationID']) or not group.endswith('/'+row['Id'])):
        raise RuntimeError('Managed unit lacks a live exact invocation/PID/cgroup: '+row['Id'])
    if not any(line.rsplit(':',1)[-1]==group for line in Path(f'/proc/{pid}/cgroup').read_text().splitlines()):
        raise RuntimeError('Managed unit PID escaped its exact cgroup')
    return {'invocation':row['InvocationID'],'process':identity,'control_group':group}


def prepare_unit_start(args,ros):
    tools = unit_tools()
    manifest = tools.load_manifest(args.unit_manifest)
    if (args.profile!='fwmini4' or ros!=[manifest['wrapper'],'ros-vulkan'] or
            (args.domain is not None and args.domain!=manifest['domain']) or
            os.environ.get('ROS_DOMAIN_ID')!=str(manifest['domain']) or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or
            any(manifest['parameters'][key]!=value for key,value in station_launch_parameters(args).items())):
        raise RuntimeError('Unit startup arguments/environment differ from the installed native manifest')
    manager = manager_identity()
    before = {role:read_managed_unit(manifest,role) for role in tools.ROLES}
    if any(row['Job']!='0' or row['ActiveState'] not in ('inactive','failed') or row['MainPID']!='0' for row in before.values()):
        raise RuntimeError('Existing component invocation/job; no unit or bootstrap dispatched')
    return {'deployment':'user-units','launch_id':getattr(args,'launch_id',None) or str(uuid.uuid4()),
            'manifest':manifest,'manifest_sha256':tools.digest(manifest),
            'manager':manager,'before':before,'units':{},'pending':None,'owner_identity':None,
            'ros':ros,'environment':{'ROS_DOMAIN_ID':str(manifest['domain']),'ROS_LOCALHOST_ONLY':'1'},'phase':'prepared'}


NATIVE_RPC = r'''
import json,sys,time
import rclpy
from std_srvs.srv import Trigger
endpoint=sys.argv[1]
if endpoint not in ('/follower/status','/follower/stop','/station/stop','/station/status','/station/stop_status','/station/ev3/connect'):raise ValueError('fixed RPC only')
deadline=time.monotonic()+max(.05,float(sys.argv[2])-1.)
rclpy.init();node=rclpy.create_node('station_lifecycle_check')
try:
    client=node.create_client(Trigger,endpoint)
    if not client.wait_for_service(timeout_sec=max(0.,min(12. if endpoint.endswith('/connect') else 2.,deadline-time.monotonic()))):raise RuntimeError('service unavailable')
    future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=max(0.,min(8. if endpoint.endswith('/connect') else 2.,deadline-time.monotonic())))
    if not future.done() or future.result() is None:raise RuntimeError('RPC acknowledgement unknown')
    response=future.result()
    result={'success':response.success}
    if endpoint=='/follower/status':
        if len(response.message)>65536:raise RuntimeError('oversize status')
        data=json.loads(response.message)
        keys=('kind','fresh','connected','owner_faulted','owner_started_monotonic','sampled_monotonic','position_monotonic','torque_monotonic','torque','arm_id','calibration_sha256','owner_source_sha256')
        result['data']={key:data.get(key) for key in keys}
        result['data']['external_job_pending']=data.get('operator_external_job') is not None
    elif endpoint in ('/station/stop','/station/status','/station/stop_status'):
        if len(response.message)>65536:raise RuntimeError('oversize workflow result')
        data=json.loads(response.message)
        data=data.get('stop',{}) if endpoint in ('/station/status','/station/stop_status') else data
        result['data']={key:data.get(key) for key in ('request_id','pending','ev3_stopped','arm_disarmed')}
    print(json.dumps(result,allow_nan=False))
finally:
    node.destroy_node();rclpy.shutdown()
'''


def native_rpc(record,endpoint,timeout=None):
    timeout = timeout if timeout is not None else (25 if endpoint.endswith('/connect') else 8)
    result = run(record['ros']+['python','-c',NATIVE_RPC,endpoint,str(timeout)],timeout=timeout,
                 capture_output=True,text=True,env={**os.environ,**record['environment']})
    if len(result.stdout)>65536:
        raise RuntimeError('Lifecycle RPC result exceeds bound')
    data = json.loads(result.stdout)
    if not isinstance(data,dict) or data.get('success') is not True:
        raise RuntimeError('Lifecycle RPC has no successful acknowledgement: '+endpoint)
    return data.get('data')


def validate_unit_owner(record,state,now,*,all_off=False,stopping=False):
    expected = record['expected_owner']
    generation = state.get('owner_started_monotonic')
    if (state.get('kind')!='follower_status' or state.get('fresh') is not True or state.get('connected') is not True or
            (not stopping and (state.get('owner_faulted') is True or state.get('external_job_pending') is not False)) or
            any(state.get(key)!=value for key,value in expected.items()) or
            type(generation) not in (int,float) or not math.isfinite(generation) or
            not record['owner_requested_monotonic']<=generation<=now or
            any(type(state.get(key)) not in (int,float) or not math.isfinite(state[key]) or not 0<=now-state[key]<.5
                for key in ('sampled_monotonic','position_monotonic','torque_monotonic'))):
        raise RuntimeError('Fresh exact owned follower identity/status unavailable; record retained')
    identity = dict(expected,owner_started_monotonic=generation)
    if record.get('owner_identity') is not None and identity!=record['owner_identity']:
        raise RuntimeError('Follower owner generation changed; record retained')
    joints = {'shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'}
    torque = state.get('torque')
    if all_off and (not isinstance(torque,dict) or set(torque)!=joints or any(type(v) is not int or v!=0 for v in torque.values())):
        raise RuntimeError('Fresh six-joint torque OFF not confirmed; record retained')
    return identity


STATION_STOP_TIMEOUT = 8.


def fresh_unit_stop(record):
    # Fault/pending work must not prevent STOP for an exact matched generation.
    validate_unit_owner(record,native_rpc(record,'/follower/status'),time.monotonic(),stopping=True)
    errors = []
    request_id = None
    for endpoint in ('/station/stop','/follower/stop'):
        try:
            reply = native_rpc(record,endpoint)
            if endpoint=='/station/stop':
                request_id = reply.get('request_id') if isinstance(reply,dict) else None
                if not isinstance(request_id,str) or not re.fullmatch(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}',request_id):
                    raise RuntimeError('Station STOP admission lacks exact request correlation')
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
            errors.append(endpoint+': '+str(error))
    if request_id is not None:
        deadline = time.monotonic()+STATION_STOP_TIMEOUT
        try:
            while True:
                remaining = deadline-time.monotonic()
                if remaining<=0:raise RuntimeError('Station STOP terminal acknowledgement unknown')
                status = native_rpc(record,'/station/stop_status',timeout=min(8.,remaining))
                if (time.monotonic()<deadline and isinstance(status,dict) and status.get('request_id')==request_id and
                        status.get('pending') is False and status.get('ev3_stopped') is True and status.get('arm_disarmed') is True):
                    break
                time.sleep(min(.05,max(0.,deadline-time.monotonic())))
        except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
            errors.append(str(error))
    try:
        state = native_rpc(record,'/follower/status')
        validate_unit_owner(record,state,time.monotonic(),all_off=True,stopping=True)
        if state.get('external_job_pending') is not False:
            raise RuntimeError('Unresolved manager job after STOP; record retained')
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as error:
        errors.append(str(error))
    if errors:
        record['stop_errors'] = errors
        raise RuntimeError('STOP acknowledgement/cleanup incomplete; no unit stops dispatched; record retained: '+'; '.join(errors))
    record['station_stop_request_id'] = request_id


def terminal_unit_start(record,role,row,pending):
    before = pending['before']
    if (row['Job']!='0' or row['InvocationID']==before['InvocationID'] or
            int(row['StateChangeTimestampMonotonic'])<pending['requested_us']):
        raise RuntimeError('No correlated new manager invocation; startup record retained')
    binding = unit_binding(row)
    record['units'][role] = binding
    record['pending'] = None


def resolved_unit_restart(record,role,current):
    """Adopt only a chain of existing bridge-manager resolved restart receipts."""
    tools = unit_tools()
    old = record['units'][role]['invocation']
    if old==current['InvocationID']:
        return
    if role not in tools.RESTART_ROLES:
        raise RuntimeError('Persistent owner/API invocation changed; record retained')
    journal = Path(record['manifest']['parameters']['operator_maintenance_journal'])
    if journal.exists():
        raise RuntimeError('Unresolved owner-manager restart journal; record retained')
    files = list(journal.parent.glob(journal.name+'.resolved-*'))
    if len(files)>256:
        raise RuntimeError('Restart receipt window exceeds256; explicit inspection required')
    import runpy
    from types import SimpleNamespace
    module = runpy.run_path(str(ROOT/'ros2/src/so101_station_control/so101_station_control/operator_restart.py'))
    params = record['manifest']['parameters']
    validator = SimpleNamespace(_operator_restart_units={params['operator_'+key+'_unit'] for key in ('camera','depth','tags')},
        _operator_station_reset_units={params['operator_'+key+'_unit'] for key in ('movegroup','objects','scene','leader_readonly')},
        _expected_arm_id=record['expected_owner']['arm_id'],_expected_calibration_sha256=record['expected_owner']['calibration_sha256'],
        _owner_source_expected=record['expected_owner']['owner_source_sha256'],
        _operator_restart_source_sha256={name:record['manifest']['sources']['ros2/src/so101_station_control/so101_station_control/'+name+'.py'] for name in ('operator_restart','trajectory_server')})
    transitions = {}
    for path in files:
        if path.stat().st_size>65536:raise RuntimeError('Oversize restart receipt')
        receipt = json.loads(path.read_text());pending,proof=receipt.get('pending',{}),receipt.get('proof',{})
        if pending.get('unit')!=tools.UNITS[role]:continue
        module['OperatorRestartMixin']._operator_restart_validate_record(validator,pending)
        after = proof.get('after',{})
        if (pending['boot_id']!=record['manager']['boot_id'] or pending['owner_generation']!=record['owner_identity']['owner_started_monotonic'] or
                path.name!=journal.name+'.resolved-'+pending['job_id'] or proof.get('job_id')!=pending['job_id'] or
                proof.get('unit')!=pending['unit'] or proof.get('owner_generation')!=pending['owner_generation'] or
                proof.get('kind')!='restart_unit_terminal' or proof.get('success') is not True or
                after.get('Id')!=pending['unit'] or after.get('Job')!='0' or after.get('ActiveState')!='active' or
                not str(after.get('StateChangeTimestampMonotonic','')).isdigit() or
                int(after['StateChangeTimestampMonotonic'])<pending['requested_monotonic_us']):
            continue
        previous = pending['before']['InvocationID']
        next_id = after.get('InvocationID')
        if not isinstance(next_id,str) or not re.fullmatch('[0-9a-f]{32}',next_id) or next_id==previous:continue
        if previous in transitions and transitions[previous]!=next_id:raise RuntimeError('Ambiguous manager restart receipts')
        transitions[previous]=next_id
    visited = set()
    while old!=current['InvocationID']:
        if old in visited or old not in transitions:
            raise RuntimeError('Changed invocation lacks exact correlated owner-manager receipt; record retained')
        visited.add(old);old=transitions[old]


def check_unit_record(record):
    tools = unit_tools()
    tools.validate_manifest(record['manifest'])
    if (record.get('deployment')!='user-units' or tools.digest(record['manifest'])!=record.get('manifest_sha256') or
            record.get('ros')!=[record['manifest']['wrapper'],'ros-vulkan'] or
            record.get('environment')!={'ROS_DOMAIN_ID':str(record['manifest']['domain']),'ROS_LOCALHOST_ONLY':'1'} or
            not set(record['units'])<=set(tools.ROLES) or manager_identity()!=record['manager']):
        raise RuntimeError('Managed unit cohort/manager identity changed; record retained')


def stop_units(path,record):
    check_unit_record(record)
    if record.get('owner_identity') is not None and 'owner' in record['units']:
        owner = read_managed_unit(record['manifest'],'owner')
        if unit_binding(owner)!=record['units']['owner']:
            raise RuntimeError('Persistent owner unit identity changed; record retained')
        try:
            fresh_unit_stop(record)
        except BaseException:
            save(path,record);raise
    if record.get('pending'):
        pending = record['pending'];role=pending['role']
        if role not in unit_tools().ROLES:raise RuntimeError('Unknown pending unit role')
        row = read_managed_unit(record['manifest'],role)
        if row['Job']=='0' and row['ActiveState'] in ('inactive','failed') and row['MainPID']=='0':
            if role=='owner' and row!=pending['before']:
                raise RuntimeError('New failed owner invocation has unproved hardware outcome; record retained')
            record['pending']=None
        else:
            terminal_unit_start(record,role,row,pending)
        save(path,record)
    # Check every identity before any STOP or manager stop. Unknowns retain records.
    for role,binding in record['units'].items():
        row = read_managed_unit(record['manifest'],role)
        resolved_unit_restart(record,role,row)
        actual = unit_binding(row)
        if row['InvocationID']==binding['invocation'] and actual!=binding:
            raise RuntimeError('Managed unit PID/cgroup identity changed; record retained')
        record['units'][role]=actual
    save(path,record)
    if not record['units']:
        path.unlink(missing_ok=True);return
    if 'owner' not in record['units'] or record.get('owner_identity') is None:
        raise RuntimeError('Owned follower identity was never established; record retained')
    for role in reversed(unit_tools().ROLES):
        if role not in record['units']:continue
        check_unit_record(record)
        row = read_managed_unit(record['manifest'],role)
        if unit_binding(row)!=record['units'][role]:
            raise RuntimeError('Unit identity changed before manager stop; record retained')
        record['stopping'] = role;save(path,record)
        requested_us = time.monotonic_ns()//1000
        run(['systemctl','--user','stop','--job-mode=fail','--',unit_tools().UNITS[role]],timeout=26,capture_output=True,text=True)
        after = read_managed_unit(record['manifest'],role)
        if (after['Job']!='0' or after['ActiveState'] not in ('inactive','failed') or after['MainPID']!='0' or
                int(after['StateChangeTimestampMonotonic'])<requested_us):
            raise RuntimeError('Unit stop outcome unknown; record retained')
        del record['units'][role];record.pop('stopping',None);save(path,record)
    path.unlink(missing_ok=True)


def start_units(args,path,record):
    contract = json.loads(args.contract.read_text())
    record['expected_owner'] = {key:contract[key] for key in ('arm_id','calibration_sha256','owner_source_sha256')}
    save(path,record)
    try:
        for role in unit_tools().ROLES:
            check_unit_record(record)
            before = read_managed_unit(record['manifest'],role)
            if before!=record['before'][role]:raise RuntimeError('Unit changed after preflight; no start dispatched')
            pending = {'role':role,'before':before,'requested_us':time.monotonic_ns()//1000}
            record['pending']=pending
            if role=='owner':record['owner_requested_monotonic']=pending['requested_us']/1e6
            save(path,record)
            run(['systemctl','--user','start','--job-mode=fail','--',unit_tools().UNITS[role]],timeout=32,capture_output=True,text=True)
            terminal_unit_start(record,role,read_managed_unit(record['manifest'],role),pending)
            save(path,record)
            if role=='owner':
                deadline=time.monotonic()+15.
                while True:
                    try:
                        record['owner_identity']=validate_unit_owner(record,native_rpc(record,'/follower/status'),time.monotonic())
                        break
                    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError):
                        if time.monotonic()>=deadline:raise
                        time.sleep(.1)
                save(path,record)
        native_rpc(record,'/station/ev3/connect')  # One explicit non-motion connect, never retried.
        fresh_unit_stop(record)
        record['phase']='running all OFF';save(path,record)
        print('Managed native user units started; six torque OFF; no motion or physical qualification asserted.')
    except BaseException as error:
        record['startup_error']=str(error);save(path,record)
        try:
            stop_units(path,record)
        except BaseException as cleanup_error:
            record['cleanup_error']=str(cleanup_error);save(path,record)
        raise


def station_launch_parameters(args):
    """One shared integrated/component recipe; role selection adds no new owner."""
    return {
        'station_root': str(ROOT), 'station_config': str(args.config), 'station_python': str(args.python),
        'state_file': str(args.state_file), 'station_lock': str(args.station_lock), 'log_dir': str(args.log_dir),
        'joint_limit_contract': str(args.contract), 'calibration_dir': str(args.calibration_dir),
        'camera_repair_prefix': str(ROOT/'ros2/.cache/realsense-cpu-texture-bounds/install'),
        'camera_sdk_prefix': str(args.camera_sdk), 'enable_hardware': 'true', 'external_state_only': 'false',
        'rviz': 'true', 'use_station_api': 'true', 'use_operator_panel_api': 'true',
        'use_ev3_tunnel': str(not args.existing_tunnel).lower(), 'web_twin_host': '127.0.0.1',
        'color_mask_backend': args.color_mask_backend,
    }


def start(args, path, lifecycle_lock=None):
    if path.exists():
        raise RuntimeError('Managed station record already exists; inspect it and run stop_station.sh first')
    existing = duplicate_owners()
    if existing:
        raise RuntimeError('Existing station components must be reused or explicitly shut down:\n'+'\n'.join(existing))
    ros = wrapper(args)
    prepared_units = prepare_unit_start(args,ros) if getattr(args,'deployment','integrated')=='user-units' else None
    prerequisites(args, ros)
    if args.bootstrap_owner:
        run([str(args.python), str(ROOT/'tools/freeze_ros_joint_limits.py'), '--config', str(args.config),
             '--state-file', str(args.state_file), '--output', str(args.contract), '--bootstrap-owner',
             '--lock', str(args.station_lock), '--log-dir', str(args.log_dir)], timeout=40)
    if not args.contract.is_file():
        raise RuntimeError('Real joint-limit contract missing; explicitly use --bootstrap-owner with no running owner')
    if prepared_units is not None:
        return start_units(args,path,prepared_units)
    command = ros+['ros2', 'launch', 'so101_station_bringup', 'station_live.launch.py',
        *(key+':='+value for key,value in station_launch_parameters(args).items())]
    args.log_dir.mkdir(parents=True, exist_ok=True)
    output = (args.log_dir/'station-launch.log').open('a') if args.background else None
    try:
        child = subprocess.Popen(command, cwd=ROOT, env=os.environ.copy(), start_new_session=True,
                                 stdout=output, stderr=subprocess.STDOUT if output else None)
    finally:
        if output:
            output.close()
    identity = process_identity(child.pid)
    if identity is None:
        raise RuntimeError('Launch exited before ownership could be recorded')
    record = {'process': identity, 'ros': ros, 'command': command,
              'environment': {key: os.environ[key] for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')
                              if key in os.environ}, 'log': str(args.log_dir/'station-launch.log')}
    save(path, record)
    if lifecycle_lock is not None:
        fcntl.flock(lifecycle_lock, fcntl.LOCK_UN)
    print(f'Managed launch PID {child.pid}; record {path}', flush=True)
    try:
        connected = run(ros+['ros2', 'service', 'call', '/station/ev3/connect',
                             'std_srvs/srv/Trigger', '{}'], timeout=30, capture_output=True, text=True)
        if 'success=True' not in connected.stdout and 'success: true' not in connected.stdout.lower():
            raise RuntimeError('EV3 connect did not acknowledge success: '+connected.stdout)
        print(connected.stdout, end='')
    except (RuntimeError, OSError, subprocess.SubprocessError, KeyboardInterrupt):
        stop_record(path, record)
        raise

    if args.background:
        time.sleep(1)
        if child.poll() is not None:
            if session_members(record['process']['session']):
                stop_record(path, record)
            else:
                path.unlink(missing_ok=True)
            raise RuntimeError('Launch exited; inspect '+record['log'])
        return
    try:
        code = child.wait()
    except KeyboardInterrupt:
        stop_record(path, record)
        code = child.wait()
    finally:
        # stop_record may have finished the managed session while retaining an
        # unknown RPC survivor. Foreground unwinding must preserve that evidence.
        if child.poll() is not None and not (record.get('rpc_cleanup_errors') or record.get('unverified_members')):
            if session_members(record['process']['session']):
                stop_record(path, record)
            else:
                path.unlink(missing_ok=True)
    if code:
        raise RuntimeError(f'Station launch exited with code {code}; inspect terminal output')


def parse_arguments(argv=None):
    home = Path.home()
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/dev/shm'))/'exciting-science'
    station = home/'.local/share/exciting-science/station'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop'])
    parser.add_argument('--profile', choices=['local', 'fwmini4'], default='local')
    parser.add_argument('--deployment',choices=['integrated','user-units'],default='integrated')
    parser.add_argument('--unit-manifest',type=Path,default=station/'fwmini4-units.json')
    parser.add_argument('--launch-id',help='Canonical UUID for this explicitly owned native startup')
    parser.add_argument('--expected-launch-id',help='Stop only this exact native launch UUID')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--python', type=Path, default=station/'venv/bin/python')
    parser.add_argument('--contract', type=Path, default=station/'joint-limits-current.json')
    parser.add_argument('--state-file', type=Path, default=runtime/'arm-state.json')
    parser.add_argument('--station-lock', type=Path, default=Path(os.environ.get('XDG_STATE_HOME', home/'.local/state'))/'exciting-science/station.lock')
    parser.add_argument('--log-dir', type=Path, default=runtime/'arm-owner-logs')
    parser.add_argument('--record', type=Path, default=runtime/'managed-station.json')
    parser.add_argument('--calibration-dir', type=Path, default=home/'git/soarm101-highschool-demos/.calibration')
    parser.add_argument('--camera-sdk', type=Path, default=home/'miniforge3/envs/so101-jazzy')
    parser.add_argument('--wrapper', help='Executable ROS wrapper; use with --wrapper-command')
    parser.add_argument('--wrapper-command', help='Wrapper subcommand (default run; fwmini4 uses ros-vulkan)')
    parser.add_argument('--domain', type=int, help='Explicit ROS domain, 1..232; otherwise retain wrapper default')
    parser.add_argument('--color-mask-backend', choices=['cpu', 'vulkan'], default='cpu')
    parser.add_argument('--bootstrap-owner', action='store_true', help='Explicit passive real-device limits export before launch')
    parser.add_argument('--existing-tunnel', action='store_true', help='Reuse an already established EV3 SSH tunnel')
    parser.add_argument('--background', action='store_true', help='Write launch logs and return after startup')
    args = parser.parse_args(argv)
    for name,action in (('launch_id','start'),('expected_launch_id','stop')):
        value=getattr(args,name)
        if value is None:continue
        if args.action!=action or args.profile!='fwmini4' or args.deployment!='user-units':
            parser.error('--'+name.replace('_','-')+' requires native fwmini4 '+action)
        try:
            canonical=str(uuid.UUID(value))
        except ValueError:
            parser.error('--'+name.replace('_','-')+' must be a canonical UUID')
        if canonical!=value:
            parser.error('--'+name.replace('_','-')+' must be a canonical UUID')
    if args.domain is not None:
        if not 1 <= args.domain <= 232:
            parser.error('--domain must be 1..232')
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ.setdefault('ROS_DOMAIN_ID', '76')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    args.config = (args.config or station/('station-fwmini4-20261022.json' if args.profile == 'fwmini4' else 'station.json')).resolve()
    for name in ('python', 'contract', 'state_file', 'station_lock', 'log_dir', 'record', 'calibration_dir', 'camera_sdk', 'unit_manifest'):
        value = getattr(args,name).expanduser()
        setattr(args,name,value.absolute() if name=='python' and args.deployment=='user-units' else value.resolve())
    return args


def main(argv=None):
    args = parse_arguments(argv)
    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action == 'stop':
            if not args.record.exists():
                if args.expected_launch_id is not None:
                    raise RuntimeError('Expected native launch identity is absent; no commands or signals sent.')
                print('No managed launch record; no commands or signals sent.')
                return
            record=json.loads(args.record.read_text())
            if args.expected_launch_id is not None and (
                    record.get('deployment')!='user-units' or record.get('launch_id')!=args.expected_launch_id):
                raise RuntimeError('Managed native launch identity changed or is missing; record retained; no commands or signals sent.')
            stop_record(args.record, record)
        else:
            start(args, args.record, lifecycle_lock=lock)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print('station: '+str(error), file=sys.stderr)
        raise SystemExit(2)
