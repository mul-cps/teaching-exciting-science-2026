#!/usr/bin/env python3
"""Own one integrated station launch. Startup never enables motion or homes."""
import argparse
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

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


def start(args, path, lifecycle_lock=None):
    if path.exists():
        raise RuntimeError('Managed station record already exists; inspect it and run stop_station.sh first')
    existing = duplicate_owners()
    if existing:
        raise RuntimeError('Existing station components must be reused or explicitly shut down:\n'+'\n'.join(existing))
    ros = wrapper(args)
    prerequisites(args, ros)
    if args.bootstrap_owner:
        run([str(args.python), str(ROOT/'tools/freeze_ros_joint_limits.py'), '--config', str(args.config),
             '--state-file', str(args.state_file), '--output', str(args.contract), '--bootstrap-owner',
             '--lock', str(args.station_lock), '--log-dir', str(args.log_dir)], timeout=40)
    if not args.contract.is_file():
        raise RuntimeError('Real joint-limit contract missing; explicitly use --bootstrap-owner with no running owner')
    command = ros+['ros2', 'launch', 'so101_station_bringup', 'station_live.launch.py',
        f'station_root:={ROOT}', f'station_config:={args.config}', f'station_python:={args.python}',
        f'state_file:={args.state_file}', f'station_lock:={args.station_lock}', f'log_dir:={args.log_dir}',
        f'joint_limit_contract:={args.contract}', f'calibration_dir:={args.calibration_dir}',
        f'camera_repair_prefix:={ROOT}/ros2/.cache/realsense-cpu-texture-bounds/install',
        f'camera_sdk_prefix:={args.camera_sdk}', 'enable_hardware:=true', 'external_state_only:=false',
        'rviz:=true', 'use_station_api:=true', 'use_operator_panel_api:=true',
        f'use_ev3_tunnel:={str(not args.existing_tunnel).lower()}', 'web_twin_host:=127.0.0.1',
        f'color_mask_backend:={args.color_mask_backend}']
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
        if child.poll() is not None:
            if session_members(record['process']['session']):
                stop_record(path, record)
            else:
                path.unlink(missing_ok=True)
    if code:
        raise RuntimeError(f'Station launch exited with code {code}; inspect terminal output')


def main(argv=None):
    home = Path.home()
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/dev/shm'))/'exciting-science'
    station = home/'.local/share/exciting-science/station'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop'])
    parser.add_argument('--profile', choices=['local', 'fwmini4'], default='local')
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
    if args.domain is not None:
        if not 1 <= args.domain <= 232:
            parser.error('--domain must be 1..232')
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ.setdefault('ROS_DOMAIN_ID', '177' if args.profile == 'fwmini4' else '76')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    args.config = (args.config or station/('station-fwmini4-20261022.json' if args.profile == 'fwmini4' else 'station.json')).resolve()
    for name in ('python', 'contract', 'state_file', 'station_lock', 'log_dir', 'record', 'calibration_dir', 'camera_sdk'):
        setattr(args, name, getattr(args, name).expanduser().resolve())
    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action == 'stop':
            if not args.record.exists():
                print('No managed launch record; no commands or signals sent.')
                return
            stop_record(args.record, json.loads(args.record.read_text()))
        else:
            start(args, args.record, lifecycle_lock=lock)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print('station: '+str(error), file=sys.stderr)
        raise SystemExit(2)
