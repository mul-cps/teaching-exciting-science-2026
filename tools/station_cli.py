"""One workshop entrypoint. Hardware commands retain the existing STOP/owner contracts."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

from tools import install_station_user_units as units
from tools import prepare_workshop_pc as prepared
from tools import station_process as process

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = Path(os.environ.get('XDG_RUNTIME_DIR', '/dev/shm')) / 'exciting-science'


def run(command, **kwargs):
    return subprocess.run(command, check=True, cwd=ROOT, **kwargs)


def manifest(args):
    value = units.load_manifest(args.manifest)
    if value['root'] != str(ROOT):
        raise RuntimeError('Unit manifest belongs to another checkout.')
    return value


def lifecycle_command(args, action):
    command = [sys.executable, str(ROOT / 'tools/station_process.py'), action,
               '--profile', args.profile, '--record', str(RUNTIME / 'managed-station.json')]
    if args.profile == 'fwmini4':
        value = manifest(args)
        command += ['--deployment', 'user-units', '--unit-manifest', str(args.manifest),
                    '--domain', str(value['domain'])]
        for flag, key in (('config', 'station_config'), ('python', 'station_python'),
                          ('contract', 'joint_limit_contract'), ('state-file', 'state_file'),
                          ('station-lock', 'station_lock'), ('log-dir', 'log_dir'),
                          ('calibration-dir', 'calibration_dir'), ('camera-sdk', 'camera_sdk_prefix')):
            command += ['--' + flag, value['parameters'][key]]
        command += ['--color-mask-backend', 'vulkan']
        if value['parameters']['use_ev3_tunnel'] == 'false':
            command += ['--existing-tunnel']
    if action == 'start':
        command += ['--bootstrap-owner']
    return command


def configured_ports(settings):
    arm = settings['arm']
    follower = arm.get('port')
    if not follower:
        serial = arm.get('usb', {}).get('serial')
        if not isinstance(serial, str) or not serial or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for c in serial):
            raise RuntimeError('Configure arm.port or an exact follower USB serial before startup.')
        follower = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_' + serial + '-if00'
    leader = settings.get('leader', {}).get('port') or settings.get('leader_port') or arm.get('leader_port')
    ports = [Path(value).expanduser() for value in (follower, leader) if value]
    if any(not path.is_absolute() for path in ports):
        raise RuntimeError('Configured arm/leader ports must be absolute paths.')
    return ports


def preflight(args):
    """Local validation only; full startup still runs the lifecycle's runtime checks."""
    command = lifecycle_command(args, 'start')
    parsed = process.parse_arguments(command[2:])
    required = [Path(process.wrapper(parsed)[0]), parsed.python, parsed.config,
                parsed.calibration_dir / 'hs_rig01_leader.json',
                ROOT / 'ros2/.cache/realsense-cpu-texture-bounds/install']
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError('Missing prerequisites:\n' + '\n'.join(missing))
    settings = json.loads(parsed.config.read_text())
    arm = settings['arm']
    calibration = Path(arm['calibration']['path'])
    if not calibration.is_absolute():
        calibration = parsed.config.parent / calibration
    import hashlib
    if hashlib.sha256(calibration.read_bytes()).hexdigest() != arm['calibration']['sha256']:
        raise RuntimeError('Follower calibration hash mismatch: ' + str(calibration))
    missing = [str(path) for path in configured_ports(settings) if not path.exists()]
    if missing:
        raise RuntimeError('Connect identified follower and leader:\n' + '\n'.join(missing))
    if args.profile == 'fwmini4':
        expected = Path.home() / '.local/share/exciting-science/station/fwmini4-units.json'
        if args.manifest != expected and not args.headless:
            raise RuntimeError('GUI launcher requires the prepared default manifest; custom manifests require --headless.')
        if not args.headless:
            prepared.station_command()  # Saved home and prepared checkout identities.
    print('Local prerequisites passed. Startup additionally verifies overlay, manager and EV3 readiness.')


def gui_command(physical=False,skills=None):
    if skills is not None and not physical: raise ValueError('private taught skills require explicit physical mode')
    # Ownership checks must still work if the private library is later removed.
    skills=str(Path(skills).expanduser().resolve()) if skills is not None else None
    return [sys.executable, str(ROOT / 'station'), '_gui', *(['--physical', '--instructor'] if physical else []), *(['--skills',skills] if skills is not None else [])]


def gui_record(path):
    record = json.loads(path.read_text())
    if record.get('root') != str(ROOT) or record.get('command') != gui_command(record.get('physical', False),record.get('skills')):
        raise RuntimeError('Unknown GUI ownership record; no process signalled.')
    if process.same_process(record['process']):
        actual = Path(f"/proc/{record['process']['pid']}/cmdline").read_bytes().rstrip(b'\0').split(b'\0')
        if actual != [os.fsencode(word) for word in gui_command(record.get('physical', False),record.get('skills'))]:
            raise RuntimeError('GUI process command identity changed; no process signalled.')
    return record


def start_gui(args):
    RUNTIME.mkdir(parents=True, exist_ok=True)
    path = RUNTIME / 'station-gui.json'
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if path.exists():
            raise RuntimeError('Managed GUI record exists; inspect status and stop it first.')
        command = gui_command(args.physical,getattr(args,'skills',None))
        child = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
        identity = process.process_identity(child.pid)
        if identity is None:
            raise RuntimeError('Prepared GUI launcher exited before ownership could be recorded.')
        record = dict(root=str(ROOT), command=command, physical=args.physical,skills=getattr(args,'skills',None), process=identity, members=[identity])
        process.save(path, record)
        try:
            code = child.wait()
        except KeyboardInterrupt:
            stop(args)
            code = child.wait()
        finally:
            if path.exists():
                process.cleanup_session(gui_record(path), [(signal.SIGTERM, 35), (signal.SIGKILL, 3)], path=path, child=child)
                path.unlink(missing_ok=True)
        if code:
            raise RuntimeError(f'Prepared station GUI exited with status {code}.')


def owned_station(path):
    record = json.loads(path.read_text())
    if record.get('deployment') == 'user-units':
        units.validate_manifest(record['manifest'])
        if record['manifest']['root'] != str(ROOT):
            raise RuntimeError('Station ownership belongs to another checkout.')
    else:
        allowed = ([str(ROOT / 'ros2/conda/so101-conda.sh'), 'run'],
                   [str(Path.home() / '.local/bin/station-fwmini4'), 'ros-vulkan'])
        if (record.get('ros') not in allowed or
                record.get('command', [])[:6] != record.get('ros', []) + ['ros2', 'launch', 'so101_station_bringup', 'station_live.launch.py'] or
                'station_root:=' + str(ROOT) not in record.get('command', [])):
            raise RuntimeError('Unknown station ownership record; no commands or signals sent.')
    return record


def stop(args):
    # Validate every local record before any action. Existing helpers verify PID/unit identities.
    path = RUNTIME / 'managed-station.json'
    record = owned_station(path) if path.exists() else None
    gui = RUNTIME / 'station-gui.json'
    if gui.exists():
        gui_record(gui)
    prepared.blockly('blockly-close')
    if record is not None:
        command = [sys.executable, str(ROOT / 'tools/station_process.py'), 'stop', '--record', str(path)]
        run(command)
    elif args.profile == 'fwmini4' and args.manifest.exists():
        value = manifest(args)
        for role in units.ROLES:
            row = process.read_managed_unit(value, role)
            if row['Job'] != '0' or row['MainPID'] != '0' or row['ActiveState'] not in ('inactive', 'failed'):
                raise RuntimeError('Active unit without lifecycle ownership; retain records and use reviewed recovery: ' + units.UNITS[role])
    if gui.exists():
        process.cleanup_session(gui_record(gui), [(signal.SIGTERM, 35), (signal.SIGKILL, 3)], path=gui)
        gui.unlink(missing_ok=True)
    print('Managed station and Blockly shutdown completed. Unmanaged apps require manual inspection.')


def status(args):
    result = {}
    for name, path, reader in (
            ('station', RUNTIME / 'managed-station.json', owned_station),
            ('gui', RUNTIME / 'station-gui.json', gui_record)):
        if path.exists():
            record = reader(path)
            result[name] = {'record': str(path), 'state': 'recorded; runtime health not probed'}
            if 'process' in record:
                result[name]['identity_matches'] = process.same_process(record['process'])
        else:
            result[name] = {'state': 'no managed record'}
    path = prepared.STATE / 'blockly-server.json'
    if path.exists():
        value = json.loads(path.read_text())
        if value.get('root') != str(ROOT):
            raise RuntimeError('Blockly record belongs to another checkout.')
        result['blockly'] = dict(physical=value['physical'], identity_matches=process.same_process(value['process']))
    else:
        result['blockly'] = {'state': 'no managed record'}
    if args.profile == 'fwmini4' and args.manifest.exists():
        value = manifest(args)
        result['units'] = {role: {key: row[key] for key in ('ActiveState', 'MainPID', 'InvocationID')}
                           for role in units.ROLES for row in [process.read_managed_unit(value, role)]}
    print(json.dumps(result, indent=2))


def boundary(path, root):
    if not path.is_relative_to(root) or path == root:
        raise RuntimeError('Refusing target outside managed boundary: ' + str(path))
    for part in (path, *path.parents):
        if part.is_symlink():
            raise RuntimeError('Refusing symlink boundary: ' + str(part))


def tracked(path):
    result = run(['git', 'ls-files', '-z', '--', str(path.relative_to(ROOT))], capture_output=True)
    return bool(result.stdout)


def purge_targets(args):
    paths = [ROOT / name for name in ('.pytest_cache', '.ruff_cache', 'build', 'ros2/build', 'ros2/install', 'ros2/log')]
    if args.runtime:
        paths += [ROOT / 'ros2/.cache' / name for name in ('conda', 'realsense-cpu-texture-bounds',
                  'rclpy-publisher-gid-7.1.12', 'rmw-cyclonedds-gid-2.2.4')]
    for name in ('tools', 'tests', 'workshop', 'ros2/src'):
        base = ROOT / name
        if base.is_symlink():
            raise RuntimeError('Refusing symlink cache search: ' + str(base))
        for directory, children, _ in os.walk(base, followlinks=False):
            children[:] = [child for child in children if not (Path(directory) / child).is_symlink()]
            if Path(directory).name == '__pycache__':
                paths.append(Path(directory)); children[:] = []
    targets = []
    for path in paths:
        boundary(path, ROOT)
        if path.exists():
            if tracked(path):
                raise RuntimeError('Refusing generated directory containing tracked source: ' + str(path))
            targets.append(path)
    if args.uninstall:
        value = manifest(args)
        for role in units.ROLES:
            path = units.unit_directory() / units.UNITS[role]
            boundary(path, units.unit_directory())
            if path.exists():
                if path.read_text() != units.unit_text(value, role):
                    raise RuntimeError('Refusing modified/unowned registration: ' + str(path))
                targets.append(path)
        for title, mode in (('Student workshop', 'student'), ('Instructor live lab', 'live'),
                            ('Instructor robot and RViz', 'station'),
                            ('Student Blockly simulation', 'blockly-simulation'),
                            ('Instructor Blockly PHYSICAL ROS', 'blockly-physical')):
            path = Path.home() / '.local/share/applications' / f'workshop-{mode}.desktop'
            boundary(path, path.parent)
            if path.exists():
                terminal = 'true' if mode in ('station', 'blockly-simulation', 'blockly-physical') else 'false'
                expected = ('[Desktop Entry]\nType=Application\nName=' + title + '\n'
                            'Comment=Open the prepared local workshop notebook\n'
                            f'Exec="{prepared.VENV / "bin/python"}" "{ROOT / "tools/prepare_workshop_pc.py"}" {mode}\n'
                            f'Icon=applications-education\nTerminal={terminal}\nCategories=Education;\n')
                if path.read_text() != expected:
                    raise RuntimeError('Refusing unknown desktop registration: ' + str(path))
                targets.append(path)
    return targets


def purge(args):
    targets = purge_targets(args)
    print(('DELETE after verified STOP' if args.yes else 'DRY RUN; no processes stopped or files removed') + ':')
    for path in targets:
        print(path)
    if not args.yes:
        return
    stop(args)
    # Revalidate after STOP: no source or registration change may widen deletion.
    if purge_targets(args) != targets:
        raise RuntimeError('Purge targets changed during STOP; nothing deleted.')
    for path in targets:
        boundary(path, ROOT if path.is_relative_to(ROOT) else path.parent)
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    if args.uninstall:
        run(['systemctl', '--user', 'daemon-reload'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog='Start opens the prepared full station/RViz. Start with --physical --instructor to include physical Blockly. Purge defaults to a dry run. No unmanaged processes are killed.')
    parser.add_argument('action', choices=('start', 'stop', 'status', 'doctor', 'purge', 'sim', 'notebook', 'blockly', '_gui'), metavar='{start,stop,status,doctor,purge,sim,notebook,blockly}')
    parser.add_argument('--json', action='store_true', help='Machine-readable doctor report')
    parser.add_argument('--profile', choices=('fwmini4', 'local'), default='fwmini4')
    parser.add_argument('--manifest', type=Path, default=Path(os.environ.get('STATION_MANIFEST', str(Path.home() / '.local/share/exciting-science/station/fwmini4-units.json'))))
    parser.add_argument('--headless', action='store_true', help='Start the managed ROS stack without the prepared GUI')
    parser.add_argument('--physical', action='store_true', help='Use real ROS hardware for Blockly')
    parser.add_argument('--instructor', action='store_true', help='Explicit instructor authorization for physical Blockly')
    parser.add_argument('--skills',type=Path,help='Private commissioned taught skill directory; requires physical instructor Blockly')
    parser.add_argument('--live', action='store_true', help='Open the instructor live notebook')
    parser.add_argument('--yes', action='store_true', help='Actually purge after successful managed STOP')
    parser.add_argument('--runtime', action='store_true', help='Include the four generated ROS/Conda runtime prefixes in purge')
    parser.add_argument('--uninstall', action='store_true', help='Also remove verified project user units and application shortcuts')
    args = parser.parse_args(argv)
    args.manifest = args.manifest.expanduser().absolute()
    try:
        if args.skills is not None:
            if not args.physical or args.action not in {'start','blockly','_gui'}:
                raise ValueError('private taught skills require explicit physical Blockly startup')
            args.skills=prepared.private_skills_directory(args.skills)
        if args.physical and not args.instructor:
            raise RuntimeError('Physical Blockly requires --physical --instructor.')
        if args.action == '_gui':
            prepared.start_station(
                before_rviz=(lambda: prepared.blockly('blockly-physical',**({'skills':args.skills} if args.skills is not None else {}))) if args.physical else None,
                before_stop=(lambda: prepared.blockly('blockly-close')) if args.physical else None)
        elif args.action == 'start':
            if args.physical and (args.headless or args.profile != 'fwmini4'):
                raise RuntimeError('One-command physical Blockly requires the prepared fwmini4 GUI profile.')
            if args.physical and (prepared.STATE / 'blockly-server.json').exists():
                raise RuntimeError('Blockly record already exists; inspect status and stop it before full startup.')
            preflight(args)
            if args.profile == 'fwmini4' and not args.headless:
                start_gui(args)
            else:
                run(lifecycle_command(args, 'start') + ['--background'])
        elif args.action == 'stop':
            stop(args)
        elif args.action == 'status':
            status(args)
        elif args.action == 'doctor':
            from tools import station_doctor
            report = station_doctor.collect(args, sys.modules[__name__])
            print(station_doctor.render(report, as_json=args.json))
            return 1 if any(row['state'] == 'blocked' for row in report['checks']) else 0
        elif args.action == 'purge':
            purge(args)
        elif args.action == 'notebook':
            prepared.launch('live' if args.live else 'student')
        else:
            physical = args.action == 'blockly' and args.physical
            if physical and not args.instructor:
                raise RuntimeError('Physical Blockly requires --physical --instructor.')
            if physical and args.profile != 'fwmini4':
                raise RuntimeError('Physical Blockly uses the prepared fwmini4 ROS profile.')
            prepared.blockly('blockly-physical' if physical else 'blockly-simulation',**({'skills':args.skills} if args.skills is not None else {}))
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.SubprocessError) as error:
        print('station: ' + str(error), file=sys.stderr)
        return 1
    return 0
