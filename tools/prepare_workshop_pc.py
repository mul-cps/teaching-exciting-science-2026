"""Prepare an isolated classroom environment and launch authenticated local JupyterLab.

Run with system Python; never activate or change the robotics Conda environment.
"""
import argparse
import hashlib
import fcntl
import json
import re
import signal
import socket
import time
import urllib.request
import os
from pathlib import Path
import subprocess
import sys
from urllib.parse import quote, urlencode
import webbrowser
import uuid

ROOT = Path(__file__).resolve().parents[1]
STATE = Path.home() / '.local/share/exciting-science/workshop-20261022'
VENV = STATE / 'venv'
STUDENT = 'notebooks/Robot_Candy_Factory_Student.ipynb'
LIVE = 'notebooks/Robot_Candy_Factory_Live_SO101.ipynb'
RUNTIME = Path(os.environ.get('XDG_RUNTIME_DIR', '/dev/shm')) / 'exciting-science'
STATION_LOCK = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'exciting-science/station.lock'
CONTRACT = Path.home() / '.local/share/exciting-science/station/joint-limits-current.json'


def clean_env():
    env = os.environ.copy()
    for key in ('PYTHONPATH', 'PYTHONHOME', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV'):
        env.pop(key, None)
    env['PATH'] = str(VENV / 'bin') + os.pathsep + os.defpath
    return env


def prepare():
    STATE.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, '-m', 'venv', str(VENV)], check=True)
    python = str(VENV / 'bin/python')
    subprocess.run([python, '-m', 'pip', 'install', '-r', str(ROOT / 'notebooks/requirements.txt')], check=True, env=clean_env())
    subprocess.run([python, '-m', 'pip', 'check'], check=True, env=clean_env())
    freeze = subprocess.check_output([python, '-m', 'pip', 'freeze'], text=True, env=clean_env())
    (STATE / 'requirements-installed.txt').write_text(freeze)
    subprocess.run([python, '-m', 'ipykernel', 'install', '--prefix', str(VENV), '--name', 'python3', '--display-name', 'Workshop Python'], check=True, env=clean_env())
    destinations = [Path.home() / '.local/share/applications']
    # xdg-user-dir respects localized desktops. Do not manufacture another user's desktop.
    try:
        desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
        if desktop != Path.home() and desktop.is_absolute():
            destinations.append(desktop)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass
    choices = [('Student workshop', 'student'), ('Instructor live lab', 'live'), ('Instructor robot and RViz', 'station'), ('Student Blockly simulation', 'blockly-simulation'), ('Instructor Blockly PHYSICAL ROS', 'blockly-physical')]
    for title, mode in choices:
        terminal = 'true' if mode in ('station', 'blockly-simulation', 'blockly-physical') else 'false'
        content = ('[Desktop Entry]\nType=Application\nName=' + title + '\n'
                   'Comment=Open the prepared local workshop notebook\n'
                   f'Exec="{python}" "{Path(__file__).resolve()}" {mode}\n'
                   f'Icon=applications-education\nTerminal={terminal}\nCategories=Education;\n')
        for destination in destinations:
            destination.mkdir(parents=True, exist_ok=True)
            target = destination / f'workshop-{mode}.desktop'
            target.write_text(content)
            target.chmod(0o755)
    print(f'Prepared {VENV}; launch: {python} {Path(__file__).resolve()} student')


def launch(mode):
    notebook = STUDENT if mode == 'student' else LIVE
    if not (ROOT / notebook).is_file():
        raise SystemExit(f'Notebook is not installed: {ROOT / notebook}')
    python = VENV / 'bin/python'
    if not python.is_file():
        raise SystemExit('Run prepare_workshop_pc.py prepare with system Python first.')
    if mode == 'live':
        launcher = ROOT / 'tools/launch_live_workshop.py'
        os.chdir(ROOT)
        os.execve(str(python), [str(python), str(launcher), '--live', '--ros-wrapper', str(Path.home() / '.local/bin/station-fwmini4'), '--ros-wrapper-command', 'ros-vulkan'], clean_env())
        return
    servers = subprocess.check_output([str(python), '-m', 'jupyter', 'server', 'list', '--json'], text=True, env=clean_env())
    for line in servers.splitlines():
        server = json.loads(line)
        if Path(server.get('root_dir', '')).resolve() == ROOT and server.get('hostname') in ('127.0.0.1', 'localhost'):
            url = server['url'] + 'lab/tree/' + quote(notebook) + '?' + urlencode({'token': server.get('token', '')})
            webbrowser.open(url)
            return
    os.chdir(ROOT)
    os.execve(str(python), [str(python), '-m', 'jupyterlab', '--ServerApp.ip=127.0.0.1',
                           '--ServerApp.port=8888', '--ServerApp.port_retries=0',
                           '--ServerApp.root_dir=' + str(ROOT), str(ROOT / notebook)], clean_env())


def station_command(launch_id=None):
    """Validate transferred identities before invoking the existing resident owner."""
    config = Path.home() / '.local/share/exciting-science/station/station-fwmini4-20261022.json'
    settings = json.loads(config.read_text())
    arm = settings['arm']
    calibration = Path(arm['calibration']['path'])
    if hashlib.sha256(calibration.read_bytes()).hexdigest() != arm['calibration']['sha256']:
        raise SystemExit('Saved follower calibration hash mismatch; station was not started.')
    home = json.loads(Path(arm['home_file']).read_text())
    if home['calibration_sha256'] != arm['calibration']['sha256'] or home['usb_identity'] != arm['usb']:
        raise SystemExit('Saved home identity mismatch; station was not started.')
    leader = Path.home() / 'git/soarm101-highschool-demos/.calibration/hs_rig01_leader.json'
    if not leader.is_file():
        raise SystemExit('Saved leader calibration missing; station was not started.')
    serial = arm['usb']['serial']
    devices = [Path('/dev/serial/by-id') / f'usb-1a86_USB_Single_Serial_{serial}-if00',
               Path('/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00')]
    missing = [str(path) for path in devices if not path.exists()]
    if missing:
        raise SystemExit('Connect the identified follower and leader before starting: ' + ', '.join(missing))
    repair = ROOT / 'ros2/.cache/realsense-cpu-texture-bounds/install'
    if not repair.is_dir():
        raise SystemExit('Camera runtime repair is missing; run the documented preparation build.')
    station_manifest(config)
    return [sys.executable, str(ROOT / 'tools/station_process.py'), 'start',
            '--profile', 'fwmini4', '--deployment', 'user-units',
            '--color-mask-backend', 'vulkan', '--bootstrap-owner',
            *(['--launch-id', launch_id] if launch_id is not None else [])]


def station_manifest(config=None):
    """Use the installed, source-pinned native recipe; never guess component units."""
    try:
        from tools import install_station_user_units as units
    except ModuleNotFoundError:
        import install_station_user_units as units
    manifest = units.load_manifest(Path.home() / '.local/share/exciting-science/station/fwmini4-units.json')
    expected = config or Path.home() / '.local/share/exciting-science/station/station-fwmini4-20261022.json'
    if manifest['root'] != str(ROOT) or manifest['parameters']['station_config'] != str(expected):
        raise ValueError('Native unit manifest does not match this workshop checkout/configuration.')
    return units, manifest


def start_station():
    try:
        launch_id = str(uuid.uuid4())
        command = station_command(launch_id)
        units, manifest = station_manifest()
        rviz_command = units.component_command(manifest, 'rviz')
    except (SystemExit, OSError, ValueError, KeyError) as error:
        print(str(error), file=sys.stderr, flush=True)
        if sys.stdin.isatty():
            input('Station was not started. Press Enter to close. ')
        raise SystemExit(1) from error
    # Keep a front-end lock for the whole graph, in addition to the resident owner lock.
    STATE.mkdir(parents=True, exist_ok=True)
    with (STATE / 'station-launch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('This workshop station launcher is already running. Use its existing RViz window.', file=sys.stderr)
            if sys.stdin.isatty():
                input('Press Enter to close. ')
            raise SystemExit(1)
        print('Starting reviewed native station services with passive limits bootstrap. Teleoperation requires separate enable.', flush=True)
        # A failed start owns its own rollback. Never stop a pre-existing station.
        result = subprocess.run(command, check=False)
        if result.returncode:
            print(f'Station startup failed with status {result.returncode}. Inspect the terminal output.', file=sys.stderr)
            raise SystemExit(result.returncode)
        rviz_code = 0
        stop_code = 0
        try:
            rviz_code = subprocess.run(rviz_command, check=False).returncode
        except KeyboardInterrupt:
            rviz_code = 130
        except OSError as error:
            print(f'RViz could not start: {error}', file=sys.stderr)
            rviz_code = 1
        finally:
            # Closing RViz must use the same correlated STOP/owner checks as CLI shutdown.
            stop_command = [sys.executable, str(ROOT / 'tools/station_process.py'), 'stop',
                            '--profile', 'fwmini4', '--deployment', 'user-units',
                            '--expected-launch-id', launch_id]
            try:
                stop_code = subprocess.run(stop_command, check=False).returncode
            except (OSError, KeyboardInterrupt) as error:
                print(f'Station shutdown could not complete: {error}', file=sys.stderr)
                stop_code = 1
            if stop_code:
                print(f'Station shutdown failed with status {stop_code}; retain the managed record and inspect STOP/recovery diagnostics.', file=sys.stderr)
        if rviz_code or stop_code:
            print(f'RViz exited with status {rviz_code}; station shutdown status {stop_code}.', file=sys.stderr)
            raise SystemExit(stop_code or rviz_code)



def blockly_probe(port=8877):
    """Child-only direct loopback read; the parent enforces the total deadline."""
    if not 1 <= port <= 65535:
        raise ValueError('Invalid loopback probe port.')
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            raise RuntimeError('Blockly probe redirects are not allowed.')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    url = f'http://127.0.0.1:{port}'
    with opener.open(url, timeout=1) as response:
        page = response.read(262144).decode()
    token = re.search(r"window\.STATION_TOKEN='([A-Za-z0-9_-]+)'", page)
    if token is None:
        raise RuntimeError('Port 8877 does not serve the prepared Blockly app.')
    request = urllib.request.Request(url + '/api/status', headers={'X-Station-Token': token.group(1)})
    with opener.open(request, timeout=1) as response:
        result = json.loads(response.read(262144))
    if not isinstance(result, dict) or type(result.get('physical')) is not bool:
        raise RuntimeError('Blockly probe returned an invalid mode.')
    return {'physical': result['physical']}


def blockly_status(port=8877):
    try:
        result = subprocess.run([sys.executable, '-I', str(Path(__file__).resolve()), '_blockly-probe',
                                 '--probe-port', str(port)], capture_output=True, text=True, timeout=2,
                                env=clean_env(), check=False)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError('Blockly probe exceeded its absolute two-second deadline.') from error
    if result.returncode:
        raise RuntimeError('Blockly direct loopback probe failed; no mode or server was changed.')
    return json.loads(result.stdout)


def blockly_command(physical):
    script = str(ROOT / 'tools/run_blockly_station.py')
    recordings = STATE / ('blockly-physical-recordings' if physical else 'blockly-simulation-recordings')
    arguments = [script, '--port', '8877', '--recordings', str(recordings)]
    if physical:
        return [str(Path.home() / '.local/bin/station-fwmini4'), 'ros-vulkan', 'python',
                *arguments, '--physical', '--instructor']
    return [sys.executable, *arguments]


def blockly(action):
    # Reuse the existing session/PID-reuse checks and bounded pidfd cleanup.
    sys.path.insert(0, str(ROOT))
    from tools.station_process import process_identity, same_process, cleanup_session, save
    STATE.mkdir(parents=True, exist_ok=True)
    path = STATE / 'blockly-server.json'
    physical = action == 'blockly-physical'
    with (STATE / 'blockly-launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            record = json.loads(path.read_text())
        except FileNotFoundError:
            record = None
        if record is not None:
            if record.get('root') != str(ROOT):
                raise RuntimeError('Blockly ownership record belongs to a different checkout; inspect it before starting.')
            if not same_process(record['process']):
                path.replace(path.with_suffix('.stale-' + str(time.time_ns()) + '.json'))
                record = None
        if record is not None:
            if action == 'blockly-close':
                cleanup_session(record, [(signal.SIGTERM, 35), (signal.SIGKILL, 3)], path=path)
                path.unlink()
                print('Blockly server closed. For physical mode inspect its log and station STOP acknowledgement.')
                return
            if record.get('physical') is not physical:
                raise RuntimeError('Blockly is already running in the other mode. Close it explicitly with blockly-close before changing mode.')
            if blockly_status().get('physical') is not physical:
                raise RuntimeError('Blockly HTTP mode differs from the recorded launch; no new server was started.')
        elif action == 'blockly-close':
            print('No managed Blockly server record; no process was signalled.')
            return
        else:
            for port in (8877, 8878):
                with socket.socket() as probe:
                    # Match HTTPServer's reuse rule so a closed server's TIME_WAIT is not mistaken for a listener.
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try:
                        probe.bind(('127.0.0.1', port))
                    except OSError as error:
                        raise RuntimeError(f'Blockly port {port} is occupied by an unmanaged server. Inspect it before starting.') from error
            env = clean_env()
            if physical:
                env.update(ROS_DOMAIN_ID='177', ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            with (STATE / 'blockly-server.log').open('a') as log:
                child = subprocess.Popen(blockly_command(physical), cwd=ROOT, env=env,
                                         stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            identity = process_identity(child.pid)
            if identity is None:
                raise RuntimeError('Blockly exited before its process identity could be recorded; inspect blockly-server.log.')
            record = {'process': identity, 'members': [identity], 'root': str(ROOT), 'physical': physical}
            save(path, record)
            deadline = time.monotonic() + 20
            while True:
                if child.poll() is not None:
                    raise RuntimeError('Blockly exited during startup; inspect blockly-server.log and its ownership record.')
                try:
                    if blockly_status().get('physical') is physical:
                        break
                except (OSError, ValueError, RuntimeError):
                    pass
                if time.monotonic() >= deadline:
                    raise RuntimeError('Blockly readiness timed out. Inspect the managed process/log; it was not automatically replaced.')
                time.sleep(.1)
    webbrowser.open('http://127.0.0.1:8877')
    print('Blockly ' + ('PHYSICAL ROS' if physical else 'SIMULATION') + ' opened. Close server: prepare_workshop_pc.py blockly-close')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'student', 'live', 'station', 'station-preflight', 'blockly-simulation', 'blockly-physical', 'blockly-close', '_blockly-probe'])
    parser.add_argument('--probe-port', type=int, default=8877, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.action == '_blockly-probe':
        print(json.dumps(blockly_probe(args.probe_port)))
    elif args.action.startswith('blockly-'):
        try:
            blockly(args.action)
        except (OSError, ValueError, RuntimeError, KeyError) as error:
            print(str(error), file=sys.stderr)
            if sys.stdin.isatty():
                input('Blockly was not started or changed. Press Enter to close. ')
            raise SystemExit(1) from error
    elif args.action == 'prepare':
        prepare()
    elif args.action == 'station-preflight':
        print(' '.join(station_command()))
    elif args.action == 'station':
        start_station()
    else:
        launch(args.action)


if __name__ == '__main__':
    main()
