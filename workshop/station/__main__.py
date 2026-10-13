"""Explicit finite station activation; detection and simulation never open hardware."""
import argparse
from importlib.resources import files
import json
from pathlib import Path
import signal
import sys
import time

from .config import WORKFLOWS, load_config, validate_config
from .coordinator import run_cycle
from .discovery import discover_arm, discover_ev3, discover_serial
from .locking import state_dir, station_lock, station_lock_path


class HealthChecked:
    """Check the EV3 lease before and after every acknowledged operation."""
    def __init__(self, device, ev3): self.device, self.ev3 = device, ev3
    def __getattr__(self, name):
        method = getattr(self.device, name)
        def call(*args):
            check = getattr(self.ev3, 'check_health', lambda: None)
            check()
            result = method(*args)
            check()
            return result
        return call


def static_arm_admission(config):
    """Reuse the adapter's pure pose/route validators before device connection.

    These private helpers are deliberately shared with physical preflight: this
    disconnected instance never calls connect, opens a port or mutates hardware.
    Current encoders and hardware calibration still require runtime preflight.
    """
    from .arm import LeRobotArm
    from workshop.safe_home import load_home
    arm = LeRobotArm(config)
    arm._calibration()
    arm._bounds()
    settings = config['arm']
    if Path(settings['calibration']['path']).name != settings['id'] + '.json':
        raise ValueError('calibration filename must match arm ID')
    home = arm._pose(load_home(settings['home_file'], settings['id'], settings['calibration']['sha256']))
    if settings['poses']['home'] != home:
        raise ValueError('taught home differs from measured home')
    for pose in settings['poses'].values(): arm._pose(pose)
    for name in ('home', 'clearance', 'load_piece', 'pick_and_place'):
        paths = settings['trajectories'][name]
        for route in paths.values() if name == 'pick_and_place' else [paths]:
            for first, second in zip(route, route[1:]):
                arm._segment(settings['poses'][first], settings['poses'][second])


def make_devices(config, simulate):
    if simulate:
        from .simulation import SimEV3, SimArm
        ev3 = SimEV3()
        return ev3, SimArm(ev3=ev3)
    from .arm import LeRobotArm
    if config['ev3']['firmware'] == 'original':
        from .ev3_usb import OriginalEV3
        ev3 = OriginalEV3(config)
    else:
        from .ev3_pybricks import PybricksEV3
        ev3 = PybricksEV3(config)
    return ev3, LeRobotArm(config, abort_event=ev3.cancel)


def startup(ev3, arm, log):
    if hasattr(ev3, 'connect') and ev3.connect() is False:
        raise RuntimeError('EV3 connect acknowledgement failed')
    # The lease exists after EV3 connect, including during arm preflight.
    e, a = HealthChecked(ev3, ev3), HealthChecked(arm, ev3)
    if hasattr(arm, 'connect') and a.connect() is False:
        raise RuntimeError('arm connect acknowledgement failed')
    for raw, device in ((ev3, e), (arm, a)):
        if hasattr(raw, 'preflight') and device.preflight() is False:
            raise RuntimeError('preflight acknowledgement failed')
    for device, method in ((e, 'stop'), (e, 'wait_stopped'), (a, 'home'),
                           (a, 'clear'), (a, 'wait_clear'), (e, 'home')):
        if getattr(device, method)() is False:
            raise RuntimeError(method + ' acknowledgement failed')
    log({'timestamp': time.time(), 'event': 'startup_complete'})


def execute(config, ev3, arm, log, startup=None):
    error = None
    try:
        (startup if startup is not None else lambda: globals()['startup'](ev3, arm, log))()
        for _ in range(config['cycle_count']):
            run_cycle(config, HealthChecked(ev3, ev3), HealthChecked(arm, ev3), log)
    except BaseException as fault:
        error = fault
        try: log({'timestamp': time.time(), 'event': 'cancelled' if isinstance(fault, KeyboardInterrupt) else 'fault', 'error': str(fault), 'notes': getattr(fault, '__notes__', [])})
        except BaseException: pass
        raise
    finally:
        failures = []
        # Never call a fallible logger until all four independent attempts finish.
        for method in ('stop', 'close'):
            for name, device in (('ev3', ev3), ('arm', arm)):
                try:
                    if hasattr(device, method): getattr(device, method)()
                except BaseException as fault:
                    failures.append((name, method, fault))
        for name, method, fault in failures:
            if error is not None: error.add_note(f'{name}.{method} cleanup failed: {fault}')
            try: log({'timestamp': time.time(), 'event': 'cleanup_failure', 'device': name, 'operation': method, 'error': str(fault)})
            except BaseException: pass
        if failures and error is None: raise failures[0][2]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--simulate', action='store_true')
    modes.add_argument('--detect', action='store_true', help='read-only local USB inventory; no SSH or device open')
    modes.add_argument('--activate', action='store_true')
    parser.add_argument('--workflow', choices=WORKFLOWS)
    parser.add_argument('--log', type=Path, default=state_dir() / 'station.jsonl')
    parser.add_argument('--lock', type=Path, default=station_lock_path())
    args = parser.parse_args(argv)
    try:
        if args.detect:
            print(json.dumps({'original_ev3': discover_ev3(), 'arm_adapters': discover_arm(),
                              'serial_devices': discover_serial(), 'pybricks': 'remote identity not probed; configure pinned SSH separately'}, indent=2))
            return 0
        with station_lock(args.lock):
            config = load_config(args.config) if args.config else json.loads(files('workshop.station').joinpath('resources/station.example.json').read_text())
            if args.workflow: config['workflow'] = args.workflow
            if args.activate and args.config is None: raise ValueError('--activate requires --config')
            validate_config(config, physical=args.activate)
            if args.activate and not config['automatic_start']:
                raise ValueError('automatic_start must be enabled after supervised commissioning')
            if args.activate: static_arm_admission(config)
            args.log.parent.mkdir(parents=True, exist_ok=True)
            with args.log.open('a', buffering=1) as stream:
                def log(record): stream.write(json.dumps(record, allow_nan=False) + '\n'); stream.flush()
                ev3, arm = make_devices(config, args.simulate)
                previous = signal.getsignal(signal.SIGTERM)
                def cancel(signum, frame):
                    if hasattr(ev3, 'cancel'): ev3.cancel.set()
                    raise KeyboardInterrupt('termination requested')
                signal.signal(signal.SIGTERM, cancel)
                try: execute(config, ev3, arm, log)
                finally: signal.signal(signal.SIGTERM, previous)
            print(f"{'simulation' if args.simulate else 'activation'} complete: {config['cycle_count']} cycles; log {args.log}")
        return 0
    except KeyboardInterrupt:
        print('station cancelled; shutdown attempted', file=sys.stderr)
        return 130
    except (Exception,) as error:
        print(f'station error: {error}', file=sys.stderr)
        for note in getattr(error, '__notes__', []): print(note, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
