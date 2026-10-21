#!/usr/bin/env python3
"""Export fresh idle owner bounds for the resident MoveIt launch.

For a new source cohort, use --bootstrap-owner or start its --observe-only owner.
A healthy idle holding owner can also export without changing torque.
The resulting contract may survive owner restart, but every motion compares it
with that owner's actual calibration, source hashes and firmware/admitted bounds.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess
import contextlib
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workshop.station.recovery_types import owner_source_sha256
from workshop.station.ros_trajectory import freeze_joint_limit_contract, resolve_joint_limit_settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--state-file', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--bootstrap-owner', action='store_true',
        help='Briefly start a new observe-only owner, export, then close it before launch; existing owner must be stopped')
    parser.add_argument('--lock', type=Path, help='Same station lock used by the resident launch')
    parser.add_argument('--log-dir', type=Path, help='Required with --bootstrap-owner')
    parser.add_argument('--launch', action='store_true', help='Launch MoveIt with exported limits after passive bootstrap exits')
    parser.add_argument('launch_arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    args.config = args.config.resolve()
    args.state_file = args.state_file.resolve()
    args.output = args.output.resolve()
    if args.bootstrap_owner and (args.lock is None or args.log_dir is None):
        parser.error('--bootstrap-owner requires --lock and --log-dir')
    settings = resolve_joint_limit_settings(json.loads(args.config.read_text())["arm"], args.config)
    path = Path(settings["calibration"]["path"])
    if not path.is_absolute():
        path = Path(__file__).resolve().parents[1] / path
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != settings["calibration"]["sha256"]:
        parser.error('calibration hash mismatch')
    root = Path(__file__).resolve().parents[1]
    sources = owner_source_sha256(root / 'workshop/station')
    with contextlib.ExitStack() as stack:
        process = None
        if args.bootstrap_owner:
            try:
                previous_session = json.loads(args.state_file.read_text()).get('owner_session_id')
            except (OSError, ValueError):
                previous_session = None
            boot_stamp = time.monotonic()
            args.log_dir.mkdir(parents=True, exist_ok=True)
            log = stack.enter_context((args.log_dir / 'limit-export-owner.log').open('a'))
            process = subprocess.Popen([sys.executable, '-m', 'workshop.station.arm_session',
                '--config', str(args.config), '--state-file', str(args.state_file),
                '--logdir', str(args.log_dir), '--lock', str(args.lock), '--observe-only',
                '--persistent-idle'], cwd=root, stdin=subprocess.PIPE, stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + (15 if process is not None else 0)
            while True:
                try:
                    state = json.loads(args.state_file.read_text())
                    if process is not None and (state.get('owner_session_id') == previous_session or state.get('monotonic', 0) < boot_stamp):
                        raise ValueError('waiting for new observe-only owner generation')
                    contract = freeze_joint_limit_contract(settings, json.loads(data), state,
                        sources, now=time.monotonic())
                    break
                except (OSError, ValueError, KeyError):
                    if process is None or process.poll() is not None or time.monotonic() >= deadline:
                        raise
                    time.sleep(.05)
        finally:
            if process is not None:
                process.stdin.close()  # EOF closes the passive owner; no motion command.
                try:
                    code = process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)
                    raise RuntimeError('passive owner did not exit; refusing planner launch')
                if code:
                    raise RuntimeError(f'passive owner failed ({code}); see {args.log_dir}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + '.tmp')
    temporary.write_text(json.dumps(contract, indent=2, allow_nan=False) + '\n')
    temporary.replace(args.output)
    print(f'Joint-limit contract saved: {args.output}')
    if args.launch:
        extra = args.launch_arguments
        if extra[:1] == ['--']:
            extra = extra[1:]
        command = ['ros2', 'launch', 'so101_station_control', 'moveit_resident.launch.py',
            *extra, f'station_root:={root}', f'station_python:={sys.executable}',
            f'station_config:={args.config}', f'state_file:={args.state_file}',
            f'joint_limit_contract:={args.output}']
        if args.lock is not None:
            command.append(f'station_lock:={args.lock}')
        if args.log_dir is not None:
            command.append(f'log_dir:={args.log_dir}')
        raise SystemExit(subprocess.call(command, cwd=root))


if __name__ == '__main__':
    main()
