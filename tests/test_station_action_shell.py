import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_quick_wrapper_forwards_typed_command_and_domain(tmp_path):
    captured = tmp_path/'args'
    stub = tmp_path/'conda'
    stub.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$CAPTURE"\nprintf "%s" "$ROS_DOMAIN_ID" > "$CAPTURE.domain"\n')
    stub.chmod(0o755)
    env = dict(os.environ, SO101_CONDA_EXE=str(stub), CAPTURE=str(captured))
    env.pop('ROS_DOMAIN_ID', None)
    result = subprocess.run([str(ROOT/'ros2/scripts/station-action.sh'), 'run', '20'], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    args = captured.read_text()
    assert str(ROOT/'tools/ros_station_action.py') in args
    assert args.endswith('run\n20\n')
    assert captured.with_suffix('.domain').read_text() == '76'


def test_quick_wrapper_rejects_domain_zero(tmp_path):
    stub = tmp_path/'conda'
    stub.write_text('#!/bin/sh\nexit 99\n')
    stub.chmod(0o755)
    result = subprocess.run([str(ROOT/'ros2/scripts/station-action.sh'), 'status'],
        env=dict(os.environ, ROS_DOMAIN_ID='0', SO101_CONDA_EXE=str(stub)), capture_output=True, text=True)
    assert result.returncode == 2
    assert 'disabled' in result.stderr
