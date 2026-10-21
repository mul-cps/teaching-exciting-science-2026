"""Launch a local standard Jupyter kernel; --live opts into read-only ROS capture."""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true', help='instructor only: read existing station topics')
    parser.add_argument('--ros-wrapper', type=Path, help='existing prepared ROS wrapper executable')
    parser.add_argument('--ros-wrapper-command', choices=('run', 'ros-vulkan'), default='run')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    if args.ros_wrapper:
        env['WORKSHOP_ROS_WRAPPER'] = str(args.ros_wrapper.expanduser().resolve())
        env['WORKSHOP_ROS_WRAPPER_COMMAND'] = args.ros_wrapper_command
    env['WORKSHOP_LIVE'] = '1' if args.live else '0'
    command = [sys.executable, '-m', 'jupyterlab', '--ServerApp.ip=127.0.0.1', '--ServerApp.port=8890', '--ServerApp.port_retries=0',
               '--ServerApp.root_dir='+str(root), str(root/'notebooks/Robot_Candy_Factory_Live_SO101.ipynb')]
    subprocess.run(command, cwd=root, env=env, check=True)


if __name__ == '__main__': main()
