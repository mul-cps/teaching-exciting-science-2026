#!/usr/bin/env bash
set -euo pipefail
export ROS_DOMAIN_ID=177
export ROS_LOCALHOST_ONLY=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_ROOT"
ROS_WRAPPER="${SO101_TEST_ROS_WRAPPER:-$TASK_ROOT/ros2/conda/so101-conda.sh}"
[[ -x "$ROS_WRAPPER" ]] || { echo "ROS test wrapper unavailable: $ROS_WRAPPER" >&2; exit 2; }
exec "$ROS_WRAPPER" run python -c '
import os,sys,datetime,json
assert os.environ.get("ROS_DOMAIN_ID")=="177", "supervisor native tests require fixed isolated domain177"
assert os.environ.get("ROS_LOCALHOST_ONLY")=="1"
assert os.environ.get("PYTEST_DISABLE_PLUGIN_AUTOLOAD")=="1"
sys.path[:0]=[os.getcwd(),os.path.join(os.getcwd(),"tests"),os.path.join(os.getcwd(),"ros2/src/so101_station_perception")]
record=lambda event:print(json.dumps(dict(event=event,pid=os.getpid(),ppid=os.getppid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),domain=os.environ["ROS_DOMAIN_ID"],argv=sys.argv[1:])),flush=True)
record("isolated_pytest_start")
try:
 import pytest
 result=pytest.main(sys.argv[1:])
finally:record("isolated_pytest_end")
raise SystemExit(result)
' "$@"
