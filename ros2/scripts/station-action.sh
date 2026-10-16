#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
exec "$ROOT_DIR/ros2/conda/so101-conda.sh" run python "$ROOT_DIR/tools/ros_station_action.py" "$@"
