#!/usr/bin/env bash
# Existing ROS environment only; no ROS services or hardware device access.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
wrapper="${WORKSHOP_ROS_WRAPPER:-$root/ros2/conda/so101-conda.sh}"
operation="${WORKSHOP_ROS_WRAPPER_COMMAND:-run}"
case "$operation" in
  run|ros-vulkan) ;;
  *) printf 'Observer wrapper command must be run or ros-vulkan\n' >&2; exit 2 ;;
esac
[[ -x "$wrapper" ]] || { printf 'Prepared ROS wrapper missing: %s\n' "$wrapper" >&2; exit 2; }
exec timeout --signal=TERM --kill-after=2s 18s "$wrapper" "$operation" python "$root/tools/live_observer_ros.py" "${1:-3}"
