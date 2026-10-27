#!/usr/bin/env bash
# Requires the caller's native ROS environment and authorized private X display.
# Starts exactly RSP + synthetic visual publisher + RViz. No camera/control owner.
set -euo pipefail
BUNDLE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
profile=${1:-vga}
case "$profile" in vga|720p|1080p) ;; *) echo 'profile must be vga, 720p or 1080p' >&2; exit 2 ;; esac
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-179}
[[ "$ROS_DOMAIN_ID" == 179 ]] || { echo 'requires isolated ROS_DOMAIN_ID=179' >&2; exit 2; }
[[ -n "${DISPLAY:-}" ]] || { echo 'requires caller-authorized private DISPLAY' >&2; exit 2; }
export ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=xcb
export AMENT_PREFIX_PATH="$BUNDLE${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
unset GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE LIBGL_ALWAYS_SOFTWARE
children=()
cleanup() {
  if ((${#children[@]})); then
    kill "${children[@]}" 2>/dev/null || true
    for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/follower --params-file "$BUNDLE/rsp.yaml" >"$BUNDLE/rsp-$profile.log" 2>&1 &
children+=("$!")
python3 "$BUNDLE/visual_scene.py" --profile "$profile" >"$BUNDLE/visual-scene-$profile.log" 2>&1 &
children+=("$!")
rviz2 -d "$BUNDLE/rviz-coexistence.rviz" >"$BUNDLE/rviz-$profile.log" 2>&1 &
children+=("$!")
wait "${children[2]}"
