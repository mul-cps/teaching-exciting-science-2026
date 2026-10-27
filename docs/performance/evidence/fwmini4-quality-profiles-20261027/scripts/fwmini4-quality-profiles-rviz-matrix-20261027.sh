#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/bjoern/.local/share/exciting-science/validation/quality-profiles-build-20261027/install/setup.bash
set -u
shared=/home/bjoern/.local/share/exciting-science/validation/rviz-coexistence-20261027
base=/home/bjoern/.local/share/exciting-science/validation/quality-profiles-rviz-20261027
native=/home/bjoern/.local/share/exciting-science/validation/quality-profiles-build-20261027
export PATH=/opt/ros/jazzy/bin:/usr/bin:/bin
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
export QT_QPA_PLATFORM=xcb DISPLAY=:2 XDG_RUNTIME_DIR=/run/user/1002
export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/intel_icd.json
unset GALLIUM_DRIVER MESA_LOADER_DRIVER_OVERRIDE LIBGL_ALWAYS_SOFTWARE LD_PRELOAD
export AMENT_PREFIX_PATH="$shared/assets:${AMENT_PREFIX_PATH}"
native_libpath="$LD_LIBRARY_PATH"
mkdir -p "$base/logs"
children=()
cleanup() {
  if ((${#children[@]})); then
    kill -INT "${children[@]}" 2>/dev/null || true
    sleep 1
    kill -TERM "${children[@]}" 2>/dev/null || true
    for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mapfile -t graphics_args < "$base/graphics-sampler-args.txt"
for profile in realtime medium high_quality; do
  case "$profile" in realtime) scene_profile=vga;; medium) scene_profile=720p;; high_quality) scene_profile=1080p;; esac
  /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -r __ns:=/follower --params-file "$shared/scene/rsp.yaml" > "$base/logs/rsp-$profile.log" 2>&1 &
  rsp_pid=$!; children+=("$rsp_pid")
  /usr/bin/python3 "$shared/scene/visual_scene.py" --profile "$scene_profile" > "$base/logs/scene-$profile.log" 2>&1 &
  scene_pid=$!; children+=("$scene_pid")
  LD_PRELOAD="$shared/probe/libswap_probe.so" /opt/ros/jazzy/lib/rviz2/rviz2 -d "$shared/scene/rviz-coexistence.rviz" > "$base/logs/rviz-$profile.log" 2>&1 &
  rviz_pid=$!; children+=("$rviz_pid")
  /usr/bin/python3 "$shared/rviz-process-sampler-20261027.py" --pid "rviz:$rviz_pid" --pid "scene:$scene_pid" --pid "rsp:$rsp_pid" "${graphics_args[@]}" --output "$base/logs/processes-$profile.jsonl" > "$base/logs/sampler-$profile.log" 2>&1 &
  sampler_pid=$!; children+=("$sampler_pid")
  printf '{"profile":"%s","rviz_pid":%s,"scene_pid":%s,"rsp_pid":%s,"sampler_pid":%s}\n' "$profile" "$rviz_pid" "$scene_pid" "$rsp_pid" "$sampler_pid" > "$base/logs/pids-$profile.json"
  if [[ "$profile" == realtime ]]; then sleep 20; else sleep 8; fi
  kill -0 "$rviz_pid" "$scene_pid" "$rsp_pid" "$sampler_pid"
  /usr/bin/python3 "$shared/rviz-check-probe-ready-20261027.py" "$base/logs/rviz-$profile.log" > "$base/logs/probe-ready-$profile.json"
  export LD_LIBRARY_PATH="$native/scalar-sdk:$native_libpath"
  /usr/bin/python3 /tmp/fwmini4-quality-profiles-runner-20261027.py --mode vulkan --profile "$profile" --prefix "$base/vulkan" --source "$native/source" > "$base/logs/camera-$profile-vulkan.log" 2>&1
  export LD_LIBRARY_PATH="$native_libpath"
  /usr/bin/python3 /tmp/fwmini4-quality-profiles-runner-20261027.py --mode cpu --profile "$profile" --prefix "$base/system-cpu" --source "$native/source" > "$base/logs/camera-$profile-cpu.log" 2>&1
  cleanup
  children=()
  echo "completed $profile"
done
