#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
ROS2_DIR="$ROOT_DIR/ros2"
CACHE="$ROS2_DIR/.cache/upstream"
UPSTREAM_URL="https://github.com/legalaspro/so101-ros-physical-ai.git"
UPSTREAM_REV="58318c905a2c61289fa907de85cb8473322fbe68"
FEETECH_REV="4c0fdbfe16c84c686f8ace09526c52d98d0110ca"
RUNTIME="$ROS2_DIR/scripts/runtime.sh"
ACTION="${1:-help}"
[[ $# -eq 0 ]] || shift

fail() { printf 'so101.sh: %s\n' "$*" >&2; exit 2; }
usage() {
  cat <<'EOF'
Usage: ros2/scripts/so101.sh fetch|build|rviz|servo|plan-smoke|inspect|real

  fetch       Fetch exact upstream + recursive Feetech source into ignored cache
  build       Resolve dependencies and build the pinned ROS 2 Jazzy workspace
  rviz        Launch mock ros2_control + MoveIt + RViz (no USB/hardware)
  plan-smoke  Plan a near-current PickIK joint goal; save without execution
  inspect     Show controllers, topics, and actions from a running mock stack
  real        Refuse unsafe upstream driver startup and explain the gate

The default and all supported launch paths use mock hardware. There is no
real-hardware launch entry point until the read-only driver mode and station
mapping are reviewed against the physical station.
EOF
}

fetch_upstream() {
  mkdir -p "$CACHE"
  local src="$CACHE/so101-ros-physical-ai"
  local fresh_clone=0
  if [[ ! -d "$src/.git" ]]; then
    git clone --no-checkout "$UPSTREAM_URL" "$src"
    fresh_clone=1
  fi
  if ((fresh_clone == 0)); then
    git -C "$src" restore --worktree --staged -- \
      so101_moveit_config/config/joint_limits.yaml \
      so101_moveit_config/config/ompl_planning.yaml \
      so101_description/urdf/so101_arm.urdf.xacro \
      so101_description/urdf/ros2_control/so101_ros2_control.xacro
    if [[ -d "$src/feetech_ros2_driver/.git" || -f "$src/feetech_ros2_driver/.git" ]]; then
      if [[ "$(git -C "$src/feetech_ros2_driver" rev-parse HEAD 2>/dev/null || true)" == "$FEETECH_REV" ]]; then
        git -C "$src/feetech_ros2_driver" restore --worktree --staged -- .
      fi
    fi
    local status actual_head
    status="$(git -C "$src" status --porcelain)"
    actual_head="$(git -C "$src" rev-parse HEAD 2>/dev/null || true)"
    if [[ -n "$status" && "$actual_head" == "$UPSTREAM_REV" ]]; then
      if awk 'substr($0, 1, 2) != "D " { exit 1 }' <<<"$status"; then
        git -C "$src" reset --hard "$UPSTREAM_REV"
        status=""
      else
        fail "upstream cache has local edits: $src"
      fi
    fi
    if [[ -n "$status" && "$actual_head" != "$UPSTREAM_REV" ]] && ! awk 'substr($0, 1, 2) != "D " { exit 1 }' <<<"$status"; then
      fail "upstream cache has local edits: $src"
    fi
  fi
  git -C "$src" fetch --depth=1 origin "$UPSTREAM_REV"
  git -C "$src" checkout --detach "$UPSTREAM_REV"
  git -C "$src" submodule update --init --recursive
  local actual
  actual="$(git -C "$src/feetech_ros2_driver" rev-parse HEAD)"
  [[ "$actual" == "$FEETECH_REV" ]] || fail "Feetech submodule is $actual, expected $FEETECH_REV"
  if [[ -f "$ROS2_DIR/patches/feetech-read-only.patch" ]]; then
    patch --batch --forward -p1 -i "$ROS2_DIR/patches/feetech-read-only.patch" -d "$src"
  fi
  cp "$ROS2_DIR/config/rig01_moveit_limits.yaml" "$src/so101_moveit_config/config/joint_limits.yaml"
  local ompl="$src/so101_moveit_config/config/ompl_planning.yaml"
  if ! grep -q 'default_planning_response_adapters/AddRuckigTrajectorySmoothing' "$ompl"; then
    sed -i '/default_planning_response_adapters\/AddTimeOptimalParameterization/a\  - default_planning_response_adapters/AddRuckigTrajectorySmoothing' "$ompl"
  fi
}

run_ros() {
  "$RUNTIME" run -- bash -lc 'set -eo pipefail; source /opt/ros/jazzy/setup.bash; if [[ -f /ws/install/setup.bash ]]; then source /ws/install/setup.bash; fi; exec "$@"' so101-container "$@"
}

case "$ACTION" in
  help|-h|--help) usage ;;
  fetch) fetch_upstream ;;
  build)
    fetch_upstream
    "$RUNTIME" build
    "$RUNTIME" run -- bash -lc 'set -eo pipefail; source /opt/ros/jazzy/setup.bash; rosdep update; rosdep install --from-paths /ws/ros2/.cache/upstream /ws/ros2/src --ignore-src -r -y --rosdistro jazzy; cd /ws; CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /ws/log build --parallel-workers 2 --base-paths /ws/ros2/.cache/upstream /ws/ros2/src --build-base /ws/build --install-base /ws/install --symlink-install'
    ;;
  rviz)
    run_ros ros2 launch so101_station_bringup mock_moveit.launch.py
    ;;
  servo)
    run_ros ros2 launch so101_station_bringup servo_mock.launch.py
    ;;
  plan-smoke)
    run_ros ros2 launch so101_station_bringup plan_only_smoke.launch.py
    ;;
  inspect)
    run_ros bash -lc 'set -e; ros2 control list_controllers -c /follower/controller_manager; ros2 topic list; ros2 action list'
    ;;
  real)
    fail "real startup blocked: fetch/build applies a tested Feetech read-only patch by default, but the unqualified Rig01 mapping and physical station have not been reviewed for state-only startup. The patched driver's read_only mode is not a motion authorization."
    ;;
  *) usage >&2; fail "unknown action '$ACTION'" ;;
esac
