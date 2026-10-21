#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
ROS2_DIR="$ROOT_DIR/ros2"
ENVIRONMENT="$ROS2_DIR/conda/environment.yml"
CONDA_ROOT="${SO101_CONDA_ROOT:-$HOME/miniforge3}"
CONDA_EXE="${SO101_CONDA_EXE:-$CONDA_ROOT/condabin/conda}"
ENV_PREFIX="${SO101_CONDA_PREFIX:-$CONDA_ROOT/envs/so101-jazzy}"
BUILD_BASE="$ROS2_DIR/.cache/conda/build"
INSTALL_BASE="$ROS2_DIR/.cache/conda/install"
LOG_BASE="$ROS2_DIR/.cache/conda/log"
ACTION="${1:-help}"
[[ $# -eq 0 ]] || shift

fail() { printf 'so101-conda.sh: %s\n' "$*" >&2; exit 2; }
usage() {
  cat <<'EOF'
Usage: ros2/conda/so101-conda.sh create|fetch|build|clean-build|station-live|realsense-cpu-build|camera-preflight|rviz|servo|plan-smoke|inspect|packages|run|shell|real

  create      Create the dedicated RoboStack ROS 2 Jazzy environment
  fetch       Fetch the pinned upstream SO-ARM101 source and driver patch
  build       Build the pinned workspace in the Conda environment
  clean-build Remove only this checkout's ROS overlay build/install/log prefixes, then rebuild
  station-live Launch the integrated live station (one follower owner and one camera)
  realsense-cpu-build Build/validate the pinned private CPU texture repair; no activation/device access
  camera-preflight Validate private repair sources, SO, SDK and child loader; no camera start
  rviz        Launch the explicitly mocked MoveIt/RViz stack
  servo       Launch MoveIt Servo with mock ros2_control
  plan-smoke  Run IK and planning only; never execute trajectories
  inspect     Inspect controllers, topics, and actions from a running mock
  packages    Check MoveIt Servo, RealSense, and TF package availability
  run         Run a command inside a clean Conda ROS environment
  shell       Open an interactive shell in the Conda ROS environment
  real        Refuse hardware startup; the station owner retains that gate

Set SO101_CONDA_ROOT, SO101_CONDA_EXE, or SO101_CONDA_PREFIX to override the
Miniforge defaults. Station commands default to ROS_DOMAIN_ID=76; an explicit
nonzero domain from 1 through 232 is preserved and domain 0 is rejected.
No ROS package is installed into Conda base.
EOF
}

if [[ "$ACTION" == "-h" || "$ACTION" == "--help" || "$ACTION" == "help" ]]; then
  usage
  exit 0
fi

ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-76}"
[[ "$ROS_DOMAIN_ID_VALUE" =~ ^[0-9]{1,3}$ ]] || fail "ROS_DOMAIN_ID must be an integer from 1 through 232"
ROS_DOMAIN_ID_NUMBER=$((10#$ROS_DOMAIN_ID_VALUE))
((ROS_DOMAIN_ID_NUMBER >= 1 && ROS_DOMAIN_ID_NUMBER <= 232)) || \
  fail "ROS_DOMAIN_ID=0 is disabled; select a domain from 1 through 232"

[[ -x "$CONDA_EXE" ]] || fail "Conda executable is not available: $CONDA_EXE"

# Dependency provenance is checked once here, never by status/image/control ticks.
GID_PREFIX="$ROS2_DIR/.cache/rclpy-publisher-gid-7.1.12/install"
GID_SITE=""
if [[ -r "$GID_PREFIX/share/rclpy_gid_overlay/provenance.json" && -x "$ENV_PREFIX/bin/python" ]]; then
  if ! GID_SITE="$(env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH "$ENV_PREFIX/bin/python" \
      "$ROS2_DIR/conda/record-rclpy-gid-overlay.py" --env-prefix "$ENV_PREFIX" \
      --validate-prefix "$GID_PREFIX")"; then
    GID_SITE=""
  fi
fi

RMW_GID_PREFIX="$ROS2_DIR/.cache/rmw-cyclonedds-gid-2.2.4/install"
GID_LIB=""
if [[ -r "$RMW_GID_PREFIX/share/rmw_gid_overlay/provenance.json" && -x "$ENV_PREFIX/bin/python" ]]; then
  if ! GID_LIB="$(env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH "$ENV_PREFIX/bin/python" \
      "$ROS2_DIR/conda/record-rmw-gid-overlay.py" --env-prefix "$ENV_PREFIX" \
      --validate-prefix "$RMW_GID_PREFIX")"; then
    GID_LIB=""
  fi
fi

clean_conda() {
  local -a rmw_env=() gid_env=() conda_context=()
  local context_name
  while IFS= read -r context_name; do
    case "$context_name" in
      CONDA_PREFIX|CONDA_PREFIX_[0-9]*|CONDA_SHLVL|CONDA_DEFAULT_ENV|CONDA_PROMPT_MODIFIER)
        conda_context+=(-u "$context_name") ;;
    esac
  done < <(compgen -e)
  if [[ -n "$GID_SITE" ]]; then
    gid_env+=("PYTHONPATH=$GID_SITE" "AMENT_PREFIX_PATH=$GID_PREFIX")
  fi
  if [[ -n "$GID_LIB" ]]; then
    gid_env+=("LD_LIBRARY_PATH=$GID_LIB" "AMENT_PREFIX_PATH=$RMW_GID_PREFIX${GID_SITE:+:$GID_PREFIX}")
  fi
  if [[ -n "$GID_LIB" && ( -z "${RMW_IMPLEMENTATION:-}" || "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ) ]]; then
    # Separately pinned Cyclone fork, with a matching public/internal alias.
    # The camera may keep its stock Cyclone backend and the same wire protocol.
    rmw_env=("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp_so101_gid")
  elif [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
    rmw_env=("RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION")
  elif [[ -f "$ENV_PREFIX/lib/librmw_cyclonedds_cpp.so" ]]; then
    rmw_env=("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  fi
  env \
    "${conda_context[@]}" \
    -u AMENT_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH \
    -u PYTHONHOME \
    -u PYTHONUSERBASE \
    -u ROS_DISTRO \
    -u ROS_VERSION \
    -u ROS_PYTHON_VERSION \
    -u ROS_ETC_DIR \
    -u LD_LIBRARY_PATH \
    -u LD_PRELOAD \
    -u PKG_CONFIG_PATH \
    "${gid_env[@]}" \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID_NUMBER" \
    ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}" \
    "${rmw_env[@]}" \
    OPENBLAS_NUM_THREADS="${SO101_BLAS_THREADS:-1}" \
    OMP_NUM_THREADS="${SO101_OMP_THREADS:-1}" \
    MKL_NUM_THREADS="${SO101_BLAS_THREADS:-1}" \
    NUMEXPR_NUM_THREADS="${SO101_BLAS_THREADS:-1}" \
    PYTHONNOUSERSITE=1 \
    CONDA_CHANNEL_PRIORITY=strict \
    "$CONDA_EXE" "$@"
}

conda_run() {
  # Conda activation prepends its own site-packages; restore the validated
  # dependency ahead of it after activation, without validating again.
  clean_conda run --no-capture-output --prefix "$ENV_PREFIX" \
    bash --noprofile --norc -c 'if [[ -n "$1" ]]; then export PYTHONPATH="$1${PYTHONPATH:+:$PYTHONPATH}"; fi; if [[ -n "$2" ]]; then export LD_LIBRARY_PATH="$2${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; export AMENT_PREFIX_PATH="${2%/lib}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"; fi; shift 2; exec "$@"' \
    so101-gid-overlay "$GID_SITE" "$GID_LIB" "$@"
}

ros_run() {
  # Workspace setup can prepend the base rclpy path too. Apply the isolated
  # dependency last, immediately before launching the requested process.
  conda_run bash --noprofile --norc -c \
    '[[ ! -r "$1" ]] || source "$1"; if [[ -n "$2" ]]; then export PYTHONPATH="$2${PYTHONPATH:+:$PYTHONPATH}"; fi; if [[ -n "$3" ]]; then export LD_LIBRARY_PATH="$3${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; export AMENT_PREFIX_PATH="${3%/lib}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"; fi; shift 3; exec "$@"' \
    so101-overlay "$INSTALL_BASE/local_setup.bash" "$GID_SITE" "$GID_LIB" "$@"
}

case "$ACTION" in
  create)
    [[ ! -e "$ENV_PREFIX/conda-meta/history" ]] || fail "environment already exists: $ENV_PREFIX"
    mkdir -p "$(dirname -- "$ENV_PREFIX")"
    clean_conda env create --yes --solver libmamba --prefix "$ENV_PREFIX" --file "$ENVIRONMENT"
    ;;
  fetch)
    "$ROS2_DIR/scripts/so101.sh" fetch
    "$ROS2_DIR/conda/fetch-pick-ik.sh"
    ;;
  build)
    "$ROS2_DIR/scripts/so101.sh" fetch
    "$ROS2_DIR/conda/fetch-pick-ik.sh"
    mkdir -p "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
    # The runtime-only GID alias has no CMake package; compile against stock RMW.
    conda_run env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon --log-base "$LOG_BASE" build \
      --parallel-workers 2 \
      --base-paths "$ROS2_DIR/.cache/upstream/so101-ros-physical-ai" "$ROS2_DIR/.cache/upstream/pick_ik" "$ROS2_DIR/src" \
      --build-base "$BUILD_BASE" \
      --install-base "$INSTALL_BASE" \
      --symlink-install
    ;;
  clean-build)
    rm -rf -- "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
    exec "$0" build
    ;;
  realsense-cpu-build)
    (($# == 0)) || fail "realsense-cpu-build takes no arguments"
    env -u LD_PRELOAD -u LD_LIBRARY_PATH "SO101_CONDA_PREFIX=$ENV_PREFIX" "$ROS2_DIR/conda/build-realsense-cpu-texture-bounds.sh"
    conda_run python "$ROS2_DIR/src/so101_station_perception/so101_station_perception/camera_startup.py" \
      --prefix "$ROS2_DIR/.cache/realsense-cpu-texture-bounds/install" --sdk-prefix "$ENV_PREFIX"
    ;;
  camera-preflight)
    conda_run python "$ROS2_DIR/src/so101_station_perception/so101_station_perception/camera_startup.py" \
      --prefix "$ROS2_DIR/.cache/realsense-cpu-texture-bounds/install" --sdk-prefix "$ENV_PREFIX" "$@"
    ;;
  station-live)
    ros_run ros2 launch so101_station_bringup station_live.launch.py \
      "station_root:=$ROOT_DIR" "camera_repair_prefix:=$ROS2_DIR/.cache/realsense-cpu-texture-bounds/install" "camera_sdk_prefix:=$ENV_PREFIX" "$@"
    ;;
  rviz)
    ros_run ros2 launch so101_station_bringup mock_moveit.launch.py
    ;;
  servo)
    ros_run ros2 launch so101_station_bringup servo_mock.launch.py
    ;;
  plan-smoke)
    ros_run ros2 launch so101_station_bringup plan_only_smoke.launch.py
    ;;
  inspect)
    ros_run bash --noprofile --norc -c 'set -e; ros2 control list_controllers -c /follower/controller_manager; ros2 topic list; ros2 action list'
    ;;
  packages)
    ros_run bash --noprofile --norc -c 'set -e; ros2 pkg prefix moveit_servo; ros2 pkg prefix realsense2_camera; ros2 pkg prefix tf2_ros'
    ;;
  run)
    (($# > 0)) || fail "run requires a command"
    ros_run "$@"
    ;;
  shell)
    ros_run bash --noprofile --norc -i
    ;;
  real)
    fail "real hardware remains gated; Conda adds no device-access or actuation authorization"
    ;;
  *) usage >&2; fail "unknown action '$ACTION'" ;;
esac
