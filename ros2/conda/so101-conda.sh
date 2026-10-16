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
Usage: ros2/conda/so101-conda.sh create|fetch|build|clean-build|station-live|rviz|servo|plan-smoke|inspect|packages|run|shell|real

  create      Create the dedicated RoboStack ROS 2 Jazzy environment
  fetch       Fetch the pinned upstream SO-ARM101 source and driver patch
  build       Build the pinned workspace in the Conda environment
  clean-build Remove only this checkout's ROS overlay build/install/log prefixes, then rebuild
  station-live Launch the integrated live station (one follower owner and one camera)
  rviz        Launch the explicitly mocked MoveIt/RViz stack
  servo       Launch MoveIt Servo with mock ros2_control
  plan-smoke  Run IK and planning only; never execute trajectories
  inspect     Inspect controllers, topics, and actions from a running mock
  packages    Check MoveIt Servo, RealSense, and TF package availability
  run         Run a command inside a clean Conda ROS environment
  shell       Open an interactive shell in the Conda ROS environment
  real        Refuse hardware startup; the station owner retains that gate

Set SO101_CONDA_ROOT, SO101_CONDA_EXE, or SO101_CONDA_PREFIX to override the
Miniforge defaults. No ROS package is installed into Conda base.
EOF
}

if [[ "$ACTION" == "-h" || "$ACTION" == "--help" || "$ACTION" == "help" ]]; then
  usage
  exit 0
fi

[[ -x "$CONDA_EXE" ]] || fail "Conda executable is not available: $CONDA_EXE"

clean_conda() {
  local -a rmw_env=()
  if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
    rmw_env=("RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION")
  elif [[ -f "$ENV_PREFIX/lib/librmw_cyclonedds_cpp.so" ]]; then
    rmw_env=("RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  fi
  env \
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
  clean_conda run --no-capture-output --prefix "$ENV_PREFIX" "$@"
}

ros_run() {
  if [[ -r "$INSTALL_BASE/local_setup.bash" ]]; then
    conda_run bash --noprofile --norc -c 'source "$1"; shift; exec "$@"' \
      so101-overlay "$INSTALL_BASE/local_setup.bash" "$@"
  else
    conda_run "$@"
  fi
}

case "$ACTION" in
  create)
    [[ ! -e "$ENV_PREFIX/conda-meta/history" ]] || fail "environment already exists: $ENV_PREFIX"
    mkdir -p "$(dirname -- "$ENV_PREFIX")"
    clean_conda env create --yes --solver libmamba --prefix "$ENV_PREFIX" --file "$ENVIRONMENT"
    ;;
  fetch)
    "$ROS2_DIR/scripts/so101.sh" fetch
    ;;
  build)
    "$ROS2_DIR/scripts/so101.sh" fetch
    mkdir -p "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
    conda_run colcon --log-base "$LOG_BASE" build \
      --parallel-workers 2 \
      --base-paths "$ROS2_DIR/.cache/upstream/so101-ros-physical-ai" "$ROS2_DIR/src" \
      --build-base "$BUILD_BASE" \
      --install-base "$INSTALL_BASE" \
      --symlink-install
    ;;
  clean-build)
    rm -rf -- "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
    exec "$0" build
    ;;
  station-live)
    ros_run ros2 launch so101_station_bringup station_live.launch.py "$@"
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
    if [[ -r "$INSTALL_BASE/local_setup.bash" ]]; then
      conda_run bash --noprofile --norc -c 'source "$1"; exec bash --noprofile --norc -i' \
        so101-overlay "$INSTALL_BASE/local_setup.bash"
    else
      conda_run bash --noprofile --norc -i
    fi
    ;;
  real)
    fail "real hardware remains gated; Conda adds no device-access or actuation authorization"
    ;;
  *) usage >&2; fail "unknown action '$ACTION'" ;;
esac
