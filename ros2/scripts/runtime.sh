#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
CONTAINERFILE="$ROOT_DIR/ros2/container/Containerfile"
IMAGE="${ROS2_IMAGE:-localhost/so101-ros2:jazzy}"
ACTION="${1:-help}"
[[ $# -eq 0 ]] || shift
GPU=0
HARDWARE=0
DEVICE=""
STATION_LOCK=""
COMMAND=()

usage() {
  cat <<'EOF'
Usage: ros2/scripts/runtime.sh plan|shell|run|build [options]

Actions:
  plan       Print the rootless Podman arguments without starting a container
  shell      Open an interactive ROS 2 Jazzy container shell
  run        Run a command in the container (append it after --)
  build      Build the pinned ROS 2 + MoveIt 2 development image

Options:
  --gpu      Pass /dev/dri and preserve host supplementary groups
  --hardware --device PATH --station-lock PATH
             Explicitly pass one serial device while holding the canonical
             station lock for the entire container process lifetime
  -h, --help Show this help

Hardware is disabled by default. The caller must arrange the station lock used
by the resident controller before opting into hardware access.
EOF
}

if [[ "$ACTION" == "-h" || "$ACTION" == "--help" ]]; then
  usage
  exit 0
fi

fail() { printf 'runtime.sh: %s\n' "$*" >&2; exit 2; }

while (($#)); do
  case "$1" in
    --) shift; COMMAND=("$@"); break ;;
    --gpu) GPU=1 ;;
    --hardware) HARDWARE=1 ;;
    --device)
      (($# >= 2)) || fail "--device requires a path"
      DEVICE="$2"; shift ;;
    --station-lock)
      (($# >= 2)) || fail "--station-lock requires a path"
      STATION_LOCK="$2"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown option: $1 (see --help)" ;;
  esac
  shift
done

if ((HARDWARE)) && { [[ -z "$DEVICE" || -z "$STATION_LOCK" ]]; }; then
  fail "--hardware requires --device and --station-lock"
fi
if ((!HARDWARE)) && [[ -n "$DEVICE" || -n "$STATION_LOCK" ]]; then
  fail "--device and --station-lock require --hardware"
fi
if ((HARDWARE)); then
  STATE_HOME="${XDG_STATE_HOME:-$HOME/.local/state}"
  [[ "$STATE_HOME" == /* ]] || fail "XDG_STATE_HOME must be an absolute path"
  CANONICAL_STATION_LOCK="$STATE_HOME/exciting-science/station.lock"
  [[ "$STATION_LOCK" == "$CANONICAL_STATION_LOCK" ]] || \
    fail "hardware mode must use canonical station lock: $CANONICAL_STATION_LOCK"
fi
case "$ACTION" in
  plan|shell|run|build) ;;
  help) usage; exit 0 ;;
  *) fail "unknown action: $ACTION (see --help)" ;;
esac
if [[ "$ACTION" == "run" && ${#COMMAND[@]} -eq 0 ]]; then
  fail "run requires a command after --"
fi
COMMAND_MODE=0
if [[ "$ACTION" == "run" || ( "$ACTION" == "plan" && ${#COMMAND[@]} -gt 0 ) ]]; then
  COMMAND_MODE=1
fi

mkdir -p "$HOME/.cache/so101-ros2/build" "$HOME/.cache/so101-ros2/install" "$HOME/.cache/so101-ros2/log" "$HOME/.cache/so101-ros2/home"

podman_args=(run --rm --network host --ipc host --userns keep-id --user "$(id -u):$(id -g)" --security-opt label=disable)
if ((COMMAND_MODE == 0)); then
  podman_args+=(--interactive --tty)
fi
podman_args+=(--env HOME=/ws/.cache/so101-ros2/home --env ROS_DISTRO=jazzy --env DISPLAY="${DISPLAY:-:0}")
podman_args+=(--volume "$ROOT_DIR:/ws:Z")
podman_args+=(--volume "$HOME/.cache/so101-ros2/build:/ws/build:Z")
podman_args+=(--volume "$HOME/.cache/so101-ros2/install:/ws/install:Z")
podman_args+=(--volume "$HOME/.cache/so101-ros2/log:/ws/log:Z")
podman_args+=(--volume "$HOME/.cache/so101-ros2/home:/ws/.cache/so101-ros2/home:Z")
podman_args+=(--volume /tmp/.X11-unix:/tmp/.X11-unix:ro)
if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
  podman_args+=(--env XAUTHORITY=/tmp/ros.xauth --volume "$XAUTHORITY:/tmp/ros.xauth:ro")
fi
if ((GPU)); then
  [[ -d /dev/dri ]] || fail "--gpu requested but /dev/dri is unavailable"
  podman_args+=(--device /dev/dri --group-add keep-groups)
fi
if ((HARDWARE)); then
  [[ -e "$DEVICE" ]] || fail "device does not exist: $DEVICE"
  [[ -c "$DEVICE" ]] || fail "device is not a character device: $DEVICE"
  podman_args+=(--device "$DEVICE:$DEVICE")
fi
podman_args+=(--workdir /ws "$IMAGE")
if ((COMMAND_MODE)); then
  podman_args+=("${COMMAND[@]}")
else
  podman_args+=(/bin/bash)
fi

case "$ACTION" in
  plan)
    printf '%q ' podman "${podman_args[@]}"
    printf '\n'
    ;;
  build)
    podman build --tag "$IMAGE" --file "$CONTAINERFILE" "$ROOT_DIR"
    ;;
  shell)
    if ((HARDWARE)); then
      mkdir -p "$(dirname -- "$CANONICAL_STATION_LOCK")"
      exec flock -n "$CANONICAL_STATION_LOCK" podman "${podman_args[@]}"
    fi
    exec podman "${podman_args[@]}"
    ;;
  run)
    if ((HARDWARE)); then
      mkdir -p "$(dirname -- "$CANONICAL_STATION_LOCK")"
      exec flock -n "$CANONICAL_STATION_LOCK" podman "${podman_args[@]}"
    fi
    exec podman "${podman_args[@]}"
    ;;
esac
