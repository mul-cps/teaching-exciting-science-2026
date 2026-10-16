#!/usr/bin/env bash
# Build a pinned optional overlay. Never install into the Conda environment.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
UPSTREAM_SHA=bd637294dd02b1c128422395c059fa06df8387a3
TASK_CACHE="$ROOT/ros2/.cache/rclpy-publisher-gid-7.1.12"
SOURCE="$TASK_CACHE/source"
PREFIX="$TASK_CACHE/install"
CONDA_ROOT="${SO101_CONDA_ROOT:-$HOME/miniforge3}"
CONDA_EXE="${SO101_CONDA_EXE:-$CONDA_ROOT/condabin/conda}"
ENV_PREFIX="${SO101_CONDA_PREFIX:-$CONDA_ROOT/envs/so101-jazzy}"
PATCH="$ROOT/ros2/patches/rclpy-7.1.12-publisher-gid.patch"
JOBS="${SO101_RCLPY_BUILD_JOBS:-2}"
fail() { printf 'build-rclpy-publisher-gid: %s\n' "$*" >&2; exit 2; }
[[ "$JOBS" == 1 || "$JOBS" == 2 ]] || fail 'build jobs must be 1 or 2'
[[ -x "$CONDA_EXE" && -x "$ENV_PREFIX/bin/python" ]] || fail 'dedicated Conda environment unavailable'
run() {
  env -u PYTHONPATH -u PYTHONHOME -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH \
    PYTHONNOUSERSITE=1 "$CONDA_EXE" run --no-capture-output -p "$ENV_PREFIX" "$@"
}
[[ "$(sha256sum "$PATCH" | cut -d ' ' -f 1)" == 30a94800573e524d03eacafa7b208a3d07574902826303166f459838cb69c4b9 ]] || fail 'patch digest mismatch'
mkdir -p "$TASK_CACHE"
run python -c 'import sys,importlib.metadata; sys.exit(0 if sys.version_info[:2]==(3,12) and importlib.metadata.version("rclpy")=="7.1.12" else "requires dedicated Python3.12/rclpy7.1.12")'
[[ -x "$ENV_PREFIX/bin/cmake" && -x "$ENV_PREFIX/bin/ninja" ]] || fail 'cmake/ninja missing; install dependencies separately'
if [[ ! -d "$SOURCE/.git" ]]; then
  [[ ! -e "$SOURCE" ]] || fail 'source cache exists without git metadata'
  git init -q "$SOURCE"
  git -C "$SOURCE" remote add origin https://github.com/ros2/rclpy.git
  git -C "$SOURCE" fetch --depth 1 origin "$UPSTREAM_SHA"
  git -C "$SOURCE" -c advice.detachedHead=false checkout --detach FETCH_HEAD
fi
[[ "$(git -C "$SOURCE" rev-parse HEAD)" == "$UPSTREAM_SHA" ]] || fail 'unexpected source revision'
if git -C "$SOURCE" diff --quiet; then
  git -C "$SOURCE" apply --check "$PATCH"
  git -C "$SOURCE" apply "$PATCH"
else
  git -C "$SOURCE" diff --no-ext-diff | cmp -s - "$PATCH" || fail 'unexpected dirty source cache; refusing overwrite'
fi
run python "$ROOT/ros2/conda/record-rclpy-gid-overlay.py" --env-prefix "$ENV_PREFIX" \
  --snapshot-base "$TASK_CACHE/base-before.json"
[[ "$(git -C "$SOURCE" status --porcelain --untracked-files=normal)" == ' M rclpy/src/rclpy/subscription.cpp' ]] || fail 'unexpected source tree changes'
# Source and patch bytes are preserved in provenance; rebuilding never resets it.
run cmake -S "$SOURCE/rclpy" -B "$TASK_CACHE/build" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" -DPython3_EXECUTABLE="$ENV_PREFIX/bin/python" \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
run cmake --build "$TASK_CACHE/build" --parallel "$JOBS"
run cmake --install "$TASK_CACHE/build"
run python "$ROOT/ros2/conda/record-rclpy-gid-overlay.py" \
  --prefix "$PREFIX" --env-prefix "$ENV_PREFIX" --source "$SOURCE" --patch "$PATCH" --upstream-sha "$UPSTREAM_SHA"
printf 'Isolated rclpy GID overlay: %s\n' "$PREFIX"
