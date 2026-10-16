#!/usr/bin/env bash
# Native GUID fix in a separate prefix; installed Conda RMW/DDS remain unchanged.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
UPSTREAM_SHA=2659b2bf9b453bd40cf4e255da559256f388427e
TASK_CACHE="$ROOT/ros2/.cache/rmw-cyclonedds-gid-2.2.4"
SOURCE="$TASK_CACHE/source"
PREFIX="$TASK_CACHE/install"
CONDA_ROOT="${SO101_CONDA_ROOT:-$HOME/miniforge3}"
CONDA_EXE="${SO101_CONDA_EXE:-$CONDA_ROOT/condabin/conda}"
ENV_PREFIX="${SO101_CONDA_PREFIX:-$CONDA_ROOT/envs/so101-jazzy}"
PATCH="$ROOT/ros2/patches/rmw-cyclonedds-2.2.4-publisher-gid.patch"
JOBS="${SO101_RCLPY_BUILD_JOBS:-2}"
fail() { printf 'build-rmw-cyclonedds-publisher-gid: %s\n' "$*" >&2; exit 2; }
[[ "$JOBS" == 1 || "$JOBS" == 2 ]] || fail 'build jobs must be 1 or 2'
[[ -x "$CONDA_EXE" && -x "$ENV_PREFIX/bin/python" ]] || fail 'dedicated Conda environment unavailable'
[[ "$(sha256sum "$PATCH" | cut -d ' ' -f 1)" == cac5cba8f1c43d4542eee2878e38f045ede91b387b15e9208cde95d1e4b5172b ]] || fail 'patch digest mismatch'
run() {
  env -u PYTHONPATH -u PYTHONHOME -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u LD_LIBRARY_PATH \
    PYTHONNOUSERSITE=1 "$CONDA_EXE" run --no-capture-output -p "$ENV_PREFIX" "$@"
}
mkdir -p "$TASK_CACHE"
run python -c 'import sys,xml.etree.ElementTree as E; from pathlib import Path; sys.exit(0 if E.parse(Path(sys.prefix)/"share/rmw_cyclonedds_cpp/package.xml").findtext("version")=="2.2.4" else "requires rmw_cyclonedds2.2.4")'
[[ -x "$ENV_PREFIX/bin/cmake" && -x "$ENV_PREFIX/bin/ninja" ]] || fail 'cmake/ninja missing'
if [[ ! -d "$SOURCE/.git" ]]; then
  [[ ! -e "$SOURCE" ]] || fail 'source cache exists without git metadata'
  git init -q "$SOURCE"
  git -C "$SOURCE" remote add origin https://github.com/ros2/rmw_cyclonedds.git
  git -C "$SOURCE" fetch --depth 1 origin "$UPSTREAM_SHA"
  git -C "$SOURCE" -c advice.detachedHead=false checkout --detach FETCH_HEAD
fi
[[ "$(git -C "$SOURCE" rev-parse HEAD)" == "$UPSTREAM_SHA" ]] || fail 'unexpected source revision'
if git -C "$SOURCE" diff --quiet; then
  git -C "$SOURCE" apply --check "$PATCH"
  git -C "$SOURCE" apply "$PATCH"
else
  git -C "$SOURCE" diff --no-ext-diff | cmp -s - "$PATCH" || fail 'unexpected dirty source; refusing overwrite'
fi
[[ "$(git -C "$SOURCE" ls-files --others --exclude-standard)" == '' ]] || fail 'unexpected source changes'
run python "$ROOT/ros2/conda/record-rmw-gid-overlay.py" --env-prefix "$ENV_PREFIX" --snapshot-base "$TASK_CACHE/base-before.json"
run cmake -S "$SOURCE/rmw_cyclonedds_cpp" -B "$TASK_CACHE/build" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" -DPython3_EXECUTABLE="$ENV_PREFIX/bin/python" \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
run cmake --build "$TASK_CACHE/build" --parallel "$JOBS"
run cmake --install "$TASK_CACHE/build"
run python "$ROOT/ros2/conda/record-rmw-gid-overlay.py" --prefix "$PREFIX" --env-prefix "$ENV_PREFIX" --source "$SOURCE" --patch "$PATCH"
printf 'Isolated Cyclone RMW GID overlay: %s\n' "$PREFIX"
