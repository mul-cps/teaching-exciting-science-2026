#!/usr/bin/env bash
# Build software-frame oracle and fixed CPU compatibility only. Never run them.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
ENV_PREFIX="${SO101_CONDA_PREFIX:-$HOME/miniforge3/envs/so101-jazzy}"
OUTPUT="$ROOT/ros2/.cache/rocm-projection"
mkdir -p "$OUTPUT"
[[ -f "$ENV_PREFIX/lib/librealsense2.so.2.58.3" && -f "$ENV_PREFIX/include/nlohmann/json.hpp" ]]
nice -n 10 "$ENV_PREFIX/bin/g++" -std=c++17 -O2 -I"$ENV_PREFIX/include" \
 "$ROOT/tools/realsense_cpu_projection_oracle.cpp" -L"$ENV_PREFIX/lib" \
 -Wl,-rpath,"$ENV_PREFIX/lib" -lrealsense2 -lcrypto -ldl -o "$OUTPUT/cpu-oracle"
nice -n 10 "$ENV_PREFIX/bin/g++" -std=c++17 -O2 -shared -fPIC \
 "$ROOT/workshop/perception/projection_compat.cpp" -o "$OUTPUT/compat.so"
sha256sum "$OUTPUT/cpu-oracle" "$OUTPUT/compat.so" "$ENV_PREFIX/lib/librealsense2.so.2.58.3"
