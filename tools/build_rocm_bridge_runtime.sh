#!/usr/bin/env bash
# Build a NEW bridge runtime; never overwrite the qualified core compatibility.
set -euo pipefail
PROJECTION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
PROJECTION_ENV="${SO101_CONDA_PREFIX:-$HOME/miniforge3/envs/so101-jazzy}"
PROJECTION_OUTPUT="$PROJECTION_ROOT/ros2/.cache/rocm-projection/bridge-runtime.so"
mkdir -p "$(dirname "$PROJECTION_OUTPUT")"
nice -n 10 "$PROJECTION_ENV/bin/g++" -std=c++17 -O2 -shared -fPIC \
 "$PROJECTION_ROOT/workshop/perception/projection_compat.cpp" \
 "$PROJECTION_ROOT/workshop/perception/rocm_wire_pack.cpp" -o "$PROJECTION_OUTPUT"
python - "$PROJECTION_ROOT" "$PROJECTION_OUTPUT" <<'PY'
from pathlib import Path
import hashlib,json,sys
root,output=map(Path,sys.argv[1:])
paths=['workshop/perception/projection_compat.cpp','workshop/perception/rocm_wire_pack.cpp','tools/build_rocm_bridge_runtime.sh']
manifest=dict(schema='so101-rocm-bridge-runtime-build-v1',core_source_modified=False,
 source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in paths},
 output=str(output),output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
 flags=['-std=c++17','-O2','-shared','-fPIC'])
output.with_suffix('.build.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))
PY
