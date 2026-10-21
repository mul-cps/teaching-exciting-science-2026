#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
SOURCE_DIR="$ROOT_DIR/ros2/.cache/upstream/pick_ik"
UPSTREAM_URL="https://github.com/PickNikRobotics/pick_ik.git"
UPSTREAM_REV="8c999995dec2a71cb950850bd5ee56beed8d4563"
SOURCE_FILE="src/pick_ik_plugin.cpp"
fail() { printf 'fetch-pick-ik.sh: %s\n' "$*" >&2; exit 2; }

if [[ ! -e "$SOURCE_DIR" ]]; then
  mkdir -p "$(dirname -- "$SOURCE_DIR")"
  git clone --no-checkout "$UPSTREAM_URL" "$SOURCE_DIR"
  git -C "$SOURCE_DIR" fetch --depth=1 origin "$UPSTREAM_REV"
  git -C "$SOURCE_DIR" checkout --detach "$UPSTREAM_REV"
fi
[[ -d "$SOURCE_DIR/.git" ]] || fail "source cache is not a Git checkout: $SOURCE_DIR"
[[ "$(git -C "$SOURCE_DIR" rev-parse HEAD)" == "$UPSTREAM_REV" ]] || fail "source cache HEAD is not pinned: $SOURCE_DIR"
git -C "$SOURCE_DIR" diff --cached --quiet || fail "source cache has staged changes"
[[ -z "$(git -C "$SOURCE_DIR" ls-files --others --exclude-standard)" ]] || fail "source cache has untracked files"
[[ -z "$(git -C "$SOURCE_DIR" diff --summary)" ]] || fail "source cache has file mode or type changes"
changed_files="$(git -C "$SOURCE_DIR" diff --name-only)"
[[ -z "$changed_files" || "$changed_files" == "$SOURCE_FILE" ]] || fail "source cache has unexpected local changes"

# PickIK 1.1.2's approximate validator mistakenly uses the strict frame tests.
# Keep both validators and thresholds intact; select its existing approximate tests.
# https://github.com/PickNikRobotics/pick_ik/blob/8c999995dec2a71cb950850bd5ee56beed8d4563/src/pick_ik_plugin.cpp
python3 - "$SOURCE_DIR/$SOURCE_FILE" <<'PY'
import hashlib
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
original_hash = "bb6285aa9578b94bb86e166c481782434597b3d720f4abe5520e40561cebd9d1"
patched_hash = "b9b405a4d7bdf9bffe0559c490ceb49ef7ec213cc9bd7eabac8435b74f41ff99"
source = path.read_bytes()
digest = hashlib.sha256(source).hexdigest()
if digest == patched_hash:
    print(f"PickIK pinned patch already present: {path}")
    sys.exit(0)
if digest != original_hash:
    sys.exit(f"Refusing unexpected PickIK source SHA256: {digest}")
pattern = rb"(\bapprox_solution_fn\s*=\s*make_is_solution_test_fn\([^;]*?\b)frame_tests(\b[^;]*;)"
patched, count = re.subn(pattern, rb"\g<1>approx_frame_tests\g<2>", source)
if count != 1 or hashlib.sha256(patched).hexdigest() != patched_hash:
    sys.exit("Refusing PickIK patch: expected exactly one replacement and pinned output hash")
path.write_bytes(patched)
print(f"Applied pinned PickIK approximate-frame validator fix: {path}")
PY
