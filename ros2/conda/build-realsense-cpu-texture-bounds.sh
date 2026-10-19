#!/usr/bin/env bash
# CPU-only repair overlay; no activation, global installation, or device access.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
ENV_PREFIX="${SO101_CONDA_PREFIX:-$HOME/miniforge3/envs/so101-jazzy}"
WRAPPER_SHA=9a11121700cb4780e273e34141f6402fe184321d
WRAPPER_ARCHIVE_SHA=99e6eec240aa043b7e9a5b77f2188cf71849b2d43dd46263063b0caace1a340e
CACHE="$ROOT/ros2/.cache/realsense-glsl"
JOBS="${SO101_REALSENSE_BUILD_JOBS:-2}"
[[ "$JOBS" == 1 || "$JOBS" == 2 ]] || { echo 'Build jobs must be 1 or 2' >&2; exit 2; }
[[ -x "$ENV_PREFIX/bin/cmake" && -x "$ENV_PREFIX/bin/g++" ]] || { echo 'Dedicated build dependencies unavailable' >&2; exit 2; }
export PATH="$ENV_PREFIX/bin:$PATH" PYTHONNOUSERSITE=1 ROS_DISTRO=jazzy ROS_VERSION=2
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH
export PKG_CONFIG_PATH="$ENV_PREFIX/lib/pkgconfig"
mkdir -p "$CACHE"
[[ -f "$CACHE/wrapper.tar.gz" ]] || curl -fLsS "https://codeload.github.com/realsenseai/realsense-ros/tar.gz/$WRAPPER_SHA" -o "$CACHE/wrapper.tar.gz"
echo "$WRAPPER_ARCHIVE_SHA  $CACHE/wrapper.tar.gz" | sha256sum -c -
CPU_CACHE="$ROOT/ros2/.cache/realsense-cpu-texture-bounds"
CPU_SOURCE="$CPU_CACHE/source/realsense-ros-$WRAPPER_SHA"
CPU_PREFIX="$CPU_CACHE/install"
CPU_PATCH="$ROOT/ros2/patches/realsense-ros-4.58.4-pointcloud-texture-bounds.patch"
echo "008b46ff14fcb1c999a28935101608a18b8d41fafbd1bd4a866716bf262e3476  $CPU_PATCH" | sha256sum -c -
mkdir -p "$CPU_CACHE/source"
[[ -d "$CPU_SOURCE" ]] || tar xf "$CACHE/wrapper.tar.gz" -C "$CPU_CACHE/source"
python - "$CACHE/wrapper.tar.gz" "$CPU_CACHE/source" <<'PYCPUCHECK'
import hashlib, pathlib, sys, tarfile
with tarfile.open(sys.argv[1]) as archive:
  expected_files = {m.name for m in archive.getmembers() if m.isfile() or m.issym() or m.islnk()}
  source_root = pathlib.Path(sys.argv[2])
  actual_files = {str(p.relative_to(source_root)) for p in source_root.rglob('*') if p.is_file() or p.is_symlink()}
  assert actual_files == expected_files, f'Source inventory differs: extra={sorted(actual_files-expected_files)}, missing={sorted(expected_files-actual_files)}'
  for member in archive.getmembers():
      if not member.isfile(): continue
      actual = hashlib.sha256((pathlib.Path(sys.argv[2])/member.name).read_bytes()).hexdigest()
      expected = hashlib.sha256(archive.extractfile(member).read()).hexdigest()
      allowed = member.name.endswith('/realsense2_camera/src/pointcloud_filter.cpp') and actual == 'd7e470309026311bbd27775388cad83c99ca16d3637756fccef7d4fb624676a8'
      assert actual == expected or allowed, f'Unexpected CPU source: {member.name}'
PYCPUCHECK
if patch --dry-run --silent -p1 -d "$CPU_SOURCE" < "$CPU_PATCH"; then
  patch --silent -p1 -d "$CPU_SOURCE" < "$CPU_PATCH"
else
  patch --dry-run --silent -R -p1 -d "$CPU_SOURCE" < "$CPU_PATCH"
fi
export CMAKE_PREFIX_PATH="$CPU_PREFIX:$ENV_PREFIX"
export AMENT_PREFIX_PATH="$CPU_PREFIX:$ENV_PREFIX"
# The CPU repair changes no interfaces; reuse the exact installed 4.58.4 msgs.
python - "$ENV_PREFIX" <<'PYCPUMSG'
import pathlib, sys, xml.etree.ElementTree as ET
p=pathlib.Path(sys.argv[1])/'share/realsense2_camera_msgs/package.xml'
assert ET.parse(p).getroot().findtext('version')=='4.58.4', 'requires installed msgs4.58.4'
version=(pathlib.Path(sys.argv[1])/'lib/cmake/realsense2/realsense2ConfigVersion.cmake').read_text()
assert 'set(PACKAGE_VERSION "2.58.3")' in version, 'requires installed CPU SDK2.58.3'
PYCPUMSG
for package in realsense2_camera; do
  nice -n 10 cmake -S "$CPU_SOURCE/$package" -B "$CPU_CACHE/build-$package" -G Ninja \
    -DCMAKE_INSTALL_PREFIX="$CPU_PREFIX" -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="$ENV_PREFIX/bin/python" -DPYTHON_EXECUTABLE="$ENV_PREFIX/bin/python" \
    -Drealsense2_DIR="$ENV_PREFIX/lib/cmake/realsense2" \
    -DCMAKE_INSTALL_RPATH="$CPU_PREFIX/lib;$ENV_PREFIX/lib" \
    -DBUILD_TESTING=OFF -DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF
  nice -n 10 cmake --build "$CPU_CACHE/build-$package" --parallel "$JOBS"
  cmake --install "$CPU_CACHE/build-$package"
done
python - "$CPU_CACHE" "$ENV_PREFIX" "$CPU_PATCH" "$WRAPPER_SHA" "$ROOT/ros2/conda/build-realsense-cpu-texture-bounds.sh" "$CPU_SOURCE" <<'PYCPUINFO'
import hashlib, json, pathlib, subprocess, sys
cache, env, patch = map(pathlib.Path, sys.argv[1:4])
paths = [patch, pathlib.Path(sys.argv[5]), *cache.glob('install/lib/*.so*'), cache/'build-realsense2_camera/CMakeCache.txt', env/'lib/librealsense2.so.2.58.3']
library = cache/'install/lib/librealsense2_camera.so'
source = pathlib.Path(sys.argv[6])
source_files = {str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source.rglob('*')) if p.is_file()}
source_digest = hashlib.sha256(json.dumps(source_files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
record = {'qualified':False, 'mode':'CPU wrapper texture bounds only', 'wrapper_commit':sys.argv[4],
 'builder_sha256':hashlib.sha256(pathlib.Path(sys.argv[5]).read_bytes()).hexdigest(),
 'source_final_sha256':source_digest, 'source_file_sha256':source_files,
 'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()},
 'linked_libraries':subprocess.check_output(['ldd',str(library)],text=True)}
(cache/'build-provenance.json').write_text(json.dumps(record,indent=2)+'\n')
PYCPUINFO
printf 'CPU-only candidate built, not activated: %s\n' "$CPU_PREFIX"
exit 0
