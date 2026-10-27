#!/bin/bash
set -e
base=/home/bjoern/.local/share/exciting-science/validation/rviz-coexistence-20261027
logs=/home/bjoern/.local/share/exciting-science/validation/quality-profiles-rviz-20261027
mkdir -p "$logs"
cd "$base/runtime"
export XDG_RUNTIME_DIR=/run/user/1002
unset DISPLAY WAYLAND_DISPLAY GALLIUM_DRIVER LIBGL_ALWAYS_SOFTWARE MESA_LOADER_DRIVER_OVERRIDE
export WESTON_MODULE_MAP="headless-backend.so=$PWD/usr/lib/x86_64-linux-gnu/libweston-13/headless-backend.so;gl-renderer.so=$PWD/usr/lib/x86_64-linux-gnu/libweston-13/gl-renderer.so;desktop-shell.so=$PWD/usr/lib/x86_64-linux-gnu/weston/desktop-shell.so;kiosk-shell.so=$PWD/usr/lib/x86_64-linux-gnu/weston/kiosk-shell.so;xwayland.so=$PWD/usr/lib/x86_64-linux-gnu/libweston-13/xwayland.so"
export LD_LIBRARY_PATH="$PWD/usr/lib/x86_64-linux-gnu:$PWD/usr/lib/x86_64-linux-gnu/weston"
export WESTON_DATA_DIR="$PWD/usr/share/weston"
export WESTON_LIBEXECDIR="$PWD/usr/libexec"
exec usr/bin/weston --backend=headless --shell=desktop-shell.so --renderer=gl --width=1920 --height=1080 --socket=workshop-rviz --xwayland --idle-time=0 --config="$base/weston.ini" --log="$logs/weston.log"
