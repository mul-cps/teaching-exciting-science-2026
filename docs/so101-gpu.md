# GPU-backed ROS visualization

The RViz and Servo mock launch paths select the rootless Podman GPU mode
whenever the host exposes `/dev/dri`:

```bash
ros2/scripts/so101.sh rviz
ros2/scripts/so101.sh servo
```

The runtime forwards `/dev/dri` and the caller's supplementary groups with
Podman's `keep-groups`. If `/dev/dri` is absent, those commands still launch
without the GPU option. Non-GUI commands such as `inspect` and `plan-smoke` do
not request GPU devices. This changes only visualization/runtime access; the
default control launch still uses mock hardware and sends no serial commands.

Inspect the exact Podman command without starting a container:

```bash
ros2/scripts/runtime.sh plan --gpu
```

For a renderer check on the active X11 display, run `glxinfo -B` on the host
and the bundled 1x1 GLX context probe in the GPU container. The renderer string
should identify the AMD/Mesa hardware driver and must not contain `llvmpipe`,
`softpipe`, or `Software Rasterizer`:

```bash
DISPLAY=:1 XAUTHORITY=/run/user/1000/xauth_hyiKAE glxinfo -B
DISPLAY=:1 XAUTHORITY=/run/user/1000/xauth_hyiKAE \
  ros2/scripts/runtime.sh run --gpu -- /ws/ros2/scripts/glx-renderer-probe.py
```

The probe creates a temporary hidden 1x1 GLX window, reports the actual
`GL_VENDOR`, `GL_RENDERER`, `GL_VERSION`, and software-rendering override, then
removes its temporary source and binary. It does not open RViz or a device.

`runtime.sh` does not set `LIBGL_ALWAYS_SOFTWARE`; software rendering should
only be used as an explicit troubleshooting override. Keep the existing GUI
session alive while checking a second runtime, and close only sessions you
started yourself.
