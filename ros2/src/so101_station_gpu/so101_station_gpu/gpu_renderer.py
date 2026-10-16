"""GLX renderer probing and fail-closed RViz GPU startup helpers."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile


GLX_PROBE_SOURCE = r'''#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>
int main(void) {
  Display *dpy = XOpenDisplay(NULL);
  if (!dpy) { fprintf(stderr, "XOpenDisplay failed\n"); return 2; }
  int screen = DefaultScreen(dpy);
  int attrs[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, None };
  XVisualInfo *visual = glXChooseVisual(dpy, screen, attrs);
  if (!visual) { fprintf(stderr, "glXChooseVisual failed\n"); return 3; }
  Colormap cmap = XCreateColormap(dpy, RootWindow(dpy, visual->screen), visual->visual, AllocNone);
  XSetWindowAttributes swa = { .colormap = cmap, .event_mask = StructureNotifyMask };
  Window win = XCreateWindow(dpy, RootWindow(dpy, visual->screen), 0, 0, 1, 1, 0,
                             visual->depth, InputOutput, visual->visual,
                             CWColormap | CWEventMask, &swa);
  GLXContext ctx = glXCreateContext(dpy, visual, NULL, True);
  if (!ctx || !glXMakeCurrent(dpy, win, ctx)) { fprintf(stderr, "GLX context failed\n"); return 4; }
  printf("GL_VENDOR=%s\nGL_RENDERER=%s\nGL_VERSION=%s\n",
         glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_VERSION));
  glXMakeCurrent(dpy, None, NULL); glXDestroyContext(dpy, ctx); XDestroyWindow(dpy, win);
  XCloseDisplay(dpy); return 0;
}
'''

SOFTWARE_RENDERER_MARKERS = (
    "llvmpipe", "softpipe", "swrast", "kms_swrast", "lavapipe", "swr",
    "software rasterizer", "software renderer",
)
DEFAULT_DRM_LIBRARIES = ("/usr/lib64/libdrm.so.2", "/usr/lib64/libdrm_amdgpu.so.1")


def rviz_preload_value(environ: dict[str, str] | None = None,
                       exists=os.path.isfile) -> str:
    """Choose only RViz's preload; empty string explicitly clears inherited values."""
    selected_env = os.environ if environ is None else environ
    override = selected_env.get("SO101_RVIZ_LD_PRELOAD")
    if override is not None:
        libraries = override.split(":")
        if any(not item or not exists(item) for item in libraries):
            raise ValueError("SO101_RVIZ_LD_PRELOAD must be a colon-separated list of existing library paths")
        return override
    if all(exists(path) for path in DEFAULT_DRM_LIBRARIES):
        return ":".join(DEFAULT_DRM_LIBRARIES)
    return ""


def probe_renderer(env: dict[str, str] | None = None) -> dict[str, str]:
    """Compile and run the GLX probe with the exact environment RViz will use."""
    selected_env = os.environ.copy() if env is None else dict(env)
    with tempfile.TemporaryDirectory(prefix="so101-glx-probe-") as directory:
        source = Path(directory) / "probe.c"
        binary = Path(directory) / "probe"
        source.write_text(GLX_PROBE_SOURCE, encoding="utf-8")
        compile_result = subprocess.run(
            ["cc", str(source), "-o", str(binary), "-lGL", "-lX11"],
            check=False, env=selected_env, capture_output=True, text=True, timeout=30,
        )
        if compile_result.returncode:
            detail = compile_result.stderr[-2000:].strip()
            raise RuntimeError(
                "could not compile the GLX preflight; install a C compiler and the "
                "OpenGL/X11 development headers (for example gcc, libgl-dev, libx11-dev)"
                + (f": {detail}" if detail else "")
            )
        completed = subprocess.run([str(binary)], check=False, env=selected_env,
                                   capture_output=True, text=True, timeout=5)
        if completed.returncode:
            detail = completed.stderr[-2000:].strip()
            raise RuntimeError("GLX preflight execution failed" + (f": {detail}" if detail else ""))
    result = {}
    for line in completed.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            result[key] = value.strip()
    unknown = {"", "(null)", "null", "unknown", "n/a"}
    if result.get("GL_RENDERER", "").strip().lower() in unknown or \
            result.get("GL_VENDOR", "").strip().lower() in unknown:
        raise RuntimeError("GLX probe did not report a renderer and vendor")
    return result


def require_hardware_renderer(info: dict[str, str]) -> None:
    renderer = info.get("GL_RENDERER", "").strip()
    if renderer.strip().lower() in {"", "(null)", "null", "unknown", "n/a"}:
        raise RuntimeError("GLX renderer probe returned an empty GL_RENDERER")
    if any(marker in renderer.lower() for marker in SOFTWARE_RENDERER_MARKERS):
        raise RuntimeError(
            f"RViz requires GPU rendering; GLX selected software renderer {renderer!r}"
        )


def main(argv: list[str] | None = None) -> None:
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    try:
        # Manual invocation and launch integration use identical local preload
        # selection; this affects only the probe and RViz child processes.
        os.environ["LD_PRELOAD"] = rviz_preload_value()
        # The station GPU is AMD gfx1151. Do not allow an inherited Mesa
        # software override or driver selection to make preflight/RViz diverge.
        os.environ["LIBGL_ALWAYS_SOFTWARE"] = "false"
        os.environ["GALLIUM_DRIVER"] = "radeonsi"
        info = probe_renderer()
        require_hardware_renderer(info)
        print("RViz GPU preflight: " + " ".join(
            f"{key}={info[key]}" for key in ("GL_VENDOR", "GL_RENDERER", "GL_VERSION")
            if info.get(key)
        ), flush=True)
        rviz = shutil.which("rviz2")
        if not rviz:
            raise RuntimeError("rviz2 executable was not found after ROS environment setup")
        os.execvpe(rviz, [rviz, *args], os.environ.copy())
    except (OSError, ValueError, subprocess.SubprocessError, RuntimeError) as exc:
        print(f"rviz_gpu_guard: refusing startup: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(78) from exc

