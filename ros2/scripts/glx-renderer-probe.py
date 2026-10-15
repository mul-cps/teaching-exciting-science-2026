#!/usr/bin/env python3
"""Create a temporary 1x1 GLX context and print its actual renderer string."""

from pathlib import Path
import os
import subprocess
import tempfile


SOURCE = r'''#include <stdio.h>
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


def main():
    with tempfile.TemporaryDirectory(prefix="so101-glx-probe-") as directory:
        source = Path(directory) / "probe.c"
        binary = Path(directory) / "probe"
        source.write_text(SOURCE)
        subprocess.run(["cc", str(source), "-o", str(binary), "-lGL", "-lX11"], check=True)
        print(f"DISPLAY={os.environ.get('DISPLAY', '')}")
        print(f"LIBGL_ALWAYS_SOFTWARE={os.environ.get('LIBGL_ALWAYS_SOFTWARE', 'unset')}")
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
