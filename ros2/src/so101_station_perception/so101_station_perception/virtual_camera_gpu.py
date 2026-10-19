"""Bounded GLES point rasterizer using native EGL; no software fallback."""
import ctypes as C
import ctypes.util
import numpy as np


class GpuPointCamera:
    """One pbuffer, shader and VBO reused for every synthetic acquisition."""

    def __init__(self, width=320, height=240, max_points=20000):
        if not 1 <= width <= 640 or not 1 <= height <= 480 or not 1 <= max_points <= 20000:
            raise ValueError("GPU virtual camera bounds exceeded")
        self.width, self.height, self.max_points = int(width), int(height), int(max_points)
        self.display = self.surface = self.context = None
        self.egl = C.CDLL(ctypes.util.find_library("EGL") or "libEGL.so.1")
        self.gl = C.CDLL(ctypes.util.find_library("GLESv2") or "libGLESv2.so.2")
        self._bind()
        try:
            self._context()
            self.renderer = self.gl.glGetString(0x1F01).decode()
            from so101_station_gpu.gpu_renderer import require_hardware_renderer
            require_hardware_renderer({"GL_RENDERER": self.renderer})
            self.program = self._program()
            self.position = self.gl.glGetAttribLocation(self.program, b"position")
            self.matrix = self.gl.glGetUniformLocation(self.program, b"clip_from_cloud")
            self.vbo = C.c_uint()
            self.gl.glGenBuffers(1, C.byref(self.vbo))
            self.gl.glBindBuffer(0x8892, self.vbo)
            self.gl.glBufferData(0x8892, self.max_points * 12, None, 0x88E0)
            self.pixels = np.empty((self.height, self.width, 4), dtype=np.uint8)
        except BaseException:
            self.close()
            raise

    def _fn(self, lib, name, result, args):
        function = getattr(lib, name)
        function.restype, function.argtypes = result, args

    def _bind(self):
        P, I, U, F = C.c_void_p, C.c_int, C.c_uint, C.c_float
        for name, result, args in (
            ("eglGetProcAddress", P, [C.c_char_p]),
            ("eglInitialize", U, [P, C.POINTER(I), C.POINTER(I)]),
            ("eglBindAPI", U, [U]),
            ("eglChooseConfig", U, [P, C.POINTER(I), C.POINTER(P), I, C.POINTER(I)]),
            ("eglCreatePbufferSurface", P, [P, P, C.POINTER(I)]),
            ("eglCreateContext", P, [P, P, P, C.POINTER(I)]),
            ("eglMakeCurrent", U, [P, P, P, P]),
            ("eglDestroyContext", U, [P, P]), ("eglDestroySurface", U, [P, P]),
            ("eglTerminate", U, [P]),
        ): self._fn(self.egl, name, result, args)
        for name, result, args in (
            ("glGetString", C.c_char_p, [U]), ("glCreateShader", U, [U]),
            ("glShaderSource", None, [U, I, C.POINTER(C.c_char_p), P]),
            ("glCompileShader", None, [U]), ("glGetShaderiv", None, [U, U, C.POINTER(I)]),
            ("glGetShaderInfoLog", None, [U, I, P, P]), ("glDeleteShader", None, [U]),
            ("glCreateProgram", U, []), ("glAttachShader", None, [U, U]),
            ("glLinkProgram", None, [U]), ("glGetProgramiv", None, [U, U, C.POINTER(I)]),
            ("glGetProgramInfoLog", None, [U, I, P, P]), ("glDeleteProgram", None, [U]),
            ("glUseProgram", None, [U]), ("glGetAttribLocation", I, [U, C.c_char_p]),
            ("glGetUniformLocation", I, [U, C.c_char_p]),
            ("glGenBuffers", None, [I, C.POINTER(U)]), ("glBindBuffer", None, [U, U]),
            ("glBufferData", None, [U, C.c_ssize_t, P, U]),
            ("glBufferSubData", None, [U, C.c_ssize_t, C.c_ssize_t, P]),
            ("glViewport", None, [I, I, I, I]), ("glClearColor", None, [F, F, F, F]),
            ("glClear", None, [U]), ("glEnable", None, [U]),
            ("glUniformMatrix4fv", None, [I, I, U, P]),
            ("glEnableVertexAttribArray", None, [U]),
            ("glVertexAttribPointer", None, [U, I, U, U, I, P]),
            ("glDrawArrays", None, [U, I, I]),
            ("glReadPixels", None, [I, I, I, I, U, U, P]), ("glGetError", U, []),
        ): self._fn(self.gl, name, result, args)

    def _context(self):
        pointer = self.egl.eglGetProcAddress(b"eglGetPlatformDisplayEXT")
        if not pointer:
            raise RuntimeError("surfaceless EGL platform unavailable")
        get_display = C.CFUNCTYPE(C.c_void_p, C.c_uint, C.c_void_p, C.POINTER(C.c_int))(pointer)
        self.display = get_display(0x31DD, None, None)
        major, minor = C.c_int(), C.c_int()
        if not self.egl.eglInitialize(self.display, C.byref(major), C.byref(minor)):
            raise RuntimeError("EGL initialization failed")
        if not self.egl.eglBindAPI(0x30A0):
            raise RuntimeError("EGL GLES binding failed")
        attrs = (C.c_int * 15)(0x3033, 1, 0x3040, 4, 0x3024, 8, 0x3023, 8,
                              0x3022, 8, 0x3021, 8, 0x3025, 24, 0x3038)
        config, count = C.c_void_p(), C.c_int()
        if not self.egl.eglChooseConfig(self.display, attrs, C.byref(config), 1, C.byref(count)) or not count.value:
            raise RuntimeError("EGL RGBA8/depth24 config unavailable")
        self.surface = self.egl.eglCreatePbufferSurface(self.display, config,
            (C.c_int * 5)(0x3057, self.width, 0x3056, self.height, 0x3038))
        self.context = self.egl.eglCreateContext(self.display, config, None,
            (C.c_int * 3)(0x3098, 2, 0x3038))
        if not self.surface or not self.context or not self.egl.eglMakeCurrent(
                self.display, self.surface, self.surface, self.context):
            raise RuntimeError("EGL pbuffer/context creation failed")

    def _program(self):
        shaders = []
        program = self.gl.glCreateProgram()
        try:
            for kind, source in ((0x8B31, b"attribute vec3 position; uniform mat4 clip_from_cloud; void main(){gl_Position=clip_from_cloud*vec4(position,1.0);gl_PointSize=1.0;}"),
                (0x8B30, b"precision mediump float; void main(){gl_FragColor=vec4(0.57,0.69,0.75,1.0);}")):
                shader = self.gl.glCreateShader(kind)
                shaders.append(shader)
                value = C.c_char_p(source)
                self.gl.glShaderSource(shader, 1, C.byref(value), None)
                self.gl.glCompileShader(shader)
                good = C.c_int()
                self.gl.glGetShaderiv(shader, 0x8B81, C.byref(good))
                if not good.value:
                    log = C.create_string_buffer(2048)
                    self.gl.glGetShaderInfoLog(shader, len(log), None, log)
                    raise RuntimeError(f"GLES shader: {log.value.decode()}")
                self.gl.glAttachShader(program, shader)
            self.gl.glLinkProgram(program)
            good = C.c_int()
            self.gl.glGetProgramiv(program, 0x8B82, C.byref(good))
            if not good.value:
                log = C.create_string_buffer(2048)
                self.gl.glGetProgramInfoLog(program, len(log), None, log)
                raise RuntimeError(f"GLES program: {log.value.decode()}")
            return program
        except BaseException:
            self.gl.glDeleteProgram(program)
            raise
        finally:
            for shader in shaders: self.gl.glDeleteShader(shader)

    def render(self, points, clip_from_cloud):
        points = np.ascontiguousarray(points, dtype=np.float32)
        matrix = np.asarray(clip_from_cloud, dtype=np.float32)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) > self.max_points
                or matrix.shape != (4, 4) or not np.isfinite(points).all()
                or not np.isfinite(matrix).all()):
            raise ValueError("bounded finite XYZ points and 4x4 projection required")
        matrix = np.ascontiguousarray(matrix.T)
        g = self.gl
        g.glViewport(0, 0, self.width, self.height)
        g.glClearColor(0., 0., 0., 0.)
        g.glClear(0x4000 | 0x0100)
        g.glEnable(0x0B71)  # GL_DEPTH_TEST; nearest observed point wins.
        g.glUseProgram(self.program)
        g.glBindBuffer(0x8892, self.vbo)
        g.glBufferSubData(0x8892, 0, points.nbytes, points.ctypes.data)
        g.glUniformMatrix4fv(self.matrix, 1, 0, matrix.ctypes.data)
        g.glEnableVertexAttribArray(self.position)
        g.glVertexAttribPointer(self.position, 3, 0x1406, 0, 12, None)
        g.glDrawArrays(0, 0, len(points))  # GL_POINTS, one pixel; never hole filling.
        g.glReadPixels(0, 0, self.width, self.height, 0x1908, 0x1401, self.pixels.ctypes.data)
        error = g.glGetError()
        if error:
            raise RuntimeError(f"GLES rasterization error {error:#x}")
        return np.ascontiguousarray(self.pixels[::-1])

    def close(self):
        if self.display:
            self.egl.eglMakeCurrent(self.display, None, None, None)
            if self.context: self.egl.eglDestroyContext(self.display, self.context)
            if self.surface: self.egl.eglDestroySurface(self.display, self.surface)
            self.egl.eglTerminate(self.display)
        self.display = self.context = self.surface = None
