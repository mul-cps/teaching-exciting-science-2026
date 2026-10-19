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
            self.rgb = self.gl.glGetAttribLocation(self.program, b"rgb")
            self.alpha = self.gl.glGetUniformLocation(self.program, b"synthetic_alpha")
            self.vbo = C.c_uint()
            self.gl.glGenBuffers(1, C.byref(self.vbo))
            self.gl.glBindBuffer(0x8892, self.vbo)
            self.gl.glBufferData(0x8892, self.max_points * 12, None, 0x88E0)
            self.color_vbo = C.c_uint()
            self.gl.glGenBuffers(1, C.byref(self.color_vbo))
            self.gl.glBindBuffer(0x8892, self.color_vbo)
            self.gl.glBufferData(0x8892, self.max_points * 3, None, 0x88E0)
            self.mesh_vbos = {}
            self.pixels = np.empty((self.height, self.width * 2, 4), dtype=np.uint8)
            self.readback_bytes = self.pixels.nbytes
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
            ("glDeleteBuffers", None, [I, C.POINTER(U)]),
            ("glDisableVertexAttribArray", None, [U]), ("glVertexAttrib3f", None, [U, F, F, F]),
            ("glUniform1f", None, [I, F]),
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
            (C.c_int * 5)(0x3057, self.width * 2, 0x3056, self.height, 0x3038))
        self.context = self.egl.eglCreateContext(self.display, config, None,
            (C.c_int * 3)(0x3098, 2, 0x3038))
        if not self.surface or not self.context or not self.egl.eglMakeCurrent(
                self.display, self.surface, self.surface, self.context):
            raise RuntimeError("EGL pbuffer/context creation failed")

    def _program(self):
        shaders = []
        program = self.gl.glCreateProgram()
        try:
            for kind, source in ((0x8B31, b"attribute vec3 position; attribute vec3 rgb; varying vec3 color; uniform mat4 clip_from_cloud; void main(){color=rgb;gl_Position=clip_from_cloud*vec4(position,1.0);gl_PointSize=1.0;}"),
                (0x8B30, b"precision mediump float; varying vec3 color; uniform float synthetic_alpha; void main(){gl_FragColor=vec4(color,synthetic_alpha);}")):
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

    def set_meshes(self, meshes):
        """Upload all bounded NOMINAL mesh visuals once; no per-frame STL work."""
        if len(meshes) > 16 or sum(len(m["points"]) for m in meshes) > 600000:
            raise ValueError("gripper visual/triangle bound exceeded")
        staged = {}
        try:
            for mesh in meshes:
                points, colors = np.asarray(mesh["points"]), np.asarray(mesh["colors"])
                if (points.ndim != 2 or points.shape[1] != 3 or len(points) % 3 or
                        not np.isfinite(points).all() or colors.dtype != np.uint8 or
                        colors.shape != points.shape or mesh["name"] in staged):
                    raise ValueError("finite triangle vertices and matching uint8 colors required")
                packed = np.empty(len(points), dtype=np.dtype([
                    ("position", "<f4", 3), ("rgb", "u1", 3), ("pad", "u1")]))
                packed["position"], packed["rgb"], packed["pad"] = points, colors, 0
                vbo = C.c_uint()
                self.gl.glGenBuffers(1, C.byref(vbo))
                staged[mesh["name"]] = (vbo, len(points))
                self.gl.glBindBuffer(0x8892, vbo)
                self.gl.glBufferData(0x8892, packed.nbytes, packed.ctypes.data, 0x88E4)
            if self.gl.glGetError():
                raise RuntimeError("GPU gripper VBO upload failed")
        except BaseException:
            for vbo, _count in staged.values(): self.gl.glDeleteBuffers(1, C.byref(vbo))
            raise
        for vbo, _count in self.mesh_vbos.values(): self.gl.glDeleteBuffers(1, C.byref(vbo))
        self.mesh_vbos = staged

    def _matrix(self, value):
        matrix = np.asarray(value, dtype=np.float32)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("finite 4x4 projection required")
        packed = np.ascontiguousarray(matrix.T)
        self.gl.glUniformMatrix4fv(self.matrix, 1, 0, packed.ctypes.data)

    def render(self, points, clip_from_cloud, colors=None):
        """Compatibility cloud-only view; atlas still uses one GPU readback."""
        return self.render_layers(points, clip_from_cloud, colors=colors)["raw"]

    def render_layers(self, points, clip_from_cloud, colors=None, mesh_matrices=None):
        points = np.ascontiguousarray(points, dtype=np.float32)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) > self.max_points
                or not np.isfinite(points).all()):
            raise ValueError("bounded finite XYZ points required")
        if colors is not None:
            colors = np.ascontiguousarray(colors)
            if colors.dtype != np.uint8 or colors.shape != points.shape:
                raise ValueError("matching uint8 RGB colors required")
        mesh_matrices = {} if mesh_matrices is None else mesh_matrices
        if len(mesh_matrices) > 16 or not mesh_matrices.keys() <= self.mesh_vbos.keys():
            raise ValueError("only cached bounded model visuals may be drawn")
        g = self.gl
        g.glClearColor(0., 0., 0., 0.)
        g.glClear(0x4000 | 0x0100)
        g.glEnable(0x0B71)
        g.glUseProgram(self.program)
        g.glUniform1f(self.alpha, 1.)
        self._matrix(clip_from_cloud)
        g.glBindBuffer(0x8892, self.vbo)
        g.glBufferSubData(0x8892, 0, points.nbytes, points.ctypes.data)
        g.glEnableVertexAttribArray(self.position)
        g.glVertexAttribPointer(self.position, 3, 0x1406, 0, 12, None)
        if colors is None:
            g.glDisableVertexAttribArray(self.rgb)
            g.glVertexAttrib3f(self.rgb, .57, .69, .75)
        else:
            g.glBindBuffer(0x8892, self.color_vbo)
            g.glBufferSubData(0x8892, 0, colors.nbytes, colors.ctypes.data)
            g.glEnableVertexAttribArray(self.rgb)
            g.glVertexAttribPointer(self.rgb, 3, 0x1401, 1, 3, None)
        # Two viewports share one pbuffer/depth allocation and one readback:
        # left preserves unoccluded measured samples, right adds NOMINAL CAD.
        g.glViewport(0, 0, self.width, self.height)
        g.glDrawArrays(0, 0, len(points))
        g.glViewport(self.width, 0, self.width, self.height)
        g.glDrawArrays(0, 0, len(points))
        g.glUniform1f(self.alpha, .5)
        g.glEnableVertexAttribArray(self.rgb)
        for name, matrix in mesh_matrices.items():
            vbo, count = self.mesh_vbos[name]
            self._matrix(matrix)
            g.glBindBuffer(0x8892, vbo)
            g.glVertexAttribPointer(self.position, 3, 0x1406, 0, 16, None)
            g.glVertexAttribPointer(self.rgb, 3, 0x1401, 1, 16, C.c_void_p(12))
            g.glDrawArrays(4, 0, count)  # GL_TRIANGLES; bounded cached visuals.
        g.glReadPixels(0, 0, self.width * 2, self.height, 0x1908, 0x1401, self.pixels.ctypes.data)
        error = g.glGetError()
        if error: raise RuntimeError(f"GLES rasterization error {error:#x}")
        raw = np.ascontiguousarray(self.pixels[::-1, :self.width])
        scene = np.ascontiguousarray(self.pixels[::-1, self.width:])
        alpha = scene[:, :, 3]
        provenance = np.where(alpha == 255, 1, np.where(alpha != 0, 2, 0)).astype(np.uint8)
        scene[:, :, 3] = np.where(provenance != 0, 255, 0)
        return {"raw": raw, "scene": scene, "provenance": provenance}

    def close(self):
        if self.display:
            self.egl.eglMakeCurrent(self.display, None, None, None)
            if self.context: self.egl.eglDestroyContext(self.display, self.context)
            if self.surface: self.egl.eglDestroySurface(self.display, self.surface)
            self.egl.eglTerminate(self.display)
        self.display = self.context = self.surface = None
