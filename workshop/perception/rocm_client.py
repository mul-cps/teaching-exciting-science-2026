"""Nonblocking parent side of one private, explicitly enabled GPU subprocess."""
import hashlib
import os
from pathlib import Path
import socket
import subprocess
import time
import tempfile
import pwd
import signal
import ctypes
import errno

from .rocm_projection import calibration_from_oracle
from .rocm_transport import (MAX_PACKET, LatestSlots, SharedSlots, SlotLayout,
                             decode_packet, encode_packet, source_hashes)


class OwnedProcessGroup:
    """Stable Linux>=6.9 pidfd group, never an integer-PID signaling fallback.

    kernel/signal.c PIDFD_SIGNAL_PROCESS_GROUP selects PIDTYPE_PGID from the
    pinned struct pid, including after its leader is dead/reaped. This prevents
    a recycled process ID from ever selecting another session. libc wrappers
    are used because this Conda Python omits the optional os/signal APIs.
    """
    def __init__(self, process):
        self.fd = None; self.supported = False
        self.libc = ctypes.CDLL(None, use_errno=True)
        self.libc.pidfd_open.argtypes = [ctypes.c_int, ctypes.c_uint]
        self.libc.pidfd_open.restype = ctypes.c_int
        self.libc.pidfd_send_signal.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint]
        self.libc.pidfd_send_signal.restype = ctypes.c_int
        fd = self.libc.pidfd_open(process.pid, 0)
        if fd < 0: raise OSError(ctypes.get_errno(), 'private worker pidfd_open failed')
        self.fd = fd

    def probe(self):
        self.send(0, group=True)
        self.supported = True

    def send(self, number, *, group=True):
        if self.fd is None: return
        result = self.libc.pidfd_send_signal(self.fd, number, None, 4 if group else 0)
        if result < 0 and ctypes.get_errno() != errno.ESRCH:
            raise OSError(ctypes.get_errno(), 'private pidfd group signal unsupported/failed')

    def close(self):
        if self.fd is not None: os.close(self.fd); self.fd = None


class ProjectionWorker:
    def __init__(self, configuration, python_executable, compatibility_library, *, enabled=False):
        if enabled is not True: raise ValueError('private worker requires explicit enabled=True')
        calibration = calibration_from_oracle(configuration)
        executable, library = Path(python_executable).resolve(), Path(compatibility_library).resolve()
        if not executable.is_file() or not library.is_file(): raise ValueError('explicit worker Python/library paths required')
        self.scheduler = LatestSlots(); self.ready = False; self.hardware = None
        self.started = time.monotonic(); self.dispatched = None; self.closed = False
        self.expected_sources = source_hashes()
        library_sha = hashlib.sha256(library.read_bytes()).hexdigest()
        self.provenance = dict(source_sha256=self.expected_sources, worker_python=str(executable),
            worker_python_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(), compatibility_sha256=library_sha)
        # Complete every fallible source/hash read before acquiring shared FDs.
        self.slots = SharedSlots.create(SlotLayout(calibration.depth_size, calibration.color_size))
        self.cache = None
        try:
            self.cache = tempfile.TemporaryDirectory(prefix='so101-private-rocm-')
            parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        except Exception:
            self.slots.close()
            if self.cache is not None: self.cache.cleanup()
            raise
        self.connection = parent; self.process = None; self.group = None
        try:
            # Construct an allowlist, not a denylist: no inherited ROS3.12,
            # AMENT/Conda/loader/Python ABI paths enter the isolated3.13 child.
            cache = Path(self.cache.name); (cache/'tmp').mkdir()
            environment = dict(PATH=str(executable.parent)+':/opt/rocm/bin:/usr/bin:/bin',
                HOME=pwd.getpwuid(os.getuid()).pw_dir, LANG='C.UTF-8', LC_ALL='C.UTF-8',
                PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
                TRITON_CACHE_DIR=str(cache/'triton'), TORCHINDUCTOR_CACHE_DIR=str(cache/'inductor'),
                XDG_CACHE_HOME=str(cache/'xdg'), TMPDIR=str(cache/'tmp'))
            self.process = subprocess.Popen([str(executable), str(Path(__file__).with_name('rocm_worker.py')),
                '--enabled', '--socket-fd', str(child.fileno()), '--input-fd', str(self.slots.input_fd),
                '--output-fd', str(self.slots.output_fd), '--compatibility-library', str(library),
                '--compatibility-sha256', library_sha],
                pass_fds=(child.fileno(), self.slots.input_fd, self.slots.output_fd),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                env=environment, cwd='/', start_new_session=True)
            self.group = OwnedProcessGroup(self.process)
            # No configure bytes (and therefore no GPU import) until a stable
            # private session/group has actually passed the kernel API probe.
            if os.getpgid(self.process.pid) != self.process.pid:
                raise RuntimeError('private worker did not create its own session')
            self.group.probe()
            parent.setblocking(False)
            parent.send(encode_packet(dict(kind='configure', protocol=1, calibration=configuration)))
        except Exception:
            self.close(); raise
        finally: child.close()

    def offer(self, depth, rgb, metadata):
        if not self.ready or self.closed: raise RuntimeError('GPU worker is not ready')
        offered = self.scheduler.offer(metadata)
        try: self.slots.write_input(offered['slot'], depth, rgb)
        except Exception:
            self.scheduler.pending = None
            raise
        return offered

    def _dispatch(self):
        job = self.scheduler.begin()
        if job is not None:
            self.connection.send(encode_packet(dict(kind='job', **job)))
            self.dispatched = time.monotonic()

    def poll(self):
        if self.closed: raise RuntimeError('worker is closed')
        now = time.monotonic()
        exit_code = self.process.poll()
        if ((not self.ready and now-self.started > 30.)
                or (self.dispatched is not None and now-self.dispatched > .5)):
            raise TimeoutError('worker startup/job deadline expired')
        try: raw = self.connection.recv(MAX_PACKET+1)
        except BlockingIOError: raw = None
        result = None
        if raw is not None:
            if not raw: raise RuntimeError('private worker transport closed')
            reply = decode_packet(raw)
            if exit_code is not None and reply.get('kind') != 'error':
                raise RuntimeError('private worker exited; no automatic restart')
            if reply.get('kind') == 'ready' and not self.ready and type(reply.get('protocol')) is int and reply.get('protocol') == 1:
                if (reply.get('qualified') is not False or type(reply.get('hardware')) is not dict
                        or reply.get('source_sha256') != self.expected_sources):
                    raise ValueError('malformed worker readiness')
                self.ready = True; self.hardware = reply['hardware']; self.startup_ms = reply.get('startup_ms')
            elif reply.get('kind') == 'result' and self.ready:
                active = self.scheduler.active
                if (reply.get('qualified') is not False or active is None
                        or reply.get('metadata') != active['metadata']
                        or type(reply.get('slot')) is not int or reply['slot'] != active['slot']
                        or type(reply.get('sequence')) is not int or reply['sequence'] != active['sequence']):
                    raise ValueError('worker provenance changed')
                # Deadline checked again after receive/validation, not only entry.
                if self.dispatched is None or time.monotonic()-self.dispatched > .5:
                    raise TimeoutError('worker result expired after receipt')
                aligned, cloud = self.slots.owned_ros_arrays(reply.get('slot'), reply.get('cloud_bytes'))
                if time.monotonic()-self.dispatched > .5:
                    raise TimeoutError('worker deadline expired during output ownership copy')
                job = self.scheduler.complete(reply); self.dispatched = None
                result = dict(reply, aligned=aligned, cloud=cloud)
            else: raise RuntimeError('worker rejected/unknown protocol result: '+str(reply)[:1200])
        if raw is None and exit_code is not None: raise RuntimeError('private worker exited; no automatic restart')
        if self.ready: self._dispatch()
        return result

    def close(self):
        if self.closed: return
        self.closed = True
        try:
            try: self.connection.send(encode_packet({'kind': 'stop'}))
            except OSError: pass
            if self.process is not None:
                if self.group is not None:
                    # Also retire compiler descendants when the leader already
                    # exited: the held pidfd cannot select a recycled group.
                    self.group.send(signal.SIGTERM, group=self.group.supported)
                    try: self.process.wait(timeout=.5)
                    except subprocess.TimeoutExpired: pass
                    self.group.send(signal.SIGKILL, group=self.group.supported)
                    self.process.wait(timeout=.5)
                else:
                    # pidfd_open failed BEFORE configure. Closing the CPU-only
                    # socket supplies EOF; no numeric PID signaling fallback.
                    self.connection.close(); self.process.wait(timeout=.5)
        finally:
            if self.group is not None: self.group.close()
            self.connection.close(); self.slots.close(); self.scheduler.invalidate(); self.cache.cleanup()
