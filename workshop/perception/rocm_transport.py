"""Bounded derived-cohort protocol and fixed memfd slots; no ROS/GPU imports."""
from dataclasses import dataclass
import json
import mmap
import os
import ctypes
from array import array
import hashlib
from pathlib import Path

import numpy as np

MAX_PACKET = 65536
MAX_AGE_NS = 500_000_000
MAX_PAIR_NS = 40_000_000
SOURCE_FILES = ('rocm_transport.py', 'rocm_client.py', 'rocm_worker.py',
                'rocm_projection.py', 'rocm_projection_backend.py', 'rocm_projection_kernels.py')


def source_hashes():
    """Startup only; never called by source callbacks or per-frame processing."""
    return {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in SOURCE_FILES}


def _memfd(name):
    # The base Conda Python omits os.memfd_create although this Linux libc
    # exports it. Same anonymous-kernel API, never a temporary-file fallback.
    if hasattr(os, 'memfd_create'): return os.memfd_create(name, os.MFD_CLOEXEC)
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.memfd_create
    function.argtypes = [ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    fd = function(name.encode('ascii'), 1)  # Linux MFD_CLOEXEC
    if fd < 0: raise OSError(ctypes.get_errno(), 'anonymous memfd creation failed')
    return fd


def encode_packet(value):
    if type(value) is not dict:
        raise ValueError('IPC requires one bounded dictionary')
    try:
        raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
    except (TypeError, ValueError, OverflowError, RecursionError) as error:
        raise ValueError('IPC requires finite JSON values') from error
    if len(raw) > MAX_PACKET:
        raise ValueError('IPC packet exceeds fixed bound')
    return raw


def decode_packet(raw):
    if type(raw) is not bytes or not 0 < len(raw) <= MAX_PACKET:
        raise ValueError('invalid IPC packet storage')
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result: raise ValueError('duplicate IPC key')
            result[key] = value
        return result
    def constant(value): raise ValueError('nonfinite IPC number')
    try:
        value = json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)
        # A large exponent may parse as infinity without parse_constant.
        encode_packet(value)
    except (UnicodeError, ValueError, TypeError, RecursionError) as error:
        raise ValueError('invalid bounded finite IPC JSON') from error
    return value


def _size(value, maximum):
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or any(type(x) is not int or not 1 <= x <= limit for x, limit in zip(value, maximum))):
        raise ValueError('unsupported fixed image dimensions')
    return tuple(value)


@dataclass(frozen=True)
class SlotLayout:
    depth_size: tuple
    color_size: tuple

    def __post_init__(self):
        object.__setattr__(self, 'depth_size', _size(self.depth_size, (1280, 720)))
        object.__setattr__(self, 'color_size', _size(self.color_size, (1920, 1080)))

    @property
    def depth_bytes(self): return self.depth_size[0]*self.depth_size[1]*2
    @property
    def color_bytes(self): return self.color_size[0]*self.color_size[1]*3
    @property
    def aligned_bytes(self): return self.color_size[0]*self.color_size[1]*2
    @property
    def cloud_capacity(self): return self.depth_size[0]*self.depth_size[1]*20
    @property
    def input_capacity(self): return self.depth_bytes+self.color_bytes
    @property
    def output_capacity(self): return self.aligned_bytes+self.cloud_capacity


class SharedSlots:
    """Exactly two fixed inputs/outputs, descriptors passed to one private child.

    The sequence scheduler owns slot lifetime. Input views are borrowed only
    while their slot is active; output publication always acquires owned bytes.
    """
    def __init__(self, layout, input_fd, output_fd):
        self.layout = layout
        self.input_fd, self.output_fd = input_fd, output_fd
        if (os.fstat(input_fd).st_size != 2*layout.input_capacity
                or os.fstat(output_fd).st_size != 2*layout.output_capacity):
            raise ValueError('shared slot file dimensions differ from admitted layout')
        self.input = mmap.mmap(input_fd, 2*layout.input_capacity)
        try: self.output = mmap.mmap(output_fd, 2*layout.output_capacity)
        except Exception:
            self.input.close(); raise

    @classmethod
    def create(cls, layout):
        descriptors = []
        try:
            for name, size in (('so101-rocm-input', 2*layout.input_capacity),
                               ('so101-rocm-output', 2*layout.output_capacity)):
                fd = _memfd(name)
                descriptors.append(fd); os.ftruncate(fd, size)
            return cls(layout, *descriptors)
        except Exception:
            for fd in descriptors: os.close(fd)
            raise

    @staticmethod
    def _slot(slot):
        if type(slot) is not int or slot not in (0, 1): raise ValueError('invalid fixed slot')
        return slot

    def write_input(self, slot, depth, rgb):
        slot = self._slot(slot); l = self.layout
        if (not isinstance(depth, np.ndarray) or depth.dtype != np.uint16
                or depth.shape != (l.depth_size[1], l.depth_size[0])
                or not depth.flags.c_contiguous or not isinstance(rgb, np.ndarray)
                or rgb.dtype != np.uint8 or rgb.shape != (l.color_size[1], l.color_size[0], 3)
                or not rgb.flags.c_contiguous):
            raise ValueError('expected exact bounded source arrays')
        offset = slot*l.input_capacity
        np.copyto(np.ndarray(depth.shape, '<u2', buffer=self.input, offset=offset), depth)
        np.copyto(np.ndarray(rgb.shape, np.uint8, buffer=self.input, offset=offset+l.depth_bytes), rgb)

    def input_views(self, slot):
        offset = self._slot(slot)*self.layout.input_capacity; l = self.layout
        depth = np.ndarray((l.depth_size[1], l.depth_size[0]), '<u2', buffer=self.input, offset=offset)
        rgb = np.ndarray((l.color_size[1], l.color_size[0], 3), np.uint8,
                         buffer=self.input, offset=offset+l.depth_bytes)
        depth.setflags(write=False); rgb.setflags(write=False)
        return depth, rgb

    def write_output(self, slot, aligned, cloud):
        l = self.layout; offset = self._slot(slot)*l.output_capacity
        if (not isinstance(aligned, np.ndarray) or aligned.dtype != np.uint16
                or aligned.shape != (l.color_size[1], l.color_size[0])
                or type(cloud) is not bytes or len(cloud) > l.cloud_capacity or len(cloud) % 20):
            raise ValueError('invalid bounded output storage')
        np.copyto(np.ndarray(aligned.shape, '<u2', buffer=self.output, offset=offset), aligned)
        self.output[offset+l.aligned_bytes:offset+l.aligned_bytes+len(cloud)] = cloud

    def write_aligned(self, slot, aligned):
        l = self.layout; offset = self._slot(slot)*l.output_capacity
        if (not isinstance(aligned, np.ndarray) or aligned.dtype != np.uint16
                or aligned.shape != (l.color_size[1], l.color_size[0])):
            raise ValueError('invalid aligned output shape/type')
        np.copyto(np.ndarray(aligned.shape, '<u2', buffer=self.output, offset=offset), aligned)

    def cloud_output_view(self, slot):
        l = self.layout; offset = self._slot(slot)*l.output_capacity+l.aligned_bytes
        return np.ndarray((l.cloud_capacity,), np.uint8, buffer=self.output, offset=offset)

    def owned_output(self, slot, cloud_bytes):
        l = self.layout; offset = self._slot(slot)*l.output_capacity
        if type(cloud_bytes) is not int or not 0 <= cloud_bytes <= l.cloud_capacity or cloud_bytes % 20:
            raise ValueError('invalid cloud output byte count')
        return (self.output[offset:offset+l.aligned_bytes],
                self.output[offset+l.aligned_bytes:offset+l.aligned_bytes+cloud_bytes])

    def owned_ros_arrays(self, slot, cloud_bytes):
        """One owned array copy per message, no intermediate bytes or mmap alias.

        Validate every range before allocating. Both arrays must exist before
        the active slot may be released/recycled by the scheduler.
        """
        l = self.layout; offset = self._slot(slot)*l.output_capacity
        if type(cloud_bytes) is not int or not 0 <= cloud_bytes <= l.cloud_capacity or cloud_bytes % 20:
            raise ValueError('invalid cloud output byte count')
        aligned, cloud = array('B'), array('B')
        with memoryview(self.output).toreadonly() as data:
            with data[offset:offset+l.aligned_bytes] as section: aligned.frombytes(section)
            start = offset+l.aligned_bytes
            with data[start:start+cloud_bytes] as section: cloud.frombytes(section)
        return aligned, cloud

    def close(self):
        self.input.close(); self.output.close()
        os.close(self.input_fd); os.close(self.output_fd)


class WirePacker:
    """One native pass directly into the active fixed output slot; no allocation."""
    def __init__(self, library):
        self.library = ctypes.CDLL(str(library))
        try: abi = self.library.rocm_wire_pack_abi()
        except AttributeError as error: raise ValueError('bridge runtime wire ABI missing') from error
        if abi != 1: raise ValueError('unsupported bridge wire ABI')
        self.function = self.library.rocm_wire_pack
        array = lambda dtype, ndim, write=False: np.ctypeslib.ndpointer(dtype=dtype, ndim=ndim,
            flags=('C_CONTIGUOUS', 'ALIGNED', 'WRITEABLE') if write else ('C_CONTIGUOUS', 'ALIGNED'))
        self.function.argtypes = [array(np.float32, 3), array(np.uint32, 2), array(np.uint8, 2),
            ctypes.c_size_t, array(np.uint8, 1, True), ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t)]
        self.function.restype = ctypes.c_int

    def pack_into(self, xyz, packed_rgb, valid, output):
        if (not isinstance(xyz, np.ndarray) or xyz.dtype != np.float32 or xyz.ndim != 3 or xyz.shape[-1] != 3
                or not isinstance(packed_rgb, np.ndarray) or packed_rgb.dtype != np.uint32 or packed_rgb.shape != xyz.shape[:2]
                or not isinstance(valid, np.ndarray) or valid.dtype not in (np.dtype('uint8'), np.dtype('bool'))
                or valid.shape != xyz.shape[:2] or not isinstance(output, np.ndarray) or output.dtype != np.uint8
                or output.ndim != 1 or not output.flags.writeable
                or not all(array.flags.c_contiguous and array.flags.aligned for array in (xyz, packed_rgb, valid, output))):
            raise ValueError('invalid native wire buffer shape/type/stride')
        count = xyz.shape[0]*xyz.shape[1]
        if count > 921600 or output.size < count*20: raise ValueError('native wire capacity/count exceeds bound')
        written = ctypes.c_size_t(0)
        error = self.function(xyz, packed_rgb, valid.view(np.uint8), count, output, output.size, ctypes.byref(written))
        if error: raise ValueError('native wire pack rejected complete cohort: '+str(error))
        if written.value > count: raise ValueError('native wire output count is invalid')
        return written.value*20


def pack_cloud(xyz, packed_rgb, valid):
    if (not isinstance(xyz, np.ndarray) or xyz.dtype != np.float32 or xyz.ndim != 3 or xyz.shape[-1] != 3
            or not isinstance(packed_rgb, np.ndarray) or packed_rgb.dtype != np.uint32
            or packed_rgb.shape != xyz.shape[:2] or not isinstance(valid, np.ndarray)
            or valid.dtype not in (np.dtype('uint8'), np.dtype('bool')) or valid.shape != xyz.shape[:2]
            or xyz.shape[0]*xyz.shape[1] > 1280*720 or np.any(valid > 1)):
        raise ValueError('invalid bounded point cloud cohort')
    keep = valid.astype(bool, copy=False)
    points = xyz[keep]
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        raise ValueError('valid cloud points must have finite positive depth')
    dtype = np.dtype({'names': ['x', 'y', 'z', 'padding', 'rgb'],
                      'formats': ['<f4', '<f4', '<f4', '<u4', '<u4'],
                      'offsets': [0, 4, 8, 12, 16], 'itemsize': 20})
    output = np.zeros(len(points), dtype)
    output['x'], output['y'], output['z'] = points.T
    output['rgb'] = packed_rgb[keep]
    return output.tobytes()


def build_lineage(metadata, result, output_binding, calibration, implementation, observation_ros_ns):
    """Canonical emitted v1 dictionary. Inputs are already admitted/cached.

    Source generation is SHA256(json.dumps(snapshot, sort_keys=True).encode())
    using Python JSON's default separators/ensure_ascii. Its exact snapshot is
    identity.device_info_request_source: {node,endpoints:{stream:{node,gid}}}.
    Calibration SHA refers to original configuration FILE BYTES, not a JSON
    reserialization. Provenance is trusted local-domain evidence, not a signature.
    """
    source = metadata['source']; endpoints = source['endpoints']; identity = metadata['identity']
    return dict(schema='so101-derived-rocm-rgbd-cohort-v1', qualified=False,
        execution_enabled=False, direct_camera_attestation=False,
        producer_generation=metadata['producer_generation'], sequence=result['sequence'],
        source_generation=metadata['source_generation'],
        original=dict(depth_stamp_ns=metadata['depth_stamp_ns'], color_stamp_ns=metadata['color_stamp_ns'],
            depth_info_stamp_ns=metadata['depth_info_stamp_ns'], color_info_stamp_ns=metadata['color_info_stamp_ns'],
            depth_frame=metadata['depth_frame'], color_frame=metadata['color_frame'],
            pair_delta_ns=abs(metadata['depth_stamp_ns']-metadata['color_stamp_ns'])),
        source=dict(camera_node=source['node'], serial_number=identity['serial_number'],
            depth_topic='/camera/depth/image_rect_raw', color_topic='/camera/color/image_raw',
            depth_publisher_gid=endpoints['depth']['gid'], color_publisher_gid=endpoints['color']['gid'],
            depth_info_publisher_gid=endpoints['depth_info']['gid'], color_info_publisher_gid=endpoints['color_info']['gid']),
        identity=identity,
        outputs=dict(output_binding, cloud_topic='/so101_perception/rocm/points',
            status_topic='/so101_perception/rocm/lineage', cloud_stamp_ns=metadata['depth_stamp_ns'],
            cloud_frame=metadata['depth_frame'], aligned_stamp_ns=metadata['depth_stamp_ns'],
            aligned_frame=metadata['color_frame'], aligned_topic='/so101_perception/rocm/aligned_depth'),
        calibration=calibration, implementation=implementation,
        health=dict(state='valid', source_receipts_monotonic=metadata['source_received_monotonic'],
            source_expiry_monotonic=min(identity['expires_monotonic'],
                *(receipt+.250 for receipt in metadata['source_received_monotonic'].values())),
            original_max_age_ns=MAX_AGE_NS, observation_ros_ns=observation_ros_ns),
        metrics=dict(points=result['cloud_bytes']//20, cloud_bytes=result['cloud_bytes'],
            backend_ms=result['backend_ms'], worker_ms=result['worker_ms']))


class ExposureLedger:
    def __init__(self, source_generation):
        if (type(source_generation) is not str or len(source_generation) != 64
                or any(x not in '0123456789abcdef' for x in source_generation)):
            raise ValueError('pinned source generation required')
        self.source_generation = source_generation
        self.depth = self.color = None

    def admit(self, metadata, now_ns):
        if type(metadata) is not dict or metadata.get('source_generation') != self.source_generation:
            raise ValueError('source generation changed')
        depth, color = metadata.get('depth_stamp_ns'), metadata.get('color_stamp_ns')
        if (any(type(x) is not int or x <= 0 for x in (depth, color, now_ns))
                or not 0 <= now_ns-depth <= MAX_AGE_NS or not 0 <= now_ns-color <= MAX_AGE_NS
                or abs(depth-color) > MAX_PAIR_NS):
            raise ValueError('source stamps are future/stale/unpaired')
        if ((self.depth is not None and depth < self.depth)
                or (self.color is not None and color < self.color)):
            raise ValueError('source stamp rollback')
        if depth == self.depth or color == self.color: return False
        self.depth, self.color = depth, color
        return True


class LatestSlots:
    """One active job plus one replaceable pending slot, no unbounded queue."""
    def __init__(self):
        self.active = self.pending = None
        self.sequence = 0

    def offer(self, metadata):
        # Finite JSON copy freezes nested provenance, never aliases caller state.
        metadata = decode_packet(encode_packet(metadata))
        self.sequence += 1
        slot = 1-self.active['slot'] if self.active is not None else 0
        self.pending = dict(slot=slot, sequence=self.sequence, metadata=metadata)
        return self.pending

    def begin(self):
        if self.active is not None or self.pending is None: return None
        self.active, self.pending = self.pending, None
        return self.active

    def complete(self, result):
        if (self.active is None or result.get('slot') != self.active['slot']
                or result.get('sequence') != self.active['sequence']
                or type(result.get('slot')) is not int or type(result.get('sequence')) is not int):
            raise ValueError('worker result is not the exact active slot/sequence')
        active, self.active = self.active, None
        return active

    def invalidate(self):
        # Call only after stopping/retiring the worker: a live writer still owns
        # its active slot and must never race a new generation's admission.
        self.active = self.pending = None
