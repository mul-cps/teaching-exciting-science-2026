"""Bounded occupied-surface voxel map and PointCloud2 decoding helpers."""

from collections import OrderedDict
import math

import numpy as np


MAX_CLOUD_POINTS = 640 * 480


def cloud_xyz_chunks(message, *, chunk_rows=16):
    """Yield small XYZ arrays from PointCloud2 without copying its full payload."""
    height, width = int(message.height), int(message.width)
    step, row_step = int(message.point_step), int(message.row_step)
    if height <= 0 or width <= 0 or height * width > MAX_CLOUD_POINTS:
        raise ValueError("PointCloud2 dimensions exceed the 640x480 bound")
    # RealSense can emit an unordered height-1 cloud whose compacted payload has
    # a stale organized row_step from its original image allocation.
    if height == 1 and len(message.data) == width * step:
        row_step = width * step
    if step <= 0 or row_step < width * step or len(message.data) < height * row_step:
        raise ValueError("Malformed PointCloud2 row/point layout")
    fields = {field.name: field for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x, y, and z fields")
    endian = ">" if message.is_bigendian else "<"
    xyz_fields = []
    for name in ("x", "y", "z"):
        field = fields[name]
        dtype_id = int(field.datatype)
        if dtype_id not in (7, 8) or int(field.count) != 1:
            raise ValueError("XYZ fields must be scalar float32 or float64")
        dtype = np.dtype(endian + ("f4" if dtype_id == 7 else "f8"))
        if int(field.offset) + dtype.itemsize > step:
            raise ValueError("XYZ field extends beyond point_step")
        xyz_fields.append((int(field.offset), dtype))

    view = memoryview(message.data)
    rows_per_chunk = max(1, int(chunk_rows))
    for start in range(0, height, rows_per_chunk):
        stop = min(height, start + rows_per_chunk)
        count = (stop - start) * width
        coords = []
        for offset, dtype in xyz_fields:
            values = np.ndarray(
                (stop - start, width), dtype=dtype, buffer=view,
                offset=start * row_step + offset,
                strides=(row_step, step),
            )
            coords.append(values.reshape(count))
        yield np.column_stack(coords).astype(np.float32, copy=False)


class SurfaceVoxelMap:
    """Sparse, bounded voxel cache; cells mean observed surfaces only."""

    def __init__(self, resolution=0.01, ttl_s=0.8, max_voxels=60000,
                 bounds=((-0.6, 0.6), (-0.6, 0.6), (-0.05, 0.8))):
        self.resolution = float(resolution)
        self.ttl_s = float(ttl_s)
        self.max_voxels = int(max_voxels)
        self.bounds = tuple(tuple(map(float, axis)) for axis in bounds)
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be positive and finite")
        if not math.isfinite(self.ttl_s) or self.ttl_s <= 0:
            raise ValueError("ttl_s must be positive and finite")
        if self.max_voxels <= 0 or any(a >= b for a, b in self.bounds):
            raise ValueError("max_voxels and bounds must be positive/ordered")
        self.cells = OrderedDict()
        self.last_input_points = 0
        self.last_accepted_points = 0
        self.last_status = "waiting"

    def clear_expired(self, now):
        cutoff = float(now) - self.ttl_s
        removed = 0
        while self.cells:
            _, seen = next(iter(self.cells.items()))
            if seen >= cutoff:
                break
            self.cells.popitem(last=False)
            removed += 1
        return removed

    def integrate(self, points, now):
        """Crop and insert a bounded cloud chunk; return accepted sample count."""
        points = np.asarray(points)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        self.last_input_points += len(points)
        finite = np.isfinite(points).all(axis=1)
        for axis, (low, high) in enumerate(self.bounds):
            finite &= (points[:, axis] >= low) & (points[:, axis] <= high)
        inside = points[finite]
        self.last_accepted_points += len(inside)
        if len(inside):
            indices = np.floor(inside / self.resolution).astype(np.int32)
            for key in set(map(tuple, indices.tolist())):
                self.cells.pop(key, None)
                self.cells[key] = float(now)
        self.clear_expired(now)
        while len(self.cells) > self.max_voxels:
            self.cells.popitem(last=False)
        return len(inside)

    def centers(self):
        if not self.cells:
            return np.empty((0, 3), dtype=np.float32)
        indices = np.asarray(sorted(self.cells), dtype=np.float32)
        return (indices + 0.5) * self.resolution

