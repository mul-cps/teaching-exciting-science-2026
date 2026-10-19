"""Bounded vectorized PointCloud2 XYZ and measured packed-color sampling."""
import base64
import math

import numpy as np


def sampled_cloud(message, max_points=20000):
    """Return aligned XYZ/RGB from the same samples; no invented scene color.

    PCL rgb/rgba fields store uint32 bits even when the declared ROS datatype
    is FLOAT32. Read those bytes as integers, preserving NaN float bit patterns.
    """
    height, width, step = int(message.height), int(message.width), int(message.point_step)
    count = height * width
    if min(height, width, step) <= 0 or count > 1280 * 720 or not 1 <= max_points <= 20000:
        raise ValueError("cloud dimensions or sample bound invalid")
    row_step = int(message.row_step)
    if height == 1 and len(message.data) == width * step:
        row_step = width * step
    if row_step < width * step or len(message.data) < row_step * height:
        raise ValueError("malformed cloud layout")
    fields = {field.name: field for field in message.fields}
    stride = math.ceil(count / max_points)
    indices = np.arange(0, count, stride)
    rows, cols = indices // width, indices % width
    points = np.empty((len(indices), 3), dtype=np.float32)
    for axis, name in enumerate(("x", "y", "z")):
        field = fields.get(name)
        if field is None or int(field.datatype) != 7 or int(field.count) != 1 or not 0 <= field.offset <= step - 4:
            raise ValueError("scalar float32 XYZ fields required")
        view = np.ndarray((height, width), buffer=message.data,
            dtype=(">f4" if message.is_bigendian else "<f4"),
            offset=int(field.offset), strides=(row_step, step))
        points[:, axis] = view[rows, cols]
    finite = np.isfinite(points).all(axis=1)
    points = np.ascontiguousarray(points[finite])
    name = "rgb" if "rgb" in fields else "rgba" if "rgba" in fields else None
    if name is None:
        return points, None, "uniform_fallback_rgb_absent"
    field = fields[name]
    if int(field.datatype) not in (6, 7) or int(field.count) != 1 or not 0 <= field.offset <= step - 4:
        raise ValueError("present RGB/RGBA must be scalar packed FLOAT32 or UINT32")
    packed_view = np.ndarray((height, width), buffer=message.data,
        dtype=(">u4" if message.is_bigendian else "<u4"),
        offset=int(field.offset), strides=(row_step, step))
    packed = packed_view[rows, cols][finite]
    colors = np.empty((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (packed >> 16) & 255
    colors[:, 1] = (packed >> 8) & 255
    colors[:, 2] = packed & 255
    return points, colors, "packed_" + name


def encode_rgb8(colors):
    colors = np.asarray(colors)
    if colors.dtype != np.uint8 or colors.ndim != 2 or colors.shape[1] != 3 or len(colors) > 20000:
        raise ValueError("bounded uint8 RGB rows required")
    return base64.b64encode(np.ascontiguousarray(colors).tobytes()).decode("ascii")


def decode_rgb8(value, count, max_points=12000):
    if not 0 <= count <= max_points or not isinstance(value, str) or len(value) != count * 4:
        raise ValueError("bounded exact RGB base64 payload required")
    raw = base64.b64decode(value, validate=True)
    if len(raw) != count * 3:
        raise ValueError("RGB byte count must match sampled XYZ count")
    return np.frombuffer(raw, dtype=np.uint8).reshape(count, 3)
