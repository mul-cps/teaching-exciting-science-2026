"""Strict station-export binary PCD admission; no general PCL parser invocation."""
from pathlib import Path
import numpy as np

MAX_POINTS = 1_000_000
MAX_HEADER = 4096
MAX_BYTES = MAX_HEADER + MAX_POINTS * 16


def checked_pcd(path):
    path = Path(path)
    if not 0 < path.stat().st_size <= MAX_BYTES:
        raise ValueError('PCD storage exceeds bounded binary schema')
    with path.open('rb') as stream:
        data = stream.read(MAX_BYTES + 1)
    if len(data) > MAX_BYTES:
        raise ValueError('PCD storage exceeds bounded binary schema')
    end = data.find(b'DATA binary\n', 0, MAX_HEADER)
    if end < 0:
        raise ValueError('PCD requires bounded station binary header (ASCII/compressed unsupported)')
    offset = end + len(b'DATA binary\n')
    try:
        fields = {}
        for line in data[:offset].decode('ascii').splitlines():
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            if key in fields:
                raise ValueError('duplicate PCD header field')
            fields[key] = values
        fixed = {'FIELDS': ['x', 'y', 'z', 'rgba'], 'SIZE': ['4']*4,
                 'TYPE': ['F', 'F', 'F', 'U'], 'COUNT': ['1']*4,
                 'VIEWPOINT': ['0', '0', '0', '1', '0', '0', '0'], 'DATA': ['binary']}
        if (set(fields) != set(fixed) | {'VERSION', 'WIDTH', 'HEIGHT', 'POINTS'}
                or fields['VERSION'] not in (['.7'], ['0.7'])
                or any(fields.get(k) != v for k, v in fixed.items())):
            raise ValueError('unsupported PCD schema')
        dimensions = []
        for name in ('WIDTH', 'HEIGHT', 'POINTS'):
            value = fields[name]
            if len(value) != 1 or not value[0].isascii() or not value[0].isdigit() or len(value[0]) > 7:
                raise ValueError('invalid bounded PCD dimensions')
            dimensions.append(int(value[0]))
        width, height, points = dimensions
        if (not 1 <= points <= MAX_POINTS or not 1 <= width <= MAX_POINTS
                or not 1 <= height <= MAX_POINTS or width*height != points
                or len(data)-offset != points*16):
            raise ValueError('PCD point count/storage mismatch')
    except (UnicodeError, KeyError) as error:
        raise ValueError('invalid PCD header') from error
    xyz = np.ndarray((points, 3), dtype='<f4', buffer=data, offset=offset, strides=(16, 4))
    if not np.isfinite(xyz).all():
        raise ValueError('PCD requires finite XYZ')
    return {'points': points, 'data_offset': offset, 'storage_bytes': len(data)}
