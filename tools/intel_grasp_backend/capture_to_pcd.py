#!/usr/bin/env python3
"""Export a frozen native H00-style optical cloud to PCD without refreshing time."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def export(capture, output):
    capture, output = Path(capture), Path(output)
    metadata_bytes = (capture/'capture.json').read_bytes()
    metadata = json.loads(metadata_bytes)
    cloud_bytes = (capture/'cloud-stride2.npz').read_bytes()
    with np.load(capture/'cloud-stride2.npz', allow_pickle=False) as data:
        points = np.asarray(data['points_optical_m'])
        rgb = np.asarray(data['rgb'])
    if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) > 1000000
            or rgb.shape != points.shape or rgb.dtype != np.uint8
            or not np.isfinite(points).all()):
        raise ValueError('need bounded finite original optical points with uint8 RGB')
    stamp_ns = metadata['stamp_ns']
    if not isinstance(stamp_ns, int) or stamp_ns <= 0 or not metadata['color_frame']:
        raise ValueError('original acquisition stamp/frame required')
    keep = (points[:, 2] > 0) & (points[:, 2] <= 1.2)
    points, rgb = points[keep], rgb[keep]
    if not len(points):
        raise ValueError('no recorded optical points within 1.2 m')
    packed = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgba', '<u4')])
    for axis, name in enumerate(('x', 'y', 'z')):
        packed[name] = points[:, axis]
    channels = rgb.astype(np.uint32)
    packed['rgba'] = (255 << 24) | (channels[:, 0] << 16) | (channels[:, 1] << 8) | channels[:, 2]
    header = (f'# Frozen native RGBD optical cloud; acquisition_ns={stamp_ns}\n'
              'VERSION .7\nFIELDS x y z rgba\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(packed.tobytes())
    report = {'schema': 'station-frozen-native-pcd-v1', 'qualified': False,
              'execution_enabled': False, 'target_association_verified': False,
              'source_capture': str(capture), 'source_capture_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
              'source_cloud_sha256': hashlib.sha256(cloud_bytes).hexdigest(),
              'pcd_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
              'acquisition_stamp_ns': stamp_ns, 'frame_id': metadata['color_frame'],
              'original_points': len(keep), 'exported_points': len(points), 'max_depth_m': 1.2,
              'source_model_sha256': metadata['model_sha256'],
              'pointcloud_provenance': metadata.get('pointcloud_provenance'),
              'live_timestamp_refresh': False}
    output.with_suffix('.provenance.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output_pcd', type=Path)
    args = parser.parse_args()
    print(json.dumps(export(args.capture, args.output_pcd), indent=2))
