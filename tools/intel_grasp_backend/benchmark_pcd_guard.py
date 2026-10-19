#!/usr/bin/env python3
"""Offline maximum supported PCD finite-scan admission benchmark, no engine/DDS."""
import argparse
import json
from pathlib import Path
import statistics
import tempfile
import time
try:
    from .pcd_guard import checked_pcd
except ImportError:
    from pcd_guard import checked_pcd


def benchmark(samples=10):
    if not 3 <= samples <= 100:
        raise ValueError('bounded 3..100 benchmark samples required')
    header = ('VERSION .7\nFIELDS x y z rgba\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
              'WIDTH 1000000\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1000000\nDATA binary\n')
    with tempfile.TemporaryDirectory(prefix='offline-pcd-guard-') as directory:
        path = Path(directory)/'maximum.pcd'
        path.write_bytes(header.encode()+bytes(16_000_000))
        checked_pcd(path)
        values = []
        for _ in range(samples):
            start = time.perf_counter_ns()
            assert checked_pcd(path)['points'] == 1_000_000
            values.append((time.perf_counter_ns()-start)/1e6)
    return {'schema': 'offline-pcd-admission-benchmark-v1', 'points': 1_000_000,
            'samples_ms': values, 'mean_ms': statistics.mean(values),
            'maximum_ms': max(values), 'finite_scan_bool_temporary_bytes_max': 3_000_000,
            'scope': 'bounded file read/header/zero-copy XYZ view/vector finite scan; excludes SHA/engine/DDS',
            'qualified': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = benchmark()
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))
