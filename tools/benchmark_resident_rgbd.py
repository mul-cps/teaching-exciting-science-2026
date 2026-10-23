#!/usr/bin/env python3
"""Frozen offline resident compute qualification with an independent OpenCV oracle.

Never opens a camera. Inputs must already be aligned and metric Z16 millimeters;
projection, morphology, shape/support/percentile/SVD admission are excluded.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np


def independent_oracle(rgb, raw, scale):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = (hsv[:, :, i] for i in range(3))
    vivid = (s >= 90) & (v >= 50)
    colors = [vivid & ((h <= 10) | (h >= 170)),
              vivid & (h >= 41) & (h <= 89),
              vivid & (h >= 90) & (h <= 140),
              vivid & (h >= 15) & (h <= 40)]
    masks = sum(mask.astype(np.uint32) << bit for bit, mask in enumerate(colors))
    depth = raw.astype(np.float32) * np.float32(scale)
    valid = np.isfinite(depth) & (depth >= np.float32(.10)) & (depth <= np.float32(1.2))
    candidate = (masks != 0) & valid
    cut = np.zeros(raw.shape, dtype=bool)
    for axis in (0, 1):
        left = (slice(None, -1), slice(None)) if axis == 0 else (slice(None), slice(None, -1))
        right = (slice(1, None), slice(None)) if axis == 0 else (slice(None), slice(1, None))
        bad = (candidate[left] & candidate[right] & (masks[left] == masks[right]) &
               (np.abs(depth[left] - depth[right]) > np.float32(.012)))
        cut[left] |= bad
        cut[right] |= bad
    candidate &= ~cut
    parents = np.zeros(raw.shape, dtype=np.uint32)
    flat_indices = np.arange(raw.size, dtype=np.uint32).reshape(raw.shape) + 1
    components = []
    for bit in range(4):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            (candidate & (masks == (1 << bit))).astype(np.uint8), connectivity=8)
        roots = np.full(count, np.iinfo(np.uint32).max, dtype=np.uint32)
        np.minimum.at(roots, labels.reshape(-1), flat_indices.reshape(-1))
        roots[0] = 0
        selected = roots[labels]
        parents[selected != 0] = selected[selected != 0]
        components.extend((int(roots[label]), int(stats[label, cv2.CC_STAT_AREA]))
                          for label in range(1, count)
                          if stats[label, cv2.CC_STAT_AREA] >= 24)
    return masks, depth, parents, cut, sorted(components)


def qualify_dump(path, rgb, raw, scale):
    values = np.fromfile(path, dtype='<u4')
    n = raw.size
    if values.size == n * 28:
        canonical_start, morphology_start = 17*n, 18*n
        layout = 'tile16_immutable_boundary_v1'
    elif values.size == n * 27:
        # Retained failed baseline is still reproducible with the same oracle.
        canonical_start, morphology_start = 6*n, 17*n
        layout = 'seven_stage_atomic_baseline'
    else:
        raise AssertionError('unexpected resident diagnostic layout')
    masks, depth, parents, cut, components = independent_oracle(rgb, raw, scale)
    arrays = {'masks': (values[:n], masks.reshape(-1)),
              'depth_bits': (values[n:2*n], depth.reshape(-1).view(np.uint32)),
              'canonical_membership': (values[canonical_start:canonical_start+n], parents.reshape(-1)),
              'depth_cut': (values[7*n:8*n], cut.reshape(-1).astype(np.uint32))}
    mismatches = {name: int(np.count_nonzero(actual != expected))
                  for name, (actual, expected) in arrays.items()}
    if any(mismatches.values()):
        raise AssertionError(mismatches)
    # Exercise the retained implementation itself, including its parent crop
    # and depth-split ordering, rather than accepting only a rewritten oracle.
    package = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_objects'
    sys.path.insert(0, str(package))
    from so101_station_objects.colored_object_validation import split_depth_coherent_components
    retained = []
    expected_morph = np.zeros((n, 9), dtype=np.uint32)
    expected_counts = np.zeros(n, dtype=np.uint32)
    for bit in range(4):
        count, labels = cv2.connectedComponents((masks == (1 << bit)).astype(np.uint8), connectivity=8)
        for label in range(1, count):
            for piece in split_depth_coherent_components(labels == label, depth):
                pixels = np.flatnonzero(piece.mask)
                retained.append((int(pixels.min()) + 1, int(pixels.size)))
                root = int(pixels.min()) + 1
                yy, xx = np.nonzero(piece.mask)
                x0, x1, y0, y1 = xx.min(), xx.max()+1, yy.min(), yy.max()+1
                closed = cv2.morphologyEx(piece.mask[y0:y1, x0:x1].astype(np.uint8),
                                          cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                my, mx = np.nonzero(closed)
                indices = (my+y0)*raw.shape[1]+mx+x0
                offsets = expected_counts[indices]
                if np.any(offsets >= 9):
                    raise AssertionError('morphology root list proof violated')
                expected_morph[indices, offsets] = root
                expected_counts[indices] += 1
                if not np.all(parents[piece.mask] == pixels.min() + 1):
                    raise AssertionError('retained depth splitter pixel membership differs')
    if sorted(retained) != components:
        raise AssertionError('retained depth-piece roots/areas differ')
    actual_morph = values[morphology_start:].reshape(n, 10)
    if not np.array_equal(actual_morph[:, 0], expected_counts):
        raise AssertionError('retained morphology per-pixel counts differ')
    actual_roots = actual_morph[:, 1:].copy()
    for k in range(9):
        actual_roots[expected_counts <= k, k] = 0
    if not np.array_equal(np.sort(actual_roots, axis=1), np.sort(expected_morph, axis=1)):
        raise AssertionError('retained per-piece morphology pixel membership differs')
    return {'resident_layout': layout,
            'independent_opencv_mismatches': mismatches,
            'retained_depth_splitter_components_exact': True,
            'retained_per_piece_closing_membership_exact': True,
            'retained_full_geometry_tested': False,
            'components_ge_24_pixels': len(components)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--warmup', type=int, default=10)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--backend', choices=('compare', 'vulkan', 'cpu'), default='compare')
    parser.add_argument('--amd-system-driver', action='store_true',
                        help='Process-only RADV/libdrm compatibility fix; never modifies system settings')
    args = parser.parse_args()
    color_path = args.fixture / 'color.png'
    depth_path = args.fixture / 'aligned_depth-raw.npy'
    metadata_path = args.fixture / 'capture.json'
    bgr = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('color.png required')
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    depth = np.load(depth_path, allow_pickle=False)
    if depth.dtype != np.uint16 or depth.shape != rgb.shape[:2]:
        raise ValueError('original aligned uint16 depth matching RGB required; no resize or unit inference')
    record = json.loads(metadata_path.read_text())
    info = record['messages']['color_info']['message']
    if any(info['d']):
        raise ValueError('nonzero distortion is outside resident deprojection qualification')
    units = record['messages']['aligned_depth']['metric_units']
    if 'millimeters' not in units:
        raise ValueError('fixture must explicitly declare millimeter Z16 units')
    k = info['k']
    height, width = depth.shape
    env = os.environ.copy()
    if args.amd_system_driver:
        env['VK_DRIVER_FILES'] = '/usr/share/vulkan/icd.d/radeon_icd.x86_64.json'
        env['LD_PRELOAD'] = '/usr/lib64/libdrm.so.2:/usr/lib64/libdrm_amdgpu.so.1'
    with tempfile.TemporaryDirectory(prefix='resident-rgbd-') as temp:
        temp = Path(temp)
        rgb.tofile(temp / 'rgb.bin')
        depth.astype('<u2', copy=False).tofile(temp / 'depth.bin')
        command = [str(args.binary), '--backend', args.backend,
                   '--rgb', str(temp / 'rgb.bin'), '--depth', str(temp / 'depth.bin'),
                   '--width', str(width), '--height', str(height),
                   '--frames', str(args.frames), '--warmup', str(args.warmup),
                   '--depth-scale', '.001', '--fx', str(k[0]), '--fy', str(k[4]),
                   '--cx', str(k[2]), '--cy', str(k[5]), '--output', str(temp / 'native.json')]
        if args.backend == 'compare':
            command.extend(['--dump', str(temp / 'scratch.bin')])
        start = time.monotonic()
        subprocess.run(command, env=env, check=True)
        result = json.loads((temp / 'native.json').read_text())
        result['wall_seconds'] = time.monotonic() - start
        if args.backend == 'compare':
            result.update(qualify_dump(temp / 'scratch.bin', rgb, depth, .001))
        result['fixture_sha256'] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (color_path, depth_path, metadata_path)}
        result['source_timestamp'] = record['messages']['aligned_depth']['header']
        result['projection_stage'] = 'prealigned recorded input; raw alignment not tested'
        result['calibration'] = info
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(args.output)


if __name__ == '__main__':
    main()
