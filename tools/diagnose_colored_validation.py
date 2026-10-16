#!/usr/bin/env python3
"""Replay timestamp-matched ROS artifacts through full component geometry.

No hardware access. Brick pixel annotations below describe this development
capture only; they are never part of runtime validation. Per-piece replay uses
one measured support plane for baseline and validated geometry, preserving the
existing contour/same-color splitting and geometry implementation.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import types

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'ros2/src/so101_station_objects'))
from so101_station_objects.colored_object_validation import split_depth_coherent_components, validate_colored_component


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = subprocess.check_output(['git', 'show', args.baseline_ref+':ros2/src/so101_station_objects/so101_station_objects/objects.py'], cwd=ROOT)
    module = types.ModuleType('colored_validation_geometry_baseline')
    module.__file__ = str(ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py')
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    original_masks = module.color_masks
    frames, details = [], []
    def annotation(obj, transform, k):
        camera = np.linalg.inv(transform) @ np.append(obj['surface_centroid'], 1.)
        pixel = camera[:2]/camera[2]*[k[0], k[4]]+[k[2], k[5]]
        return bool(350 < pixel[0] < 575 and 290 < pixel[1] < 445), pixel.tolist()
    for directory in sorted(args.capture.glob('frame-*')):
        metadata = json.loads((directory/'info.json').read_text())
        transform = np.asarray(metadata['board_from_camera'])
        if transform.shape != (4, 4):
            continue
        rgb = cv2.cvtColor(cv2.imread(str(directory/'color.png')), cv2.COLOR_BGR2RGB)
        depth = np.load(directory/'depth_aligned.npy')
        z = depth.astype(np.float32)*.001
        k = metadata['k']
        intrinsics = dict(fx=k[0], fy=k[4], ppx=k[2], ppy=k[5])
        base = module.analyze_rgbd(rgb, depth, intrinsics, metadata['d'], .001, transform)
        result_objects = []
        accepted, rejected, reasons = 0, 0, Counter()
        try:
            for color, mask in original_masks(rgb).items():
                count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
                for label in range(1, count):
                    if stats[label, cv2.CC_STAT_AREA] < 30:
                        continue
                    for component in split_depth_coherent_components(labels == label, z):
                        validation = validate_colored_component(rgb, z, component.mask, intrinsics,
                            metadata['d'], support_mask=component.support_mask)
                        reasons[validation.reason] += 1
                        if not validation.accepted:
                            rejected += 1
                            continue
                        accepted += 1
                        masks = {name: np.zeros_like(mask) for name in original_masks(rgb)}
                        masks[color] = component.mask
                        module.color_masks = lambda image, masks=masks: masks
                        report = module.analyze_rgbd(rgb, depth, intrinsics, metadata['d'], .001,
                            transform, support_plane=base['support_plane'])
                        result_objects.extend(report['objects'])
        finally:
            module.color_masks = original_masks
        row = {'frame': directory.name, 'stamp': metadata['stamp'], 'coherent_accepted': accepted,
               'coherent_rejected': rejected, 'validation_reasons': dict(reasons)}
        for name, objects in [('baseline', base['objects']), ('validated', result_objects)]:
            labeled = [(obj, *annotation(obj, transform, k)) for obj in objects]
            row[name] = {'objects': len(objects), 'hand_labeled_table_bricks': sum(label for _, label, _ in labeled),
                         'other_blue_surfaces': sum(obj['color'] == 'blue' and not label for obj, label, _ in labeled)}
            details.extend({'frame': directory.name, 'mode': name, 'color': obj['color'], 'handlabel_brick': label,
                'pixel': pixel, 'centroid_world': np.asarray(obj['surface_centroid']).tolist(),
                'extent_m': np.asarray(obj['dimensions_m']).tolist()} for obj, label, pixel in labeled)
        frames.append(row)
    output = {'baseline_ref': args.baseline_ref, 'depth_support_threshold': .90,
        'qualification': 'development capture only; no held-out or physical semantic qualification',
        'annotation': 'eight visible table bricks in this stationary capture; offline pixel labels only',
        'frames': frames, 'objects': details}
    args.output.write_text(json.dumps(output, indent=2))
    print(json.dumps({'frames': len(frames), 'before_bricks': sum(f['baseline']['hand_labeled_table_bricks'] for f in frames),
        'after_bricks': sum(f['validated']['hand_labeled_table_bricks'] for f in frames),
        'before_other_blue': sum(f['baseline']['other_blue_surfaces'] for f in frames),
        'after_other_blue': sum(f['validated']['other_blue_surfaces'] for f in frames)}, sort_keys=True))


if __name__ == '__main__':
    main()
