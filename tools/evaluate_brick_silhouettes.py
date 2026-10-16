#!/usr/bin/env python3
"""Offline final-child contour evidence from existing synchronized ROS captures.

No camera/actuator access. Pixel annotations identify the eight stationary table
bricks in these development captures only; they are not runtime workspace gates.
"""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'ros2/src/so101_station_objects'))
from so101_station_objects import objects
from so101_station_objects.colored_object_validation import validate_brick_silhouette


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    original_split = objects._split_elongated_contours
    original_visible = objects._visible_component_geometry
    original_support = objects._component_geometry
    current = {}

    def split(contours):
        for contour in original_split(contours):
            current['contour'] = np.asarray(contour).copy()
            yield contour

    def trace(function):
        def wrapped(*values, **keywords):
            result = function(*values, **keywords)
            if result is not None:
                result['_diagnostic_final_contour'] = current['contour'].copy()
            return result
        return wrapped

    objects._split_elongated_contours = split
    objects._visible_component_geometry = trace(original_visible)
    objects._component_geometry = trace(original_support)
    rows = []
    try:
        for capture in args.capture:
            for directory in sorted(capture.glob('frame-*')):
                metadata = json.loads((directory/'info.json').read_text())
                transform = np.asarray(metadata['board_from_camera'])
                if transform.shape != (4,4):
                    continue
                rgb = cv2.cvtColor(cv2.imread(str(directory/'color.png')),cv2.COLOR_BGR2RGB)
                depth = np.load(directory/'depth_aligned.npy')
                k = metadata['k']
                report = objects.analyze_rgbd(rgb,depth,dict(fx=k[0],fy=k[4],ppx=k[2],ppy=k[5]),
                    metadata['d'],.001,transform)
                for item in report['objects']:
                    camera = np.linalg.inv(transform) @ np.append(item['surface_centroid'],1.)
                    pixel = camera[:2]/camera[2]*[k[0],k[4]]+[k[2],k[5]]
                    brick = bool(350 < pixel[0] < 575 and 290 < pixel[1] < 445)
                    contour = item['_diagnostic_final_contour']
                    validation = validate_brick_silhouette(contour)
                    rows.append(dict(capture=capture.name,frame=directory.name,stamp=metadata['stamp'],
                        color=item['color'],pixel=pixel.tolist(),manual_table_brick=brick,
                        contour=contour.reshape(-1,2).tolist(),accepted=validation.accepted,
                        reason=validation.reason,metrics=validation.metrics))
    finally:
        objects._split_elongated_contours = original_split
        objects._visible_component_geometry = original_visible
        objects._component_geometry = original_support
    summaries = {}
    for capture in args.capture:
        selected = [row for row in rows if row['capture']==capture.name]
        summaries[capture.name] = dict(real=sum(row['manual_table_brick'] for row in selected),
            real_accepted=sum(row['manual_table_brick'] and row['accepted']for row in selected),
            other=sum(not row['manual_table_brick']for row in selected),
            other_rejected=sum(not row['manual_table_brick']and not row['accepted']for row in selected))
    args.output.write_text(json.dumps({'qualification':'stationary development contours only; no semantic identity or held/occlusion qualification',
        'threshold':.45,'annotation':'eight visually annotated stationary table bricks; never runtime ROI',
        'summaries':summaries,'contours':rows},indent=2))
    print(json.dumps(summaries,sort_keys=True))


if __name__ == '__main__':
    main()
