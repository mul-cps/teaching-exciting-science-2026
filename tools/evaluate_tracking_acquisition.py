#!/usr/bin/env python3
"""Offline tracker probation replay of an existing ROS status/RGBD capture.

No devices or ROS nodes are opened. Status track labels and image pixel labels
are development annotations only; neither affects runtime tracker admission.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects.objects import StableObjectTracker, analyze_rgbd


def replay_statuses(capture, real_ids):
    statuses = json.loads((capture / 'statuses.json').read_text())
    state = StableObjectTracker(acquisition_min_frames=3)
    rows = []
    source_real = source_extra = 0
    for status in statuses:
        objects = []
        for item in status.get('objects', []):
            real = item['track_id'] in real_ids
            source_real += int(real)
            source_extra += int(not real)
            objects.append({'color':item['color'],
                            'position':np.asarray(item['position_world']),
                            '_annotated_real':real, **{key:item[key]for key in ('acquisition_eligible','acquisition_eligibility_reason','image_boundary_sides')if key in item}})
        records = state.update(objects, status['stamp'])
        rows.append({'stamp':status['stamp'],
                     'real':sum(item['_annotated_real'] for item in records),
                     'extra':sum(not item['_annotated_real'] for item in records),
                     'acquisition':state.acquisition_diagnostics()})
    return {'source_reports':len(statuses), 'source_real':source_real,
            'source_extra':source_extra, 'published_real':sum(row['real'] for row in rows),
            'published_extra':sum(row['extra'] for row in rows), 'reports':rows}


def replay_rgbd(capture, annotation_x_offset_px=0):
    state = StableObjectTracker(acquisition_min_frames=3)
    rows = []
    durations = []
    for directory in sorted(capture.glob('frame-*')):
        metadata = json.loads((directory/'info.json').read_text())
        transform = np.asarray(metadata['board_from_camera'])
        if transform.shape != (4,4):
            continue
        rgb = cv2.cvtColor(cv2.imread(str(directory/'color.png')),cv2.COLOR_BGR2RGB)
        depth = np.load(directory/'depth_aligned.npy')
        k = metadata['k']
        started = time.perf_counter()
        report = analyze_rgbd(rgb,depth,dict(fx=k[0],fy=k[4],ppx=k[2],ppy=k[5]),
                              metadata['d'],.001,transform)
        for item in report['objects']:
            camera = np.linalg.inv(transform) @ np.append(item['surface_centroid'],1.)
            pixel = camera[:2]/camera[2]*[k[0],k[4]]+[k[2],k[5]]
            item['_annotated_real'] = bool(350 + annotation_x_offset_px < pixel[0] < 575 + annotation_x_offset_px and 290 < pixel[1] < 445)
        records = state.update(report['objects'], metadata['stamp'])
        durations.append(time.perf_counter()-started)
        rows.append({'frame':directory.name,'stamp':metadata['stamp'],
                     'detected_real':sum(item['_annotated_real'] for item in report['objects']),
                     'detected_extra':sum(not item['_annotated_real'] for item in report['objects']),
                     'acquisition_eligible_real':sum(item['_annotated_real'] and item['acquisition_eligible'] for item in report['objects']),
                     'ineligible_extra':sum(not item['_annotated_real'] and not item['acquisition_eligible'] for item in report['objects']),
                     'published_real':sum(item['_annotated_real'] for item in records),
                     'published_extra':sum(not item['_annotated_real'] for item in records)})
    return {'usable_frames':len(rows),
            **{key:sum(row[key]for row in rows)for key in ('detected_real','detected_extra','acquisition_eligible_real','ineligible_extra','published_real','published_extra')},
            'analyzer_and_tracker_s':{'median':float(np.median(durations)),
                                      'p95':float(np.quantile(durations,.95)),
                                      'max':max(durations)},
            'frames':rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--real-track-ids', type=int, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--annotation-x-offset-px', type=int, default=0,
                        help='Offline table-brick annotation translation for changed image crop only')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    result = {'qualification':'offline development evidence; temporal persistence does not prove brick identity',
              'acquisition_min_frames':3,
              'annotation':'source real IDs explicitly supplied; RGBD eight stationary table bricks annotated in pixel space only',
              'status_replay':replay_statuses(args.capture,set(args.real_track_ids)),
              'rgbd_replay':replay_rgbd(args.capture,args.annotation_x_offset_px)}
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({key:{k:v for k,v in value.items() if k not in {'reports','frames'}}
                      for key,value in result.items() if isinstance(value,dict)},sort_keys=True))


if __name__ == '__main__':
    main()
