"""Offline exact-source replay with complete component diagnostics, no ROS calls."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2/src/so101_station_objects')]
from so101_station_objects import objects


def jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    return value


def analysis_arguments(metadata):
    p=metadata['parameters'];ci=metadata['native_color_info'];k=ci['k']
    roi=lambda prefix:tuple(p[f'{prefix}_{axis}_{edge}'] for axis in 'xyz' for edge in ('min','max'))
    return dict(intrinsics=dict(fx=k[0],fy=k[4],ppx=k[2],ppy=k[5]),distortion=ci['d'],depth_scale_m=p['depth_scale_m'],
                board_from_camera=np.linalg.inv(metadata['camera_from_board']),world_roi=roi('world_roi'),pickup_roi=roi('pickup_roi'),
                observation_roi=roi('observation_roi') if p['observation_roi_enabled'] else None,
                max_opening_m=p['max_gripper_opening_m'],max_object_height_m=p['max_object_height_m'],
                max_observation_depth_m=p['max_observation_depth_m'])


def replay(rgb,depth,arguments):
    events=[];masks={};original_profile=sys.getprofile()
    names={'validate_colored_component','validate_brick_silhouette','validate_child_visibility','reject_component'}
    package=str(ROOT/'ros2/src/so101_station_objects/so101_station_objects')
    def observe(frame,event,arg):
        if event!='return' or frame.f_code.co_name not in names or not frame.f_code.co_filename.startswith(package):return
        if len(events)>=10000:raise ValueError('diagnostic exceeds10000 component events')
        local=frame.f_locals;parent=frame.f_back.f_locals if frame.f_back else {};record={'stage':frame.f_code.co_name,'index':len(events)}
        if hasattr(arg,'accepted'):record.update(accepted=arg.accepted,reason=arg.reason,metrics=jsonable(arg.metrics))
        if frame.f_code.co_name=='reject_component':record.update({k:jsonable(local[k]) for k in ('color','reason','parent_pixels','component_pixels','metrics') if k in local})
        for key in ('offset_x','offset_y','x0','y0','color','label','area'):
            value=parent.get(key)
            if isinstance(value,(str,int,np.integer)):record[key]=jsonable(value)
        for key in ('component_mask','support_mask','observed_mask'):
            value=local.get(key)
            if isinstance(value,np.ndarray):
                array_key=f'event_{len(events)}_{key}';masks[array_key]=value.astype(np.uint8,copy=True);record[key]=array_key
                yy,xx=np.nonzero(value);record[key+'_bbox_local_xywh']=None if not len(xx) else [int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)]
        if frame.f_code.co_name=='reject_component' and all(k in parent for k in ('x','y','width','height')):
            record['parent_global_bbox_xywh']=[int(parent[k]) for k in ('x','y','width','height')]
        events.append(record)
    try:
        sys.setprofile(observe);report=objects.analyze_rgbd(rgb,depth,**arguments)
    finally:sys.setprofile(original_profile)
    for color,mask in objects.color_masks(rgb).items():
        masks['raw_color_'+color]=mask.astype(np.uint8)
        count,labels,stats,_=objects.cv2.connectedComponentsWithStatsWithAlgorithm(mask.astype(np.uint8),8,objects.cv2.CV_32S,objects.cv2.CCL_GRANA)
        for label in range(1,count):
            if len(events)>=10000:raise ValueError('diagnostic exceeds10000 component events')
            x,y,width,height,area=map(int,stats[label]);events.append(dict(stage='raw_color_parent',color=color,label=label,pixel_bbox_xywh=[x,y,width,height],pixels=area,analysis_gate='below_min_component_pixels30' if area<30 else 'considered_by_analyzer'))
    # Instrumentation must not change a single report value or admission.
    ordinary=objects.analyze_rgbd(rgb,depth,**arguments)
    assert json.dumps(jsonable(report),sort_keys=True,allow_nan=False)==json.dumps(jsonable(ordinary),sort_keys=True,allow_nan=False)
    return report,events,masks


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');p.add_argument('--output',required=True);args=p.parse_args(argv)
    source=Path(args.capture);metadata=json.loads((source/'capture.json').read_bytes())
    require_hash=hashlib.sha256((source/'rgbd.npz').read_bytes()).hexdigest();assert require_hash==metadata['rgbd_sha256']
    for name,digest in metadata['component_source_sha256'].items():
        assert hashlib.sha256((ROOT/'ros2/src/so101_station_objects/so101_station_objects'/name).read_bytes()).hexdigest()==digest,'component source changed; use pinned checkout'
    with np.load(source/'rgbd.npz',allow_pickle=False) as saved:rgb=saved['rgb'];depth=saved['depth']
    report,events,masks=replay(rgb,depth,analysis_arguments(metadata))
    output=Path(args.output);output.mkdir(exist_ok=False,parents=True)
    with (output/'component-masks.npz').open('xb') as stream:np.savez_compressed(stream,**masks)
    result=dict(schema='object-component-replay-v1',qualified=False,execution_enabled=False,
                source_capture_sha256=hashlib.sha256((source/'capture.json').read_bytes()).hexdigest(),
                original=metadata['original'],report=jsonable(report),all_component_events=events,
                semantics='raw single-acquisition analysis; not live tracker, cached-support, attachment or physical qualification',
                exact_report_parity=True,masks_sha256=hashlib.sha256((output/'component-masks.npz').read_bytes()).hexdigest())
    with (output/'replay.json').open('x') as stream:json.dump(result,stream,sort_keys=True,allow_nan=False)
    print(json.dumps({'objects':len(report['objects']),'component_events':len(events),'output':str(output),'qualified':False}))


if __name__=='__main__':main()
