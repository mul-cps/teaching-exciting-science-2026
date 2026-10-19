import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tests')]
from capture_object_diagnostic import save_capture,parameters_names
from replay_object_diagnostic import replay,analysis_arguments,jsonable
from test_ros2_objects import tabletop_scene


def test_exclusive_capture_readback_hashes_and_no_pickle(tmp_path):
    rgb,depth,*_=tabletop_scene();out=tmp_path/'capture'
    result=save_capture(out,rgb,depth,{'original':{'depth_stamp_ns':1}})
    assert len(result['rgbd_sha256'])==64
    value=json.loads((out/'capture.json').read_text());assert value['qualified'] is False
    with pytest.raises(FileExistsError):save_capture(out,rgb,depth,{})
    with np.load(out/'rgbd.npz',allow_pickle=False) as data:np.testing.assert_array_equal(data['rgb'],rgb)


def test_replay_keeps_all_rejections_and_restores_profile():
    rgb,depth,k,d,tf=tabletop_scene()
    # At least six independent too-small components; production samples cap5.
    for i in range(6):rgb[150:156,180+i*20:186+i*20]=[255,0,0];depth[150:156,180+i*20:186+i*20]=780
    previous=sys.getprofile()
    report,events,masks=replay(rgb,depth,dict(intrinsics=k,distortion=d,depth_scale_m=.001,board_from_camera=tf))
    assert sys.getprofile() is previous
    rejected=[e for e in events if e['stage']=='reject_component']
    assert len(rejected)>=6 and len(report['validation_diagnostics']['rejected_samples'])==5
    assert any('observed_mask' in e for e in events)
    assert set('raw_color_'+c for c in ('red','green','blue','yellow'))<=set(masks)
    assert len(report['objects'])==1


def test_profile_restored_on_analysis_failure():
    previous=sys.getprofile()
    with pytest.raises((TypeError,ValueError)):replay(np.zeros((2,2,3),np.uint8),np.zeros((2,2),np.uint16),{})
    assert sys.getprofile() is previous


def test_capture_static_bounds_no_authority_or_cloud_subscription():
    import ast
    source=(ROOT/'tools/capture_object_diagnostic.py').read_text();tree=ast.parse(source)
    assert 'args.timeout<=10' in source and 'deque(maxlen=4)' in source
    assert '/points' not in source and 'JointState' not in source and 'hardware_reset' not in source
    forbidden={'ProjectionWorker','ArmSession','connect','call_service','SetParameters','FollowJointTrajectory'}
    assert not any(isinstance(n,ast.Name) and n.id in forbidden for n in ast.walk(tree))
    assert 'executor.spin_once' in source and 'exist_ok=False' in source
    assert len(parameters_names())==len(set(parameters_names()))


def test_complete_saved_replay_cli_offline(tmp_path):
    import hashlib
    from replay_object_diagnostic import main
    from so101_station_objects import objects
    rgb,depth,k,d,tf=tabletop_scene()
    params=dict(depth_scale_m=.001,max_gripper_opening_m=.055,max_object_height_m=.025,max_observation_depth_m=1.2,observation_roi_enabled=False)
    for prefix in ('world_roi','pickup_roi','observation_roi'):
        params.update({f'{prefix}_{axis}_{edge}':v for (axis,edge),v in zip(((a,e) for a in 'xyz' for e in ('min','max')),objects.DEFAULT_TABLE_ROI)})
    metadata=dict(parameters=params,native_color_info={'k':[500.,0.,320.,0.,500.,240.,0.,0.,1.],'d':d.tolist()},camera_from_board=np.linalg.inv(tf).tolist(),original={'depth_stamp_ns':123},component_source_sha256={n:hashlib.sha256((ROOT/'ros2/src/so101_station_objects/so101_station_objects'/n).read_bytes()).hexdigest() for n in ('objects.py','colored_object_validation.py')})
    save_capture(tmp_path/'capture',rgb,depth,metadata)
    main([str(tmp_path/'capture'),'--output',str(tmp_path/'replay')])
    result=json.loads((tmp_path/'replay/replay.json').read_text())
    assert result['exact_report_parity'] and not result['qualified']
    assert result['all_component_events'] and len(result['report']['objects'])==1
