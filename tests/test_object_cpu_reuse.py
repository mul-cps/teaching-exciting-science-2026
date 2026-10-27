"""CPU reuse preserves full-workspace objects/ghost rejection and parent labels."""
import ast
import inspect
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'ros2/src/so101_station_objects')]
from so101_station_objects import objects,colored_object_validation as validation
from test_ros2_objects import tabletop_scene


def test_support_and_detection_share_exact_color_masks_once(monkeypatch):
    from so101_station_objects import color_mask_backend
    rgb,depth,k,d,transform=tabletop_scene();original=color_mask_backend.color_masks;calls=[]
    def masks(value):calls.append(value);return original(value)
    monkeypatch.setattr(color_mask_backend,'color_masks',masks)
    report=objects.analyze_rgbd(rgb,depth,k,d,.001,transform)
    assert len(report['objects'])==1 and len(calls)==1
    assert calls[0] is rgb


@pytest.mark.parametrize('shape',[(1,1),(1,9),(9,1),(3,3),(7,9),(16,17),(31,32),(64,65)])
def test_grana_parent_labels_stats_and_centroids_equal_legacy(shape):
    rng=np.random.default_rng(8060+sum(shape))
    for _ in range(200):
        mask=(rng.random(shape)<rng.uniform(.01,.95)).astype(np.uint8)
        old=cv2.connectedComponentsWithStats(mask,connectivity=8)
        new=cv2.connectedComponentsWithStatsWithAlgorithm(mask,8,cv2.CV_32S,cv2.CCL_GRANA)
        for a,b in zip(old,new):np.testing.assert_array_equal(a,b)


def test_analyzer_parent_ccl_is_native_grana_with_unchanged_eight_connectivity():
    source=inspect.getsource(objects.analyze_rgbd)
    assert 'connectedComponentsWithStatsWithAlgorithm(mask, 8, cv2.CV_32S, cv2.CCL_GRANA)' in source
    assert 'setNumThreads' not in source and 'thread' not in source.lower()


def test_same_array_extents_quantiles_batched_without_altered_method():
    for function in (objects._visible_component_geometry,validation.validate_colored_component):
        source=inspect.getsource(function)
        tree=ast.parse(source)
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and ast.unparse(n.func)=='linear_percentile'
               and ast.unparse(n.args[0])=='principal']
        assert len(calls)==1 and ast.literal_eval(calls[0].args[1])==[5,95]
        assert [(keyword.arg, ast.literal_eval(keyword.value))
                for keyword in calls[0].keywords] == [('axis', 0)]


@pytest.mark.parametrize('change',['normal','outside_pickup','boundary','hole','depth_jump'])
def test_exact_synthetic_report_matches_pinned_baseline(change):
    from tools.benchmark_component_locality import baseline_analyzer,assert_same
    # Historical CPU-reuse evidence used b56ff993d21d838c2a07eda898a4933ff946f629.
    # Current exact-output regression uses the accepted canonical-policy epoch.
    old=baseline_analyzer()
    rgb,depth,k,d,t=tabletop_scene()
    if change=='outside_pickup':
        rgb[220:250,305:335]=128;depth[220:250,305:335]=800
        rgb[220:250,20:50]=[255,0,0];depth[220:250,20:50]=780
    if change=='boundary':rgb[:30,:30]=[0,255,0];depth[:30,:30]=780
    if change=='hole':depth[230:240,315:325]=0
    if change=='depth_jump':depth[220:250,320:335]=500
    assert_same(old(rgb,depth,k,d,.001,t),objects.analyze_rgbd(rgb,depth,k,d,.001,t))
