"""Exact component pixels/support survive local buffering at global coordinates."""
import ast
import inspect
import subprocess
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects.colored_object_validation import (
    split_depth_coherent_components, validate_colored_component)


def scene(kind):
    rgb = np.full((300, 400, 3), 100, np.uint8)
    depth = np.ones((300, 400), np.float32)
    parent = np.zeros(depth.shape, bool)
    parent[120:170, 160:230] = True
    depth[parent] = .5
    if kind == 'hole': depth[135:145, 180:190] = 0
    if kind == 'nan': depth[135:145, 180:190] = np.nan
    if kind == 'jump': depth[120:170, 200:230] = .9
    if kind == 'out_of_range': depth[135:145, 180:190] = 2.
    if kind == 'invalid_bridge':
        parent[30:80, 30:300] = True
        parent[79:121, 190:192] = True
        depth[30:120, 30:300] = 0
    if kind == 'boundary':
        parent[:25, :30] = True
        depth[:25, :30] = .5
    rgb[parent] = [0, 0, 255]
    return rgb, depth, parent


def expand(piece, shape):
    x, y = piece.origin_xy
    out = np.zeros(shape, bool)
    out[y:y+piece.mask.shape[0], x:x+piece.mask.shape[1]] = piece.mask
    support = np.zeros(shape, bool)
    support[y:y+piece.mask.shape[0], x:x+piece.mask.shape[1]] = piece.support_mask
    return out, support


@pytest.mark.parametrize('kind', ['plain', 'hole', 'nan', 'jump', 'out_of_range', 'invalid_bridge', 'boundary'])
def test_local_buffers_preserve_legacy_pixels_support_and_validation(kind):
    rgb, depth, parent = scene(kind)
    legacy = split_depth_coherent_components(parent, depth)
    # Exact parent bounding box supplied by connected-component metadata in the analyzer.
    yy, xx = np.nonzero(parent)
    x, y, right, bottom = int(xx.min()), int(yy.min()), int(xx.max())+1, int(yy.max())+1
    crop = (slice(y, bottom), slice(x, right))
    local = split_depth_coherent_components(parent[crop], depth[crop],
        local_output=True, pixel_origin=(x, y))
    assert len(local) == len(legacy)
    intr = dict(fx=300., fy=301., ppx=199.2, ppy=149.1)
    for a, b in zip(legacy, local):
        pixels, support = expand(b, parent.shape)
        np.testing.assert_array_equal(pixels, a.mask)
        np.testing.assert_array_equal(support, a.support_mask)
        assert (b.parent_pixels, b.depth_cut_pixels) == (a.parent_pixels, a.depth_cut_pixels)
        assert not b.mask.flags.writeable and not b.support_mask.flags.writeable
        assert b.mask.size <= a.mask.size
        bx, by = b.origin_xy
        window = (slice(by, by+b.mask.shape[0]), slice(bx, bx+b.mask.shape[1]))
        shifted = dict(intr, ppx=intr['ppx']-bx, ppy=intr['ppy']-by)
        old = validate_colored_component(rgb, depth, a.mask, intr,
            [.02, -.01, .001, 0, 0], support_mask=a.support_mask)
        new = validate_colored_component(rgb[window], depth[window], b.mask, shifted,
            [.02, -.01, .001, 0, 0], support_mask=b.support_mask)
        assert (old.accepted, old.reason) == (new.accepted, new.reason)
        assert old.metrics.keys() == new.metrics.keys()
        for key, value in old.metrics.items():
            if isinstance(value, (str, type(None))): assert new.metrics[key] == value
            else: np.testing.assert_allclose(new.metrics[key], value, rtol=0, atol=1e-12)


@pytest.mark.parametrize('origin', [(True, 0), (-1, 0), (0., 0), (0,), '0,0'])
def test_local_origin_requires_two_nonnegative_exact_integer_coordinates(origin):
    _, depth, parent = scene('plain')
    with pytest.raises(ValueError):
        split_depth_coherent_components(parent, depth, local_output=True, pixel_origin=origin)


def test_analyzer_splits_exact_local_parent_views_not_full_frame(monkeypatch):
    from so101_station_objects import objects
    from test_ros2_objects import tabletop_scene
    rgb, depth, intrinsics, distortion, transform = tabletop_scene()
    original = objects.split_depth_coherent_components
    seen = []
    def split(mask, metric, **kwargs):
        assert kwargs.get('local_output') is True
        assert mask.shape == metric.shape and mask.shape != depth.shape
        assert kwargs['pixel_origin'] == (305, 220)
        seen.append(mask.shape)
        return original(mask, metric, **kwargs)
    monkeypatch.setattr(objects, 'split_depth_coherent_components', split)
    report = objects.analyze_rgbd(rgb, depth, intrinsics, distortion, .001, transform)
    assert len(report['objects']) == 1 and seen == [(30, 30)]


def test_full_synthetic_analyzer_matches_unmodified_baseline():
    from tools.benchmark_component_locality import baseline_analyzer, assert_same
    from so101_station_objects.objects import analyze_rgbd
    from test_ros2_objects import tabletop_scene
    try:
        original = baseline_analyzer()
    except subprocess.CalledProcessError:
        pytest.skip('immutable legacy baseline522a658 unavailable in this Git history')
    for fault in ('none', 'holes', 'touching'):
        rgb, depth, intrinsics, distortion, transform = tabletop_scene()
        if fault == 'holes': depth[225:230, 310:320] = 0
        if fault == 'touching':
            rgb[220:250, 335:365] = [255, 0, 0]
            depth[220:250, 335:365] = 770
        before = original(rgb, depth, intrinsics, distortion, .001, transform)
        after = analyze_rgbd(rgb, depth, intrinsics, distortion, .001, transform)
        assert_same(before, after)


def test_analyzer_component_loop_uses_only_local_mask_and_depth_views():
    from so101_station_objects.objects import analyze_rgbd
    tree = ast.parse(inspect.getsource(analyze_rgbd))
    parent = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                  any(isinstance(t, ast.Name) and t.id == 'parent_mask' for t in n.targets))
    assert ast.unparse(parent.value) == 'labels[crop] == label'
    split = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and
                 isinstance(n.func, ast.Name) and n.func.id == 'split_depth_coherent_components')
    assert ast.unparse(split.args[1]) == 'depth_m[crop]'
    assert any(k.arg == 'local_output' and isinstance(k.value, ast.Constant) and k.value.value is True for k in split.keywords)
    validate = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and
                    isinstance(n.func, ast.Name) and n.func.id == 'validate_colored_component')
    assert [ast.unparse(v) for v in validate.args[:2]] == ['rgb[piece_crop]', 'depth_m[piece_crop]']


def test_benchmark_comparator_accepts_only_added_default_cpu_metadata():
    from tools.benchmark_component_locality import assert_same
    before = {'objects': [{'center': [0.1, 0.2, 0.3]}]}
    metadata = {'requested': 'cpu', 'selected': 'cpu', 'gpu_available': None,
                'device': None, 'fallback_reason': None}
    assert_same(before, dict(before, color_mask_backend=metadata))
    assert_same(before, before)
    with pytest.raises(AssertionError):
        assert_same(before, dict(before, unknown=True, color_mask_backend=metadata))
    with pytest.raises(AssertionError):
        assert_same(before, {'objects': [{'center': [0.1, 0.2, 0.4]}],
                             'color_mask_backend': metadata})
    with pytest.raises(AssertionError):
        assert_same({'nested': before},
                    {'nested': dict(before, color_mask_backend=metadata)})
    for changed in (dict(metadata, selected='vulkan'),
                    dict(metadata, fallback_reason='failed')):
        with pytest.raises(AssertionError):
            assert_same(before, dict(before, color_mask_backend=changed))
        # Same-key reports retain strict recursive comparison.
        with pytest.raises(AssertionError):
            assert_same(dict(before, color_mask_backend=metadata),
                        dict(before, color_mask_backend=changed))
