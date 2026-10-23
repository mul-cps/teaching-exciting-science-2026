"""Independent CPU oracle guards for the qualified aligned-input boundary."""
import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('resident_oracle', ROOT / 'tools/benchmark_resident_rgbd.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_depth_jump_cuts_both_sides_before_connectivity():
    rgb = np.zeros((8, 16, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    depth = np.full((8, 16), 500, dtype=np.uint16)
    depth[:, 8:] = 520
    masks, meters, labels, cut, components = module.independent_oracle(rgb, depth, .001)
    assert np.all(masks == 1)
    assert cut[:, 7:9].all()
    assert not cut[:, :7].any()
    assert len(components) == 2
    assert np.all(labels[:, 7:9] == 0)
    assert sorted(area for _, area in components) == [56, 56]


def test_invalid_depth_and_different_colors_are_separate():
    rgb = np.zeros((12, 12, 3), dtype=np.uint8)
    rgb[:, :6, 0] = 255
    rgb[:, 6:, 1] = 255
    depth = np.full((12, 12), 500, dtype=np.uint16)
    depth[2, 2] = 0
    _, _, labels, cut, components = module.independent_oracle(rgb, depth, .001)
    assert labels[2, 2] == 0
    assert not cut.any()
    assert sorted(area for _, area in components) == [71, 72]


def test_diagonal_connectivity_is_eight_connected():
    rgb = np.zeros((20, 20, 3), dtype=np.uint8)
    indices = np.arange(20)
    rgb[indices, indices, 2] = 255
    depth = np.full((20, 20), 500, dtype=np.uint16)
    _, _, labels, _, components = module.independent_oracle(rgb, depth, .001)
    assert len(np.unique(labels[labels > 0])) == 1
    assert components == []  # min_pixels remains 24


def test_dump_compares_retained_splitter_and_rejects_one_pixel(tmp_path):
    import pytest
    rgb = np.zeros((8, 16, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    raw = np.full((8, 16), 500, dtype=np.uint16)
    raw[:, 8:] = 520
    masks, depth, parents, cut, _ = module.independent_oracle(rgb, raw, .001)
    n = raw.size
    scratch = np.zeros(28*n, dtype='<u4')
    scratch[:n] = masks.reshape(-1)
    scratch[n:2*n] = depth.reshape(-1).view(np.uint32)
    scratch[17*n:18*n] = parents.reshape(-1)
    scratch[7*n:8*n] = cut.reshape(-1)
    morphology = scratch[18*n:].reshape(n, 10)
    morphology[:, 0] = (parents.reshape(-1) != 0)
    morphology[:, 1] = parents.reshape(-1)
    path = tmp_path / 'scratch.bin'
    scratch.tofile(path)
    result = module.qualify_dump(path, rgb, raw, .001)
    assert result['retained_depth_splitter_components_exact'] is True
    scratch[17*n] = 999
    scratch.tofile(path)
    with pytest.raises(AssertionError, match='canonical_membership'):
        module.qualify_dump(path, rgb, raw, .001)


def test_independent_closing_memberships_have_nine_source_candidates():
    """Center erosion includes center dilation, even at a tight bbox border."""
    import cv2
    rng = np.random.default_rng(1012026)
    for _ in range(32):
        source = rng.integers(0, 13, size=(12, 16), dtype=np.uint32)
        selected_count = np.zeros(source.shape, dtype=np.uint8)
        for root in range(1, 13):
            ys, xs = np.nonzero(source == root)
            if not len(xs):
                continue
            x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
            piece = (source[y0:y1+1, x0:x1+1] == root).astype(np.uint8)
            closed = cv2.morphologyEx(piece, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            for py, px in zip(*np.nonzero(closed)):
                y, x = y0 + py, x0 + px
                candidates = set(source[max(0, y-1):y+2, max(0, x-1):x+2].flat) - {0}
                assert root in candidates
                assert len(candidates) <= 9
                selected_count[y, x] += 1
        assert selected_count.max() <= 9
