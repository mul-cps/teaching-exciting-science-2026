"""CPU model of immutable tile/boundary phases; this is not GPU evidence."""
from pathlib import Path
import importlib.util

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('resident_oracle', ROOT / 'tools/benchmark_resident_rgbd.py')
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)


def canonical_forest(forest):
    """A is frozen for an entire walk phase; no path-splitting write occurs."""
    result = forest.copy()
    active = result != 0
    indices = np.arange(1, result.size + 1, dtype=np.uint32)
    assert np.all(forest <= indices)
    for _ in range(forest.size):
        next_result = np.where(active, forest[np.maximum(result, 1)-1], 0)
        if np.array_equal(next_result, result):
            return result
        assert np.all(next_result <= result)
        result = next_result
    raise AssertionError('invalid/nonmonotone forest traversal')


def tiled_candidate(colors, candidate, rounds=32):
    height, width = colors.shape
    yy, xx = np.indices(colors.shape)
    tiles = (yy//16)*((width+15)//16)+xx//16
    labels = np.where(candidate, np.arange(colors.size, dtype=np.uint32).reshape(colors.shape)+1, 0)
    rounds_used = 0
    # Immutable ping-pong source/destination labels; a tile has <=256vertices.
    for iteration in range(256):
        padded_labels = np.pad(labels, 1)
        padded_colors = np.pad(colors, 1)
        padded_tiles = np.pad(tiles, 1, constant_values=-1)
        following = labels.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                neighbor = padded_labels[1+dy:height+1+dy, 1+dx:width+1+dx]
                color = padded_colors[1+dy:height+1+dy, 1+dx:width+1+dx]
                tile = padded_tiles[1+dy:height+1+dy, 1+dx:width+1+dx]
                valid = candidate & (neighbor != 0) & (color == colors) & (tile == tiles)
                following[valid] = np.minimum(following[valid], neighbor[valid])
        rounds_used = iteration+1
        if np.array_equal(following, labels):
            break
        labels = following
    else:
        raise AssertionError('256tile propagation rounds must cover diameter<=255')
    forest = labels.reshape(-1).copy()
    # Visit every original undirected edge once, including diagonal corner edges.
    all_left, all_right, boundary_left, boundary_right = [], [], [], []
    pixels = np.arange(colors.size).reshape(colors.shape)
    for dy, dx in ((0, 1), (1, -1), (1, 0), (1, 1)):
        y0, y1 = 0, height-dy
        x0, x1 = max(0, -dx), min(width, width-dx)
        a = (slice(y0, y1), slice(x0, x1))
        b = (slice(y0+dy, y1+dy), slice(x0+dx, x1+dx))
        edge = candidate[a] & candidate[b] & (colors[a] == colors[b])
        all_left.extend(pixels[a][edge])
        all_right.extend(pixels[b][edge])
        cross = edge & (tiles[a] != tiles[b])
        boundary_left.extend(pixels[a][cross])
        boundary_right.extend(pixels[b][cross])
    a, b = np.asarray(boundary_left, dtype=int), np.asarray(boundary_right, dtype=int)
    for _ in range(rounds):
        roots = canonical_forest(forest)  # B writes complete before A is hooked
        left, right = roots[a], roots[b]
        higher, lower = np.maximum(left, right), np.minimum(left, right)
        np.minimum.at(forest, higher-1, lower)  # models commutative atomic min
    result = canonical_forest(forest)
    fault = int(np.any(result[np.asarray(all_left, dtype=int)] != result[np.asarray(all_right, dtype=int)]))
    return result.reshape(colors.shape), fault, rounds_used


def independent_labels(colors, candidate):
    expected = np.zeros(colors.shape, np.uint32)
    indices = np.arange(colors.size, dtype=np.uint32).reshape(colors.shape)+1
    for color in np.unique(colors):
        if not color:
            continue
        count, labels = cv2.connectedComponents((candidate & (colors == color)).astype(np.uint8), connectivity=8)
        roots = np.full(count, np.iinfo(np.uint32).max, np.uint32)
        np.minimum.at(roots, labels.reshape(-1), indices.reshape(-1))
        roots[0] = 0
        values = roots[labels]
        expected[values != 0] = values[values != 0]
    return expected


@pytest.mark.parametrize('name', ['holes', 'rings', 'diagonal', 'depthsplit', 'overlap_checker', 'long_snake'])
def test_tile_boundary_model_preserves_original_all_pixel_membership(name):
    width, height = 65, 49  # partial tiles must retain inactive participating lanes
    yy, xx = np.indices((height, width))
    colors = np.ones((height, width), np.uint32)
    candidate = np.ones(colors.shape, bool)
    if name == 'holes':
        candidate[4::5, 3::7] = False
    elif name == 'rings':
        distance = np.minimum.reduce([xx, yy, width-1-xx, height-1-yy])
        candidate = distance % 4 == 0
    elif name == 'diagonal':
        candidate = xx == yy
    elif name == 'depthsplit':
        candidate[:, width//2-1:width//2+1] = False
    elif name == 'overlap_checker':
        colors = (xx+yy) % 2 + 1
    elif name == 'long_snake':
        candidate[:] = False
        for row in range(0, height, 2):
            candidate[row] = True
            if row+1 < height:
                candidate[row+1, width-1 if row//2 % 2 == 0 else 0] = True
    actual, fault, tile_rounds = tiled_candidate(colors, candidate)
    assert fault == 0
    np.testing.assert_array_equal(actual, independent_labels(colors, candidate))
    assert tile_rounds <= 256


def test_final_convergence_guard_refuses_an_unprocessed_cross_tile_edge():
    colors = np.ones((17, 17), np.uint32)
    actual, fault, _ = tiled_candidate(colors, np.ones(colors.shape, bool), rounds=0)
    assert fault == 1
    assert len(np.unique(actual)) == 4  # tiles remain independent, must not publish


def test_every_original_crossing_diagonal_is_covered_by_forward_boundary_edges():
    for width, height in ((1, 1), (17, 19), (33, 31)):
        rng = np.random.default_rng(width*100+height)
        for _ in range(5):
            colors = rng.integers(0, 5, (height, width), dtype=np.uint32)
            candidate = colors != 0
            actual, fault, _ = tiled_candidate(colors, candidate)
            assert fault == 0
            np.testing.assert_array_equal(actual, independent_labels(colors, candidate))


def test_capacity5120_roots_are_preserved_for_existing_explicit_refusal():
    width, height = 640, 192
    yy, xx = np.indices((height, width))
    colors = (xx//4) % 2 + 2*((yy//6) % 2) + 1
    actual, fault, _ = tiled_candidate(colors, np.ones(colors.shape, bool))
    assert fault == 0
    roots, areas = np.unique(actual, return_counts=True)
    assert len(roots) == 5120
    assert np.all(areas == 24)
    np.testing.assert_array_equal(actual, independent_labels(colors, np.ones(colors.shape, bool)))


def test_actual_boundary_lane_gate_covers_every_cross_tile_undirected_edge():
    for width, height in ((1, 1), (17, 19), (33, 31), (65, 49)):
        expected, observed = set(), set()
        for y in range(height):
            for x in range(width):
                for dy, dx in ((0, 1), (1, -1), (1, 0), (1, 1)):
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < width and ny < height):
                        continue
                    if (nx//16, ny//16) == (x//16, y//16):
                        continue
                    edge = ((x, y), (nx, ny))
                    expected.add(edge)
                    # Literal production early gate, independent of vector model.
                    if (x & 15) != 0 and (x & 15) != 15 and (y & 15) != 15:
                        continue
                    observed.add(edge)
        assert observed == expected


def test_256_vertex_bound_includes_final_unchanged_round():
    # General graph worst bound; an induced image component can only be shorter.
    labels = np.arange(1, 257)
    for iteration in range(256):
        following = labels.copy()
        following[1:] = np.minimum(labels[1:], labels[:-1])
        if np.array_equal(following, labels):
            break
        labels = following
    else:
        raise AssertionError('no confirmation round inside bound')
    assert iteration == 255
    assert np.all(labels == 1)
