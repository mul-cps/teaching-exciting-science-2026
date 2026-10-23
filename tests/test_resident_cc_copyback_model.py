"""Independent graph/forest proof of B->A copies, CPU only."""
import numpy as np
import pytest
from resident_cc_copyback_model import graph, solve, reference


def scene(name, width=65, height=49):
    y, x = np.indices((height, width))
    colors = np.ones((height, width), np.uint32)
    active = np.ones((height, width), bool)
    if name == 'holes':
        active[4::5, 3::7] = False
    elif name == 'rings':
        active = np.minimum.reduce([x, y, width-1-x, height-1-y]) % 4 == 0
    elif name == 'checker':
        colors = (x+y) % 2+1
    elif name == 'split':
        active[:, width//2-1:width//2+1] = False
    elif name == 'snake':
        active[:] = False
        for row in range(0, height, 2):
            active[row] = True
            if row+1 < height:
                active[row+1, width-1 if row//2 % 2 == 0 else 0] = True
    elif name == 'diagonal':
        active = x == y
    return colors, active


@pytest.mark.parametrize('name', ['flat', 'holes', 'rings', 'checker', 'split', 'snake', 'diagonal'])
def test_original_and_copyback_equal_independent_exact_minimum_roots(name):
    colors, active = scene(name)
    forest, original_edges, boundary = graph(colors, active)
    expected = reference(colors, active)
    old = solve(forest, original_edges, boundary, copyback=False)
    new = solve(forest, original_edges, boundary, copyback=True)
    assert old['fault'] == new['fault'] == 0
    assert old['roots'] == new['roots'] == expected
    assert new['copy_bytes'] == 32*colors.size*4
    assert new['parent_reads'] <= old['parent_reads']


@pytest.mark.parametrize('width,height', [(1, 1), (1, 33), (17, 19), (33, 31)])
def test_partial_tiles_multicolor_inactive_and_repeated_edges(width, height):
    rng = np.random.default_rng(width*100+height)
    for _ in range(4):
        colors = rng.integers(0, 5, (height, width))
        active = (colors != 0) & (rng.random(colors.shape) > .15)
        forest, edges, boundary = graph(colors, active)
        for copyback in (False, True):
            actual = solve(forest, edges, boundary*3, copyback=copyback)
            assert actual['fault'] == 0
            assert actual['roots'] == reference(colors, active)


def test_long_monotone_forest_is_flattened_without_partition_change():
    forest = [1] + list(range(1, 1024))
    old = solve(forest, [], [], copyback=False)
    new = solve(forest, [], [], copyback=True)
    assert old['roots'] == new['roots'] == [1]*1024
    assert old['fault'] == new['fault'] == 0
    assert new['parent_reads'] < old['parent_reads']//10


@pytest.mark.parametrize('forest', [[2, 1], [1, 3], [1, 0, 2], [1, 3, 2]])
def test_invalid_or_inactive_parent_fault_is_sticky_after_copy(forest):
    for copyback in (False, True):
        assert solve(forest, [], [], copyback=copyback)['fault'] & 4


def test_final_original_edge_guard_detects_missing_hooks_with_or_without_copy():
    colors, active = scene('flat', 17, 17)
    forest, edges, boundary = graph(colors, active)
    for copyback in (False, True):
        assert solve(forest, edges, [], copyback=copyback)['fault'] == 1
        assert solve(forest, edges, boundary, copyback=copyback, rounds=0)['fault'] == 1


def test_independent_reference_detects_false_edges_not_detected_by_convergence_guard():
    colors = np.array([[1, 2]])
    forest, edges, boundary = graph(colors, np.ones(colors.shape, bool))
    assert not edges and not boundary
    for copyback in (False, True):
        corrupted = solve(forest, edges, [(0, 1)], copyback=copyback)
        assert corrupted['fault'] == 0  # convergence is not a false-union oracle
        assert corrupted['roots'] != reference(colors, np.ones(colors.shape, bool))
