"""Structural guards avoid brittle machine-specific timing assertions."""
import ast
import inspect
from workshop.digital_twin.grasp_geometry import fuse_depth


def test_fusion_has_no_pixel_loops_or_full_frame_nan_reduction():
    tree = ast.parse(inspect.getsource(fuse_depth))
    assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.GeneratorExp)) for n in ast.walk(tree))
    calls = [n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    assert 'nanmedian' not in calls
    assert 'nanpercentile' not in calls
    # Cropping precedes the first sorting operation.
    source = inspect.getsource(fuse_depth)
    assert source.index('a[:, y0:y1, x0:x1]') < source.index('np.sort')
