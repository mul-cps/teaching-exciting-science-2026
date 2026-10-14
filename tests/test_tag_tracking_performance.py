"""Keep saved-frame tracking free of hardware and accelerated-model dependencies."""
import ast
import inspect

from workshop.digital_twin import tag_tracking


def test_tracking_is_cpu_saved_array_only():
    tree = ast.parse(inspect.getsource(tag_tracking))
    forbidden = {'torch', 'pyrealsense2', 'serial', 'lerobot', 'ev3dev', 'pybricks'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            assert all(alias.name.split('.')[0] not in forbidden for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert (node.module or '').split('.')[0] not in forbidden
        elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {'cuda', 'synchronize', 'VideoCapture'}


def test_detector_has_no_python_pixel_iteration():
    tree = ast.parse(inspect.getsource(tag_tracking))
    detect = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == 'detect')
    assert not any(isinstance(node, (ast.For, ast.While)) for node in ast.walk(detect))
    for node in ast.walk(detect):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {'ndindex', 'ndenumerate'}
