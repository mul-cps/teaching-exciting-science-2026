"""Mask backend contract; hardware tests skip without an explicit usable GPU."""
import sys
from pathlib import Path
import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects.color_mask_backend import ColorMaskBackend
from tools.propose_rgbd_grasp import color_masks


def assert_masks(actual, rgb):
    expected = color_masks(rgb)
    assert actual.keys() == expected.keys()
    for color in actual:
        np.testing.assert_array_equal(actual[color], expected[color])
        assert actual[color].dtype == bool


def test_cpu_default_does_not_initialize_opencl(monkeypatch):
    def forbidden(*args):
        raise AssertionError('CPU requested OpenCL')
    monkeypatch.setattr(cv2.ocl, 'setUseOpenCL', forbidden)
    backend = ColorMaskBackend()
    assert_masks(backend(np.zeros((17, 29, 3), np.uint8)), np.zeros((17, 29, 3), np.uint8))
    assert backend.status()['selected'] == 'cpu'


def test_unavailable_fallback_uses_each_current_frame_and_is_stable(monkeypatch):
    calls = []
    monkeypatch.setattr(cv2.ocl, 'haveOpenCL', lambda: calls.append(1) or False)
    backend = ColorMaskBackend('opencl')
    for shape, color in [((21, 31, 3), (255, 0, 0)), ((13, 7, 3), (0, 255, 0))]:
        rgb = np.full(shape, color, np.uint8)
        assert_masks(backend(rgb), rgb)
    assert len(calls) == 1
    assert backend.status()['fallback_reason']
    assert backend.status()['requested'] == 'opencl'
    assert backend.status()['selected'] == 'cpu'


def test_execution_exception_falls_back_on_same_frame(monkeypatch):
    backend = ColorMaskBackend('opencl')
    monkeypatch.setattr(backend, '_initialize', lambda: setattr(backend, '_selected', 'opencl'))
    def fail(rgb):
        raise RuntimeError('device lost')
    monkeypatch.setattr(backend, '_opencl_masks', fail)
    rgb = np.full((23, 19, 3), [0, 0, 255], np.uint8)
    assert_masks(backend(rgb), rgb)
    assert 'device lost' in backend.status()['fallback_reason']
    assert backend.status()['selected'] == 'cpu'


def test_invalid_backend_rejected():
    with pytest.raises(ValueError, match='backend'):
        ColorMaskBackend('auto')


@pytest.mark.parametrize('name', ['vulkan', 'opencl'])
def test_hardware_all_hsv_boundaries_random_and_changed_shapes(name):
    backend = ColorMaskBackend(name)
    hsv = np.array([[[h, s, v] for s in (0, 89, 90, 255) for v in (0, 49, 50, 255)]
                    for h in range(180)], np.uint8)
    images = [cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)]
    rng = np.random.default_rng(802)
    images += [rng.integers(0, 256, shape, np.uint8) for shape in
               ((37, 61, 3), (480, 640, 3), (17, 23, 3))]
    for rgb in images:
        masks = backend(rgb)
        if backend.status()['selected'] != name:
            pytest.skip(backend.status()['fallback_reason'])
        assert_masks(masks, rgb)


def test_analyzer_backend_preserves_complete_report(monkeypatch):
    from test_ros2_objects import tabletop_scene
    from so101_station_objects.objects import analyze_rgbd
    rgb, depth, intrinsics, distortion, transform = tabletop_scene()
    args = rgb, depth, intrinsics, distortion, .001, transform
    expected = analyze_rgbd(*args)
    monkeypatch.setattr(cv2.ocl, 'haveOpenCL', lambda: False)
    actual = analyze_rgbd(*args, color_mask_backend=ColorMaskBackend('opencl'))
    status = actual.pop('color_mask_backend')
    expected.pop('color_mask_backend')
    import json
    def encode(value):
        return value.tolist() if isinstance(value, np.ndarray) else value.item()
    assert json.dumps(actual, sort_keys=True, default=encode) == json.dumps(expected, sort_keys=True, default=encode)
    assert status['selected'] == 'cpu' and status['fallback_reason']


def test_vulkan_initialization_exception_falls_back(monkeypatch):
    backend = ColorMaskBackend('vulkan')
    def fail():
        raise RuntimeError('Vulkan unavailable')
    monkeypatch.setattr(backend, '_initialize', fail)
    rgb = np.full((15, 11, 3), [255, 255, 0], np.uint8)
    assert_masks(backend(rgb), rgb)
    assert backend.status()['selected'] == 'cpu'
    assert 'Vulkan unavailable' in backend.status()['fallback_reason']


def test_station_launch_forwards_explicit_mask_backend():
    import ast
    path = ROOT / 'ros2/src/so101_station_bringup/launch/station_live.launch.py'
    tree = ast.parse(path.read_text())
    defaults = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in node.targets))
    values = {key.value: value for key, value in zip(defaults.keys, defaults.values)}
    assert ast.literal_eval(values['color_mask_backend']) == 'cpu'
    includes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name) and node.func.id == 'include'
                and len(node.args) > 2 and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == 'so101_station_objects']
    assert len(includes) == 1
    config = includes[0].args[2]
    pairs = {key.value: value for key, value in zip(config.keys, config.values) if isinstance(key, ast.Constant)}
    value = pairs['color_mask_backend']
    assert value.func.id == 'LaunchConfiguration'
    assert ast.literal_eval(value.args[0]) == 'color_mask_backend'
