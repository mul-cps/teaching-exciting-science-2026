"""Mock Slang API counts and frame identity without any GPU initialization."""
from collections import Counter
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'ros2/src/so101_station_objects'), str(ROOT / 'tests')]
from so101_station_objects.color_mask_backend import ColorMaskBackend
from so101_station_objects.vulkan_color_masks import VulkanColorMasks
from tools.propose_rgbd_grasp import color_masks


@pytest.fixture
def fake_slang(monkeypatch):
    counts = Counter()

    class Buffer:
        def __init__(self, data):
            self.data = data

        def copy_from_numpy(self, value):
            counts['upload'] += 1
            counts['upload_bytes'] += value.nbytes
            self.data = value.copy()

        def to_numpy(self):
            counts['readback'] += 1
            counts['readback_bytes'] += self.data.nbytes
            return self.data

    class Device:
        def __init__(self, **kwargs):
            counts['device'] += 1
            assert kwargs['type'] == 1
            assert kwargs['enable_cuda_interop'] is False
            self.info = SimpleNamespace(adapter_name='Intel Review Mock GPU')

        def load_module(self, path):
            counts['compile'] += 1
            assert Path(path).name == 'color_masks.slang' and Path(path).is_file()
            return SimpleNamespace(entry_point=lambda name: name)

        def link_program(self, modules, entries):
            counts['link'] += 1
            assert entries == ['main']
            return None

        def create_compute_kernel(self, program):
            return SimpleNamespace(dispatch=self.dispatch)

        def create_buffer(self, **kwargs):
            counts['allocate'] += 1
            if 'data' in kwargs:
                assert kwargs['struct_size'] == 4
                return Buffer(kwargs['data'].copy())
            if 'size' in kwargs:
                return Buffer(np.zeros(kwargs['size'], dtype=np.uint8))
            assert kwargs['struct_size'] == 4
            return Buffer(np.zeros(kwargs['element_count'], dtype=np.uint32))

        def dispatch(self, *, thread_count, vars):
            counts['dispatch'] += 1
            assert set(vars) == {'rgb', 'divisions', 'masks', 'pixelCount'}
            n = vars['pixelCount']
            assert thread_count == [(n+3)//4, 1, 1]
            assert vars['divisions'].data.shape == (512,)
            image = vars['rgb'].data[:3*n].reshape(1, n, 3)
            packed = np.zeros((n+3) & ~3, dtype=np.uint8)
            for bit, mask in enumerate(color_masks(image).values()):
                packed[:n] |= mask.reshape(-1).astype(np.uint8) << bit
            vars['masks'].data = packed.view(np.uint32)

    monkeypatch.setitem(sys.modules, 'slangpy', SimpleNamespace(Device=Device, DeviceType=SimpleNamespace(vulkan=1), BufferUsage=SimpleNamespace(shader_resource=1, unordered_access=2), __version__='review-mock'))
    return counts


def test_one_frame_transfer_counts_and_shape_resources(fake_slang):
    backend = VulkanColorMasks()
    assert fake_slang['device'] == fake_slang['compile'] == fake_slang['link'] == 1
    assert fake_slang['allocate'] == 1  # fixed division table
    frames = [np.full((17, 19, 3), [255, 0, 0], np.uint8), np.full((17, 19, 3), [0, 255, 0], np.uint8), np.full((23, 31, 3), [0, 0, 255], np.uint8)[::2, ::-1]]
    expected_upload = expected_readback = 0
    for index, image in enumerate(frames):
        result = backend(image)
        assert tuple(result) == ('red', 'green', 'blue', 'yellow')
        for color, expected in color_masks(image).items():
            np.testing.assert_array_equal(result[color], expected)
        assert fake_slang['allocate'] == (3 if index < 2 else 5)
        assert fake_slang['upload'] == fake_slang['dispatch'] == fake_slang['readback'] == index + 1
        expected_upload += (image.size + 3) & ~3
        expected_readback += (image.shape[0] * image.shape[1] + 3) & ~3
    assert fake_slang['compile'] == fake_slang['link'] == 1
    assert fake_slang['upload_bytes'] == expected_upload
    assert fake_slang['readback_bytes'] == expected_readback


def test_runtime_device_failure_permanently_falls_back_on_current_frame(fake_slang):
    backend = ColorMaskBackend('vulkan')
    red = np.full((17, 19, 3), [255, 0, 0], np.uint8)
    backend(red)
    assert backend.status()['selected'] == 'vulkan'
    def fail(**kwargs):
        raise RuntimeError('review injected device lost')
    backend._vulkan.kernel.dispatch = fail
    blue = np.full((11, 23, 3), [0, 0, 255], np.uint8)
    actual = backend(blue)
    assert backend.status()['selected'] == 'cpu'
    assert backend.status()['gpu_available'] is True  # capability observed before failure
    assert 'device lost' in backend.status()['fallback_reason']
    for color, expected in color_masks(blue).items():
        np.testing.assert_array_equal(actual[color], expected)
    allocations = fake_slang['allocate']
    backend(red)
    assert fake_slang['allocate'] == allocations
    assert fake_slang['device'] == 1 and fake_slang['dispatch'] == 1


@pytest.mark.parametrize('shape', [(0, 20, 3), (4097, 4097, 3)])
def test_pixel_bound_rejects_before_frame_allocation(fake_slang, shape):
    backend = VulkanColorMasks()
    with pytest.raises(ValueError, match='4096x4096 pixel bound'):
        backend(SimpleNamespace(shape=shape))
    assert fake_slang['allocate'] == 1
    assert fake_slang['upload'] == fake_slang['dispatch'] == fake_slang['readback'] == 0


@pytest.mark.parametrize('kind', ['float_rgb', 'rgba', 'mismatched_depth'])
def test_analyzer_rejects_invalid_inputs_before_any_backend_call(kind):
    from test_ros2_objects import tabletop_scene
    from so101_station_objects.objects import analyze_rgbd
    rgb, depth, intrinsics, distortion, transform = tabletop_scene()
    if kind == 'float_rgb':
        rgb = rgb.astype(np.float32)
    elif kind == 'rgba':
        rgb = np.concatenate([rgb, rgb[..., :1]], axis=2)
    else:
        depth = depth[:-1]
    def forbidden(image):
        raise AssertionError('invalid input reached mask backend')
    with pytest.raises(ValueError, match='expected matching uint8 RGB'):
        analyze_rgbd(rgb, depth, intrinsics, distortion, .001, transform, color_mask_backend=forbidden)
