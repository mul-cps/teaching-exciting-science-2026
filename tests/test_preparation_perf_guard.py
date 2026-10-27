"""CPU-only scheduling guards for the separately reviewed preparation modules."""
from pathlib import Path
import re
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'
GEOMETRY_INITIALIZER = ('GeometryPreparation::GeometryPreparation(std::shared_ptr<Context> context,'
                        'const GeometryCalibration& cal,const std::string& directory,GeometryPrecision precision)')


def body(source, signature):
    """Balanced scopes keep nested diagnostics/loops inside the checked function."""
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start+1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start+1:end]
    raise AssertionError('unbalanced function scope')


def test_hot_paths_have_no_allocation_wait_dense_cpu_loop_or_mapping():
    for module, signatures in {
        'alignment': ('Alignment::acquire(', 'Alignment::submit(', 'Alignment::ready('),
        'geometry': ('GeometryPreparation::submit(', 'GeometryPreparation::ready(',
                     'GeometryPreparation::finish(', 'GeometryPreparation::collect('),
    }.items():
        source = (PACKAGE/f'src/{module}.cpp').read_text()
        for signature in signatures:
            hot = body(source, signature)
            assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|\.create\(|vk(?:Allocate|Create|Map|Wait)|vk(?:Queue|Device)WaitIdle|std::vector|make_shared', hot), signature
            assert not re.search(r'for\s*\([^;]*;[^;]*(?:307200|640|480|\.count|\.grid_count)', hot), signature
        assert 'c.command_mutex' not in source  # dedicated pools through full recording
        assert 'VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT' in source
        assert 'vkDestroyCommandPool' in source and 'c.queue_mutex' in source
    # Whole-module residual checks also catch forbidden work moved into helpers.
    for module, initialization, destructor in (
        ('alignment', ('Alignment::Alignment(', 'void initialize('), '~AlignmentState('),
        ('geometry', (GEOMETRY_INITIALIZER,), '~GeometryStorage('),
    ):
        source = (PACKAGE/f'src/{module}.cpp').read_text()
        for signature in (*initialization, destructor):
            source = source.replace(body(source, signature), '')
        assert not re.search(r'vk(?:Wait|Allocate|Create|Map)|vk(?:Queue|Device)WaitIdle|\.create\(|make_shared|std::vector', source)
    geometry = (PACKAGE/'src/geometry.cpp').read_text()
    assert 'commands=VK_NULL_HANDLE' in geometry
    assert 'cmd_info.commandPool=s->commands' in geometry
    assert 'source.fence' in body(geometry, 'GeometryPreparation::submit(')
    assert 'shaderFloat64' in body(geometry, GEOMETRY_INITIALIZER)
    assert 'maxComputeWorkGroupInvocations<256' in geometry
    assert 'maxStorageBufferRange' in geometry



@pytest.mark.parametrize('location,payload', [
    ('hot', 'auto forbidden = std::make_shared<int>(0);'),
    ('hot', 'vkWaitForFences();'),
    ('helper', 'auto forbidden = std::make_shared<int>(0);'),
    ('helper', 'vkWaitForFences();'),
    ('delegating_constructor', 'auto forbidden = std::make_shared<int>(0);'),
    ('delegating_constructor', 'vkWaitForFences();'),
])
def test_geometry_guard_rejects_hot_helper_and_delegation_mutants(monkeypatch, location, payload):
    # Exercise the same complete guard using in-memory source mutations only.
    # Only the exact cold initializer may escape the whole-module residual scan.
    path = PACKAGE/'src/geometry.cpp'
    source = path.read_text()
    if location == 'helper':
        mutated = source + '\nvoid forbidden_helper(){' + payload + '}\n'
    else:
        signature = ('GeometryPreparation::submit(' if location == 'hot'
                     else 'GeometryPreparation::GeometryPreparation(')
        start = source.index('{', source.index(signature)) + 1
        mutated = source[:start] + payload + source[start:]
    read_text = Path.read_text
    def read_mutated(candidate, *args, **kwargs):
        return mutated if candidate == path else read_text(candidate, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', read_mutated)
    with pytest.raises(AssertionError):
        test_hot_paths_have_no_allocation_wait_dense_cpu_loop_or_mapping()


def test_normal_readback_capacities_and_private_pool_serialization():
    alignment = (PACKAGE/'src/alignment.cpp').read_text()
    hot = body(alignment, 'Alignment::submit(')
    assert 'color_n=state_->color_n' in hot
    assert 'status_copy{color_n*4,0,4}' in hot
    assert hot.index('color_n=state_->color_n') < hot.index('status_copy{color_n*4,0,4}')
    assert 'vkCmdCopyBuffer(s.command,s.scratch.buffer,s.status.buffer,1,&status_copy)' in hot
    assert 'if(state_->diagnostics)' in hot
    diagnostics = body(hot, 'if(state_->diagnostics)')
    normal = hot.replace(diagnostics, '')
    assert normal.count('vkCmdCopyBuffer(') == 2  # native upload + four-byte color-grid fault
    assert '.diagnostic.buffer' not in normal
    assert 'lock(state_->mutex)' in hot
    geometry = (PACKAGE/'src/geometry.cpp').read_text()
    assert 'e.counts.create(c,16,' in geometry
    assert 'vkCmdCopyBuffer' not in body(geometry, 'GeometryPreparation::submit(')
    assert 'lock(s.mutex)' in body(geometry, 'GeometryPreparation::submit(')
    assert 'vkGetFenceStatus' in body(geometry, 'GeometryPreparation::submit(')


def test_all_hues_pin_retained_known_color_union_and_threshold_rounding():
    # Independent OpenCV RGB->HSV truth, including rounding on inverse conversion.
    # Retained support uses OR(color_masks), NOT hue-independent S/V alone.
    import cv2
    import numpy as np
    from tools.propose_rgbd_grasp import color_masks
    hues, saturations, values = np.meshgrid(np.arange(180, dtype=np.uint8),
                                           np.array([0, 88, 89, 90, 91, 255], np.uint8),
                                           np.array([0, 48, 49, 50, 51, 255], np.uint8), indexing='ij')
    hsv = np.stack([hues, saturations, values], axis=-1).reshape(-1, 1, 3)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    h, s, v = np.moveaxis(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV), -1, 0)
    expected = (s>=90) & (v>=50) & ((h<=10) | ((h>=15)&(h<=140)) | (h>=170))
    # Evaluate the existing resident integer RGB conversion, comparing against
    # the independently computed cv2 truth rather than another color-mask copy.
    r, g, b = np.moveaxis(rgb.astype(np.int32), -1, 0)
    maximum = np.maximum(np.maximum(r, g), b)
    delta = maximum - np.minimum(np.minimum(r, g), b)
    divisors = np.zeros(256, np.int32)
    hue_divisors = np.zeros(256, np.int32)
    divisors[1:] = np.rint((255 << 12) / np.arange(1, 256)).astype(np.int32)
    hue_divisors[1:] = np.rint((180 << 12) / (6*np.arange(1, 256))).astype(np.int32)
    actual_s = (delta*divisors[maximum]+2048)>>12
    numerator = np.where(maximum==r, g-b, np.where(maximum==g, b-r+2*delta, r-g+4*delta))
    actual_h = (numerator*hue_divisors[delta]+2048)>>12
    actual_h = np.where(actual_h<0, actual_h+180, actual_h)
    resident = (actual_s>=90)&(maximum>=50)&((actual_h<=10)|((actual_h>=15)&(actual_h<=140))|(actual_h>=170))
    np.testing.assert_array_equal(resident, expected)
    resident_shader = (PACKAGE/'shaders/resident.slang').read_text()
    assert 's=(d*divisions[v]+2048)>>12' in resident_shader
    assert 'h=(h*divisions[256+d]+2048)>>12' in resident_shader
    assert 'vivid=s>=90&&v>=50' in resident_shader
    masks = color_masks(rgb)
    np.testing.assert_array_equal(np.logical_or.reduce(tuple(masks.values())), expected)
    assert np.any((s>=90)&(v>=50)&~expected)  # hue gaps really occur
    shader = (PACKAGE/'shaders/geometry.slang').read_text()
    assert 'source.Load(4*(uint(sy)*p.width+uint(sx)))!=0' in shader
    retained = (ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py').read_text()
    assert 'vivid = np.logical_or.reduce(tuple(masks.values()))' in retained
