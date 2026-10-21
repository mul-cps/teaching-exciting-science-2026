"""Independent no-device regressions for the frozen mask-offload data path."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_objects/so101_station_objects'


def method(filename, cls, method):
    tree = ast.parse((PACKAGE / filename).read_text())
    klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls)
    return next(n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name == method)


def test_hot_call_one_upload_dispatch_and_readback_without_compilation_or_wait():
    tree = method('vulkan_color_masks.py', 'VulkanColorMasks', '__call__')
    names = [ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n, ast.Call)]
    assert names.count('self.rgb.copy_from_numpy') == 1
    assert names.count('self.kernel.dispatch') == 1
    assert names.count('self.output.to_numpy') == 1
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr in {'zeros', 'create_buffer', 'reshape', 'copy_from_numpy', 'dispatch', 'to_numpy', 'view'}
    assert len([n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'dispatch']) == 1
    assert len([n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'to_numpy']) == 1
    assert not any(any(x in name.lower() for x in ('wait', 'synchronize', 'load_module', 'link_program', 'compute_kernel', 'open', 'read_text', 'fsync', 'sleep')) for name in names)
    assert not any(isinstance(n, (ast.For, ast.While)) for n in ast.walk(tree))
    assert len([n for n in ast.walk(tree) if isinstance(n, ast.comprehension)]) == 1
    assert "('red', 'green', 'blue', 'yellow')" in ast.unparse(tree)


def test_full_frame_setup_is_guarded_by_shape_change():
    tree = method('vulkan_color_masks.py', 'VulkanColorMasks', '__call__')
    branch = next(n for n in tree.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'self.shape != image.shape')
    setup_calls = [ast.unparse(n.func) for n in ast.walk(branch) if isinstance(n, ast.Call)]
    assert setup_calls.count('np.zeros') == 1
    assert setup_calls.count('self.device.create_buffer') == 2
    outside = [n for n in tree.body if n is not branch]
    assert not any(isinstance(n, ast.Call) and ast.unparse(n.func) in {'np.zeros', 'self.device.create_buffer'} for line in outside for n in ast.walk(line))
    assert 'count > 4096 * 4096' in ast.unparse(tree)


def test_gpu_import_and_device_creation_are_lazy():
    tree = ast.parse((PACKAGE / 'vulkan_color_masks.py').read_text())
    assert not any(isinstance(n, (ast.Import, ast.ImportFrom)) and 'slangpy' in ast.unparse(n) for n in tree.body)
    constructor = method('color_mask_backend.py', 'ColorMaskBackend', '__init__')
    assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in {'_initialize', 'Device', 'setUseOpenCL'} for n in ast.walk(constructor))
    call = method('color_mask_backend.py', 'ColorMaskBackend', '__call__')
    assert any(isinstance(n, ast.Assign) and any(ast.unparse(t) == 'self._selected' for t in n.targets) and ast.literal_eval(n.value) == 'cpu' for n in ast.walk(call))
    assert ast.unparse(call.body[-1]) == 'return color_masks(rgb)'


def test_shader_layout_and_dispatch_bounds_remain_packed_integer_only():
    source = (PACKAGE / 'color_masks.slang').read_text()
    assert 'ByteAddressBuffer rgb;' in source
    assert 'StructuredBuffer<int> divisions;' in source
    assert 'RWStructuredBuffer<uint> masks;' in source
    assert 'uniform uint pixelCount;' in source
    assert 'if (i >= pixelCount) return 0u;' in source
    assert 'if (i >= pixelCount) return;' in source
    assert 'uint i = tid.x * 4u;' in source
    assert 'masks[tid.x] = maskAt(i)' in source
    assert '<< 8u' in source and '<< 16u' in source and '<< 24u' in source
    tokens = source.replace('//', ' ').split()
    assert not {'float', 'double', 'float64_t', 'InterlockedAdd', 'GroupMemoryBarrierWithGroupSync'}.intersection(tokens)


def test_shader_installed_at_runtime_module_sibling_path():
    cmake = (ROOT / 'ros2/src/so101_station_objects/CMakeLists.txt').read_text()
    source = (PACKAGE / 'vulkan_color_masks.py').read_text()
    assert 'install(FILES so101_station_objects/color_masks.slang' in cmake
    assert 'DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")' in cmake
    assert "Path(__file__).with_name('color_masks.slang')" in source
