"""CPU-only source resource contracts for the optional AOT Slang adapter.

These guards supplement the native allocation/equivalence executable; they do
not claim to measure allocator internals inside LLVM-generated object code.
"""
from pathlib import Path
import re
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_vulkan'


def clean(source):
    return re.sub(r'//[^\n]*|/\*.*?\*/', '', source, flags=re.S)


def body(source, signature):
    source = clean(source)
    start = source.index('{', source.index(signature)) + 1
    depth = 1
    for end in range(start, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start:end]
    raise AssertionError('unterminated function')


def guard_adapter(source):
    code = clean(source)
    hot = body(code, 'void cpu_slang(')
    # The only native crossings are two whole-frame loops, followed by the same
    # completion implementation used by cpu_reference. No pixel call boundary.
    assert not re.search(r'\b(for|while|do)\b', hot)
    assert re.findall(r'\bcpu_(convert|candidates|complete_reference)\s*\(', hot) == [
        'convert', 'candidates', 'complete_reference']
    assert 'Grid grid{{0,0,0},{1,1,1}};' in hot
    assert 'cpu_convert(&grid,nullptr,&g);cpu_candidates(&grid,nullptr,&g);' in hot
    for view in ('s.upload.mapped,s.upload.size', 's.cpu_masks.data(),n',
                 's.cpu_depth.data(),n', 's.cpu_xyz.data(),4*n',
                 's.cpu_parents.data(),n'):
        assert view in hot
    assert hot.count('{nullptr,0}') == 2  # unbound GPU scratch and compact
    assert not re.search(
        r'\bnew\b|\b(?:malloc|calloc|realloc|free|memcpy|memmove|memset)\s*\('
        r'|\.(?:resize|reserve|assign|push_back|create|wait|compile)\s*\('
        r'|\b(?:vk[A-Z]\w*|slang\w*|scratch_bytes|morphology_bytes)\s*\('
        r'|Context::|std::(?:vector|thread)|\b(?:system|popen|exec\w*)\s*\(', hot)


def guard_shader(source):
    code = clean(source)
    assert '#include "resident.slang"' in code
    assert code.count('[numthreads(1,1,1)]') == 2
    for name in ('cpu_convert', 'cpu_candidates'):
        hot = body(code, 'void ' + name + '(')
        loop = '[loop]for(uint i=0;i<p.count;i++){'
        assert hot.strip().startswith(loop) and hot.strip().endswith('}')
        assert hot.count(loop) == 1
        assert not re.search(r'\b(for|while|do)\b', hot.replace(loop, '', 1))
        assert 'tid.' not in hot
    assert not re.search(r'\b(?:Interlocked\w*|\w*Barrier\w*|groupshared)\b', code)
    assert not re.search(r'\b(?:scratch|compact)\s*\[', code)
    assert 'cpuMasks[i]=uint8_t(maskAt(i));cpuDepth[i]=d;' in code
    assert 'cpuParents[i]=ok&&!cut?i+1:0;' in code


def test_actual_aot_adapter_resource_contract():
    guard_adapter((PACKAGE / 'src/cpu_slang.cpp').read_text())


@pytest.mark.parametrize('old,new', [
    ('const size_t n=', 'auto* temporary=new float[100];const size_t n='),
    ('const size_t n=', 'void* temporary=malloc(4096);const size_t n='),
    ('const size_t n=', 'vkDeviceWaitIdle(nullptr);const size_t n='),
    ('const size_t n=', 'Context::shared();const size_t n='),
    ('const size_t n=', 's.cpu_xyz.resize(100);const size_t n='),
    ('const size_t n=', 'memcpy(s.cpu_masks.data(),s.upload.mapped,1);const size_t n='),
    ('const size_t n=', 'system("slangc");const size_t n='),
    ('{nullptr,0}', '{s.scratch.mapped,s.scratch.size}'),
    ('s.cpu_xyz.data(),4*n', 's.cpu_xyz.data(),n'),
    ('Grid grid{{0,0,0},{1,1,1}};', 'Grid grid{{0,0,0},{uint32_t((n+255)/256),1,1}};'),
    ('cpu_convert(&grid,nullptr,&g);', 'for(size_t i=0;i<n;i++)cpu_convert(&grid,nullptr,&g);'),
    ('cpu_convert(&grid,nullptr,&g);', 'cpu_convert(&grid,nullptr,&g);cpu_convert(&grid,nullptr,&g);'),
    ('cpu_complete_reference(s,w,h);', 'cpu_reference(s,w,h);'),
])
def test_adapter_resource_mutants_rejected(old, new):
    source = (PACKAGE / 'src/cpu_slang.cpp').read_text()
    assert old in source
    with pytest.raises(AssertionError):
        guard_adapter(source.replace(old, new, 1))


def test_actual_cpu_shader_resource_contract():
    guard_shader((PACKAGE / 'shaders/resident_cpu.slang').read_text())


@pytest.mark.parametrize('old,new', [
    ('i<p.count', 'i<=p.count'),
    ('[numthreads(1,1,1)]', '[numthreads(256,1,1)]'),
    ('cpuMasks[i]=', 'scratch[i]=0;cpuMasks[i]='),
    ('cpuParents[i]=', 'GroupMemoryBarrierWithGroupSync();cpuParents[i]='),
    ('cpuParents[i]=', 'for(uint k=0;k<256;k++)cpuParents[i]='),
])
def test_shader_resource_mutants_rejected(old, new):
    source = (PACKAGE / 'shaders/resident_cpu.slang').read_text()
    assert old in source
    with pytest.raises(AssertionError):
        guard_shader(source.replace(old, new, 1))


def test_cpu_modes_bypass_vulkan_and_share_completion():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    constructor = body(source, 'Pipeline::Pipeline(')
    # Keep hardware-first automatic selection; explicit CPU modes never enter
    # this branch and create only a CPU FramePool.
    assert 'if(mode=="auto"||mode=="vulkan")' in constructor
    vulkan = body(constructor, 'if(mode=="auto"||mode=="vulkan")')
    assert 'Context::shared()' in vulkan and 'create_gpu()' in vulkan
    assert constructor.count('Context::shared()') == 1
    assert 'FramePool>(w,h,nullptr,debug)' in constructor
    submit = body(source, 'void Pipeline::submit_impl(')
    assert 'if(backend_=="cpu-slang"){cpu_slang(s,width(),height());s.submitted=true;return;}' in submit
    assert 'if(backend_=="cpu"){cpu_reference(s,width(),height());s.submitted=true;return;}' in submit
    cpu = (PACKAGE / 'src/cpu_pipeline.cpp').read_text()
    assert 'cpu_complete_reference(s,w,h);' in body(cpu, 'void cpu_reference(')
    complete = body(cpu, 'void cpu_complete_reference(')
    assert 'cpu_morphology(s,w,h);' in complete
    assert not re.search(r'\bnew\b|\.(?:resize|reserve|assign|push_back)\s*\(', complete)


def abi_fixture():
    """Minimal complete Slang 2026.19 LLVM reflection for the supported ABI."""
    from copy import deepcopy
    sizes = lambda value, alignment: [dict(kind='uniform', value=value, alignment=alignment)]
    binding = lambda offset, size: dict(kind='uniform', offset=offset, size=size, elementStride=0)
    scalar = lambda name, width: dict(kind='scalar', scalarType=name, sizes=sizes(width, width))
    names = ('input', 'scratch', 'divisions', 'compact', 'p', 'cpuMasks', 'cpuDepth', 'cpuXYZ', 'cpuParents')
    offsets = (0, 16, 32, 48, 64, 72, 88, 104, 120)
    element_types = dict(divisions=('int32', 4), cpuMasks=('uint8', 1),
                         cpuDepth=('float32', 4), cpuXYZ=('float32', 4), cpuParents=('uint32', 4))
    fields = []
    for i, name in enumerate(('width', 'height', 'count', 'capacity', 'scale', 'fx', 'fy', 'cx', 'cy', 'maxDepth', 'jump', 'round')):
        fields.append(dict(name=name, binding=binding(i*4, 4),
                           type=scalar('uint32' if i < 4 or i == 11 else 'float32', 4)))
    parameters = []
    for name, offset in zip(names, offsets):
        if name == 'p':
            t = dict(kind='constantBuffer', elementType=dict(kind='struct', name='Parameters', fields=fields, sizes=sizes(48, 4)))
        else:
            t = dict(kind='resource', baseShape='structuredBuffer' if name in element_types else 'byteAddressBuffer')
            if name in element_types:
                t['resultType'] = scalar(*element_types[name])
        t['sizes'] = sizes(8 if name == 'p' else 16, 8)
        parameters.append(dict(name=name, type=t, binding=binding(offset, 8 if name == 'p' else 16)))
    tid = dict(name='tid', semanticName='SV_DISPATCHTHREADID',
               type=dict(kind='vector', elementCount=3, elementType=dict(kind='scalar', scalarType='uint32'), sizes=[]))
    entries = []
    for name in ('cpu_convert', 'cpu_candidates'):
        entries.append(dict(name=name, stage='compute', threadGroupSize=[1, 1, 1],
                            parameters=[deepcopy(tid)],
                            scope=dict(kind='constantBuffer', binding=binding(0, 8), parameters=[deepcopy(tid)]),
                            bindings=[dict(name=p['name'], binding=deepcopy(p['binding'])) for p in parameters]))
    return dict(version='1.1', parameters=parameters, entryPoints=entries)


def abi_validator():
    import importlib.util
    spec = importlib.util.spec_from_file_location('resource_aot_builder', ROOT / 'tools/performance/build_native_slang_cpu.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.validate


def test_supported_reflected_cpu_abi():
    abi_validator()(abi_fixture())


@pytest.mark.parametrize('mutation', [
    'added_uniform_entry_argument', 'missing_entry_parameters', 'missing_entry_scope',
    'wrong_tid_vector_width', 'wrong_scope_tid_width', 'global_binding_stride',
    'entry_binding_stride', 'parameter_field_stride', 'missing_binding_stride', 'old_group256',
])
def test_cpu_abi_rejects_null_entry_argument_and_stride_mismatches(mutation):
    reflected = abi_fixture()
    entry = reflected['entryPoints'][0]
    if mutation == 'added_uniform_entry_argument':
        entry['parameters'].append(dict(name='uniformValue', binding=dict(kind='uniform', offset=0, size=4, elementStride=0), type=dict(kind='scalar', scalarType='uint32')))
    elif mutation == 'missing_entry_parameters':
        del entry['parameters']
    elif mutation == 'missing_entry_scope':
        del entry['scope']
    elif mutation == 'wrong_tid_vector_width':
        entry['parameters'][0]['type']['elementCount'] = 4
    elif mutation == 'wrong_scope_tid_width':
        entry['scope']['parameters'][0]['type']['elementCount'] = 4
    elif mutation == 'global_binding_stride':
        reflected['parameters'][0]['binding']['elementStride'] = 16
    elif mutation == 'entry_binding_stride':
        entry['bindings'][0]['binding']['elementStride'] = 16
    elif mutation == 'parameter_field_stride':
        reflected['parameters'][4]['type']['elementType']['fields'][0]['binding']['elementStride'] = 4
    elif mutation == 'old_group256':
        entry['threadGroupSize'] = [256, 1, 1]
    else:
        del reflected['parameters'][0]['binding']['elementStride']
    with pytest.raises((ValueError, KeyError)):
        abi_validator()(reflected)
