"""Independent parent resource-flow, forest-race and CPU orchestration review."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'


def test_parent_cache_compiler_identity_and_mode_are_truthful(tmp_path):
    compiler = Path.home()/'.local/opt/slang-2026.19/bin/slangc'
    if not compiler.is_file():
        pytest.skip('real Slang required for cold parent-cache identity fixture')
    fixture = tmp_path/'source'
    for directory in ('shaders', 'cmake', 'src', 'test'):
        (fixture/directory).mkdir(parents=True)
    for rel in ('shaders/parents.slang', 'shaders/hash_shader.cmake',
                'cmake/parents.cmake', 'src/parents.cpp', 'src/frame_pool.cpp',
                'test/parent_preparation_contract.cpp'):
        shutil.copy2(PACKAGE/rel, fixture/rel)
    (fixture/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(parent_cache_contract CXX)
find_package(Vulkan REQUIRED)
add_library(resident_rgbd INTERFACE)
include(cmake/parents.cmake)
''')
    counter = tmp_path/'compiler-calls.json'
    counter.write_text('[]')
    wrapper = tmp_path/'slang-wrapper'
    def replace_compiler(version):
        wrapper.write_text(f'#!{sys.executable}\n'+f'''import json,pathlib,subprocess,sys
if sys.argv[1:] == ['-version']:
 print({version!r});sys.exit(0)
p=pathlib.Path({str(counter)!r});calls=json.loads(p.read_text());calls.append({version!r});p.write_text(json.dumps(calls))
sys.exit(subprocess.call([{str(compiler)!r},*sys.argv[1:]]))
''')
        wrapper.chmod(0o755)
    def command(*args, success=True):
        result = subprocess.run([str(ROOT/'ros2/conda/so101-conda.sh'), 'run', 'env',
                                 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp', 'cmake', *map(str,args)],
                                capture_output=True,text=True)
        if success:
            assert result.returncode == 0, result.stdout+result.stderr
        else:
            assert result.returncode != 0
        return result.stdout+result.stderr
    build = tmp_path/'build'
    output = build/'parent-shaders'
    def configure(*args, success=True):
        return command('-S',fixture,'-B',build,f'-DSLANGC_EXECUTABLE={wrapper}',*args,success=success)
    def compile():
        command('--build',build,'--target','parent_spirv','--parallel','1')
    replace_compiler('parent-wrapper-v1')
    configure()
    compile()
    assert len(json.loads(counter.read_text())) == 9
    times = {p.name:p.stat().st_mtime_ns for p in output.glob('*.spv')}
    previous_provenance = (output/'provenance.txt').read_bytes() if (output/'provenance.txt').exists() else None
    provenance_time = (output/'provenance.txt').stat().st_mtime_ns if previous_provenance else None
    configure()
    compile()
    assert len(json.loads(counter.read_text())) == 9
    assert times == {p.name:p.stat().st_mtime_ns for p in output.glob('*.spv')}
    if previous_provenance:
        assert (output/'provenance.txt').stat().st_mtime_ns == provenance_time
    # Same path/version/content change must recompile, not just relabel metadata.
    replace_compiler('parent-wrapper-v2')
    configure()
    if previous_provenance:
        assert (output/'provenance.txt').read_bytes() == previous_provenance
    compile()
    assert json.loads(counter.read_text()) == ['parent-wrapper-v1']*9+['parent-wrapper-v2']*9
    provenance = (output/'provenance.txt').read_text()
    assert 'version=parent-wrapper-v2' in provenance
    assert f'compiler_sha256={hashlib.sha256(wrapper.read_bytes()).hexdigest()}' in provenance
    replace_compiler('parent-wrapper-v3')
    compile()  # Compiler executable alone must trigger automatic reconfiguration.
    assert json.loads(counter.read_text()) == ['parent-wrapper-v1']*9+['parent-wrapper-v2']*9+['parent-wrapper-v3']*9
    cache = tmp_path/'offline'
    shutil.copytree(output,cache)
    configure(f'-DPRECOMPILED_PARENT_SHADER_DIR={cache}')
    compile()
    assert len(json.loads(counter.read_text())) == 27
    assert (output/'provenance.txt').read_bytes() == (cache/'provenance.txt').read_bytes()
    configure('-DPRECOMPILED_PARENT_SHADER_DIR=')
    compile()
    assert json.loads(counter.read_text()) == ['parent-wrapper-v1']*9+['parent-wrapper-v2']*9+['parent-wrapper-v3']*18
    configure(f'-DPRECOMPILED_PARENT_SHADER_DIR={cache}')
    shader = fixture/'shaders/parents.slang'
    shader.write_text(shader.read_text()+'\n// parent-cache source mutation\n')
    assert 'digest mismatch' in configure(success=False)


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start+1, len(source)):
        depth += (source[end] == '{')-(source[end] == '}')
        if not depth:
            return source[start+1:end]
    raise AssertionError('unbalanced parent function')


def resource_guard(cpp, shader):
    def uncomment(text):
        import re
        return re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    import re
    submit = body(cpp,'ParentLease ParentPreparation::submit(')
    result = body(cpp,'ParentCounts result(')
    hot = [submit,result,body(cpp,'bool ParentPreparation::ready('),
           body(cpp,'void ParentPreparation::collect('),body(cpp,'void collect_locked(')]
    forbidden = r'\b(?:new|malloc|calloc|realloc)\b|std::vector|make_shared|\.create\(|vk(?:Wait|Allocate|Create|Map)|vk(?:Queue|Device)WaitIdle'
    for block in hot:
        code = re.sub(r'"(?:\\.|[^"\\])*"','""',uncomment(block))
        assert not re.search(forbidden,code)
        assert not re.search(r'for\s*\([^;]*;[^;]*<(?:\s*)(?:s\.)?p\.(?:count|nodes|blocks)',code)
    assert 'VkBufferCopy copy{0,0,e.compact.size}' in submit
    assert 'e.compact.buffer,e.readback.buffer,1,&copy' in submit
    assert cpp.count('vkCmdCopyBuffer') == submit.count('vkCmdCopyBuffer') == 1
    assert '.arena.mapped' not in cpp and '.scratch.mapped' not in cpp
    # Detect forbidden work moved into an ordinary helper, excluding only the
    # exact cold constructor/destructor, not arbitrary functions/overloads.
    ctor = 'ParentPreparation::ParentPreparation(std::shared_ptr<Context> context,FramePoolToken token,const ParentConfig& config,const std::string& directory)'
    assert 'VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT,VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT' in body(cpp,ctor)
    residual = cpp
    for signature in (ctor,'~ParentStorage('):
        residual = residual.replace(body(residual,signature),'')
    assert not re.search(forbidden,uncomment(residual))
    assert 'for(unsigned round=0;round<32;round++)' in submit
    assert 'e.decoded=false' in submit and 'belongs_to(s.source_token)' in submit
    assert 'source.calibration.generation!=s.config.calibration_generation' in submit
    assert 'source.metadata.calibration_generation!=s.config.calibration_generation' in submit
    assert 'if(e.decoded)return e.completed_counts;' in result
    assert result.index('std::lock_guard<std::mutex> lock(mutex)') < result.index('if(e.decoded)return e.completed_counts;')
    assert result.index('if(e.decoded)return e.completed_counts;') < result.index('std::memcpy(')
    assert 'words[7]>p.capacity||words[8]||words[13]' in result
    assert 'counts.record_count>total' in result
    assert 'size_t(counts.record_count)*sizeof(ParentRecord)' in result
    assert result.index('for(unsigned j=0;j<counts.record_count;j++)') < result.index('e.decoded=true')
    assert 'std::array<ParentRecord,kMaxComponents> records{}' in cpp
    config = body(cpp,'void ParentConfig::validate(')
    assert 'if(!min_parent_pixels)' in config
    assert not re.search(r'min_parent_pixels\s*>',uncomment(config))
    # Snapshot B is immutable throughout hooks; A receives only atomic minima.
    edge = uncomment(body(shader,'void hookEdge('))
    hook = uncomment(body(shader,'void parents_hook('))
    for block in (edge,hook):
        assert 'storeWord(' not in block and '.Store(' not in block
        assert 'loadWord(p.nodes+' not in block
    assert 'arena.InterlockedMin(4*(p.nodes+j),loadWord(2*p.nodes+j),ignored)' in hook
    assert 'loadWord(2*p.nodes+j)' in edge and 'loadWord(2*p.nodes+k)' in edge
    assert 'InterlockedMin(4*(p.nodes+max(a,b)-1),min(a,b),ignored)' in edge
    flatten = uncomment(body(shader,'void parents_flatten('))
    assert re.findall(r'storeWord\(([^,]+),',flatten) == ['2*p.nodes+j']*4
    assert 'Interlocked' not in flatten and 'loadWord(p.nodes+' in flatten
    assert 'steps<p.blocks' in flatten and 'next>=root' in flatten and 'fault(4)' in flatten
    validate = body(shader,'void validateEdge(')
    assert 'loadWord(2*p.nodes+j)!=loadWord(2*p.nodes+k)' in validate and 'fault(4)' in validate
    scan = uncomment(body(shader,'void parents_scan('))
    assert 'return' not in scan and 'GroupMemoryBarrierWithGroupSync' in scan
    assert 'p.phase==0||loadWord(5*p.nodes+j)>=p.minArea' in scan
    assert 'if(all>p.capacity)fault(2)' in body(shader,'void parents_offsets(')
    records = body(shader,'void parents_records(')
    assert 'compact.Load(32)!=0' in records
    assert 'compact.Store(out+4,loadWord(3*p.nodes+j))' in records


MUTANTS = [
    ('hot_wait', 'cpp', 'vkCmdFillBuffer(e.command,e.compact.buffer',
     'vkDeviceWaitIdle(c.device);vkCmdFillBuffer(e.command,e.compact.buffer'),
    ('hot_heap', 'cpp', 'vkCmdFillBuffer(e.command,e.compact.buffer',
     'std::vector<int> hot(4);vkCmdFillBuffer(e.command,e.compact.buffer'),
    ('hot_map', 'cpp', 'vkCmdFillBuffer(e.command,e.compact.buffer',
     'vkMapMemory(c.device,memory,0,4,0,&mapped);vkCmdFillBuffer(e.command,e.compact.buffer'),
    ('dense_copy', 'cpp', 'VkBufferCopy copy{0,0,e.compact.size}',
     'VkBufferCopy copy{0,0,e.arena.size}'),
    ('dense_cpu_loop', 'cpp', 'vkCmdFillBuffer(e.command,e.compact.buffer',
     'for(unsigned i=0;i<s.p.count;i++)scan(i);vkCmdFillBuffer(e.command,e.compact.buffer'),
    ('repeat_decode', 'cpp', 'if(e.decoded)return e.completed_counts;', ''),
    ('plain_forest_store', 'shader', 'arena.InterlockedMin(4*(p.nodes+j),loadWord(2*p.nodes+j),ignored)',
     'storeWord(p.nodes+j,loadWord(2*p.nodes+j))'),
    ('hook_reads_mutating_A', 'shader', 'uint a=loadWord(2*p.nodes+j),b=loadWord(2*p.nodes+k),ignored',
     'uint a=loadWord(p.nodes+j),b=loadWord(2*p.nodes+k),ignored'),
    ('flatten_writes_A', 'shader', 'storeWord(2*p.nodes+j,root);return;',
     'storeWord(p.nodes+j,root);return;'),
    ('fault_ignored', 'cpp', 'words[8]||words[13]', 'false||words[13]'),
    ('renumber_filtered', 'shader', 'compact.Store(out+4,loadWord(3*p.nodes+j))',
     'compact.Store(out+4,globalRank(j,4))'),
]


def test_parent_resource_graph_and_decode_guard():
    resource_guard((PACKAGE/'src/parents.cpp').read_text(), (PACKAGE/'shaders/parents.slang').read_text())


@pytest.mark.parametrize('name,target,old,new', MUTANTS, ids=[x[0] for x in MUTANTS])
def test_guard_rejects_resource_race_and_ranking_mutants(name, target, old, new):
    cpp = (PACKAGE/'src/parents.cpp').read_text()
    shader = (PACKAGE/'shaders/parents.slang').read_text()
    assert (cpp if target == 'cpp' else shader).count(old) == 1, name
    with pytest.raises(AssertionError):
        resource_guard(cpp.replace(old,new) if target == 'cpp' else cpp,
                       shader.replace(old,new) if target == 'shader' else shader)


@pytest.fixture(scope='module')
def actual_parent_perf(tmp_path_factory):
    directory = tmp_path_factory.mktemp('native-parent-perf')
    binary = directory/'parent-perf'
    command = [os.environ.get('CXX','g++'), '-std=c++17','-O2','-UNDEBUG','-pthread',
               '-I'+str(PACKAGE/'include'), str(PACKAGE/'test/parent_preparation_perf.cpp'),
               str(PACKAGE/'src/parents.cpp'),str(PACKAGE/'src/frame_pool.cpp'),'-o',str(binary)]
    result = subprocess.run(command,capture_output=True,text=True,timeout=60)
    (directory/'compile.log').write_text(result.stdout+result.stderr)
    assert result.returncode == 0, result.stderr
    symbols = subprocess.check_output(['nm','-u',str(binary)],text=True)
    assert 'vkCreate' not in symbols and 'Context' not in symbols
    (directory/'undefined-symbols.txt').write_text(symbols)
    shaders = directory/'shaders'
    shaders.mkdir()
    import struct
    for entry in ('init','hook','flatten','validate','scan','offsets','pixels','labels','records'):
        # Synthetic module header: accepted only by fake vkCreateShaderModule.
        # No shader is validated or executed by this host orchestration test.
        (shaders/('parents_'+entry+'.spv')).write_bytes(struct.pack('<5I',0x07230203,0x10000,0,1,0))
    return directory,binary,shaders


def test_actual_parent_host_paths_allocation_wait_and_decode_cost(actual_parent_perf):
    directory,binary,shaders = actual_parent_perf
    result = subprocess.run([str(binary),str(shaders)],capture_output=True,text=True,timeout=60)
    (directory/'host.stdout').write_text(result.stdout)
    (directory/'host.stderr').write_text(result.stderr)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert not report['gpu_executed'] and not report['qualified']
    assert report['iterations_per_trial']*report['trials'] >= 10000
    assert report['dispatches_per_iteration'] == 73
    assert report['compact_copy_bytes_per_iteration'] == 64+32*4096
    assert [c['record_count'] for c in report['scenarios']] == [0,1,4096]
    for scenario in report['scenarios']:
        assert len(scenario['samples']) == report['trials']
        for sample in scenario['samples']:
            assert sample['cpu_ns'] > 0 and sample['wall_ns'] > 0
            assert sample['heap_allocations'] == sample['buffer_allocations'] == sample['wait_calls'] == 0
            assert sample['dispatches'] == 73*report['iterations_per_trial']
    (directory/'host-measurements.json').write_text(json.dumps(report,indent=2)+'\n')


def test_exact_fused_root_read_model_is_not_gpu_timing(actual_parent_perf):
    directory,binary,_ = actual_parent_perf
    model = json.loads(subprocess.check_output([str(binary),'--model'],text=True))
    assert model == {'baseline_A_reads':715296,'fused_A_reads':85841,
                     'reverse_fused_A_reads':85841,'plain_store_mutant_rejected':True}
    (directory/'root-read-model.json').write_text(json.dumps(model,indent=2)+'\n')
