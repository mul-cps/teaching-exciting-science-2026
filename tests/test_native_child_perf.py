"""Independent resident-child resource bounds and actual-module CPU host costs."""
from pathlib import Path
import os
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'


def test_child_named_payload_includes_status_readback_and_CPU_caches(tmp_path):
    source = tmp_path/'layout.cpp'
    binary = tmp_path/'layout'
    source.write_text('''#include "so101_station_vulkan/child_graph.hpp"
#include <iostream>
using namespace so101_station_vulkan;
int main(){ChildGraphLayout full(640,480),tiny(1,1),small(1,1,1,1),parents2(1,1,2,1),children2(1,1,1,2);
std::cout<<full.workspace_bytes<<" "<<full.compact_bytes<<" "<<full.device_bytes()<<" "<<full.readback_bytes()<<" "<<full.cache_bytes()<<" "<<full.total_bytes()<<" "<<tiny.draft_offset<<" "<<tiny.total_bytes()<<" "<<small.total_bytes()<<" "<<parents2.total_bytes()<<" "<<children2.total_bytes()<<"\\n";}
''')
    result = subprocess.run([os.environ.get('CXX','g++'), '-std=c++17',
                             '-I'+str(PACKAGE/'include'),str(source),'-o',str(binary)],
                            capture_output=True,text=True,timeout=30)
    assert result.returncode == 0, result.stderr
    values = list(map(int,subprocess.check_output([str(binary)],text=True,timeout=5).split()))
    # Literal budgets include16B/parent statuses in device+readback AND both
    # CPUdecoded caches. Tiny/highcapacity scratch must not overlap live drafts.
    assert values == [14745600,327808,23882960,327808,327680,24538448,2048,1378696,856,904,1144]


def body(source, signature):
    start = source.index('{',source.index(signature))
    depth = 1
    for end in range(start+1,len(source)):
        depth += (source[end]=='{')-(source[end]=='}')
        if depth == 0:
            return source[start+1:end]
    raise AssertionError('unbalanced child function')


def shader_resource_guard(shader):
    import re
    # Fault is sticky, atomic. A group may sample it while another group faults;
    # ordinary fault loads are allowed only in nonmutating/serial later phases.
    assert re.search(r'compact\.InterlockedOr\(28,\s*0,',shader)
    snapshot = body(shader,'uint faultSnapshot(')
    assert 'compact.InterlockedOr(28,0,value)' in snapshot
    assert shader.count('compact.InterlockedOr(28,0,') == 1
    assert 'hl(7)' not in shader
    for helper in ('faultGroup','groupState'):
        code = body(shader,'void '+helper+'(')
        assert 'if(lane==0)' in code and 'groupFault=faultSnapshot()' in code
        barrier = code.index('GroupMemoryBarrierWithGroupSync')
        assert code.index('groupFault=faultSnapshot()') < barrier
        assert code[:barrier].count('{') == code[:barrier].count('}')
    gather = body(shader,'void child_gather(')
    assert 'faultGroup(lane.x)' in gather and 'groupState(' not in gather
    assert 'hl(4)' not in gather  # count mutates atomically in this dispatch
    for name in ('child_reduce','child_gather','child_scatter','child_jobs',
                 'child_packed_init','child_fill_hook','child_fill_jump',
                 'child_fill_validate','child_boundary','child_support'):
        code = body(shader,'void '+name+'(')
        assert 'faultSnapshot(' not in code, name  # never per-pixel RMW
        assert 'faultGroup(' in code or 'groupState(' in code, name
    for name in ('child_hook','child_fill_hook'):
        code = body(shader,'void '+name+'(')
        assert 'InterlockedMin' in code and 'for(' not in code
    for name in ('child_jump','child_fill_jump'):
        code = body(shader,'void '+name+'(')
        assert 'for(' not in code and 'while(' not in code
    child_edge = body(shader,'void childEdge(')
    fill_edge = body(shader,'void fillEdge(')
    for edge,checks in ((child_edge,('root>i+1','other>j+1','selectedParent(other-1)')),
                        (fill_edge,('root<=begin','root>end','other<=begin','other>end','other>j+1'))):
        assert 'if(other==root)return' in edge
        for check in checks:
            assert edge.index(check) < edge.index('if(other==root)return')
    slot = body(shader,'uint hashSlot(')
    assert 'probe<256' in slot and 'InterlockedCompareExchange' in slot
    init = body(shader,'void child_init(')
    assert 'InterlockedAdd(sums[slot]' in init and 'InterlockedAdd(sums2[slot]' in init
    assert init.count('compact.InterlockedAdd') == 2
    reduce = body(shader,'void child_reduce(')
    assert reduce.count('work.Interlocked') == 6
    assert 'if(root==i+1)ws(8*p.count+i,par-1)' in reduce
    packed = body(shader,'void child_packed_init(')
    assert 'cut=ownParent&&' in packed
    support = body(shader,'void child_support(')
    assert 'step=128;step>0;step/=2' in support
    assert support.count('compact.InterlockedAdd') == 2
    assert 'recordBase(uint r){return 32+4*p.parentCapacity+16*r;}' in shader
    assert 'count<=p.parentCapacity' in body(shader,'void child_lookup(')
    assert not re.search(r'\b(?:double|uint64_t|int64_t)\b',shader)


def test_child_shader_resource_race_and_bounded_work():
    shader_resource_guard((PACKAGE/'shaders/children.slang').read_text())


# Mutants modify in-memory source only; no production file or GPU is touched.
SHADER_MUTANTS = [
    ('plain_fault_load','compact.InterlockedOr(28,0,value)','value=hl(7)'),
    ('divergent_group_barrier','if(lane==0)groupFault=faultSnapshot();GroupMemoryBarrierWithGroupSync()',
     'if(lane==0){groupFault=faultSnapshot();GroupMemoryBarrierWithGroupSync();}'),
    ('gather_reads_mutating_count','faultGroup(lane.x);if(groupFault!=0)return;uint i=tid.x;if(i>=p.count||pl(i)!=i+1)',
     'groupState(lane.x);if(groupFault!=0)return;uint i=tid.x;if(i>=p.count||pl(i)!=i+1)'),
    ('unbounded_shared_probe','probe<256','probe<p.count'),
    ('per_pixel_status_global','if(par!=0&&valid){uint slot=hashSlot(par);',
     'if(par!=0&&valid){uint ignored;compact.InterlockedAdd(128,1,ignored);uint slot=hashSlot(par);'),
    ('racy_same_value_parent_index','if(root==i+1)ws(8*p.count+i,par-1)',
     'ws(8*p.count+i,par-1)'),
    ('cross_parent_cut_count','cut=ownParent&&','cut='),
    ('wrong_reduction_bound','step=128;step>0;step/=2','step=256;step>0;step/=2'),
    ('status_sized_by_child_capacity','recordBase(uint r){return 32+4*p.parentCapacity+16*r;}',
     'recordBase(uint r){return 32+4*p.capacity+16*r;}'),
    ('parent_count_child_bound','count<=p.parentCapacity','count<=p.capacity'),
    ('lost_child_equal_skip','selectedParent(other-1)!=par){fault(4);return;}if(other==root)return;',
     'selectedParent(other-1)!=par){fault(4);return;}'),
    ('lost_fill_equal_skip','other>j+1){fault(32);return;}if(other==root)return;',
     'other>j+1){fault(32);return;}'),
    ('root_walk_in_jump','ws(p.count+i,next);',
     'while(next!=wl(next-1))next=wl(next-1);ws(p.count+i,next);'),
]


@pytest.mark.parametrize('name,old,new', SHADER_MUTANTS, ids=[m[0] for m in SHADER_MUTANTS])
def test_child_shader_resource_mutants(name,old,new):
    source = (PACKAGE/'shaders/children.slang').read_text()
    assert source.count(old) == 1, name
    with pytest.raises(AssertionError):
        shader_resource_guard(source.replace(old,new))


def modeled_edge_atomics(roots, width, eight_connected, skip_equal):
    """Count fixed-snapshot union updates only; never predicts roots/GPU time."""
    count = 0
    for i,root in enumerate(roots):
        if root == 0:
            continue
        x,y = i%width,i//width
        neighbors = []
        if x:
            neighbors.append(i-1)
        if y:
            neighbors.append(i-width)
            if eight_connected and x:
                neighbors.append(i-width-1)
            if eight_connected and x+1<width:
                neighbors.append(i-width+1)
        for j in neighbors:
            other = roots[j]
            if other and (not skip_equal or other!=root):
                count += 1
    return count


def test_equal_root_skip_removes_only_redundant_union_operations():
    # Roots1/3 are monotone anchors of two not-yet-merged halves. Cross-root
    # edges still require updates; same-root min(A[root-1],root) is a no-op
    # because A[root-1]<=root, including concurrent lower atomic hooks.
    dense = [1]*16
    halves = [1,1,3,3]*4
    unique = list(range(1,17))
    absent = [0]*16
    for eight,expected in [(True,[(42,0),(42,10),(42,42),(0,0)]),
                           (False,[(24,0),(24,4),(24,24),(0,0)])]:
        actual = [(modeled_edge_atomics(scene,4,eight,False),
                   modeled_edge_atomics(scene,4,eight,True))
                  for scene in (dense,halves,unique,absent)]
        assert actual == expected
    # Dense640x480 converged candidate snapshot: this many edge atomics/round
    # disappear; distributed per-node self-fold remains307200/round.
    assert 480*639+640*479+2*639*479 == 1225442


def cpp_resource_guard(source):
    import re
    uncomment = lambda text:re.sub(r'/\*.*?\*/|//[^\n]*','',text,flags=re.S)
    forbidden = r'\b(?:new|malloc|calloc|realloc)\b|make_shared|\.create\(|\.resize\(|\.push_back\(|vk(?:Wait|Allocate|Create|Map)|vk(?:Device|Queue)WaitIdle'
    signatures = ('void record_graph(', 'ChildLease ChildPreparation::submit(',
                  'ChildCounts result(', 'void collect_locked(', 'void release(',
                  'bool ready(unsigned', 'void ChildPreparation::collect(',
                  'bool ChildPreparation::ready(', 'ChildCounts ChildPreparation::finish(')
    for signature in signatures:
        code = uncomment(body(source,signature))
        no_strings = re.sub(r'"(?:\\.|[^"\\])*"','""',code)
        assert not re.search(forbidden,no_strings), signature
        assert 'std::vector' not in code
        assert not re.search(r'for\s*\([^;]*;[^;]*<\s*(?:s\.)?(?:p|g)\.(?:count|packedCapacity)',code)
    constructor = 'ChildPreparation::ChildPreparation(std::shared_ptr<Context> context,FramePoolToken token,const ParentConfig& expected,const ChildConfig& config,const std::string& directory)'
    residual = source
    for signature in (constructor,'~ChildStorage('):
        residual = residual.replace(body(residual,signature),'')
    assert not re.search(forbidden,uncomment(residual))
    graph = uncomment(body(source,'void record_graph('))
    assert source.count('vkCmdCopyBuffer') == graph.count('vkCmdCopyBuffer') == 1
    assert 'VkBufferCopy copy{0,0,compact_bytes}' in graph
    assert 'vkCmdCopyBuffer(command,compact,readback,1,&copy)' in graph
    assert graph.count('round<32') == 2 and 'pass<8' in graph
    for token in ('VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT','VK_ACCESS_INDIRECT_COMMAND_READ_BIT',
                  'VK_ACCESS_SHADER_READ_BIT','VK_PIPELINE_STAGE_HOST_BIT'):
        assert token in graph
    assert 'vkCmdDispatchIndirect(command,arguments,0)' in graph
    assert 'dispatch(10,p.groups)' in graph and graph.index('dispatch(10,p.groups)') < graph.index('indirect(14)')
    assert '.scratch.mapped' not in source and '.workspace.mapped' not in source
    assert '.persistent.mapped' not in source and '.packed.mapped' not in source
    result = body(source,'ChildCounts result(')
    assert 'if(e.decoded)return e.counts' in result
    assert result.index('std::lock_guard<std::mutex> lock(mutex)') < result.index('if(e.decoded)') < result.index('std::memcpy(')
    assert 'words[3]>p.parentCapacity' in result and 'words[4]>p.capacity' in result
    assert 'words[7]||' in result and 'size_t(counts.parent_count)*sizeof(ParentChildStatus)' in result
    assert 'size_t(counts.record_count)*sizeof(ChildRecord)' in result
    assert result.index('e.decoded=true') > result.index('child packed job total fault')
    assert 'cursor-start!=e.statuses[parent].kept_children||area>e.statuses[parent].candidate_pixels' in result
    ctor = body(source,constructor)
    assert 'e.records.resize(g.record_capacity);e.statuses.resize(g.parent_capacity)' in ctor
    assert 'VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT' in ctor
    submit = body(source,'ChildLease ChildPreparation::submit(')
    for token in ('same_parent_config(configuration,s.expected)','source_frame.belongs_to(s.source_token)',
                  'source_frame.context()!=s.context','source.scratch.size<size_t(s.p.count)*8',
                  'status[0]>kMaxComponents||status[1]','e.parent=parent;e.source=source_frame;e.decoded=false'):
        assert token in submit
    assert 'if(e.references.load()==0&&e.submitted&&ready(i))' in body(source,'void collect_locked(')


def test_child_cpp_compact_only_nonblocking_and_decode_once():
    cpp_resource_guard((PACKAGE/'src/children.cpp').read_text())


CPP_MUTANTS = [
    ('hot_heap','auto p=original;','std::vector<int> perframe(3);auto p=original;'),
    ('hot_wait','auto p=original;','vkDeviceWaitIdle(device);auto p=original;'),
    ('hot_map','auto p=original;','vkMapMemory(device,memory,0,4,0,&pointer);auto p=original;'),
    ('dense_copy','VkBufferCopy copy{0,0,compact_bytes}','VkBufferCopy copy{0,0,size_t(p.count)*4}'),
    ('dense_CPU_scan','auto p=original;','for(unsigned i=0;i<p.count;i++)scan(i);auto p=original;'),
    ('missing_indirect_dependency','VK_ACCESS_INDIRECT_COMMAND_READ_BIT','0'),
    ('repeat_decode','if(e.decoded)return e.counts;',''),
    ('status_bound_child_capacity','words[3]>p.parentCapacity','words[3]>p.capacity'),
    ('missing_perparent_counts','cursor-start!=e.statuses[parent].kept_children||area>e.statuses[parent].candidate_pixels',
     'false||area>e.statuses[parent].candidate_pixels'),
    ('drop_before_fence','if(e.references.load()==0&&e.submitted&&ready(i))','if(e.references.load()==0&&e.submitted)'),
]


@pytest.mark.parametrize('name,old,new', CPP_MUTANTS, ids=[m[0] for m in CPP_MUTANTS])
def test_child_cpp_resource_mutants(name,old,new):
    source = (PACKAGE/'src/children.cpp').read_text()
    assert source.count(old) == 1, name
    with pytest.raises(AssertionError):
        cpp_resource_guard(source.replace(old,new))


@pytest.fixture(scope='module')
def actual_child_host_benchmark(tmp_path_factory):
    import json
    import struct
    directory = tmp_path_factory.mktemp('native-child-host-perf')
    binary = directory/'child-perf'
    command = [os.environ.get('CXX','g++'),'-std=c++17','-O2','-UNDEBUG','-pthread',
               '-I'+str(PACKAGE/'include'),str(PACKAGE/'test/child_preparation_perf.cpp'),
               str(PACKAGE/'src/children.cpp'),str(PACKAGE/'src/parents.cpp'),
               str(PACKAGE/'src/frame_pool.cpp'),'-o',str(binary)]
    result = subprocess.run(command,capture_output=True,text=True,timeout=90)
    (directory/'compile.log').write_text(result.stdout+result.stderr)
    (directory/'compile-command.json').write_text(json.dumps(command,indent=2)+'\n')
    assert result.returncode == 0,result.stderr
    symbols = subprocess.check_output(['nm','-u',str(binary)],text=True)
    assert 'vkCreate' not in symbols and 'Context' not in symbols
    (directory/'undefined-symbols.txt').write_text(symbols)
    parent_shaders,child_shaders = directory/'parent-shaders',directory/'child-shaders'
    parent_shaders.mkdir();child_shaders.mkdir()
    # Synthetic headers are accepted ONLY by the fake ABI. They are not claimed
    # validated/executed SPVs; author validates actual21SPVs separately.
    header = struct.pack('<5I',0x07230203,0x10000,0,1,0)
    for entry in ('init','hook','flatten','validate','scan','offsets','pixels','labels','records'):
        (parent_shaders/('parents_'+entry+'.spv')).write_bytes(header)
    import re
    cpp = (PACKAGE/'src/children.cpp').read_text()
    declaration = cpp.split('kernel_names{{',1)[1].split('}};',1)[0]
    names = re.findall(r'"(child_\w+)"',declaration)
    assert len(names)==21
    for name in names:
        (child_shaders/(name+'.spv')).write_bytes(header)
    return directory,binary,parent_shaders,child_shaders


def test_actual_child_host_zero_allocation_wait_and_compact_decode_cost(actual_child_host_benchmark):
    import json
    directory,binary,parent_shaders,child_shaders = actual_child_host_benchmark
    result = subprocess.run([str(binary),str(parent_shaders),str(child_shaders)],
                            capture_output=True,text=True,timeout=60)
    (directory/'host.stdout').write_text(result.stdout)
    (directory/'host.stderr').write_text(result.stderr)
    assert result.returncode == 0,result.stderr
    data = json.loads(result.stdout)
    assert data['gpu_executed'] is False and data['qualified'] is False
    assert data['trials']*data['iterations_per_trial']>=10000
    assert data['compact_copy_bytes_per_iteration']==327808
    assert data['fake_CPU_fill_bytes_per_iteration']==1294544
    assert [s['records'] for s in data['scenarios']]==[0,1,4096]
    for scenario in data['scenarios']:
        assert len(scenario['samples'])==5
        for sample in scenario['samples']:
            assert sample['cpu_ns']>0 and sample['wall_ns']>0
            assert sample['heap_allocations']==sample['buffer_allocations']==sample['wait_calls']==0
            assert sample['dispatches']==166*data['iterations_per_trial']
            assert sample['indirects']==68*data['iterations_per_trial']
    (directory/'host-measurements.json').write_text(json.dumps(data,indent=2)+'\n')


def test_child_cache_compiler_identity_mode_and_publication(tmp_path):
    import hashlib
    import json
    import shutil
    import sys
    compiler = Path.home()/'.local/opt/slang-2026.19/bin/slangc'
    if not compiler.is_file():
        pytest.skip('real Slang required for child cold cache identity proof')
    fixture = tmp_path/'source'
    for name in ('shaders','cmake','src','test'):
        (fixture/name).mkdir(parents=True)
    for rel in ('shaders/children.slang','shaders/hash_shader.cmake','cmake/children.cmake',
                'src/children.cpp','src/parents.cpp','src/frame_pool.cpp','test/child_preparation_contract.cpp'):
        shutil.copy2(PACKAGE/rel,fixture/rel)
    (fixture/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(child_cache_contract CXX)
find_package(Vulkan REQUIRED)
add_library(resident_parents INTERFACE)
add_custom_target(parent_spirv)
include(cmake/children.cmake)
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
    def command(*args,success=True):
        result = subprocess.run([str(ROOT/'ros2/conda/so101-conda.sh'),'run','env',
                                 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp','cmake',*map(str,args)],
                                capture_output=True,text=True,timeout=90)
        if success:
            assert result.returncode==0,result.stdout+result.stderr
        else:
            assert result.returncode!=0
        return result.stdout+result.stderr
    build = tmp_path/'build'
    output = build/'child-shaders'
    def configure(*args,success=True):
        return command('-S',fixture,'-B',build,'-DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
                       f'-DSLANGC_EXECUTABLE={wrapper}',*args,success=success)
    def compile_shaders():
        command('--build',build,'--target','child_spirv','--parallel','1')
    replace_compiler('child-wrapper-v1');configure();compile_shaders()
    assert json.loads(counter.read_text())==['child-wrapper-v1']*21
    old_provenance = (output/'provenance.txt').read_bytes()
    stable = {p.name:p.stat().st_mtime_ns for p in output.iterdir() if p.is_file()}
    configure();compile_shaders()
    assert json.loads(counter.read_text())==['child-wrapper-v1']*21
    assert stable=={p.name:p.stat().st_mtime_ns for p in output.iterdir() if p.is_file()}
    replace_compiler('child-wrapper-v2');configure()
    assert (output/'provenance.txt').read_bytes()==old_provenance
    compile_shaders()
    assert json.loads(counter.read_text())==['child-wrapper-v1']*21+['child-wrapper-v2']*21
    provenance = (output/'provenance.txt').read_text()
    assert 'version=child-wrapper-v2' in provenance
    assert f'compiler_sha256={hashlib.sha256(wrapper.read_bytes()).hexdigest()}' in provenance
    replace_compiler('child-wrapper-v3');compile_shaders()
    assert json.loads(counter.read_text())==['child-wrapper-v1']*21+['child-wrapper-v2']*21+['child-wrapper-v3']*21
    cache = tmp_path/'offline';shutil.copytree(output,cache)
    configure(f'-DPRECOMPILED_CHILD_SHADER_DIR={cache}');compile_shaders()
    assert len(json.loads(counter.read_text()))==63
    assert (output/'provenance.txt').read_bytes()==(cache/'provenance.txt').read_bytes()
    configure('-DPRECOMPILED_CHILD_SHADER_DIR=');compile_shaders()
    assert json.loads(counter.read_text())==['child-wrapper-v1']*21+['child-wrapper-v2']*21+['child-wrapper-v3']*42
    configure(f'-DPRECOMPILED_CHILD_SHADER_DIR={cache}')
    shader = fixture/'shaders/children.slang';shader.write_text(shader.read_text()+'\n// child offline source mutation\n')
    assert 'digest mismatch' in configure(success=False)


def test_actual_slang_scalar_bound_guard_short_circuits(tmp_path):
    compiler = Path.home()/'.local/opt/slang-2026.19/bin/slangc'
    disassembler = Path.home()/'miniforge3/envs/so101-jazzy/bin/spirv-dis'
    if not compiler.is_file() or not disassembler.is_file():
        pytest.skip('actual Slang/SPIRV tools required for scalar guard lowering proof')
    source = tmp_path/'scalar-guard.slang'
    output = tmp_path/'scalar-guard.spv'
    source.write_text('''[[vk::binding(0,0)]] RWByteAddressBuffer state;
bool rightHandSide(uint i){uint old;state.InterlockedAdd(4,1,old);return state.Load(4*i)==0;}
[numthreads(1,1,1)] void main(){uint i=state.Load(0);if(i>7||rightHandSide(i))state.Store(8,1);}
''')
    result = subprocess.run([str(compiler),str(source),'-entry','main','-stage','compute',
                             '-target','spirv','-profile','sm_6_0','-fp-mode','precise','-O2','-o',str(output)],
                            capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
    disassembly = subprocess.check_output([str(disassembler),str(output)],text=True,timeout=10)
    (tmp_path/'scalar-guard.spvasm').write_text(disassembly)
    # The RHS has an observable atomic side effect. It must occur behind the
    # first conditional branch, rather than eagerly before the OR condition.
    assert 'OpAtomicIAdd' in disassembly and 'OpBranchConditional' in disassembly
    assert disassembly.index('OpBranchConditional') < disassembly.index('OpAtomicIAdd')
