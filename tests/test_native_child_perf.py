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
    for edge,checks in ((child_edge,('root>i+1','other>j+1','graphParent(other-1)')),
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
    parent_cache_resource_guard(shader)
    packed = body(shader,'void child_packed_init(')
    assert 'cut=ownParent&&' in packed
    support = body(shader,'void child_support(')
    assert 'step=128;step>0;step/=2' in support
    assert support.count('compact.InterlockedAdd') == 2
    assert 'recordBase(uint r){return 32+4*p.parentCapacity+16*r;}' in shader
    assert 'count<=p.parentCapacity' in body(shader,'void child_lookup(')
    assert not re.search(r'\b(?:double|uint64_t|int64_t)\b',shader)



def parent_cache_resource_guard(shader):
    import re
    assert body(shader, 'uint graphParent(').strip() == 'return wl(8*p.count+i);'
    init = body(shader, 'void child_init(')
    assert 'uint par=sourceSelectedParent(i);' in init
    assert 'graphParent(' not in init
    assert init.count('sourceSelectedParent(') == 5
    assert 'ws(8*p.count+i,par);' in init
    assert shader.count('ws(8*p.count+i,') == 1
    # Map publication is at the same scope as own-parent resolution: every
    # in-range pixel, including background, invalid-depth and cut pixels.
    before = init[init.index('uint par=sourceSelectedParent(i);'):init.index('ws(8*p.count+i,par);')]
    assert before.count('{') == before.count('}')
    for name, kind in (('candidate','bool'), ('childEdge','void'), ('child_hook','void'),
                       ('child_jump','void'), ('validateChildEdge','void'),
                       ('child_validate','void'), ('child_reduce','void'), ('child_gather','void')):
        code = body(shader, kind+' '+name+'(')
        assert 'graphParent(' in code and 'sourceSelectedParent(' not in code
    gather = body(shader, 'void child_gather(')
    guard = 'if(par==0||par>hl(3)||area==0){fault(64);return;}'
    assert guard in gather
    assert gather.index(guard) < gather.index('uint parent=par-1;')
    late = shader[shader.index('groupshared uint digits'):]
    assert 'graphParent(' not in late
    assert 'sourceSelectedParent(global)' in body(shader, 'void child_packed_init(')
    assert len(re.findall(r'vk::binding\(', shader)) == 11
    assert shader.count('[numthreads(') == 21
    assert shader.count('GroupMemoryBarrierWithGroupSync()') == 14


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
    ('late_parent_map_overwrite','else if(par>hl(3)){fault(64);}else{',
     'else if(par>hl(3)){fault(64);}else{ws(8*p.count+i,par-1);'),
    ('cross_parent_cut_count','cut=ownParent&&','cut='),
    ('wrong_reduction_bound','step=128;step>0;step/=2','step=256;step>0;step/=2'),
    ('status_sized_by_child_capacity','recordBase(uint r){return 32+4*p.parentCapacity+16*r;}',
     'recordBase(uint r){return 32+4*p.capacity+16*r;}'),
    ('parent_count_child_bound','count<=p.parentCapacity','count<=p.capacity'),
    ('lost_child_equal_skip','graphParent(other-1)!=par){fault(4);return;}if(other==root)return;',
     'graphParent(other-1)!=par){fault(4);return;}'),
    ('lost_fill_equal_skip','other>j+1){fault(32);return;}if(other==root)return;',
     'other>j+1){fault(32);return;}'),
    ('root_walk_in_jump','ws(p.count+i,next);',
     'while(next!=wl(next-1))next=wl(next-1);ws(p.count+i,next);'),
]

SHADER_MUTANTS += [
    ('background_cache_lost','ws(8*p.count+i,par);','if(root!=0){ws(8*p.count+i,par);}'),
    ('cache_neighbor_before_publication','sourceSelectedParent(i-1)','graphParent(i-1)'),
    ('gather_zero_parent_underflow','if(par==0||par>hl(3)||area==0)','if(par>hl(3)||area==0)'),
    ('packed_cache_after_alias','sourceSelectedParent(global)','graphParent(global)'),
    ('repeated_resolver_in_hook','if(root==0)return;uint par=graphParent(i),x=i%p.width','if(root==0)return;uint par=sourceSelectedParent(i),x=i%p.width'),
    ('added_shader_barrier','ws(8*p.count+i,par);','ws(8*p.count+i,par);GroupMemoryBarrierWithGroupSync();'),
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
                  'ChildCounts header(', 'ChildCounts result(', 'void collect_locked(', 'void release(',
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
    assert 'const auto counts=header(i)' in result
    assert result.index('std::lock_guard<std::mutex> lock(mutex)') < result.index('if(e.decoded)') < result.index('const auto counts=header(i)') < result.index('std::memcpy(')
    header = body(source,'ChildCounts header(')
    assert header.index('if(!ready(i))') < header.index('e.readback.mapped')
    for check in ('words[0]!=1', 'words[1]!=p.width', 'words[2]!=p.height',
                  'words[3]>p.parentCapacity', 'words[4]>p.capacity',
                  'words[5]>p.packedCapacity', 'words[6]>p.jobCapacity', 'words[7]||',
                  'words[8]!=p.calLo', 'words[9]!=p.calHi', 'words[10]!=p.parentLo',
                  'words[11]!=p.parentHi', 'words[12]!=p.childLo', 'words[13]!=p.childHi',
                  'words[14]!=p.maxBits', 'words[15]!=p.jumpBits', 'words[16]!=p.minChild',
                  'words[17]!=p.capacity', 'words[18]!=p.packedCapacity',
                  'words[19]!=p.jobCapacity', 'words[20]!=p.parentCapacity', 'words[21]||',
                  'words[22]!=p.maxLo', 'words[23]!=p.maxHi', 'words[24]!=p.jumpLo',
                  'words[25]!=p.jumpHi', 'for(unsigned j=26;j<32;j++)if(words[j])'):
        assert check in header
    assert 'size_t(counts.parent_count)*sizeof(ParentChildStatus)' in result
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
    ('sever_header_validation','const auto counts=header(i)','const ChildCounts counts{}'),
    ('missing_header_fault','words[7]||',''),
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


def test_actual_parent_resolver_cache_witness_and_bounded_CPU_cost(tmp_path):
    """Run extracted shader resolver/map access and publication on host arrays.

    This proves cache bytes/load counts only, not shader execution or GPU time.
    """
    import json
    shader = (PACKAGE/'shaders/children.slang').read_text()
    parent_cache_resource_guard(shader)
    init = body(shader, 'void child_init(')
    resolve = 'uint par=sourceSelectedParent(i);'
    publish = 'ws(8*p.count+i,par);'
    assert resolve in init and publish in init
    code = r'''
#include <vector>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <stdexcept>
using uint=uint32_t;
uint faults=0;void fault(uint bit){faults|=bit;}
struct Buffer{std::vector<uint> a;size_t loads=0;uint Load(uint at){++loads;return a.at(at/4);}};
Buffer source,parentLabels,lookup;std::vector<uint> work;
struct Parameters{uint count,blocks,parentLabelWords;}p;
size_t workLoads=0;
uint wl(uint i){++workLoads;return work.at(i);}void ws(uint i,uint v){work.at(i)=v;}
__attribute__((noinline)) uint sourceSelectedParent(uint i){RESOLVER}
__attribute__((noinline)) uint graphParent(uint i){ACCESSOR}
void materialize(uint i){if(i<p.count){RESOLVE PUBLISH}}
void require(bool b){if(!b)throw std::runtime_error("source cache witness failure");}
volatile uint64_t sink=0;
int main(){
 for(uint n:{1u,35u,307200u}){
  p={n,16,3};source.a.assign(2*n,0);parentLabels.a.assign(n+3,0);lookup.a.assign(4*17,0);work.assign(9*n,0xdeadbeef);
  for(uint color=0;color<4;color++)for(uint label=1;label<=16;label++)lookup.a[color*17+label]=(color+label)%5==0?0:100*color+label;
  lookup.a[3*17+16]=4096;
  const uint masks[]={0,1,2,4,8,3,8};
  std::vector<uint> expected(n);
  for(uint i=0;i<n;i++){
   uint mask=masks[i%7],label=i%13==0?0:(i%13==1?17:1+i%16);if(i==0){mask=8;label=16;}
   source.a[i]=mask;source.a[n+i]=i%2?0:0x3f000000;parentLabels.a[3+i]=label;
   uint color=mask==1?0:mask==2?1:mask==4?2:3;
   expected[i]=(mask==1||mask==2||mask==4||mask==8)&&label>0&&label<=16?lookup.a[color*17+label]:0;
  }
  faults=0;for(uint i=0;i<n;i++)materialize(i);materialize(n); // out of range does not touch memory
  require(work[8*n]==4096); // upper one-based parent capacity, never zero based
  for(uint i=0;i<8*n;i++)require(work[i]==0xdeadbeef);
  for(uint i=0;i<n;i++)require(graphParent(i)==expected[i]);
  require(n<35||faults==1); // invalid labels retain the source resolver fault
  // Reusing the cache after gather cannot affect late source membership.
  for(uint i=0;i<n;i++)work[8*n+i]=0xabcdef;
  for(uint i=0;i<n;i++)require(sourceSelectedParent(i)==expected[i]);
  for(uint i=0;i<n;i++)materialize(i);
  if(n!=307200)continue;
  source.loads=parentLabels.loads=lookup.loads=workLoads=0;
  for(uint i=0;i<n;i++)sink+=sourceSelectedParent(i);
  size_t oldLoads=source.loads+parentLabels.loads+lookup.loads;
  for(uint i=0;i<n;i++)sink+=graphParent(i);
  require(source.loads==n&&workLoads==n&&oldLoads>workLoads);
  std::printf("{\"pixels\":%u,\"old_source_loads\":%zu,\"old_label_loads\":%zu,\"old_lookup_loads\":%zu,\"cache_loads\":%zu,\"rounds\":32,\"GPU_executed\":false,\"qualified\":false,\"samples\":[",n,source.loads,parentLabels.loads,lookup.loads,workLoads);
  for(uint trial=0;trial<3;trial++){
   auto start=std::chrono::steady_clock::now();uint64_t sum=0;
   for(uint round=0;round<32;round++)for(uint i=0;i<n;i++)sum+=sourceSelectedParent(i);
   auto middle=std::chrono::steady_clock::now();
   for(uint round=0;round<32;round++)for(uint i=0;i<n;i++)sum+=graphParent(i);
   auto end=std::chrono::steady_clock::now();sink+=sum;
   std::printf("%s{\"source_ns\":%lld,\"cache_ns\":%lld}",trial?",":"",(long long)std::chrono::duration_cast<std::chrono::nanoseconds>(middle-start).count(),(long long)std::chrono::duration_cast<std::chrono::nanoseconds>(end-middle).count());
  }std::puts("]}");
 }
}
'''
    code = code.replace('RESOLVER', body(shader, 'uint sourceSelectedParent(')).replace('ACCESSOR', body(shader, 'uint graphParent(')).replace('RESOLVE', resolve).replace('PUBLISH', publish)
    unit, binary = tmp_path/'cache-cost.cpp', tmp_path/'cache-cost'
    unit.write_text(code)
    compile_command = [os.environ.get('CXX','g++'), '-std=c++17', '-O2', str(unit), '-o', str(binary)]
    compiled = subprocess.run(compile_command, capture_output=True, text=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['cache_loads'] == payload['pixels'] == 307200
    assert payload['old_source_loads']+payload['old_label_loads']+payload['old_lookup_loads'] > payload['cache_loads']
    assert payload['GPU_executed'] is payload['qualified'] is False
    assert len(payload['samples']) == 3
    assert all(s['source_ns'] > 0 and s['cache_ns'] > 0 for s in payload['samples'])
    (tmp_path/'compile-command.json').write_text(json.dumps(compile_command)+'\n')
    (tmp_path/'cache-cost.json').write_text(json.dumps(payload,indent=2)+'\n')
