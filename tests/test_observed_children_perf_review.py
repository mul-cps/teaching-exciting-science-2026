"""Independent observed-child resource guards and CPU command-encoding cost.

No Vulkan device/driver is loaded. The benchmark compiles the actual encoder body
against counters; its cost is not GPU, transfer, or Vulkan-driver performance.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if depth == 0:
            return source[start + 1:end]
    raise AssertionError('unbalanced function: ' + signature)


def cpp_guard(cpp, header, child, geometry):
    signatures = ('void record_graph(', 'ObservedLease ObservedPreparation::submit(',
                  'bool ready(unsigned', 'ObservedCounts result(', 'void collect_locked(',
                  'void release(', 'bool ObservedPreparation::ready(',
                  'ObservedCounts ObservedPreparation::finish(')
    for signature in signatures:
        code = re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"', '', body(cpp, signature), flags=re.S)
        assert not re.search(r'\b(?:new|malloc|calloc|realloc|memcpy|memmove)\b|make_shared|std::vector|\.(?:resize|create|push_back)\(|vk(?:Wait|Allocate|Create|Map)|vk(?:Device|Queue)WaitIdle', code), signature
        assert not re.search(r'(?:for|while)\s*\([^)]*\b(?:count|capacity|parentCapacity)\b', code), signature
    graph = body(cpp, 'void record_graph(')
    assert cpp.count('vkCmdCopyBuffer(') == graph.count('vkCmdCopyBuffer(') == 1
    assert 'VkBufferCopy copy{0,0,bytes}' in graph
    assert 'vkCmdCopyBuffer(cmd,compact,readback,1,&copy)' in graph
    assert 'vkCmdFillBuffer(cmd,compact,0,bytes,0)' in graph
    assert 'for(unsigned i=0;i<kernel_count;i++)' in graph
    assert 'const unsigned groups[]={ (p.count+255)/256,(p.capacity+255)/256,(p.count+255)/256,(p.capacity+255)/256,(p.count+255)/256,1}' in graph
    assert 'constexpr unsigned kernel_count=6' in header
    assert 'arena_bytes=size_t(48)*count;compact_bytes=64+size_t(80)*capacity' in header
    assert 'uint64_t(w)*h>307200' in header and 'records>4096' in header
    assert 'closed_capacity>4*count' in header
    for token in ('VK_PIPELINE_STAGE_HOST_BIT', 'VK_ACCESS_HOST_READ_BIT',
                  'VK_ACCESS_TRANSFER_WRITE_BIT', 'VK_ACCESS_SHADER_WRITE_BIT',
                  'VK_ACCESS_SHADER_READ_BIT', 'VK_ACCESS_TRANSFER_READ_BIT'):
        assert token in graph
    constructor = body(cpp, 'ObservedPreparation::ObservedPreparation(')
    native = body(constructor, 'if(gc.precision==GeometryPrecision::NativeFloat64)')
    assert 'if(!context->enabled_features.features.shaderFloat64)' in native
    assert 'if(!context->enabled_features.features.shaderInt64)' in native
    geometry_native = geometry.split('case GeometryPrecision::NativeFloat64:', 1)[1].split('break;', 1)[0]
    assert 'if(!context->enabled_features.features.shaderFloat64)' in geometry_native
    assert 'if(!context->enabled_features.features.shaderInt64)' in geometry_native
    assert constructor.index('gc.precision==GeometryPrecision::NativeFloat64') < constructor.index('std::make_shared<ObservedStorage>')
    assert geometry.index('case GeometryPrecision::NativeFloat64:') < geometry.index('std::make_shared<GeometryStorage>')
    assert constructor.count('.create(') == 4  # one ROI plus three buffers/slot
    assert 'for(auto& e:s->entries)' in constructor
    assert 'for(size_t bytes:' in constructor and 'limits.maxStorageBufferRange' in constructor
    submit = body(cpp, 'ObservedLease ObservedPreparation::submit(')
    assert 'e.child=child;e.geometry=geometry' in submit
    assert 'Guard' in submit and 'e.child={};e.geometry={}' in submit
    assert 'geometry.belongs_to(s.geometry_token)' in submit
    assert 'source.belongs_to(s.source_token)' in submit
    assert 'source.index()!=input.source.index()' in submit
    assert '&source.slot()!=&input.source.slot()' in submit
    assert 'source.slot().calibration.depth_scale!=float(s.geometry_config.depth_scale_m)' in submit
    assert 's.collect_locked()' in submit
    assert 'e.compact.buffer,e.readback.buffer,e.compact.size' in submit
    assert 'vkQueueSubmit' in submit and 'queue_lock(c.queue_mutex)' in submit
    collect = body(cpp, 'void collect_locked(')
    assert 'e.references.load()==0&&e.submitted&&ready(i)' in collect
    assert 'e.child={};e.geometry={};e.submitted=false' in collect
    child_view = body(child, 'ChildResidentView ChildLease::resident_view(')
    assert 's.header(index_)' in child_view
    assert '.result(' not in child_view and 'memcpy' not in child_view
    assert 'validate_completed(index_)' in body(geometry, 'GeometryResidentView GeometryLease::resident_view(')
    result = body(cpp, 'ObservedCounts result(')
    assert 'h[6]>p.closedCapacity' in result and 'h[7]||' in result
    assert 'h[3]>p.capacity' in result and 'h[5]>p.count' in result


def shader_guard(shader):
    assert len(re.findall(r'vk::binding\(', shader)) == 7
    assert shader.count('[numthreads(256,1,1)]') == 5
    assert shader.count('[numthreads(1,1,1)]') == 1
    assert 'GroupMemoryBarrier' not in shader
    valid = body(shader, 'bool validRecord(')
    for token in ('r>=min(head(3),p.capacity)', 'root==0||root>p.count',
                  'parent>=children.Load(12)||parent>=p.parentCapacity',
                  'pw>p.width-px||ph>p.height-py', 'w>pw-x||h>ph-y',
                  'area>w*h', 'childWord(r,15)!=0'):
        assert token in valid
    assert valid.index('parent>=p.parentCapacity') < valid.index('uint b=64+32*parent')
    observe = body(shader, 'void observed_observe(')
    assert observe.index('if(i>=p.count)return') < observe.index('roots.Load(4*i)')
    assert 'if(root>p.count)' in observe
    assert observe.index('if(root>p.count)') < observe.index('arena.Load(4*(root-1))')
    assert 'id>min(head(3),p.capacity)' in observe and 'if(!recordValid(r))' in observe
    assert not re.search(r'(?:for|while)\s*\(', observe)
    assert 'validRecord(' not in observe  # full record checks once, not per pixel
    assert 'outWord(r,18)&4' in body(shader, 'bool recordValid(')
    for token in ('childWord(r,2)>=((pw+1)/2)*((ph+1)/2)',
                  'packed>4*p.count||offset>packed||hw*hh>packed-offset',
                  'support<area||support>min(hw*hh,parents.Load(b+12))',
                  'cut>support||cut>hw*hh-area',
                  'previous>=children.Load(12)||previous>=p.parentCapacity',
                  'offset!=po+phw*phh', 'roots.Load(4*(root-1))!=root'):
        assert token in valid
    assert 'validRecord(' not in body(shader, 'void observed_close(')
    assert shader.count('#ifdef OBSERVED_CPU_WITNESS') == 3
    assert 'compact.InterlockedAdd(at,value,ignored)' in body(shader, 'void addCount(')
    assert 'arena.InterlockedCompareExchange(at,0,value,before)' in body(shader, 'uint insertRoot(')
    close = body(shader, 'void observed_close(')
    assert 'uint ids[9]' in close and 'if(n==9)' in close
    assert 'selectedCount' in close and '9*i+selectedCount' in close
    assert 'id>min(head(3),p.capacity)' in close
    assert 'for(uint r=0;r<min(head(3),p.capacity);r++)' in body(shader, 'void observed_finalize(')
    final = body(shader, 'void observed_finalize(')
    assert 'total>p.closedCapacity' in final
    assert final.index('uint cursor=0,') < final.index('for(uint parent=0;')
    parent_pass = body(final, 'for(uint parent=0;')
    assert 'cursor=0' not in parent_pass and 'for(uint r=' not in parent_pass
    assert 'while(cursor<min(head(3),p.capacity)&&childWord(cursor,1)==parent)' in parent_pass
    assert '++cursor' in parent_pass
    for token in ('valid>parentPixels||candidate>valid||all>candidate||kept>all',
                  'cursor-begin!=kept||area>candidate',
                  'cursor!=head(3)||keptTotal!=head(3)',
                  'packedEnd!=children.Load(20)||jobs!=children.Load(24)'):
        assert token in final
    assert 'compact.InterlockedOr(28,value,ignored)' in body(shader, 'void fault(')


def sources():
    return [(P / f).read_text() for f in ('src/observed_children.cpp',
            'include/so101_station_vulkan/observed_children.hpp', 'src/children.cpp', 'src/geometry.cpp')]


def test_actual_host_and_shader_resource_bounds():
    cpp_guard(*sources())
    shader_guard((P / 'shaders/observed_children.slang').read_text())


CPP_MUTANTS = [
    ('allocation', 'auto& s=*storage_;std::lock_guard', 'auto* perframe=new int[20];auto& s=*storage_;std::lock_guard'),
    ('blocking_gpu_wait', 's.collect_locked();', 'vkDeviceWaitIdle(c.device);s.collect_locked();'),
    ('dense_readback', 'VkBufferCopy copy{0,0,bytes}', 'VkBufferCopy copy{0,0,size_t(p.count)*48}'),
    ('perpixel_cpu_loop', 's.collect_locked();', 'for(unsigned i=0;i<s.p.count;i++)scan(i);s.collect_locked();'),
    ('missing_geometry_lease', 'e.child=child;e.geometry=geometry', 'e.child=child;'),
    ('recycle_before_fence', 'e.references.load()==0&&e.submitted&&ready(i)', 'e.references.load()==0&&e.submitted'),
    ('foreign_geometry', 'geometry.belongs_to(s.geometry_token)', 'true'),
    ('drop_sticky_fault', 'h[7]||', ''),
    ('capacity_sized_pixel_dispatch', '(p.count+255)/256,1}', '(p.capacity+255)/256,1}'),
    ('native_without_Int64', 'if(!context->enabled_features.features.shaderInt64)throw std::runtime_error("observed native byte-address Float64 requires enabled shaderInt64");', ''),
]


@pytest.mark.parametrize('name,old,new', CPP_MUTANTS, ids=[m[0] for m in CPP_MUTANTS])
def test_cpp_regressions_rejected(name, old, new):
    cpp, *rest = sources()
    assert old in cpp, name
    with pytest.raises(AssertionError):
        cpp_guard(cpp.replace(old, new), *rest)


SHADER_MUTANTS = [
    ('root_address_unchecked', 'if(root>p.count){fault(2);return;}', ''),
    ('parent_address_unchecked', 'parent>=children.Load(12)||parent>=p.parentCapacity', 'false'),
    ('record_count_unbounded', 'r>=min(head(3),p.capacity)', 'r>=head(3)'),
    ('bbox_overflow', 'w>pw-x||h>ph-y', 'x+w>pw||y+h>ph'),
    ('unbounded_candidate_array', 'uint ids[9]', 'uint ids[4096]'),
    ('pixel_record_scan', 'uint root=roots.Load(4*i);', 'for(uint r=0;r<p.capacity;r++)scan(r);uint root=roots.Load(4*i);'),
    ('membership_overflow_ignored', 'total>p.closedCapacity', 'false'),
    ('non_atomic_fault', 'compact.InterlockedOr(28,value,ignored)', 'compact.Store(28,value)'),
    ('repeat_full_validation_perpixel', 'if(!recordValid(r)){fault(2);return;}', 'if(!validRecord(r)){fault(2);return;}'),
    ('unchecked_packed_extent', 'packed>4*p.count||offset>packed||hw*hh>packed-offset', 'false'),
    ('unchecked_previous_parent', 'previous>=children.Load(12)||previous>=p.parentCapacity', 'false'),
    ('quadratic_parent_scan', 'uint begin=cursor,area=0;', 'cursor=0;uint begin=cursor,area=0;'),
    ('ignored_parent_status', 'valid>parentPixels||candidate>valid||all>candidate||kept>all', 'false'),
    ('ignored_packed_jobs', 'packedEnd!=children.Load(20)||jobs!=children.Load(24)', 'false'),
]


@pytest.mark.parametrize('name,old,new', SHADER_MUTANTS, ids=[m[0] for m in SHADER_MUTANTS])
def test_shader_regressions_rejected(name, old, new):
    shader = (P / 'shaders/observed_children.slang').read_text()
    assert old in shader, name
    with pytest.raises(AssertionError):
        shader_guard(shader.replace(old, new))


def test_actual_encoder_zero_heap_fixed_commands_and_host_cost(tmp_path):
    cpp = (P / 'src/observed_children.cpp').read_text()
    code = r'''
#include "so101_station_vulkan/observed_children.hpp"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
using namespace so101_station_vulkan;
static bool hot=false;static unsigned allocations=0;static volatile unsigned dispatches=0,barriers=0,copies=0,fills=0,binds=0,pushes=0;
static unsigned observed_groups[6];static size_t copy_bytes=0,fill_bytes=0;
void* operator new(size_t n){if(hot)++allocations;void* p=std::malloc(n);if(!p)throw std::bad_alloc();return p;}
void* operator new[](size_t n){return ::operator new(n);}void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}void operator delete(void* p,size_t)noexcept{std::free(p);}void operator delete[](void* p,size_t)noexcept{std::free(p);}
extern "C" {
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdFillBuffer(VkCommandBuffer,VkBuffer,VkDeviceSize offset,VkDeviceSize size,uint32_t value){assert(offset==0&&value==0);++fills;fill_bytes=size;}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdPipelineBarrier(VkCommandBuffer,VkPipelineStageFlags,VkPipelineStageFlags,VkDependencyFlags,uint32_t n,const VkMemoryBarrier*,uint32_t,const VkBufferMemoryBarrier*,uint32_t,const VkImageMemoryBarrier*){assert(n==1);++barriers;}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdBindDescriptorSets(VkCommandBuffer,VkPipelineBindPoint,VkPipelineLayout,uint32_t,uint32_t n,const VkDescriptorSet*,uint32_t,const uint32_t*){assert(n==1);++binds;}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdPushConstants(VkCommandBuffer,VkPipelineLayout,VkShaderStageFlags,uint32_t offset,uint32_t size,const void*){assert(offset==0&&size==64);++pushes;}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdBindPipeline(VkCommandBuffer,VkPipelineBindPoint,VkPipeline){}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdDispatch(VkCommandBuffer,uint32_t x,uint32_t y,uint32_t z){assert(dispatches<6&&y==1&&z==1);observed_groups[dispatches++]=x;}
VKAPI_ATTR __attribute__((noinline,noclone)) void VKAPI_CALL vkCmdCopyBuffer(VkCommandBuffer,VkBuffer,VkBuffer,uint32_t n,const VkBufferCopy* copy){assert(n==1&&copy->srcOffset==0&&copy->dstOffset==0);++copies;copy_bytes=copy->size;}
}
namespace so101_station_vulkan {
void barrier(VkCommandBuffer cmd,VkPipelineStageFlags from,VkPipelineStageFlags to,VkAccessFlags src,VkAccessFlags dst){BARRIER_BODY}
namespace observed_detail {
__attribute__((noinline,noclone)) void record_graph(VkCommandBuffer cmd,VkPipelineLayout layout,const std::array<VkPipeline,kernel_count>& kernels,VkDescriptorSet descriptors,const Parameters& p,VkBuffer compact,VkBuffer readback,size_t bytes){ENCODER_BODY}
}}
int main(){std::array<VkPipeline,6> kernels{};double timings[2]{};
for(unsigned variant=0;variant<2;++variant){unsigned w=variant?640:64,h=variant?480:48,c=variant?4096:32;ObservedLayout g(w,h,c);observed_detail::Parameters p{};p.count=g.count;p.capacity=g.capacity;
const unsigned warmup=1000,iterations=100000;std::chrono::steady_clock::time_point start;
for(unsigned i=0;i<warmup+iterations;++i){if(i==warmup)start=std::chrono::steady_clock::now();dispatches=barriers=copies=fills=binds=pushes=0;hot=true;observed_detail::record_graph(VK_NULL_HANDLE,VK_NULL_HANDLE,kernels,VK_NULL_HANDLE,p,VK_NULL_HANDLE,VK_NULL_HANDLE,g.compact_bytes);hot=false;
assert(dispatches==6&&barriers==8&&copies==1&&fills==1&&binds==1&&pushes==1);assert(copy_bytes==64+80*c&&fill_bytes==copy_bytes);assert(observed_groups[0]==(g.count+255)/256&&observed_groups[1]==(c+255)/256&&observed_groups[2]==observed_groups[0]&&observed_groups[3]==observed_groups[1]&&observed_groups[4]==observed_groups[0]&&observed_groups[5]==1);}
timings[variant]=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/iterations;}
assert(allocations==0);ObservedLayout full(640,480);assert(3*(full.arena_bytes+2*full.compact_bytes)+48==46203312);
std::printf("{\"iterations_per_variant\":100000,\"warmup\":1000,\"encoder_ns_64x48_C32\":%.6f,\"encoder_ns_640x480_C4096\":%.6f,\"hot_new_calls\":%u,\"dispatches_per_frame\":6,\"barriers_per_frame\":8,\"compact_copy_bytes_C4096\":327744,\"three_slot_own_payload_bytes\":46203312}\n",timings[0],timings[1],allocations);}
'''.replace('BARRIER_BODY', body(cpp, 'void barrier(')).replace('ENCODER_BODY', body(cpp, 'void record_graph('))
    source, binary = tmp_path / 'encoder.cpp', tmp_path / 'encoder'
    source.write_text(code)
    command = [os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-Wall', '-Wextra',
               '-Werror', '-Wno-missing-field-initializers', '-I' + str(P / 'include'), str(source), '-o', str(binary)]
    built = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert built.returncode == 0, built.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    proof = json.loads(result.stdout)
    assert proof['hot_new_calls'] == 0
    assert proof['three_slot_own_payload_bytes'] == 46203312
    proof['source_sha256'] = hashlib.sha256(cpp.encode()).hexdigest()
    proof['benchmark_sha256'] = hashlib.sha256(code.encode()).hexdigest()
    proof['scope'] = 'actual encoder body; counter-only Vulkan stubs, no GPU/driver cost'
    print(json.dumps(proof, sort_keys=True))
    if os.environ.get('OBSERVED_REVIEW_ARTIFACT_DIR'):
        dest = Path(os.environ['OBSERVED_REVIEW_ARTIFACT_DIR'])
        dest.mkdir(parents=True, exist_ok=True)
        (dest / 'encoder-benchmark.cpp').write_text(code)
        (dest / 'encoder-benchmark.json').write_text(json.dumps(proof, indent=2) + '\n')


@pytest.mark.parametrize('width,height,capacity', [(64, 48, 32), (640, 480, 4096)])
def test_actual_module_submit_ready_finish_hot_cost(tmp_path, width, height, capacity):
    """Reuse author's synthetic admission fixture, independently instrument it.

    Real production modules and lease methods run. Fake GPU fill/copy commands
    touch only the 64B synthetic header during timing, never a CPU full-frame
    payload copy. This measures host submission/lease overhead, not transfers.
    """
    fixture = P / 'test/observed_children_host_contract.cpp'
    original = fixture.read_text()
    source = original[:original.index('int main(')]
    edits = [
        ('#define main inherited_child_contract_main', '#define vkCmdFillBuffer inherited_vkCmdFillBuffer\n#define main inherited_child_contract_main'),
        ('#undef main', '#undef main\n#undef vkCmdFillBuffer'),
        ('unsigned mode=0;', 'bool perf_hot=false;uint64_t perf_copy_calls=0,perf_fill_calls=0;unsigned mode=0;'),
        ('extern "C" {\nvoid vkUpdateDescriptorSets', 'extern "C" {\nvoid vkCmdFillBuffer(VkCommandBuffer cmd,VkBuffer b,VkDeviceSize offset,VkDeviceSize size,uint32_t value){if(perf_hot){++perf_fill_calls;require(offset==0&&value==0);std::memset(fake_memory[uintptr_t(b)].data,0,64);}else inherited_vkCmdFillBuffer(cmd,b,offset,size,value);}\nvoid vkUpdateDescriptorSets'),
        (' for(unsigned i=0;i<n;i++)std::memcpy(static_cast<char*>(fake_memory[uintptr_t(to)].data)+copies[i].dstOffset,static_cast<char*>(fake_memory[uintptr_t(from)].data)+copies[i].srcOffset,copies[i].size);last_readback=to;',
         ' if(perf_hot){++perf_copy_calls;require(n==1&&copies[0].size==64+80*observed_parameters[3]);}for(unsigned i=0;i<n;i++)std::memcpy(static_cast<char*>(fake_memory[uintptr_t(to)].data)+copies[i].dstOffset,static_cast<char*>(fake_memory[uintptr_t(from)].data)+copies[i].srcOffset,perf_hot?64:copies[i].size);last_readback=to;'),
    ]
    for old, new in edits:
        assert source.count(old) == 1, old
        source = source.replace(old, new)
    source += r'''
int main(int argc,char** argv){try{
 require(argc==5);ParentConfig pc;pc.width=WIDTH;pc.height=HEIGHT;pc.min_parent_pixels=1;pc.record_capacity=CAPACITY;pc.calibration_generation=7;pc.generation=11;
 ChildConfig cc;cc.generation=13;cc.min_child_pixels=8;cc.record_capacity=CAPACITY;ObservedConfig oc;oc.generation=17;
 auto context=std::make_shared<Context>();context->enabled_features.features.shaderFloat64=VK_TRUE;context->enabled_features.features.shaderInt64=VK_TRUE;
 auto pool=std::make_shared<FramePool>(WIDTH,HEIGHT,context,false);auto frame=pool->acquire();source_metadata(frame,pc);
 ParentPreparation parents(context,pool->source_token(),pc,argv[1]);auto parent=parents.submit(frame);parent_result(1,WIDTH,HEIGHT);
 ChildPreparation children(context,pool->source_token(),pc,cc,argv[2]);clear_result();auto child=children.submit(parent);
 GeometryCalibration gc;gc.width=WIDTH;gc.height=HEIGHT;gc.generation=7;gc.camera_rays.resize(WIDTH*HEIGHT);gc.depth_scale_m=double(frame.slot().calibration.depth_scale);gc.board_from_camera={1,0,0,0,0,1,0,0,0,0,1,0};
 GeometryPreparation geometry(context,gc,argv[3]);auto geo=geometry.submit(frame);
 ObservedPreparation observed(context,pool->source_token(),geometry.producer_token(),pc,cc,oc,argv[4]);
 for(unsigned i=0;i<1000;i++){auto a=observed.submit(child,geo);require(observed.ready(a));observed.finish(a);}
 auto heaps=heap_allocations,buffers=buffer_allocations,waits=wait_calls;auto before=dispatches;require(heaps>0&&buffers>0);
 perf_hot=true;auto begin=std::chrono::steady_clock::now();
 for(unsigned i=0;i<100000;i++){auto a=observed.submit(child,geo);require(observed.ready(a)&&observed.finish(a).record_count==0);}
 auto elapsed=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-begin).count();perf_hot=false;
 require(heap_allocations==heaps&&buffer_allocations==buffers&&wait_calls==waits&&dispatches-before==600000);
 require(perf_copy_calls==100000&&perf_fill_calls==100000);
 std::printf("{\"iterations\":100000,\"ns_submit_ready_finish_release\":%.6f,\"hot_heap\":0,\"hot_buffer_allocations\":0,\"hot_waits\":0,\"dispatches\":600000,\"compact_copies\":100000,\"compact_fills\":100000}\n",elapsed/100000);
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
'''.replace('WIDTH', str(width)).replace('HEIGHT', str(height)).replace('CAPACITY', str(capacity))
    source_file, binary = tmp_path / 'host-perf.cpp', tmp_path / 'host-perf'
    source_file.write_text(source)
    if os.environ.get('OBSERVED_REVIEW_ARTIFACT_DIR'):
        dest = Path(os.environ['OBSERVED_REVIEW_ARTIFACT_DIR'])
        dest.mkdir(parents=True, exist_ok=True)
        (dest / f'host-{width}x{height}-C{capacity}.cpp').write_text(source)
    modules = ['src/observed_children.cpp', 'src/children.cpp', 'src/parents.cpp',
               'src/geometry.cpp', 'src/frame_pool.cpp']
    command = [os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-pthread',
               '-I' + str(P / 'include'), '-I' + str(P / 'test'), str(source_file),
               *[str(P / s) for s in modules], '-o', str(binary)]
    built = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stderr
    # Names come from actual encoders/constructor, not an alternative protocol.
    dirs = []
    entries = [
        ('parents', ['parents_init', 'parents_hook', 'parents_flatten', 'parents_validate', 'parents_scan', 'parents_offsets', 'parents_pixels', 'parents_labels', 'parents_records']),
        ('children', ['child_lookup', 'child_init', 'child_hook', 'child_jump', 'child_validate', 'child_reduce', 'child_gather', 'child_radix_hist', 'child_radix_offsets', 'child_radix_scatter', 'child_scatter', 'child_pack_scan', 'child_pack_offsets', 'child_jobs', 'child_packed_init', 'child_fill_hook', 'child_fill_jump', 'child_fill_validate', 'child_boundary', 'child_support', 'child_finalize']),
        ('geometry', ['geometry_coordinates', 'geometry_support_scan', 'geometry_support_offsets', 'geometry_support_scatter', 'geometry_valid_count']),
        ('observed', ['observed_clear', 'observed_records', 'observed_observe', 'observed_eligible', 'observed_close', 'observed_finalize']),
    ]
    for group, names in entries:
        dest = tmp_path / group
        dest.mkdir()
        dirs.append(str(dest))
        for name in names:
            (dest / (name + '.spv')).write_bytes(b'\x03\x02\x23\x07')
    result = subprocess.run([str(binary), *dirs], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    proof = json.loads(result.stdout.splitlines()[0])
    proof.update(width=width, height=height, capacity=capacity,
                 fixture_sha256=hashlib.sha256(original.encode()).hexdigest(),
                 instrumented_fixture_sha256=hashlib.sha256(source.encode()).hexdigest(),
                 source_sha256={name: hashlib.sha256((P / name).read_bytes()).hexdigest() for name in modules},
                 scope='actual modules; synthetic empty GPU results; counter commands and 64B synthetic header copy only; no driver/GPU/transfer timing')
    print(json.dumps(proof, sort_keys=True))
    if os.environ.get('OBSERVED_REVIEW_ARTIFACT_DIR'):
        dest = Path(os.environ['OBSERVED_REVIEW_ARTIFACT_DIR'])
        dest.mkdir(parents=True, exist_ok=True)
        (dest / f'host-{width}x{height}-C{capacity}.cpp').write_text(source)
        (dest / f'host-{width}x{height}-C{capacity}.json').write_text(json.dumps(proof, indent=2) + '\n')


def test_owned_and_transitively_retained_payload_budget(tmp_path):
    geometry = (P / 'src/geometry.cpp').read_text()
    # Compile the real geometry layout body alongside the public integer layouts.
    source = r'''
#include "so101_station_vulkan/observed_children.hpp"
#include "so101_station_vulkan/child_graph.hpp"
#include <iostream>
using namespace so101_station_vulkan;
struct Parameters{uint32_t width,height,count,grid_count,grid_width,blocks;};
Parameters dimensions(unsigned w,unsigned h){DIMENSIONS}
size_t geometry_bytes(unsigned width,unsigned height){GEOMETRY_BYTES}
int main(){ObservedLayout o(640,480);ChildGraphLayout c(640,480);ParentGraphLayout p(640,480);
size_t own=3*(o.arena_bytes+2*o.compact_bytes)+48;
size_t child=3*c.total_bytes();
size_t parent=3*(p.arena_bytes()+2*(64+32*4096)+sizeof(ParentRecord)*4096);
size_t geo=3*(geometry_bytes(640,480)+16)+16*307200+152;
size_t frame=3*(2*input_bytes(640,480)+scratch_bytes(640,480)+2*compact_bytes()+28*307200+sizeof(Component)*4096);
std::cout<<own<<" "<<child<<" "<<parent<<" "<<geo<<" "<<frame<<" "<<own+child+parent+geo+frame<<"\n";}
'''.replace('DIMENSIONS', body(geometry, 'Parameters dimensions(')).replace('GEOMETRY_BYTES', body(geometry, 'size_t geometry_arena_bytes('))
    cpp, binary = tmp_path / 'budget.cpp', tmp_path / 'budget'
    cpp.write_text(source)
    built = subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                            '-I' + str(P / 'include'), str(cpp), '-o', str(binary)],
                           capture_output=True, text=True, timeout=30)
    assert built.returncode == 0, built.stderr
    actual = list(map(int, subprocess.check_output([str(binary)], text=True, timeout=5).split()))
    # Original FramePool GPU mode, diagnostics=false; no duplicated lease copies
    # or driver object/allocation-alignment overhead. Existing source pool stays
    # alive through child/parent/geometry storage; it is not a new observed cost.
    assert actual == [46203312, 73615344, 41759232, 31181456, 102850656, 295610000]


def test_native_geometry_requires_int64_before_any_buffer_allocation():
    cpp, header, child, geometry = sources()
    old = 'if(!context->enabled_features.features.shaderInt64)throw std::runtime_error("geometry unavailable: native byte-address Float64 requires enabled shaderInt64");'
    assert old in geometry
    with pytest.raises(AssertionError):
        cpp_guard(cpp, header, child, geometry.replace(old, ''))
