"""Source regression guards: CPU-only; GPU qualification is a separate gate."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'

def body(file, symbol):
    source = (ROOT / file).read_text()
    at = source.index(symbol)
    start = source.index('{', at)
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def test_frame_hot_paths_never_allocate_or_wait_for_device():
    for file, symbol in [('src/gpu_pipeline.cpp', 'void Pipeline::submit('),
                         ('src/gpu_pipeline.cpp', 'void Pipeline::finish('),
                         ('src/gpu_pipeline.cpp', 'void decode_compact('),
                         ('src/frame_pool.cpp', 'FrameLease FramePool::acquire('),
                         ('src/frame_pool.cpp', 'bool FramePool::ready('),
                         ('src/cpu_pipeline.cpp', 'void cpu_reference('),
                         ('src/cpu_pipeline.cpp', 'void cpu_morphology(')]:
        code = body(file, symbol)
        assert not re.search(r'vk(?:DeviceWaitIdle|QueueWaitIdle|WaitForFences|Allocate\w*|Create\w*|MapMemory)\s*\(', code), symbol
        assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|\.(?:resize|reserve|push_back|emplace_back)\s*\(', code), symbol


def test_submission_host_synchronization_and_bounded_readback():
    code = body('src/gpu_pipeline.cpp', 'void Pipeline::submit(')
    assert 'command_lock(c.command_mutex)' in code
    assert 'queue_lock(c.queue_mutex)' in code
    # Dense download is available solely behind explicit diagnostic mode.
    assert 'copy.size=s.compact.size;' in code
    assert re.search(r'if\(pool_->diagnostics\)\{[^}]*s\.diagnostic\.buffer', code)
    assert code.count('s.readback.buffer') == 1
    assert 'pool_->owns(f)' in code
    acquire = body('src/frame_pool.cpp', 'FrameLease FramePool::acquire(')
    assert 'compare_exchange_strong' in acquire
    assert acquire.index('vkGetFenceStatus') < acquire.index('s.submitted=false')


def test_cleanup_and_offline_shader_compile_contract():
    code = body('src/gpu_pipeline.cpp', 'Pipeline::Pipeline(')
    assert 'destroy_gpu();pool_.reset();divisions_.reset();context_.reset();' in code
    assert 'catch(...){release_resources();throw;}' in body('src/frame_pool.cpp', 'FramePool::FramePool(')
    assert 'k.stage.pName="main";' in body('src/gpu_pipeline.cpp', 'void Pipeline::create_gpu(')
    assert 'slangc' not in (ROOT / 'src/gpu_pipeline.cpp').read_text()


def test_component_color_uses_root_gather_without_per_pixel_scatter():
    shader = (ROOT / 'shaders/resident.slang').read_text()
    assert 'scratch.Store(a+28,m)' not in shader
    assert 'compact.Store(o+4,mask(i))' in shader


def test_parent_reads_and_atomic_hooks_have_disjoint_dispatch_phases():
    root = body('shaders/resident.slang', 'uint root(')
    compress = body('shaders/resident.slang', 'void compress(')
    connect = body('shaders/resident.slang', 'void connect(')
    assert 'forestParent(' in root
    assert 'InterlockedMin' not in root
    assert 'InterlockedAdd' not in root
    assert 'scratch.Store(canonicalOffset(i),root(i))' in compress
    assert 'forestParent(' not in connect and 'root(' not in connect
    assert 'canonicalParent(' in connect
    assert 'scratch.InterlockedMin(parentOffset(max(a,b)-1),min(a,b),ignored)' in connect
    assert 'InterlockedAdd(parentOffset' not in (ROOT / 'shaders/resident.slang').read_text()


def test_bounded_root_walk_faults_instead_of_returning_partial_geometry():
    root = body('shaders/resident.slang', 'uint root(')
    assert 'j<p.count' in root
    assert 'if(next==r)return r' in root
    assert 'if(next==0||next>=r)break' in root
    assert 'compact.InterlockedOr(4,4,ignored)' in root
    assert root.rstrip().endswith('return 0;\n}')
    assert 'if(s.fault)throw' in body('src/gpu_pipeline.cpp', 'void decode_compact(')


def test_tile_all_lanes_and_immutable_pingpong_cover_bounded_local_graph():
    shader = (ROOT / 'shaders/resident.slang').read_text()
    tile = body('shaders/resident.slang', 'void tiled_components(')
    assert 'return' not in tile
    assert 'tileLabels0[256]' in shader and 'tileLabels1[256]' in shader
    assert 'iteration<256' in tile
    assert 'if(source==0)tileLabels1[local]=next;else tileLabels0[local]=next' in tile
    assert tile.count('GroupMemoryBarrierWithGroupSync()') == 4
    assert 'uint changed=tileChanged' in tile
    assert 'if(!converged&&local==0)' in tile
    connect = body('shaders/resident.slang', 'void connect(')
    assert 'uint(nx)/16==x/16&&uint(ny)/16==y/16' in connect
    submit = body('src/gpu_pipeline.cpp', 'void Pipeline::submit(')
    assert 'dispatch(7);for(unsigned i=0;i<32;i++){dispatch(3);copy_canonical_parents();dispatch(2);}dispatch(3)' in submit
    assert 'if(i==7)vkCmdDispatch(s.command,(p.width+15)/16,(p.height+15)/16,1)' in submit


def test_closing_preserves_overlapping_roots_and_original_piece_borders():
    code = body('shaders/resident.slang', 'void morphology(')
    assert 'uint roots[9]' in code
    assert 'for(int dy=-1;dy<=1' in code
    assert 'if(qx<x0||qx>x1||qy<y0||qy>y1)continue' in code
    assert 'if(nx<x0||nx>x1||ny<y0||ny>y1)continue' in code
    assert 'if(p.round!=0)scratch.Store' in code
    assert 'scratch.InterlockedAdd(a+32,1,ignored)' in code
    assert 'dispatch(4);dispatch(6);dispatch(5)' in re.sub(r'stamp\(\d+\);', '', body('src/gpu_pipeline.cpp', 'void Pipeline::submit('))


def test_frozen_canonical_labels_remove_morphology_atomic_reads():
    for symbol in ('void reduce(', 'void morphology('):
        code = body('shaders/resident.slang', symbol)
        assert 'root(' not in code
        assert not re.search(r'(?<![A-Za-z])parent\(', code)
        assert 'canonicalParent(' in code
    submit = body('src/gpu_pipeline.cpp', 'void Pipeline::submit(')
    assert 'dispatch(3);copy_canonical_parents();dispatch(2);' in submit
    assert 'dispatch(4);dispatch(6);dispatch(5)' in re.sub(r'stamp\(\d+\);', '', submit)


def test_pixel_count_walk_bound_covers_long_immutable_chains():
    # A positive parent is at most its own one-based index. Every nonterminal
    # traversal strictly decreases r, regardless of concurrent monotone updates.
    n = 4096
    parents = [1] + list(range(1, n))
    r = parents[-1]
    visited = []
    for _ in range(n):
        visited.append(r)
        next_root = parents[r-1]
        if next_root == r:
            break
        assert 0 < next_root < r
        r = next_root
    else:
        raise AssertionError('pixel count bound exhausted on a valid monotone tree')
    assert r == 1
    assert len(visited) > 64
    assert all(b < a for a, b in zip(visited, visited[1:]))


def test_optional_timestamp_profiling_never_waits_or_allocates_per_frame():
    submit = body('src/gpu_pipeline.cpp', 'void Pipeline::submit(')
    finish = body('src/gpu_pipeline.cpp', 'void Pipeline::finish(')
    assert 'if(s.timestamps)vkCmdResetQueryPool' in submit
    assert 'VK_QUERY_RESULT_WAIT_BIT' not in finish
    assert 'VK_QUERY_RESULT_64_BIT' in finish
    assert finish.index('if(!ready(f))') < finish.index('vkGetQueryPoolResults')
    assert 'vkCreateQueryPool' not in submit + finish


def test_capacity_refusal_clears_count_before_root_map_use():
    code = body('src/cpu_pipeline.cpp', 'void cpu_reference(')
    assert 'if(s.component_count>=kMaxComponents){s.fault|=2;s.cpu_xyz[4*i+3]=0;continue;}' in code
    assert 'if(!index)continue;' in code
