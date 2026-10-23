"""CPU-only source contract for fused atomic copyback; no GPU work."""
import re
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def test_copyback_is_fused_atomic_work_without_bulk_parent_transfers():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    assert 'copy_canonical_parents' not in source and 'parent_copy' not in source
    assert 'vkCmdCopyBuffer(s.command,s.scratch.buffer,s.scratch.buffer' not in source
    assert 'for(unsigned i=0;i<32;i++){dispatch(3);dispatch(2);}dispatch(3)' in source
    assert source.count('vkCmdCopyBuffer(') == 4  # RGB/legacy upload, optional device depth, bounded/diagnostic outputs
    shader = (PACKAGE / 'shaders/resident.slang').read_text()
    start = shader.index('void connect(')
    end = shader.index('\n[numthreads', start)
    connect = shader[start:end]
    assert connect.index('scratch.InterlockedMin(parentOffset(i),a,ignored)') < connect.index('if((x&15u)')
    assert connect.count('scratch.InterlockedMin(') == 2
    assert 'forestParent(' not in connect and 'scratch.Store(' not in connect


def test_scratch_allows_transfer_destination_only_in_cold_pool_construction():
    source = (PACKAGE / 'src/frame_pool.cpp').read_text()
    scratch = next(line for line in source.splitlines() if 's.scratch.create(' in line)
    for usage in ('STORAGE_BUFFER', 'TRANSFER_SRC', 'TRANSFER_DST'):
        assert f'VK_BUFFER_USAGE_{usage}_BIT' in scratch
    assert 'VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT' in scratch


def test_shader_inventory_and_production_fault_thresholds_are_frozen():
    import hashlib
    shader = (PACKAGE / 'shaders/resident.slang').read_text()
    # Normalize only the approved shared include and two exact Core quotients.
    include = '#include "exact_divide.slang"\n'
    assert shader.count(include) == 1
    shader = shader.replace(include, '')
    for exact, original in (
        ('alignment_divide((float(i%p.width)-p.cx),p.fx)', '(float(i%p.width)-p.cx)/p.fx'),
        ('alignment_divide((float(i/p.width)-p.cy),p.fy)', '(float(i/p.width)-p.cy)/p.fy'),
    ):
        assert shader.count(exact) == 1
        shader = shader.replace(exact, original)
    start = shader.index('{', shader.index('void connect(')); end = start+1; level = 1
    while level:
        level += (shader[end] == '{') - (shader[end] == '}'); end += 1
    # All helpers/other stages are byte-frozen; only connect's approved body changes.
    normalized = shader[:start] + '{CONNECT_BODY}' + shader[end:]
    assert hashlib.sha256(normalized.encode()).hexdigest() == 'cc1c54770e601f6763221542560fc384fb018fae0088c3b0c21f5c93d3348df5'
    header = (PACKAGE / 'include/so101_station_vulkan/pipeline.hpp').read_text()
    assert 'return size_t(w)*h*72;' in header
    assert 'kMaxComponents = 4096' in header or 'kMaxComponents=4096' in header


def test_final_download_keeps_conservative_compute_transfer_visibility():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    assert 'barrier(s.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT)' in source
