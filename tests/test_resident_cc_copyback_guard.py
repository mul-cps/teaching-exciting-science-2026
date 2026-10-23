"""CPU-only source contract for the transfer copyback candidate; no GPU work."""
import re
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def test_copyback_has_disjoint_full_parent_planes_and_both_memory_dependencies():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    assert 'auto copy_canonical_parents=' in source
    start = source.index('auto copy_canonical_parents=')
    end = source.index('\n };', start)
    copy = source[start:end]
    assert 'VkDeviceSize parent_bytes=VkDeviceSize(p.count)*sizeof(uint32_t)' in source
    assert 'VkBufferCopy parent_copy{17*parent_bytes,6*parent_bytes,parent_bytes}' in source
    # Same VkBuffer is legal only with disjoint source/destination ranges.
    for count in (1, 17*19, 640*480):
        size = count*4
        assert 6*size+size <= 17*size
        assert 17*size+size == 18*size == count*72  # scratch_bytes, no diagnostic needed
    assert re.search(r'barrier\(s.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_READ_BIT\|VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT\|VK_ACCESS_TRANSFER_WRITE_BIT\)', copy)
    assert 'vkCmdCopyBuffer(s.command,s.scratch.buffer,s.scratch.buffer,1,&parent_copy)' in copy
    assert re.search(r'barrier\(s.command,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_SHADER_READ_BIT\|VK_ACCESS_SHADER_WRITE_BIT\)', copy)
    assert copy.index('VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT') < copy.index('vkCmdCopyBuffer') < copy.index('VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT')
    assert 'for(unsigned i=0;i<32;i++){dispatch(3);copy_canonical_parents();dispatch(2);}dispatch(3)' in source
    # Copy stays device local. The existing upload/compact/diagnostic paths are separate.
    assert not re.search(r'Wait|Allocate|Create|MapMemory|readback|diagnostic', copy)


def test_scratch_allows_transfer_destination_only_in_cold_pool_construction():
    source = (PACKAGE / 'src/frame_pool.cpp').read_text()
    scratch = next(line for line in source.splitlines() if 's.scratch.create(' in line)
    for usage in ('STORAGE_BUFFER', 'TRANSFER_SRC', 'TRANSFER_DST'):
        assert f'VK_BUFFER_USAGE_{usage}_BIT' in scratch
    assert 'VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT' in scratch


def test_shader_inventory_and_production_fault_thresholds_are_frozen():
    import hashlib
    shader = (PACKAGE / 'shaders/resident.slang').read_bytes()
    assert hashlib.sha256(shader).hexdigest() == 'd53bc3fbe1dd1abac0f2e5f387fbb555d4a44b66cbe0af2bbf0bfc9fff1388df'
    header = (PACKAGE / 'include/so101_station_vulkan/pipeline.hpp').read_text()
    assert 'return size_t(w)*h*72;' in header
    assert 'kMaxComponents = 4096' in header or 'kMaxComponents=4096' in header


def test_final_download_dependency_includes_copied_parent_plane_writes():
    source = (PACKAGE / 'src/gpu_pipeline.cpp').read_text()
    assert 'barrier(s.command,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT)' in source
