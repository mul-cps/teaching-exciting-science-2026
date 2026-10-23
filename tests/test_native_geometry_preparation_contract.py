"""CPU/static contract gates; these do not execute a GPU or enumerate devices."""
from pathlib import Path
import re
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'


def test_prepare_submit_has_no_hot_path_allocations_readbacks_or_waits():
    source = (PACKAGE/'src/geometry.cpp').read_text()
    submit = source.split('GeometryLease GeometryPreparation::submit(', 1)[1].split('bool GeometryPreparation::ready(', 1)[0]
    for forbidden in ('vkWait', 'vkDeviceWaitIdle', 'vkQueueWaitIdle', '.create(', 'vkAllocate',
                      'new ', 'make_shared', 'vkCmdCopyBuffer', 'std::vector', 'vkMapMemory'):
        assert forbidden not in submit, forbidden
    assert 'source.readback.mapped' in submit  # existing compact status only
    assert 'std::array<VkWriteDescriptorSet,4>' in submit
    finish = source.split('GeometryCounts GeometryPreparation::finish(', 1)[1]
    assert '.counts.mapped' in finish and '.arena.mapped' not in source
    assert 'shaderFloat64' in source
    assert 'calibration_generation!=s.generation' in submit
    assert 'source.calibration.generation!=s.generation' in submit


def test_support_scan_is_ordered_bounded_and_empty_lanes_participate():
    shader = (PACKAGE/'shaders/geometry.slang').read_text()
    scan = shader.split('void geometry_support_scan(', 1)[1].split('[numthreads(1,1,1)]', 1)[0]
    assert 'return' not in scan  # no skipped workgroup barrier on tail lanes
    assert 'scan0[256]' in shader and 'scan1[256]' in shader
    assert 'for(uint stride=1;stride<256;stride*=2)' in scan
    assert 'rank-1:0xffffffff' in scan
    assert 'InterlockedAdd' not in scan  # atomic order may never select sample order
    source = (PACKAGE/'src/geometry.cpp').read_text()
    assert 'w>640||h>480' in source
    assert 'counts[1]>4800' in source


def test_cpu_scan_proof_covers_all_4800_samples_and_partial_groups():
    rng = np.random.default_rng(20261022)
    for count in (1, 255, 256, 257, 4800):
        for flags in (np.zeros(count, bool), np.ones(count, bool), rng.random(count)>.7):
            nblocks = (count+255)//256
            local_ranks = np.full(count, -1, np.int32)
            block_counts = []
            for block in range(nblocks):
                start = block*256
                lanes = np.zeros(256, np.uint32)
                lanes[:min(count-start, 256)] = flags[start:start+256]
                side = lanes.copy()
                for stride in (1, 2, 4, 8, 16, 32, 64, 128):
                    updated = side.copy(); updated[stride:] += side[:-stride]; side = updated
                valid = min(count-start, 256)
                local_ranks[start:start+valid] = np.where(lanes[:valid], side[:valid].astype(np.int64)-1, -1)
                block_counts.append(int(side[-1]))
            offsets = np.cumsum([0]+block_counts[:-1])
            selected = np.full(sum(block_counts), -1, np.int32)
            for i in range(count):
                if local_ranks[i]>=0:
                    selected[offsets[i//256]+local_ranks[i]] = i
            np.testing.assert_array_equal(selected, np.flatnonzero(flags))
