"""Resource-flow guards for the isolated diagnostic; no Vulkan code executes."""
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tools/performance/qualify_soft_binary64.cpp'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start + 1:end]
    raise AssertionError('unbalanced source')


def check_resource_flow(source):
    run = body(source, 'int run(')
    batch = body(run, 'for(size_t base=0;')
    setup = run[:run.index('for(size_t base=0;')]
    assert 'constexpr uint32_t kBatch=16384,kGroup=256;' in source
    assert 'vkDeviceWaitIdle' not in run and 'vkQueueWaitIdle' not in run
    assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|\.create\s*\(|vkCreate\w*\s*\(|vkAllocate\w*\s*\(|\.resize\s*\(|\.reserve\s*\(|push_back\s*\(', batch)
    assert setup.count('p.input.create(') == setup.count('p.output.create(') == 1
    assert setup.count('vkAllocateCommandBuffers(') == setup.count('vkAllocateDescriptorSets(') == 1
    assert setup.count('vkCreateFence(') == setup.count('vkCreateCommandPool(') == 1
    assert batch.count('vkQueueSubmit(') == batch.count('vkCmdDispatch(') == batch.count('vkWaitForFences(') == 1
    assert 'vkWaitForFences(d,1,&p.fence,VK_TRUE,10000000000ULL)' in batch
    assert batch.index('vkResetCommandPool(') < batch.index('vkQueueSubmit(') < batch.index('p.in_flight=true') < batch.index('vkWaitForFences(') < batch.index('p.in_flight=false') < batch.index('p.output.mapped);')
    failed_wait = body(batch, 'if(completion!=VK_SUCCESS&&completion!=VK_ERROR_DEVICE_LOST)')
    assert 'std::_Exit(2)' in failed_wait
    assert 'if(in_flight)std::_Exit(2)' in body(source, '~Probe()')
    assert run.index('SOFT_BINARY64_EXTERNAL_WATCHDOG') < run.index('Probe p;')
    assert 'kBatch*24' in setup and 'kBatch*32' in setup
    assert 'std::vector<uint8_t> output_bytes(c.rows.size()*32)' in setup
    assert 'count=uint32_t(std::min<size_t>(kBatch,c.rows.size()-base))' in batch
    assert 'VK_ACCESS_HOST_WRITE_BIT' in batch and 'VK_ACCESS_HOST_READ_BIT' in batch


def test_bounded_reusable_probe_resource_flow():
    check_resource_flow(SOURCE.read_text())


@pytest.mark.parametrize('before,after', [
    ('10000000000ULL', 'UINT64_MAX'),
    ('std::memset(p.output.mapped', 'vkDeviceWaitIdle(d);std::memset(p.output.mapped'),
    ('std::memset(p.output.mapped', 'p.input.create(*p.context,1,0,0);std::memset(p.output.mapped'),
    ('vkCmdDispatch(command,', 'vkCmdDispatch(command,1,1,1);vkCmdDispatch(command,'),
    ('if(in_flight)std::_Exit(2)', 'if(in_flight){}'),
])
def test_resource_guards_reject_unbounded_wait_allocation_submit_or_pending_free(before, after):
    source = SOURCE.read_text()
    assert before in source
    with pytest.raises(AssertionError):
        check_resource_flow(source.replace(before, after, 1))
