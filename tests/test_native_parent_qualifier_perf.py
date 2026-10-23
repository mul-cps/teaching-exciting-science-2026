"""Bounded diagnostic resource-flow guards; never invokes --vulkan."""
from pathlib import Path
import re

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'tools/performance/qualify_parents.cpp'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start+1, len(source)):
        depth += (source[end] == '{')-(source[end] == '}')
        if not depth:
            return source[start+1:end]
    raise AssertionError('unbalanced diagnostic function')


def diagnostic_guard(source):
    load = body(source, 'Fixture load(')
    assert load.index('bytes>=48&&bytes<=48+640*480*4') < load.index('std::array<unsigned char,48>')
    assert load.index('q.width&&q.width<=640&&q.height&&q.height<=480') < load.index('q.mask.resize(n)')
    assert load.index('size_t(bytes)==48+4*n') < load.index('q.mask.resize(n)')
    assert 'q.capacity<=4096&&q.flags<=1' in load
    assert 'le32(' in load and 'le64(' in load and 'q.hash=hash_file(path)' in load
    main = body(source, 'int main(')
    assert 'require(argc==6&&std::string(argv[5])=="--external-watchdog-ack",' in main
    assert main.index('external-watchdog-ack') < main.index('auto q=load(argv[2]);return gpu(')
    assert '#ifdef PARENT_QUALIFIER_CPU_ONLY' in main
    assert 'CPU-only build cannot create a Vulkan device' in main
    gpu = body(source, 'int gpu(')
    assert gpu.index('missing source/build SHA256 tags') < gpu.index('Context::shared()')
    assert gpu.index('output prefix is not fresh') < gpu.index('Context::shared()')
    assert gpu.index('source/provenance disagree') < gpu.index('Context::shared()')
    assert 'auto* state=new GPUState' in gpu and 'auto& s=*state' in gpu
    catch = gpu.split('}catch(const std::exception& error)',1)[1]
    assert 'fatal("parent diagnostic failed;' in catch and 'delete state' not in catch
    assert 'std::_Exit(code)' in body(source, 'void fatal(')
    destructor = body(source, '~Transfer(')
    assert 'if(pending)fatal(' in destructor
    assert destructor.index('if(pending)fatal(') < destructor.index('vkDestroyFence')
    run = body(source, 'void run(VkFence target)')
    assert 'if(result!=VK_SUCCESS)fatal(' in run
    assert 'pending=target' in run and 'VK_TRUE,10000000000ULL' in run
    assert 'if(r!=VK_SUCCESS)fatal(' in run
    assert run.index('if(r!=VK_SUCCESS)fatal(') < run.index('pending=VK_NULL_HANDLE')
    assert 'throw' not in run
    assert not re.search(r'vk(?:Device|Queue)WaitIdle|UINT64_MAX', source)
    assert source.count('vkWaitForFences(') == 1
    complete = body(source, 'void complete(')
    assert 'std::chrono::seconds(10)' in complete and 'fatal(' in complete
    assert 'sleep_for(std::chrono::milliseconds(1))' in complete
    assert 'std::make_unique<Transfer>(c,q.mask.size()*4)' in gpu
    snapshot = body(source, 'void snapshot(')
    assert 'const ParentLease& lease' in source
    assert 'copy{lease.label_offset_bytes(),0,download.size}' in snapshot
    assert 'vkCmdCopyBuffer(command,lease.resident_buffer(),download.buffer,1,&copy)' in snapshot
    assert snapshot.index('run(fence)') > snapshot.index('vkCmdCopyBuffer(')
    valid = gpu.split('auto counts=parent.finish(retained);',1)[1]
    assert valid.index('transfer.snapshot(retained);') < valid.index('retained={}')
    assert valid.index('transfer.snapshot(retained);') < valid.index('fresh_file(prefix+".labels.bin"')
    assert 'fresh_file(prefix+".labels.bin",transfer.download.mapped,q.mask.size()*4)' in valid
    assert 'finish&&arena&&records_buffer&&mask&&records' in gpu
    assert 'parent.finish(retained)' in gpu and 'retained.original_mask_buffer()' in gpu
    assert 'O_WRONLY|O_CREAT|O_EXCL' in body(source, 'void fresh_file(')
    assert 'O_TRUNC' not in source
    assert 'requires_float64\\\":false' in source and 'requires_int64\\\":false' in source
    assert not re.search(r'require\([^;]*(?:shaderFloat64|shaderInt64)', gpu)
    assert 'parent_graph' not in source and 'ParentStorage' not in source



MUTANTS = [
    ('allocate_before_bounds', 'q.mask.resize(n);', 'q.mask.resize(n);',
     'size_t n=size_t(q.width)*q.height;', 'size_t n=size_t(q.width)*q.height;q.mask.resize(n);'),
    ('unbounded_fence', '10000000000ULL', 'UINT64_MAX', None, None),
    ('device_idle', 'void begin(){', 'void begin(){vkDeviceWaitIdle(context->device);', None, None),
    ('throw_pending_fence', 'if(r!=VK_SUCCESS)fatal(', 'if(r!=VK_SUCCESS)throw std::runtime_error(', None, None),
    ('free_pending_transfer', 'if(pending)fatal(', 'if(false)fatal(', None, None),
    ('free_state_on_exception', 'fatal("parent diagnostic failed;', 'delete state;fatal("parent diagnostic failed;', None, None),
    ('missing_watchdog_ack', 'require(argc==6&&std::string(argv[5])=="--external-watchdog-ack",', 'require(true,', None, None),
    ('early_lease_release', 'transfer.snapshot(retained);', 'retained={};transfer.snapshot(retained);', None, None),
    ('full_arena_readback', 'copy{lease.label_offset_bytes(),0,download.size}', 'copy{0,0,parent_arena_bytes(640,480)}', None, None),
    ('unbounded_transfer_size', 'std::make_unique<Transfer>(c,q.mask.size()*4)', 'std::make_unique<Transfer>(c,UINT64_MAX)', None, None),
    ('partial_negative_views', 'finish&&arena&&records_buffer&&mask&&records', 'finish', None, None),
    ('overwrite_output', 'O_WRONLY|O_CREAT|O_EXCL', 'O_WRONLY|O_CREAT|O_TRUNC', None, None),
    ('float64_admission', 'auto c=s.context;', 'auto c=s.context;require(c->enabled_features.features.shaderFloat64,"requires f64");', None, None),
]


def test_bounded_diagnostic_resource_flow():
    diagnostic_guard(SOURCE.read_text())


@pytest.mark.parametrize('name,old,new,extra_old,extra_new', MUTANTS, ids=[m[0] for m in MUTANTS])
def test_guard_rejects_diagnostic_resource_mutants(name, old, new, extra_old, extra_new):
    source = SOURCE.read_text()
    assert source.count(old) == 1, name
    mutant = source.replace(old, new)
    if extra_old:
        assert mutant.count(extra_old) == 1
        mutant = mutant.replace(extra_old, extra_new)
    with pytest.raises(AssertionError):
        diagnostic_guard(mutant)
