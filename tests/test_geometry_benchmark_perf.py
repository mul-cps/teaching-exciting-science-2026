"""Independent resource-flow guards and actual-loop synthetic CPU measurements."""
from pathlib import Path
import importlib.util
import json
import os
import re
import statistics
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'
SOURCE = PACKAGE/'test/benchmark_geometry.cpp'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start+1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start+1:end]
    raise AssertionError('unbalanced benchmark scope')


def resource_guard(source):
    one = body(source, 'Sample one(')
    run = body(source, 'Measurement run(')
    trial = body(run, 'for(uint32_t trial=')
    hot = (one, trial, body(source, 'uint64_t clock_ns('))
    forbidden = (r'\b(?:new|malloc|calloc|realloc)\b|std::vector|make_shared|'
                 r'\.create\(|\.resize\(|\.push_back\(|\.mapped|resident_buffer|'
                 r'vk(?:Allocate|Create|Map|Wait)|vk(?:Queue|Device)WaitIdle|'
                 r'vkCmdCopyBuffer|memcpy|read_file|fresh_file')
    for block in hot:
        code = re.sub(r'"(?:\\.|[^"\\])*"|//[^\n]*', '""', block)
        assert not re.search(forbidden, code)
    # The scalar orchestration may call only the reviewed leaves/methods. A
    # forbidden operation cannot escape into an unreviewed newly called helper.
    calls = set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', one))
    assert calls <= {'clock_ns', 'submit', 'index', 'while', 'ready', 'require',
                     'long', 'nanosleep', 'finish', 'slot'}
    calls = set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', trial))
    assert calls <= {'load', 'clock_ns', 'for', 'size_t', 'one', 'uint64_t', 'require'}
    calls = set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', body(source, 'uint64_t clock_ns(')))
    assert calls <= {'require', 'clock_gettime', 'uint64_t'}
    assert re.sub(r'\s+', '', body(source, 'void require(')) == 'if(!ok)throwstd::runtime_error(message);'
    assert 'c.poll_us>=1&&c.poll_us<=10000' in source
    assert 'kDeadlineNs=10000000000ULL' in source
    assert 'nanosleep(&remaining,&remaining)' in one and '}while(rc!=0);' in one
    assert 'sample.sleep_wall_ns+=after-before' in one and '++sample.sleep_calls' in one
    assert 'after-start<kDeadlineNs' in one
    assert one.index('pending.active=true') < one.index('geometry.submit(')
    poll_body = body(one, 'while(!geometry.ready(')
    assert 'pending.active=false' not in poll_body
    assert one.index('geometry.ready(') < one.index('pending.active=false') < one.index('geometry.finish(')
    assert one.index('geometry.finish(') < one.index('sample.latency_ns=clock_ns(CLOCK_MONOTONIC)-start')
    assert one.index('lease={}') < one.index('sample.cpu_ns=clock_ns(CLOCK_PROCESS_CPUTIME_ID)-cpu_start')
    assert 'sample.latency_ns<kDeadlineNs' in one
    assert 'std::_Exit(2)' in body(source, 'struct PendingExit')
    assert 'std::_Exit(2)' in body(body(source, '~SourceUpload('), 'if(pending)')
    upload = body(source, 'void complete(FrameLease& source')
    assert 'vkWaitForFences(context->device,1,&s.fence,VK_TRUE,kDeadlineNs)' in upload
    assert source.count('vkCmdCopyBuffer') == upload.count('vkCmdCopyBuffer')
    assert run.index('GEOMETRY_BENCHMARK_EXTERNAL_WATCHDOG') < run.index('Context::shared(')
    assert 'watchdog&&std::string(watchdog)=="1"' in run
    assert run.index('out.samples.resize') < run.index('Context::shared(')
    assert run.index('out.trials.resize') < run.index('Context::shared(')
    assert run.count('upload.complete(') == 1 and run.count('pool->acquire(') == 1
    warmup = run.index('for(uint32_t i=0;i<c.warmup;')
    measured = run.index('for(uint32_t trial=')
    assert warmup < measured
    assert 'clock_ns(CLOCK_PROCESS_CPUTIME_ID)' not in run[:measured]
    assert 'auto cpu_start=clock_ns(CLOCK_PROCESS_CPUTIME_ID)' in trial
    iterations = body(trial, 'for(uint32_t i=0;i<c.iterations;')
    assert 'sample=one(geometry,source,c)' in iterations
    assert 'c.cpu_budget_ms*1000000ULL' in iterations
    assert trial.index('sample=one(') < trial.index('measured.cpu_ns=clock_ns(CLOCK_PROCESS_CPUTIME_ID)-cpu_start')
    assert 'cpp_allocations.load' in trial and 'context->allocation_count.load' in trial
    assert '!measured.cpp_new_calls&&!measured.buffer_allocations' in trial


def test_benchmark_resource_and_measurement_source_guard():
    resource_guard(SOURCE.read_text())


MUTATIONS = [
    ('device_wait', 'Sample sample;PendingExit pending;',
     'Sample sample;PendingExit pending;vkDeviceWaitIdle(VK_NULL_HANDLE);'),
    ('heap_allocation', 'Sample sample;PendingExit pending;',
     'Sample sample;PendingExit pending;std::vector<int> extra(3);'),
    ('dense_readback', 'Sample sample;PendingExit pending;',
     'Sample sample;PendingExit pending;vkCmdCopyBuffer(command,a,b,1,&copy);'),
    ('sleep_remainder', 'nanosleep(&remaining,&remaining)', 'nanosleep(&remaining,nullptr)'),
    ('zero_poll_budget', 'c.poll_us>=1&&', ''),
    ('latency_before_finish', 'auto counts=geometry.finish(lease);sample.latency_ns=clock_ns(CLOCK_MONOTONIC)-start;',
     'sample.latency_ns=clock_ns(CLOCK_MONOTONIC)-start;auto counts=geometry.finish(lease);'),
    ('trial_cpu_boundary', 'auto cpu_start=clock_ns(CLOCK_PROCESS_CPUTIME_ID);\n  for(uint32_t i=0;i<c.iterations;',
     'auto cpu_start=clock_ns(CLOCK_MONOTONIC);\n  for(uint32_t i=0;i<c.iterations;'),
    ('pending_cleanup', 'std::_Exit(2);}}};', 'return;}}};'),
]


@pytest.mark.parametrize('name,old,new', MUTATIONS, ids=[m[0] for m in MUTATIONS])
def test_resource_guard_rejects_in_memory_mutants(name, old, new):
    source = SOURCE.read_text()
    assert source.count(old) == 1, name
    with pytest.raises(AssertionError):
        resource_guard(source.replace(old, new))


def test_guard_rejects_cpu_clock_moved_before_warmup():
    source = SOURCE.read_text()
    original = body(source, 'Measurement run(')
    timer = 'auto cpu_start=clock_ns(CLOCK_PROCESS_CPUTIME_ID);'
    warmup = 'for(uint32_t i=0;i<c.warmup;'
    assert original.count(timer) == original.count(warmup) == 1
    changed = original.replace(timer, '').replace(warmup, timer+warmup)
    with pytest.raises(AssertionError):
        resource_guard(source.replace(original, changed))


def author_fixture():
    path = ROOT/'tests/test_geometry_benchmark.py'
    spec = importlib.util.spec_from_file_location('geometry_benchmark_author_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def counter_stub(source):
    # Restore the real Buffer allocation counter semantics in the synthetic ABI.
    marker = '++buffer_allocations;owner='
    assert source.count(marker) == 1
    source = source.replace(marker, '++buffer_allocations;++c.allocation_count;owner=')
    # Interrupt every first sleep attempt in a separate profile; the actual
    # benchmark must retry its remainder rather than repoll before the budget.
    source = source.replace('#define main actual_benchmark_main', r'''#include <cerrno>
int perf_nanosleep(const timespec* requested,timespec* remaining){
 if(std::getenv("BENCH_FAKE_EINTR")){static bool retry=false;if(!retry){retry=true;if(remaining)*remaining=*requested;errno=EINTR;return -1;}retry=false;}
 return ::nanosleep(requested,remaining);
}
#define nanosleep perf_nanosleep
#define main actual_benchmark_main''')
    marker = 'if(live_command_pools)return 99;return rc;'
    assert source.count(marker) == 1
    return source.replace(marker, r'''std::cerr<<"BENCH_CPU_COUNTERS {\"wait_calls\":"<<wait_calls<<",\"buffer_allocations\":"<<buffer_allocations<<",\"submits\":"<<submits<<",\"live_command_pools\":"<<live_command_pools<<"}\n";if(live_command_pools)return 99;return rc;''')


@pytest.fixture(scope='module')
def synthetic(tmp_path_factory):
    module = author_fixture()
    directory = tmp_path_factory.mktemp('geometry-benchmark-perf')
    binary = module.build_synthetic_benchmark(directory, stub_transform=counter_stub)
    fixture, shaders = module.fixture_inputs(directory)
    return directory, binary, fixture, shaders


def actual_run(synthetic, label, iterations, trials, warmup, no_poll, interrupted=False):
    directory, binary, fixture, shaders = synthetic
    output = directory/(label+'.json')
    environment = dict(os.environ, GEOMETRY_BENCHMARK_EXTERNAL_WATCHDOG='1')
    if no_poll:
        environment['BENCH_FAKE_NO_POLL'] = '1'
    if interrupted:
        environment['BENCH_FAKE_EINTR'] = '1'
    result = subprocess.run([str(binary), '--run', '--software-binary64', str(fixture),
                             str(shaders), str(output), '--expected-valid', '0',
                             '--expected-support', '0', '--iterations', str(iterations),
                             '--trials', str(trials), '--warmup', str(warmup), '--poll-us', '50'],
                            env=environment, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report == json.loads(output.read_text())
    counters = json.loads(result.stderr.split('BENCH_CPU_COUNTERS ', 1)[1].splitlines()[0])
    assert counters['live_command_pools'] == 0
    assert counters['submits'] == 1+warmup+iterations*trials
    assert report['synthetic_driver'] and not report['gpu_executed']
    assert report['resident_source_uploads'] == report['max_in_flight'] == 1
    assert report['latency_clock'] == 'CLOCK_MONOTONIC'
    assert report['cpu_clock'] == 'CLOCK_PROCESS_CPUTIME_ID'
    assert len(report['samples']) == iterations*trials
    assert report['measured_process_cpu_ns'] == sum(t['process_cpu_ns'] for t in report['trial_samples'])
    assert all(t['cpp_new_calls'] == t['station_buffer_allocations'] == 0 for t in report['trial_samples'])
    assert all(s['arena'] == report['source_slot'] and s['valid'] == s['support'] == 0 for s in report['samples'])
    for key in ('qualified', 'unique_delivery', 'frame_age_qualified', 'capture_qualified',
                'full_geometry_qualified', 'throughput_qualified', 'performance_acceptance', 'full_station_qualified'):
        assert report[key] is False
    return report, counters


def test_actual_loop_cpu_host_cost_and_fixed_cold_resources(synthetic):
    short, short_counts = actual_run(synthetic, 'short', 2, 1, 1, True)
    ready, ready_counts = actual_run(synthetic, 'ready-host', 512, 4, 8, True)
    polling, polling_counts = actual_run(synthetic, 'polling-host', 32, 3, 4, False)
    interrupted, interrupted_counts = actual_run(synthetic, 'polling-eintr-host', 32, 3, 4, False, interrupted=True)
    # Actual orchestration scales to 2048 measured iterations without extra waits
    # or buffer allocation; counted waits belong only to cold upload/teardown.
    assert short_counts['wait_calls'] == ready_counts['wait_calls'] == polling_counts['wait_calls'] == interrupted_counts['wait_calls']
    assert short_counts['buffer_allocations'] == ready_counts['buffer_allocations'] == polling_counts['buffer_allocations'] == interrupted_counts['buffer_allocations']
    assert all(s['not_ready_polls'] == s['sleep_calls'] == s['observed_sleep_wall_ns'] == 0 for s in ready['samples'])
    assert all(s['not_ready_polls'] >= 1 and s['sleep_calls'] >= s['not_ready_polls']
               and s['observed_sleep_wall_ns'] >= s['not_ready_polls']*50000 for s in polling['samples'])
    assert all(s['sleep_calls'] >= 2*s['not_ready_polls']
               and s['observed_sleep_wall_ns'] >= s['not_ready_polls']*50000 for s in interrupted['samples'])
    measurements = {'scope': 'actual benchmark one/run + real GeometryPreparation/frame_pool; synthetic Vulkan only',
                    'real_driver_or_gpu_cost_included': False,
                    'hot_cpp_allocations': 0, 'hot_buffer_allocations': 0, 'hot_wait_calls': 0,
                    'cold_counters': {'short': short_counts, 'ready': ready_counts, 'polling': polling_counts, 'interrupted': interrupted_counts}}
    for label, report in [('ready', ready), ('polling', polling), ('polling_eintr', interrupted)]:
        n = len(report['samples'])
        measurements[label] = {'iterations': n,
                               'process_cpu_ns_per_iteration': report['measured_process_cpu_ns']/n,
                               'median_submit_finish_ns': statistics.median(s['submit_to_finish_ns'] for s in report['samples']),
                               'median_sample_process_cpu_ns': statistics.median(s['process_cpu_ns'] for s in report['samples']),
                               'median_observed_sleep_wall_ns': statistics.median(s['observed_sleep_wall_ns'] for s in report['samples']),
                               'requested_poll_sleep_us': report['requested_poll_sleep_us']}
    (synthetic[0]/'host-measurements.json').write_text(json.dumps(measurements, indent=2)+'\n')
