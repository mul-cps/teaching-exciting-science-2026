"""Private failure-only probe resource guards; CPU evidence is not GPU qualification.

currentCPP01 can copy raw candidates only for unmarked EXACT4 AFTER graph;
marked diagnostic samples remain failures and unqualified.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tests/fixtures/child_hook_probe/children-8412aec1.slang'
OUTPUT = Path('/tmp/workshop-hook-probe-resource-20261027')
SOURCE_SHA = '8412aec1442aed563f99581a11bd3105db60b60947b5909ccf4e2e57ec0fe0ec'
PREFIX = '[numthreads(256,1,1)]\nvoid child_hook('


def function(source, name):
    start = source.index('void ' + name + '(')
    opening = source.index('{', start)
    level = 1
    end = opening + 1
    while level:
        level += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def guard(original, derived):
    """Only integer packing, one CAS, and the original fault OR are allowed."""
    assert hashlib.sha256(original.encode()).hexdigest() == SOURCE_SHA
    helper = function(derived, 'hookFaultSample')
    assert not re.search(r'Load|Store|Barrier|Wait|wait|fence|alloc|Buffer|Dispatch|Copy|new\b', helper)
    assert not re.search(r'\b(?:source|work|persistent|lookup|parentLabels|arguments)\b', helper)
    assert '[' not in helper and ']' not in helper
    assert re.findall(r'\b([A-Za-z_]\w*)\s*\(', helper) == [
        'hookFaultSample', 'uint', 'uint', 'InterlockedCompareExchange', 'fault']
    assert re.findall(r'Interlocked\w+', helper) == ['InterlockedCompareExchange']
    assert re.findall(r'compact\.\w+', helper) == ['compact.InterlockedCompareExchange']
    assert 'compact.InterlockedCompareExchange(28,0u,payload,ignored);' in helper
    assert helper.count('fault(4);') == 1
    assert helper.index('InterlockedCompareExchange') < helper.index('fault(4);')
    assert function(derived, 'fault') == function(original, 'fault')
    assert function(original, 'fault') == 'void fault(uint bit){uint ignored;compact.InterlockedOr(28,bit,ignored);}'
    start = original.index(PREFIX)
    end = original.index('\n', start + len(PREFIX))
    old = original[start:end]
    new = old.replace('fault(4)', 'hookFaultSample(i,root,par)', 1)
    assert old.count('fault(4)') == 1
    # Exact inverse proves no source loads, resource declarations, dispatches,
    # allocations, copies, fences, or any other changes outside these two sites.
    assert derived.count(helper + '\n') == 1
    assert derived.count('hookFaultSample(i,root,par)') == 1
    assert derived.replace(helper + '\n', '', 1).replace(new, old, 1) == original
    # Existing failure predicate and early return are byte-exact, including
    # short-circuit source reads; only this failure path invokes the helper.
    assert 'if(par==0||root>i+1||selectedParent(root-1)!=par){hookFaultSample(i,root,par);return;}' in new
    return helper


@pytest.fixture(scope='module')
def sources():
    OUTPUT.mkdir(exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix='resource-', dir=OUTPUT))
    spec = importlib.util.spec_from_file_location('resource_hook_probe', ROOT / 'tools/performance/child_hook_probe.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.prepare(SOURCE, directory / 'derived')
    return SOURCE.read_bytes().decode(), (directory / 'derived/children.slang').read_bytes().decode(), directory


def test_exact_failure_only_resource_delta(sources):
    guard(*sources[:2])


def test_manifest_requires_one_group_literal_fixture_scope(sources):
    manifest = json.loads((sources[2] / 'derived/manifest.json').read_text())
    assert manifest['required_fixture_sha256'] == 'e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd'
    assert (manifest['required_width'], manifest['required_height'], manifest['required_dispatch_groups']) == (7, 5, 1)
    assert manifest['qualifying'] is False
    assert any('8-bit i' in limit and 'one 256-lane' in limit for limit in manifest['limitations'])
    assert any('earlier nonzero fault prevents CAS capture' in limit for limit in manifest['limitations'])


@pytest.mark.parametrize('addition', [
    'uint extra=source.Load(0);',
    'uint extra=compact.Load(28);',
    'uint extra=compact[7];',
    'GroupMemoryBarrierWithGroupSync();',
    'wait();',
    'Fence();',
    'uint* memory=new uint[32];',
    'RWByteAddressBuffer extra;',
    'vkCmdCopyBuffer();',
    'Dispatch();',
    'compact.InterlockedOr(28,4u,ignored);',
    'compact.InterlockedCompareExchange(28,0u,payload,ignored);',
    'fault(4);',
    'uint extra=selectedParent(root-1);',
])
def test_reject_extra_load_wait_resource_or_atomic(sources, addition):
    original, derived, _ = sources
    mutated = derived.replace('    fault(4);', '    ' + addition + '\n    fault(4);', 1)
    assert mutated != derived
    with pytest.raises(AssertionError):
        guard(original, mutated)


@pytest.mark.parametrize('old,new', [
    ('uint ignored;compact.InterlockedOr(28,bit,ignored);', 'uint ignored;compact.InterlockedOr(28,4,ignored);'),
    ('if(par==0||root>i+1||selectedParent(root-1)!=par){hookFaultSample', 'if(true){hookFaultSample'),
    ('uint root=wl(p.count+i);', 'uint root=wl(p.count+i);uint extra=source.Load(0);'),
])
def test_reject_changes_outside_helper(sources, old, new):
    original, derived, _ = sources
    assert old in derived
    with pytest.raises(AssertionError):
        guard(original, derived.replace(old, new, 1))


@pytest.fixture(scope='module')
def actual_helper_result(sources):
    original, derived, directory = sources
    helper = guard(original, derived)
    fault = function(original, 'fault')
    # Paste the ACTUAL extracted Slang helper and fault function unchanged.
    # Only the compact object's ABI is faked with std::atomic<uint32_t>.
    unit = r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
using uint=uint32_t;
struct Compact {
 std::atomic<uint32_t> word{0};uint64_t cas=0,ors=0;
 void InterlockedCompareExchange(uint offset,uint expected,uint desired,uint& old){
  assert(offset==28);++cas;word.compare_exchange_strong(expected,desired);old=expected;
 }
 void InterlockedOr(uint offset,uint bits,uint& old){assert(offset==28);++ors;old=word.fetch_or(bits);}
} compact;
FAULT_FUNCTION
ACTUAL_HELPER
int main(){
 // A known first successful CAS wins. Every later sample loses its CAS but
 // retains the original OR operation. This is not wall-clock-first evidence.
 compact.word=0;hookFaultSample(37,7,3);const uint captured=0x81839284u;
 assert(compact.word==captured);
 for(uint j=0;j<512;j++){hookFaultSample(j,0xffffffffu,j);assert(compact.word==captured);}
 // Every prior low-fault mask: CAS captures only from exactly zero.
 for(uint mask=0;mask<128;mask++){
  compact.word=mask;auto a=compact.cas,b=compact.ors;hookFaultSample(37,7,3);
  assert(compact.word==(mask?mask|4u:captured));assert(compact.cas==a+1&&compact.ors==b+1);
 }
 // Independent known encodings include overflow and full uint32 input.
 const uint values[][4]={{0,0,0,0x80000004u},{255,255,63,0x9fffff84u},
  {256,256,64,0xe0000004u},{35,256,64,0xe0001184u},
  {0xffffffffu,0xffffffffu,0xffffffffu,0xffffff84u}};
 for(auto& v:values){compact.word=0;hookFaultSample(v[0],v[1],v[2]);assert(compact.word==v[3]);}
 // Subsequent legal OR fault masks never touch the captured high payload.
 for(uint mask=0;mask<128;mask++){
  compact.word=captured;fault(mask);assert(compact.word==(captured|mask));
  assert((compact.word&0xffffff80u)==(captured&0xffffff80u));
 }
 constexpr uint64_t n=500000;
 std::printf("{\"prior_masks\":128,\"or_masks\":128,\"overflow_cases\":5,\"winner_followups\":512,\"calls_per_trial\":%llu,\"trials\":[",(unsigned long long)n);
 for(uint trial=0;trial<5;trial++){
  compact.word=0;compact.cas=compact.ors=0;
  auto start=std::chrono::steady_clock::now();
  for(uint64_t j=0;j<n;j++)hookFaultSample(uint(j),uint(j+256),uint(j+64));
  auto end=std::chrono::steady_clock::now();
  assert(compact.cas==n&&compact.ors==n);
  double ns=std::chrono::duration<double,std::nano>(end-start).count()/n;
  std::printf("%s{\"cpu_ns_per_call\":%.6f,\"fake_atomic_operations\":%llu,\"operations_per_call\":2}",trial?",":"",ns,(unsigned long long)(compact.cas+compact.ors));
 }
 std::puts("]}");
}
'''.replace('FAULT_FUNCTION', fault).replace('ACTUAL_HELPER', helper)
    source = directory / 'actual_helper.cpp'
    source.write_text(unit)
    binary = directory / 'actual_helper'
    command = [shutil.which('g++'), '-std=c++17', '-O2', '-Wall', '-Wextra', str(source), '-o', str(binary)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    (directory / 'compile.log').write_text(compiled.stdout + compiled.stderr)
    compiled.check_returncode()
    ran = subprocess.run([str(binary)], capture_output=True, text=True, check=True, timeout=30)
    (directory / 'benchmark.json').write_text(ran.stdout)
    (directory / 'commands.json').write_text(json.dumps([command, [str(binary)]], indent=2))
    return json.loads(ran.stdout)


def test_actual_extracted_helper_atomic_semantics(actual_helper_result):
    assert actual_helper_result['prior_masks'] == 128
    assert actual_helper_result['or_masks'] == 128
    assert actual_helper_result['overflow_cases'] == 5
    assert actual_helper_result['winner_followups'] == 512


def test_actual_extracted_helper_cpu_benchmark_two_fake_atomics(actual_helper_result):
    """CPU ns/call only: no GPU performance, zero-cost, or FPS inference."""
    for trial in actual_helper_result['trials']:
        assert trial['cpu_ns_per_call'] > 0
        assert trial['operations_per_call'] == 2
        assert trial['fake_atomic_operations'] == 2 * actual_helper_result['calls_per_trial']


def test_current_cpp01_raw_fallback_only_unmarked_exact4_after_graph():
    source = (ROOT / 'tools/performance/qualify_children.cpp').read_text()
    assert 'ChildNonconvergence=4' in (ROOT / 'ros2/src/so101_station_vulkan/include/so101_station_vulkan/children.hpp').read_text().replace(' ', '')
    raw = source.index('if(w[7]==ChildNonconvergence&&q.hash==')
    assert source.index('k.execute(t,q);') < raw
    assert source.index('if(!compact_valid){compact_failure_header') < raw
    assert raw < source.index('require(compact_valid,"literal compact bounds/fault")')
    # Any marked word is nonzero and >127, therefore remains invalid and can
    # never satisfy the exact raw-fallback value 4 (no low-bit-only comparison).
    assert 'const bool compact_valid=!w[7]&&' in source
    assert 't.candidate_snapshot(' in source[raw:source.index('require(compact_valid,"literal compact bounds/fault")')]
