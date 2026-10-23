"""Independent CPU-only performance guards for exact alignment division."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import json

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    start = source.index('{', source.index(signature))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start + 1:end]
    raise AssertionError('unbalanced source')


def test_exact_helper_bounded_cost_and_callsite_association():
    source = (PACKAGE / 'shaders/alignment.slang').read_text()
    helper = source.split('// BEGIN EXACT ALIGNMENT DIVISION\n')[1].split('// END EXACT ALIGNMENT DIVISION')[0]
    assert not re.search(r'\b(?:while|double|uint64|int64|new|malloc|Interlocked\w*|\w*Barrier\w*)\b', helper)
    assert re.findall(r'for\s*\(([^)]*)\)', helper) == [
        'uint correction=0;correction<4;++correction', 'int bit=22;bit>=0;--bit',
        'uint shift=0;shift<23 && a<8388608u;++shift',
        'uint shift=0;shift<23 && b<8388608u;++shift']
    assert re.sub(r'//[^\n]*', '', helper).count('/') == 1  # only native seed division
    assert 'uint seed=uint((float(a)/float(b))*8388608.0f)' in helper
    assert 'seed>=8388608u && seed<=16777216u' in helper
    assert 'remainder.hi==0 && remainder.lo<normalized_b' in helper
    assert 'result.whole=q;result.remainder=remainder.lo;return result;' in helper
    # Remove only the reviewed substitutions and compare full production body to
    # its frozen pre-fix shape, so transform/multiply associations cannot drift.
    project = body(source, 'int2 project(')
    point = body(source, 'float4 point(')
    project = project.replace('alignment_divide(tx,tz)', 'tx/tz').replace('alignment_divide(ty,tz)', 'ty/tz')
    point = point.replace('alignment_divide((float(i%640)-p.cx),p.fx)', '(float(i%640)-p.cx)/p.fx')
    point = point.replace('alignment_divide((float(i/640)-p.cy),p.fy)', '(float(i/640)-p.cy)/p.fy')
    assert 'tx=p.r0*x+(p.r3*y+(p.r6*z+p.t0));ty=p.r1*x+(p.r4*y+(p.r7*z+p.t1));tz=p.r2*x+(p.r5*y+(p.r8*z+p.t2));' in project
    assert 'tx=((p.r0*x+p.r3*y)+p.r6*z)+p.t0;ty=((p.r1*x+p.r4*y)+p.r7*z)+p.t1;tz=((p.r2*x+p.r5*y)+p.r8*z)+p.t2;' in project
    assert 'u=((tx/tz)*p.fx+p.cx)+0.5,v=((ty/tz)*p.fy+p.cy)+0.5' in project
    assert 'x=z*((float(i%640)-p.cx)/p.fx),y=z*((float(i/640)-p.cy)/p.fy)' in point
    assert source.count('alignment_divide(') == 5  # definition plus exactly four sites
    assert 'alignment_divide' not in (PACKAGE / 'src/alignment_reference.cpp').read_text()
    assert 'alignment_divide' not in (PACKAGE / 'src/alignment.cpp').read_text()


def test_normal_host_path_no_new_wait_allocation_or_readback():
    source = (PACKAGE / 'src/alignment.cpp').read_text()
    for signature in ('Alignment::acquire(', 'Alignment::submit(', 'Alignment::ready('):
        hot = body(source, signature)
        assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b|\.create\(|vk(?:Allocate|Create|Map|Wait)|vk(?:Queue|Device)WaitIdle|std::vector|make_shared', hot)
    submit = body(source, 'Alignment::submit(')
    diagnostics = body(submit, 'if(state_->diagnostics)')
    normal = submit.replace(diagnostics, '')
    assert normal.count('vkCmdCopyBuffer(') == 2  # fixed upload and 4-byte fault only
    assert 'copy{0,0,N*2}' in normal
    assert 'status_copy{N*4,0,4}' in normal
    assert '.diagnostic.buffer' not in normal
    assert 'for(' not in submit and 'while(' not in submit


def test_actual_helper_cpu_microbenchmark():
    source = (PACKAGE / 'shaders/alignment.slang').read_text()
    helper = source.split('// BEGIN EXACT ALIGNMENT DIVISION\n')[1].split('// END EXACT ALIGNMENT DIVISION')[0]
    compiler = shutil.which('c++')
    assert compiler
    with tempfile.TemporaryDirectory(prefix='alignment-divide-perf-') as directory:
        directory = Path(directory)
        (directory / 'alignment_divide_shared.h').write_text(helper)
        binary = directory / 'benchmark'
        subprocess.run([compiler, '-O2', '-std=c++17', '-ffp-contract=off', '-fno-fast-math',
                        '-I', str(directory), str(PACKAGE / 'test/alignment_divide_cpu_benchmark.cpp'),
                        '-o', str(binary)], check=True, capture_output=True, text=True)
        report = json.loads(subprocess.check_output([str(binary)], text=True))
        assert report['cpu_only'] and report['samples_per_case'] == 307200
        assert set(report['cases']) == {'projection', 'deprojection', 'quotient_native_seed', 'quotient_four_corrections', 'quotient_fallback_out_of_range', 'quotient_fallback_distant_seed'}
        for case in report['cases'].values():
            assert case['helper_ns_per_op'] > 0 and case['hardware_ns_per_op'] > 0
        print(json.dumps(report, sort_keys=True))


def test_offline_alignment_has_only_32bit_capabilities():
    # Opt in explicitly in compiler qualification; ordinary source-only test
    # environments need not carry a platform-specific shader toolchain.
    import os
    import pytest
    slang = os.environ.get('SLANGC_EXECUTABLE') or shutil.which('slangc')
    tools = Path(os.environ.get('SPIRV_TOOLS_DIR', '/usr/bin'))
    if not slang or not (tools / 'spirv-dis').exists() or not (tools / 'spirv-val').exists():
        pytest.skip('offline shader toolchain unavailable')
    with tempfile.TemporaryDirectory(prefix='alignment-divide-spirv-') as directory:
        for entry in ('splat', 'finalize'):
            output = Path(directory) / (entry + '.spv')
            subprocess.run([slang, str(PACKAGE / 'shaders/alignment.slang'), '-entry', entry,
                            '-stage', 'compute', '-target', 'spirv', '-profile', 'sm_6_0',
                            '-fp-mode', 'precise', '-O2', '-o', str(output)], check=True,
                           capture_output=True, text=True)
            subprocess.run([str(tools / 'spirv-val'), '--target-env', 'vulkan1.1', str(output)], check=True)
            assembly = subprocess.check_output([str(tools / 'spirv-dis'), str(output)], text=True)
            capabilities = re.findall(r'OpCapability (\w+)', assembly)
            assert set(capabilities) == {'Shader'}
            assert not re.search(r'OpType(?:Int|Float) 64\b', assembly)
            arithmetic = set(re.findall(r'(%\w+)\s*=\s*Op(?:FAdd|FSub|FMul|FDiv|Dot)\b', assembly))
            decorated = set(re.findall(r'OpDecorate\s+(%\w+)\s+NoContraction', assembly))
            assert arithmetic <= decorated
            assert ' Fma ' not in assembly
