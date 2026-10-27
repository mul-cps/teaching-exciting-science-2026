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
    helper = (PACKAGE / 'shaders/exact_divide.slang').read_text().split('// BEGIN EXACT ALIGNMENT DIVISION\n')[1].split('// END EXACT ALIGNMENT DIVISION')[0]
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
    point = point.replace('alignment_divide((float(i%p.color_width)-p.cx),p.fx)', '(float(i%p.color_width)-p.cx)/p.fx')
    point = point.replace('alignment_divide((float(i/p.color_width)-p.cy),p.fy)', '(float(i/p.color_width)-p.cy)/p.fy')
    assert 'tx=p.r0*x+(p.r3*y+(p.r6*z+p.t0));ty=p.r1*x+(p.r4*y+(p.r7*z+p.t1));tz=p.r2*x+(p.r5*y+(p.r8*z+p.t2));' in project
    assert 'tx=((p.r0*x+p.r3*y)+p.r6*z)+p.t0;ty=((p.r1*x+p.r4*y)+p.r7*z)+p.t1;tz=((p.r2*x+p.r5*y)+p.r8*z)+p.t2;' in project
    assert 'u=((tx/tz)*p.fx+p.cx)+0.5,v=((ty/tz)*p.fy+p.cy)+0.5' in project
    assert 'x=z*((float(i%p.color_width)-p.cx)/p.fx),y=z*((float(i/p.color_width)-p.cy)/p.fy)' in point
    assert helper.count('float alignment_divide(') == 1
    assert source.count('alignment_divide(') == 4
    core = (PACKAGE / 'shaders/resident.slang').read_text()
    for parent in (source, core):
        assert parent.count('#include "exact_divide.slang"') == 1
        assert 'float alignment_divide(' not in parent
        assert 'alignment_divide_bits' not in parent
    assert core.count('alignment_divide(') == 2
    convert = body(core, 'void convert(')
    for expression, original in (
        ('alignment_divide((float(i%p.width)-p.cx),p.fx)', '(float(i%p.width)-p.cx)/p.fx'),
        ('alignment_divide((float(i/p.width)-p.cy),p.fy)', '(float(i/p.width)-p.cy)/p.fy'),
    ):
        assert convert.count(expression) == 1
        convert = convert.replace(expression, original)
    import hashlib
    assert hashlib.sha256(convert.encode()).hexdigest() == '2dce81cf33e863b1e7f05ce3081cf593727f58252973fa265edaf5e9ff9d95bf'
    assert 'alignment_divide' not in (PACKAGE / 'src/cpu_pipeline.cpp').read_text()
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
    assert 'copy{0,0,depth_n*2}' in normal
    assert 'status_copy{color_n*4,0,4}' in normal
    assert '.diagnostic.buffer' not in normal
    assert 'for(' not in submit and 'while(' not in submit


def test_actual_helper_cpu_microbenchmark():
    source = (PACKAGE / 'shaders/alignment.slang').read_text()
    helper = (PACKAGE / 'shaders/exact_divide.slang').read_text().split('// BEGIN EXACT ALIGNMENT DIVISION\n')[1].split('// END EXACT ALIGNMENT DIVISION')[0]
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
        for parent, entry in (('alignment.slang', 'splat'), ('alignment.slang', 'finalize'), ('resident.slang', 'convert')):
            output = Path(directory) / (entry + '.spv')
            subprocess.run([slang, str(PACKAGE / 'shaders' / parent), '-entry', entry,
                            '-stage', 'compute', '-target', 'spirv', '-profile', 'sm_6_0',
                            '-fp-mode', 'precise', '-O2', '-o', str(output)], check=True,
                           capture_output=True, text=True)
            subprocess.run([str(tools / 'spirv-val'), '--target-env', 'vulkan1.1', str(output)], check=True)
            assembly = subprocess.check_output([str(tools / 'spirv-dis'), str(output)], text=True)
            if parent == 'alignment.slang':
                # The four extent words are appended to the shared 92-byte
                # host/kernel push ABI; do not silently slide a named field.
                extents = {'depth_width': (19,76), 'depth_height': (20,80),
                           'color_width': (21,84), 'color_height': (22,88)}
                for field, (index, offset) in extents.items():
                    member = re.search(r'OpMemberName\s+(%\w+)\s+'+str(index)+r'\s+"'+field+r'"', assembly)
                    assert member is not None, field
                    assert re.search(r'OpMemberDecorate\s+'+re.escape(member[1])+r'\s+'+str(index)+r'\s+Offset\s+'+str(offset)+r'\b', assembly), field
            capabilities = re.findall(r'OpCapability (\w+)', assembly)
            assert set(capabilities) == {'Shader'}
            assert not re.search(r'OpType(?:Int|Float) 64\b', assembly)
            arithmetic = set(re.findall(r'(%\w+)\s*=\s*Op(?:FAdd|FSub|FMul|FDiv|Dot)\b', assembly))
            decorated = set(re.findall(r'OpDecorate\s+(%\w+)\s+NoContraction', assembly))
            assert arithmetic <= decorated
            assert ' Fma ' not in assembly


def test_core_deprojection_causal_vectors_match_native_division():
    # Unchanged fixture 1 observations from the independent AMD/Intel cause
    # report, not a second arithmetic implementation. Keep division then depth
    # multiplication; the old reciprocal model crosses the existing 1e-7 gate.
    fixture = r'''
#include <cstdint>
#include <cstring>
#include <cmath>
using uint = uint32_t;
uint asuint(float f) {uint u;std::memcpy(&u,&f,4);return u;}
float asfloat(uint u) {float f;std::memcpy(&f,&u,4);return f;}
#include "alignment_divide_shared.h"
int main() {
 struct Case {uint numerator,depth,quotient,cpu,gpu;float focal;};
 const Case cases[] = {
  {0xc398599au,0x404e872cu,0xbeffbf8au,0xbfce532bu,0xbfce532cu,610.0f},
  {0x436acccdu,0x404d6042u,0x3ec6b4efu,0x3f9f699au,0x3f9f699bu,605.0f},
 };
 for(const auto& c:cases) {
  volatile float a=asfloat(c.numerator),b=c.focal,d=asfloat(c.depth);
  volatile float quotient=a/b;
  volatile float expected=quotient*d;
  volatile float actual=alignment_divide(a,b)*d;
  volatile float reciprocal=1.0f/b;
  volatile float old_quotient=a*reciprocal;
  volatile float old=old_quotient*d;
  if(asuint(quotient)!=c.quotient || asuint(expected)!=c.cpu ||
     asuint(actual)!=c.cpu || asuint(old)!=c.gpu ||
     std::fabs(old-expected)<=1e-7f) return 1;
 }
}
'''
    with tempfile.TemporaryDirectory(prefix='core-divide-regression-') as directory:
        directory = Path(directory)
        (directory / 'alignment_divide_shared.h').write_bytes((PACKAGE / 'shaders/exact_divide.slang').read_bytes())
        (directory / 'fixture.cpp').write_text(fixture)
        binary = directory / 'fixture'
        subprocess.run([shutil.which('c++'), '-O2', '-std=c++17', '-ffp-contract=off',
                        '-fno-fast-math', str(directory / 'fixture.cpp'), '-o', str(binary)],
                       check=True, capture_output=True, text=True)
        subprocess.run([str(binary)], check=True)
