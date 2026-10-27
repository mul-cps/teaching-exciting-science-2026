"""Independent CPU arithmetic/cost guard for the shared Core division helper."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'
HELPER_SHA = '43281d1db2cc83f6d47cedcd56a838301030ce9449d4f99d860d66fd6c228cb3'


def body(source, name):
    start = source.index('{', source.index(name))
    depth = 1
    for end in range(start + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[start + 1:end]
    raise AssertionError('unbalanced body')


def test_shared_helper_retains_exact_bytes_and_six_quotients_with_reviewed_extents():
    helper = (P / 'shaders/exact_divide.slang').read_bytes()
    assert hashlib.sha256(helper).hexdigest() == HELPER_SHA
    alignment = (P / 'shaders/alignment.slang').read_text()
    resident = (P / 'shaders/resident.slang').read_text()
    include = '#include "exact_divide.slang"\n'
    assert alignment.count(include) == resident.count(include) == 1
    expanded = alignment.replace(include, helper.decode())
    # The prior expanded VGA shader identity was 00437cde16a72965cb7bb01f338b1b84968655337e58e657b9b74ce2e03a6061.
    # This reviewed identity adds independent depth/color extents and bounds;
    # the canonical helper bytes and projection arithmetic remain unchanged.
    assert hashlib.sha256(expanded.encode()).hexdigest() == 'c6bf1e52bcecd5e52ad61f1d1f138bfc0d20b56f96140a6155ff820c7d608535'
    assert alignment.count('alignment_divide(') == 4
    assert resident.count('alignment_divide(') == 2
    assert helper.decode().count('float alignment_divide(') == 1
    convert = body(resident, 'void convert(')
    assert 'precise float d=float(raw)*p.scale;' in convert
    assert 'precise float x=alignment_divide((float(i%p.width)-p.cx),p.fx);precise float y=alignment_divide((float(i/p.width)-p.cy),p.fy);' in convert
    assert 'float4(x*d,y*d,d,raw!=0?1.0:0.0)' in convert
    assert not re.search(r'\b(?:valid|maxDepth|clamp|saturate)\b', convert)
    for name in ('cpu_pipeline', 'alignment_reference', 'alignment'):
        assert 'exact_divide' not in (P / f'src/{name}.cpp').read_text()
    assert re.findall(r'for\s*\(([^)]*)\)', helper.decode()) == [
        'uint correction=0;correction<4;++correction', 'int bit=22;bit>=0;--bit',
        'uint shift=0;shift<23 && a<8388608u;++shift', 'uint shift=0;shift<23 && b<8388608u;++shift']
    assert not re.search(r'\b(?:double|uint64|int64|new|malloc|while|Interlocked\w*|\w*Barrier\w*)\b', helper.decode())


CPP = r'''
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cfenv>
#include <vector>
#include <algorithm>
#include <chrono>
#include <fstream>
using uint=uint32_t;
static uint asuint(float x){uint u;std::memcpy(&u,&x,4);return u;}
static float asfloat(uint u){float x;std::memcpy(&x,&u,4);return x;}
@@HELPER@@
struct float4{float x,y,z,w;float4(float a,float b,float c,float d):x(a),y(b),z(c),w(d){}};
struct Parameters{uint width=640;float scale=.001f,fx=610,fy=605,cx=319.7f,cy=239.2f,rcpx=0,rcpy=0;};
struct Sample{uint i,raw;Parameters p;};
__attribute__((noinline)) float4 exact(const Sample& s){uint i=s.i,raw=s.raw;const auto& p=s.p;@@DEPTH@@ @@EXACT@@ return @@OUTPUT@@;}
__attribute__((noinline)) float4 native(const Sample& s){uint i=s.i,raw=s.raw;const auto& p=s.p;volatile float fx=p.fx,fy=p.fy;@@DEPTH@@ @@NATIVE@@ return @@OUTPUT@@;}
__attribute__((noinline)) float4 reciprocal(const Sample& s){uint i=s.i,raw=s.raw;const auto& p=s.p;@@DEPTH@@ @@RECIPROCAL@@ return @@OUTPUT@@;}
static volatile uint checksum=0;
using Operation=float4(*)(const Sample&);
static double measure(const std::vector<Sample>& samples,Operation op){std::vector<double> times;for(unsigned repeat=0;repeat<7;repeat++){uint sum=0;auto start=std::chrono::steady_clock::now();for(const auto& s:samples){auto v=op(s);sum+=asuint(v.x)^asuint(v.y)^asuint(v.z)^asuint(v.w);}times.push_back(std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/samples.size());checksum=sum;}std::sort(times.begin(),times.end());return times[3];}
static void require(bool ok){if(!ok)std::exit(1);}
static Parameters parameters(unsigned variant=0){Parameters p;if(variant==1){p.fx=600;p.fy=600;}if(variant==2){p.fx=615.5f;p.fy=611.75f;}volatile float one=1,fx=p.fx,fy=p.fy;p.rcpx=one/fx;p.rcpy=one/fy;return p;}
static bool same(float4 a,float4 b){return asuint(a.x)==asuint(b.x)&&asuint(a.y)==asuint(b.y)&&asuint(a.z)==asuint(b.z)&&asuint(a.w)==asuint(b.w);}
template<class T>std::vector<T> load(const char* path,size_t count){std::vector<T> result(count);std::ifstream file(path,std::ios::binary|std::ios::ate);require(bool(file)&&size_t(file.tellg())==count*sizeof(T));file.seekg(0);file.read(reinterpret_cast<char*>(result.data()),count*sizeof(T));require(bool(file));return result;}
int main(int argc,char** argv){require(std::fesetround(FE_TONEAREST)==0);constexpr uint n=640*480;
 if(argc==4){auto raw=load<uint16_t>(argv[1],n);auto cpu=load<float>(argv[2],n*4),gpu=load<float>(argv[3],n*4);uint exact_cpu=0,native_cpu=0,rcp_gpu=0,over=0,pixels=0;for(uint i=0;i<n;i++){Sample s{i,raw[i],parameters()};auto e=exact(s),h=native(s),r=reciprocal(s);float ea[]={e.x,e.y,e.z,e.w},ha[]={h.x,h.y,h.z,h.w},ra[]={r.x,r.y,r.z,r.w};bool failing=false;for(uint a=0;a<4;a++){exact_cpu+=asuint(ea[a])!=asuint(cpu[4*i+a]);native_cpu+=asuint(ha[a])!=asuint(cpu[4*i+a]);rcp_gpu+=asuint(ra[a])!=asuint(gpu[4*i+a]);if(a<2&&std::abs(ea[a]-ra[a])>1e-7f){++over;failing=true;}}pixels+=failing;}std::printf("{\"pixels\":%u,\"float4_lanes\":%u,\"exact_cpu_bit_mismatches\":%u,\"native_cpu_bit_mismatches\":%u,\"rcp_gpu_unexplained_bits\":%u,\"old_rcp_coordinate_errors_over_1e_7\":%u,\"old_rcp_failing_pixels\":%u}\n",n,4*n,exact_cpu,native_cpu,rcp_gpu,over,pixels);return exact_cpu||native_cpu||rcp_gpu||over!=490||pixels!=490;}
 Sample recorded{655,3227,parameters()};auto e=exact(recorded),h=native(recorded),r=reciprocal(recorded);require(same(e,h)&&asuint(e.x)==0xbfce532bu&&asuint(r.x)==0xbfce532cu);
 std::printf("{\"cpu_only\":true,\"samples_per_case\":%u,\"repetitions\":7,\"statistic\":\"median\",\"units\":\"ns_per_xy_pair_with_depth_multiply\",\"cases\":{",n);
 for(unsigned scenario=0;scenario<2;scenario++){std::vector<Sample> samples;samples.reserve(n);for(uint i=0;i<n;i++){Sample s{i,scenario?65535u:(i%7?250u+(i*719u)%4250u:0u),parameters(i%3)};require(same(exact(s),native(s)));if(scenario)require(exact(s).z>65&&exact(s).w==1);samples.push_back(s);}double exact_ns=measure(samples,exact),hardware_ns=measure(samples,native),rcp_ns=measure(samples,reciprocal);std::printf("%s\"%s\":{\"exact_ns\":%.3f,\"native_ns\":%.3f,\"cached_rne_rcp_ns\":%.3f}",scenario?",":"",scenario?"far65535":"normal",exact_ns,hardware_ns,rcp_ns);}
 std::printf("},\"checked_exact_native_pairs\":%u,\"recorded_failure_bits_preserved\":true,\"far_points_preserved\":true,\"checksum\":%u}\n",2*n,checksum);
}
'''


def test_actual_core_expressions_and_canonical_helper_cpu_benchmark(tmp_path):
    source = (P / 'shaders/resident.slang').read_text()
    convert = body(source, 'void convert(')
    depth = re.search(r'precise float d=.*?;', convert).group().replace('precise ', '')
    declarations = re.search(r'precise float x=.*?;precise float y=.*?;', convert).group().replace('precise ', '')
    output = re.search(r'float4\(x\*d,y\*d,d,raw!=0\?1\.0:0\.0\)', convert).group()
    native = declarations.replace('alignment_divide((float(i%p.width)-p.cx),p.fx)', '(float(i%p.width)-p.cx)/fx').replace('alignment_divide((float(i/p.width)-p.cy),p.fy)', '(float(i/p.width)-p.cy)/fy')
    reciprocal = declarations.replace('alignment_divide((float(i%p.width)-p.cx),p.fx)', '(float(i%p.width)-p.cx)*p.rcpx').replace('alignment_divide((float(i/p.width)-p.cy),p.fy)', '(float(i/p.width)-p.cy)*p.rcpy')
    cpp = CPP
    for name, value in [('HELPER', (P / 'shaders/exact_divide.slang').read_text()), ('DEPTH', depth), ('EXACT', declarations), ('OUTPUT', output), ('NATIVE', native), ('RECIPROCAL', reciprocal)]:
        cpp = cpp.replace('@@' + name + '@@', value)
    generated = tmp_path / 'core_expression_benchmark.cpp'
    generated.write_text(cpp)
    compiler = shutil.which('c++')
    assert compiler
    binary = tmp_path / 'core_expression_benchmark'
    subprocess.run([compiler, '-O2', '-std=c++17', '-ffp-contract=off', '-fno-fast-math', str(generated), '-o', str(binary)], check=True, capture_output=True, text=True)
    report = json.loads(subprocess.check_output([str(binary)], text=True))
    assert report['checked_exact_native_pairs'] == 614400
    assert report['recorded_failure_bits_preserved'] and report['far_points_preserved']
    if destination := os.environ.get('SHARED_DIVIDE_PERF_ARTIFACTS'):
        destination = Path(destination)
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(generated, destination / generated.name)
        shutil.copy2(binary, destination / binary.name)
        (destination / 'microbenchmark.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, sort_keys=True))
