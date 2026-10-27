"""Exact integer div/sqrt CPU conformance; no Vulkan execution."""
from pathlib import Path
import os,subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'ros2/src/so101_station_vulkan'
HOST=P/'test/soft_binary64_divsqrt_host_test.cpp'
MODULE=P/'shaders/soft_binary64.slang'
FLAGS=['-std=c++17','-O2','-Wall','-Wextra','-ffp-contract=off','-fno-fast-math','-frounding-math']
def build(source,binary,extra=(),compiler=None):
 r=subprocess.run([compiler or os.environ.get('CXX','c++'),*FLAGS,*extra,str(source),'-o',str(binary)],capture_output=True,text=True);assert r.returncode==0,r.stderr

def test_missing_divsqrt_apis_and_native_conformance(tmp_path):
 binary=tmp_path/'host';build(HOST,binary)
 r=subprocess.run([str(binary)],capture_output=True,text=True,timeout=30);assert r.returncode==0,r.stdout+r.stderr
 assert 'random_pairs=1000000' in r.stdout

@pytest.mark.parametrize('mutation',[('division-sticky','quotient | uint64_t(remainder != 0)','quotient'),('sqrt-boundary','remainder >= trial','remainder > trial'),('sqrt-sticky','root | uint64_t(remainder != 0)','root')])
def test_behavioral_correction_mutants(tmp_path,mutation):
 name,old,new=mutation;source=MODULE.read_text();assert source.count(old)==1
 module=tmp_path/(name+'.slang');module.write_text(source.replace(old,new));host=tmp_path/(name+'.cpp');host.write_text(HOST.read_text().replace('"../shaders/soft_binary64.slang"','"'+str(module)+'"'));binary=tmp_path/name;build(host,binary)
 result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=30);assert result.returncode==1,(name,result.stdout,result.stderr);assert 'got=' in result.stderr

def test_undefined_behavior_sanitizer(tmp_path):
 binary=tmp_path/'ubsan';build(HOST,binary,['-fsanitize=undefined','-fno-sanitize-recover=undefined'],compiler=os.environ.get('SF64_UBSAN_CXX'))
 result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=30);assert result.returncode==0,result.stderr

def test_official_testfloat_divsqrt(tmp_path):
 generator=Path(os.environ.get('SF64_TESTFLOAT_GEN','/tmp/workshop-testfloat-reference-20261027/TestFloat-3e/build/Linux-x86_64-GCC/testfloat_gen'))
 if not generator.is_file():pytest.skip('official TestFloat generator unavailable; set SF64_TESTFLOAT_GEN')
 binary=tmp_path/'host';build(HOST,binary)
 for operation in ('f64_div','f64_sqrt'):
  vectors=tmp_path/(operation+'.txt')
  with vectors.open('w') as stream:subprocess.run([str(generator),'-level','1','-rnear_even','-tininessafter',operation],stdout=stream,check=True)
  result=subprocess.run([str(binary),operation,str(vectors)],capture_output=True,text=True,timeout=30);assert result.returncode==0,result.stderr
  assert int(result.stdout.split('vectors=')[1])>=600
