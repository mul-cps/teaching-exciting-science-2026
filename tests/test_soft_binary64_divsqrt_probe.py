"""Distinct SF64D001 offline diagnostic, no GPU execution."""
from pathlib import Path
import hashlib,json,struct,subprocess,shutil
import pytest
from test_soft_binary64_probe import executable,invoke
ROOT=Path(__file__).resolve().parents[1]
SHADER=ROOT/'tools/performance/soft_binary64_divsqrt_probe.slang'

def test_divsqrt_generation_validation_and_kind(executable,tmp_path):
 a=tmp_path/'a.bin';b=tmp_path/'b.bin';report=invoke(executable,'--generate-divsqrt',a,'--count',16384)
 invoke(executable,'--generate-divsqrt',b,'--count',16384)
 assert a.read_bytes()==b.read_bytes();magic,count,seed=struct.unpack_from('<8sQQ',a.read_bytes());assert magic==b'SF64D001' and count==16384 and seed==17
 assert report['corpus_kind']=='SF64D001' and report['generator']=='native-volatile-ieee754-rne-divsqrt-v1'
 assert report['corpus_sha256']==hashlib.sha256(a.read_bytes()).hexdigest();assert invoke(executable,'--validate',a)['corpus_kind']=='SF64D001'
 rows=list(struct.iter_unpack('<7Q',a.read_bytes()[24:]));assert rows[0][3:5]==(0xfff8000000000000,0)
 invoke(executable,'--generate-divsqrt',a,success=False)
 data=a.read_bytes()
 for i,mutant in enumerate((b'',data[:-1],data+b'x',b'SF64P001'+data[8:],data[:8]+struct.pack('<Q',4000001)+data[16:])):
  path=tmp_path/f'mutant{i}';path.write_bytes(mutant);invoke(executable,'--validate',path,success=False)
 changed=bytearray(data);changed[24+3*8]^=1;path=tmp_path/'expectation';path.write_bytes(changed);invoke(executable,'--validate',path,success=False)

def test_distinct_shader_compiles_and_kind_marker(executable,tmp_path):
 compiler=Path('/home/bjoern/.local/opt/slang-2026.19/bin/slangc');validator=Path('/home/bjoern/miniforge3/envs/so101-jazzy/bin/spirv-val');disassembler=validator.with_name('spirv-dis')
 if not compiler.is_file():pytest.skip('qualified offline Slang unavailable')
 assert SHADER.is_file();source=SHADER.read_text();assert 'sf64_div_bits(' in source and 'sf64_sqrt_bits(' in source
 spv=tmp_path/'probe.spv';r=subprocess.run([str(compiler),str(SHADER),'-I',str(ROOT/'ros2/src/so101_station_vulkan/shaders'),'-entry','main','-stage','compute','-target','spirv','-profile','sm_6_0','-fp-mode','precise','-O2','-o',str(spv)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 subprocess.run([str(validator),'--target-env','vulkan1.1',str(spv)],check=True,capture_output=True,text=True)
 text=subprocess.check_output([str(disassembler),str(spv)],text=True);assert 'OpCapability Int64' in text and 'OpCapability Float64' not in text and 'OpTypeFloat' not in text and 'OpFDiv' not in text and 'Sqrt ' not in text
 assert 'probe_input_SF64D001' in text
 assert invoke(executable,'--validate-spirv',spv,'--kind','SF64D001')['corpus_kind']=='SF64D001'
 invoke(executable,'--validate-spirv',spv,'--kind','SF64P001',success=False)
 old=tmp_path/'old.spv';subprocess.run([str(compiler),str(ROOT/'tools/performance/soft_binary64_probe.slang'),'-I',str(ROOT/'ros2/src/so101_station_vulkan/shaders'),'-entry','main','-stage','compute','-target','spirv','-profile','sm_6_0','-fp-mode','precise','-O2','-o',str(old)],check=True,capture_output=True,text=True)
 invoke(executable,'--validate-spirv',old,'--kind','SF64P001')
 invoke(executable,'--validate-spirv',old,'--kind','SF64D001',success=False)


def test_official_append_and_wrong_expectations(executable,tmp_path):
 directory=tmp_path/'testfloat';directory.mkdir()
 (directory/'f64_div.txt').write_text('3FF0000000000000 4000000000000000 3FE0000000000000 00\n')
 (directory/'f64_sqrt.txt').write_text('4010000000000000 4000000000000000 00\n')
 (directory/'f32_to_f64.txt').write_text('00000001 36A0000000000000 00\n')
 output=tmp_path/'official';report=invoke(executable,'--generate-divsqrt',output,'--count',16384,'--testfloat-dir',directory)
 assert report['testfloat_rows']==3 and report['case_count']==16387
 invoke(executable,'--validate',output)
 (directory/'f64_sqrt.txt').write_text('4010000000000000 3FF0000000000000 00\n')
 bad=tmp_path/'bad';result=invoke(executable,'--generate-divsqrt',bad,'--testfloat-dir',directory,success=False)
 assert 'disagrees with TestFloat' in result['error'] and not bad.exists()
