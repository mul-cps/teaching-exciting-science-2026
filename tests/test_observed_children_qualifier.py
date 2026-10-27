"""Offline qualifier/parser and actual compiled SPIR-V seals; never executes Vulkan."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil
import struct
import subprocess
import numpy as np
import pytest
from test_observed_children import shader_witness, execute, cases
ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'ros2/src/so101_station_vulkan'
CACHE=Path('/tmp/observed-children-shaders-20261027')

@pytest.fixture(scope='module')
def seal(tmp_path_factory):
    d=tmp_path_factory.mktemp('observed-seal');src=d/'seal.cpp';binary=d/'seal'
    src.write_text('#define OBSERVED_QUALIFIER_ENTRY observed_main\n#include "'+str(P/'test/qualify_observed_children.cpp')+'"\nint main(int argc,char** argv){try{std::vector<std::string> names={argv[3]};std::cout<<precision_shader_identity(argv[1],names,argv[2],argv[3],"");return 0;}catch(const std::exception& e){std::cerr<<e.what();return 1;}}\n')
    r=subprocess.run(['c++','-std=c++17','-O1','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-I',str(P/'include'),str(src),'-ldl','-o',str(binary)],capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    undefined=subprocess.check_output(['nm','-u',str(binary)],text=True)
    assert 'vkCreate' not in undefined and 'Context' not in undefined
    return binary

@pytest.mark.parametrize('mode',['native','software'])
def test_actual_offline_spirv_precision_seal(seal,mode):
    if not CACHE.is_dir():pytest.skip('explicit offline qualification caches not staged')
    directory=CACHE/mode
    r=subprocess.run([str(seal),str(directory),mode,'observed_observe'],capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    assert json.loads(r.stdout)['precision']==mode
    wrong=subprocess.run([str(seal),str(directory),'software' if mode=='native' else 'native','observed_observe'],capture_output=True,text=True)
    assert wrong.returncode!=0 and 'capability' in wrong.stderr


def test_actual_software_geometry_precision_seal(seal):
    if not CACHE.is_dir():pytest.skip('explicit offline qualification caches not staged')
    r=subprocess.run([str(seal),str(CACHE/'software-geometry'),'software','geometry_coordinates'],capture_output=True,text=True)
    assert r.returncode==0,r.stderr

@pytest.mark.parametrize('mutation',['extra-capability','missing-scalar','bad-instruction','wrong-digest'])
def test_spirv_seal_rejects_resealed_corruptions(seal,tmp_path,mutation):
    if not CACHE.is_dir():pytest.skip('explicit offline qualification caches not staged')
    directory=CACHE/'native';name='observed_observe.spv'
    for file in ('source.sha256','provenance.txt',name,name+'.sha256'):shutil.copyfile(directory/file,tmp_path/file)
    data=(tmp_path/name).read_bytes();words=list(struct.unpack('<'+str(len(data)//4)+'I',data))
    if mutation=='extra-capability':words[5:5]=[(2<<16)|17,22]
    elif mutation=='missing-scalar':
        at=5
        while at<len(words):
            count=words[at]>>16
            if words[at]&65535==17 and words[at+1]==10:del words[at:at+count];break
            at+=count
    elif mutation=='bad-instruction':words[5]=17
    else:words[-1]^=1
    data=struct.pack('<'+str(len(words))+'I',*words);(tmp_path/name).write_bytes(data)
    if mutation!='wrong-digest':(tmp_path/(name+'.sha256')).write_text(hashlib.sha256(data).hexdigest()+'  '+name+'\n')
    r=subprocess.run([str(seal),str(tmp_path),'native','observed_observe'],capture_output=True,text=True)
    assert r.returncode!=0


def test_actual_shader_membership_capacity_fault(shader_witness):
    assert execute(shader_witness,cases()[0],capacity=1)=={'usable':False}


def test_parser_offline_640_and_no_gpu_path(tmp_path):
    spec=importlib.util.spec_from_file_location('childq',ROOT/'tools/qualify_native_children.py');q=importlib.util.module_from_spec(spec);spec.loader.exec_module(q)
    mask=np.zeros((480,640),dtype='<u4');mask[100:130,120:160]=1;depth=np.full(mask.shape,np.float32(.5).view(np.uint32),dtype='<u4');labels,parents=q.parent_oracle(mask)
    cfg=q.configuration(640,480,'typed_parent',minimum=24);fixture=tmp_path/'640.childq';fixture.write_bytes(q.encode(cfg,mask,depth,labels,parents))
    binary=tmp_path/'parser';r=subprocess.run(['c++','-std=c++17','-O2','-DOBSERVED_QUALIFIER_CPU_ONLY',str(P/'test/qualify_observed_children.cpp'),'-o',str(binary)],capture_output=True,text=True);assert r.returncode==0,r.stderr
    r=subprocess.run([str(binary),'--offline',str(fixture)],capture_output=True,text=True);assert r.returncode==0,r.stderr
    report=json.loads(r.stdout);assert report['width']==640 and report['height']==480 and report['min_child_pixels']==24 and report['gpu_executed'] is False
    r=subprocess.run([str(binary),'--vulkan',str(fixture)],capture_output=True,text=True);assert r.returncode!=0 and 'cannot execute Vulkan' in r.stderr
    data=bytearray(fixture.read_bytes());data[0]=0;fixture.write_bytes(data)
    assert subprocess.run([str(binary),'--offline',str(fixture)],capture_output=True).returncode!=0


def test_retained_acceptance_oracle_640_roundtrip_and_exact_mutants(tmp_path):
    spec=importlib.util.spec_from_file_location('observed_acceptance',ROOT/'tools/qualify_native_observed_children.py');tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    directory=tmp_path/'fixtures';manifest=tool.prepare(directory);name='scoped-640x480';c=manifest['cases'][name];prefix=tmp_path/'synthetic-test-output'
    # Synthetic comparator fixture ONLY: no claim that these bytes came from GPU.
    with np.load(directory/c['expected'],allow_pickle=False) as expected:
        for key in ('observed','records','counts','child_roots','world'):Path(str(prefix)+'.'+key+'.bin').write_bytes(expected[key].tobytes())
        assert expected['counts'].tolist()==[2,2,2350,2350]
    report=dict(scope=manifest['scope'],gpu_executed=True,production_views_used=True,physical_camera=False,objects=False,world_qualified=False,fixture_sha256=c['fixture_sha256'],width=640,height=480,source_frame=91,depth_source_frame=90,publication_session_token=37,unique_inputs=1,completed_outputs=1,malformation=0,precision='software',qualifier_source_sha256=manifest['sources']['ros2/src/so101_station_vulkan/test/qualify_observed_children.cpp'],qualifier_helper_source_sha256=manifest['sources']['tools/performance/qualify_children.cpp'],external_watchdog_acknowledged=True,status='valid',loaded_libraries=[dict(sha256='a'*64) for _ in range(5)])
    for key in ('compact_submit_finish_wall_ms','compact_submit_finish_process_cpu_ms','dense_diagnostic_wall_ms','dense_diagnostic_process_cpu_ms'):report[key]=1.0
    path=Path(str(prefix)+'.json');path.write_text(json.dumps(report));assert tool.compare(directory,name,prefix,'software')['matched']
    report['publication_session_token']=38;path.write_text(json.dumps(report))
    with pytest.raises(ValueError,match='metadata'):tool.compare(directory,name,prefix,'software')
    report['publication_session_token']=37;path.write_text(json.dumps(report));output=Path(str(prefix)+'.observed.bin');data=bytearray(output.read_bytes());data[-1]^=1;output.write_bytes(data)
    with pytest.raises(ValueError,match='exact observed'):tool.compare(directory,name,prefix,'software')
