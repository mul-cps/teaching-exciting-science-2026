"""CPU-only factory/parser/exact diagnostic comparison. No Vulkan execution."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import shutil
import struct
import subprocess
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
CPP=ROOT/'tools/performance/qualify_children.cpp'
SPEC=importlib.util.spec_from_file_location('child_qualifier',ROOT/'tools/qualify_native_children.py')
q=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(q)
@pytest.fixture(scope='session')
def binary(tmp_path_factory):
    target=tmp_path_factory.mktemp('child-parser')/'qualify'
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-Wall','-Wextra','-Werror','-DCHILD_QUALIFIER_CPU_ONLY',str(CPP),'-o',str(target)],check=True)
    return target
@pytest.fixture(scope='session')
def prepared(tmp_path_factory):
    directory=tmp_path_factory.mktemp('child-factory')/'exclusive'
    m=q.prepare(directory);return directory,m

def run(binary,fixture,*args):return subprocess.run([str(binary),'--offline',str(fixture),*map(str,args)],capture_output=True,text=True)
def test_committed_factory_coverage_and_exact_offline_parse(binary,prepared):
    directory,m=prepared
    assert len(m['cases'])==58
    assert sum(c['profile']=='literal_kernel' for c in m['cases'].values())==29
    assert sum(c['profile']=='typed_parent' for c in m['cases'].values())==29
    assert sum(c['expected_unusable'] for c in m['cases'].values())==3
    assert q.verify(directory)==m
    for c in m['cases'].values():
        result=run(binary,directory/c['fixture']);assert result.returncode==0,result.stderr
        parsed=json.loads(result.stdout)
        assert parsed['fixture_sha256']==c['fixture_sha256']
        assert parsed['profile']==c['profile'] and parsed['expected_unusable']==c['expected_unusable']
        assert parsed['gpu_executed'] is False and parsed['executable_sha256']==q.digest(binary)
    assert m['cases']['literal-tall-1025x7']['height']>480
    assert 'typed-tall-1025x7' not in m['cases']
    assert m['cases']['two-parents-P2C1-empty']['parent_count']==2
    assert m['cases']['one-parent-P1C2']['parent_count']==1
    text=(ROOT/'tools/qualify_native_children.py').read_text()
    assert 'child_graph_model' not in text and 'ChildGraphLayout' not in text
    assert 'trace_case(case)' in text and 'CCL_GRANA' in text

def fixture_bytes(prepared):
    directory,m=prepared;c=m['cases']['one-parent-P1C2'];return (directory/c['fixture']).read_bytes()
@pytest.mark.parametrize('field,value',[(0,0),(0,2049),(1,2049),(2,3),(3,2),(4,0),(4,4097),(5,0),(5,4097),(6,0),(6,0xffffffff),(7,7),(8,0),(9,0),(11,0),(13,0),(16,0x7ff00000),(18,0x7ff00000),(19,4097),(20,0),(21,1),(28,1)])
def test_header_bounds_before_allocation(binary,prepared,tmp_path,field,value):
    payload=bytearray(fixture_bytes(prepared));struct.pack_into('<I',payload,12+4*field,value);f=tmp_path/'bad.childq';f.write_bytes(payload)
    assert run(binary,f).returncode!=0
@pytest.mark.parametrize('mutation',['magic','truncated','extra','oversized','mask','label','bbox','endian','typed-tall'])
def test_file_payload_and_endian_rejection(binary,prepared,tmp_path,mutation):
    payload=bytearray(fixture_bytes(prepared));w,h=struct.unpack_from('<II',payload,12);n=w*h
    if mutation=='magic':payload[0]=0
    elif mutation=='truncated':payload=payload[:-1]
    elif mutation=='extra':payload+=b'x'
    elif mutation=='oversized':payload=payload+b'0'*4000000
    elif mutation=='mask':struct.pack_into('<I',payload,128,3)
    elif mutation=='label':struct.pack_into('<I',payload,128+8*n,0xffffffff)
    elif mutation=='bbox':struct.pack_into('<I',payload,128+12*n+4*6,0xffffffff)
    elif mutation=='endian':payload[12:16]=payload[12:16][::-1]
    else:
        directory,m=prepared;payload=bytearray((directory/m['cases']['literal-tall-1025x7']['fixture']).read_bytes());struct.pack_into('<I',payload,20,1)
    f=tmp_path/'bad.childq';f.write_bytes(payload);assert run(binary,f).returncode!=0

def test_exclusive_factory_and_offline_outputs(binary,prepared,tmp_path):
    directory,m=prepared
    with pytest.raises(FileExistsError):q.prepare(directory)
    case=directory/next(iter(m['cases'].values()))['fixture'];prefix=tmp_path/'result'
    assert run(binary,case,prefix).returncode==0
    before=(tmp_path/'result.offline.json').read_bytes();assert run(binary,case,prefix).returncode!=0
    assert (tmp_path/'result.offline.json').read_bytes()==before

def synthetic(prepared,tmp_path,name):
    directory,m=prepared;c=m['cases'][name];prefix=tmp_path/name
    r=dict(scope='childprep_only',physical_camera=False,objects=False,world_qualified=False,profile=c['profile'],fixture_sha256=c['fixture_sha256'],width=c['width'],height=c['height'],calibration_generation=17,parent_generation=23,configuration_generation=29,external_watchdog_acknowledged=True,qualifier_source_sha256=m['sources']['tools/performance/qualify_children.cpp'],child_module_source_sha256=m['sources']['ros2/src/so101_station_vulkan/src/children.cpp'],shader_source_sha256=m['sources']['ros2/src/so101_station_vulkan/shaders/children.slang'],status='unusable' if c['expected_unusable'] else 'valid',production_views_used=c['profile']=='typed_parent',parent_shader_identity=None)
    if c['expected_unusable']:
        r.update(fault_views_rejected={k:True for k in q.VIEWS},fault=1,production_views_used=c['profile']=='typed_parent')
    else:
        r.update({k:True for k in ('metadata_preserved','source_retained','source_pool_collision_rejected','generation_rejected','source_fault_rejected','pending_source_rejected','copied_lease','three_slot_retention','dropped_source_reaped','reuse_empty_restore')});r['frame_numbers']=[91,92,93]
        with np.load(directory/c['expected'],allow_pickle=False) as e:
            for suffix,ek in ([('','')] if c['profile']=='literal_kernel' else [('',''),('.empty','empty_'),('.restore','')]):
                for k in q.ARRAYS:Path(str(prefix)+suffix+'.'+k+'.bin').write_bytes(e[ek+k].tobytes())
    Path(str(prefix)+'.json').write_text(json.dumps(r));return prefix,r
@pytest.mark.parametrize('name',['literal-u-membership','one-parent-P1C2','two-colors-boundary','record-overflow','packed-overflow','literal-invalid-parent-label'])
def test_exact_compare_and_profile_fault_boundary(prepared,tmp_path,name):
    prefix,r=synthetic(prepared,tmp_path,name);assert q.compare(prepared[0],name,prefix)['matched']
@pytest.mark.parametrize('key',q.ARRAYS)
def test_full_arrays_not_just_counts_or_hashes(prepared,tmp_path,key):
    name='one-parent-P1C2';prefix,_=synthetic(prepared,tmp_path,name);f=Path(str(prefix)+'.'+key+'.bin');payload=bytearray(f.read_bytes());assert payload;payload[0]^=1;f.write_bytes(payload)
    with pytest.raises(ValueError,match='exact'):q.compare(prepared[0],name,prefix)
@pytest.mark.parametrize('suffix',['.empty','.restore'])
def test_reuse_empty_restore_entire_arrays(prepared,tmp_path,suffix):
    name='one-parent-P1C2';prefix,_=synthetic(prepared,tmp_path,name);f=Path(str(prefix)+suffix+'.flags.bin');a=bytearray(f.read_bytes());a[0]^=1;f.write_bytes(a)
    with pytest.raises(ValueError,match='exact'):q.compare(prepared[0],name,prefix)
@pytest.mark.parametrize('view',q.VIEWS)
def test_every_negative_typed_view_must_reject(prepared,tmp_path,view):
    name='record-overflow';prefix,r=synthetic(prepared,tmp_path,name);r['fault_views_rejected'][view]=False;Path(str(prefix)+'.json').write_text(json.dumps(r))
    with pytest.raises(ValueError,match='fault views'):q.compare(prepared[0],name,prefix)
def test_negative_never_publishes_partial_arrays(prepared,tmp_path):
    name='packed-overflow';prefix,_=synthetic(prepared,tmp_path,name);Path(str(prefix)+'.records.bin').write_bytes(b'')
    with pytest.raises(ValueError,match='published arrays'):q.compare(prepared[0],name,prefix)
def test_expected_corruption_and_metadata_mutation(prepared,tmp_path):
    directory,m=prepared;clone=tmp_path/'clone';shutil.copytree(directory,clone);name='one-parent-P1C2';f=clone/m['cases'][name]['expected'];f.write_bytes(f.read_bytes()+b'bad')
    with pytest.raises(ValueError,match='artifact hash'):q.verify(clone)
    prefix,r=synthetic(prepared,tmp_path,name);r['profile']='literal_kernel';Path(str(prefix)+'.json').write_text(json.dumps(r))
    with pytest.raises(ValueError,match='profile'):q.compare(directory,name,prefix)
def test_global_valid_and_same_parent_cut_from_independent_cpu(prepared):
    directory,m=prepared
    with np.load(directory/m['cases']['two-colors-boundary']['expected']) as e:
        assert np.all(e['flags']==1) and np.all(e['statuses'][:,0]==80) and e['records'].shape==(2,16)
    with np.load(directory/m['cases']['one-parent-P1C2']['expected']) as e:
        assert np.any(e['empty_flags']) and not np.any(e['empty_roots']) and e['empty_statuses'].shape==(0,4)
def test_other_parent_cut_does_not_contaminate_packed_flags(prepared):
    directory,m=prepared
    with np.load(directory/m['cases']['overlapping-parent-cut-halos']['expected']) as e:
        assert np.any(e['flags']&2)
        red=e['records'][0];assert red[1]==0 and red[13]==0
        payload=e['packed'][red[12]:red[12]+red[10]*red[11]]
        assert not np.any(payload&8)
        assert e['records'][1:,13].sum()>0

def test_no_device_mode_and_ack_required_before_context(binary,prepared):
    directory,m=prepared;case=directory/next(iter(m['cases'].values()))['fixture']
    a=subprocess.run([str(binary),'--vulkan',str(case),'parents','children','prefix'],capture_output=True,text=True)
    assert a.returncode!=0 and 'BEFORE Context' in a.stderr
    a=subprocess.run([str(binary),'--vulkan',str(case),'parents','children','prefix','--external-watchdog-ack'],capture_output=True,text=True)
    assert a.returncode!=0 and 'CPU-only' in a.stderr
    text=CPP.read_text();assert 'child_detail::record_graph(t.command' in text
    assert 'vkDeviceWaitIdle' not in text and 'vkQueueWaitIdle' not in text
    assert 'std::_Exit(code)' in text and '10000000000ULL' in text

def test_sparse_oversized_output_rejected_before_read(prepared,tmp_path,monkeypatch):
    name='one-parent-P1C2';prefix,_=synthetic(prepared,tmp_path,name);output=Path(str(prefix)+'.roots.bin')
    with output.open('r+b') as stream:stream.truncate(1<<34)
    original=Path.read_bytes
    def guarded(path):
        if path==output:raise AssertionError('oversized output was read')
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',guarded)
    with pytest.raises(ValueError,match='size differs'):q.compare(prepared[0],name,prefix)

@pytest.mark.parametrize('profile,name',[('literal_kernel','literal-u-membership'),('typed_parent','one-parent-P1C2')])
def test_shader_pinned_manifest_respects_actual_profile(prepared,tmp_path,profile,name):
    directory,m=prepared;clone=tmp_path/'pinned';shutil.copytree(directory,clone);pinned=json.loads((clone/'manifest.json').read_text());child={'source_sha256':'child-synthetic-test-identity'};parent={'source_sha256':'parent-synthetic-test-identity'};pinned['shader_artifacts']={'child_shader_identity':child,'parent_shader_identity':parent};(clone/'manifest.json').write_text(json.dumps(pinned))
    prefix,r=synthetic(prepared,tmp_path,name);r.update(child_shader_identity=child,parent_shader_identity=parent if profile=='typed_parent' else None);Path(str(prefix)+'.json').write_text(json.dumps(r))
    assert q.compare(clone,name,prefix)['matched']
    for wrong in ([None,{'wrong':'identity'}] if profile=='typed_parent' else [parent]):
        r['parent_shader_identity']=wrong;Path(str(prefix)+'.json').write_text(json.dumps(r))
        with pytest.raises(ValueError,match='parent_shader_identity'):q.compare(clone,name,prefix)
    if profile=='literal_kernel':
        del r['parent_shader_identity'];Path(str(prefix)+'.json').write_text(json.dumps(r))
        with pytest.raises(ValueError,match='parent_shader_identity'):q.compare(clone,name,prefix)


def test_failure_header_observation_reads_exactly32_completed_words(tmp_path):
    # Put the already-mapped128-byte header immediately before an unreadable
    # page. A wider snapshot faults; exact values/length are also asserted.
    source=tmp_path/'header.cpp';binary=tmp_path/'header'
    harness=r'''#include <sys/mman.h>
#define main qualifier_program_main
#define CHILD_QUALIFIER_CPU_ONLY
#include "QUALIFIER_CPP"
#undef main
int main(){
 auto page=size_t(sysconf(_SC_PAGESIZE));auto* memory=static_cast<unsigned char*>(mmap(nullptr,2*page,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0));require(memory!=MAP_FAILED,"mmap");require(mprotect(memory+page,page,PROT_NONE)==0,"guardpage");auto* words=reinterpret_cast<uint32_t*>(memory+page-128);for(unsigned j=0;j<32;j++)words[j]=j*7;words[3]=1;words[7]=64;compact_failure_header(std::cout,words,1);munmap(memory,2*page);
}
'''
    source.write_text(harness.replace('QUALIFIER_CPP',str(CPP)))
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    result=subprocess.run([str(binary)],capture_output=True,text=True,check=True)
    words=[j*7 for j in range(32)];words[3]=1;words[7]=64
    assert result.stdout=='literal_compact_header32=['+','.join(map(str,words))+'] expected_parent_count=1\n'
    text=CPP.read_text()
    assert 'if(!compact_valid){compact_failure_header(std::cerr,w,q.parent_count);' in text
    assert 'require(compact_valid,"literal compact bounds/fault")' in text


def test_raw_candidate_snapshot_is_bounded_and_explicitly_unusable(tmp_path):
    source=tmp_path/'candidate.cpp';binary=tmp_path/'candidate'
    harness=r'''#include <sys/mman.h>
#define main qualifier_program_main
#define CHILD_QUALIFIER_CPU_ONLY
#include "QUALIFIER_CPP"
#undef main
int main(){
 require(candidate_snapshot_part_bytes(35,560)==280,"twoN U32 part");bool rejected=false;try{candidate_snapshot_part_bytes(35,559);}catch(const std::exception&){rejected=true;}require(rejected,"reject capacity559");for(size_t n:{size_t(0),size_t(307201),SIZE_MAX}){bool bad=false;try{candidate_snapshot_part_bytes(n,SIZE_MAX);}catch(const std::exception&){bad=true;}require(bad,"reject count before multiplication");}
 auto page=size_t(sysconf(_SC_PAGESIZE));auto* memory=static_cast<unsigned char*>(mmap(nullptr,2*page,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0));require(memory!=MAP_FAILED,"mmap");require(mprotect(memory+page,page,PROT_NONE)==0,"guardpage");auto* words=reinterpret_cast<u32*>(memory+page-560);for(unsigned j=0;j<140;j++)words[j]=j+1;std::array<u32,36> compact{};compact[3]=1;compact[7]=4;compact[32]=compact[33]=24;Fixture q;q.width=7;q.height=5;q.parent_count=1;q.hash="unit-test-only";candidate_unusable_observation(std::cout,q,compact.data(),words,560);munmap(memory,2*page);
}
'''
    source.write_text(harness.replace('QUALIFIER_CPP',str(CPP)))
    subprocess.run([shutil.which('g++'),'-std=c++17','-O2','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    result=json.loads(subprocess.run([str(binary)],capture_output=True,text=True,check=True).stdout)
    assert result['status']=='UNUSABLE' and result['normal_result_published'] is False
    assert result['scope']=='raw_candidate_observation_only' and result['N']==35
    assert result['first_parent_status']==[24,24,0,0]
    for a,name in enumerate(('A','B','persistent_roots','source_flags')):assert result[name]==list(range(35*a+1,35*(a+1)+1))

def test_raw_candidate_copies_are_failure_only_and_reuse_existing_fence():
    text=CPP.read_text();method=text.split(' void candidate_snapshot(',1)[1].split(' void snapshot(',1)[0]
    assert method.count('copy(')==2 and method.count('run(fence)')==1
    assert 'copy(work.buffer,download.buffer,part);copy(persistent.buffer,download.buffer,part,0,part);' in method
    assert 'candidate_snapshot_part_bytes(n,download.size)' in method
    assert 'work.size>=part&&persistent.size>=part' in method
    assert '.create(' not in method and 'vkAllocate' not in method
    assert 'if(w[7]==ChildNonconvergence&&q.hash=="e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd")' in text
    assert 'require(compact_valid,"literal compact bounds/fault")' in text
    assert '.candidate-unusable.json' in text
