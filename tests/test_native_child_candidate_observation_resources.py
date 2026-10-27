"""Bounded cold raw-candidate diagnostic; CPU helpers/command recorder only."""
from pathlib import Path
import hashlib
import importlib.util
import json
import re
import shutil
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
CPP=ROOT/'tools/performance/qualify_children.cpp'
HASH='e85eb99b89f9173a5ffd50f85638d8b5542dd482f90100d116012da9c368c3dd'


def header_contract():
    spec=importlib.util.spec_from_file_location('header_resource_contract',ROOT/'tests/test_native_child_header_observation_resources.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def method(source):
    a=source.index(' void candidate_snapshot(')
    return source[a:source.index(' void snapshot(',a)]


def guard(source):
    bounds=source[source.index('size_t candidate_snapshot_part_bytes'):source.index('void candidate_unusable_observation')]
    assert bounds.index('require(n&&n<=307200') < bounds.index('size_t part=8*n')
    assert 'part<=capacity&&part<=capacity-part' in bounds
    capture=source[source.index('void candidate_unusable_observation'):source.index('std::string common(')]
    assert 'q.width==7&&q.height==5&&q.n()==35' in capture
    assert 'require(2*part==560' in capture
    assert 'std::array<u32,36> compact{};std::array<u32,140> snapshot{};' in capture
    assert 'std::memcpy(compact.data(),header,sizeof(compact));std::memcpy(snapshot.data(),raw,sizeof(snapshot));' in capture
    assert capture.index('candidate_snapshot_part_bytes(q.n(),capacity)') < capture.index('std::memcpy') < capture.index('out<<')
    for key in ('raw_candidate_observation_only','UNUSABLE','normal_result_published','first_parent_status','persistent_roots','source_flags'):
        assert key in capture
    m=method(source)
    assert m.index('candidate_snapshot_part_bytes(n,download.size)') < m.index('work.size>=part&&persistent.size>=part') < m.index('begin();')
    assert m.count('copy(')==2 and m.count('run(fence)')==1
    assert 'copy(work.buffer,download.buffer,part);copy(persistent.buffer,download.buffer,part,0,part);' in m
    assert m.index('VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT') < m.index('copy(')
    assert m.index('VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_HOST_READ_BIT') > m.rindex('copy(')
    assert m.index('run(fence)') > m.index('VK_ACCESS_HOST_READ_BIT')
    assert not re.search(r'\.create\(|vk[A-Z]|\bnew\b|malloc|vkMap|Dispatch',m)
    gate='if(w[7]==ChildNonconvergence&&q.hash=="'+HASH+'")'
    assert gate in source
    call=source[source.index('if(!compact_valid){compact_failure_header'):source.index('require(compact_valid,"literal compact bounds/fault")')]
    assert call.index('compact_failure_header') < call.index(gate) < call.index('t.candidate_snapshot(k.buffers[3],k.buffers[4],q.n())') < call.index('candidate_unusable_observation(') < call.index('fresh_text(prefix+".candidate-unusable.json"')
    assert '".candidate-unusable.json"' in source[source.index('for(const std::string cycle:'):source.index('auto* state=new GPUState')]
    assert 'VK_BUFFER_USAGE_STORAGE_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_SRC_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT' in source
    # Strong delta proof: preserve the entire previous header-only qualifier,
    # including all typed/success/negative/fence/allocation paths.
    inverse=header_contract().without_candidate_observation(source)
    assert hashlib.sha256(inverse.encode()).hexdigest()=='731aa618570c4d44fa9d95db8f6911924d1688d26b5e042f6045f22716e86e6f'
    header_contract().guard(source)


def test_actual_raw_observer_contract():guard(CPP.read_text())


MUTANTS=[
 ('require(n&&n<=307200','require(n'),
 ('part<=capacity&&part<=capacity-part','part<=capacity'),
 ('work.size>=part&&persistent.size>=part','work.size>=part'),
 ('std::array<u32,140> snapshot{}','std::array<u32,141> snapshot{}'),
 ('std::array<u32,36> compact{}','std::array<u32,37> compact{}'),
 ('raw,sizeof(snapshot)','raw,sizeof(snapshot)+4'),
 ('copy(persistent.buffer,download.buffer,part,0,part)','copy(persistent.buffer,download.buffer,part,0,0)'),
 ('VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT','VK_ACCESS_HOST_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT'),
 ('run(fence);}\n void snapshot','run(fence);run(fence);}\n void snapshot'),
 ('w[7]==ChildNonconvergence&&q.hash==','w[7]!=0&&q.hash=='),
 ('q.hash=="'+HASH+'"','true'),
 ('"status\\\":\\\"UNUSABLE','"status\\\":\\\"valid'),
 ('copy(work.buffer,download.buffer,part);','download.create();copy(work.buffer,download.buffer,part);'),
]
@pytest.mark.parametrize('old,new',MUTANTS)
def test_resource_or_scope_mutants_rejected(old,new):
    source=CPP.read_text();assert old in source
    with pytest.raises((AssertionError,ValueError)):guard(source.replace(old,new,1))


PRELUDE=r'''
#include <cstdlib>
#include <cstdio>
#include <new>
#include <streambuf>
#include <sys/mman.h>
static bool observing=false;static size_t allocations=0,total_bytes=0,largest=0;
void* operator new(size_t n){if(observing){++allocations;total_bytes+=n;if(n>largest)largest=n;}if(void*p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,size_t)noexcept{std::free(p);}void operator delete[](void*p,size_t)noexcept{std::free(p);}
#define main diagnostic_main
#define CHILD_QUALIFIER_CPU_ONLY
#include "SOURCE_INCLUDE_PATH"
#undef main
'''


def compile_unit(directory,text,name):
    unit=directory/(name+'.cpp');target=directory/name
    unit.write_text(text.replace('SOURCE_INCLUDE_PATH',str(CPP)))
    source_sha=hashlib.sha256(CPP.read_bytes()).hexdigest()
    cmd=[shutil.which('g++'),'-std=c++17','-O2','-DCHILD_QUALIFIER_SOURCE_SHA256="'+source_sha+'"',str(unit),'-o',str(target)]
    result=subprocess.run(cmd,capture_output=True,text=True,timeout=60)
    (directory/(name+'.compile.log')).write_text(result.stdout+result.stderr)
    (directory/(name+'.command.json')).write_text(json.dumps(cmd,indent=2))
    result.check_returncode();return target


@pytest.fixture(scope='module')
def capture_binary(tmp_path_factory):
    d=tmp_path_factory.mktemp('candidate-capture-resource')
    code=PRELUDE+r'''
struct Fixed:std::streambuf{
 char data[4096]{};size_t used=0;u32* header=nullptr;u32* raw=nullptr;bool mutate=false;
 void changed(){if(mutate){for(unsigned j=0;j<36;j++)header[j]=0;for(unsigned j=0;j<140;j++)raw[j]=0;mutate=false;}}
 std::streamsize xsputn(const char*s,std::streamsize n)override{changed();if(n<0||used+size_t(n)>sizeof(data))return 0;std::memcpy(data+used,s,size_t(n));used+=size_t(n);return n;}
 int_type overflow(int_type c)override{changed();if(traits_type::eq_int_type(c,traits_type::eof()))return traits_type::not_eof(c);if(used==sizeof(data))return traits_type::eof();data[used++]=traits_type::to_char_type(c);return c;}
};
struct Region{unsigned char* base;u32* words;size_t page;Region(size_t bytes,bool end){page=size_t(sysconf(_SC_PAGESIZE));base=static_cast<unsigned char*>(mmap(nullptr,3*page,PROT_NONE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0));require(base!=MAP_FAILED,"mapping");require(mprotect(base+page,page,PROT_READ|PROT_WRITE)==0,"guard");words=reinterpret_cast<u32*>(base+page+(end?page-bytes:0));}~Region(){munmap(base,3*page);}};
int main(int argc,char**argv){
 require(argc==2,"position");bool end=argv[1][0]=='e';Region header(144,end),raw(560,end),inaccessible(0,false);
 for(unsigned j=0;j<36;j++)header.words[j]=j%2?0xffffffffu:j;
 for(unsigned j=0;j<140;j++)raw.words[j]=j%2?0xffffffffu:j;
 Fixture q;q.width=7;q.height=5;q.parent_count=1;q.hash=std::string(64,'e');
 // Capacity/count errors must reject BEFORE touching even inaccessible inputs.
 bool rejected=false;try{candidate_unusable_observation(std::cout,q,reinterpret_cast<u32*>(inaccessible.base),reinterpret_cast<u32*>(inaccessible.base),559);}catch(const std::exception&){rejected=true;}require(rejected,"capacity559read");
 for(size_t n:{size_t(0),size_t(307201),SIZE_MAX}){bool bad=false;try{candidate_snapshot_part_bytes(n,SIZE_MAX);}catch(const std::exception&){bad=true;}require(bad,"count before multiplication");}
 Fixed buffer;std::ostream out(&buffer);out<<0;buffer.used=0;buffer.header=header.words;buffer.raw=raw.words;buffer.mutate=true;
 observing=true;candidate_unusable_observation(out,q,header.words,raw.words,560);observing=false;
 require(out.good()&&buffer.used<4096,"boundedoutput");require(header.words[1]==0&&raw.words[1]==0,"mutated input");
 std::fwrite(buffer.data,1,buffer.used,stdout);std::printf("allocations=%zu total_bytes=%zu largest=%zu output_bytes=%zu\n",allocations,total_bytes,largest,buffer.used);
}
'''
    return compile_unit(d,code,'capture')


@pytest.mark.parametrize('position',['start','end'])
def test_actual_capture_bounds_snapshot_and_cold_host_allocations(capture_binary,position):
    result=subprocess.run([str(capture_binary),position],capture_output=True,text=True,check=True,timeout=10)
    lines=result.stdout.splitlines();payload=json.loads(lines[0]);metrics=dict((k,int(v)) for k,v in re.findall(r'(\w+)=(\d+)',lines[1]))
    assert payload['scope']=='raw_candidate_observation_only' and payload['status']=='UNUSABLE'
    assert payload['normal_result_published'] is False and payload['physical_camera'] is False and payload['objects'] is False and payload['world_qualified'] is False
    original=lambda j:0xffffffff if j%2 else j
    assert payload['compact_header32']==[original(j) for j in range(32)]
    assert payload['first_parent_status']==[original(j) for j in range(32,36)]
    for a,name in enumerate(('A','B','persistent_roots','source_flags')):assert payload[name]==[original(j) for j in range(35*a,35*(a+1))]
    assert metrics['total_bytes']<=16384 and metrics['largest']<=4096 and metrics['output_bytes']<4096
    (capture_binary.parent/(position+'.observation.json')).write_text(json.dumps(dict(metrics=metrics,payload=payload),indent=2))


def test_actual_candidate_transfer_method_command_recorder(tmp_path):
    # Compile the actual method text, not a duplicate transfer sequence. Adapter
    # operations only record calls: this proves CPU orchestration, not a driver.
    code=PRELUDE+r'''
constexpr u32 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT=1,VK_PIPELINE_STAGE_TRANSFER_BIT=2,VK_PIPELINE_STAGE_HOST_BIT=4;
constexpr u32 VK_ACCESS_SHADER_WRITE_BIT=8,VK_ACCESS_TRANSFER_READ_BIT=16,VK_ACCESS_TRANSFER_WRITE_BIT=32,VK_ACCESS_HOST_READ_BIT=64;
struct Buffer{size_t size;u32 buffer;};struct Event{u32 type,a,b,c,d;size_t bytes,source,destination;};
struct Recorder{
 Buffer download{131556,3};u32 fence=123;std::array<Event,6> events{};unsigned count=0;
 void begin(){events.at(count++)={1,0,0,0,0,0,0,0};}
 void barrier(u32 a,u32 b,u32 c,u32 d){events.at(count++)={2,a,b,c,d,0,0,0};}
 void copy(u32 a,u32 b,size_t bytes,size_t source=0,size_t dest=0){events.at(count++)={3,a,b,0,0,bytes,source,dest};}
 void run(u32 f){events.at(count++)={4,f,0,0,0,0,0,0};}
ACTUAL_METHOD
};
int main(){Recorder r;Buffer work{264192,1},persistent{280,2};observing=true;r.candidate_snapshot(work,persistent,35);observing=false;require(allocations==0&&r.count==6,"events/allocs");auto& e=r.events;require(e[0].type==1&&e[1].type==2&&e[1].a==1&&e[1].b==2&&e[1].c==8&&e[1].d==16,"prebarrier");require(e[2].type==3&&e[2].a==1&&e[2].b==3&&e[2].bytes==280&&e[2].source==0&&e[2].destination==0,"copyA/B");require(e[3].type==3&&e[3].a==2&&e[3].b==3&&e[3].bytes==280&&e[3].source==0&&e[3].destination==280,"copyroots/flags");require(e[4].type==2&&e[4].a==2&&e[4].b==4&&e[4].c==32&&e[4].d==64&&e[5].type==4&&e[5].a==123,"hostbarrier/fence");
 for(unsigned bad=0;bad<5;bad++){Recorder invalid;Buffer w=work,p=persistent;size_t n=35;if(bad==0)invalid.download.size=559;if(bad==1)w.size=279;if(bad==2)p.size=279;if(bad==3)n=0;if(bad==4)n=SIZE_MAX;bool rejected=false;try{invalid.candidate_snapshot(w,p,n);}catch(const std::exception&){rejected=true;}require(rejected&&invalid.count==0,"invalid before commandrecording");}
 std::puts("actual_method:2copies/560bytes/2barriers/1existingfence/0operatornew;5invalid-before-begin");}
'''
    code=code.replace('ACTUAL_METHOD',method(CPP.read_text()))
    binary=compile_unit(tmp_path,code,'transfer-recorder')
    result=subprocess.run([str(binary)],capture_output=True,text=True,check=True,timeout=10)
    assert '2copies/560bytes/2barriers/1existingfence/0operatornew' in result.stdout
    symbols=subprocess.run(['nm','-u',str(binary)],capture_output=True,text=True,check=True).stdout
    assert not re.search(r'vk[A-Z]|Context',symbols)
