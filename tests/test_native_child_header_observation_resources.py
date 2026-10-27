"""Cold diagnostic-header logger resource contract. CPU-only; no Vulkan calls."""
from pathlib import Path
import hashlib
import re
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT/'tools/performance/qualify_children.cpp'
BASE_SHA = '4cd3b0fa217447ea287b4e821b8e10ee4c1d412f76cab41da9adaa76a9f866a5'
PREDICATE = '!w[7]&&w[3]==q.parent_count&&w[4]<=q.C&&w[5]<=q.S&&w[6]<=k.graph.job_capacity'
OLD = 'else{require('+PREDICATE+',"literal compact bounds/fault");'
NEW = 'else{const bool compact_valid='+PREDICATE+';if(!compact_valid)compact_failure_header(std::cerr,w,q.parent_count);require(compact_valid,"literal compact bounds/fault");'


def without_candidate_observation(source):
    """Inverse only the separately reviewed raw diagnostic additions."""
    if '// Cold raw observation of' not in source:
        return source
    start = source.index('// Cold raw observation of')
    end = source.index('std::string common(', start)
    helpers = source[start:end]
    assert hashlib.sha256(helpers.encode()).hexdigest() == 'b829e5331ff59a55933b8d2f6478d005619383d4971577d0317c8994386689d9'
    source = source[:start] + source[end:]
    start = source.index(' void candidate_snapshot(')
    end = source.index(' void snapshot(', start)
    method = source[start:end]
    assert hashlib.sha256(method.encode()).hexdigest() == '1f45eae6e2135c6200bcf4523ff29a714b160bcd9a7f5c3ed3f23182c0494be9'
    source = source[:start] + source[end:]
    start = source.index('if(!compact_valid){compact_failure_header')
    end = source.index('require(compact_valid,"literal compact bounds/fault")', start)
    call = source[start:end]
    assert hashlib.sha256(call.encode()).hexdigest() == '27ccddf0b4ac05a29d47a1832cab2e8d67c5985950a5f042ca0cbc4b72fd654a'
    source = source[:start] + 'if(!compact_valid)compact_failure_header(std::cerr,w,q.parent_count);' + source[end:]
    assert source.count(',".candidate-unusable.json"') == 1
    return source.replace(',".candidate-unusable.json"', '', 1)


def guard(source):
    source = without_candidate_observation(source)
    start = source.index('// Cold failure observation only:')
    end = source.index('std::string common(', start)
    helper = source[start:end]
    assert 'std::array<u32,32> snapshot{};' in helper
    assert 'std::memcpy(snapshot.data(),mapped,sizeof(snapshot));' in helper
    assert 'i<snapshot.size()' in helper
    assert not re.search(r'\bnew\b|malloc|vector|vk[A-Z]|Buffer|Context|\.resize\(|\.create\(', helper)
    assert helper.count('mapped') == 3  # comment, parameter, bounded memcpy
    assert source.count('compact_failure_header(') == 2
    assert NEW in source
    assert source.index('k.execute(t,q);') < source.index('if(!compact_valid)compact_failure_header')
    # Inverse the two authorized additions. Every existing success, ownership,
    # fence, transfer, parser, allocation and fail-closed path must be identical.
    original = (source[:start]+source[end:]).replace(NEW, OLD, 1)
    assert hashlib.sha256(original.encode()).hexdigest() == BASE_SHA


def test_exact_bounded_failure_only_delta():
    guard(CPP.read_text())


MUTANTS = [
    ('std::array<u32,32> snapshot{};', 'std::array<u32,33> snapshot{};'),
    ('mapped,sizeof(snapshot)', 'mapped,sizeof(snapshot)+4'),
    ('if(!compact_valid){compact_failure_header', 'if(true){compact_failure_header'),
    ('std::array<u32,32> snapshot{};', 'auto* heap=new u32[32];std::array<u32,32> snapshot{};'),
    ('std::memcpy(snapshot.data(),mapped,sizeof(snapshot));', 'vkCmdCopyBuffer(0,0,0,0,nullptr);std::memcpy(snapshot.data(),mapped,sizeof(snapshot));'),
    ('!w[7]&&w[3]==q.parent_count', '!w[7]&&w[3]<=q.parent_count'),
    ('require(compact_valid,"literal compact bounds/fault")', 'require(true,"literal compact bounds/fault")'),
    ('std::chrono::seconds(10)', 'std::chrono::seconds(1000)'),
    ('copy(roots,download.buffer,8*n)', 'copy(roots,download.buffer,download.size)'),
]


@pytest.mark.parametrize('old,new', MUTANTS)
def test_changed_resources_or_acceptance_rejected(old,new):
    source = CPP.read_text()
    assert old in source
    with pytest.raises((AssertionError, ValueError)):
        guard(source.replace(old,new,1))


@pytest.fixture(scope='module')
def helper_binary(tmp_path_factory):
    directory = tmp_path_factory.mktemp('child-header-resource')
    unit = directory/'observe.cpp'
    unit.write_text(r'''
#include <cstdlib>
#include <cstdio>
#include <new>
#include <streambuf>
#include <sys/mman.h>
static bool observing=false;static unsigned allocations=0;
void* operator new(size_t n){if(observing)++allocations;if(void*p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,size_t)noexcept{std::free(p);}void operator delete[](void*p,size_t)noexcept{std::free(p);}
#define main diagnostic_main
#define CHILD_QUALIFIER_CPU_ONLY
#include "SOURCE_INCLUDE_PATH"
#undef main
struct Fixed:std::streambuf{
 char bytes[1024]{};size_t used=0;uint32_t* input=nullptr;bool mutate=false;
 void changed(){if(mutate){for(unsigned j=0;j<32;++j)input[j]=0;mutate=false;}}
 std::streamsize xsputn(const char*s,std::streamsize n)override{changed();if(n<0||used+size_t(n)>sizeof(bytes))return 0;std::memcpy(bytes+used,s,size_t(n));used+=size_t(n);return n;}
 int_type overflow(int_type c)override{changed();if(traits_type::eq_int_type(c,traits_type::eof()))return traits_type::not_eof(c);if(used==sizeof(bytes))return traits_type::eof();bytes[used++]=traits_type::to_char_type(c);return c;}
};
int main(int argc,char**argv){
 if(argc!=2)return 2;auto page=size_t(sysconf(_SC_PAGESIZE));auto* memory=static_cast<unsigned char*>(mmap(nullptr,3*page,PROT_NONE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0));require(memory!=MAP_FAILED,"mapping");require(mprotect(memory+page,page,PROT_READ|PROT_WRITE)==0,"middlepage");
 bool end=argv[1][0]=='e';auto* words=reinterpret_cast<uint32_t*>(memory+page+(end?page-128:0));uint32_t expected[32];for(unsigned j=0;j<32;j++)words[j]=expected[j]=j%2?0xffffffffu:j*11;words[3]=expected[3]=1;words[7]=expected[7]=64;
 Fixed buffer;std::ostream out(&buffer);out<<0;buffer.used=0;buffer.input=words;buffer.mutate=true;
 observing=true;compact_failure_header(out,words,1);observing=false;
 require(out.good()&&allocations==0,"helper allocated/overflowed");require(words[7]==0,"stream did not mutate mapped input");
 std::fwrite(buffer.bytes,1,buffer.used,stdout);std::printf("operator_new_calls=%u\n",allocations);munmap(memory,3*page);
}
'''.replace('SOURCE_INCLUDE_PATH',str(CPP)))
    target = directory/'observe'
    command = [shutil.which('g++'),'-std=c++17','-O2',str(unit),'-o',str(target)]
    result = subprocess.run(command,capture_output=True,text=True,timeout=60)
    (directory/'compile.log').write_text(result.stdout+result.stderr)
    result.check_returncode()
    return target


@pytest.mark.parametrize('position',['start','end'])
def test_actual_helper_guardpages_snapshot_and_no_heap(helper_binary,position):
    # The output stream mutates all mapped words on its first write. Printed
    # originals therefore also prove snapshot completion precedes formatting.
    result = subprocess.run([str(helper_binary),position],capture_output=True,text=True,timeout=10,check=True)
    words=[0xffffffff if j%2 else j*11 for j in range(32)];words[3]=1;words[7]=64
    assert result.stdout == 'literal_compact_header32=['+','.join(map(str,words))+'] expected_parent_count=1\noperator_new_calls=0\n'
    symbols = subprocess.run(['nm','-u',str(helper_binary)],capture_output=True,text=True,check=True).stdout
    assert not re.search(r'vk[A-Z]|Context',symbols)
