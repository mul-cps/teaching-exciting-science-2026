"""Resource-flow guards for the one-shot child diagnostic; never runs Vulkan."""
from pathlib import Path
import importlib.util
import json
import inspect
import re
import shutil
import struct
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / 'tools/performance/qualify_children.cpp'


def body(source, marker):
    start = source.index('{', source.index(marker))
    level = 1
    for end in range(start + 1, len(source)):
        level += (source[end] == '{') - (source[end] == '}')
        if not level:
            return source[start + 1:end]
    raise AssertionError('unterminated function')


def guard(source):
    load = body(source, 'Fixture load(')
    before = load[:load.index('q.mask.resize')]
    assert not re.search(r'\.resize\(|\bnew\b|\bmalloc\(', before)
    for check in ('length<=128+12*307200+32*4096', 'q.n()<=307200',
                  'q.P<=4096', 'q.C<=4096', 'q.S<=4*q.n()',
                  'q.parent_count<=q.P', 'exact file length'):
        assert check in before
    assert 'q.parents.resize(size_t(q.parent_count)*8)' in load
    assert 'std::array<unsigned char,128> header' in before
    assert 'std::array<char,16384> chunk' in source
    assert 'vkDeviceWaitIdle' not in source and 'vkQueueWaitIdle' not in source
    assert source.count('vkWaitForFences(') == 1
    assert 'vkWaitForFences(context->device,1,&target,VK_TRUE,10000000000ULL)' in source
    assert 'std::_Exit(code)' in source
    assert '~Transfer(){if(pending)fatal(' in source
    run = body(source, 'void run(VkFence target)')
    assert run.index('pending=target') < run.index('vkWaitForFences') < run.index('pending=VK_NULL_HANDLE')
    assert 'if(vkQueueSubmit' in run and 'fatal("submit failed' in run
    gpu = body(source, 'int gpu(')
    assert gpu.index('shader_identity(') < gpu.index('auto* state=new GPUState') < gpu.index('Context::shared()')
    assert 'auto* state=new GPUState;\n try{' in gpu
    assert 'catch(const std::exception& e)' in gpu and 'fatal("child diagnostic exception' in gpu
    assert 'catch(...){fatal(' in gpu
    assert 'delete state;' in gpu and gpu.index('delete state;') < gpu.index('catch(const std::exception& e)')
    main = body(source, 'int main(')
    assert 'require(argc==7&&std::string(argv[6])=="--external-watchdog-ack"' in main
    assert main.index('--external-watchdog-ack') < main.index('auto q=load(argv[2]);return gpu(')
    offline = main[main.index('require(mode=="--offline"'):]
    assert not re.search(r'Context|gpu\(|vk[A-Z]', offline)
    assert 'O_WRONLY|O_CREAT|O_EXCL' in source
    snap = body(source, 'void snapshot(')
    assert '8*n+4*pixels<=download.size' in snap
    assert 'copy(roots,download.buffer,8*n)' in snap
    assert 'copy(packed,download.buffer,4*pixels,0,8*n)' in snap
    assert snap.index('run(fence)') > snap.index('copy(packed')
    typed = gpu[gpu.index('auto lease=child.submit(par)'):]
    start = typed.index('t.snapshot(retained.source_root_buffer()')
    end = typed.index('retained={};child.collect();parent.collect()', start)
    assert 'publish(prefix+suffix' in typed[start:end]
    assert 'retained={}' not in typed[:end]
    assert 'auto retained=lease;lease={};complete(child,retained)' in typed
    assert 'three slot retention lost' in typed and 'dropped child10s deadline' in typed
    assert 'std::chrono::seconds(10)' in source and 'std::this_thread::sleep_for' in source
    for view in ('finish(retained)', 'retained.parent()', 'retained.source_root_buffer()',
                 'retained.source_flags_offset_bytes()', 'retained.packed_flags_buffer()',
                 'retained.resident_records_buffer()', 'retained.records()', 'retained.parent_status()'):
        assert 'rejects([&]{' + ('child.' if view == 'finish(retained)' else '') + view + ';})' in typed
    assert 'child_detail::record_graph(t.command' in source
    assert 'physical_camera\\\":false' in source and 'world_qualified\\\":false' in source


def test_actual_resource_flow():
    guard(CPP.read_text())


MUTANTS = [
    ('std::array<unsigned char,128> header{};', 'std::vector<unsigned char> header(4000000);q.mask.resize(307200);'),
    ('length<=128+12*307200+32*4096', 'length<=999999999'),
    ('q.parent_count<=q.P', 'q.parent_count<=4096'),
    ('10000000000ULL', 'UINT64_MAX'),
    ('pending=target;', 'pending=VK_NULL_HANDLE;'),
    ('~Transfer(){if(pending)fatal(', '~Transfer(){if(false)fatal('),
    ('std::_Exit(code)', 'std::exit(code)'),
    ('auto* state=new GPUState;\n try{', 'GPUState local;auto* state=&local;\n try{'),
    ('std::string(argv[6])=="--external-watchdog-ack"', 'std::string(argv[6])=="--ack-removed"'),
    ('O_WRONLY|O_CREAT|O_EXCL', 'O_WRONLY|O_CREAT|O_TRUNC'),
    ('copy(roots,download.buffer,8*n)', 'copy(roots,download.buffer,download.size)'),
    ('t.snapshot(retained.source_root_buffer()', 'retained={};t.snapshot(retained.source_root_buffer()'),
    ('auto retained=lease;lease={};complete(child,retained)', 'auto retained=lease;retained={};complete(child,retained)'),
    ('rejects([&]{retained.parent_status();})', 'true'),
    ('void metadata(', 'void unsafe(){vkDeviceWaitIdle(0);}void metadata('),
]


@pytest.mark.parametrize('old,new', MUTANTS)
def test_resource_mutants_fail(old, new):
    source = CPP.read_text()
    assert old in source
    with pytest.raises((AssertionError, ValueError)):
        guard(source.replace(old, new, 1))


@pytest.fixture(scope='module')
def allocation_binary(tmp_path_factory):
    directory = tmp_path_factory.mktemp('child-parser-resource')
    unit = directory / 'observe.cpp'
    unit.write_text(r'''
#include <cstdlib>
#include <cstdio>
#include <new>
static bool observe=false;static size_t allocated=0,largest=0,calls=0;
void* operator new(size_t n){if(observe){allocated+=n;++calls;if(n>largest)largest=n;}if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,size_t)noexcept{std::free(p);}void operator delete[](void*p,size_t)noexcept{std::free(p);}
#define CHILD_QUALIFIER_CPU_ONLY
#define main qualifier_original_main
#include "''' + str(CPP) + r'''"
#undef main
int main(int argc,char**argv){if(argc!=2)return 2;bool ok=false;observe=true;try{auto q=load(argv[1]);ok=true;}catch(const std::exception&){}observe=false;std::printf("{\"valid\":%s,\"allocated\":%zu,\"largest\":%zu,\"calls\":%zu}\n",ok?"true":"false",allocated,largest,calls);}
''')
    target = directory / 'observe'
    command = [shutil.which('g++'), '-std=c++17', '-O2', str(unit), '-o', str(target)]
    subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
    return target


def payload(w=640, h=480, P=4096, C=4096, count=0):
    maxbits, jumpbits = [struct.unpack('<Q', struct.pack('<d', x))[0] for x in (1.2, .012)]
    words = [w,h,1,0,P,C,4*w*h,8,1,17,0,23,0,29,0,maxbits&0xffffffff,maxbits>>32,jumpbits&0xffffffff,jumpbits>>32,count,w*h,*([0]*8)]
    return struct.pack('<12s29I', b'CHILDQv1\0\0\0\0', *words) + bytes(12*w*h +32*count)


def test_actual_parser_bounded_allocations(allocation_binary, tmp_path):
    good = tmp_path / 'largest.childq'
    data = bytearray(payload(count=4096))
    for j in range(4096):
        struct.pack_into('<8I', data, 128+12*307200+32*j, 0, j+1, j, 1, 2*(j%320), 2*(j//320), 1, 1)
    good.write_bytes(data)
    def observe(path):
        result = subprocess.run([str(allocation_binary), str(path)], capture_output=True, text=True, timeout=10, check=True)
        return json.loads(result.stdout)
    observed = observe(good)
    assert observed['valid'] is True
    assert observed['largest'] == 4*307200
    assert 12*307200+32*4096 <= observed['allocated'] < 12*307200+32*4096+100000
    bad = bytearray(good.read_bytes());struct.pack_into('<I',bad,12,0xffffffff)
    invalid = tmp_path/'malformed.childq';invalid.write_bytes(bad)
    rejected = observe(invalid)
    assert rejected['valid'] is False and rejected['largest'] <= 16384
    assert rejected['allocated'] < 32768
    (tmp_path/'allocation-observations.json').write_text(json.dumps(dict(maximum=observed,malformed=rejected),indent=2))


def test_cpu_only_binary_has_no_device_entry(allocation_binary):
    symbols = subprocess.run(['nm','-u',str(allocation_binary)],capture_output=True,text=True,check=True).stdout
    assert not re.search(r'vk[A-Z]|Context', symbols)
    libraries = subprocess.run(['ldd',str(allocation_binary)],capture_output=True,text=True,check=True).stdout
    assert 'libvulkan' not in libraries


def author_tests():
    spec = importlib.util.spec_from_file_location('child_qualifier_author_resources', ROOT/'tests/test_native_child_qualifier.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('remove_size_guard', [False, True])
def test_oversized_output_rejected_before_read(tmp_path, monkeypatch, remove_size_guard):
    author = author_tests()
    directory = tmp_path/'factory'
    manifest = author.q.prepare(directory)
    prefix, _ = author.synthetic((directory, manifest), tmp_path, 'one-parent-P1C2')
    output = Path(str(prefix)+'.roots.bin')
    with output.open('r+b') as stream:
        stream.truncate(output.stat().st_size+1)
    if remove_size_guard:
        function = inspect.getsource(author.q.compare)
        line = next(line for line in function.splitlines() if 'output.stat().st_size' in line)
        exec(function.replace(line, ''), author.q.__dict__)
    original = Path.read_bytes
    def bounded_read(path):
        assert path != output, 'oversized output read before length check'
        return original(path)
    monkeypatch.setattr(Path, 'read_bytes', bounded_read)
    with pytest.raises(AssertionError if remove_size_guard else ValueError,
                       match='before length check' if remove_size_guard else 'size'):
        author.q.compare(directory, 'one-parent-P1C2', prefix)
