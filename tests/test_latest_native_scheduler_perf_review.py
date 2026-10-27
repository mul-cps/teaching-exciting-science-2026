"""Independent SDK-free scheduler ownership, resource and host-cost witnesses."""
from pathlib import Path
import json
import os
import re
import shutil
import subprocess

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'
HEADER = P / 'include/so101_station_vulkan/latest_native_scheduler.hpp'


def intake_body(source):
    start = source.index(' bool accept(')
    left = source.index('{', start)
    depth = 1
    end = left + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[left:end]


def guard(source):
    body = re.sub(r'//[^\n]*|/\*.*?\*/', '', intake_body(source), flags=re.S)
    assert re.search(r'std::unique_lock\s*<\s*std::mutex\s*>\s+lock\s*\(\s*mutex_\s*,\s*std::try_to_lock\s*\)', body)
    assert 'lock.owns_lock()' in body
    forbidden = r'\b(?:new|malloc|calloc|realloc|memcpy|memmove|serialize|publish|glFinish|glReadPixels|vkDeviceWaitIdle|vkQueueWaitIdle)\b|std::(?:lock_guard|scoped_lock)|\.(?:lock|wait|wait_for|wait_until|push|push_back|emplace|emplace_back|resize|reserve)\s*\(|\b(?:sleep_for|sleep_until|yield)\s*\('
    assert not re.search(forbidden, body)
    assert re.search(r'swap\s*\(\s*pending_\s*,\s*incoming\s*\)', body)


CPP = r'''
#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <new>
#include <thread>
#include <stdexcept>
using namespace so101_station_vulkan;
using Clock=std::chrono::steady_clock;
std::atomic<uint64_t> allocations{0},owned_release_inside{0};
thread_local bool inside=false;
void require(bool x){if(!x)throw std::runtime_error("independent resource contract");}
struct Handle{
 uint64_t id=0;
 Handle()=default;explicit Handle(uint64_t n):id(n){}
 Handle(const Handle&)=delete;Handle& operator=(const Handle&)=delete;
 Handle(Handle&& n)noexcept:id(std::exchange(n.id,0)){}
 void reset(){if(id&&inside)++owned_release_inside;id=0;}
 Handle& operator=(Handle&& n)noexcept{reset();id=std::exchange(n.id,0);return *this;}
 ~Handle(){reset();}
 explicit operator bool()const{return id!=0;}
};
struct Meta{
 uint64_t frame_number=1,depth_frame_number=11,calibration_generation=7;
 double hardware_timestamp_ms=1,depth_hardware_timestamp_ms=1.5;
 int timestamp_domain=1,depth_timestamp_domain=2;
 std::string serial="stable-camera-serial-outside-small-string-storage";
};
using Q=LatestNativeScheduler<Handle,Meta>;
constexpr unsigned N=100000;
struct Call{Call(){inside=true;}~Call(){inside=false;}};
double ns(Clock::time_point a){return std::chrono::duration<double,std::nano>(Clock::now()-a).count();}
void stable(const Q::Snapshot&s){require(s.conserved()&&s.active<=1&&s.pending<=1);}
struct Costs{double intake,duplicate,stopped,dispatch_raw,skip_raw,clock_pair,cycle,contended,admission;};
Costs hot(){
 Costs c{};Q q;q.reserve_serial(128);Meta m,out;out.serial.reserve(128);Handle in,selected;
 auto before=allocations.load();auto start=Clock::now();
 for(unsigned i=1;i<=N;i++){in=Handle(i);{Call call;require(q.accept(in,m,{true},i));}in.reset();}
 c.intake=ns(start)/N;stable(q.snapshot());require(q.snapshot().replaced==N-1);
 {Call call;require(q.take(selected,out));q.complete(1,1);}selected.reset();
 start=Clock::now();for(unsigned i=0;i<N;i++){Call call;require(!q.accept(in,m,{false},i));}c.duplicate=ns(start)/N;
 double dispatch=0,skip=0,clock_pair=0;
 for(unsigned i=1;i<=N;i++){
  in=Handle(i);{Call call;require(q.accept(in,m,{true},i));}
  {Call call;auto t=Clock::now();require(q.take(selected,out));dispatch+=ns(t);}
  {Call call;auto t=Clock::now();q.skip_no_slot();skip+=ns(t);}selected.reset();
  auto t=Clock::now();clock_pair+=ns(t);
 }
 c.dispatch_raw=dispatch/N;c.skip_raw=skip/N;c.clock_pair=clock_pair/N;stable(q.snapshot());
 start=Clock::now();for(unsigned i=1;i<=N;i++){
  in=Handle(i);{Call call;require(q.accept(in,m,{true},i));require(q.take(selected,out));q.submitted();q.computed();q.complete(i,1);}selected.reset();
 }c.cycle=ns(start)/N;stable(q.snapshot());
 q.request_stop();start=Clock::now();for(unsigned i=0;i<N;i++){Call call;require(!q.accept(in,m,{true},i));}c.stopped=ns(start)/N;stable(q.snapshot());
 require(allocations.load()==before);
 NativeSourceAdmission admission;m.frame_number=0;m.depth_frame_number=10;m.hardware_timestamp_ms=0;m.depth_hardware_timestamp_ms=.5;admission.seed(m);
 // seed() is cold and may allocate its fixed serial storage.
 before=allocations.load();start=Clock::now();
 for(unsigned i=1;i<=N;i++){m.frame_number=i;m.depth_frame_number=i+10;m.hardware_timestamp_ms=i;m.depth_hardware_timestamp_ms=i+.5;require(admission.observe(m).unique);}
 c.admission=ns(start)/N;
 require(allocations.load()==before);require(owned_release_inside.load()==0);return c;
}
// Real worker waits behind a simulated final publisher. It owns its input until
// complete(); capture must keep replacing just one pending handle meanwhile.
void blocked_publisher(){
 Q q;q.reserve_serial(128);Meta m,out;out.serial.reserve(128);Handle in(1);
 q.accept(in,m,{true},1);std::mutex gate;std::condition_variable cv;bool entered=false,release=false;
 std::thread worker([&]{Handle n;{Call call;require(q.take(n,out));q.submitted();q.computed();}
  {std::unique_lock<std::mutex> l(gate);entered=true;cv.notify_one();cv.wait(l,[&]{return release;});}
  {Call call;q.complete(1,1);}
 });
 {std::unique_lock<std::mutex> l(gate);cv.wait(l,[&]{return entered;});}
 auto before=allocations.load();for(unsigned i=2;i<=300;i++){Handle n(i);{Call call;require(q.accept(n,m,{true},i));}}
 require(allocations.load()==before);auto s=q.snapshot();stable(s);require(s.active==1&&s.pending==1&&s.replaced==298);
 {std::lock_guard<std::mutex> l(gate);release=true;}cv.notify_one();worker.join();Handle n;
 {Call call;require(q.take(n,out));}require(n.id==300);{Call call;q.skip_no_slot();}stable(q.snapshot());require(owned_release_inside.load()==0);
}
struct BlockingMeta:Meta{
 static inline std::mutex gate;static inline std::condition_variable cv;
 static inline bool entered=false,release=false;static inline const BlockingMeta* target=nullptr;
 BlockingMeta& operator=(const BlockingMeta& other){Meta::operator=(other);if(this==target){std::unique_lock<std::mutex> l(gate);entered=true;cv.notify_one();cv.wait(l,[]{return release;});}return *this;}
};
double contention(){
 LatestNativeScheduler<Handle,BlockingMeta> q;q.reserve_serial(128);BlockingMeta m,out;out.serial.reserve(128);Handle in(1);q.accept(in,m,{true},1);BlockingMeta::target=&out;
 std::thread worker([&]{Handle n;require(q.take(n,out));q.cancel_active();});
 {std::unique_lock<std::mutex> l(BlockingMeta::gate);BlockingMeta::cv.wait(l,[]{return BlockingMeta::entered;});}
 auto before=allocations.load();auto t=Clock::now();for(unsigned i=1;i<=N;i++){Handle n(i);{Call call;require(!q.accept(n,m,{true},i));}require(n.id==i);}double elapsed=ns(t)/N;
 require(allocations.load()==before);{std::lock_guard<std::mutex> l(BlockingMeta::gate);BlockingMeta::release=true;}BlockingMeta::cv.notify_one();worker.join();auto s=q.snapshot();require(s.conserved()&&s.contention==N);BlockingMeta::target=nullptr;return elapsed;
}
void overload(uint64_t duration){
 Q q;q.reserve_serial(128);Meta m,out;out.serial.reserve(128);Handle active;unsigned offered=0,done=0;uint64_t due=0;
 for(uint64_t now=0;now<11000000000ULL;now+=1000000){
  if(active&&now>=due){{Call call;q.complete(now,1);}active.reset();done++;}
  if(offered<300&&now>=offered*33333333ULL){Handle n(++offered);{Call call;q.accept(n,m,{true},now);}}
  if(!active){{Call call;if(q.take(active,out)){require(active.id==offered);due=now+duration;}}}
  stable(q.snapshot());
 }
 require(offered==300&&!active&&q.snapshot().pending==0&&done==q.snapshot().completed&&q.snapshot().replaced>0);
}
void shutdown_ownership(){
 Q q;q.reserve_serial(128);Meta m;Handle input(71),retired;{Call call;require(q.accept(input,m,{true},1));}
 q.request_stop();{Call call;q.clear_pending(retired);}require(retired.id==71&&!input);stable(q.snapshot());require(owned_release_inside.load()==0);
}
void* operator new(std::size_t n){if(void*p=std::malloc(n?n:1)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,std::size_t)noexcept{std::free(p);}void operator delete[](void*p,std::size_t)noexcept{std::free(p);}
int main(){try{
 overload(80000000);overload(180000000);blocked_publisher();shutdown_ownership();auto c=hot();c.contended=contention();
 std::cout<<"{\"calls_per_cost\":"<<N<<",\"intake_replace_ns\":"<<c.intake<<",\"admission_observe_ns\":"<<c.admission<<",\"duplicate_ns\":"<<c.duplicate<<",\"stopped_skip_ns\":"<<c.stopped<<",\"dispatch_raw_ns\":"<<c.dispatch_raw<<",\"skip_no_slot_raw_ns\":"<<c.skip_raw<<",\"clock_pair_baseline_ns\":"<<c.clock_pair<<",\"full_cycle_ns\":"<<c.cycle<<",\"contended_intake_ns\":"<<c.contended<<",\"hot_heap_allocations\":0,\"owned_handle_releases_inside_scheduler\":"<<owned_release_inside.load()<<",\"simulated_30Hz_offered\":300,\"overload_ms\":[80,180],\"blocked_publication_latest_id\":300,\"contention_attempts\":"<<N<<"}\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<" owned_releases_inside="<<owned_release_inside.load()<<'\n';return 1;}}
'''


def test_intake_static_guard_and_negative_mutants():
    source = HEADER.read_text()
    guard(source)
    mutants = [source.replace('std::try_to_lock', 'std::defer_lock'),
               source.replace('if(!lock.owns_lock())', 'lock.lock();if(!lock.owns_lock())'),
               source.replace('Write write(*this);++counts_.received;', 'Write write(*this);memcpy(a,b,n);++counts_.received;', 1),
               source.replace('Write write(*this);++counts_.received;', 'Write write(*this);serialize(incoming);++counts_.received;', 1),
               source.replace('Write write(*this);++counts_.received;', 'Write write(*this);vkDeviceWaitIdle(device);++counts_.received;', 1),
               source.replace('Write write(*this);++counts_.received;', 'Write write(*this);queue.push_back(incoming);++counts_.received;', 1)]
    for mutant in mutants:
        assert mutant != source
        try:
            guard(mutant)
        except AssertionError:
            pass
        else:
            raise AssertionError('forbidden intake resource mutant accepted')


def test_actual_scheduler_independent_ownership_contention_and_hot_cost(tmp_path):
    cpp = tmp_path / 'review.cpp'
    binary = tmp_path / 'review'
    cpp.write_text(CPP)
    built = subprocess.run([shutil.which('c++'), '-std=c++17', '-O2', '-pthread', '-Wall', '-Wextra', '-Werror',
                            '-I', str(P / 'include'), str(cpp), '-o', str(binary)], capture_output=True, text=True)
    assert built.returncode == 0, built.stderr
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(result.stdout)
    assert report['hot_heap_allocations'] == report['owned_handle_releases_inside_scheduler'] == 0
    assert report['calls_per_cost'] == report['contention_attempts'] == 100000
    assert report['simulated_30Hz_offered'] == report['blocked_publication_latest_id'] == 300
    assert report['overload_ms'] == [80, 180]
    for name in ('intake_replace_ns', 'admission_observe_ns', 'duplicate_ns', 'stopped_skip_ns', 'dispatch_raw_ns', 'skip_no_slot_raw_ns', 'full_cycle_ns', 'contended_intake_ns'):
        assert report[name] > 0
    if os.environ.get('LATEST_NATIVE_REVIEW_ARTIFACT_DIR'):
        destination = Path(os.environ['LATEST_NATIVE_REVIEW_ARTIFACT_DIR'])
        destination.mkdir(parents=True, exist_ok=True)
        (destination / 'microbenchmark.json').write_text(json.dumps(report, indent=2)+'\n')
        (destination / 'review.cpp').write_text(CPP)

def resource_body(source, signature):
    begin=source.index('{',source.index(signature));depth=1
    for end in range(begin+1,len(source)):
        depth+=(source[end]=='{')-(source[end]=='}')
        if not depth:return source[begin+1:end]
    raise AssertionError(signature)


def capture_resource_guard(camera, gate):
    # Sleep-until2ms is the explicit outer polling cadence, not shared-worker
    # backpressure. The actual intake helpers cannot sleep/wait/notify or copy.
    forbidden=r'\b(?:new|malloc|calloc|realloc|memcpy|memmove|serialize|vkDeviceWaitIdle|vkQueueWaitIdle)\b|bridge_|\.publish\s*\(|->publish\s*\(|copy_raw_planes|copy_vendor_planes|vendor_alignment_|\.snapshot\s*\(|try_snapshot\s*\(|(?:std::)?(?:lock_guard|unique_lock|scoped_lock)|\.lock\s*\(|\.wait(?:_for|_until)?\s*\(|\.(?:resize|reserve|push_back|emplace_back)\s*\('
    for signature in ('void capture_loop(', 'void capture_pending(', 'void record_failure(', 'void original_metadata(', 'void verify_original(', 'CameraSourceObservation inspect_original('):
        code=resource_body(camera,signature)
        assert not re.search(forbidden,code), signature
        if signature!='void capture_loop(':assert not re.search(r'\b(?:sleep_until|sleep_for|yield)\s*\(',code)
    polling=resource_body(camera,'void capture_loop(')
    assert polling.count('std::this_thread::sleep_until(')==1
    assert 'std::this_thread::sleep_for(' not in polling
    assert 'std::chrono::nanoseconds(started))+2ms)' in polling
    failure=resource_body(camera,'void record_failure(')
    assert 'ownership_.revoke()' in failure and 'fault_claimed_.compare_exchange_strong' in failure
    assert 'publication_gate()' not in failure
    report=resource_body(camera,'void report_status(')
    assert 'scheduler_.try_snapshot(status_counts_,8)' in report
    assert 'if(snapshot_fresh)status_counts_ns_=sampled_ns' in report
    assert 'counter_snapshot_fresh=' in report and 'counter_snapshot_age_ms=' in report
    revoke=resource_body(gate,'bool revoke(')
    assert 'compare_exchange_strong(token,0,std::memory_order_acq_rel)' in revoke
    assert not re.search(r'lock|wait|new|malloc|publish|invalidate',revoke)


def test_actual_capture_static_resource_boundary_and_mutants():
    camera=(P/'src/camera_component.cpp').read_text();gate=(P/'include/so101_station_vulkan/raw_camera_admission.hpp').read_text()
    capture_resource_guard(camera,gate)
    for addition in ('std::lock_guard<std::mutex> lock(mutex_);','bridge_->collect(frame);','memcpy(a,b,n);','frames_->publish(frame);','scheduler_.snapshot();','publication_gate().invalidate(this,1);'):
        c=camera.replace('void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{','void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{'+addition)
        try:capture_resource_guard(c,gate)
        except AssertionError:pass
        else:raise AssertionError(addition)
    for addition in ('std::lock_guard<std::mutex> lock(mutex_);','publish();','wait();'):
        g=gate.replace('bool revoke(uint64_t token)noexcept {','bool revoke(uint64_t token)noexcept {'+addition)
        try:capture_resource_guard(camera,g)
        except AssertionError:pass
        else:raise AssertionError(addition)


def test_actual_bounded_observer_stale_sample_under_writer(tmp_path):
    cpp=CPP.replace('thread_local bool inside=false;','thread_local bool inside=false;double bounded_snapshot_ns=0;')
    anchor='auto before=allocations.load();auto t=Clock::now();for(unsigned i=1;i<=N;i++){Handle n(i);'
    replacement='''typename decltype(q)::Snapshot prior;prior.received=777;prior.unique=778;
 auto mark=allocations.load();auto timer=Clock::now();for(unsigned i=0;i<N;i++)require(!q.try_snapshot(prior,8));bounded_snapshot_ns=ns(timer)/N;
 require(prior.received==777&&prior.unique==778&&allocations.load()==mark);
 auto before=allocations.load();auto t=Clock::now();for(unsigned i=1;i<=N;i++){Handle n(i);'''
    assert anchor in cpp
    cpp=cpp.replace(anchor,replacement)
    anchor='<<c.contended<<",\\\"hot_heap_allocations'
    replacement='<<c.contended<<",\\\"bounded_snapshot_stale_ns\\\":"<<bounded_snapshot_ns<<",\\\"hot_heap_allocations'
    assert anchor in cpp
    cpp=cpp.replace(anchor,replacement)
    source=tmp_path/'bounded.cpp';source.write_text(cpp);binary=tmp_path/'bounded'
    subprocess.run(['c++','-std=c++17','-O2','-pthread','-Wall','-Wextra','-Werror','-I',str(P/'include'),str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True,timeout=30)
    report=json.loads(result.stdout);assert report['bounded_snapshot_stale_ns']>0 and report['hot_heap_allocations']==0
    if os.environ.get('LATEST_NATIVE_REVIEW_ARTIFACT_DIR'):
        destination=Path(os.environ['LATEST_NATIVE_REVIEW_ARTIFACT_DIR']);destination.mkdir(parents=True,exist_ok=True)
        (destination/'bounded-observer.cpp').write_text(cpp)
        (destination/'bounded-observer.json').write_text(json.dumps(report,indent=2)+'\n')


def test_maximum_payload_store_orders_the_reader_acquire_fence():
    # Maxima can be the only newly observed payload in a bounded snapshot. Its
    # store must participate in the reader's trailing acquire-fence ordering.
    # TSan cannot establish this fence-based snapshot-coherence contract.
    def check(source):
        code=resource_body(source, 'static void maximum(')
        assert re.search(r'field\.store\(value,std::memory_order_release\)', code)
        assert 'memory_order_relaxed' not in code[code.index('field.store'):]
    source=HEADER.read_text()
    check(source)
    mutant=source.replace('field.store(value,std::memory_order_release)',
                          'field.store(value,std::memory_order_relaxed)')
    assert mutant != source
    try:
        check(mutant)
    except AssertionError:
        pass
    else:
        raise AssertionError('relaxed maxima payload store accepted')
