
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
thread_local bool inside=false;double bounded_snapshot_ns=0;
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
 typename decltype(q)::Snapshot prior;prior.received=777;prior.unique=778;
 auto mark=allocations.load();auto timer=Clock::now();for(unsigned i=0;i<N;i++)require(!q.try_snapshot(prior,8));bounded_snapshot_ns=ns(timer)/N;
 require(prior.received==777&&prior.unique==778&&allocations.load()==mark);
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
 std::cout<<"{\"calls_per_cost\":"<<N<<",\"intake_replace_ns\":"<<c.intake<<",\"admission_observe_ns\":"<<c.admission<<",\"duplicate_ns\":"<<c.duplicate<<",\"stopped_skip_ns\":"<<c.stopped<<",\"dispatch_raw_ns\":"<<c.dispatch_raw<<",\"skip_no_slot_raw_ns\":"<<c.skip_raw<<",\"clock_pair_baseline_ns\":"<<c.clock_pair<<",\"full_cycle_ns\":"<<c.cycle<<",\"contended_intake_ns\":"<<c.contended<<",\"bounded_snapshot_stale_ns\":"<<bounded_snapshot_ns<<",\"hot_heap_allocations\":0,\"owned_handle_releases_inside_scheduler\":"<<owned_release_inside.load()<<",\"simulated_30Hz_offered\":300,\"overload_ms\":[80,180],\"blocked_publication_latest_id\":300,\"contention_attempts\":"<<N<<"}\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<" owned_releases_inside="<<owned_release_inside.load()<<'\n';return 1;}}
