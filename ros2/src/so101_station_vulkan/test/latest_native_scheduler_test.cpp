#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
namespace {
std::atomic<uint64_t> allocations{0};
void require(bool ok){if(!ok)throw std::runtime_error("latest-native contract failed");}
struct Native {uint64_t id=0;explicit operator bool()const{return id!=0;}};
struct Metadata {
 uint64_t frame_number=0,depth_frame_number=0,timestamp_ns=0,calibration_generation=7;
 double hardware_timestamp_ms=0,depth_hardware_timestamp_ms=0;
 int timestamp_domain=1,depth_timestamp_domain=2;std::string serial;
};
using Queue=LatestNativeScheduler<Native,Metadata>;
Metadata metadata(uint64_t id){Metadata m;m.frame_number=id;m.depth_frame_number=id+10;m.timestamp_ns=id*33333333;m.hardware_timestamp_ms=double(id);m.depth_hardware_timestamp_ms=double(id)+.5;m.serial="source-serial-that-exceeds-small-string-capacity";return m;}
void conservation(const Queue::Snapshot& s){require(s.received==s.unique+s.duplicate+s.invalid);require(s.unique==s.completed+s.skipped()+s.active+s.pending);require(s.pending<=1&&s.active<=1);}
void overload(uint64_t duration){
 Queue q;q.reserve_serial(100);NativeSourceAdmission admission;auto m=metadata(0);admission.seed(m);Metadata selected;selected.serial.reserve(100);Native active;uint64_t offered=0,copied=0,due=0;
 for(uint64_t now=0;now<11000000000ull;now+=1000000){
  if(active&&now>=due){q.computed();q.complete(now,now-selected.timestamp_ns);active={};}
  if(offered<300&&now>=offered*33333333){++offered;m=metadata(offered);Native input{offered};q.accept(input,m,admission.observe(m),now);}
  if(!active&&q.take(active,selected)){require(active.id==offered);require(selected.timestamp_ns==active.id*33333333);require(selected.hardware_timestamp_ms==double(active.id));++copied;q.submitted();due=now+duration;}
  conservation(q.snapshot());
 }
 require(offered==300&&!active);auto s=q.snapshot();conservation(s);require(s.received==300&&s.completed==copied&&s.replaced>0&&s.pending==0);
}
void observed_watermarks(){
 NativeSourceAdmission a;a.seed(metadata(10));auto m=metadata(12);m.depth_frame_number=20;require(!a.observe(m).unique);
 bool failed=false;try{a.observe(metadata(11));}catch(const std::runtime_error&){failed=true;}require(failed);
 NativeSourceAdmission b;b.seed(metadata(10));m=metadata(10);m.hardware_timestamp_ms=20;require(!b.observe(m).unique);failed=false;try{b.observe(metadata(11));}catch(const std::runtime_error&){failed=true;}require(failed);
}
void invalid_stop_and_timing(){
 Queue q;q.reserve_serial(100);NativeSourceAdmission admission;admission.seed(metadata(1));
 // Malformed input is checked even while a newer pending cohort would replace it.
 auto m=metadata(2);Native native{2};q.accept(native,m,admission.observe(m),100);q.invalid_received();
 for(unsigned field=0;field<4;++field){NativeSourceAdmission a;a.seed(metadata(1));auto bad=metadata(3);if(field==0)bad.calibration_generation++;if(field==1)bad.serial="other";if(field==2)bad.timestamp_domain++;if(field==3)bad.depth_timestamp_domain++;bool rejected=false;try{a.observe(bad);}catch(const std::runtime_error&){rejected=true;}require(rejected);}
 q.capture_service(33333333,33333333);q.capture_service(33333334,33333334);
 m=metadata(4);native.id=4;q.accept(native,m,admission.observe(m),40000100);
 q.request_stop();Native released;q.clear_pending(released);require(released.id==4);
 m=metadata(5);native.id=5;require(!q.accept(native,m,admission.observe(m),80000100));
 auto state=q.snapshot();conservation(state);require(state.invalid==1&&state.replaced==1&&state.stopped_skip==2&&state.color_gap==1&&state.depth_gap==1);
 require(state.capture_gap_deadlines==1&&state.capture_duration_deadlines==1&&state.intake_gap_deadlines==2&&state.intake_gap_max_ns==40000000);
}
struct BlockingMetadata:Metadata {
 inline static std::atomic<bool> entered{false},release{false};inline static const BlockingMetadata* target=nullptr;
 BlockingMetadata& operator=(const BlockingMetadata& other){Metadata::operator=(other);if(this==target){entered=true;while(!release.load())std::this_thread::yield();}return *this;}
};
void contention(){
 LatestNativeScheduler<Native,BlockingMetadata> q;q.reserve_serial(100);BlockingMetadata m,out;static_cast<Metadata&>(m)=metadata(1);out.serial.reserve(100);Native n{1};NativeSourceAdmission a;a.seed(metadata(0));q.accept(n,m,a.observe(m),1);
 BlockingMetadata::target=&out;
 std::thread worker([&]{Native taken;require(q.take(taken,out));q.cancel_active();});
 while(!BlockingMetadata::entered.load())std::this_thread::yield();
 LatestNativeScheduler<Native,BlockingMetadata>::Snapshot diagnostic;diagnostic.received=999;require(!q.try_snapshot(diagnostic,8)&&diagnostic.received==999);
 static_cast<Metadata&>(m)=metadata(2);n.id=2;require(!q.accept(n,m,a.observe(m),2));require(n.id==2);
 BlockingMetadata::release=true;worker.join();auto s=q.snapshot();require(s.contention==1&&s.conserved());BlockingMetadata::target=nullptr;
}
void blocked_publication(){
 Queue q;q.reserve_serial(100);Metadata m=metadata(1),out;out.serial.reserve(100);Native input{1};NativeSourceAdmission a;a.seed(metadata(0));q.accept(input,m,a.observe(m),1);
 std::atomic<bool> entered{false},release{false};
 std::thread worker([&]{Native n;require(q.take(n,out));q.submitted();q.computed();entered=true;while(!release.load())std::this_thread::yield();q.complete(100,50);});
 while(!entered.load())std::this_thread::yield();
 for(uint64_t id=2;id<=300;++id){m=metadata(id);Native n{id};q.accept(n,m,a.observe(m),id*33333333);}
 auto s=q.snapshot();require(s.received==300&&s.pending==1&&s.active==1&&s.replaced==298);conservation(s);release=true;worker.join();
 Native n;require(q.take(n,out)&&n.id==300);q.skip_no_slot();conservation(q.snapshot());
 q.request_stop();Native displaced;require(!q.take(n,out));q.clear_pending(displaced);conservation(q.snapshot());
}
}
void* operator new(std::size_t n){if(void* p=std::malloc(n?n:1)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}void operator delete[](void* p)noexcept{std::free(p);}void operator delete(void* p,std::size_t)noexcept{std::free(p);}void operator delete[](void* p,std::size_t)noexcept{std::free(p);}
int main(){try{
 overload(80000000);overload(180000000);observed_watermarks();invalid_stop_and_timing();blocked_publication();contention();
 Queue q;q.reserve_serial(100);Metadata m=metadata(1),out;out.serial.reserve(100);Native n;NativeSourceAdmission a;a.seed(metadata(0));
 auto before=allocations.load();auto start=std::chrono::steady_clock::now();
 for(uint64_t id=1;id<=100000;++id){m.frame_number=id;m.depth_frame_number=id+10;m.hardware_timestamp_ms=id;m.depth_hardware_timestamp_ms=id+.5;m.timestamp_ns=id*33333333;n.id=id;q.accept(n,m,a.observe(m),m.timestamp_ns);require(q.take(n,out));q.submitted();q.computed();q.complete(m.timestamp_ns+1,1);n={};}
 auto elapsed=std::chrono::duration<double,std::nano>(std::chrono::steady_clock::now()-start).count()/100000;
 require(allocations.load()==before);conservation(q.snapshot());
 std::cout<<"{\"offered_per_overload_case\":300,\"overload_cases\":2,\"max_pending\":1,\"max_active\":1,\"hot_heap_allocations\":0,\"host_ns_per_cycle\":"<<elapsed<<",\"blocked_publication_intake_progress\":true,\"contention_discard_nonblocking\":true}\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
