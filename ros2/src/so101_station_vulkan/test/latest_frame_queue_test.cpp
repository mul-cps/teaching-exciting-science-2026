#include "so101_station_vulkan/latest_frame_queue.hpp"
#include "so101_station_vulkan/pipeline.hpp"
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>
using namespace so101_station_vulkan;
#define REQUIRE(x) do{if(!(x))throw std::runtime_error(#x);}while(false)
static bool count_allocations=false;static size_t allocation_count=0;
void* operator new(std::size_t n){if(count_allocations)++allocation_count;if(auto p=std::malloc(n))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{std::free(p);}void operator delete(void* p,std::size_t)noexcept{std::free(p);}
void* operator new[](std::size_t n){return ::operator new(n);}void operator delete[](void* p)noexcept{::operator delete(p);}void operator delete[](void* p,std::size_t)noexcept{::operator delete(p);}
struct FakeSlot {uint64_t id=0,received=0;double hardware=0;int domain=7;bool submitted=false,ready=true;int64_t due=0;};
struct Lease {std::shared_ptr<FakeSlot> p;explicit operator bool()const noexcept{return bool(p);}};
struct Pool {
 std::array<std::shared_ptr<FakeSlot>,3> slots;
 Pool(){for(auto& p:slots)p=std::make_shared<FakeSlot>();}
 Lease acquire(uint64_t id,uint64_t received){for(auto& p:slots)if(p.use_count()==1&&(!p->submitted||p->ready)){*p=FakeSlot{};p->id=id;p->received=received;p->hardware=100.+double(id)/30.;return {p};}return {};}
};
static void newest_and_lifetime(){
 Pool pool;LatestFrameQueue<Lease> q;REQUIRE(q.accept_source(1,1));q.offer(pool.acquire(1,10));q.dispatch_pending([](const auto& f){f.p->submitted=true;f.p->ready=false;});
 auto late=q.active();REQUIRE(q.accept_source(2,2));q.offer(pool.acquire(2,20));
 q.active().p->ready=true;auto completed=q.take_completed();REQUIRE(completed.p->id==1);
 // Completion and a new camera frame share this tick. Offer FIRST, then submit.
 REQUIRE(q.accept_source(3,3));q.offer(pool.acquire(3,30));uint64_t submitted=0;
 q.dispatch_pending([&](const auto& f){submitted=f.p->id;f.p->submitted=true;f.p->ready=false;});
 REQUIRE(submitted==3);REQUIRE(q.counters().replaced==1);REQUIRE(!q.has_pending());
 completed={};auto third=pool.acquire(4,40);REQUIRE(third);REQUIRE(!pool.acquire(5,50));
 // Published/copy-held frame1 is not recycled while the late consumer owns it.
 REQUIRE(late.p->id==1);late={};REQUIRE(pool.acquire(5,50));
 REQUIRE(!q.accept_source(3,3));REQUIRE(!q.accept_source(2,4));REQUIRE(!q.accept_source(4,2));
 REQUIRE(q.counters().duplicate_or_regressed==3);REQUIRE(q.accept_source(4,4));
 REQUIRE(q.counters().completed==1);
}
static void no_arrival_progress(){
 Pool pool;LatestFrameQueue<Lease> q;q.offer(pool.acquire(1,0));q.dispatch_pending([](const auto& f){f.p->submitted=true;f.p->ready=false;});q.offer(pool.acquire(2,1));
 q.active().p->ready=true;auto frame=q.take_completed();q.dispatch_pending([](const auto& f){f.p->submitted=true;});
 REQUIRE(frame.p->id==1);REQUIRE(q.active().p->id==2);
 auto age=source_received_age_ms(1000000,46000000);REQUIRE(age&&*age==45.);
 REQUIRE(!source_received_age_ms(1000000,999999));REQUIRE(!source_received_age_ms(0,-1));
}
static void simulate(int64_t processing_us){
 Pool pool;LatestFrameQueue<Lease> q;uint64_t incoming=1,last=0,published=0;int64_t next_source=0;
 for(int64_t now=0;now<3000000;now+=2000){
  if(q.active()&&now>=q.active().p->due){q.active().p->ready=true;auto f=q.take_completed();REQUIRE(f.p->id>last);last=f.p->id;++published;REQUIRE(f.p->domain==7);REQUIRE(f.p->hardware==100.+double(f.p->id)/30.);auto age=source_received_age_ms(f.p->received,now*1000);REQUIRE(age&&*age>=double(processing_us)/1000.);}
  if(now>=next_source){REQUIRE(q.accept_source(incoming,incoming));q.discard_pending();auto f=pool.acquire(incoming,uint64_t(now)*1000);if(f)q.offer(std::move(f));else q.no_slot();++incoming;next_source+=33333;}
  q.dispatch_pending([&](const auto& f){f.p->submitted=true;f.p->ready=false;f.p->due=now+processing_us;});
  REQUIRE(q.counters().completed==published);REQUIRE(pool.slots.size()==3);
 }
 REQUIRE(published>0);REQUIRE(q.counters().no_slot==0);
 if(processing_us==5000){REQUIRE(q.counters().replaced==0);REQUIRE(published>=89);}
 else {REQUIRE(q.counters().replaced>0);REQUIRE(published<uint64_t(incoming-1));}
 std::cout<<"processing_us="<<processing_us<<" completed="<<published<<" replaced="<<q.counters().replaced<<" no_slot="<<q.counters().no_slot<<"\n";
}
static void real_lease_lifetime(){
 auto pool=std::make_shared<FramePool>(16,16,nullptr,false);LatestFrameQueue<FrameLease> q;
 auto first=pool->acquire();first.slot().metadata.frame_number=17;first.slot().metadata.timestamp_ns=123456;
 first.slot().metadata.hardware_timestamp_ms=765.432;first.slot().metadata.timestamp_domain=8;
 first.slot().metadata.depth_frame_number=19;first.slot().metadata.depth_hardware_timestamp_ms=764.123;
 first.slot().metadata.depth_timestamp_domain=9;first.slot().metadata.calibration_generation=23;
 q.offer(std::move(first));q.dispatch_pending([](const FrameLease&){});auto late=q.active();
 auto done=q.take_completed();REQUIRE(!q.active());REQUIRE(done.slot().metadata.frame_number==17);
 REQUIRE(done.slot().metadata.timestamp_ns==123456);REQUIRE(done.slot().metadata.hardware_timestamp_ms==765.432);REQUIRE(done.slot().metadata.timestamp_domain==8);
 REQUIRE(done.slot().metadata.depth_frame_number==19);REQUIRE(done.slot().metadata.depth_hardware_timestamp_ms==764.123);
 REQUIRE(done.slot().metadata.depth_timestamp_domain==9);REQUIRE(done.slot().metadata.calibration_generation==23);
 auto second=pool->acquire(),third=pool->acquire();REQUIRE(second&&third);done={};REQUIRE(!pool->acquire());late={};REQUIRE(pool->acquire());
}
static void telemetry_format(){
 FrameMetadata m;m.frame_number=UINT64_MAX;m.depth_frame_number=UINT64_MAX-1;m.timestamp_ns=UINT64_MAX;m.calibration_generation=UINT64_MAX;
 m.hardware_timestamp_ms=std::numeric_limits<double>::max();m.depth_hardware_timestamp_ms=-std::numeric_limits<double>::max();m.timestamp_domain=INT32_MIN;m.depth_timestamp_domain=INT32_MAX;
 std::string text;text.reserve(kMeasurementTextCapacity);const auto capacity=text.capacity();
 format_unqualified_measurement(text,m,UINT32_MAX,INT64_MAX);
 REQUIRE(text.capacity()==capacity);REQUIRE(text.size()<kMeasurementTextCapacity);REQUIRE(text.find("received_age_ms=unknown")!=std::string::npos);
 REQUIRE(text.find("source_frame=18446744073709551615")!=std::string::npos);REQUIRE(text.find("timestamp_domain=-2147483648")!=std::string::npos);
 REQUIRE(text.find("depth_source_frame=18446744073709551614;")!=std::string::npos);
 REQUIRE(text.find("depth_timestamp_domain=2147483647;")!=std::string::npos);
 REQUIRE(text.find("calibration_generation=18446744073709551615;")!=std::string::npos);
 REQUIRE(text.find("geometry_admission=unqualified; execution_eligible=false")!=std::string::npos);
 const auto start=text.find("hardware_timestamp_ms=")+std::string("hardware_timestamp_ms=").size();
 REQUIRE(std::stod(text.substr(start,text.find(';',start)-start))==m.hardware_timestamp_ms);
 // Model publish(const Message&): downstream retains its own copied snapshot.
 const std::string held=text;m.frame_number=18;m.timestamp_ns=1000000;m.hardware_timestamp_ms=NAN;
 format_unqualified_measurement(text,m,3,46000000);REQUIRE(held.find("source_frame=18446744073709551615")!=std::string::npos);
 REQUIRE(text.find("source_frame=18;")==0);REQUIRE(text.find("hardware_timestamp_ms=unknown")!=std::string::npos);REQUIRE(text.find("received_age_ms=45;")!=std::string::npos);
 format_unqualified_measurement(text,m,3,-1);REQUIRE(text.find("received_age_ms=unknown")!=std::string::npos);
 std::string short_buffer;bool threw=false;try{format_unqualified_measurement(short_buffer,m,3,46000000);}catch(const std::length_error&){threw=true;}REQUIRE(threw);REQUIRE(short_buffer.empty());
}
static void benchmark(){
 auto pool=std::make_shared<FramePool>(16,16,nullptr,false);LatestFrameQueue<FrameLease> q;volatile uint64_t checksum=0;
 std::string text;text.reserve(kMeasurementTextCapacity);
 constexpr uint64_t iterations=100000;allocation_count=0;count_allocations=true;
 auto start=std::chrono::steady_clock::now();
 for(uint64_t i=1;i<=iterations;++i){auto input=pool->acquire();input.slot().metadata.frame_number=i;q.accept_source(i,i);q.offer(std::move(input));q.dispatch_pending([&](const FrameLease& f){checksum=checksum+f.slot().metadata.frame_number;});auto f=q.take_completed();format_unqualified_measurement(text,f.slot().metadata,3,123456789);checksum=checksum^f.slot().metadata.frame_number;}
 auto end=std::chrono::steady_clock::now();count_allocations=false;
 REQUIRE(allocation_count==0);REQUIRE(q.counters().completed==iterations);REQUIRE(q.counters().replaced==0);REQUIRE(!q.active()&&!q.has_pending());
 std::cout<<"{\"iterations\":"<<iterations<<",\"host_ns_per_cycle\":"<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count())/iterations<<",\"hot_heap_allocations\":"<<allocation_count<<",\"checksum\":"<<checksum<<",\"real_cpu_frame_lease\":true,\"gpu\":false,\"physical\":false}\n";
}
int main()try{newest_and_lifetime();no_arrival_progress();real_lease_lifetime();telemetry_format();for(auto us:{5000,45000,80000})simulate(us);benchmark();std::cout<<"PASS latest frame queue/source age/lease lifetime\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
