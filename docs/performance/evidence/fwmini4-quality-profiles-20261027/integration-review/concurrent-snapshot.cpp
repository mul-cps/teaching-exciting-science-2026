#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <atomic>
#include <thread>
#include <cassert>
#include <iostream>
using namespace so101_station_vulkan;
struct Native {uint64_t id=0;explicit operator bool()const{return id!=0;}};
struct Meta {uint64_t frame_number=0,depth_frame_number=10,calibration_generation=7;double hardware_timestamp_ms=0,depth_hardware_timestamp_ms=.5;int timestamp_domain=1,depth_timestamp_domain=2;std::string serial="stable-long-source-serial-outside-small-string-capacity";};
using Q=LatestNativeScheduler<Native,Meta>;
int main(){Q q;q.reserve_serial(128);std::atomic<bool> start{false},producer_done{false},worker_done{false};std::atomic<uint64_t> successes{0},failures{0},during_both{0};
 std::thread producer([&]{Meta m;NativeSourceAdmission a;a.seed(m);while(!start.load()){}for(uint64_t i=1;i<=100000;++i){m.frame_number=i;m.depth_frame_number=i+10;m.hardware_timestamp_ms=i;m.depth_hardware_timestamp_ms=i+.5;Native n{i};q.accept(n,m,a.observe(m),i);if(i%37==0)std::this_thread::yield();}producer_done.store(true,std::memory_order_release);});
 std::thread worker([&]{Meta m;m.serial.reserve(128);while(!start.load()){}while(true){Native n;if(q.take(n,m)){q.submitted();q.computed();q.complete(n.id,1);}else if(producer_done.load(std::memory_order_acquire))break;else std::this_thread::yield();}worker_done.store(true,std::memory_order_release);});
 std::thread observer([&]{while(!start.load()){}Q::Snapshot s;while(!worker_done.load(std::memory_order_acquire)){if(q.try_snapshot(s,8)){assert(s.conserved()&&s.active<=1&&s.pending<=1);++successes;if(!producer_done.load(std::memory_order_acquire))++during_both;}else ++failures;}});
 start=true;producer.join();worker.join();observer.join();auto s=q.snapshot();assert(s.conserved()&&s.received==100000&&s.active==0&&s.pending==0);assert(successes>0&&during_both>0);
 std::cout<<"{\"offered\":100000,\"accepted_consistent_snapshots\":"<<successes<<",\"accepted_while_producer_and_worker_live\":"<<during_both<<",\"bounded_retry_failures\":"<<failures<<",\"completed\":"<<s.completed<<",\"replaced\":"<<s.replaced<<",\"contention\":"<<s.contention<<"}\n";
}
