
#include "so101_station_vulkan/raw_camera_admission.hpp"
#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <thread>
#include <cassert>
#include <exception>
#include <new>
#include <cstdlib>
using namespace so101_station_vulkan;
std::atomic<bool> fail_heap{false};
void* operator new(std::size_t n){if(fail_heap.load())throw std::bad_alloc();if(void*p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{std::free(p);}void operator delete(void*p,std::size_t)noexcept{std::free(p);}
struct Native{explicit operator bool()const{return false;}};
struct Harness{
 ResidentCameraOwner ownership_;LatestNativeScheduler<Native,FrameMetadata> scheduler_;
 std::atomic<bool> stopping_{false},fault_claimed_{false},fault_ready_{false};std::exception_ptr failure_;FrameMetadata failure_input_;
 Harness(ResidentPublicationGate& g):ownership_(g,this){failure_input_.serial.reserve(128);g.admit(this,ownership_.token(),7);}
 void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{
  ownership_.revoke();stopping_.store(true,std::memory_order_release);scheduler_.request_stop();
  bool expected=false;
  if(fault_claimed_.compare_exchange_strong(expected,true,std::memory_order_acq_rel)){
   failure_=error;failure_input_=metadata;fault_ready_.store(true,std::memory_order_release);
  }
 }
};
int main(){ResidentPublicationGate g;auto error=std::make_exception_ptr(std::bad_alloc());FrameMetadata a,b;a.serial=b.serial="source-serial-that-exceeds-small-string-capacity";a.frame_number=100;b.frame_number=200;
 {Harness h(g);fail_heap=true;h.record_failure(error,a);fail_heap=false;assert(h.fault_ready_.load()&&h.failure_input_.frame_number==100);assert(!g.publish(7,h.ownership_.token(),[]{assert(false);}));}
 for(unsigned i=0;i<200;++i){Harness h(g);std::atomic<bool> start{false};std::thread t1([&]{while(!start.load()){}h.record_failure(error,a);});std::thread t2([&]{while(!start.load()){}h.record_failure(error,b);});start=true;while(!h.fault_ready_.load(std::memory_order_acquire)){}assert(h.failure_input_.frame_number==100||h.failure_input_.frame_number==200);assert(h.failure_input_.serial==a.serial);assert(h.stopping_&&h.scheduler_.stopped());assert(!g.publish(7,h.ownership_.token(),[]{assert(false);}));t1.join();t2.join();}
}
