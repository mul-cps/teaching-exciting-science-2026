
#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <thread>
#include <cassert>
#include <atomic>
#include <string>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace rs2 {struct frameset {int id=0;explicit operator bool()const{return id!=0;}};}
struct Metadata {std::string serial;};
struct TestThread {
 static inline int created=0,fail_at=0;
 std::thread thread;
 TestThread()=default;
 template<class F>explicit TestThread(F f){if(++created==fail_at)throw std::runtime_error("injected thread construction failure");thread=std::thread(f);}
 TestThread(TestThread&&)=default;TestThread& operator=(TestThread&&)=default;
 bool joinable(){return thread.joinable();}void join(){thread.join();}
};
struct Owner {std::atomic<bool> revoked{false};void revoke(){revoked=true;}};
struct Harness {
 Owner ownership_;std::atomic<bool> stopping_{false},capture_live{false},worker_live{false};
 std::atomic<unsigned> capture_calls{0},worker_calls{0};unsigned stop_calls=0;
 TestThread capture_thread_,worker_thread_;LatestNativeScheduler<rs2::frameset,Metadata> scheduler_;
 bool running_=true;
 void capture_loop(){capture_live=true;while(!stopping_.load()){++capture_calls;std::this_thread::yield();}capture_live=false;}
 void worker_loop(){worker_live=true;while(!stopping_.load()){++worker_calls;std::this_thread::yield();}worker_live=false;}
 void stop_owned_camera(){assert(!capture_live.load()&&!capture_thread_.joinable());if(running_){++stop_calls;running_=false;}}
 void shutdown_threads()noexcept {
  ownership_.revoke();stopping_.store(true,std::memory_order_release);scheduler_.request_stop();
  if(capture_thread_.joinable())capture_thread_.join();
  {rs2::frameset released;scheduler_.clear_pending(released);}
  stop_owned_camera();
  if(worker_thread_.joinable())worker_thread_.join();
 }
 void start_threads(){
  try{worker_thread_=TestThread([this]{worker_loop();});capture_thread_=TestThread([this]{capture_loop();});}
  catch(...){shutdown_threads();throw;}
 }
 ~Harness(){shutdown_threads();assert(!worker_live.load()&&!worker_thread_.joinable());}
};
int main(){
 for(int failure=0;failure<=2;++failure){
  TestThread::created=0;TestThread::fail_at=failure;Harness h;bool threw=false;
  try{h.start_threads();}catch(const std::runtime_error&){threw=true;}
  assert(threw==(failure!=0));
  if(!failure){while(!h.capture_calls.load()||!h.worker_calls.load())std::this_thread::yield();}
  h.shutdown_threads();assert(h.ownership_.revoked&&h.scheduler_.stopped());
  assert(!h.capture_live&&!h.worker_live&&!h.capture_thread_.joinable()&&!h.worker_thread_.joinable());
  assert(h.stop_calls==1);h.shutdown_threads();assert(h.stop_calls==1);
 }
}
