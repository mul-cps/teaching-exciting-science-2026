#include "so101_station_vulkan/raw_camera_admission.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
using Clock=std::chrono::steady_clock;
std::atomic<uint64_t> allocations{0};
void require(bool x){if(!x)throw std::runtime_error("actual publication gate resource contract");}
double ns(Clock::time_point t){return std::chrono::duration<double,std::nano>(Clock::now()-t).count();}
constexpr unsigned N=100000;
void* operator new(std::size_t n){if(void*p=std::malloc(n?n:1)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,std::size_t)noexcept{std::free(p);}void operator delete[](void*p,std::size_t)noexcept{std::free(p);}
int main(){try{
 static_assert(std::atomic<uint64_t>::is_always_lock_free,"target requires lock-free token atomic");
 ResidentPublicationGate gate;int owner=0;auto token=gate.reserve(&owner);gate.admit(&owner,token,17);
 std::mutex mutex;std::condition_variable cv;bool entered=false,release=false;std::atomic<unsigned> order{0};
 std::thread publisher([&]{require(gate.publish(17,token,[&]{std::unique_lock<std::mutex> lock(mutex);entered=true;cv.notify_one();cv.wait(lock,[&]{return release;});require(order.fetch_add(1)==0);}));});
 {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return entered;});}
 const auto before=allocations.load();auto t=Clock::now();require(gate.revoke(token));double blocked_success=ns(t);
 t=Clock::now();for(unsigned i=0;i<N;i++)require(!gate.revoke(token));double blocked_failed=ns(t)/N;require(allocations.load()==before);
 // Ordered UNKNOWN is deliberately deferred behind the original writer; it is
 // not capture work. This is a blocked host callback, never real DDS/SDK/GPU.
 std::thread finalizer([&]{gate.finalize_unknown(&owner,token,[&]{require(order.fetch_add(1)==1);});});
 {std::lock_guard<std::mutex> lock(mutex);release=true;}cv.notify_one();publisher.join();finalizer.join();
 require(order==2);require(!gate.publish(17,token,[]{throw std::runtime_error("revoked callback entered");}));gate.release(&owner,token);
 // Sample successful CAS separately from cold session reserve/admit locks.
 double success_raw=0,clock_pair=0;auto mark=allocations.load();
 for(unsigned i=0;i<N;i++){
  token=gate.reserve(&owner);gate.admit(&owner,token,17);
  auto s=Clock::now();require(gate.revoke(token));success_raw+=ns(s);
  gate.release(&owner,token);s=Clock::now();clock_pair+=ns(s);
 }
 require(allocations.load()==mark);
 token=gate.reserve(&owner);gate.admit(&owner,token,17);require(!gate.revoke(token-1));require(gate.publish(17,token,[]{}));
 t=Clock::now();for(unsigned i=0;i<N;i++)require(!gate.revoke(0));double zero=ns(t)/N;gate.release(&owner,token);
 std::cout<<"{\"calls_per_batch\":"<<N<<",\"successful_revoke_raw_ns\":"<<success_raw/N<<",\"clock_pair_baseline_ns\":"<<clock_pair/N<<",\"blocked_publisher_first_revoke_ns\":"<<blocked_success<<",\"blocked_publisher_repeated_revoke_ns\":"<<blocked_failed<<",\"zero_token_ns\":"<<zero<<",\"hot_heap_allocations\":0,\"blocked_callback_entered_before_revoke\":true,\"ordered_unknown_after_entered_callback\":true,\"new_callback_after_revoke_denied\":true,\"stale_token_does_not_revoke_new_session\":true,\"SDK_GPU_DDS_calls\":0}\n";
 return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
