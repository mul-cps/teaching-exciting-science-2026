
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
static bool deny_heap=false,throw_log=false;static int log_calls=0;
void* operator new(size_t n){if(deny_heap)throw std::bad_alloc();if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{std::free(p);}void operator delete(void* p,size_t)noexcept{std::free(p);}
void logger(){++log_calls;if(throw_log)throw 9;}
#define RCLCPP_ERROR(...) logger()
namespace std_msgs {namespace msg {struct String{std::string data;};}}
struct Owner{bool live=true;void revoke(){live=false;}unsigned token(){return 7;}};
struct Scheduler{bool stopped=false;void request_stop(){stopped=true;}};
struct Timer{bool fail=false;int calls=0;void cancel(){++calls;if(fail)throw 8;}};
struct Publisher{int fail=0,calls=0;void publish(const std_msgs::msg::String&){++calls;if(fail==1)throw std::bad_alloc();if(fail==2)throw 7;}};
struct Gate{int calls=0;template<class F>void finalize_unknown(const void*,unsigned,F f){++calls;f();}};
struct Metadata{unsigned long long frame_number=3,depth_frame_number=4,timestamp_ns=5;};
struct Harness{
 Owner ownership_;Scheduler scheduler_;std::atomic<bool> stopping_{false},failure_finalized_{false},fault_ready_{false};
 Timer timer_storage;Timer* timer_=&timer_storage;Publisher unknown,status;Publisher* failures_=&unknown;Publisher* status_=&status;Gate gate;
 Metadata failure_input_;std_msgs::msg::String failure_diagnostic_,failure_status_;int shutdowns=0,reports=0;bool capture_stopped=false,worker_stopped=false;
 std::exception_ptr failure_;unsigned long long next_status_ns_=0;
 Harness(){failure_diagnostic_.data.reserve(2048);failure_status_.data.reserve(2048);}
 Gate& publication_gate(){return gate;}
 void shutdown_threads()noexcept{assert(!ownership_.live&&stopping_&&scheduler_.stopped);++shutdowns;capture_stopped=worker_stopped=true;}
 unsigned long long steady_ns(){return 1;}void report_status(){++reports;throw 2;}
 void latch_failure(const char* reason)noexcept{
  ownership_.revoke();stopping_.store(true,std::memory_order_release);scheduler_.request_stop();
  if(failure_finalized_.exchange(true,std::memory_order_acq_rel))return;
  bool cancel_failed=false,unknown_failed=false;
  try{if(timer_)timer_->cancel();}catch(...){cancel_failed=true;}
  shutdown_threads();
  const auto& m=failure_input_;char text[2048];
  try{
   const int n=std::snprintf(text,sizeof(text),"{\"status\":\"unknown\",\"geometry_admission\":\"unqualified\",\"execution_eligible\":false,\"source_frame_number\":%llu,\"source_depth_frame_number\":%llu,\"source_timestamp_ns\":%llu,\"error\":\"resident_capture_failed_restart_required\"}",static_cast<unsigned long long>(m.frame_number),static_cast<unsigned long long>(m.depth_frame_number),static_cast<unsigned long long>(m.timestamp_ns));
   if(n<0||size_t(n)>=sizeof(text))unknown_failed=true;
   else {failure_diagnostic_.data.assign(text,size_t(n));publication_gate().finalize_unknown(this,ownership_.token(),[&]{failures_->publish(static_cast<const std_msgs::msg::String&>(failure_diagnostic_));});}
  }catch(...){unknown_failed=true;}
  try{
   const int n=std::snprintf(text,sizeof(text),"fatal; geometry_admission=unqualified; execution_eligible=false; restart_required=true; unknown_publish_failed=%s; timer_cancel_failed=%s; reason=%.512s",unknown_failed?"true":"false",cancel_failed?"true":"false",reason?reason:"non-standard resident exception");
   if(n>=0&&size_t(n)<sizeof(text)){failure_status_.data.assign(text,size_t(n));status_->publish(static_cast<const std_msgs::msg::String&>(failure_status_));}
  }catch(...){}
  try{RCLCPP_ERROR(get_logger(),"resident source closed; UNKNOWN delivery failed=%s; %.512s",unknown_failed?"true":"false",reason?reason:"non-standard resident exception");}catch(...){}
 }
 void tick(){
  if(fault_ready_.load(std::memory_order_acquire)){
   try{std::rethrow_exception(failure_);}catch(const std::exception& error){latch_failure(error.what());}catch(...){latch_failure("non-standard resident exception");}
   return;
  }
  const auto current=steady_ns();if(current>=next_status_ns_){report_status();next_status_ns_=current+1000000000;}
 }
 void timer_callback()noexcept{try{tick();}catch(const std::exception& error){latch_failure(error.what());}catch(...){latch_failure("non-standard resident exception");}}
};
int main(){
 for(int failure=0;failure<6;++failure){Harness h;h.timer_storage.fail=failure==1;h.unknown.fail=failure==2?1:failure==3?2:0;h.status.fail=failure==4?1:0;throw_log=failure==5;log_calls=0;
  deny_heap=true;h.latch_failure("bounded failure");h.latch_failure("must not retry");deny_heap=false;
  assert(!h.ownership_.live&&h.stopping_&&h.scheduler_.stopped&&h.capture_stopped&&h.worker_stopped);
  assert(h.shutdowns==1&&h.timer_storage.calls==1&&h.unknown.calls==1&&h.status.calls==1&&h.gate.calls==1&&log_calls==1);
  if(failure==2||failure==3)assert(h.failure_status_.data.find("unknown_publish_failed=true")!=std::string::npos);
 }
 throw_log=false;
 {Harness h;try{throw 42;}catch(...){h.failure_=std::current_exception();}h.fault_ready_=true;h.timer_callback();h.timer_callback();assert(h.shutdowns==1&&!h.ownership_.live&&h.reports==0);}
 {Harness h;h.timer_callback();h.timer_callback();assert(h.shutdowns==1&&!h.ownership_.live);}
 {Harness h;std::string huge(10000,'x');deny_heap=true;h.latch_failure(huge.c_str());deny_heap=false;assert(h.failure_status_.data.size()<2048);}
}
