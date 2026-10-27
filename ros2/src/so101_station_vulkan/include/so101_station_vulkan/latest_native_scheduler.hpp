#pragma once
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
namespace so101_station_vulkan {
struct NativeSourceDecision {bool unique;uint64_t color_gap=0,depth_gap=0;};
// Capture-thread-only observed watermarks. Rejected partial/duplicate cohorts
// still advance observed watermarks; selected/accepted IDs are separate state.
class NativeSourceAdmission {
 uint64_t color_=0,depth_=0,accepted_color_=0,accepted_depth_=0,generation_=0;
 double color_time_=0,depth_time_=0;int color_domain_=0,depth_domain_=0;std::string serial_;
public:
 template<class Metadata>void seed(const Metadata& m){
  color_=accepted_color_=m.frame_number;depth_=accepted_depth_=m.depth_frame_number;
  color_time_=m.hardware_timestamp_ms;depth_time_=m.depth_hardware_timestamp_ms;
  color_domain_=m.timestamp_domain;depth_domain_=m.depth_timestamp_domain;
  generation_=m.calibration_generation;serial_=m.serial;
 }
 template<class Metadata>NativeSourceDecision observe(const Metadata& m){
  if(!m.frame_number||!m.depth_frame_number||!std::isfinite(m.hardware_timestamp_ms)||!std::isfinite(m.depth_hardware_timestamp_ms)||m.calibration_generation!=generation_||m.serial!=serial_||m.timestamp_domain!=color_domain_||m.depth_timestamp_domain!=depth_domain_||m.frame_number<color_||m.depth_frame_number<depth_||m.hardware_timestamp_ms<color_time_||m.depth_hardware_timestamp_ms<depth_time_)throw std::runtime_error("source identity/timestamp regressed or source session changed; restart required");
  NativeSourceDecision d{m.frame_number>accepted_color_&&m.depth_frame_number>accepted_depth_,m.frame_number>color_?m.frame_number-color_-1:0,m.depth_frame_number>depth_?m.depth_frame_number-depth_-1:0};
  color_=m.frame_number;depth_=m.depth_frame_number;color_time_=m.hardware_timestamp_ms;depth_time_=m.depth_hardware_timestamp_ms;
  if(d.unique){accepted_color_=m.frame_number;accepted_depth_=m.depth_frame_number;}
  return d;
 }
};
// One pending native handle, one worker-owned input through FINAL publication.
// Native is moved/swapped only. Caller destroys displaced/selected handles after
// these methods return; no native handle release or heavy work under this lock.
template<class Native,class Metadata>class LatestNativeScheduler {
#define NATIVE_FIELDS(X) X(received) X(unique) X(duplicate) X(invalid) X(replaced) X(contention) X(no_slot) X(stopped_skip) X(selected) X(submitted) X(computed) X(completed) X(published) X(active) X(pending) X(color_gap) X(depth_gap) X(first_unique_ns) X(last_unique_ns) X(intake_gap_max_ns) X(intake_gap_deadlines) X(first_output_ns) X(last_output_ns) X(capture_calls) X(capture_gap_max_ns) X(capture_duration_max_ns) X(capture_gap_deadlines) X(capture_duration_deadlines) X(age_count) X(age_sum_ns) X(age_max_ns) X(age_last_ns) X(age_unknown)
public:
 struct Snapshot {
#define PLAIN(name) uint64_t name=0;
  NATIVE_FIELDS(PLAIN)
#undef PLAIN
  uint64_t skipped()const{return replaced+contention+no_slot+stopped_skip;}
  bool conserved()const{return received==unique+duplicate+invalid&&unique==completed+skipped()+active+pending;}
 };
private:
 struct Counters {
#define ATOMIC(name) std::atomic<uint64_t> name{0};
  NATIVE_FIELDS(ATOMIC)
#undef ATOMIC
 } counts_;
 Native pending_;Metadata pending_metadata_;std::mutex mutex_;
 std::atomic<bool> stopped_{false};std::atomic<uint64_t> writers_{0},version_{0};
 struct Write {
  LatestNativeScheduler& q;
  explicit Write(LatestNativeScheduler& owner):q(owner){q.writers_.fetch_add(1,std::memory_order_acq_rel);}
  ~Write(){q.version_.fetch_add(1,std::memory_order_release);q.writers_.fetch_sub(1,std::memory_order_release);}
 };
 static void maximum(std::atomic<uint64_t>& field,uint64_t value){if(value>field.load(std::memory_order_relaxed))field.store(value,std::memory_order_release);}
public:
 void reserve_serial(size_t n){pending_metadata_.serial.reserve(n);} // Startup only.
 bool stopped()const noexcept{return stopped_.load(std::memory_order_acquire);}
 void request_stop()noexcept{stopped_.store(true,std::memory_order_release);}
 // Capture never waits for the worker-held mutex. Contention is an explicit
 // input discard, not a hidden queue. Structural validation precedes this call.
 bool accept(Native& incoming,const Metadata& metadata,NativeSourceDecision d,uint64_t now){
  Write write(*this);++counts_.received;counts_.color_gap+=d.color_gap;counts_.depth_gap+=d.depth_gap;
  if(!d.unique){++counts_.duplicate;return false;}
  if(counts_.unique.fetch_add(1)==0)counts_.first_unique_ns=now;
  else {const auto gap=now-counts_.last_unique_ns.load(std::memory_order_relaxed);maximum(counts_.intake_gap_max_ns,gap);if(gap>33333333)++counts_.intake_gap_deadlines;}
  counts_.last_unique_ns=now;
  if(stopped()){++counts_.stopped_skip;return false;}
  std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
  if(!lock.owns_lock()){++counts_.contention;return false;}
  if(stopped()){++counts_.stopped_skip;return false;}
  if(pending_)++counts_.replaced;
  using std::swap;swap(pending_,incoming);pending_metadata_=metadata;counts_.pending=1;return true;
 }
 void invalid_received(){Write write(*this);++counts_.received;++counts_.invalid;}
 // Worker only; output's Native must be empty, metadata capacity reserved cold.
 bool take(Native& output,Metadata& metadata){
  std::lock_guard<std::mutex> lock(mutex_);
  if(stopped()||!pending_)return false;
  Write write(*this);
  using std::swap;swap(pending_,output);metadata=pending_metadata_;
  counts_.pending=0;counts_.active=1;++counts_.selected;return true;
 }
 void submitted(){Write write(*this);++counts_.submitted;}
 void computed(){Write write(*this);++counts_.computed;}
 void skip_no_slot(){Write write(*this);counts_.active=0;++counts_.no_slot;}
 void cancel_active(){Write write(*this);if(counts_.active.exchange(0))++counts_.stopped_skip;}
 void complete(uint64_t now,uint64_t age_ns){
  Write write(*this);counts_.active=0;++counts_.completed;
  if(counts_.published.fetch_add(1)==0)counts_.first_output_ns=now;
  counts_.last_output_ns=now;
  if(age_ns==std::numeric_limits<uint64_t>::max())++counts_.age_unknown;
  else {++counts_.age_count;counts_.age_sum_ns+=age_ns;counts_.age_last_ns=age_ns;maximum(counts_.age_max_ns,age_ns);}
 }
 // Shutdown only, AFTER capture join. Destruction of returned native is outside
 // the mutex. Worker stop prevents taking another pending cohort.
 void clear_pending(Native& released){
  std::lock_guard<std::mutex> lock(mutex_);Write write(*this);
  if(pending_){using std::swap;swap(pending_,released);counts_.pending=0;++counts_.stopped_skip;}
 }
 // Capture thread is the only writer of capture timing maxima.
 void capture_service(uint64_t gap_ns,uint64_t duration_ns){
  Write write(*this);++counts_.capture_calls;maximum(counts_.capture_gap_max_ns,gap_ns);maximum(counts_.capture_duration_max_ns,duration_ns);
  if(gap_ns>33333333)++counts_.capture_gap_deadlines;
  if(duration_ns>33333333)++counts_.capture_duration_deadlines;
 }
 // Diagnostics can keep their previous sample when a writer is preempted.
 // Never use either snapshot method on capture/admission.
 bool try_snapshot(Snapshot& result,unsigned attempts=8)const{
  Snapshot s;
  for(unsigned attempt=0;attempt<attempts;++attempt){
   const auto before=version_.load(std::memory_order_acquire);
   if(writers_.load(std::memory_order_acquire))continue;
#define LOAD(name) s.name=counts_.name.load(std::memory_order_relaxed);
   NATIVE_FIELDS(LOAD)
#undef LOAD
   std::atomic_thread_fence(std::memory_order_acquire);
   if(!writers_.load(std::memory_order_acquire)&&version_.load(std::memory_order_acquire)==before){result=s;return true;}
  }
  return false;
 }
 Snapshot snapshot()const{Snapshot s;while(!try_snapshot(s)){}return s;} // Host observers/tests only.
#undef NATIVE_FIELDS
};
}
