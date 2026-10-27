#include "so101_station_vulkan/frame_adapter.hpp"
#include "so101_station_vulkan/raw_capture_bridge.hpp"
#include "so101_station_vulkan/raw_camera_admission.hpp"
#include "so101_station_vulkan/camera_startup_gate.hpp"
#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <exception>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace so101_station_vulkan {
using namespace std::chrono_literals;
namespace {
ResidentPublicationGate& publication_gate(){static ResidentPublicationGate gate;return gate;}
// Access only under publication_gate: every current-owner measurement uses one
// DDS writer, so success and UNKNOWN share both host and per-writer ordering.
std::weak_ptr<rclcpp::Publisher<std_msgs::msg::String>>& measurement_writer(){
 static std::weak_ptr<rclcpp::Publisher<std_msgs::msg::String>> writer;return writer;
}
CameraProfileIdentity profile_identity(const rs2::video_stream_profile& p){
 return {p.unique_id(),p.stream_index(),p.width(),p.height(),p.fps(),int(p.format()),int(p.stream_type())};
}
CameraSourceCalibration source_calibration(const rs2::video_stream_profile& color,const rs2::video_stream_profile& depth,float units){
 CameraSourceCalibration s;s.color=profile_identity(color);s.depth=profile_identity(depth);
 s.alignment.color=color.get_intrinsics();s.alignment.depth=depth.get_intrinsics();
 s.alignment.depth_to_color=depth.get_extrinsics_to(color);s.alignment.depth_units=units;
 s.alignment.convention=AlignmentConvention::Scalar;return s;
}
uint64_t session_nonce(){
 static std::atomic<uint64_t> sessions{0};
 return uint64_t(std::chrono::steady_clock::now().time_since_epoch().count())^(++sessions);
}
void copy_rows(uint8_t* destination,size_t capacity,const rs2::video_frame& source,unsigned row_bytes){
 const auto* data=static_cast<const uint8_t*>(source.get_data());
 const auto stride=source.get_stride_in_bytes(),height=source.get_height(),width=source.get_width(),bpp=source.get_bytes_per_pixel();
 const int64_t available=source.get_data_size();
 if(!destination||!data||!row_bytes||width<=0||height<=0||bpp<=0||size_t(width)*unsigned(bpp)!=row_bytes||stride<0||size_t(stride)<row_bytes||capacity%row_bytes||size_t(height)!=capacity/row_bytes||available<0||size_t(available)<row_bytes||(height>1&&size_t(stride)>(size_t(available)-row_bytes)/size_t(height-1)))throw std::runtime_error("invalid camera plane extent/stride/byte bounds");
 if(stride==int(row_bytes)){std::memcpy(destination,data,size_t(row_bytes)*unsigned(height));return;}
 for(unsigned y=0;y<unsigned(height);y++)std::memcpy(destination+size_t(y)*row_bytes,data+size_t(y)*unsigned(stride),row_bytes);
}
}
// Experimental raw acquisition only. Every result remains geometry-unqualified.
class CameraComponent : public rclcpp::Node {
 ResidentCameraOwner ownership_;
 rs2::context cameras_;
 rs2::pipeline camera_;
 rs2::align vendor_alignment_{RS2_STREAM_COLOR};
 std::unique_ptr<RawCaptureBridge> bridge_;
 CameraSourceCalibration source_;
 CameraCaptureProfile selected_profile_{};std::string profile_name_,profile_requested_;
 uint64_t publication_session_token_=0,normal_start_ns_=0,next_status_ns_=0;
 LatestNativeScheduler<rs2::frameset,FrameMetadata> scheduler_;
 LatestNativeScheduler<rs2::frameset,FrameMetadata>::Snapshot status_counts_;
 uint64_t status_counts_ns_=0;
 NativeSourceAdmission source_admission_;
 std::thread capture_thread_,worker_thread_;
 std::atomic<bool> stopping_{false},fault_claimed_{false},fault_ready_{false},failure_finalized_{false};
 std::exception_ptr failure_;
 rclcpp::Publisher<FrameLease>::SharedPtr frames_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,failures_;
 rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::string serial_,fallback_reason_,shaders_,alignment_shaders_;
 bool running_=false,domains_known_=false;
 int color_domain_=0,depth_domain_=0;
 std_msgs::msg::String status_message_,failure_diagnostic_,failure_status_;
 FrameMetadata last_input_,failure_input_,worker_input_;
 uint64_t previous_color_=0,previous_depth_=0;
 double previous_color_time_=0,previous_depth_time_=0;
 void stop_owned_camera() noexcept {
  if(!running_)return;
  running_=false;try{camera_.stop();}catch(...){}
 }
 void publish_status(const std::string& message){std_msgs::msg::String s;s.data=message;status_->publish(s);}
 size_t color_pixels()const{return size_t(source_.alignment.color.width)*unsigned(source_.alignment.color.height);}
 size_t depth_pixels()const{return size_t(source_.alignment.depth.width)*unsigned(source_.alignment.depth.height);}
 void validate_selected_profiles(const CameraSourceCalibration& s)const {
  validate_camera_profiles(s);
  if(s.color.width!=int(selected_profile_.color_width)||s.color.height!=int(selected_profile_.color_height)||s.depth.width!=int(selected_profile_.depth_width)||s.depth.height!=int(selected_profile_.depth_height))throw std::runtime_error("resolved/active camera extents differ from selected profile");
 }
 Calibration core_calibration()const {
  const auto& k=source_.alignment;return {k.color.fx,k.color.fy,k.color.ppx,k.color.ppy,k.depth_units,k.generation};
 }
 CameraSourceObservation inspect_original(const rs2::frameset& native)const {
  auto color=native.get_color_frame();auto depth=native.get_depth_frame();
  if(!color||!depth)throw std::runtime_error("source cohort lacks original RGB/depth");
  auto current=source_calibration(color.get_profile().as<rs2::video_stream_profile>(),depth.get_profile().as<rs2::video_stream_profile>(),depth.get_units());
  if(!same_camera_source(source_,current))throw std::runtime_error("source profile/calibration/units changed; restart required");
  const CameraSourceObservation observed{color.get_frame_number(),depth.get_frame_number(),color.get_timestamp(),depth.get_timestamp(),int(color.get_frame_timestamp_domain()),int(depth.get_frame_timestamp_domain())};
  if(!std::isfinite(observed.color_time_ms)||!std::isfinite(observed.depth_time_ms))throw std::runtime_error("invalid original frame extent/stride/provenance");
  for(const auto& frame:{rs2::video_frame(color),rs2::video_frame(depth)}){
   const bool is_color=frame.get_profile().stream_type()==RS2_STREAM_COLOR;
   const auto& expected=is_color?source_.color:source_.depth;const unsigned bpp=is_color?3:2,row=unsigned(expected.width)*bpp;
   if(frame.get_width()!=expected.width||frame.get_height()!=expected.height||frame.get_bytes_per_pixel()!=int(bpp)||frame.get_stride_in_bytes()<int(row)||frame.get_data_size()<0||size_t(frame.get_data_size())<size_t(frame.get_stride_in_bytes())*unsigned(expected.height-1)+row)throw std::runtime_error("invalid original frame extent/stride/provenance");
  }
  return observed;
 }
 void verify_original(const rs2::frameset& native){
  const auto observed=inspect_original(native);
  if(!observed.color_frame||!observed.depth_frame)throw std::runtime_error("invalid original frame extent/stride/provenance");
  if(domains_known_&&(observed.color_domain!=color_domain_||observed.depth_domain!=depth_domain_))throw std::runtime_error("source timestamp domain changed; restart required");
  domains_known_=true;color_domain_=observed.color_domain;depth_domain_=observed.depth_domain;
 }
 void remember_source(const FrameMetadata& metadata){
  previous_color_=metadata.frame_number;previous_depth_=metadata.depth_frame_number;
  previous_color_time_=metadata.hardware_timestamp_ms;previous_depth_time_=metadata.depth_hardware_timestamp_ms;
 }
 static uint64_t steady_ns(){
  return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
 }
 // Only pre-admission source counter/domain anomalies may be quarantined.
 // Structural/profile failures still throw, and no measurements are eligible.
 void wait_startup_stable(){
  try {
  publish_status("starting; source_admitted=false; execution_eligible=false; geometry_admission=unqualified; waiting_for_stable_source=true");
  CameraStartupGate gate(steady_ns());
  while(true){
   gate.check_deadline(steady_ns());
   rs2::frameset native;
   if(!camera_.poll_for_frames(&native)){std::this_thread::sleep_for(2ms);continue;}
   const auto observed=inspect_original(native);
   if(!gate.observe(observed,steady_ns()))continue;
   domains_known_=true;color_domain_=observed.color_domain;depth_domain_=observed.depth_domain;
   previous_color_=observed.color_frame;previous_depth_=observed.depth_frame;
   previous_color_time_=observed.color_time_ms;previous_depth_time_=observed.depth_time_ms;
   return;
  }
  }catch(...){publication_gate().invalidate(this,ownership_.token());stop_owned_camera();throw;}
 }
 void original_metadata(FrameMetadata& m,const rs2::frameset& native,uint64_t receipt){
  auto color=native.get_color_frame();auto depth=native.get_depth_frame();
  m.width=source_.alignment.color.width;m.height=source_.alignment.color.height;m.frame_number=color.get_frame_number();m.depth_frame_number=depth.get_frame_number();
  m.timestamp_ns=receipt;m.hardware_timestamp_ms=color.get_timestamp();m.depth_hardware_timestamp_ms=depth.get_timestamp();
  m.timestamp_domain=int(color.get_frame_timestamp_domain());m.depth_timestamp_domain=int(depth.get_frame_timestamp_domain());
  m.calibration_generation=source_.alignment.generation;m.publication_session_token=publication_session_token_;m.serial=serial_;
 }
 // Raw normal path: exactly one native RGB upload + one native Z16 upload.
 // Alignment and compact core are submitted consecutively by the bridge.
 void copy_raw_planes(RawFrame& frame,const rs2::frameset& native,const FrameMetadata& metadata){
  auto& slot=frame.core.slot();slot.metadata=metadata;slot.calibration=core_calibration();
  frame.depth.metadata()=metadata;
  copy_rows(static_cast<uint8_t*>(slot.upload.mapped),color_pixels()*3,native.get_color_frame(),unsigned(source_.alignment.color.width)*3);
  copy_rows(reinterpret_cast<uint8_t*>(frame.depth.upload()),depth_pixels()*2,native.get_depth_frame(),unsigned(source_.alignment.depth.width)*2);
 }
 // Vendor CPU alignment is used only by the explicitly selected CPU path.
 void copy_vendor_planes(RawFrame& frame,const rs2::frameset& native,const FrameMetadata& metadata){
  auto aligned=vendor_alignment_.process(native).as<rs2::frameset>();
  auto& slot=frame.core.slot();slot.metadata=metadata;slot.calibration=core_calibration();
  auto* bytes=static_cast<uint8_t*>(slot.upload.mapped);
  copy_rows(bytes,color_pixels()*3,aligned.get_color_frame(),unsigned(source_.alignment.color.width)*3);
  copy_rows(bytes+color_pixels()*3,color_pixels()*2,aligned.get_depth_frame(),unsigned(source_.alignment.color.width)*2);
 }
 std::unique_ptr<RawCaptureBridge> make_bridge(bool raw,bool diagnostics=false){
  auto core=std::make_unique<Pipeline>(source_.alignment.color.width,source_.alignment.color.height,raw?"vulkan":"cpu",shaders_);
  std::unique_ptr<Alignment> depth;
  if(raw)depth=std::make_unique<Alignment>(core->context(),source_.alignment,alignment_shaders_,diagnostics);
  auto bridge=std::make_unique<RawCaptureBridge>(std::move(core),std::move(depth));
  // Reserve every reused provenance string before normal callbacks begin.
  std::array<RawFrame,kSlots> reserved;
  for(auto& f:reserved){f=bridge->acquire();if(!f)throw std::runtime_error("startup frame pool reservation failed");f.core.slot().metadata.serial.reserve(serial_.size());if(f.depth)f.depth.metadata().serial.reserve(serial_.size());}
  return bridge;
 }
 rs2::frameset wait_original(){
  rs2::frameset original;
  if(!camera_.try_wait_for_frames(&original,1000))throw std::runtime_error("cold camera frame deadline");
  verify_original(original);return original;
 }
 // Cold admission only: SDK alignment and dense diagnostic reads are forbidden
 // after this function returns. Original frames are retained through comparison.
 void cold_live_selfcheck(std::unique_ptr<RawCaptureBridge> cold){
  try {
  Pipeline independent(source_.alignment.color.width,source_.alignment.color.height,"cpu",shaders_);
  std::vector<uint32_t> reference_depth(color_pixels());
  std::vector<float> reference_xyz(color_pixels()*4);
  ColdCameraSelfcheck gate(source_.alignment.generation,serial_,source_.alignment.color.width,source_.alignment.color.height,source_.alignment.depth.width,source_.alignment.depth.height);
  if(!cold->accept_source(previous_color_,previous_depth_))throw std::runtime_error("cold source startup baseline rejected");
  for(unsigned pass=0;pass<3;pass++){
   const auto original=wait_original();const auto receipt=uint64_t(now().nanoseconds());
   FrameMetadata metadata;original_metadata(metadata,original,receipt);
   if(metadata.frame_number<previous_color_||metadata.depth_frame_number<previous_depth_||metadata.hardware_timestamp_ms<previous_color_time_||metadata.depth_hardware_timestamp_ms<previous_depth_time_)throw std::runtime_error("cold source identity/timestamp regressed; restart required");
   if(!cold->accept_source(metadata.frame_number,metadata.depth_frame_number))throw std::runtime_error("cold source cohort regressed/duplicated");
   auto frame=cold->acquire();if(!frame)throw std::runtime_error("cold bridge pool unavailable");
   copy_raw_planes(frame,original,metadata);
   const uint16_t* raw_reference=frame.depth.upload(); // Capture immutable upload view before submission.
   cold->offer(frame);cold->dispatch_pending();
   const auto deadline=std::chrono::steady_clock::now()+10s;
   // Only cold admission may wait for its own consumer fence. This is a
   // polling budget, not a startup/recovery deadline: safe Core destruction
   // still drains in-flight fences on a hung driver. Qualification must use
   // an external process timeout; in-flight resources must never be freed.
   while(!cold->pipeline().ready(frame.core)){
    if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("cold GPU selfcheck consumer deadline");
    std::this_thread::yield();
   }
   RawFrame completed;if(!cold->collect(completed))throw std::runtime_error("cold GPU selfcheck collection failed");
   auto sdk=vendor_alignment_.process(original).as<rs2::frameset>().get_depth_frame();
   auto expected=independent.acquire();if(!expected)throw std::runtime_error("cold CPU reference pool unavailable");
   auto& cpu=expected.slot();cpu.metadata=metadata;cpu.calibration=core_calibration();
   auto* bytes=static_cast<uint8_t*>(cpu.upload.mapped);
   copy_rows(bytes,color_pixels()*3,original.get_color_frame(),unsigned(source_.alignment.color.width)*3);copy_rows(bytes+color_pixels()*3,color_pixels()*2,sdk,unsigned(source_.alignment.color.width)*2);
   alignment_reference(source_.alignment,raw_reference,reference_depth.data(),reference_xyz.data());
   // Independent CPU reference consumes the SDK-aligned original cohort.
   independent.submit(expected);independent.finish(expected);
   gate.verify(metadata,raw_reference,bytes+color_pixels()*3,completed.depth.diagnostic_depth(),reference_depth.data(),reference_xyz.data(),completed.depth.diagnostic_xyz(),cpu,completed.core.slot());
   remember_source(metadata);
  }
  gate.require_complete();
  }catch(...){publication_gate().invalidate(this,ownership_.token());stop_owned_camera();throw;}
 }
 // Capture only: inspect EVERY received cohort before the bounded handoff.
 // Displaced SDK handles stay in native and are released by the caller outside
 // the scheduler lock. No copies, alignment, backend work or DDS on this path.
 void capture_pending(rs2::frameset& native,uint64_t received_ros_ns){
  bool classified=false;
  try {
   last_input_.timestamp_ns=received_ros_ns;
   last_input_.frame_number=native.get_color_frame()?native.get_color_frame().get_frame_number():0;
   last_input_.depth_frame_number=native.get_depth_frame()?native.get_depth_frame().get_frame_number():0;
   if(native.get_color_frame()&&native.get_depth_frame())original_metadata(last_input_,native,received_ros_ns);
   verify_original(native);
   const auto decision=source_admission_.observe(last_input_);
   classified=true;scheduler_.accept(native,last_input_,decision,steady_ns());
  }catch(...){if(!classified)scheduler_.invalid_received();throw;}
 }
 // Both normal threads may report failure. The first writer owns this record;
 // release/acquire publication replaces a worker-held error mutex entirely.
 void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{
  ownership_.revoke();stopping_.store(true,std::memory_order_release);scheduler_.request_stop();
  bool expected=false;
  if(fault_claimed_.compare_exchange_strong(expected,true,std::memory_order_acq_rel)){
   failure_=error;failure_input_=metadata;fault_ready_.store(true,std::memory_order_release);
  }
 }
 void capture_loop()noexcept{
  uint64_t previous_service=0;
  try {
   while(!stopping_.load(std::memory_order_acquire)){
    const auto started=steady_ns();
    {
     rs2::frameset native;
     if(camera_.poll_for_frames(&native)){
      const auto received_ros_ns=uint64_t(now().nanoseconds());
      capture_pending(native,received_ros_ns);
     }
    } // SDK handle release, including displaced input, is outside handoff lock.
    scheduler_.capture_service(previous_service?started-previous_service:0,steady_ns()-started);previous_service=started;
    std::this_thread::sleep_until(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(started))+2ms);
   }
  }catch(...){record_failure(std::current_exception(),last_input_);}
 }
 void worker_loop()noexcept{
  rs2::frameset selected;bool processing=false;
  try {
   while(!stopping_.load(std::memory_order_acquire)){
    if(processing){
     RawFrame completed;
     if(!bridge_->collect(completed)){std::this_thread::sleep_for(2ms);continue;}
     scheduler_.computed();
     const auto& metadata=completed.core.slot().metadata;
     bool published=false;
     if(!stopping_.load(std::memory_order_acquire))published=publication_gate().publish(metadata.calibration_generation,metadata.publication_session_token,[&]{frames_->publish(completed.core);});
     if(published){
      const auto current=now().nanoseconds();
      const uint64_t age=current>=0&&uint64_t(current)>=metadata.timestamp_ns?uint64_t(current)-metadata.timestamp_ns:std::numeric_limits<uint64_t>::max();
      scheduler_.complete(steady_ns(),age);
     }else scheduler_.cancel_active();
     processing=false;continue;
    }
    if(!scheduler_.take(selected,worker_input_)){std::this_thread::sleep_for(2ms);continue;}
    auto frame=bridge_->acquire();
    if(!frame){selected={};scheduler_.skip_no_slot();continue;}
    if(bridge_->gpu_alignment())copy_raw_planes(frame,selected,worker_input_);
    else copy_vendor_planes(frame,selected,worker_input_);
    selected={}; // Release original SDK cohort before potentially slow CPU Core.
    if(stopping_.load(std::memory_order_acquire)){scheduler_.cancel_active();continue;}
    bridge_->offer(std::move(frame));scheduler_.submitted();processing=true;bridge_->dispatch_pending();
   }
  }catch(...){record_failure(std::current_exception(),worker_input_);}
  selected={};scheduler_.cancel_active();
 }
 // Main/control thread only. Never stop SDK while capture can still poll it;
 // never destroy bridge/fences until the compute worker has returned.
 void shutdown_threads()noexcept{
  ownership_.revoke();stopping_.store(true,std::memory_order_release);scheduler_.request_stop();
  if(capture_thread_.joinable())capture_thread_.join();
  {rs2::frameset released;scheduler_.clear_pending(released);}
  stop_owned_camera();
  if(worker_thread_.joinable())worker_thread_.join();
 }
 void start_threads(){
  try{worker_thread_=std::thread([this]{worker_loop();});capture_thread_=std::thread([this]{capture_loop();});}
  catch(...){shutdown_threads();throw;}
 }
 void tick(){
  if(fault_ready_.load(std::memory_order_acquire)){
   try{std::rethrow_exception(failure_);}catch(const std::exception& error){latch_failure(error.what());}catch(...){latch_failure("non-standard resident exception");}
   return;
  }
  const auto current=steady_ns();if(current>=next_status_ns_){report_status();next_status_ns_=current+1000000000;}
 }
 void report_status(){
  const bool snapshot_fresh=scheduler_.try_snapshot(status_counts_,8);
  const auto sampled_ns=steady_ns();if(snapshot_fresh)status_counts_ns_=sampled_ns;
  const auto& counts=status_counts_;const auto context=bridge_->pipeline().context();
  const auto& profile=selected_profile_;const size_t native_bytes=3*color_pixels()+2*depth_pixels();
  const auto u=[](uint64_t n){return static_cast<unsigned long long>(n);};
  const double elapsed=normal_start_ns_?double(steady_ns()-normal_start_ns_)*1e-9:0;
  const double acquisition=counts.unique>1&&counts.last_unique_ns>counts.first_unique_ns?double(counts.unique-1)*1e9/double(counts.last_unique_ns-counts.first_unique_ns):0;
  const double output=counts.published>1&&counts.last_output_ns>counts.first_output_ns?double(counts.published-1)*1e9/double(counts.last_output_ns-counts.first_output_ns):0;
  char text[4096];
  const int length=std::snprintf(text,sizeof(text),
   "experimental; backend=%s; alignment=%s; profile=%s; profile_requested=%s; skip_policy=latest_native; counter_snapshot_fresh=%s; counter_snapshot_age_ms=%.3f; color_width=%u; color_height=%u; depth_width=%u; depth_height=%u; fps=%u; vulkan_allocation_count=%llu; fallback_reason=%.1024s; geometry_admission=unqualified; execution_eligible=false; full_station_qualified=false; recording=disabled; received=%llu; received_unique=%llu; completed=%llu; skipped=%llu; dropped=%llu; replaced=%llu; contention_discard=%llu; stopped_discard=%llu; duplicate_or_regressed=%llu; invalid_received=%llu; selected=%llu; submitted=%llu; computed=%llu; published=%llu; active=%llu; pending=%llu; counter_conservation=%s; sdk_color_id_gaps=%llu; sdk_depth_id_gaps=%llu; acquisition_unique_hz=%.3f; acquisition_gap_max_ms=%.3f; acquisition_gap_deadline_misses=%llu; computed_unique_hz=%.3f; published_unique_hz=%.3f; capture_service_calls=%llu; capture_service_gap_max_ms=%.3f; capture_duration_max_ms=%.3f; capture_gap_deadline_misses=%llu; capture_duration_deadline_misses=%llu; received_age_last_ms=%.3f; received_age_mean_ms=%.3f; received_age_max_ms=%.3f; received_age_unknown=%llu; host_plane_copy_bytes=%zu; native_upload_bytes=%zu; aligned_d2d_bytes=%zu; alignment_fault_readback_bytes=%u; core_compact_readback_bytes=%zu",
   bridge_->pipeline().backend().c_str(),bridge_->gpu_alignment()?"raw_vulkan":"vendor_cpu",profile_name_.c_str(),profile_requested_.c_str(),snapshot_fresh?"true":"false",status_counts_ns_?double(sampled_ns-status_counts_ns_)*1e-6:0,profile.color_width,profile.color_height,profile.depth_width,profile.depth_height,profile.fps,u(context?context->allocation_count.load():0),fallback_reason_.c_str(),
   u(counts.received),u(counts.unique),u(counts.completed),u(counts.skipped()),u(counts.no_slot),u(counts.replaced),u(counts.contention),u(counts.stopped_skip),u(counts.duplicate),u(counts.invalid),u(counts.selected),u(counts.submitted),u(counts.computed),u(counts.published),u(counts.active),u(counts.pending),counts.conserved()?"true":"false",u(counts.color_gap),u(counts.depth_gap),acquisition,double(counts.intake_gap_max_ns)*1e-6,u(counts.intake_gap_deadlines),elapsed?double(counts.computed)/elapsed:0,output,u(counts.capture_calls),double(counts.capture_gap_max_ns)*1e-6,double(counts.capture_duration_max_ns)*1e-6,u(counts.capture_gap_deadlines),u(counts.capture_duration_deadlines),double(counts.age_last_ns)*1e-6,counts.age_count?double(counts.age_sum_ns)/double(counts.age_count)*1e-6:0,double(counts.age_max_ns)*1e-6,u(counts.age_unknown),
   bridge_->gpu_alignment()?native_bytes:color_pixels()*5,bridge_->gpu_alignment()?native_bytes:size_t(0),bridge_->gpu_alignment()?color_pixels()*2:size_t(0),bridge_->gpu_alignment()?4u:0u,bridge_->gpu_alignment()?compact_bytes():size_t(0));
  if(length<0||size_t(length)>=sizeof(text))throw std::runtime_error("resident status formatting overflow/failure");
  status_message_.data.assign(text,size_t(length));
  status_->publish(static_cast<const std_msgs::msg::String&>(status_message_));
 }
 // Best effort diagnostics: irrevocable closure precedes every fallible action.
 // Delivery is not guaranteed under allocation failure, unavailable DDS or a hung driver.
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

public:
 explicit CameraComponent(const rclcpp::NodeOptions& options):Node("resident_camera",options),ownership_(publication_gate(),this),camera_(cameras_){
  const auto mode=declare_parameter<std::string>("backend","auto");serial_=declare_parameter<std::string>("serial","");
  profile_requested_=declare_parameter<std::string>("profile","realtime");profile_name_=canonical_camera_profile(profile_requested_);selected_profile_=camera_capture_profile(profile_name_);publication_session_token_=ownership_.token();
  require_raw_or_cpu_fallback(mode,{});status_message_.data.reserve(4096);failure_diagnostic_.data.reserve(2048);failure_status_.data.reserve(2048);
  status_=create_publisher<std_msgs::msg::String>("/station/resident/status",rclcpp::QoS(1).transient_local());
  failures_=create_publisher<std_msgs::msg::String>("/station/resident/measurements",rclcpp::QoS(1).transient_local());
  publication_gate().invalidate(this,ownership_.token(),[&]{measurement_writer()=failures_;});
  try {
   // Positively identify actual loaded SDK arithmetic before camera.start.
   SdkAlignmentProbe probe;
   if(mode!="cpu"){probe=probe_loaded_sdk_alignment();if(!probe.admitted())fallback_reason_=probe.reason.empty()?"loaded SDK convention not identified":probe.reason;}
   require_raw_or_cpu_fallback(mode,fallback_reason_);
   auto devices=cameras_.query_devices();rs2::device selected;
   for(auto device:devices)if(serial_.empty()||serial_==device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)){selected=device;break;}
   if(!selected)throw std::runtime_error("resident camera: requested RealSense device not found; no camera started");
   serial_=selected.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
   bool rgb=false,z16=false;
   for(auto sensor:selected.query_sensors())for(auto profile:sensor.get_stream_profiles()){
    auto video=profile.as<rs2::video_stream_profile>();if(!video||video.fps()!=int(selected_profile_.fps))continue;
    rgb|=video.stream_type()==RS2_STREAM_COLOR&&video.format()==RS2_FORMAT_RGB8&&video.width()==int(selected_profile_.color_width)&&video.height()==int(selected_profile_.color_height);
    z16|=video.stream_type()==RS2_STREAM_DEPTH&&video.format()==RS2_FORMAT_Z16&&video.width()==int(selected_profile_.depth_width)&&video.height()==int(selected_profile_.depth_height);
   }
   if(!rgb||!z16)throw std::runtime_error("resident camera: enumerated selected exact 30 Hz RGB8/Z16 profiles unavailable");
   rs2::config config;config.enable_device(serial_);config.enable_stream(RS2_STREAM_COLOR,selected_profile_.color_width,selected_profile_.color_height,RS2_FORMAT_RGB8,selected_profile_.fps);config.enable_stream(RS2_STREAM_DEPTH,selected_profile_.depth_width,selected_profile_.depth_height,RS2_FORMAT_Z16,selected_profile_.fps);
   // Resolve selected exact profiles before this component starts its own owner.
   auto resolved=config.resolve(camera_);
   auto resolved_source=source_calibration(resolved.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),resolved.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),resolved.get_device().first<rs2::depth_sensor>().get_depth_scale());
   validate_selected_profiles(resolved_source);
   source_=resolved_source;source_.alignment.generation=camera_generation(source_,serial_,session_nonce());
   last_input_.serial.reserve(serial_.size());failure_input_.serial.reserve(serial_.size());worker_input_.serial.reserve(serial_.size());scheduler_.reserve_serial(serial_.size());
   const auto share=ament_index_cpp::get_package_share_directory("so101_station_vulkan");shaders_=share+"/shaders";alignment_shaders_=share+"/alignment-shaders";
   bool raw=mode!="cpu"&&fallback_reason_.empty();
   if(raw){fallback_reason_=raw_admission_failure(probe,source_);raw=fallback_reason_.empty();require_raw_or_cpu_fallback(mode,fallback_reason_);}
   // GPU resource/admission failures are resolved before owning USB capture.
   std::unique_ptr<RawCaptureBridge> cold;
   if(raw){try{cold=make_bridge(true,true);}catch(const std::exception& error){fallback_reason_=std::string("cold raw GPU preparation failed: ")+error.what();require_raw_or_cpu_fallback(mode,fallback_reason_);raw=false;}}
   running_=true;auto profile=camera_.start(config);
   if(serial_!=profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))throw std::runtime_error("active camera serial changed");
   const auto active_source=source_calibration(profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),profile.get_device().first<rs2::depth_sensor>().get_depth_scale());
   validate_selected_profiles(active_source);if(!same_camera_source(source_,active_source))throw std::runtime_error("resolved and active camera calibration differ");
   wait_startup_stable();
   if(raw){try{cold_live_selfcheck(std::move(cold));bridge_=make_bridge(true);}catch(const std::exception& error){fallback_reason_=std::string("cold raw GPU selfcheck failed: ")+error.what();require_raw_or_cpu_fallback(mode,fallback_reason_);raw=false;}}
   if(!raw&&!running_){
    // The cold function stopped only this pipeline before safely draining GPU
    // resources. Auto may restart this selected configuration once for CPU.
    auto restart_expected=config.resolve(camera_);
    const auto expected_source=source_calibration(restart_expected.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),restart_expected.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),restart_expected.get_device().first<rs2::depth_sensor>().get_depth_scale());
    validate_selected_profiles(expected_source);
    running_=true;auto restarted=camera_.start(config);
    if(serial_!=restarted.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))throw std::runtime_error("CPU fallback restart camera serial changed");
    source_=source_calibration(restarted.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),restarted.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),restarted.get_device().first<rs2::depth_sensor>().get_depth_scale());
    validate_selected_profiles(source_);if(!same_camera_source(expected_source,source_))throw std::runtime_error("CPU fallback restart source calibration differs from resolved profiles");
    source_.alignment.generation=camera_generation(source_,serial_,session_nonce());domains_known_=false;
    wait_startup_stable();
   }
   if(!raw)bridge_=make_bridge(false);
   frames_=create_publisher<FrameLease>("/station/resident/frame",rclcpp::QoS(1));
   info_=create_publisher<sensor_msgs::msg::CameraInfo>("/station/resident/color/camera_info",rclcpp::QoS(1).transient_local());
   const auto& k=source_.alignment.color;sensor_msgs::msg::CameraInfo ci;ci.width=k.width;ci.height=k.height;ci.header.frame_id="camera_color_optical_frame";ci.distortion_model="plumb_bob";ci.k={k.fx,0,k.ppx,0,k.fy,k.ppy,0,0,1};ci.d.assign(k.coeffs,k.coeffs+5);info_->publish(ci);
   if(!bridge_->accept_source(previous_color_,previous_depth_))throw std::runtime_error("normal source startup baseline rejected");
   last_input_.frame_number=previous_color_;last_input_.depth_frame_number=previous_depth_;last_input_.hardware_timestamp_ms=previous_color_time_;last_input_.depth_hardware_timestamp_ms=previous_depth_time_;last_input_.timestamp_domain=color_domain_;last_input_.depth_timestamp_domain=depth_domain_;last_input_.calibration_generation=source_.alignment.generation;last_input_.serial=serial_;source_admission_.seed(last_input_);
   publication_gate().admit(this,ownership_.token(),source_.alignment.generation);
   normal_start_ns_=steady_ns();next_status_ns_=normal_start_ns_+1000000000;
   report_status();timer_=create_wall_timer(20ms,[this]{try{tick();}catch(const std::exception& error){latch_failure(error.what());}catch(...){latch_failure("non-standard resident exception");}});start_threads();
  }catch(...){shutdown_threads();publication_gate().invalidate(this,ownership_.token());throw;}
 }
 ~CameraComponent()override{if(timer_)timer_->cancel();shutdown_threads();publication_gate().invalidate(this,ownership_.token());bridge_.reset();}
};

class ObjectsComponent : public rclcpp::Node {
  rclcpp::Subscription<FrameLease>::SharedPtr subscription_;
  std_msgs::msg::String measurement_;

public:
  explicit ObjectsComponent(const rclcpp::NodeOptions &options)
      : Node("resident_measurements", options) {
    measurement_.data.reserve(kMeasurementTextCapacity);
    subscription_ = create_subscription<FrameLease>(
        "/station/resident/frame", rclcpp::QoS(1),
        [this](const FrameLease &frame) {
          const auto &slot = frame.slot();
          format_unqualified_measurement(measurement_.data, slot.metadata,
                                         slot.component_count, now().nanoseconds());
          // Const-reference publication copies into ROS-owned storage. Never
          // move/loan this reusable member to an asynchronous consumer.
          publication_gate().publish(slot.metadata.calibration_generation,slot.metadata.publication_session_token,[&]{
            auto measurements_=measurement_writer().lock();
            if(measurements_)measurements_->publish(static_cast<const std_msgs::msg::String &>(measurement_));
          });
        });
  }
};
} // namespace so101_station_vulkan
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::CameraComponent)
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::ObjectsComponent)
