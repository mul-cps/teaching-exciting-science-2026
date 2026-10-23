#include "so101_station_vulkan/frame_adapter.hpp"
#include "so101_station_vulkan/raw_capture_bridge.hpp"
#include "so101_station_vulkan/raw_camera_admission.hpp"
#include "so101_station_vulkan/camera_startup_gate.hpp"
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
constexpr unsigned kWidth=640,kHeight=480,kPixels=kWidth*kHeight;
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
void copy_rows(uint8_t* destination,const rs2::video_frame& source,unsigned row_bytes){
 const auto* data=static_cast<const uint8_t*>(source.get_data());
 const auto stride=source.get_stride_in_bytes();
 if(stride==int(row_bytes)){std::memcpy(destination,data,size_t(row_bytes)*kHeight);return;}
 for(unsigned y=0;y<kHeight;y++)std::memcpy(destination+y*row_bytes,data+y*stride,row_bytes);
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
 rclcpp::Publisher<FrameLease>::SharedPtr frames_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,failures_;
 rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
 rclcpp::TimerBase::SharedPtr timer_;
 std::string serial_,fallback_reason_,shaders_,alignment_shaders_;
 bool running_=false,domains_known_=false;
 int color_domain_=0,depth_domain_=0;
 std_msgs::msg::String status_message_;
 FrameMetadata last_input_,failure_input_;
 uint64_t previous_color_=0,previous_depth_=0;
 double previous_color_time_=0,previous_depth_time_=0;
 void stop_owned_camera() noexcept {
  if(!running_)return;
  running_=false;try{camera_.stop();}catch(...){}
 }
 void publish_status(const std::string& message){std_msgs::msg::String s;s.data=message;status_->publish(s);}
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
   const unsigned row=frame.get_profile().stream_type()==RS2_STREAM_COLOR?kWidth*3:kWidth*2;
   if(frame.get_width()!=int(kWidth)||frame.get_height()!=int(kHeight)||frame.get_stride_in_bytes()<int(row)||frame.get_data_size()<size_t(frame.get_stride_in_bytes())*(kHeight-1)+row)throw std::runtime_error("invalid original frame extent/stride/provenance");
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
  }catch(...){publication_gate().invalidate(this);stop_owned_camera();throw;}
 }
 void original_metadata(FrameMetadata& m,const rs2::frameset& native,uint64_t receipt){
  auto color=native.get_color_frame();auto depth=native.get_depth_frame();
  m.width=kWidth;m.height=kHeight;m.frame_number=color.get_frame_number();m.depth_frame_number=depth.get_frame_number();
  m.timestamp_ns=receipt;m.hardware_timestamp_ms=color.get_timestamp();m.depth_hardware_timestamp_ms=depth.get_timestamp();
  m.timestamp_domain=int(color.get_frame_timestamp_domain());m.depth_timestamp_domain=int(depth.get_frame_timestamp_domain());
  m.calibration_generation=source_.alignment.generation;m.serial=serial_;
 }
 // Raw normal path: exactly one native RGB upload + one native Z16 upload.
 // Alignment and compact core are submitted consecutively by the bridge.
 void copy_raw_planes(RawFrame& frame,const rs2::frameset& native,const FrameMetadata& metadata){
  auto& slot=frame.core.slot();slot.metadata=metadata;slot.calibration=core_calibration();
  frame.depth.metadata()=metadata;
  copy_rows(static_cast<uint8_t*>(slot.upload.mapped),native.get_color_frame(),kWidth*3);
  copy_rows(reinterpret_cast<uint8_t*>(frame.depth.upload()),native.get_depth_frame(),kWidth*2);
 }
 // Vendor CPU alignment is used only by the explicitly selected CPU path.
 void copy_vendor_planes(RawFrame& frame,const rs2::frameset& native,const FrameMetadata& metadata){
  auto aligned=vendor_alignment_.process(native).as<rs2::frameset>();
  auto& slot=frame.core.slot();slot.metadata=metadata;slot.calibration=core_calibration();
  auto* bytes=static_cast<uint8_t*>(slot.upload.mapped);
  copy_rows(bytes,aligned.get_color_frame(),kWidth*3);
  copy_rows(bytes+kPixels*3,aligned.get_depth_frame(),kWidth*2);
 }
 std::unique_ptr<RawCaptureBridge> make_bridge(bool raw,bool diagnostics=false){
  auto core=std::make_unique<Pipeline>(kWidth,kHeight,raw?"vulkan":"cpu",shaders_);
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
  Pipeline independent(kWidth,kHeight,"cpu",shaders_);
  std::vector<uint32_t> reference_depth(kPixels);
  std::vector<float> reference_xyz(kPixels*4);
  ColdCameraSelfcheck gate(source_.alignment.generation,serial_);
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
   copy_rows(bytes,original.get_color_frame(),kWidth*3);copy_rows(bytes+kPixels*3,sdk,kWidth*2);
   alignment_reference(source_.alignment,raw_reference,reference_depth.data(),reference_xyz.data());
   // Independent CPU reference consumes the SDK-aligned original cohort.
   independent.submit(expected);independent.finish(expected);
   gate.verify(metadata,raw_reference,bytes+kPixels*3,completed.depth.diagnostic_depth(),reference_depth.data(),reference_xyz.data(),completed.depth.diagnostic_xyz(),cpu,completed.core.slot());
   remember_source(metadata);
  }
  gate.require_complete();
  }catch(...){publication_gate().invalidate(this);stop_owned_camera();throw;}
 }
 void capture_pending(const rs2::frameset& native,uint64_t received_ros_ns){
  // Preserve receipt/source IDs even when validation itself fails.
  failure_input_.timestamp_ns=received_ros_ns;
  failure_input_.frame_number=native.get_color_frame()?native.get_color_frame().get_frame_number():0;
  failure_input_.depth_frame_number=native.get_depth_frame()?native.get_depth_frame().get_frame_number():0;
  if(native.get_color_frame()&&native.get_depth_frame()){original_metadata(last_input_,native,received_ros_ns);failure_input_=last_input_;}
  verify_original(native);
  const auto& metadata=last_input_;
  if(metadata.frame_number<previous_color_||metadata.depth_frame_number<previous_depth_||metadata.hardware_timestamp_ms<previous_color_time_||metadata.depth_hardware_timestamp_ms<previous_depth_time_)throw std::runtime_error("source identity/timestamp regressed; restart required");
  if(!bridge_->accept_source(metadata.frame_number,metadata.depth_frame_number))return;
  previous_color_=metadata.frame_number;previous_depth_=metadata.depth_frame_number;previous_color_time_=metadata.hardware_timestamp_ms;previous_depth_time_=metadata.depth_hardware_timestamp_ms;
  bridge_->discard_pending();auto frame=bridge_->acquire();
  if(!frame){bridge_->no_slot();return;}
  if(bridge_->gpu_alignment())copy_raw_planes(frame,native,metadata);
  else copy_vendor_planes(frame,native,metadata);
  bridge_->offer(std::move(frame));
 }
 void tick(){
  if(bridge_->active())failure_input_=bridge_->active().core.slot().metadata;
  RawFrame completed;const bool have_completed=bridge_->collect(completed);
  if(have_completed)frames_->publish(completed.core);
  rs2::frameset native;
  if(camera_.poll_for_frames(&native)){
   // Receipt precedes all copies and vendor processing.
   const auto received_ros_ns=uint64_t(now().nanoseconds());
   capture_pending(native,received_ros_ns);
  }
  bridge_->dispatch_pending();
  if(have_completed&&bridge_->counters().completed%30==0)report_status();
 }
 void report_status(){
  const auto& counts=bridge_->counters();
  char text[4096];
  const int length=std::snprintf(text,sizeof(text),
   "experimental; backend=%s; alignment=%s; fallback_reason=%.1024s; geometry_admission=unqualified; execution_eligible=false; full_station_qualified=false; recording=disabled; received=%llu; completed=%llu; dropped=%llu; replaced=%llu; duplicate_or_regressed=%llu; host_plane_copy_bytes=%u; native_upload_bytes=%u; aligned_d2d_bytes=%u; alignment_fault_readback_bytes=%u; core_compact_readback_bytes=%zu",
   bridge_->pipeline().backend().c_str(),bridge_->gpu_alignment()?"raw_vulkan":"vendor_cpu",fallback_reason_.c_str(),
   static_cast<unsigned long long>(counts.received),static_cast<unsigned long long>(counts.completed),static_cast<unsigned long long>(counts.no_slot),static_cast<unsigned long long>(counts.replaced),static_cast<unsigned long long>(counts.duplicate_or_regressed),
   kPixels*5,bridge_->gpu_alignment()?kPixels*5:0,bridge_->gpu_alignment()?kPixels*2:0,bridge_->gpu_alignment()?4u:0u,bridge_->gpu_alignment()?compact_bytes():size_t(0));
  if(length<0)throw std::runtime_error("resident status formatting failed");
  status_message_.data.assign(text,size_t(length)<sizeof(text)?size_t(length):sizeof(text)-1);
  status_->publish(static_cast<const std_msgs::msg::String&>(status_message_));
 }
 void latch_failure(const std::exception& error){
  timer_->cancel();
  const auto& m=failure_input_;
  std_msgs::msg::String diagnostic;
  diagnostic.data="{\"status\":\"unknown\",\"geometry_admission\":\"unqualified\",\"execution_eligible\":false,\"source_frame_number\":"+std::to_string(m.frame_number)+",\"source_depth_frame_number\":"+std::to_string(m.depth_frame_number)+",\"source_timestamp_ns\":"+std::to_string(m.timestamp_ns)+",\"error\":\"resident_capture_failed_restart_required\"}";
  publication_gate().invalidate(this,[&]{failures_->publish(diagnostic);});
  stop_owned_camera();publish_status(std::string("fatal; geometry_admission=unqualified; execution_eligible=false; restart_required=true; reason=")+error.what());
  RCLCPP_ERROR(get_logger(),"%s",error.what());
 }
public:
 explicit CameraComponent(const rclcpp::NodeOptions& options):Node("resident_camera",options),ownership_(publication_gate(),this),camera_(cameras_){
  const auto mode=declare_parameter<std::string>("backend","auto");serial_=declare_parameter<std::string>("serial","");
  require_raw_or_cpu_fallback(mode,{});status_message_.data.reserve(4096);
  status_=create_publisher<std_msgs::msg::String>("/station/resident/status",rclcpp::QoS(1).transient_local());
  failures_=create_publisher<std_msgs::msg::String>("/station/resident/measurements",rclcpp::QoS(1).transient_local());
  publication_gate().invalidate(this,[&]{measurement_writer()=failures_;});
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
    auto video=profile.as<rs2::video_stream_profile>();if(!video||video.width()!=640||video.height()!=480||video.fps()!=30)continue;
    rgb|=video.stream_type()==RS2_STREAM_COLOR&&video.format()==RS2_FORMAT_RGB8;
    z16|=video.stream_type()==RS2_STREAM_DEPTH&&video.format()==RS2_FORMAT_Z16;
   }
   if(!rgb||!z16)throw std::runtime_error("resident camera: enumerated exact 640x480x30 RGB8/Z16 unavailable");
   rs2::config config;config.enable_device(serial_);config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_RGB8,30);config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
   // Resolve selected exact profiles before this component starts its own owner.
   auto resolved=config.resolve(camera_);
   auto resolved_source=source_calibration(resolved.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),resolved.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),resolved.get_device().first<rs2::depth_sensor>().get_depth_scale());
   validate_camera_profiles(resolved_source);
   source_=resolved_source;source_.alignment.generation=camera_generation(source_,serial_,session_nonce());
   last_input_.serial.reserve(serial_.size());failure_input_.serial.reserve(serial_.size());
   const auto share=ament_index_cpp::get_package_share_directory("so101_station_vulkan");shaders_=share+"/shaders";alignment_shaders_=share+"/alignment-shaders";
   bool raw=mode!="cpu"&&fallback_reason_.empty();
   if(raw){fallback_reason_=raw_admission_failure(probe,source_);raw=fallback_reason_.empty();require_raw_or_cpu_fallback(mode,fallback_reason_);}
   // GPU resource/admission failures are resolved before owning USB capture.
   std::unique_ptr<RawCaptureBridge> cold;
   if(raw){try{cold=make_bridge(true,true);}catch(const std::exception& error){fallback_reason_=std::string("cold raw GPU preparation failed: ")+error.what();require_raw_or_cpu_fallback(mode,fallback_reason_);raw=false;}}
   running_=true;auto profile=camera_.start(config);
   if(serial_!=profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))throw std::runtime_error("active camera serial changed");
   const auto active_source=source_calibration(profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),profile.get_device().first<rs2::depth_sensor>().get_depth_scale());
   validate_camera_profiles(active_source);if(!same_camera_source(source_,active_source))throw std::runtime_error("resolved and active camera calibration differ");
   wait_startup_stable();
   if(raw){try{cold_live_selfcheck(std::move(cold));bridge_=make_bridge(true);}catch(const std::exception& error){fallback_reason_=std::string("cold raw GPU selfcheck failed: ")+error.what();require_raw_or_cpu_fallback(mode,fallback_reason_);raw=false;}}
   if(!raw&&!running_){
    // The cold function stopped only this pipeline before safely draining GPU
    // resources. Auto may restart this selected configuration once for CPU.
    auto restart_expected=config.resolve(camera_);
    const auto expected_source=source_calibration(restart_expected.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),restart_expected.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),restart_expected.get_device().first<rs2::depth_sensor>().get_depth_scale());
    validate_camera_profiles(expected_source);
    running_=true;auto restarted=camera_.start(config);
    if(serial_!=restarted.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))throw std::runtime_error("CPU fallback restart camera serial changed");
    source_=source_calibration(restarted.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>(),restarted.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>(),restarted.get_device().first<rs2::depth_sensor>().get_depth_scale());
    validate_camera_profiles(source_);if(!same_camera_source(expected_source,source_))throw std::runtime_error("CPU fallback restart source calibration differs from resolved profiles");
    source_.alignment.generation=camera_generation(source_,serial_,session_nonce());domains_known_=false;
    wait_startup_stable();
   }
   if(!raw)bridge_=make_bridge(false);
   frames_=create_publisher<FrameLease>("/station/resident/frame",rclcpp::QoS(1));
   info_=create_publisher<sensor_msgs::msg::CameraInfo>("/station/resident/color/camera_info",rclcpp::QoS(1).transient_local());
   const auto& k=source_.alignment.color;sensor_msgs::msg::CameraInfo ci;ci.width=640;ci.height=480;ci.header.frame_id="camera_color_optical_frame";ci.distortion_model="plumb_bob";ci.k={k.fx,0,k.ppx,0,k.fy,k.ppy,0,0,1};ci.d.assign(k.coeffs,k.coeffs+5);info_->publish(ci);
   if(!bridge_->accept_source(previous_color_,previous_depth_))throw std::runtime_error("normal source startup baseline rejected");
   publication_gate().admit(this,source_.alignment.generation);
   report_status();timer_=create_wall_timer(2ms,[this]{try{tick();}catch(const std::exception& error){latch_failure(error);}});
  }catch(...){publication_gate().invalidate(this);stop_owned_camera();publication_gate().release(this);throw;}
 }
 ~CameraComponent()override{if(timer_)timer_->cancel();publication_gate().invalidate(this);stop_owned_camera();bridge_.reset();publication_gate().release(this);}
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
          publication_gate().publish(slot.metadata.calibration_generation,[&]{
            auto measurements_=measurement_writer().lock();
            if(measurements_)measurements_->publish(static_cast<const std_msgs::msg::String &>(measurement_));
          });
        });
  }
};
} // namespace so101_station_vulkan
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::CameraComponent)
RCLCPP_COMPONENTS_REGISTER_NODE(so101_station_vulkan::ObjectsComponent)
