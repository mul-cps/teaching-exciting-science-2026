#pragma once
#include "so101_station_vulkan/alignment.hpp"
#include <string>
namespace so101_station_vulkan {
struct CameraCaptureProfile {unsigned color_width,color_height,depth_width,depth_height,fps;};
CameraCaptureProfile camera_capture_profile(const std::string& name);
// Bounded immutable source description, compared by value without padding bytes.
struct CameraProfileIdentity {
 int unique_id=0, stream_index=0, width=0, height=0, fps=0, format=0, stream=0;
 bool operator==(const CameraProfileIdentity&)const;
};
struct CameraSourceCalibration {
 CameraProfileIdentity color,depth;
 AlignmentCalibration alignment;
};
bool same_camera_source(const CameraSourceCalibration&,const CameraSourceCalibration&);
void validate_camera_profiles(const CameraSourceCalibration&);
uint64_t camera_generation(const CameraSourceCalibration&,const std::string& serial,uint64_t session_nonce);
struct SdkAlignmentProbe {
 unsigned fixtures=0;
 bool scalar_exact=false, sse_distinguished=false;
 bool admitted()const{return fixtures==3&&scalar_exact&&sse_distinguished;}
 std::string reason;
};
// Runs only software_device frames on the actual loaded SDK; never opens USB or Vulkan.
SdkAlignmentProbe probe_loaded_sdk_alignment();
// No caller-selectable convention or qualification bypass.
std::string raw_admission_failure(const SdkAlignmentProbe&,const CameraSourceCalibration&);
// CPU-testable cold verifier. A failed or duplicate cohort permanently closes
// this startup gate; only three distinct, completely verified originals admit.
class ColdCameraSelfcheck {
 uint64_t generation_,color_=0,depth_=0;
 unsigned color_width_,color_height_;size_t color_pixels_,depth_pixels_;
 unsigned passes_=0;
 bool failed_=false;
 std::string serial_;
public:
 ColdCameraSelfcheck(uint64_t generation,const std::string& serial,unsigned color_width=640,unsigned color_height=480,unsigned depth_width=640,unsigned depth_height=480);
 void verify(const FrameMetadata&,const uint16_t* native_z16,const uint8_t* sdk_z16,const uint16_t* aligned,
             const uint32_t* reference,const float* reference_xyz,const float* actual_xyz,
             const Slot& cpu,const Slot& gpu);
 void require_complete()const;
 unsigned passes()const{return passes_;}
};
// One owner for the absolute /station/resident measurement stream. Publication
// and invalidation share a lock, so queued old callbacks cannot overwrite UNKNOWN.
class ResidentPublicationGate {
 std::mutex mutex_;
 const void* owner_=nullptr;
 uint64_t generation_=0;
public:
 void reserve(const void* owner);
 void admit(const void* owner,uint64_t generation);
 void release(const void* owner);
 void invalidate(const void* owner);
 template<class Publish> void invalidate(const void* owner,Publish&& publish_unknown){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!owner||owner_!=owner)return;
  generation_=0;publish_unknown();
 }
 template<class Publish> bool publish(uint64_t generation,Publish&& publish_current){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!owner_||!generation||generation_!=generation)return false;
  publish_current();return true;
 }
};
class ResidentCameraOwner {
 ResidentPublicationGate& gate_;
 const void* owner_;
public:
 ResidentCameraOwner(ResidentPublicationGate& gate,const void* owner):gate_(gate),owner_(owner){gate_.reserve(owner_);}
 ~ResidentCameraOwner(){gate_.release(owner_);}
 ResidentCameraOwner(const ResidentCameraOwner&)=delete;
 ResidentCameraOwner& operator=(const ResidentCameraOwner&)=delete;
};
void require_raw_or_cpu_fallback(const std::string& mode,const std::string& failure);
}
