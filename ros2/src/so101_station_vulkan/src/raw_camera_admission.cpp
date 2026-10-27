#include "so101_station_vulkan/raw_camera_admission.hpp"
#include <librealsense2/hpp/rs_internal.hpp>
#include <array>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace so101_station_vulkan {
CameraCaptureProfile camera_capture_profile(const std::string& name){
 if(name=="vga")return {640,480,640,480,30};
 if(name=="720p")return {1280,720,1280,720,30};
 if(name=="1080p")return {1920,1080,1280,720,30};
 throw std::invalid_argument("camera profile must be vga, 720p or 1080p");
}
bool CameraProfileIdentity::operator==(const CameraProfileIdentity& b)const {
 return unique_id==b.unique_id&&stream_index==b.stream_index&&width==b.width&&height==b.height&&fps==b.fps&&format==b.format&&stream==b.stream;
}
namespace {
bool qualified_camera_pair(unsigned cw,unsigned ch,unsigned dw,unsigned dh){
 return (cw==640&&ch==480&&dw==640&&dh==480)||(cw==1280&&ch==720&&dw==1280&&dh==720)||(cw==1920&&ch==1080&&dw==1280&&dh==720);
}
bool intrinsic_equal(const rs2_intrinsics& a,const rs2_intrinsics& b){
 if(a.width!=b.width||a.height!=b.height||a.ppx!=b.ppx||a.ppy!=b.ppy||a.fx!=b.fx||a.fy!=b.fy||a.model!=b.model)return false;
 for(unsigned i=0;i<5;i++)if(a.coeffs[i]!=b.coeffs[i])return false;
 return true;
}
// Canonical field encoding (no struct padding, no private values in diagnostics).
struct Hash {
 uint64_t value=14695981039346656037ull;
 void number(uint64_t n){for(unsigned i=0;i<8;i++){value^=uint8_t(n>>(8*i));value*=1099511628211ull;}}
 void real(float n){uint32_t bits;std::memcpy(&bits,&n,4);number(bits);}
 void intrinsic(const rs2_intrinsics& k){number(k.width);number(k.height);real(k.ppx);real(k.ppy);real(k.fx);real(k.fy);number(k.model);for(float v:k.coeffs)real(v);}
 void profile(const CameraProfileIdentity& p){number(p.unique_id);number(p.stream_index);number(p.width);number(p.height);number(p.fps);number(p.format);number(p.stream);}
};
std::vector<uint16_t> software_sdk_align(const AlignmentCalibration& k,std::vector<uint16_t>& raw){
 constexpr size_t n=640*480;
 rs2::software_device device;auto ds=device.add_sensor("Depth"),cs=device.add_sensor("Color");
 auto depth=ds.add_video_stream({RS2_STREAM_DEPTH,0,1,640,480,30,2,RS2_FORMAT_Z16,k.depth});
 auto color=cs.add_video_stream({RS2_STREAM_COLOR,0,2,640,480,30,3,RS2_FORMAT_RGB8,k.color});
 depth.register_extrinsics_to(color,k.depth_to_color);device.create_matcher(RS2_MATCHER_DEFAULT);
 ds.add_read_only_option(RS2_OPTION_DEPTH_UNITS,k.depth_units);
 rs2::syncer sync;ds.open(depth);cs.open(color);ds.start(sync);cs.start(sync);
 std::vector<uint8_t> rgb(n*3);std::vector<uint16_t> output(n);rs2::align align(RS2_STREAM_COLOR);bool got=false;
 try {
  for(int f=1;f<=5;f++){
   ds.on_video_frame({raw.data(),[](void*){},640*2,2,f*1000.0/30,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,f,depth,k.depth_units});
   cs.on_video_frame({rgb.data(),[](void*){},640*3,3,f*1000.0/30,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,f,color});
   auto frames=sync.wait_for_frames(1000);if(!frames.get_depth_frame()||!frames.get_color_frame())continue;
   auto result=align.process(frames).as<rs2::frameset>().get_depth_frame();
   if(result.get_frame_number()!=uint64_t(f)||result.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)throw std::runtime_error("SDK software probe provenance changed");
   for(unsigned y=0;y<480;y++)std::memcpy(output.data()+y*640,static_cast<const uint8_t*>(result.get_data())+y*result.get_stride_in_bytes(),640*2);
   got=true;
  }
 } catch(...) {ds.stop();cs.stop();ds.close();cs.close();throw;}
 ds.stop();cs.stop();ds.close();cs.close();if(!got)throw std::runtime_error("SDK software probe did not synchronize");return output;
}
}
bool same_camera_source(const CameraSourceCalibration& a,const CameraSourceCalibration& b){
 if(!(a.color==b.color)||!(a.depth==b.depth)||!intrinsic_equal(a.alignment.color,b.alignment.color)||!intrinsic_equal(a.alignment.depth,b.alignment.depth)||a.alignment.depth_units!=b.alignment.depth_units)return false;
 for(unsigned i=0;i<9;i++)if(a.alignment.depth_to_color.rotation[i]!=b.alignment.depth_to_color.rotation[i])return false;
 for(unsigned i=0;i<3;i++)if(a.alignment.depth_to_color.translation[i]!=b.alignment.depth_to_color.translation[i])return false;
 return true;
}
void validate_camera_profiles(const CameraSourceCalibration& source){
 if(!qualified_camera_pair(source.color.width,source.color.height,source.depth.width,source.depth.height))throw std::invalid_argument("active camera profile pair is not exact vga, 720p or 1080p");
 for(const auto* p:{&source.color,&source.depth})if(p->fps!=30||p->unique_id<=0)throw std::invalid_argument("active camera profile requires exact 30 Hz and valid unique IDs");
 if(source.alignment.color.width!=source.color.width||source.alignment.color.height!=source.color.height||source.alignment.depth.width!=source.depth.width||source.alignment.depth.height!=source.depth.height)throw std::invalid_argument("camera profile and calibration extents differ");
 if(source.color.format!=RS2_FORMAT_RGB8||source.color.stream!=RS2_STREAM_COLOR||source.depth.format!=RS2_FORMAT_Z16||source.depth.stream!=RS2_STREAM_DEPTH)throw std::invalid_argument("active camera formats are not RGB8/Z16");
}
uint64_t camera_generation(const CameraSourceCalibration& s,const std::string& serial,uint64_t nonce){
 if(!nonce||serial.empty())throw std::invalid_argument("camera session identity is missing");
 Hash h;h.number(nonce);h.profile(s.color);h.profile(s.depth);h.intrinsic(s.alignment.color);h.intrinsic(s.alignment.depth);h.real(s.alignment.depth_units);
 for(float v:s.alignment.depth_to_color.rotation)h.real(v);
 for(float v:s.alignment.depth_to_color.translation)h.real(v);
 for(unsigned char c:serial)h.number(c);
 return h.value?h.value:1;
}
SdkAlignmentProbe probe_loaded_sdk_alignment(){
 SdkAlignmentProbe result;
 try {
  constexpr size_t n=640*480;bool exact=true,distinguished=false;
  // Same deterministic, nonambiguous calibration/raw cases 2/3/4 as the oracle.
  for(unsigned fixture:{2u,3u,4u}){
   AlignmentCalibration k;k.depth={640,480,319.5f,239.5f,fixture%2?390.f:600.f,fixture%2?390.f:600.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};
   k.color={640,480,319.7f,239.2f,610.f,605.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};
   k.depth_to_color={{1,0,0,0,1,0,0,0,1},{.0234f,fixture>2?-.0027f:0.f,fixture>3?.001f:0.f}};
   k.convention=AlignmentConvention::Scalar;k.generation=1;k.depth_units=.001f;
   std::vector<uint16_t> raw(n);uint32_t seed=0x452331ab;
   for(size_t i=0;i<n;i++){seed=1664525*seed+1013904223;raw[i]=i%7==0?0:uint16_t(250+(seed%4250));}
   if(fixture==4)for(size_t i=0;i<n;i++)raw[i]=i%101?0:65535;
   auto actual=software_sdk_align(k,raw);std::vector<uint32_t> reference(n);std::vector<float> xyz(n*4);
   alignment_reference(k,raw.data(),reference.data(),xyz.data());
   for(size_t i=0;i<n;i++)exact&=reference[i]==actual[i];
   k.convention=AlignmentConvention::Sse;alignment_reference(k,raw.data(),reference.data(),xyz.data());
   for(size_t i=0;i<n;i++)distinguished|=reference[i]!=actual[i];
   ++result.fixtures;
  }
  result.scalar_exact=exact;result.sse_distinguished=distinguished;
  if(!exact)result.reason="loaded SDK is not the qualified scalar alignment convention";
  else if(!distinguished)result.reason="loaded SDK scalar convention was not positively distinguished";
 } catch(const std::exception& e){result.reason=std::string("loaded SDK software alignment probe failed: ")+e.what();}
 return result;
}
std::string raw_admission_failure(const SdkAlignmentProbe& probe,const CameraSourceCalibration& source){
 if(!probe.admitted())return probe.reason.empty()?"loaded SDK scalar convention not identified":probe.reason;
 try {validate_camera_profiles(source);if(source.alignment.convention!=AlignmentConvention::Scalar)throw std::invalid_argument("only identified scalar convention is qualified");validate_alignment(source.alignment);}
 catch(const std::exception& e){return e.what();}
 return {};
}
ColdCameraSelfcheck::ColdCameraSelfcheck(uint64_t generation,const std::string& serial,unsigned cw,unsigned ch,unsigned dw,unsigned dh):generation_(generation),color_width_(cw),color_height_(ch),color_pixels_(size_t(cw)*ch),depth_pixels_(size_t(dw)*dh),serial_(serial){
 if(!qualified_camera_pair(cw,ch,dw,dh))throw std::invalid_argument("cold selfcheck requires an exact camera profile pair");
 if(!generation||serial.empty())throw std::invalid_argument("cold selfcheck source identity is missing");
}
void ColdCameraSelfcheck::verify(const FrameMetadata& m,const uint16_t* native,const uint8_t* sdk,const uint16_t* aligned,const uint32_t* reference,const float* expected_xyz,const float* actual_xyz,const Slot& cpu,const Slot& gpu){
 if(failed_)throw std::runtime_error("cold selfcheck failed; restart required");
 try {
  if(passes_>=3||m.width!=color_width_||m.height!=color_height_||m.calibration_generation!=generation_||m.serial!=serial_||m.frame_number<=color_||m.depth_frame_number<=depth_||!std::isfinite(m.hardware_timestamp_ms)||!std::isfinite(m.depth_hardware_timestamp_ms))throw std::runtime_error("cold selfcheck original source cohort mismatch");
  if(!native||!sdk||!aligned||!reference||!expected_xyz||!actual_xyz)throw std::invalid_argument("cold selfcheck buffers missing");
  unsigned native_nonzero=0,aligned_nonzero=0;
  for(size_t i=0;i<depth_pixels_;i++)native_nonzero+=native[i]!=0;
  for(size_t i=0;i<color_pixels_;i++){
   const auto vendor=uint16_t(sdk[2*i]|unsigned(sdk[2*i+1])<<8);
   aligned_nonzero+=vendor!=0;
   if(aligned[i]!=vendor||reference[i]!=vendor)throw std::runtime_error("cold GPU/SDK exact Z16 selfcheck mismatch");
   for(unsigned a=0;a<3;a++)if(!std::isfinite(actual_xyz[4*i+a])||!std::isfinite(expected_xyz[4*i+a])||std::abs(actual_xyz[4*i+a]-expected_xyz[4*i+a])>1e-7f)throw std::runtime_error("cold GPU XYZ selfcheck mismatch");
  }
  if(native_nonzero<24||aligned_nonzero<24)throw std::runtime_error("insufficient depth coverage for startup GPU test");
  if(gpu.fault||cpu.fault||gpu.component_count>kMaxComponents||cpu.component_count>kMaxComponents||gpu.component_count!=cpu.component_count)throw std::runtime_error("cold GPU compact count/fault mismatch");
  for(unsigned i=0;i<cpu.component_count;i++)if(!(gpu.cpu_components[i]==cpu.cpu_components[i]))throw std::runtime_error("cold GPU compact component mismatch");
  color_=m.frame_number;depth_=m.depth_frame_number;++passes_;
 }catch(...){failed_=true;throw;}
}
void ColdCameraSelfcheck::require_complete()const{
 if(failed_||passes_!=3)throw std::runtime_error("three original-cohort cold selfchecks are required");
}
void ResidentPublicationGate::reserve(const void* owner){
 std::lock_guard<std::mutex> lock(mutex_);
 if(!owner||owner_)throw std::runtime_error("resident measurement stream already has a camera owner");
 owner_=owner;generation_=0;
}
void ResidentPublicationGate::admit(const void* owner,uint64_t generation){
 std::lock_guard<std::mutex> lock(mutex_);
 if(!owner||owner_!=owner||!generation)throw std::runtime_error("resident publication owner/generation mismatch");
 generation_=generation;
}
void ResidentPublicationGate::release(const void* owner){
 std::lock_guard<std::mutex> lock(mutex_);
 if(owner_==owner){generation_=0;owner_=nullptr;}
}
void ResidentPublicationGate::invalidate(const void* owner){
 std::lock_guard<std::mutex> lock(mutex_);
 if(owner_==owner)generation_=0;
}
void require_raw_or_cpu_fallback(const std::string& mode,const std::string& failure){
 if(mode!="auto"&&mode!="vulkan"&&mode!="cpu")throw std::invalid_argument("unknown camera backend");
 if(mode=="vulkan"&&!failure.empty())throw std::runtime_error("strict Vulkan camera admission refused: "+failure);
}
}
