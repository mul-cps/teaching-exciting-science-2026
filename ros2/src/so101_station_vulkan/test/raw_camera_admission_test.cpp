#include "so101_station_vulkan/raw_camera_admission.hpp"
#undef NDEBUG
#include <cassert>
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <sys/mman.h>
#include <unistd.h>
using namespace so101_station_vulkan;
// Every qualified depth plane is page-sized. A protected following page makes
// any cold-verifier read beyond native depth deterministic, including 1080p RGB.
class GuardedNativeDepth {
 uint16_t* data_;size_t bytes_,page_;
public:
 explicit GuardedNativeDepth(size_t n):bytes_(2*n),page_(size_t(sysconf(_SC_PAGESIZE))){
  assert(bytes_%page_==0);void* mapped=mmap(nullptr,bytes_+page_,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0);assert(mapped!=MAP_FAILED);data_=static_cast<uint16_t*>(mapped);
  assert(mprotect(reinterpret_cast<uint8_t*>(data_)+bytes_,page_,PROT_NONE)==0);
 }
 ~GuardedNativeDepth(){munmap(data_,bytes_+page_);}
 uint16_t* data(){return data_;}uint16_t& operator[](size_t i){return data_[i];}
};
int main(){
 CameraSourceCalibration s;s.color={2,0,640,480,30,RS2_FORMAT_RGB8,RS2_STREAM_COLOR};s.depth={1,0,640,480,30,RS2_FORMAT_Z16,RS2_STREAM_DEPTH};
 s.alignment.depth={640,480,319.5f,239.5f,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};s.alignment.color=s.alignment.depth;
 s.alignment.depth_to_color={{1,0,0,0,1,0,0,0,1},{0,0,0}};s.alignment.convention=AlignmentConvention::Scalar;
 SdkAlignmentProbe good{3,true,true,{}};assert(raw_admission_failure(good,s).empty());
 auto bad=s;bad.depth.fps=15;assert(!raw_admission_failure(good,bad).empty());
 bad=s;bad.alignment.color.coeffs[0]=.1f;assert(!raw_admission_failure(good,bad).empty());
 bad=s;bad.alignment.convention=AlignmentConvention::Sse;assert(!raw_admission_failure(good,bad).empty());
 for(SdkAlignmentProbe probe: {SdkAlignmentProbe{},SdkAlignmentProbe{3,false,true,{}},SdkAlignmentProbe{3,true,false,{}},SdkAlignmentProbe{2,true,true,{}}})assert(!raw_admission_failure(probe,s).empty());
 auto generation=camera_generation(s,"fake",1);assert(generation&&generation==camera_generation(s,"fake",1));assert(generation!=camera_generation(s,"fake",2));assert(generation!=camera_generation(s,"other",1));
 bad=s;bad.color.unique_id++;assert(!same_camera_source(s,bad));assert(generation!=camera_generation(bad,"fake",1));
 bad=s;bad.alignment.depth_units=.00025f;assert(!same_camera_source(s,bad));
 bad=s;bad.alignment.depth_to_color.rotation[0]=.9f;assert(!same_camera_source(s,bad));
 assert(same_camera_source(s,s));
 require_raw_or_cpu_fallback("auto","probe failed");require_raw_or_cpu_fallback("cpu","probe failed");bool rejected=false;try{require_raw_or_cpu_fallback("vulkan","probe failed");}catch(const std::runtime_error&){rejected=true;}assert(rejected);
 // Procedural cold-live fakes exercise the real admission verifier, never GPU.
 constexpr unsigned n=640*480;
 std::vector<uint8_t> sdk(n*2);std::vector<uint16_t> native(n),aligned(n);std::vector<uint32_t> reference(n);std::vector<float> xyz(n*4),actual_xyz(n*4);
 auto positive=[&]{
  std::fill(sdk.begin(),sdk.end(),0);std::fill(native.begin(),native.end(),0);std::fill(aligned.begin(),aligned.end(),0);std::fill(reference.begin(),reference.end(),0);std::fill(xyz.begin(),xyz.end(),0);std::fill(actual_xyz.begin(),actual_xyz.end(),0);
  for(unsigned i=0;i<24;i++){native[i]=aligned[i]=reference[i]=1000;sdk[2*i]=0xe8;sdk[2*i+1]=3;xyz[4*i+2]=actual_xyz[4*i+2]=1;}
 };positive();
 Slot cpu,gpu;FrameMetadata m;m.width=640;m.height=480;m.calibration_generation=generation;m.serial="fake";m.frame_number=1;m.depth_frame_number=2;
 auto verify=[&](ColdCameraSelfcheck& gate){gate.verify(m,native.data(),sdk.data(),aligned.data(),reference.data(),xyz.data(),actual_xyz.data(),cpu,gpu);};
 auto rejects=[&](auto mutation){
  ColdCameraSelfcheck gate(generation,"fake");mutation();bool threw=false;try{verify(gate);}catch(const std::exception&){threw=true;}assert(threw);assert(gate.passes()==0);
  // Failure latches even if input buffers/source metadata are repaired.
  positive();gpu.fault=0;gpu.component_count=0;cpu.component_count=0;m.calibration_generation=generation;m.frame_number=1;m.depth_frame_number=2;m.serial="fake";
  threw=false;try{verify(gate);}catch(const std::exception&){threw=true;}assert(threw);
  threw=false;try{gate.require_complete();}catch(const std::exception&){threw=true;}assert(threw);
 };
 rejects([&]{std::fill(native.begin(),native.end(),0);});
 rejects([&]{std::fill(sdk.begin(),sdk.end(),0);std::fill(aligned.begin(),aligned.end(),0);std::fill(reference.begin(),reference.end(),0);std::fill(xyz.begin(),xyz.end(),0);std::fill(actual_xyz.begin(),actual_xyz.end(),0);});
 rejects([&]{native[23]=0;});
 rejects([&]{aligned[17]=1;});rejects([&]{reference[17]=1;});
 rejects([&]{actual_xyz[0]=std::numeric_limits<float>::quiet_NaN();});
 rejects([&]{actual_xyz[1]=std::numeric_limits<float>::infinity();});
 rejects([&]{actual_xyz[0]=1.01e-7f;});rejects([&]{gpu.fault=1;});
 rejects([&]{gpu.component_count=1;});
 rejects([&]{gpu.component_count=cpu.component_count=1;gpu.cpu_components[0].root=2;cpu.cpu_components[0].root=1;});
 rejects([&]{gpu.component_count=cpu.component_count=kMaxComponents+1;});rejects([&]{m.calibration_generation++;});
 rejects([&]{m.serial="other";});rejects([&]{m.frame_number=0;});
 ColdCameraSelfcheck gate(generation,"fake");
 bool incomplete=false;try{gate.require_complete();}catch(const std::exception&){incomplete=true;}assert(incomplete);
 actual_xyz[0]=1e-7f; // Existing acceptance threshold is inclusive.
 for(unsigned pass=0;pass<3;pass++){m.frame_number=pass+1;m.depth_frame_number=pass+2;verify(gate);assert(gate.passes()==pass+1);}
 gate.require_complete();
 ColdCameraSelfcheck duplicate(generation,"fake");verify(duplicate);bool duplicated=false;try{verify(duplicate);}catch(const std::exception&){duplicated=true;}assert(duplicated);
 // A real mutex orders in-progress publication before UNKNOWN, then blocks
 // queued old generations. No Vulkan/ROS objects are involved in this race test.
 ResidentPublicationGate publication;int owner1=0,owner2=0;publication.reserve(&owner1);
 bool competing=false;try{publication.reserve(&owner2);}catch(const std::exception&){competing=true;}assert(competing);
 assert(!publication.publish(0,[]{assert(false);}));publication.admit(&owner1,17);
 std::mutex barrier_mutex;std::condition_variable barrier;bool entered=false,release_publish=false;
 std::atomic<bool> invalidating{false};std::atomic<unsigned> order{0};unsigned success_order=0,unknown_order=0;
 std::thread publishing([&]{assert(publication.publish(17,[&]{
  std::unique_lock<std::mutex> lock(barrier_mutex);entered=true;barrier.notify_all();barrier.wait(lock,[&]{return release_publish;});success_order=++order;
 }));});
 {std::unique_lock<std::mutex> lock(barrier_mutex);barrier.wait(lock,[&]{return entered;});}
 std::thread failing([&]{invalidating=true;publication.invalidate(&owner1,[&]{unknown_order=++order;});});
 while(!invalidating.load())std::this_thread::yield();assert(order==0);
 {std::lock_guard<std::mutex> lock(barrier_mutex);release_publish=true;}barrier.notify_all();
 publishing.join();failing.join();assert(success_order==1&&unknown_order==2);
 assert(!publication.publish(17,[]{assert(false);}));publication.release(&owner1);
 publication.reserve(&owner2);publication.admit(&owner2,18);
 publication.invalidate(&owner1,[]{assert(false);});publication.release(&owner1);
 assert(!publication.publish(17,[]{assert(false);}));assert(publication.publish(18,[]{}));publication.release(&owner2);
 {ResidentCameraOwner ownership(publication,&owner1);} // Constructor unwinding cleanup is RAII.
 publication.reserve(&owner2);publication.release(&owner2);
 // Admit exactly the selected native pairs, retaining stream identity and rates.
 for(auto dimensions:{std::array<int,4>{640,480,640,480},{1280,720,1280,720},{1920,1080,1280,720}}){
  auto high=s;high.color.width=dimensions[0];high.color.height=dimensions[1];high.depth.width=dimensions[2];high.depth.height=dimensions[3];
  high.alignment.color.width=high.color.width;high.alignment.color.height=high.color.height;
  high.alignment.depth.width=high.depth.width;high.alignment.depth.height=high.depth.height;
  validate_camera_profiles(high);assert(raw_admission_failure(good,high).empty());
  auto invalid=[&](auto change){auto bad=high;change(bad);bool threw=false;try{validate_camera_profiles(bad);}catch(const std::invalid_argument&){threw=true;}assert(threw);};
  invalid([](auto& x){x.color.fps=15;});invalid([](auto& x){x.depth.fps=15;});
  invalid([](auto& x){x.color.unique_id=0;});invalid([](auto& x){x.depth.format=RS2_FORMAT_RGB8;});
  invalid([](auto& x){x.color.stream=RS2_STREAM_DEPTH;});
  invalid([](auto& x){x.depth.width=640;x.depth.height=720;});
  const unsigned color_n=unsigned(high.color.width)*high.color.height,depth_n=unsigned(high.depth.width)*high.depth.height;
  GuardedNativeDepth raw(depth_n);std::vector<uint16_t> aligned(color_n);std::vector<uint8_t> vendor(2*color_n);
  std::vector<uint32_t> reference(color_n);std::vector<float> xyz(4*color_n);
  // The native coverage and aligned coverage are deliberately at opposite tails.
  for(unsigned i=0;i<24;i++){raw[depth_n-1-i]=1000;aligned[color_n-1-i]=reference[color_n-1-i]=1000;vendor[2*(color_n-1-i)]=0xe8;vendor[2*(color_n-1-i)+1]=3;}
  FrameMetadata m;m.width=high.color.width;m.height=high.color.height;m.serial="fake";m.calibration_generation=generation;
  ColdCameraSelfcheck gate(generation,"fake",m.width,m.height,high.depth.width,high.depth.height);
  auto verify=[&](ColdCameraSelfcheck& g){g.verify(m,raw.data(),vendor.data(),aligned.data(),reference.data(),xyz.data(),xyz.data(),cpu,gpu);};
  for(unsigned pass=1;pass<=3;pass++){m.frame_number=pass;m.depth_frame_number=pass+9;verify(gate);}gate.require_complete();
  auto rejected=[&](auto mutation){ColdCameraSelfcheck g(generation,"fake",m.width,m.height,high.depth.width,high.depth.height);mutation();bool threw=false;try{verify(g);}catch(const std::exception&){threw=true;}assert(threw);};
  rejected([&]{m.calibration_generation++;});m.calibration_generation=generation;
  rejected([&]{aligned.back()=1;});aligned.back()=1000;
  rejected([&]{raw[depth_n-24]=0;});raw[depth_n-24]=1000;
  rejected([&]{m.width--;});m.width=high.color.width;
  auto decoded=camera_capture_profile(high.color.width==640?"vga":high.color.width==1280?"720p":"1080p");
  assert(decoded.color_width==unsigned(high.color.width)&&decoded.color_height==unsigned(high.color.height)&&decoded.depth_width==unsigned(high.depth.width)&&decoded.depth_height==unsigned(high.depth.height)&&decoded.fps==30);
  assert(camera_generation(high,"fake",1)!=camera_generation(high,"fake",2));
 }
 for(auto pair:{std::array<int,4>{1920,1080,640,480},{1280,720,640,480},{640,480,1280,720}}){auto bad=s;bad.color.width=pair[0];bad.color.height=pair[1];bad.depth.width=pair[2];bad.depth.height=pair[3];bool rejected=false;try{validate_camera_profiles(bad);}catch(const std::exception&){rejected=true;}assert(rejected);}
 bool invalid_name=false;try{camera_capture_profile("custom");}catch(const std::invalid_argument&){invalid_name=true;}assert(invalid_name);
 std::cout<<"PASS high_resolution_profiles cold_native_depth_coverage\n";
 // Actual loaded SDK software frames: no Context, no USB, no physical PASS.
 auto actual=probe_loaded_sdk_alignment();assert(actual.fixtures==3);assert(actual.admitted());
 std::cout<<"PASS source_identity profile_calibration generation SDK_scalar_probe fallback strict physical=false\n";
}
