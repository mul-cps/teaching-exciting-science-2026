#include "so101_station_vulkan/alignment.hpp"
#include <librealsense2/hpp/rs_internal.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
constexpr size_t N=640*480;
std::vector<uint16_t> sdk_align(const AlignmentCalibration& k,std::vector<uint16_t>& raw){
 rs2::software_device device;auto ds=device.add_sensor("Depth"),cs=device.add_sensor("Color");
 auto depth=ds.add_video_stream({RS2_STREAM_DEPTH,0,1,640,480,30,2,RS2_FORMAT_Z16,k.depth});
 auto color=cs.add_video_stream({RS2_STREAM_COLOR,0,2,640,480,30,3,RS2_FORMAT_RGB8,k.color});
 depth.register_extrinsics_to(color,k.depth_to_color);device.create_matcher(RS2_MATCHER_DEFAULT);ds.add_read_only_option(RS2_OPTION_DEPTH_UNITS,k.depth_units);
 rs2::syncer sync;ds.open(depth);cs.open(color);ds.start(sync);cs.start(sync);std::vector<uint8_t> rgb(N*3);
 rs2::align align(RS2_STREAM_COLOR);std::vector<uint16_t> output;
 for(int f=1;f<=5;f++){
  ds.on_video_frame({raw.data(),[](void*){},640*2,2,f*1000.0/30,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,f,depth,k.depth_units});
  cs.on_video_frame({rgb.data(),[](void*){},640*3,3,f*1000.0/30,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,f,color});
  auto frames=sync.wait_for_frames(1000);if(!frames.get_depth_frame()||!frames.get_color_frame())continue;
  auto result=align.process(frames).as<rs2::frameset>().get_depth_frame();
  if(result.get_frame_number()!=uint64_t(f)||result.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)throw std::runtime_error("SDK changed depth frame provenance");
  output.resize(N);std::memcpy(output.data(),result.get_data(),N*2);
 }
 ds.stop();cs.stop();ds.close();cs.close();if(output.empty())throw std::runtime_error("software device did not synchronize");return output;
}
void rejection_tests(){
 AlignmentCalibration k;k.depth={640,480,319,239,600,600,RS2_DISTORTION_NONE,{0,0,0,0,0}};k.color=k.depth;k.depth_to_color={{1,0,0,0,1,0,0,0,1},{0,0,0}};k.convention=AlignmentConvention::Scalar;
 auto reject=[&](AlignmentCalibration bad){bool threw=false;try{validate_alignment(bad);}catch(const std::invalid_argument&){threw=true;}if(!threw)throw std::runtime_error("invalid calibration accepted");};
 auto bad=k;bad.convention=AlignmentConvention::Unknown;reject(bad);bad=k;bad.depth.coeffs[0]=.01f;reject(bad);bad=k;bad.depth_units=0;reject(bad);bad=k;bad.depth_to_color.translation[0]=NAN;reject(bad);bad=k;bad.color.fx=INFINITY;reject(bad);bad=k;bad.depth.width=641;reject(bad);
 std::vector<uint16_t> raw(N);raw[239*640+319]=1000;std::vector<uint32_t> out(N);std::vector<float> xyz(N*4);
 k.depth.fx=k.depth.fy=10;alignment_reference(k,raw.data(),out.data(),xyz.data());
 k.depth.fx=k.depth.fy=9;bool fault=false;try{alignment_reference(k,raw.data(),out.data(),xyz.data());}catch(const std::runtime_error&){fault=true;}if(!fault)throw std::runtime_error("unbounded footprint was accepted");
 k.depth.fx=k.depth.fy=600;k.depth_to_color.translation[0]=1e30f;alignment_reference(k,raw.data(),out.data(),xyz.data());if(std::any_of(out.begin(),out.end(),[](auto d){return d!=0;}))throw std::runtime_error("overflow projection fabricated depth");
 std::cout<<"PASS calibration_rejections=6 footprint_boundary=pass overflow_invalid=pass\n";
}
int main(int argc,char** argv)try{
 rejection_tests();bool gpu=argc>1&&std::string(argv[1])=="--vulkan";unsigned cases=0;double max_xyz=0;
 for(unsigned fixture=0;fixture<8;fixture++){
  AlignmentCalibration k;k.depth={640,480,319.5f,239.5f,fixture%2?390.f:600.f,fixture%2?390.f:600.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};k.color={640,480,319.7f,239.2f,610.f,605.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};k.depth_to_color={ {1,0,0,0,1,0,0,0,1},{fixture>1?.0234f:0.f,fixture>2?-.0027f:0.f,fixture>3?.001f:0.f}};k.convention=AlignmentConvention::Scalar;k.generation=17;k.depth_units=fixture==5?.00025f:.001f;
  std::vector<uint16_t> raw(N);uint32_t seed=0x452331ab;for(size_t i=0;i<N;i++){seed=1664525*seed+1013904223;raw[i]=i%7==0?0:uint16_t(250+(seed%4250));}if(fixture==0)std::fill(raw.begin(),raw.end(),1000);if(fixture==4)for(size_t i=0;i<N;i++)raw[i]=i%101?0:65535;
  if(fixture==6)std::fill(raw.begin(),raw.end(),0);if(fixture==7){std::fill(raw.begin(),raw.end(),0);for(unsigned i:{0u,639u,306560u,307199u,153919u})raw[i]=1000;}
  auto oracle=sdk_align(k,raw);std::vector<uint32_t> reference(N);std::vector<float> xyz(N*4);
  size_t mismatch=0;alignment_reference(k,raw.data(),reference.data(),xyz.data());for(size_t i=0;i<N;i++)mismatch+=reference[i]!=oracle[i];
  if(mismatch)throw std::runtime_error("Pinned scalar CPU SDK convention mismatch case="+std::to_string(fixture)+" pixels="+std::to_string(mismatch));
  std::cout<<"case="<<fixture<<" convention="<<uint32_t(k.convention)<<" corners="<<alignment_uses_corners(k)<<" cpu_sdk_mismatch=0\n";
  // Independent SDK aligned-pixel deprojection; metric tolerance unchanged 1um.
  for(size_t i=0;i<N;i++){float pixel[2]{float(i%640),float(i/640)},point[3];rs2_deproject_pixel_to_point(point,&k.color,pixel,oracle[i]*k.depth_units);for(unsigned a=0;a<3;a++)max_xyz=std::max(max_xyz,double(std::abs(point[a]-xyz[4*i+a])));}
  if(gpu){Alignment pipeline(Context::shared(),k,ALIGNMENT_SHADER_DIR,true);auto frame=pipeline.acquire();std::memcpy(frame.upload(),raw.data(),N*2);frame.metadata().frame_number=91;frame.metadata().depth_frame_number=90;frame.metadata().timestamp_ns=12345678;pipeline.submit(frame);auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);while(!pipeline.ready(frame)){if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("GPU alignment deadline");std::this_thread::yield();}mismatch=0;auto aligned=frame.diagnostic_depth();auto actual_xyz=frame.diagnostic_xyz();for(size_t i=0;i<N;i++){mismatch+=aligned[i]!=oracle[i];for(unsigned a=0;a<3;a++)max_xyz=std::max(max_xyz,double(std::abs(xyz[i*4+a]-actual_xyz[i*4+a])));}if(mismatch)throw std::runtime_error("GPU SDK mismatch case="+std::to_string(fixture)+" pixels="+std::to_string(mismatch));if(frame.metadata().timestamp_ns!=12345678||frame.metadata().depth_frame_number!=90||frame.metadata().calibration_generation!=17)throw std::runtime_error("frame provenance lost");auto b=pipeline.acquire(),c=pipeline.acquire();if(!b||!c||pipeline.acquire())throw std::runtime_error("alignment pool bound");auto retained=frame;frame={};if(pipeline.acquire())throw std::runtime_error("copied lease lost ownership");}
  cases++;
 }
 if(max_xyz>1e-6)throw std::runtime_error("metric deprojection exceeds 1um");std::cout<<"PASS cases="<<cases<<" max_xyz_error_m="<<max_xyz<<" gpu="<<gpu<<" physical=false\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
