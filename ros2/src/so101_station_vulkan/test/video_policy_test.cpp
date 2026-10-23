#include "so101_station_vulkan/video.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
using namespace so101_station_vulkan;
int main(){
 auto require=[](bool v){if(!v)throw std::runtime_error("video policy assertion");};
 VideoQualification q;q.evidence="test-profile-digest";
 auto p=select_video_backend("auto",q);require(p.backend=="cpu" && !p.reason.empty());
 q.vulkan_available=true;p=select_video_backend("auto",q);require(p.backend=="cpu");
 q.vaapi_available=true;q.vaapi_qualified=true;p=select_video_backend("auto",q);require(p.backend=="cpu");
 q.vaapi_same_device_interop=true;p=select_video_backend("auto",q);require(p.backend=="vaapi");
 q.vulkan_qualified=true;p=select_video_backend("auto",q);require(p.backend=="vulkan");
 p=select_video_backend("cpu",q);require(p.backend=="cpu");
 q.vulkan_qualified=false;bool refused=false;try{select_video_backend("vulkan",q);}catch(const std::runtime_error&){refused=true;}require(refused);
 bool invalid=false;try{select_video_backend("magic",q);}catch(const std::invalid_argument&){invalid=true;}require(invalid);
 require(!SharedVideoDevice::compatible(nullptr).empty());
 const std::string path="/tmp/resident-video-qualification-test-"+std::to_string(::getpid())+".json";
 {std::ofstream f(path);f << R"({"schema":"so101-video-qualification-v1","profile_key":"different","backend":"vulkan","encode_frames":60,"cpu_decode_frames":60,"hardware_decode_frames":60,"same_decoded_pixels":true,"rgb_correct":true})";}
 require(!load_video_qualification(path,"expected").vulkan_qualified);
 {nlohmann::json record={{"schema","so101-video-qualification-v1"},{"profile_key","expected"},{"backend","vulkan"},{"encode_frames",60},{"cpu_decode_frames",60},{"hardware_decode_frames",60},{"same_decoded_pixels",true},{"rgb_correct",true},{"passed",true},{"cpu_frame_hashes",std::vector<uint64_t>(60,123)},{"hardware_frame_hashes",std::vector<uint64_t>(60,123)}};std::vector<double> means;for(int i=0;i<60;i++){int r=30+(i%10)*10;means.push_back(((66*r+129*90+25*150+128)>>8)+16);}record["cpu_luma_means"]=means;std::ofstream f(path);f<<record;}
 require(load_video_qualification(path,"expected").vulkan_qualified);require(!load_video_qualification(path,"expected").vulkan_available);
 VideoSession session(64,48,30,"auto",path);require(session.selection().backend=="cpu");
 {std::ofstream f(path);f << "{}";}require(!load_video_qualification(path,"expected").vulkan_qualified);std::remove(path.c_str());
 std::cout << "video policy and bounded admission PASS\n";
}
