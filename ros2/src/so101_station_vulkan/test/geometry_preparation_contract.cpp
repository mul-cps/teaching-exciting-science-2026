// Host-only contract tests. Never create a Vulkan context or enumerate devices.
#include "so101_station_vulkan/geometry.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace so101_station_vulkan;
template<class F> void invalid(F f){bool rejected=false;try{f();}catch(const std::invalid_argument&){rejected=true;}assert(rejected);}
int main(){
 GeometryCalibration c;c.width=640;c.height=480;c.generation=17;c.camera_rays.resize(640*480);c.board_from_camera={1,0,0,0,0,1,0,0,0,0,1,0};c.validate();
 assert(geometry_arena_bytes(640,480)==8755352);
 assert(geometry_arena_bytes(1,1)==68);
 invalid([&]{geometry_arena_bytes(641,1);});invalid([&]{geometry_arena_bytes(1,481);});invalid([&]{geometry_arena_bytes(0,480);});
 c.camera_rays[0][0]=std::numeric_limits<double>::quiet_NaN();invalid([&]{c.validate();});c.camera_rays[0][0]=0;
 c.support_roi[0]=c.support_roi[1];invalid([&]{c.validate();});c.support_roi[0]=-.03;
 c.generation=0;invalid([&]{c.validate();});c.generation=17;
 c.camera_rays.pop_back();invalid([&]{c.validate();});c.camera_rays.push_back({0,0});
 c.depth_scale_m=std::numeric_limits<double>::denorm_min();invalid([&]{c.validate();});c.depth_scale_m=.001;
 invalid([&]{GeometryPreparation p(nullptr,c,"unopened");});
 GeometryLease empty,copy=empty;GeometryLease moved=std::move(copy);assert(!empty&&!moved);
 bool threw=false;try{empty.resident_buffer();}catch(const std::logic_error&){threw=true;}assert(threw);
 std::cout<<"geometry host-only validation, bounded arena, empty lease: PASS\n";
}
