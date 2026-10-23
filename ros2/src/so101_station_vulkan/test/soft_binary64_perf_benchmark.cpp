// CPU-only shared shader arithmetic benchmark. No Vulkan linkage or GPU activity.
#include <algorithm>
#include <array>
#include <cfenv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>
#include <vector>
using uint=uint32_t;
#include "../shaders/soft_binary64.slang"
// Generated verbatim from geometry_soft.slang by test_soft_binary64_perf.py.
#include "soft_binary64_geometry_expr.hpp"
namespace {uint64_t allocations=0; volatile uint64_t sink=0;}
void* operator new(size_t n){if(void* p=std::malloc(n?n:1)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);} void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);} void operator delete[](void* p,size_t)noexcept{std::free(p);}
uint64_t bits(double d){uint64_t u;std::memcpy(&u,&d,8);return u;}
double value(uint64_t u){double d;std::memcpy(&d,&u,8);return d;}
uint64_t next(uint64_t& s){s^=s<<13;s^=s>>7;s^=s<<17;return s;}
struct Input { ShaderExpression shader; uint32_t depth; };
std::array<uint64_t,3> native(const Input& in){
 const auto& c=in.shader.calibration;float f;std::memcpy(&f,&in.depth,4);
 volatile double z=double(f),x=value(c[0])*z,y=value(c[1])*z;
 std::array<uint64_t,3> out{};
 for(uint row=0;row<3;row++){
  uint base=2+4*row;volatile double v=value(c[base])*x;
  volatile double t=value(c[base+1])*y;v=v+t;
  t=value(c[base+2])*z;v=v+t;v=v+value(c[base+3]);out[row]=bits(v);
 }return out;
}
std::array<uint64_t,3> software(const Input& in){return in.shader.coordinate(in.depth);}
template<class F>double measure(const std::vector<Input>& inputs,F fn){
 uint64_t checksum=0;auto before=allocations;
 auto start=std::chrono::steady_clock::now();
 for(unsigned repeat=0;repeat<16;repeat++)for(const auto& in:inputs){auto out=fn(in);checksum+=out[0]+out[1]+out[2];}
 auto end=std::chrono::steady_clock::now();sink=checksum;
 if(allocations!=before)throw std::runtime_error("hot math allocated");
 return std::chrono::duration<double,std::nano>(end-start).count()/(16*inputs.size());
}
int main(int argc,char** argv){
 if(std::fesetround(FE_TONEAREST))return 2;
 uint64_t seed=argc>1?std::strtoull(argv[1],nullptr,10):20261023;
 constexpr const char* names[]={"random","axis_aligned_calibration","rotated_calibration"};
 std::cout<<"{\"scope\":\"CPU shared integer helper and actual shader coordinate expression; excludes GPU FPS, dispatch, transfer and driver\",\"trials\":7,\"coordinates_per_trial\":65536,\"mul_per_coordinate\":11,\"add_per_coordinate\":9,\"scenarios\":[";
 for(unsigned scenario=0;scenario<3;scenario++){
  std::vector<Input> inputs(4096);
  for(auto& in:inputs){
   auto random=[&](){return double(int64_t(next(seed)%2000001)-1000000)/1000000.;};
   for(auto& c:in.shader.calibration)c=bits(random());
   float z=float(.1+std::abs(random()));std::memcpy(&in.depth,&z,4);
   if(scenario){
    // Runtime calibration values and float32 depth: compiler cannot fold them.
    double a=random()*.5,cs=std::cos(a),sn=std::sin(a);
    double m[]={1,0,0,.01,0,1,0,-.02,0,0,1,.03};
    if(scenario==2){
     double pitch=random()*.5,roll=random()*.5,cp=std::cos(pitch),sp=std::sin(pitch),cr=std::cos(roll),sr=std::sin(roll);
     m[0]=cs*cp;m[1]=cs*sp*sr-sn*cr;m[2]=cs*sp*cr+sn*sr;
     m[4]=sn*cp;m[5]=sn*sp*sr+cs*cr;m[6]=sn*sp*cr-cs*sr;
     m[8]=-sp;m[9]=cp*sr;m[10]=cp*cr;
    }
    for(unsigned k=0;k<12;k++)in.shader.calibration[k+2]=bits(m[k]);
   }
   if(software(in)!=native(in))throw std::runtime_error("coordinate mismatch");
  }
  std::array<double,7> soft{},hard{};
  for(unsigned t=0;t<7;t++){if(t%2){hard[t]=measure(inputs,native);soft[t]=measure(inputs,software);}else{soft[t]=measure(inputs,software);hard[t]=measure(inputs,native);}}
  auto sorted_soft=soft,sorted_hard=hard;std::sort(sorted_soft.begin(),sorted_soft.end());std::sort(sorted_hard.begin(),sorted_hard.end());
  if(scenario)std::cout<<",";
  std::cout<<"{\"name\":\""<<names[scenario]<<"\",\"software_median_ns\":"<<sorted_soft[3]<<",\"native_ordered_median_ns\":"<<sorted_hard[3]<<",\"hot_heap_allocations\":0,\"input_bytes\":"<<inputs.size()*sizeof(Input)<<",\"software_samples_ns\":[";
  for(unsigned t=0;t<7;t++)std::cout<<(t?",":"")<<soft[t];std::cout<<"],\"native_samples_ns\":[";
  for(unsigned t=0;t<7;t++)std::cout<<(t?",":"")<<hard[t];std::cout<<"]}";
 }std::cout<<"]}\n";
}
