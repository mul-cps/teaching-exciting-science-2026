// CPU arithmetic cost only: no Vulkan/ROS linkage, dispatch, transfer or station claim.
#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <new>
#include <stdexcept>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
using uint=uint32_t;
#include "../shaders/soft_binary64.slang"
namespace { uint64_t allocations=0; volatile uint64_t sink=0; }
void* operator new(size_t n){if(void* p=std::malloc(n?n:1)){++allocations;return p;}throw std::bad_alloc();}
void* operator new[](size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);} void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,size_t)noexcept{std::free(p);} void operator delete[](void* p,size_t)noexcept{std::free(p);}
uint64_t bits(double d){uint64_t u;std::memcpy(&u,&d,8);return u;}
double value(uint64_t u){double d;std::memcpy(&d,&u,8);return d;}
uint64_t canonical(double d){return std::isnan(d)?0xfff8000000000000ULL:bits(d);}
__attribute__((noinline)) uint64_t soft_div(uint64_t a,uint64_t b){return sf64_div_bits(a,b);}
__attribute__((noinline)) uint64_t soft_sqrt(uint64_t a,uint64_t){return sf64_sqrt_bits(a);}
__attribute__((noinline)) uint64_t native_div(uint64_t a,uint64_t b){volatile double x=value(a),y=value(b);return canonical(x/y);}
__attribute__((noinline)) uint64_t native_sqrt(uint64_t a,uint64_t){volatile double x=value(a);return canonical(std::sqrt(x));}
uint64_t next(uint64_t& s){s^=s<<13;s^=s>>7;s^=s<<17;return s;}
struct Input{uint64_t a,b;};
using Function=uint64_t(*)(uint64_t,uint64_t);
struct Sample{double wall,cpu;uint64_t checksum;};
double now(clockid_t clock){timespec t{};if(clock_gettime(clock,&t))throw std::runtime_error("clock");return double(t.tv_sec)*1e9+t.tv_nsec;}
Sample measure(const std::array<Input,1024>& input,Function fn){
 Function volatile actual=fn;uint64_t checksum=0;const auto before=allocations;
 const double cpu=now(CLOCK_PROCESS_CPUTIME_ID),wall=now(CLOCK_MONOTONIC);
 for(unsigned repeat=0;repeat<32;repeat++)for(const auto& in:input)checksum+=actual(in.a,in.b);
 const double elapsed=now(CLOCK_MONOTONIC)-wall,used=now(CLOCK_PROCESS_CPUTIME_ID)-cpu;
 sink=checksum;if(allocations!=before)throw std::runtime_error("hot heap allocation");
 return {elapsed/32768,used/32768,checksum};
}
void samples(const std::array<Sample,7>& x,bool cpu){for(unsigned i=0;i<7;i++)std::cout<<(i?",":"")<<(cpu?x[i].cpu:x[i].wall);}
double median(const std::array<Sample,7>& x,bool cpu){std::array<double,7> v{};for(unsigned i=0;i<7;i++)v[i]=cpu?x[i].cpu:x[i].wall;std::sort(v.begin(),v.end());return v[3];}
int main(int argc,char** argv){
 if(std::fesetround(FE_TONEAREST))return 2;
#if defined(__SSE__)
 _mm_setcsr(_mm_getcsr()&~((1u<<15)|(1u<<6))); // Native gradual underflow, FTZ/DAZ disabled.
 if(_mm_getcsr()&((1u<<15)|(1u<<6)))return 3;
#endif
 uint64_t seed=argc>1?std::strtoull(argv[1],nullptr,10):20261027;
 constexpr uint64_t special[]={0,0x8000000000000000ULL,0x7ff0000000000000ULL,0xfff0000000000000ULL,0x7ff8000000000001ULL,0x7ff0000000000001ULL,1,0x8000000000000001ULL,0x0010000000000000ULL,0x7fefffffffffffffULL,0x3ff0000000000000ULL,0xbff0000000000000ULL};
 const char* names[]={"ordinary","subnormal","special"};
 std::cout<<"{\"scope\":\"CPU arithmetic only; excludes GPU and station performance\",\"operations_per_trial\":32768,\"warmup_trials\":2,\"trials\":7,\"rounding\":\"nearest ties even; FTZ/DAZ disabled\",\"scenarios\":[";
 for(unsigned category=0;category<3;category++){
  std::array<Input,1024> input{};
  for(auto& in:input){auto generate=[&]() -> uint64_t {auto x=next(seed);if(category==0)return (x&0x800fffffffffffffULL)|((uint64_t(512+next(seed)%1023))<<52);if(category==1)return (x&0x800fffffffffffffULL)|1ULL;return special[next(seed)%(sizeof(special)/8)];};in={generate(),generate()};}
  for(unsigned op=0;op<2;op++){
   // Ordinary/subnormal sqrt costs must exercise the recurrence, not negative-domain early exits.
   if(op && category<2)for(auto& in:input)in.a&=0x7fffffffffffffffULL;
   const Function soft=op?soft_sqrt:soft_div,hard=op?native_sqrt:native_div;
   for(const auto& in:input)if(soft(in.a,in.b)!=hard(in.a,in.b))throw std::runtime_error("IEEE mismatch");
   for(unsigned w=0;w<2;w++){measure(input,soft);measure(input,hard);}
   std::array<Sample,7> a{},b{};
   for(unsigned t=0;t<7;t++){if(t%2){b[t]=measure(input,hard);a[t]=measure(input,soft);}else{a[t]=measure(input,soft);b[t]=measure(input,hard);}if(a[t].checksum!=b[t].checksum)throw std::runtime_error("checksum mismatch");}
   if(category||op)std::cout<<",";
   std::cout<<"{\"population\":\""<<names[category]<<"\",\"operation\":\""<<(op?"sqrt":"div")<<"\",\"hot_heap_allocations\":0,\"input_bytes\":"<<sizeof(input)<<",\"checksum\":"<<a[0].checksum<<",\"software_p50_ns\":"<<median(a,false)<<",\"native_p50_ns\":"<<median(b,false)<<",\"software_cpu_p50_ns\":"<<median(a,true)<<",\"native_cpu_p50_ns\":"<<median(b,true)<<",\"software_samples_ns\":[";samples(a,false);std::cout<<"],\"native_samples_ns\":[";samples(b,false);std::cout<<"],\"software_cpu_samples_ns\":[";samples(a,true);std::cout<<"],\"native_cpu_samples_ns\":[";samples(b,true);std::cout<<"]}";
  }
 }std::cout<<"]}\n";
}
