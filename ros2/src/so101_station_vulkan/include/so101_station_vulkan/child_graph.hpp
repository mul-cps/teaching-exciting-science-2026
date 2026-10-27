#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
namespace so101_station_vulkan {
// Pure cold/layout/predicate helpers, also compiled by the CPU graph proof.
// The same word ABI is used by children.slang; no real CCL numeric IDs exposed.
inline uint64_t child_double_bits(double x){uint64_t bits;std::memcpy(&bits,&x,8);return bits;}
inline uint32_t child_f32_rne_bits(double x){
 auto bits=child_double_bits(x);uint32_t sign=uint32_t(bits>>63)<<31;unsigned exponent=unsigned((bits>>52)&2047);uint64_t fraction=bits&0xfffffffffffffull;
 if(exponent==2047)return sign|0x7f800000u|(fraction?0x400000u:0);
 if(!exponent)return sign;
 int e=int(exponent)-1023;uint64_t sig=fraction|(uint64_t(1)<<52);
 auto rounded=[&](unsigned shift){if(shift>53)return uint64_t(0);uint64_t q=sig>>shift,rem=sig&((uint64_t(1)<<shift)-1),half=uint64_t(1)<<(shift-1);return q+uint64_t(rem>half||(rem==half&&(q&1)));};
 if(e>127)return sign|0x7f800000u;
 if(e<-126){auto mant=rounded(unsigned(-e-97));return sign|uint32_t(mant);}
 auto mant=rounded(29);if(mant==(uint64_t(1)<<24)){mant>>=1;++e;}
 return e>127?sign|0x7f800000u:sign|(uint32_t(e+127)<<23)|(uint32_t(mant)&0x7fffffu);
}
inline float child_float(uint32_t bits){float x;std::memcpy(&x,&bits,4);return x;}
inline bool child_depth_valid(uint32_t depth_bits,uint32_t maximum_bits){float z=child_float(depth_bits);return std::isfinite(z)&&z>=child_float(0x3dcccccdu)&&z<=child_float(maximum_bits);}
inline bool child_bad_edge(uint32_t left_bits,uint32_t right_bits,uint32_t jump_bits){return std::fabs(child_float(left_bits)-child_float(right_bits))>child_float(jump_bits);}
inline uint32_t child_local_key(unsigned x,unsigned y,unsigned parent_x,unsigned parent_y,unsigned parent_width){return ((y-parent_y)/2)*((parent_width+1)/2)+(x-parent_x)/2;}
inline bool child_same_parent(uint32_t a,uint32_t b){return a!=0&&a==b;}
enum ChildSourceFlag:uint32_t {ChildDepthValid=1,ChildDepthCut=2};
enum ChildPackedFlag:uint32_t {ChildRegion=1,ChildOriginalParent=2,ChildNearby=4,ChildPackedCut=8,ChildSupport=16};
struct ChildGraphLayout {
 uint32_t width,height,count,blocks,parent_capacity,record_capacity,packed_capacity,job_capacity,groups,key_bits,lookup_count;
 size_t draft_offset,workspace_bytes,persistent_bytes,lookup_bytes,packed_bytes,jobs_bytes,pairs_bytes,compact_bytes;
 ChildGraphLayout(unsigned w,unsigned h,unsigned parents=4096,unsigned records=4096,unsigned packed=0){
  if(!w||!h||w>2048||h>2048||uint64_t(w)*h>307200||!parents||parents>4096||!records||records>4096)throw std::invalid_argument("child graph dimensions/capacities invalid");
  width=w;height=h;count=w*h;blocks=((w+1)/2)*((h+1)/2);parent_capacity=parents;record_capacity=records;packed_capacity=packed?packed:4*count;
  if(packed_capacity>4*count)throw std::invalid_argument("child packed capacity exceeds explicit four-frame budget");
  job_capacity=records+(packed_capacity+255)/256;groups=(records+255)/256;lookup_count=4*(blocks+1);key_bits=0;while((uint32_t(1)<<key_bits)<blocks)++key_bits;
  unsigned parent_bits=0;while((uint32_t(1)<<parent_bits)<parents)++parent_bits;
  if(key_bits+parent_bits>32)throw std::invalid_argument("child composite key does not fit U32");
  // Histogram/offset scratch aliases dead graph stats, never live draft records.
  draft_offset=std::max(size_t(36)*count,size_t(128)*groups);workspace_bytes=std::max(draft_offset+size_t(64)*records,size_t(12)*packed_capacity);
  persistent_bytes=size_t(8)*count;lookup_bytes=size_t(4)*lookup_count;packed_bytes=size_t(4)*packed_capacity;jobs_bytes=size_t(16)*job_capacity;pairs_bytes=size_t(16)*records;
  compact_bytes=128+size_t(16)*parents+size_t(64)*records;
 }
 size_t device_bytes()const{return workspace_bytes+persistent_bytes+lookup_bytes+packed_bytes+jobs_bytes+pairs_bytes+compact_bytes+64;}
 size_t readback_bytes()const{return compact_bytes;}
 size_t cache_bytes()const{return size_t(16)*parent_capacity+size_t(64)*record_capacity;}
 size_t total_bytes()const{return device_bytes()+readback_bytes()+cache_bytes();}
};
}
