#include "so101_station_vulkan/pipeline.hpp"
#include <cstddef>
// Slang 2026.19 direct LLVM host ABI. ByteAddressBuffer count is bytes;
// StructuredBuffer count is elements. No GPU scratch storage is bound/read.
namespace so101_station_vulkan {
namespace {
struct View {void* data;size_t count;};
struct Parameters {uint32_t width,height,count,capacity;float scale,fx,fy,cx,cy,maxDepth,jump;uint32_t round;};
struct Globals {View input,scratch,divisions,compact;Parameters* p;View masks,depth,xyz,parents;};
struct Grid {uint32_t start[3],end[3];};
static_assert(sizeof(void*)==8&&sizeof(size_t)==8,"Slang CPU ABI requires 64-bit target");
static_assert(sizeof(View)==16&&alignof(View)==8&&sizeof(Parameters)==48&&sizeof(Grid)==24);
static_assert(offsetof(Parameters,scale)==16&&offsetof(Parameters,round)==44);
static_assert(offsetof(Globals,p)==64&&offsetof(Globals,masks)==72&&offsetof(Globals,depth)==88&&offsetof(Globals,xyz)==104&&offsetof(Globals,parents)==120&&sizeof(Globals)==136&&alignof(Globals)==8);
extern "C" void cpu_convert(Grid*,void*,void*);
extern "C" void cpu_candidates(Grid*,void*,void*);
}
void cpu_slang(Slot& s,unsigned w,unsigned h){
 const size_t n=size_t(w)*h;auto& c=s.calibration;auto& t=cpu_divisions();
 Parameters p{w,h,uint32_t(n),kMaxComponents,c.depth_scale,c.fx,c.fy,c.cx,c.cy,1.2f,.012f,0};
 Globals g{{s.upload.mapped,s.upload.size},{nullptr,0},{const_cast<int*>(t.data()),t.size()},{nullptr,0},&p,{s.cpu_masks.data(),n},{s.cpu_depth.data(),n},{s.cpu_xyz.data(),4*n},{s.cpu_parents.data(),n}};
 Grid grid{{0,0,0},{1,1,1}};
 cpu_convert(&grid,nullptr,&g);cpu_candidates(&grid,nullptr,&g);
 cpu_complete_reference(s,w,h);
}
}
