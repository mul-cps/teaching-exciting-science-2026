#include "so101_station_vulkan/pipeline.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <atomic>
#include <cstdlib>
#include <new>
#include <stdexcept>
using namespace so101_station_vulkan;
static std::atomic<size_t> allocations{0};
void* operator new(std::size_t n){++allocations;if(void* p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p) noexcept{std::free(p);}
void operator delete[](void* p) noexcept{std::free(p);}
void operator delete(void* p,std::size_t) noexcept{std::free(p);}
void operator delete[](void* p,std::size_t) noexcept{std::free(p);}

void require(bool b){if(!b)throw std::runtime_error("native Slang CPU equivalence failure");}
int main(){try{
#ifndef RESIDENT_SLANG_CPU
 bool rejected=false;try{Pipeline p(7,5,"cpu-slang","");}catch(const std::invalid_argument& e){rejected=std::string(e.what()).find("not built")!=std::string::npos;}require(rejected);
#else
 for(auto dims:{std::pair<unsigned,unsigned>{7,5},{64,48},{512,224}}){auto w=dims.first,h=dims.second;size_t n=size_t(w)*h;
 Pipeline a(w,h,"cpu",""),b(w,h,"cpu-slang","");require(!b.context()&&b.backend()=="cpu-slang");
 for(unsigned fixture=0;fixture<6;fixture++){
  std::array<FrameLease,kSlots> aa,bb;
  for(unsigned slot=0;slot<kSlots;slot++){
   aa[slot]=a.acquire();bb[slot]=b.acquire();require(bool(aa[slot])&&bool(bb[slot]));auto& x=aa[slot].slot();auto& y=bb[slot].slot();auto* in=static_cast<uint8_t*>(x.upload.mapped);std::memset(in,0,x.upload.size);
   uint32_t random=12345+slot;
   for(size_t i=0;i<n;i++){auto px=i%w,py=i/w;random=random*1664525u+1013904223u;unsigned z=500;auto* rgb=in+3*i;
    if(fixture==1)rgb[0]=255;
    if(fixture==2){rgb[(px/8)%3]=255;z=(px+py)%17==0?0:(px>w/2?520:500);}
    if(fixture==3){rgb[0]=random>>24;rgb[1]=random>>16;rgb[2]=random>>8;z=random%1500;}
    if(fixture==4){rgb[0]=255;z=i%2?100:1200;}
    if(fixture==5){
     if(py<192){const uint8_t colors[4][3]={{255,0,0},{0,255,0},{0,0,255},{255,255,0}};unsigned color=(px/4)%2+2*((py/6)%2);for(unsigned k=0;k<3;k++)rgb[k]=colors[color][k];}
     else if(py>=194&&py<204)rgb[0]=255;
    }
    in[3*n+2*i]=z;in[3*n+2*i+1]=z>>8;
   }
   x.calibration.fx=fixture==4?.001f:603.25f;x.calibration.fy=fixture==4?1e7f:599.1f;x.calibration.cx=-31.75f;x.calibration.cy=999.25f;y.calibration=x.calibration;std::memcpy(y.upload.mapped,in,x.upload.size);
   const auto before=allocations.load();a.submit(aa[slot]);b.submit(bb[slot]);require(allocations.load()==before);require(x.fault==y.fault);
   if(fixture==5&&w==512){require(x.fault==2&&x.component_count==kMaxComponents);}require(x.cpu_masks==y.cpu_masks&&x.cpu_depth==y.cpu_depth&&x.cpu_parents==y.cpu_parents);require(x.component_count==y.component_count);
   for(size_t i=0;i<4*n;i++)require(std::abs(x.cpu_xyz[i]-y.cpu_xyz[i])<=1e-7f);require(x.cpu_morph_counts==y.cpu_morph_counts);
   for(size_t i=0;i<n;i++)for(unsigned j=0;j<x.cpu_morph_counts[i];j++)require(x.cpu_morph_roots[9*i+j]==y.cpu_morph_roots[9*i+j]);
   for(unsigned i=0;i<x.component_count;i++)require(x.cpu_components[i]==y.cpu_components[i]);
   if(!x.fault){a.finish(aa[slot]);b.finish(bb[slot]);}else{for(auto pair:{std::pair<Pipeline*,FrameLease*>{&a,&aa[slot]},{&b,&bb[slot]}}){bool rejected=false;try{pair.first->finish(*pair.second);}catch(const std::runtime_error&){rejected=true;}require(rejected);}}
  }
  require(!a.acquire()&&!b.acquire());
 }
 }
#endif
 std::cout<<"native Slang CPU contract passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
