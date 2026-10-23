#include "so101_station_vulkan/pipeline.hpp"
#include <iostream>
#include <stdexcept>
using namespace so101_station_vulkan;
#define REQUIRE(x) do{if(!(x))throw std::runtime_error(#x);}while(0)
int main(){try{
 auto pool=std::make_shared<FramePool>(8,8,nullptr,false);
 auto a=pool->acquire(),b=pool->acquire(),c=pool->acquire();REQUIRE(a&&b&&c);REQUIRE(!pool->acquire());
 auto late_encoder=a;auto old_index=a.index();a={};REQUIRE(!pool->acquire());late_encoder={};auto replacement=pool->acquire();REQUIRE(replacement.index()==old_index);
 FrameLease pending=b; b={};REQUIRE(!pool->acquire());pending={};REQUIRE(pool->acquire());
 REQUIRE(c.slot().metadata.width==8&&c.slot().metadata.height==8);
 auto foreign=std::make_shared<FramePool>(8,8,nullptr,false)->acquire();
 bool rejected=false;try{pool->ready(foreign);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 Pipeline cpu(8,8,"cpu","");auto unsubmitted=cpu.acquire();rejected=false;try{cpu.finish(unsubmitted);}catch(const std::logic_error&){rejected=true;}REQUIRE(rejected);
 rejected=false;try{cpu.submit(foreign);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 auto invalid=cpu.acquire();invalid.slot().calibration.fx=0;rejected=false;try{cpu.submit(invalid);}catch(const std::invalid_argument&){rejected=true;}REQUIRE(rejected);
 Slot faulted;std::array<uint32_t,4> header{{0,4,0,0}};faulted.readback.mapped=header.data();rejected=false;try{decode_compact(faulted);}catch(const std::runtime_error&){rejected=true;}REQUIRE(rejected);
 cpu.submit(unsubmitted);cpu.finish(unsubmitted);rejected=false;try{cpu.submit(unsubmitted);}catch(const std::logic_error&){rejected=true;}REQUIRE(rejected);
 auto survivor=c;pool.reset();c={};REQUIRE(survivor.slot().references==1);survivor={};
 // 4096 distinct 24-pixel pieces followed by a large overflow piece. The
 // root count must never be consumed as a compact-array index on refusal.
 auto crowded=std::make_shared<FramePool>(512,224,nullptr,false);auto cf=crowded->acquire();auto& cs=cf.slot();auto* input=static_cast<uint8_t*>(cs.upload.mapped);constexpr unsigned pixels=512*224;
 const uint8_t colors[4][3]={{255,0,0},{0,255,0},{0,0,255},{255,255,0}};
 for(unsigned y=0;y<224;y++)for(unsigned x=0;x<512;x++){unsigned i=y*512+x;if(y<192){unsigned color=(x/4)%2+2*((y/6)%2);for(unsigned k=0;k<3;k++)input[3*i+k]=colors[color][k];}else if(y>=194&&y<204)input[3*i]=255;input[3*pixels+2*i]=500&255;input[3*pixels+2*i+1]=500>>8;}
 cpu_reference(cs,512,224);REQUIRE(cs.fault==2);REQUIRE(cs.component_count==kMaxComponents);for(unsigned i=0;i<kMaxComponents;i++)REQUIRE(cs.cpu_components[i].count==24);
 std::cout<<"three slots, late encoder/consumer, pending replacement, pool lifetime: PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
