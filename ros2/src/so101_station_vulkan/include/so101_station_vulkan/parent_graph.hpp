#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
namespace so101_station_vulkan {
// Pure integer ABI shared by cold host layout checks and the CPU graph witness.
// parents.slang uses these exact word offsets. Zero means absent; node zero is
// encoded as 1. Bounds count all four colors, including odd clipped blocks.
struct ParentGraphLayout {
 uint32_t width,height,count,block_width,blocks,nodes,scan_groups;
 explicit ParentGraphLayout(unsigned w,unsigned h){
  if(!w||!h||w>640||h>480)throw std::invalid_argument("parents require dimensions within 640x480");
  width=w;height=h;count=w*h;block_width=(w+1)/2;blocks=block_width*((h+1)/2);nodes=4*blocks;scan_groups=(blocks+255)/256;
 }
 size_t arena_bytes()const{return size_t(10*nodes+count+8*scan_groups)*4;}
 size_t label_offset()const{return size_t(10)*nodes*4;}
};
// Current block vs left/up/up-left/up-right predecessor. Bits are raster
// [0,1;2,3]. Every possible 2x2 same-color occupancy is itself 8-connected.
inline bool parent_block_edge(uint32_t current,uint32_t predecessor,unsigned direction){
 switch(direction){
 case 0:return (current&5)&&(predecessor&10);
 case 1:return (current&3)&&(predecessor&12);
 case 2:return (current&1)&&(predecessor&8);
 case 3:return (current&2)&&(predecessor&4);
 default:throw std::invalid_argument("invalid parent edge direction");
 }
}
}
