#pragma once
#include "so101_station_vulkan/pipeline.hpp"
#include "so101_station_vulkan/parent_graph.hpp"
namespace so101_station_vulkan {
constexpr uint32_t kParentLayoutVersion=1;
enum ParentFault:uint32_t { ParentInvalidMask=1,ParentOverflow=2,ParentNonconvergence=4 };
struct ParentConfig {
 // Any positive U32 threshold is valid, including >N (zero compact records).
 unsigned width=0,height=0,min_parent_pixels=30,record_capacity=kMaxComponents;
 uint64_t calibration_generation=0,generation=0;
 void validate()const;
};
// Full original-color stats. Color ordinal: red, green, blue, yellow. Label is
// full-mask GRANA order BEFORE filtering; area includes invalid-depth pixels.
struct ParentRecord { uint32_t color_ordinal,label_id,min_block_key,parent_pixels,x,y,width,height; };
struct ParentCounts {
 std::array<uint32_t,4> all_parents{};uint32_t record_count=0;
 uint64_t calibration_generation=0,configuration_generation=0;
};
struct ParentStorage;
class ParentLease {
 friend class ParentPreparation;
 std::shared_ptr<ParentStorage> storage_;unsigned index_=0;
 ParentLease(std::shared_ptr<ParentStorage>,unsigned);
public:
 ParentLease()=default;ParentLease(const ParentLease&);ParentLease(ParentLease&&)noexcept;
 ParentLease& operator=(ParentLease);~ParentLease();void swap(ParentLease&)noexcept;
 explicit operator bool()const{return bool(storage_);}unsigned index()const{return index_;}
 // Borrowed const metadata and resident outputs live only through this lease.
 // Consumers retain their own ParentLease copy THROUGH THEIR OWN GPU fence.
 const FrameMetadata& metadata()const;
 // arena fullLabel[N] begins at label_offset_bytes(); stats and roots use
 // the word layout in parents.slang. Label values are local to mask color.
 VkBuffer resident_buffer()const;size_t label_offset_bytes()const;
 // Same 64-byte completion header / eight-U32 records ABI as parents.slang.
 VkBuffer resident_records_buffer()const;
 // Retained Core scratch: ONLY first width*height U32 words are the original
 // one-hot known-color mask (0,1,2,4,8), needed to interpret local label IDs.
 // It includes invalid-depth color pixels. Borrow only through this lease.
 VkBuffer original_mask_buffer()const;
 // Ordered compact output only. Never exposes/maps a dense host label map.
 const ParentRecord* records()const;
 // Validated copies; no borrowed/private pool state or source-slot exemplar.
 FrameLease source_frame()const;ParentConfig configuration()const;
};
// Preparation only: no accepted objects, world coordinates or acquisition
// qualification. Bind to one specific producer control block without reserving
// a source slot or prolonging the producer pool lifetime.
class ParentPreparation {
 std::shared_ptr<ParentStorage> storage_;
public:
 ParentPreparation(std::shared_ptr<Context>,FramePoolToken,const ParentConfig&,const std::string& shader_directory);
 ParentLease submit(const FrameLease&);bool ready(const ParentLease&)const;
 ParentCounts finish(const ParentLease&); // nonblocking; pending/fault throws
 void collect();
};
size_t parent_arena_bytes(unsigned width,unsigned height);
size_t parent_compact_bytes(unsigned capacity);
}
