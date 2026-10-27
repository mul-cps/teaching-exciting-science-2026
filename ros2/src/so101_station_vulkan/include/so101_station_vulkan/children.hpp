#pragma once
#include "so101_station_vulkan/parents.hpp"
#include "so101_station_vulkan/child_graph.hpp"
namespace so101_station_vulkan {
struct ChildConfig {
 uint64_t generation=0;double max_observation_depth_m=1.2,max_neighbor_jump_m=.012;
 // Standalone splitter permits >=8; full analyzer caller supplies max(24,parent_min).
 // Initial supported packed budget:0auto=4N, or1..4N; larger budgets need new qualification.
 unsigned min_child_pixels=24,record_capacity=4096,packed_pixel_capacity=0;
 void validate(unsigned width,unsigned height,unsigned parent_capacity)const;
};
struct ParentChildStatus {uint32_t valid_depth_pixels,candidate_pixels,all_children,kept_children;};
// candidate_pixels is POST-cut, valid_depth_pixels is original-parent & valid.
// bbox and halo coordinates are parent-local; root is opaque min source pixel+1.
struct ChildRecord {uint32_t root,parent_record_index,key,region_pixels,x,y,width,height,left,top,halo_width,halo_height,packed_offset,depth_cut_pixels,support_pixels,reserved;};
struct ChildCounts {uint32_t parent_count=0,record_count=0,packed_pixels=0,jobs=0;uint64_t calibration_generation=0,parent_generation=0,configuration_generation=0;};
enum ChildFault:uint32_t {ChildInvalidInput=1,ChildRecordOverflow=2,ChildNonconvergence=4,ChildPackedOverflow=8,ChildJobOverflow=16,ChildFillNonconvergence=32,ChildInvalidResult=64};
struct ChildResidentView {FrameLease source;ParentLease parent;ParentConfig parent_config;ChildConfig configuration;ChildCounts counts;VkBuffer roots=VK_NULL_HANDLE,records=VK_NULL_HANDLE;};
struct ChildStorage;
class ChildLease {
 friend class ChildPreparation;std::shared_ptr<ChildStorage> storage_;unsigned index_=0;
 ChildLease(std::shared_ptr<ChildStorage>,unsigned);
public:
 ChildLease()=default;ChildLease(const ChildLease&);ChildLease(ChildLease&&)noexcept;
 ChildLease& operator=(ChildLease);~ChildLease();void swap(ChildLease&)noexcept;
 explicit operator bool()const{return bool(storage_);}unsigned index()const{return index_;}
 ChildResidentView resident_view()const;
 const FrameMetadata& metadata()const;ParentLease parent()const;
 // All computed views validate completion/faults. Borrow only while this lease
 // survives; downstream consumers keep their copy THROUGH THEIR OWN fence.
 VkBuffer source_root_buffer()const; // root[N] at0, source flags[N] at4*N
 size_t source_flags_offset_bytes()const;
 VkBuffer packed_flags_buffer()const; // U32 ChildPackedFlag words, compact offsets
 VkBuffer resident_records_buffer()const;
 const ChildRecord* records()const;const ParentChildStatus* parent_status()const;
};
class ChildPreparation {
 std::shared_ptr<ChildStorage> storage_;
public:
 ChildPreparation(std::shared_ptr<Context>,FramePoolToken,const ParentConfig&,const ChildConfig&,const std::string& shader_directory);
 ChildLease submit(const ParentLease&);bool ready(const ChildLease&)const;ChildCounts finish(const ChildLease&);void collect();
};
// Internal shared encoder for a later literal-fixture diagnostic adapter. It
// accepts already-bound buffers/validated layout, never bypasses production
// ParentLease admission. Same actual SPVs/graph are used by both paths.
namespace child_detail {
struct Parameters {uint32_t width,height,count,blocks,parentCapacity,capacity,packedCapacity,jobCapacity,keyBits,groups,maxBits,jumpBits,minChild,phase,radixShift,radixSide,calLo,calHi,parentLo,parentHi,childLo,childHi,maxLo,maxHi,jumpLo,jumpHi,draftWords,lookupCount,parentLabelWords,reserved[3];};
constexpr unsigned kernel_count=21;
extern const std::array<const char*,kernel_count> kernel_names;
void record_graph(VkCommandBuffer,VkPipelineLayout,const std::array<VkPipeline,kernel_count>&,VkDescriptorSet,const Parameters&,VkBuffer lookup,VkBuffer compact,VkBuffer readback,size_t compact_bytes,VkBuffer arguments);
}
}
