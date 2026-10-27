#pragma once
#include "so101_station_vulkan/children.hpp"
#include "so101_station_vulkan/geometry.hpp"
#include <optional>
namespace so101_station_vulkan {
// Preparation only. ROI is an observation gate, NEVER a clip or object decision.
struct ObservedConfig {
 uint64_t generation=0;
 std::optional<std::array<double,6>> observation_roi;
 unsigned closed_membership_capacity=0; // 0=>4N, admitted resource budget.
 void validate(unsigned count)const;
};
struct ObservedLayout {
 uint32_t width,height,count,capacity,parent_capacity,closed_capacity;
 size_t arena_bytes,compact_bytes;
 ObservedLayout(unsigned w,unsigned h,unsigned records=4096,unsigned parents=4096,unsigned closed=0){
  if(!w||!h||w>2048||h>2048||uint64_t(w)*h>307200||!records||records>4096||!parents||parents>4096)throw std::invalid_argument("observed child dimensions/capacities invalid");
  width=w;height=h;count=w*h;capacity=records;parent_capacity=parents;closed_capacity=closed?closed:4*count;
  if(closed_capacity>4*count)throw std::invalid_argument("observed membership capacity exceeds admitted4N budget");
  arena_bytes=size_t(48)*count;compact_bytes=64+size_t(80)*capacity;
 }
};
enum ObservedFlag:uint32_t {ObservationEligible=1,ClosedComputed=2};
enum ObservedFault:uint32_t {ObservedInvalidInput=1,ObservedInvalidRecord=2,ObservedMembershipOverflow=4,ObservedInvalidResult=8};
struct ObservedRecord {ChildRecord child;uint32_t roi_pixels,closed_pixels,flags,reserved;};
struct ObservedCounts {uint32_t record_count=0,eligible_count=0,original_pixels=0,closed_memberships=0;uint64_t calibration_generation=0,child_generation=0,configuration_generation=0;};
struct ObservedStorage;
class ObservedLease {
 friend class ObservedPreparation;std::shared_ptr<ObservedStorage> storage_;unsigned index_=0;
 ObservedLease(std::shared_ptr<ObservedStorage>,unsigned);
public:
 ObservedLease()=default;ObservedLease(const ObservedLease&);ObservedLease(ObservedLease&&)noexcept;
 ObservedLease& operator=(ObservedLease);~ObservedLease();void swap(ObservedLease&)noexcept;
 explicit operator bool()const{return bool(storage_);}unsigned index()const{return index_;}
 // Borrow only through this lease. Arena U32 layout: lookup[N], originalID[N],
 // closedCount[N], closedIDs[9N]. IDs are ordered record index+1, 0=none.
 VkBuffer resident_buffer()const;const ObservedRecord* records()const;
 ChildLease children()const;GeometryLease geometry()const;
};
class ObservedPreparation {
 std::shared_ptr<ObservedStorage> storage_;
public:
 ObservedPreparation(std::shared_ptr<Context>,FramePoolToken,GeometryProducerToken,const ParentConfig&,const ChildConfig&,const ObservedConfig&,const std::string& shader_directory);
 ObservedLease submit(const ChildLease&,const GeometryLease&);
 bool ready(const ObservedLease&)const;ObservedCounts finish(const ObservedLease&);void collect();
};
namespace observed_detail {
struct Parameters {uint32_t width,height,count,capacity,parentCapacity,closedCapacity,roiEnabled,calLo,calHi,parentLo,parentHi,childLo,childHi,observedLo,observedHi,reserved;};
constexpr unsigned kernel_count=6;
extern const std::array<const char*,kernel_count> kernel_names;
void record_graph(VkCommandBuffer,VkPipelineLayout,const std::array<VkPipeline,kernel_count>&,VkDescriptorSet,const Parameters&,VkBuffer compact,VkBuffer readback,size_t compact_bytes);
}
}
