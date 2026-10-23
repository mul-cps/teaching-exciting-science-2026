#pragma once
#include "so101_station_vulkan/pipeline.hpp"
#include <librealsense2/rsutil.h>
namespace so101_station_vulkan {
// Explicitly pinned SDK implementation; SIMD and scalar rasterization differ.
enum class AlignmentConvention : uint32_t { Scalar=0, Sse=1, Unknown=2 };
struct AlignmentCalibration {
 rs2_intrinsics depth{},color{}; rs2_extrinsics depth_to_color{};
 float depth_units=.001f; uint64_t generation=0;
 AlignmentConvention convention=AlignmentConvention::Unknown;
};
void validate_alignment(const AlignmentCalibration&);
bool alignment_uses_corners(const AlignmentCalibration&);
// Caller-owned outputs; no frame-time allocations. Native Z16 minimum wins.
void alignment_reference(const AlignmentCalibration&,const uint16_t*,uint32_t*,float* xyz);
class AlignmentState;
class AlignmentLease {
 friend class Alignment; std::shared_ptr<AlignmentState> state_; unsigned index_=0;
 AlignmentLease(std::shared_ptr<AlignmentState>,unsigned);
public:
 AlignmentLease()=default; AlignmentLease(const AlignmentLease&); AlignmentLease(AlignmentLease&&) noexcept;
 AlignmentLease& operator=(AlignmentLease); ~AlignmentLease(); void swap(AlignmentLease&) noexcept;
 explicit operator bool()const{return bool(state_);}
 uint16_t* upload()const; const Buffer& aligned()const; const Buffer& xyz()const;
 FrameMetadata& metadata()const; VkFence fence()const;
 // Only available for explicitly requested diagnostics after ready().
 const uint16_t* diagnostic_depth()const; const float* diagnostic_xyz()const;
};
class Alignment {
 std::shared_ptr<AlignmentState> state_;
public:
 Alignment(std::shared_ptr<Context>,AlignmentCalibration,const std::string& shader_dir,bool diagnostics=false);
 AlignmentLease acquire(); void submit(const AlignmentLease&); bool ready(const AlignmentLease&)const;
};
}
