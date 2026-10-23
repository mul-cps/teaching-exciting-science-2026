#pragma once
#include "so101_station_vulkan/pipeline.hpp"
#include <array>
#include <memory>
#include <vector>
namespace so101_station_vulkan {
// Calibrated once per generation using retained cv::undistortPoints semantics.
// Each entry is (normalized camera x,y); footprint rays intentionally differ.
struct GeometryCalibration {
  unsigned width=0,height=0;
  uint64_t generation=0;
  std::vector<std::array<double,2>> camera_rays;
  std::array<double,12> board_from_camera{}; // row-major affine 3x4
  std::array<double,6> support_roi{{-.03,.21,-.03,.16,-.05,.06}};
  double max_observation_depth_m=1.2,depth_scale_m=.001;
  void validate()const;
};
struct GeometryCounts { uint32_t valid_pixels=0,support_points=0; uint64_t calibration_generation=0; };
struct GeometryStorage;
class GeometryLease {
  friend class GeometryPreparation;
  std::shared_ptr<GeometryStorage> storage_; unsigned index_=0;
  GeometryLease(std::shared_ptr<GeometryStorage>,unsigned);
public:
  GeometryLease()=default; GeometryLease(const GeometryLease&); GeometryLease(GeometryLease&&)noexcept;
  GeometryLease& operator=(GeometryLease); ~GeometryLease(); void swap(GeometryLease&)noexcept;
  explicit operator bool()const{return bool(storage_);}
  // Resident arena is valid until the last lease copy is destroyed. Never map
  // it for the ordinary path. Layout documented in geometry.slang.
  VkBuffer resident_buffer()const;
  unsigned index()const{return index_;}
};
// First geometry gate only: calibrated coordinates, vivid exclusion, valid
// depth and deterministic stride8 support population. NO accepted objects,
// support plane fit, contours or executable detections are produced here.
enum class GeometryPrecision { NativeFloat64, SoftwareBinary64 };
class GeometryPreparation {
  std::shared_ptr<GeometryStorage> storage_;
public:
  GeometryPreparation(std::shared_ptr<Context>, const GeometryCalibration&,const std::string& shader_directory);
  // Explicit preparation-only prototype; directory must contain matching precision shaders.
  GeometryPreparation(std::shared_ptr<Context>, const GeometryCalibration&,const std::string& shader_directory,GeometryPrecision);
  GeometryLease submit(const FrameLease&); // source must already be complete
  bool ready(const GeometryLease&)const;
  GeometryCounts finish(const GeometryLease&); // nonblocking, throws if pending
  // Also releases dropped in-flight leases after their fence completes.
  void collect();
};
size_t geometry_arena_bytes(unsigned width,unsigned height);
}
