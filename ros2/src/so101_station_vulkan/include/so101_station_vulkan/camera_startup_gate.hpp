#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace so101_station_vulkan {

struct CameraSourceObservation {
  uint64_t color_frame, depth_frame;
  double color_time_ms, depth_time_ms;
  int color_domain, depth_domain;
};

// Cold admission only. Normal operation must retain its stricter failure policy.
class CameraStartupGate {
  uint64_t start_ns_;
  mutable uint64_t last_now_ns_;
  uint64_t stable_start_ns_ = 0;
  CameraSourceObservation previous_{};
  unsigned advancing_count_ = 0;
  bool have_baseline_ = false;

public:
  explicit CameraStartupGate(uint64_t start_ns)
      : start_ns_(start_ns), last_now_ns_(start_ns) {}

  void check_deadline(uint64_t now_ns) const {
    if (now_ns < start_ns_ || now_ns < last_now_ns_)
      throw std::runtime_error("camera startup monotonic clock regressed");
    if (now_ns - start_ns_ >= 10000000000ULL)
      throw std::runtime_error("camera startup source stability deadline");
    last_now_ns_ = now_ns;
  }

  bool observe(const CameraSourceObservation& current, uint64_t now_ns) {
    check_deadline(now_ns);
    if (!std::isfinite(current.color_time_ms) ||
        !std::isfinite(current.depth_time_ms))
      throw std::runtime_error("camera startup source timestamp is nonfinite");
    if (!current.color_frame || !current.depth_frame) {
      have_baseline_ = false;
      advancing_count_ = 0;
      return false;
    }
    const bool advancing =
        have_baseline_ && current.color_frame > previous_.color_frame &&
        current.depth_frame > previous_.depth_frame &&
        current.color_time_ms >= previous_.color_time_ms &&
        current.depth_time_ms >= previous_.depth_time_ms &&
        current.color_domain == previous_.color_domain &&
        current.depth_domain == previous_.depth_domain;
    previous_ = current;
    if (!advancing) {
      have_baseline_ = true;
      stable_start_ns_ = now_ns;
      advancing_count_ = 1;
      return false;
    }
    if (advancing_count_ < 5) ++advancing_count_;
    return now_ns - start_ns_ >= 2000000000ULL &&
           now_ns - stable_start_ns_ >= 500000000ULL && advancing_count_ >= 5;
  }
};

}  // namespace so101_station_vulkan
