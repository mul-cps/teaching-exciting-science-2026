#pragma once
#include <cstdint>
#include <cstddef>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <optional>
#include <utility>

namespace so101_station_vulkan {
// A submitted frame is never replaced. Only the newest unsubmitted frame is
// retained; Frame's own lease controls recycling and late-consumer lifetime.
template<class Frame> class LatestFrameQueue {
  Frame active_, pending_;
  bool seen_source_=false;
  uint64_t last_color_=0,last_depth_=0;
public:
  struct Counters { uint64_t received=0,completed=0,replaced=0,no_slot=0,duplicate_or_regressed=0; };
private:
  Counters counts_;
public:
  const Counters& counters()const noexcept{return counts_;}
  const Frame& active()const noexcept{return active_;}
  bool has_pending()const noexcept{return bool(pending_);}
  bool accept_source(uint64_t color,uint64_t depth) noexcept {
    ++counts_.received;
    if(seen_source_&&(color<=last_color_||depth<=last_depth_)){
      ++counts_.duplicate_or_regressed;return false;
    }
    seen_source_=true;last_color_=color;last_depth_=depth;return true;
  }
  void discard_pending(){if(pending_){pending_=Frame{};++counts_.replaced;}}
  void offer(Frame frame){discard_pending();pending_=std::move(frame);}
  void no_slot()noexcept{++counts_.no_slot;}
  // Call only after the backend has verified active completion. The returned
  // lease survives publication and may be copied by downstream consumers.
  Frame take_completed(){Frame result=std::move(active_);active_=Frame{};if(result)++counts_.completed;return result;}
  template<class Submit> void dispatch_pending(Submit&& submit){
    if(!active_&&pending_){active_=std::move(pending_);pending_=Frame{};submit(active_);}
  }
};

// This is host receipt age, not a conversion of the device's hardware clock.
// A backwards ROS-clock jump is unknown, never falsely reported as fresh.
inline std::optional<double> source_received_age_ms(uint64_t received,int64_t now)noexcept{
  if(now<0||uint64_t(now)<received)return std::nullopt;
  return double(uint64_t(now)-received)*1e-6;
}
inline constexpr std::size_t kMeasurementTextCapacity=1024;
template<class Number> void measurement_append_number(std::string& text,Number value){
  char buffer[64];std::to_chars_result result;
  if constexpr(std::is_floating_point<Number>::value){
    if(!std::isfinite(value)){text+="unknown";return;}
    result=std::to_chars(buffer,buffer+sizeof(buffer),value,std::chars_format::general,std::numeric_limits<Number>::max_digits10);
  }else result=std::to_chars(buffer,buffer+sizeof(buffer),value);
  if(result.ec!=std::errc{})throw std::runtime_error("source measurement number formatting failed");
  const auto length=std::size_t(result.ptr-buffer);
  if(text.size()+length>text.capacity())throw std::length_error("source measurement text overflow");
  text.append(buffer,length);
}
// Caller reserves the bounded buffer once. ROS publish(const Message&) retains
// its own snapshot; ROS/DDS copy/serialization allocations are outside this helper.
template<class Metadata> void format_unqualified_measurement(std::string& text,const Metadata& m,unsigned components,int64_t now){
  if(text.capacity()<kMeasurementTextCapacity)throw std::length_error("source measurement buffer was not preallocated");
  text.clear();text+="source_frame=";measurement_append_number(text,m.frame_number);
  text+="; components=";measurement_append_number(text,components);
  text+="; source_received_ros_ns=";measurement_append_number(text,m.timestamp_ns);
  text+="; hardware_timestamp_ms=";measurement_append_number(text,m.hardware_timestamp_ms);
  text+="; depth_source_frame=";measurement_append_number(text,m.depth_frame_number);
  text+="; depth_hardware_timestamp_ms=";measurement_append_number(text,m.depth_hardware_timestamp_ms);
  text+="; timestamp_domain=";measurement_append_number(text,m.timestamp_domain);
  text+="; depth_timestamp_domain=";measurement_append_number(text,m.depth_timestamp_domain);
  text+="; calibration_generation=";measurement_append_number(text,m.calibration_generation);
  text+="; received_age_ms=";const auto age=source_received_age_ms(m.timestamp_ns,now);
  if(age)measurement_append_number(text,*age);else text+="unknown";
  text+="; geometry_admission=unqualified; execution_eligible=false";
}

}
