#pragma once
#include "so101_station_vulkan/alignment.hpp"
#include "so101_station_vulkan/latest_frame_queue.hpp"

namespace so101_station_vulkan {
struct RawFrame {
 FrameLease core;
 AlignmentLease depth;
 explicit operator bool()const{return bool(core);}
};
// Serialized by the camera callback group. Pending pairs have never been
// submitted; active retains the actual source slot until consumer validation.
class RawCaptureBridge {
 std::unique_ptr<Alignment> depth_;
 LatestFrameQueue<RawFrame> queue_;
 std::unique_ptr<Pipeline> core_;
 bool failed_=false;
 void validate(const RawFrame&)const;
 void require_healthy()const;
public:
 using Counters=LatestFrameQueue<RawFrame>::Counters;
 RawCaptureBridge(std::unique_ptr<Pipeline>,std::unique_ptr<Alignment> depth=nullptr);
 ~RawCaptureBridge();
 RawFrame acquire();
 void discard_pending(){queue_.discard_pending();}
 void no_slot(){queue_.no_slot();}
 bool accept_source(uint64_t color,uint64_t depth){require_healthy();return queue_.accept_source(color,depth);}
 void offer(RawFrame);
 void dispatch_pending();
 bool collect(RawFrame& completed);
 const Counters& counters()const{return queue_.counters();}
 const RawFrame& active()const{return queue_.active();}
 Pipeline& pipeline(){return *core_;}
 bool gpu_alignment()const{return bool(depth_);}
 Calibration expected_calibration()const;
};
}
