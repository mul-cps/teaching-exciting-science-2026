#pragma once
#include "pipeline.hpp"
#include <array>
#include <memory>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
}
namespace so101_station_vulkan {
// These are qualification INPUTS for the exact adapter/driver/codec/profile.
// Capability enumeration never sets the qualified bits.
struct VideoQualification {
 bool vulkan_available=false,vulkan_qualified=false;
 bool vaapi_available=false,vaapi_qualified=false,vaapi_same_device_interop=false;
 std::string evidence;
};
struct VideoSelection {std::string backend,reason;};
std::string video_profile_key(const std::shared_ptr<Context>&,unsigned width,unsigned height,unsigned fps);
VideoQualification load_video_qualification(const std::string& path,const std::string& expected_profile_key,std::shared_ptr<Context> current_context={});
VideoSelection select_video_backend(const std::string&,const VideoQualification&);
class SharedVideoDevice {
 struct Impl;std::unique_ptr<Impl> impl_;
public:
 static std::string compatible(const std::shared_ptr<Context>&);
 explicit SharedVideoDevice(std::shared_ptr<Context>);
 ~SharedVideoDevice(); SharedVideoDevice(const SharedVideoDevice&)=delete;
 AVBufferRef* ref()const;std::shared_ptr<Context> context()const;
};
// Single producer/single consumer; caller serializes calls. Three reusable input
// frames. EAGAIN is backpressure, never implicit allocation of another frame.
// RGB is copied from a completed RGB-D lease; depth is never read or modified.
class VideoEncoder {
 struct Impl;std::unique_ptr<Impl> impl_;
public:
 VideoEncoder(unsigned width,unsigned height,unsigned fps,const VideoSelection&,
              std::shared_ptr<SharedVideoDevice> device={});
 ~VideoEncoder();VideoEncoder(const VideoEncoder&)=delete;
 int submit_rgb(const FrameLease&,int64_t pts);
 int receive(AVPacket*);int flush();const std::string& backend()const;
};
// Normal runtime admission. Probe code uses VideoEncoder directly; station
// callers use this facade so saved evidence cannot replace live capability.
class VideoSession {
 VideoSelection selection_;std::unique_ptr<VideoEncoder> encoder_;
public:
 VideoSession(unsigned width,unsigned height,unsigned fps,const std::string& mode,
              const std::string& qualification_path,std::shared_ptr<Context> context={});
 VideoEncoder& encoder(){return *encoder_;}
 const VideoSelection& selection()const{return selection_;}
};
class VideoDecoder {
 struct Impl;std::unique_ptr<Impl> impl_;
public:
 VideoDecoder(const VideoSelection&,std::shared_ptr<SharedVideoDevice> device={});
 ~VideoDecoder();VideoDecoder(const VideoDecoder&)=delete;
 int send(const AVPacket*);int receive(AVFrame*);
};
}
