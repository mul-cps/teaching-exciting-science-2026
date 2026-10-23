#include "so101_station_vulkan/video.hpp"
#include "video_conversion.hpp"
extern "C" {
#include <libavutil/hwcontext_vulkan.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cerrno>
#include <stdexcept>
namespace so101_station_vulkan {
namespace {
void avcheck(int result,const char* where){if(result<0){char error[AV_ERROR_MAX_STRING_SIZE];av_strerror(result,error,sizeof(error));throw std::runtime_error(std::string(where)+": "+error);}}
struct DeviceLifetime {std::shared_ptr<Context> owner;std::vector<const char*> extensions;};
void lock_queue(AVHWDeviceContext* d,uint32_t,uint32_t){static_cast<DeviceLifetime*>(d->user_opaque)->owner->queue_mutex.lock();}
void unlock_queue(AVHWDeviceContext* d,uint32_t,uint32_t){static_cast<DeviceLifetime*>(d->user_opaque)->owner->queue_mutex.unlock();}
void free_device(AVHWDeviceContext* d){delete static_cast<DeviceLifetime*>(d->user_opaque);}
}
std::string video_profile_key(const std::shared_ptr<Context>& context,unsigned width,unsigned height,unsigned fps){
 std::ostringstream key;key<<"h264-main-nv12-no-bframes|"<<width<<"x"<<height<<"@"<<fps<<"|avcodec="<<avcodec_version()<<"|avutil="<<avutil_version()<<"|native="<<VIDEO_NATIVE_SOURCE_HASH<<"|shader="<<VIDEO_SHADER_SOURCE_HASH;
 if(context){VkPhysicalDeviceIDProperties id{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES};VkPhysicalDeviceDriverProperties driver{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES};id.pNext=&driver;VkPhysicalDeviceProperties2 props{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};props.pNext=&id;vkGetPhysicalDeviceProperties2(context->physical,&props);key<<"|uuid=";for(auto byte:id.deviceUUID)key<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);key<<std::dec<<"|vendor="<<props.properties.vendorID<<"|device="<<props.properties.deviceID<<"|driver="<<props.properties.driverVersion<<"|driver-id="<<driver.driverID<<"|driver-info="<<driver.driverInfo<<"|api="<<props.properties.apiVersion;}
 if(context){key<<"|video="<<context->video_capable;for(const auto& e:context->enabled_extensions)key<<"|ext="<<e;for(const auto& q:context->video_queues)key<<"|q="<<q.family<<":"<<q.flags<<":"<<q.codecs;key<<"|timeline="<<context->enabled12.timelineSemaphore<<"|sync2="<<context->enabled13.synchronization2;}else key<<"|cpu";return key.str();
}
VideoQualification load_video_qualification(const std::string& path,const std::string& expected,std::shared_ptr<Context> current_context){
 VideoQualification q;q.evidence="qualification record missing, mismatched or incomplete";
 try{std::ifstream file(path,std::ios::binary|std::ios::ate);if(!file||file.tellg()<1||file.tellg()>65536)return q;file.seekg(0);auto j=nlohmann::json::parse(file);if(j.at("schema")!="so101-video-qualification-v1"||j.at("profile_key")!=expected||j.at("backend")!="vulkan")return q;
 if(!j.at("passed").get<bool>()||j.contains("error")||j.at("encode_frames").get<unsigned>()!=60||j.at("cpu_decode_frames")!=j.at("encode_frames")||j.at("hardware_decode_frames")!=j.at("encode_frames")||!j.at("same_decoded_pixels").get<bool>()||!j.at("rgb_correct").get<bool>())return q;
 auto cpu=j.at("cpu_frame_hashes").get<std::vector<uint64_t>>();auto hardware=j.at("hardware_frame_hashes").get<std::vector<uint64_t>>();if(cpu.size()!=60||cpu!=hardware)return q;
 auto luma=j.at("cpu_luma_means").get<std::vector<double>>();if(luma.size()!=60)return q;for(unsigned i=0;i<60;i++){int r=30+(i%10)*10;int expected_y=((66*r+129*90+25*150+128)>>8)+16;if(!std::isfinite(luma[i])||std::abs(luma[i]-expected_y)>8)return q;}
 q.vulkan_available=SharedVideoDevice::compatible(current_context).empty();if(current_context){VkVideoCodecOperationFlagsKHR operations=0;for(auto& queue:current_context->video_queues)operations|=queue.codecs;q.vulkan_available&=(operations&VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR)&&(operations&VK_VIDEO_CODEC_OPERATION_DECODE_H264_BIT_KHR);}
 q.vulkan_qualified=true;q.evidence=path;
 }catch(const std::exception&){}return q;
}
VideoSelection select_video_backend(const std::string& mode,const VideoQualification& q){
 if(mode!="auto"&&mode!="vulkan"&&mode!="cpu")throw std::invalid_argument("video mode must be auto, vulkan or cpu");
 if(mode=="cpu")return {"cpu","explicit CPU comparison"};
 if(q.vulkan_available&&q.vulkan_qualified&&!q.evidence.empty())return {"vulkan","qualified shared-device H.264 profile: "+q.evidence};
 const std::string reason="Vulkan H.264 unavailable or exact profile unqualified";
 if(mode=="vulkan")throw std::runtime_error(reason);
 if(q.vaapi_available&&q.vaapi_qualified&&q.vaapi_same_device_interop&&!q.evidence.empty())return {"vaapi",reason+"; qualified same-device VAAPI interoperability: "+q.evidence};
 return {"cpu",reason+"; qualified same-device VAAPI interoperability unavailable"};
}
struct SharedVideoDevice::Impl {std::shared_ptr<Context> context;AVBufferRef* device=nullptr;~Impl(){av_buffer_unref(&device);}};
std::string SharedVideoDevice::compatible(const std::shared_ptr<Context>& c){if(!c)return "shared Vulkan Context absent";if(!c->video_capable)return c->video_reason.empty()?"shared device video features not enabled":c->video_reason;if(c->instance_version<VK_API_VERSION_1_3)return "FFmpeg requires Vulkan instance 1.3";return {};}
SharedVideoDevice::SharedVideoDevice(std::shared_ptr<Context> c):impl_(new Impl){auto why=compatible(c);if(!why.empty())throw std::runtime_error(why);impl_->context=std::move(c);impl_->device=av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_VULKAN);if(!impl_->device)throw std::bad_alloc();
 auto* d=reinterpret_cast<AVHWDeviceContext*>(impl_->device->data);auto* v=static_cast<AVVulkanDeviceContext*>(d->hwctx);auto life=std::make_unique<DeviceLifetime>();life->owner=impl_->context;for(auto& name:life->owner->enabled_extensions)life->extensions.push_back(name.c_str());
 v->get_proc_addr=vkGetInstanceProcAddr;v->inst=life->owner->instance;v->phys_dev=life->owner->physical;v->act_dev=life->owner->device;v->device_features=life->owner->enabled_features;v->enabled_dev_extensions=life->extensions.data();v->nb_enabled_dev_extensions=life->extensions.size();
 for(auto& q:life->owner->video_queues){if(v->nb_qf>=64)throw std::runtime_error("too many video queue families");auto& out=v->qf[v->nb_qf++];out.idx=q.family;out.num=1;out.flags=static_cast<VkQueueFlagBits>(q.flags);out.video_caps=static_cast<VkVideoCodecOperationFlagBitsKHR>(q.codecs);}
#if FF_API_VULKAN_SYNC_QUEUES
 v->lock_queue=lock_queue;v->unlock_queue=unlock_queue;
#else
 #error "Shared queue locking requires FFmpeg queue callbacks or internally synchronized queues"
#endif
 d->user_opaque=life.release();d->free=free_device;avcheck(av_hwdevice_ctx_init(impl_->device),"initialize borrowed Vulkan device");
}
SharedVideoDevice::~SharedVideoDevice()=default;
AVBufferRef* SharedVideoDevice::ref()const{return impl_->device;}
std::shared_ptr<Context> SharedVideoDevice::context()const{return impl_->context;}
struct VideoEncoder::Impl {
 AVCodecContext* codec=nullptr;AVBufferRef* frames=nullptr;SwsContext* converter=nullptr;
 std::array<AVFrame*,kSlots> cpu{},gpu{};std::shared_ptr<SharedVideoDevice> device;
 VkVideoEncodeH264ProfileInfoKHR h264_profile{VK_STRUCTURE_TYPE_VIDEO_ENCODE_H264_PROFILE_INFO_KHR};
 VkVideoProfileInfoKHR video_profile{VK_STRUCTURE_TYPE_VIDEO_PROFILE_INFO_KHR};
 VkVideoProfileListInfoKHR profile_list{VK_STRUCTURE_TYPE_VIDEO_PROFILE_LIST_INFO_KHR};
 std::unique_ptr<VideoConversion> conversion;
 unsigned width,height;std::string backend;bool flushed=false;
 ~Impl(){conversion.reset();avcodec_free_context(&codec);for(auto*& f:gpu)av_frame_free(&f);for(auto*& f:cpu)av_frame_free(&f);av_buffer_unref(&frames);sws_freeContext(converter);}
};
VideoEncoder::VideoEncoder(unsigned w,unsigned h,unsigned fps,const VideoSelection& selected,std::shared_ptr<SharedVideoDevice> device):impl_(new Impl){
 if(!w||!h||w>640||h>480||w%2||h%2||!fps||fps>60)throw std::invalid_argument("video requires even RGB dimensions <=640x480 and 1..60 fps");
 auto& p=*impl_;p.width=w;p.height=h;p.backend=selected.backend;if(p.backend!="cpu")p.device=std::move(device);
 if(p.backend!="cpu"&&p.backend!="vulkan")throw std::runtime_error("VAAPI shared-device mapping not qualified/implemented; select CPU fallback");
 if(p.backend=="vulkan"&&!p.device)throw std::invalid_argument("Vulkan encoding requires resident shared device");
 const AVCodec* codec=avcodec_find_encoder_by_name(p.backend=="vulkan"?"h264_vulkan":"libx264");if(!codec)throw std::runtime_error("selected H.264 encoder unavailable");p.codec=avcodec_alloc_context3(codec);if(!p.codec)throw std::bad_alloc();p.codec->width=w;p.codec->height=h;p.codec->time_base={1,int(fps)};p.codec->framerate={int(fps),1};p.codec->pix_fmt=p.backend=="vulkan"?AV_PIX_FMT_VULKAN:AV_PIX_FMT_NV12;p.codec->gop_size=fps;p.codec->max_b_frames=0;p.codec->profile=AV_PROFILE_H264_MAIN;p.codec->thread_count=1;
 if(p.device){p.codec->hw_device_ctx=av_buffer_ref(p.device->ref());p.frames=av_hwframe_ctx_alloc(p.device->ref());if(!p.frames)throw std::bad_alloc();auto* f=reinterpret_cast<AVHWFramesContext*>(p.frames->data);f->format=AV_PIX_FMT_VULKAN;f->sw_format=AV_PIX_FMT_NV12;f->width=w;f->height=h;f->initial_pool_size=kSlots;auto* vkframes=static_cast<AVVulkanFramesContext*>(f->hwctx);vkframes->flags=AV_VK_FRAME_FLAG_NONE;vkframes->format[0]=VK_FORMAT_G8_B8R8_2PLANE_420_UNORM;p.h264_profile.stdProfileIdc=STD_VIDEO_H264_PROFILE_IDC_MAIN;p.video_profile.pNext=&p.h264_profile;p.video_profile.videoCodecOperation=VK_VIDEO_CODEC_OPERATION_ENCODE_H264_BIT_KHR;p.video_profile.chromaSubsampling=VK_VIDEO_CHROMA_SUBSAMPLING_420_BIT_KHR;p.video_profile.lumaBitDepth=VK_VIDEO_COMPONENT_BIT_DEPTH_8_BIT_KHR;p.video_profile.chromaBitDepth=VK_VIDEO_COMPONENT_BIT_DEPTH_8_BIT_KHR;p.profile_list.profileCount=1;p.profile_list.pProfiles=&p.video_profile;vkframes->create_pnext=&p.profile_list;vkframes->usage=static_cast<VkImageUsageFlagBits>(VK_IMAGE_USAGE_VIDEO_ENCODE_SRC_BIT_KHR);avcheck(av_hwframe_ctx_init(p.frames),"video frame pool");p.codec->hw_frames_ctx=av_buffer_ref(p.frames);}
 AVDictionary* options=nullptr;if(p.backend=="cpu"){av_dict_set(&options,"preset","ultrafast",0);av_dict_set(&options,"tune","zerolatency",0);}else av_dict_set(&options,"async_depth","1",0);
 auto result=avcodec_open2(p.codec,codec,&options);av_dict_free(&options);avcheck(result,"open H.264 encoder");
 if(p.device)p.conversion=std::make_unique<VideoConversion>(p.device->context(),w,h);
 if(!p.device){p.converter=sws_getContext(w,h,AV_PIX_FMT_RGB24,w,h,AV_PIX_FMT_NV12,SWS_POINT,nullptr,nullptr,nullptr);if(!p.converter)throw std::runtime_error("RGB conversion context unavailable");}
 for(unsigned i=0;i<kSlots;i++){if(p.device){p.gpu[i]=av_frame_alloc();if(!p.gpu[i])throw std::bad_alloc();avcheck(av_hwframe_get_buffer(p.frames,p.gpu[i],0),"preallocate video frame");}else{p.cpu[i]=av_frame_alloc();if(!p.cpu[i])throw std::bad_alloc();p.cpu[i]->format=AV_PIX_FMT_NV12;p.cpu[i]->width=w;p.cpu[i]->height=h;avcheck(av_frame_get_buffer(p.cpu[i],32),"CPU video frame");}}
}
VideoEncoder::~VideoEncoder()=default;
int VideoEncoder::submit_rgb(const FrameLease& lease,int64_t pts){auto& p=*impl_;if(p.flushed)return AVERROR_EOF;if(!lease)return AVERROR(EINVAL);auto& s=lease.slot();if(s.metadata.width!=p.width||s.metadata.height!=p.height||!s.upload.mapped)return AVERROR(EINVAL);
 // Input must be complete; never wait on a perception fence in the video lane.
 if(p.device&&lease.context()!=p.device->context())return AVERROR(EINVAL);
 if(p.device&&!s.submitted)return AVERROR(EINVAL);
 if(s.submitted&&s.fence){auto owner=lease.context();if(!owner)return AVERROR(EINVAL);auto r=vkGetFenceStatus(owner->device,s.fence);if(r==VK_NOT_READY)return AVERROR(EAGAIN);if(r!=VK_SUCCESS)return AVERROR_EXTERNAL;}
 unsigned i=0;for(;i<kSlots;i++){auto* f=p.device?p.gpu[i]:p.cpu[i];if(f->buf[0]&&av_buffer_get_ref_count(f->buf[0])==1&&(!p.conversion||p.conversion->ready(i)))break;}if(i==kSlots)return AVERROR(EAGAIN);
 AVFrame* frame=p.device?p.gpu[i]:p.cpu[i];
 if(p.device){auto r=p.conversion->submit(i,lease,frame);if(r<0)return r;}else{
 const uint8_t* data[]={static_cast<const uint8_t*>(s.upload.mapped)};int stride[]={int(p.width*3)};if(sws_scale(p.converter,data,stride,0,p.height,p.cpu[i]->data,p.cpu[i]->linesize)!=int(p.height))return AVERROR_EXTERNAL;
 }
 frame->pts=pts;
 return avcodec_send_frame(p.codec,frame);
}
int VideoEncoder::receive(AVPacket* packet){return packet?avcodec_receive_packet(impl_->codec,packet):AVERROR(EINVAL);}
int VideoEncoder::flush(){if(impl_->flushed)return AVERROR_EOF;int r=avcodec_send_frame(impl_->codec,nullptr);if(r>=0)impl_->flushed=true;return r;}
const std::string& VideoEncoder::backend()const{return impl_->backend;}
VideoSession::VideoSession(unsigned width,unsigned height,unsigned fps,const std::string& mode,const std::string& path,std::shared_ptr<Context> context){
 auto qualification=load_video_qualification(path,video_profile_key(context,width,height,fps),context);selection_=select_video_backend(mode,qualification);
 try{std::shared_ptr<SharedVideoDevice> shared;if(selection_.backend=="vulkan")shared=std::make_shared<SharedVideoDevice>(context);encoder_=std::make_unique<VideoEncoder>(width,height,fps,selection_,shared);}
 catch(const std::exception& error){if(mode!="auto"||selection_.backend=="cpu")throw;selection_={"cpu",std::string("qualified Vulkan profile failed current initialization: ")+error.what()+"; same-device VAAPI interoperability unqualified; CPU fallback"};encoder_=std::make_unique<VideoEncoder>(width,height,fps,selection_);}
}
struct VideoDecoder::Impl {AVCodecContext* codec=nullptr;std::shared_ptr<SharedVideoDevice> device;std::array<AVBufferRef*,kSlots> tickets{};~Impl(){avcodec_free_context(&codec);for(auto*& ticket:tickets)av_buffer_unref(&ticket);}};
VideoDecoder::VideoDecoder(const VideoSelection& selection,std::shared_ptr<SharedVideoDevice> device):impl_(new Impl){auto& p=*impl_;if(selection.backend!="cpu"&&selection.backend!="vulkan")throw std::runtime_error("VAAPI mapping unqualified");if(selection.backend=="vulkan"&&!device)throw std::invalid_argument("Vulkan decode requires resident shared device");if(selection.backend!="cpu")p.device=std::move(device);for(auto*& ticket:p.tickets){ticket=av_buffer_alloc(1);if(!ticket)throw std::bad_alloc();}auto* codec=avcodec_find_decoder(AV_CODEC_ID_H264);if(!codec)throw std::runtime_error("H.264 decoder unavailable");p.codec=avcodec_alloc_context3(codec);if(!p.codec)throw std::bad_alloc();p.codec->max_pixels=640*480;p.codec->thread_count=1;p.codec->err_recognition=AV_EF_EXPLODE|AV_EF_BITSTREAM|AV_EF_BUFFER;p.codec->extra_hw_frames=kSlots;
 if(p.device){p.codec->hw_device_ctx=av_buffer_ref(p.device->ref());p.codec->get_format=[](AVCodecContext* codec,const AVPixelFormat* formats){if(codec->coded_width>640||codec->coded_height>480)return AV_PIX_FMT_NONE;for(;*formats!=AV_PIX_FMT_NONE;formats++)if(*formats==AV_PIX_FMT_VULKAN)return *formats;return AV_PIX_FMT_NONE;};}
 avcheck(avcodec_open2(p.codec,codec,nullptr),"open H.264 decoder");}
VideoDecoder::~VideoDecoder()=default;
int VideoDecoder::send(const AVPacket* packet){if(packet&&packet->size>2*1024*1024)return AVERROR(EINVAL);return avcodec_send_packet(impl_->codec,packet);}
int VideoDecoder::receive(AVFrame* frame){if(!frame)return AVERROR(EINVAL);AVBufferRef* ticket=nullptr;for(auto* candidate:impl_->tickets)if(av_buffer_get_ref_count(candidate)==1){ticket=candidate;break;}if(!ticket)return AVERROR(EAGAIN);int result=avcodec_receive_frame(impl_->codec,frame);if(result>=0){av_buffer_unref(&frame->opaque_ref);frame->opaque_ref=av_buffer_ref(ticket);if(!frame->opaque_ref){av_frame_unref(frame);return AVERROR(ENOMEM);}}return result;}
}
