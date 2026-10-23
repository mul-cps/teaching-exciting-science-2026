"""CPU-only review guards; FFmpeg internal allocations remain a measured boundary."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'

def body(file, symbol):
    text = (ROOT / file).read_text()
    start = text.index('{', text.index(symbol)); end = start + 1; depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}'); end += 1
    return text[start:end]


def test_video_has_private_command_pool_and_complete_source_admission():
    code = (ROOT / 'src/video_conversion.cpp').read_text()
    assert 'context_->commands' not in code
    assert 'cb.commandPool=commands_' in code
    assert 'vkCreateCommandPool' in body('src/video_conversion.cpp', 'VideoConversion::VideoConversion(')
    assert 'vkDestroyCommandPool' in body('src/video_conversion.cpp', 'void VideoConversion::cleanup(')
    submit = body('src/video_conversion.cpp', 'int VideoConversion::submit(')
    assert 'input.slot().submitted' in submit
    assert submit.index('vkGetFenceStatus') < submit.index('vkUpdateDescriptorSets')
    assert 'queue_mutex' in submit


def test_video_normal_vulkan_methods_never_allocate_or_download_or_wait():
    for file, symbol in [('src/video_conversion.cpp','int VideoConversion::submit('),
                         ('src/video_conversion.cpp','bool VideoConversion::ready('),
                         ('src/video.cpp','int VideoEncoder::submit_rgb('),
                         ('src/video.cpp','int VideoDecoder::receive(')]:
        code = body(file, symbol)
        assert not re.search(r'vk(?:Create\w*|Allocate\w*|WaitForFences|QueueWaitIdle|DeviceWaitIdle|MapMemory)\s*\(', code)
        assert 'av_hwframe_transfer_data' not in code
        assert 'av_hwframe_get_buffer' not in code
        assert not re.search(r'\b(?:new|malloc|calloc|realloc)\b', code)
    # Explicit diagnostic transfer stays in the qualification executable.
    assert 'av_hwframe_transfer_data' in (ROOT / 'src/qualify_video.cpp').read_text()


def test_ffmpeg_frame_lock_ownership_and_64bit_timeline_contract():
    code = body('src/video_conversion.cpp','int VideoConversion::submit(')
    assert 'fc->lock_frame(frames,image)' in code
    assert 'c->unlock_frame(f,i)' in code
    assert 'image->queue_family[i]!=VK_QUEUE_FAMILY_IGNORED&&image->queue_family[i]!=context_->family' in code
    assert 'uint64_t wait[2]' in code
    assert 'image->sem[0]==image->sem[1]' in code
    assert 's.input=input' in code
    assert 'if(r==VK_SUCCESS){s.input={};' in body('src/video_conversion.cpp','bool VideoConversion::ready(')
    assert 'std::array<Resources,kSlots>' in (ROOT / 'src/video_conversion.hpp').read_text()


def test_runtime_decoder_backpressure_and_cpu_backend_do_not_borrow_gpu():
    receive = body('src/video.cpp','int VideoDecoder::receive(')
    assert 'if(!ticket)return AVERROR(EAGAIN)' in receive
    assert 'if(selection.backend!="cpu")p.device=std::move(device)' in (ROOT/'src/video.cpp').read_text()
    assert 'if(p.backend!="cpu")p.device=std::move(device)' in (ROOT/'src/video.cpp').read_text()
    assert 'packet->size>2*1024*1024' in body('src/video.cpp','int VideoDecoder::send(')


def test_gpu_encoder_does_not_construct_unused_software_conversion_pool():
    code = body('src/video.cpp','VideoEncoder::VideoEncoder(')
    assert 'if(!p.device){p.converter=sws_getContext' in code
    assert '}else{p.cpu[i]=av_frame_alloc()' in code


def test_context_negotiates_maintenance1_with_stable_queried_feature_chain():
    header = (ROOT / 'include/so101_station_vulkan/pipeline.hpp').read_text()
    code = body('src/vulkan_context.cpp', 'Context::Context(')
    feature_type = 'VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VIDEO_MAINTENANCE_1_FEATURES_KHR'
    assert f'VkPhysicalDeviceVideoMaintenance1FeaturesKHR enabled_video_maintenance1{{{feature_type}}}' in header
    inventory = code.index('vkEnumerateDeviceExtensionProperties')
    query = code.index('vkGetPhysicalDeviceFeatures2')
    chain = code.index('if(has("VK_KHR_video_maintenance1"))enabled13.pNext=&enabled_video_maintenance1;')
    assert inventory < chain < query
    assert 'enabled_features.pNext=&enabled11;enabled11.pNext=&enabled12;enabled12.pNext=&enabled13;' in code
    optional = re.search(r'const char\* optional\[\]=\{([^}]+)\}', code).group(1)
    assert '"VK_KHR_video_maintenance1"' in optional
    assert 'has("VK_KHR_video_maintenance1")&&enabled_video_maintenance1.videoMaintenance1' in code
    assert '&&enabled12.timelineSemaphore&&enabled13.synchronization2' in code
    assert 'if(video_capable){d.pNext=&enabled_features;' in code
    reset = body('src/vulkan_context.cpp', 'if(!video_capable){enabled_features=')
    assert f'enabled_video_maintenance1={{{feature_type}}};' in reset
    retry = body('src/vulkan_context.cpp', 'if(result!=VK_SUCCESS&&video_capable)')
    assert 'video_capable=false' in retry and 'd.pNext=nullptr' in retry
    assert 'd.enabledExtensionCount=0' in retry and 'd.queueCreateInfoCount=1' in retry
    # Borrowed FFmpeg copies the root while retaining the Context-owned tail.
    video = (ROOT / 'src/video.cpp').read_text()
    assert 'life->owner=impl_->context' in video
    assert 'v->device_features=life->owner->enabled_features' in video
