// Offline software-frame diagnostic. No physical-device discovery or live input.
// Build both CPU-only and SO101_BENCH_GLSL variants; GPU execution needs review.
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now()-start).count();
}
static std::string library_for(const void *symbol) {
    Dl_info info{};
    return dladdr(symbol, &info) && info.dli_fname ? info.dli_fname : "unknown";
}
static std::string sha256(const std::vector<uint8_t> &bytes) {
    unsigned char hash[SHA256_DIGEST_LENGTH]; SHA256(bytes.data(),bytes.size(),hash);
    std::ostringstream text; for(auto byte:hash) text<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);
    return text.str();
}
static std::vector<uint8_t> read_bytes(const std::filesystem::path &path,size_t expected) {
    if(std::filesystem::file_size(path)!=expected) throw std::runtime_error("input size mismatch");
    std::vector<uint8_t> bytes(expected); std::ifstream stream(path,std::ios::binary);
    if(!stream.read(reinterpret_cast<char*>(bytes.data()),expected)) throw std::runtime_error("input read failed");
    return bytes;
}
struct Config {
    int width=640,height=480,color_width=960,color_height=540;
    rs2_intrinsics kd{640,480,319.25f,239.75f,470.f,465.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};
    rs2_intrinsics kc{960,540,477.3f,269.2f,710.f,705.f,RS2_DISTORTION_NONE,{0,0,0,0,0}};
    rs2_extrinsics extrinsics{{0.99995f,0.f,-0.00999983f,0.f,1.f,0.f,0.00999983f,0.f,0.99995f},{0.025f,0.001f,0.003f}};
    std::vector<uint8_t> saved_depth,saved_color;
    Json provenance=Json::object();
    bool recorded=false;
    void load(const std::filesystem::path &path) {
        if(std::filesystem::file_size(path)>65536) throw std::runtime_error("metadata too large");
        std::ifstream input(path); input>>provenance;
        if(provenance.at("schema")!="realsense-offline-cohort-v1" || provenance.at("qualified")!=false
            || provenance.at("diagnostic_repeats_are_new_acquisitions")!=false) throw std::runtime_error("invalid cohort schema");
        auto profiles=provenance.at("profiles");
        width=profiles.at("depth").at("width"); height=profiles.at("depth").at("height");
        color_width=profiles.at("color").at("width"); color_height=profiles.at("color").at("height");
        if(width<1||height<1||width>1280||height>720||color_width<1||color_height<1||color_width>1920||color_height>1080)
            throw std::runtime_error("cohort dimensions exceed bounds");
        auto intrinsics=[](const Json &p,int w,int h) {
            auto k=p.at("k").get<std::vector<float>>(); auto d=p.at("d").get<std::vector<float>>();
            if(k.size()!=9||d.size()!=5) throw std::runtime_error("invalid calibration size");
            for(float value:k) if(!std::isfinite(value)) throw std::runtime_error("nonfinite calibration");
            for(float value:d) if(value!=0) throw std::runtime_error("nonzero distortion unsupported");
            if(k[0]<=0||k[4]<=0||k[1]!=0||k[3]!=0||k[6]!=0||k[7]!=0||k[8]!=1) throw std::runtime_error("invalid K");
            return rs2_intrinsics{w,h,k[2],k[5],k[0],k[4],RS2_DISTORTION_NONE,{0,0,0,0,0}};
        };
        kd=intrinsics(profiles.at("depth"),width,height); kc=intrinsics(profiles.at("color"),color_width,color_height);
        auto r=provenance.at("rotation_column_major").get<std::vector<float>>();
        auto t=provenance.at("translation_m").get<std::vector<float>>();
        if(r.size()!=9||t.size()!=3||provenance.at("depth_scale_m")!=0.001) throw std::runtime_error("invalid extrinsics/units");
        for(float v:r) if(!std::isfinite(v)) throw std::runtime_error("nonfinite rotation");
        for(float v:t) if(!std::isfinite(v)) throw std::runtime_error("nonfinite translation");
        std::copy(r.begin(),r.end(),extrinsics.rotation); std::copy(t.begin(),t.end(),extrinsics.translation);
        saved_depth=read_bytes(path.parent_path()/"depth.z16",size_t(width)*height*2);
        saved_color=read_bytes(path.parent_path()/"color.rgb8",size_t(color_width)*color_height*3);
        if(sha256(saved_depth)!=provenance.at("raw_sha256").at("depth.z16")
            ||sha256(saved_color)!=provenance.at("raw_sha256").at("color.rgb8")) throw std::runtime_error("raw SHA256 mismatch");
        recorded=true;
    }
};
struct SoftwareFrames {
    const Config &config;
    rs2::software_device device;
    rs2::software_sensor depth=device.add_sensor("offline-depth");
    rs2::software_sensor color=device.add_sensor("offline-color");
    rs2::stream_profile dp, cp;
    rs2::syncer sync{4};
    Json startup_discarded=Json::array();
    std::vector<uint16_t> pixels;
    std::vector<uint8_t> rgb;
    int number=0;
    explicit SoftwareFrames(const Config &c): config(c),pixels(size_t(c.width)*c.height),rgb(size_t(c.color_width)*c.color_height*3,127) {
        dp=depth.add_video_stream({RS2_STREAM_DEPTH,0,101,c.width,c.height,30,2,RS2_FORMAT_Z16,c.kd});
        cp=color.add_video_stream({RS2_STREAM_COLOR,0,102,c.color_width,c.color_height,30,3,RS2_FORMAT_RGB8,c.kc});
        dp.register_extrinsics_to(cp,c.extrinsics);
        depth.add_read_only_option(RS2_OPTION_DEPTH_UNITS,0.001f);
        device.create_matcher(RS2_MATCHER_DEFAULT);
        depth.open(dp); color.open(cp); depth.start(sync); color.start(sync);
        if(c.recorded) {std::memcpy(pixels.data(),c.saved_depth.data(),c.saved_depth.size());rgb=c.saved_color;}
        // Pinned SDK syncer test pytest-ts-same-fps.py documents separate D0/C0
        // startup outputs before the matcher knows both streams. Drain only this
        // explicit unmeasured pair; never combine partials or restamp them.
        send(0,0.0);
        rs2::frameset initial;
        for(int drained=0;drained<4 && sync.poll_for_frames(&initial);++drained) {
            Json entry=Json::array();
            for(auto frame:initial) {
                if(frame.get_frame_number()!=0 || frame.get_timestamp()!=0.0
                    || frame.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
                    throw std::runtime_error("unexpected matcher startup frame");
                entry.push_back({{"stream",frame.get_profile().stream_type()},
                    {"frame_number",frame.get_frame_number()},{"stamp_ms",frame.get_timestamp()}});
            }
            startup_discarded.push_back(std::move(entry));
        }
        if(startup_discarded.empty() || sync.poll_for_frames(&initial))
            throw std::runtime_error("matcher startup drain exceeded bounds or produced no output");
    }
    void send(int frame_number,double stamp) {
        depth.on_video_frame({pixels.data(),[](void*){},config.width*2,2,stamp,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,frame_number,dp,0.001f});
        color.on_video_frame({rgb.data(),[](void*){},config.color_width*3,3,stamp,RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,frame_number,cp,0.f});
    }
    ~SoftwareFrames() { depth.stop(); color.stop(); depth.close(); color.close(); }
    rs2::frameset next(int scenario) {
        if(!config.recorded) for(int y=0;y<config.height;++y) for(int x=0;x<config.width;++x) {
            uint16_t z=1000;
            if(scenario==1 && ((x/16+y/16)%7==0)) z=0;
            if(scenario==2) {z=(x>=200&&x<360&&y>=100&&y<380)?450:1800;if(x<12||y<9||(x>=380&&x<390)) z=0;}
            pixels[y*config.width+x]=z;
        }
        ++number;
        // Software scheduling stamps are diagnostic only. Original captured
        // headers stay exact in provenance; repeats are never new acquisitions.
        double stamp=number*(1000.0/30.0);
        send(number,stamp);
        auto frames=sync.wait_for_frames(1000);
        auto d=frames.get_depth_frame();auto c=frames.get_color_frame();
        if(!d||!c) throw std::runtime_error("software cohort incomplete after explicit startup drain");
        if(d.get_frame_number()!=static_cast<unsigned long long>(number)
            ||c.get_frame_number()!=static_cast<unsigned long long>(number)
            ||d.get_timestamp()!=stamp||c.get_timestamp()!=stamp
            ||d.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK
            ||c.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
            throw std::runtime_error("software cohort differs from exact scheduled pair");
        return frames;
    }
};
struct Output {
    std::vector<uint16_t> depth;
    std::vector<rs2::vertex> vertices;
    std::vector<rs2::texture_coordinate> uv;
    explicit Output(const Config &c):depth(size_t(c.color_width)*c.color_height),vertices(size_t(c.width)*c.height),uv(vertices.size()) {}
    double align_submit_ms=0, align_readback_copy_ms=0, cloud_submit_ms=0, cloud_readback_copy_ms=0, complete_ms=0;
    bool align_gpu=false, cloud_gpu=false, stamps_preserved=false;
};
static void process(rs2::align &align, rs2::pointcloud &pc, const rs2::frameset &input, Output &out, const Config &config) {
    auto start=Clock::now(), phase=start;
    auto aligned=align.process(input).as<rs2::frameset>();
    auto depth=aligned.get_depth_frame();
    if(!depth || depth.get_width()!=config.color_width || depth.get_height()!=config.color_height
        || depth.get_profile().format()!=RS2_FORMAT_Z16 || depth.get_bytes_per_pixel()!=2
        || depth.get_stride_in_bytes()!=config.color_width*2
        || depth.get_data_size()!=out.depth.size()*sizeof(uint16_t))
        throw std::runtime_error("unexpected aligned-depth storage");
    out.align_submit_ms=ms(phase);
    phase=Clock::now();
    // Exactly one aligned-depth host access and one bounded copy per cohort.
    std::memcpy(out.depth.data(),depth.get_data(),out.depth.size()*sizeof(uint16_t));
    out.align_readback_copy_ms=ms(phase);
    phase=Clock::now();
    pc.map_to(input.get_color_frame());
    auto points=pc.calculate(input.get_depth_frame());
    out.cloud_submit_ms=ms(phase);
    phase=Clock::now();
    if(points.size()!=out.vertices.size()) throw std::runtime_error("unexpected point dimensions");
    // SDK exposes separate XYZ/UV arrays: each accessed once, no per-point API.
    std::memcpy(out.vertices.data(),points.get_vertices(),out.vertices.size()*sizeof(rs2::vertex));
    std::memcpy(out.uv.data(),points.get_texture_coordinates(),out.uv.size()*sizeof(rs2::texture_coordinate));
    out.cloud_readback_copy_ms=ms(phase);
    out.complete_ms=ms(start);
    out.stamps_preserved=depth.get_timestamp()==input.get_depth_frame().get_timestamp()
        && depth.get_frame_number()==input.get_depth_frame().get_frame_number()
        && depth.get_frame_timestamp_domain()==input.get_depth_frame().get_frame_timestamp_domain()
        && points.get_timestamp()==input.get_depth_frame().get_timestamp()
        && points.get_frame_number()==input.get_depth_frame().get_frame_number();
}
static size_t nonfinite_count(const Output &output) {
    size_t count=0;
    for(size_t i=0;i<output.vertices.size();++i) {
        const auto &p=output.vertices[i];const auto &t=output.uv[i];
        count+=!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||!std::isfinite(t.u)||!std::isfinite(t.v);
    }
    return count;
}
// Diagnostic serialization uses only the already copied host buffers.

static void output_bytes(const std::filesystem::path& path,const void* data,size_t bytes) {
    std::ofstream stream(path,std::ios::binary);if(!stream.write(static_cast<const char*>(data),bytes))throw std::runtime_error("output failed");
}
int main(int argc,char**argv)try {
    Config config;std::filesystem::path output;int scenario=0,iterations=3;
    for(int i=1;i<argc;++i){std::string arg=argv[i];
        if(arg=="--cohort"&&i+1<argc)config.load(argv[++i]);
        else if(arg=="--output"&&i+1<argc)output=argv[++i];
        else if(arg=="--scenario"&&i+1<argc)scenario=std::stoi(argv[++i]);
        else if(arg=="--iterations"&&i+1<argc)iterations=std::stoi(argv[++i]);
        else throw std::runtime_error("unknown argument");}
    if(output.empty()||scenario<0||scenario>2||iterations<1||iterations>50)throw std::runtime_error("output/scenario required");
    std::filesystem::create_directories(output);
    SoftwareFrames source(config);rs2::align align(RS2_STREAM_COLOR);rs2::pointcloud pc;
    rs2::frameset frames;Output out(config);Json timings=Json::array();
    for(int iteration=0;iteration<iterations;++iteration){frames=source.next(scenario);process(align,pc,frames,out,config);
        if(!out.stamps_preserved||nonfinite_count(out)!=0)throw std::runtime_error("invalid repeated CPU output");
        timings.push_back(out.complete_ms);}
    if(!out.stamps_preserved||nonfinite_count(out)!=0)throw std::runtime_error("invalid CPU output");
    output_bytes(output/"aligned.z16",out.depth.data(),out.depth.size()*2);
    output_bytes(output/"vertices.f32",out.vertices.data(),out.vertices.size()*sizeof(rs2::vertex));
    output_bytes(output/"uv.f32",out.uv.data(),out.uv.size()*sizeof(rs2::texture_coordinate));
    output_bytes(output/"source.z16",source.pixels.data(),source.pixels.size()*2);
    output_bytes(output/"color.rgb8",source.rgb.data(),source.rgb.size());
    Json result={{"schema","realsense-cpu-projection-oracle-v1"},{"qualified",false},{"api_version",RS2_API_VERSION},
        {"sdk_library",library_for(reinterpret_cast<const void*>(&rs2_get_api_version))},
        {"align_name",align.get_info(RS2_CAMERA_INFO_NAME)},{"pointcloud_name",pc.get_info(RS2_CAMERA_INFO_NAME)},
        {"occlusion_option",pc.supports(RS2_OPTION_FILTER_MAGNITUDE)?pc.get_option(RS2_OPTION_FILTER_MAGNITUDE):-1},
        {"depth_size",{config.width,config.height}},{"color_size",{config.color_width,config.color_height}},
        {"depth_K",{config.kd.fx,config.kd.fy,config.kd.ppx,config.kd.ppy}},
        {"color_K",{config.kc.fx,config.kc.fy,config.kc.ppx,config.kc.ppy}},
        {"rotation_column_major",std::vector<float>(config.extrinsics.rotation,config.extrinsics.rotation+9)},
        {"translation_m",std::vector<float>(config.extrinsics.translation,config.extrinsics.translation+3)},
        {"depth_scale_m",0.001},{"source_provenance",config.provenance},{"scenario",scenario},
        {"original_headers_not_refreshed",true},{"diagnostic_repeats_are_new_acquisitions",false},
        {"software_frame_number",frames.get_depth_frame().get_frame_number()},
        {"software_stamp_ms",frames.get_depth_frame().get_timestamp()},{"startup_discarded",source.startup_discarded},
        {"stamps_preserved",out.stamps_preserved},{"complete_ms",out.complete_ms},{"sdk_complete_ms",timings}};
    (std::ofstream(output/"oracle.json"))<<result.dump(2)<<"\n";std::cout<<result.dump()<<"\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 2;}
