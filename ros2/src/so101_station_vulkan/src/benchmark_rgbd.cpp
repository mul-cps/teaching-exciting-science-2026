#include "so101_station_vulkan/pipeline.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
static void read(const std::string &path, void *out, size_t n) {
  std::ifstream f(path, std::ios::binary);
  if (!f || !f.read(static_cast<char *>(out), n) || f.peek() != EOF)
    throw std::runtime_error("fixture size mismatch: " + path);
}
static std::string json_string(const std::string& value) {
  std::string result;
  for (unsigned char c : value) {
    if(c=='"'||c=='\\'){result+='\\';result+=char(c);}
    else if(c=='\n')result+="\\n";
    else if(c=='\r')result+="\\r";
    else if(c=='\t')result+="\\t";
    else if(c<32)result+='?';
    else result+=char(c);
  }
  return result;
}
int main(int argc, char **argv) {
  try {
    Calibration calibration;
    bool custom_center = false, profile=false;
    unsigned width = 640, height = 480, frames = 12, warmup = 2;
    std::string mode = "compare", shader = DEFAULT_SHADER_DIR, rgb, depth,
                output, dump;
    for (int i = 1; i < argc; i++) {
      std::string a = argv[i];
      if (i + 1 >= argc)
        throw std::invalid_argument("missing argument value");
      std::string v = argv[++i];
      if(a=="--timestamps")profile=std::stoul(v)!=0;
      else if (a == "--depth-scale")
        calibration.depth_scale = std::stof(v);
      else if (a == "--fx")
        calibration.fx = std::stof(v);
      else if (a == "--fy")
        calibration.fy = std::stof(v);
      else if (a == "--cx") {
        calibration.cx = std::stof(v);
        custom_center = true;
      } else if (a == "--cy") {
        calibration.cy = std::stof(v);
        custom_center = true;
      } else if (a == "--backend")
        mode = v;
      else if (a == "--width")
        width = std::stoul(v);
      else if (a == "--height")
        height = std::stoul(v);
      else if (a == "--frames")
        frames = std::stoul(v);
      else if (a == "--warmup")
        warmup = std::stoul(v);
      else if (a == "--shaders")
        shader = v;
      else if (a == "--rgb")
        rgb = v;
      else if (a == "--depth")
        depth = v;
      else if (a == "--output")
        output = v;
      else if (a == "--dump")
        dump = v;
      else
        throw std::invalid_argument("unknown argument: " + a);
    }
    if (!frames || frames > 100000 || warmup > 10000)
      throw std::invalid_argument("invalid bounded iteration count");
    if (mode == "cpu" && !dump.empty())
      throw std::invalid_argument(
          "dense GPU diagnostic dump requires Vulkan backend");
    const bool compare = mode == "compare";
    Pipeline p(width, height, compare ? "vulkan" : mode, shader,
               compare || !dump.empty(),profile);
    Slot reference;
    size_t n = size_t(width) * height;
    std::vector<uint8_t> fixture(input_bytes(width, height));
    reference.cpu_masks.resize(n);
    reference.cpu_parents.resize(n);
    reference.cpu_depth.resize(n);
    reference.cpu_xyz.resize(n * 4);
    reference.cpu_work.resize(n);reference.cpu_dilated.resize(n);reference.cpu_morph_counts.resize(n);reference.cpu_morph_roots.resize(n*9);
    reference.upload.mapped = fixture.data();
    if (!rgb.empty() && !depth.empty()) {
      read(rgb, fixture.data(), n * 3);
      read(depth, fixture.data() + 3 * n, n * 2);
    } else {
      // Frozen deterministic checker/holes/depth discontinuities and long snake
      // edges.
      uint32_t random = 0x1012026u;
      for (unsigned i = 0; i < n; i++) {
        unsigned x = i % width, y = i / width;
        uint8_t *c = fixture.data() + 3 * i;
        random = random * 1664525u + 1013904223u;
        if (y < height / 3) {
          c[0] = uint8_t(random >> 24);
          c[1] = uint8_t(random >> 16);
          c[2] = uint8_t(random >> 8);
        } else {
          unsigned group = (x / (std::max(1u, width / 8))) % 4;
          c[group % 3] = 255;
          if (group == 3)
            c[0] = c[1] = 255;
        }
        uint16_t z = ((x + y) % 71 == 0)
                         ? 0
                         : ((x > width / 2 && y > height / 2) ? 520 : 500);
        fixture[3 * n + 2 * i] = z & 255;
        fixture[3 * n + 2 * i + 1] = z >> 8;
      }
    }
    reference.calibration = calibration;
    if (!custom_center) {
      reference.calibration.cx = float(width) / 2;
      reference.calibration.cy = float(height) / 2;
    }
    cpu_reference(reference, width, height);
    std::array<double,8> stage_sums{};
    std::vector<double> times;
    times.reserve(frames);
    uint64_t allocations =
        p.context() ? p.context()->allocation_count.load() : 0;
    for (unsigned frame = 0; frame < frames + warmup; frame++) {
      auto f = p.acquire();
      if (!f)
        throw std::runtime_error("no free slot in serial qualification");
      auto &s = f.slot();
      std::memcpy(s.upload.mapped, fixture.data(), fixture.size());
      s.calibration = reference.calibration;
      s.metadata.frame_number = frame;
      auto start = std::chrono::steady_clock::now();
      p.submit(f);
      while (!p.ready(f))
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      p.finish(f);
      double ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - start)
                      .count();
      if (frame >= warmup){times.push_back(ms);for(unsigned k=0;k<8;k++)stage_sums[k]+=s.gpu_stage_ms[k];}
      if (compare) {
        auto *bytes = static_cast<uint32_t *>(s.diagnostic.mapped);
        size_t mask_bad = 0, depth_bad = 0, parent_bad = 0, xyz_bad = 0;
        for (size_t i = 0; i < n; i++) {
          mask_bad += bytes[i] != reference.cpu_masks[i];
          depth_bad +=
              std::memcmp(bytes + n + i, &reference.cpu_depth[i], 4) != 0;
          parent_bad += bytes[17 * n + i] != reference.cpu_parents[i];
          for (unsigned k = 0; k < 4; k++) {
            float value;
            std::memcpy(&value, bytes + 2 * n + 4 * i + k, 4);
            xyz_bad += std::abs(value - reference.cpu_xyz[4 * i + k]) > 1e-7f;
          }
        }
        if (mask_bad || depth_bad || parent_bad || xyz_bad)
          throw std::runtime_error(
              "dense mismatch masks=" + std::to_string(mask_bad) +
              " depth=" + std::to_string(depth_bad) + " parents=" +
              std::to_string(parent_bad) + " xyz=" + std::to_string(xyz_bad));
        for(size_t i=0;i<n;i++){
        auto offset=18*n+10*i;auto count=bytes[offset];if(count!=reference.cpu_morph_counts[i]||count>9)throw std::runtime_error("morphology membership count mismatch");
        std::array<uint32_t,9> actual{};for(unsigned k=0;k<count;k++)actual[k]=bytes[offset+1+k];std::sort(actual.begin(),actual.begin()+count);
        for(unsigned k=0;k<count;k++)if(actual[k]!=reference.cpu_morph_roots[i*9+k])throw std::runtime_error("morphology membership mismatch");
      }
      if (s.component_count != reference.component_count)
          throw std::runtime_error("component count mismatch");
        for (unsigned i = 0; i < s.component_count; i++)
          if (!(s.cpu_components[i] == reference.cpu_components[i]))
            throw std::runtime_error("compact component mismatch");
      }
      if (frame + 1 == frames + warmup && !dump.empty()) {
        std::ofstream f(dump, std::ios::binary);
        f.write(static_cast<char *>(s.diagnostic.mapped),s.diagnostic.size);
      }
    }
    if (p.context() && p.context()->allocation_count != allocations)
      throw std::runtime_error("hot path Vulkan allocation detected");
    std::sort(times.begin(), times.end());
    auto percentile = [&](double p) {
      return times[size_t((times.size() - 1) * p)];
    };
    std::string device =
        p.context() ? p.context()->properties.deviceName : "native CPU";
    std::string result =
        "{\"backend\":\"" + p.backend() + "\",\"fallback_reason\":\"" + json_string(p.fallback_reason()) + "\",\"device\":\"" + json_string(device) +
        "\",\"frames\":" + std::to_string(frames) +
        ",\"width\":" + std::to_string(width) +
        ",\"height\":" + std::to_string(height) + ",\"dense_readback\":" +
        (p.backend() == "vulkan" && (compare || !dump.empty()) ? "true"
                                                               : "false") +
        ",\"qualified_scope\":\"aligned pinhole RGB8/Z16: integer HSV, depth "
        "cuts, 8-connected membership, compact "
        "counts/"
        "bounds\",\"full_station_qualified\":false,\"hot_path_vulkan_"
        "allocations\":0,\"upload_bytes_per_frame\":" +
        std::to_string(fixture.size()) + ",\"readback_bytes_per_frame\":" +
        std::to_string(p.backend() == "vulkan"
                           ? compact_bytes() +
                                 ((compare || !dump.empty())
                                      ? scratch_bytes(width, height)+morphology_bytes(width,height)
                                      : 0)
                           : 0) +
        ",\"p50_ms\":" + std::to_string(percentile(.5)) +
        ",\"p95_ms\":" + std::to_string(percentile(.95)) +
        ",\"p99_ms\":" + std::to_string(percentile(.99)) + "}";
    if(profile&&p.backend()=="vulkan"){const char* names[]={"upload","convert","cuts","tiled_boundary_cc","reduce","morphology","gather","readback"};result.pop_back();result+=",\"gpu_stage_mean_ms\":{";for(unsigned k=0;k<8;k++){if(k)result+=",";result+="\""+std::string(names[k])+"\":"+std::to_string(stage_sums[k]/frames);}result+="}}";}
    std::cout << result << '\n';
    if (!output.empty()) {
      std::ofstream f(output);
      f << result << '\n';
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "resident RGBD failure: " << e.what() << '\n';
    return 1;
  }
}
