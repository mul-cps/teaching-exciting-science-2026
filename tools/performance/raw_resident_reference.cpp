// Qualification only. No camera acquisition, normal-path change or timing
// claim.
#include "so101_station_vulkan/raw_capture_bridge.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <librealsense2/hpp/rs_internal.hpp>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <thread>
using namespace so101_station_vulkan;
using json = nlohmann::json;
constexpr size_t N = 640 * 480;
std::vector<uint16_t> sdk_align(const AlignmentCalibration &k,
                                std::vector<uint16_t> &raw) {
  rs2::software_device device;
  auto ds = device.add_sensor("Depth"), cs = device.add_sensor("Color");
  auto depth = ds.add_video_stream(
      {RS2_STREAM_DEPTH, 0, 1, 640, 480, 30, 2, RS2_FORMAT_Z16, k.depth});
  auto color = cs.add_video_stream(
      {RS2_STREAM_COLOR, 0, 2, 640, 480, 30, 3, RS2_FORMAT_RGB8, k.color});
  depth.register_extrinsics_to(color, k.depth_to_color);
  device.create_matcher(RS2_MATCHER_DEFAULT);
  ds.add_read_only_option(RS2_OPTION_DEPTH_UNITS, k.depth_units);
  rs2::syncer sync;
  ds.open(depth);
  cs.open(color);
  ds.start(sync);
  cs.start(sync);
  std::vector<uint8_t> rgb(N * 3);
  rs2::align align(RS2_STREAM_COLOR);
  std::vector<uint16_t> output;
  for (int f = 1; f <= 5; f++) {
    ds.on_video_frame({raw.data(), [](void *) {}, 640 * 2, 2, f * 1000.0 / 30,
                       RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, f, depth,
                       k.depth_units});
    cs.on_video_frame({rgb.data(), [](void *) {}, 640 * 3, 3, f * 1000.0 / 30,
                       RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, f, color});
    auto frames = sync.wait_for_frames(1000);
    if (!frames.get_depth_frame() || !frames.get_color_frame())
      continue;
    auto result = align.process(frames).as<rs2::frameset>().get_depth_frame();
    if (result.get_frame_number() != uint64_t(f) ||
        result.get_frame_timestamp_domain() !=
            RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
      throw std::runtime_error("SDK changed depth frame provenance");
    output.resize(N);
    std::memcpy(output.data(), result.get_data(), N * 2);
  }
  ds.stop();
  cs.stop();
  ds.close();
  cs.close();
  if (output.empty())
    throw std::runtime_error("software device did not synchronize");
  return output;
}
static void require(bool b, const std::string &text) {
  if (!b)
    throw std::runtime_error(text);
}
static void bytes(const std::string &p, const void *data, size_t n) {
  std::ofstream f(p, std::ios::binary);
  f.write(static_cast<const char *>(data), n);
  require(bool(f), "fixture output failed");
}
static AlignmentCalibration calibration(unsigned fixture) {
  AlignmentCalibration k;
  k.depth = {640,
             480,
             319.5f,
             239.5f,
             fixture % 2 ? 390.f : 600.f,
             fixture % 2 ? 390.f : 600.f,
             RS2_DISTORTION_NONE,
             {0, 0, 0, 0, 0}};
  k.color = {640,
             480,
             319.7f,
             239.2f,
             610.f,
             605.f,
             RS2_DISTORTION_NONE,
             {0, 0, 0, 0, 0}};
  k.depth_to_color = {{1, 0, 0, 0, 1, 0, 0, 0, 1},
                      {fixture > 1 ? .0234f : 0.f, fixture > 2 ? -.0027f : 0.f,
                       fixture > 3 ? .001f : 0.f}};
  k.convention = AlignmentConvention::Scalar;
  k.generation = 17;
  k.depth_units = fixture == 5 ? .00025f : .001f;
  return k;
}
static std::vector<uint16_t> raw_input(unsigned fixture) {
  std::vector<uint16_t> raw(N);
  uint32_t seed = 0x452331ab;
  for (size_t i = 0; i < N; i++) {
    seed = 1664525 * seed + 1013904223;
    raw[i] = i % 7 == 0 ? 0 : uint16_t(250 + (seed % 4250));
  }
  if (fixture == 0)
    std::fill(raw.begin(), raw.end(), 1000);
  if (fixture == 4)
    for (size_t i = 0; i < N; i++)
      raw[i] = i % 101 ? 0 : 65535;
  if (fixture == 6)
    std::fill(raw.begin(), raw.end(), 0);
  if (fixture == 7) {
    std::fill(raw.begin(), raw.end(), 0);
    for (unsigned i : {0u, 639u, 306560u, 307199u, 153919u})
      raw[i] = 1000;
  }
  return raw;
}
static std::vector<uint8_t> rgb_input() {
  std::vector<uint8_t> rgb(N * 3);
  for (unsigned c = 0; c < 3; c++)
    for (unsigned y = 80; y < 104; y++)
      for (unsigned x = 80 + c * 160; x < 112 + c * 160; x++)
        rgb[3 * (y * 640 + x) + c] = 255;
  return rgb;
}
static void prepare_reference(Slot &r, std::vector<uint8_t> &packed,
                              const Calibration &k) {
  r.cpu_masks.resize(N);
  r.cpu_parents.resize(N);
  r.cpu_depth.resize(N);
  r.cpu_xyz.resize(N * 4);
  r.cpu_work.resize(N);
  r.cpu_dilated.resize(N);
  r.cpu_morph_counts.resize(N);
  r.cpu_morph_roots.resize(N * 9);
  r.upload.mapped = packed.data();
  r.calibration = k;
  cpu_reference(r, 640, 480);
  require(r.fault == 0, "reference fault");
}
static json check_core(const Slot &actual, const Slot &reference, bool gpu) {
  json mismatch = {{"masks", 0}, {"depth_bits", 0}, {"canonical_membership", 0},
                   {"xyz", 0},   {"morphology", 0}, {"compact", 0}};
  size_t masks = 0, depth = 0, parent = 0, xyz = 0, morph = 0, compact = 0;
  const auto *words =
      gpu ? static_cast<const uint32_t *>(actual.diagnostic.mapped) : nullptr;
  for (size_t i = 0; i < N; i++) {
    masks += (gpu ? words[i] : actual.cpu_masks[i]) != reference.cpu_masks[i];
    if (gpu)
      depth += std::memcmp(words + N + i, &reference.cpu_depth[i], 4) != 0;
    else
      depth +=
          std::memcmp(&actual.cpu_depth[i], &reference.cpu_depth[i], 4) != 0;
    parent += (gpu ? words[17 * N + i] : actual.cpu_parents[i]) !=
              reference.cpu_parents[i];
    for (unsigned a = 0; a < 4; a++) {
      float v;
      if (gpu)
        std::memcpy(&v, words + 2 * N + 4 * i + a, 4);
      else
        v = actual.cpu_xyz[4 * i + a];
      const float expected = reference.cpu_xyz[4 * i + a];
      xyz += !std::isfinite(v) || !std::isfinite(expected) ||
             std::abs(v - expected) > 1e-7f;
    }
    unsigned count = gpu ? words[18 * N + 10 * i] : actual.cpu_morph_counts[i];
    if (count != reference.cpu_morph_counts[i] || count > 9) {
      morph++;
      continue;
    }
    std::array<uint32_t, 9> roots{};
    for (unsigned a = 0; a < count; a++)
      roots[a] = gpu ? words[18 * N + 10 * i + 1 + a]
                     : actual.cpu_morph_roots[9 * i + a];
    std::sort(roots.begin(), roots.begin() + count);
    for (unsigned a = 0; a < count; a++)
      morph += roots[a] != reference.cpu_morph_roots[9 * i + a];
  }
  compact += actual.component_count != reference.component_count;
  for (unsigned i = 0;
       i < std::min(actual.component_count, reference.component_count); i++)
    compact += !(actual.cpu_components[i] == reference.cpu_components[i]);
  mismatch = {
      {"masks", masks}, {"depth_bits", depth}, {"canonical_membership", parent},
      {"xyz", xyz},     {"morphology", morph}, {"compact", compact}};
  require(masks + depth + parent + xyz + morph + compact == 0,
          "core arrays/compact mismatch: " + mismatch.dump());
  require(actual.fault == 0, "core fault");
  return mismatch;
}
int main(int argc, char **argv) {
  json report = {{"schema", "raw-capture-bridge-offline-v1"},
                 {"physical", false},
                 {"execution_eligible", false},
                 {"geometry_admission", "unqualified"},
                 {"cases", json::array()}};
  try {
    require(argc == 5, "usage: headless --cpu|--vulkan CORE_SHADERS "
                       "ALIGNMENT_SHADERS OUTPUT_DIRECTORY");
    const bool gpu = std::string(argv[1]) == "--vulkan";
    require(gpu || std::string(argv[1]) == "--cpu", "invalid mode");
    const std::string output = argv[4];
    report["gpu"] = gpu;
    for (unsigned fixture = 0; fixture < 8; fixture++) {
      const auto k = calibration(fixture);
      auto raw = raw_input(fixture);
      const auto rgb = rgb_input();
      auto sdk = sdk_align(k, raw);
      std::vector<uint32_t> aligned(N);
      std::vector<float> aligned_xyz(N * 4);
      alignment_reference(k, raw.data(), aligned.data(), aligned_xyz.data());
      size_t cpu_sdk = 0;
      for (size_t i = 0; i < N; i++)
        cpu_sdk += aligned[i] != sdk[i];
      require(cpu_sdk == 0, "independent CPU SDK depth mismatch");
      std::vector<uint8_t> packed(input_bytes(640, 480));
      std::memcpy(packed.data(), rgb.data(), N * 3);
      std::memcpy(packed.data() + N * 3, sdk.data(), N * 2);
      Calibration core_k{k.color.fx,  k.color.fy,    k.color.ppx,
                         k.color.ppy, k.depth_units, k.generation};
      Slot reference;
      prepare_reference(reference, packed, core_k);
      if (fixture == 0)
        require(reference.component_count == 3,
                "uniform fixture must retain exactly3 RGB components");
      const std::string prefix = output + "/case-" + std::to_string(fixture);
      bytes(prefix + "-raw-z16.bin", raw.data(), N * 2);
      bytes(prefix + "-rgb.bin", rgb.data(), N * 3);
      bytes(prefix + "-sdk-z16.bin", sdk.data(), N * 2);
      json cal = {
          {"fixture", fixture},
          {"depth_fx", k.depth.fx},
          {"depth_fy", k.depth.fy},
          {"depth_ppx", k.depth.ppx},
          {"depth_ppy", k.depth.ppy},
          {"color_fx", k.color.fx},
          {"color_fy", k.color.fy},
          {"color_ppx", k.color.ppx},
          {"color_ppy", k.color.ppy},
          {"depth_units", k.depth_units},
          {"synthetic", true},
          {"generation", k.generation},
          {"translation",
           {k.depth_to_color.translation[0], k.depth_to_color.translation[1],
            k.depth_to_color.translation[2]}},
          {"rotation", {1, 0, 0, 0, 1, 0, 0, 0, 1}},
          {"distortion_coefficients", {0, 0, 0, 0, 0}},
          {"extent", {640, 480}}};
      {
        std::ofstream f(prefix + "-calibration.json");
        f << cal.dump(2) << "\n";
        require(bool(f), "calibration output failed");
      }
      auto core = std::make_unique<Pipeline>(640, 480, gpu ? "vulkan" : "cpu",
                                             argv[2], gpu);
      const auto context = core->context();
      std::unique_ptr<Alignment> depth;
      if (gpu)
        depth = std::make_unique<Alignment>(context, k, argv[3], true);
      RawCaptureBridge bridge(std::move(core), std::move(depth));
      auto frame = bridge.acquire();
      require(bool(frame), "raw bridge initial acquire failed");
      auto b = bridge.acquire(), c = bridge.acquire();
      require(bool(b) && bool(c) && !bridge.acquire(),
              "raw bridge3slot bound failed");
      b = {};
      c = {};
      auto &slot = frame.core.slot();
      std::memcpy(slot.upload.mapped, rgb.data(), N * 3);
      slot.calibration = core_k;
      auto &m = slot.metadata;
      m.frame_number = 100 + fixture;
      m.depth_frame_number = 200 + fixture;
      m.timestamp_ns = 12345678 + fixture;
      m.calibration_generation = k.generation;
      m.hardware_timestamp_ms = 33.3 + fixture;
      m.depth_hardware_timestamp_ms = 34.4 + fixture;
      m.timestamp_domain = 0;
      m.depth_timestamp_domain = 0;
      m.serial = "synthetic";
      if (gpu) {
        frame.depth.metadata() = m;
        std::memcpy(frame.depth.upload(), raw.data(), N * 2);
      } else
        std::memcpy(static_cast<uint8_t *>(slot.upload.mapped) + N * 3,
                    sdk.data(), N * 2);
      require(bridge.accept_source(m.frame_number, m.depth_frame_number),
              "fresh source rejected");
      bridge.offer(std::move(frame));
      bridge.dispatch_pending();
      RawFrame done;
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (!bridge.collect(done)) {
        if (std::chrono::steady_clock::now() > deadline) {
          std::cerr << "{\"error\":\"raw bridge GPU completion "
                       "deadline\",\"physical\":false}\n"
                    << std::flush;
          std::_Exit(2);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      require(
          done.core.slot().metadata.frame_number == 100 + fixture &&
              done.core.slot().metadata.depth_frame_number == 200 + fixture &&
              done.core.slot().metadata.calibration_generation == k.generation,
          "cohort provenance changed");
      double max_xyz = 0;
      size_t depth_bad = 0;
      if (gpu) {
        require(vkGetFenceStatus(context->device, done.core.slot().fence) ==
                    VK_SUCCESS,
                "core fence incomplete after collect");
        require(vkGetFenceStatus(context->device, done.depth.fence()) ==
                    VK_SUCCESS,
                "source fence incomplete after consumer");
        const auto *z = done.depth.diagnostic_depth();
        const auto *xyz = done.depth.diagnostic_xyz();
        for (size_t i = 0; i < N; i++) {
          depth_bad += z[i] != sdk[i];
          for (unsigned a = 0; a < 3; a++) {
            const float expected = aligned_xyz[4 * i + a],
                        actual = xyz[4 * i + a];
            require(std::isfinite(actual) && std::isfinite(expected),
                    "nonfinite alignedXYZ");
            max_xyz = std::max(max_xyz, double(std::abs(actual - expected)));
          }
        }
        require(depth_bad == 0, "rawbridge aligned depth mismatch");
        require(max_xyz <= 1e-7, "rawbridge alignedXYZ exceeds1e-7");
      }
      auto arrays = check_core(done.core.slot(), reference, gpu);
      const auto count = done.core.slot().component_count;
      if (fixture == 0)
        require(count == 3,
                "uniform bridge output must retain exactly3 components");
      RawFrame retained = done;
      done = {};
      auto spare1 = bridge.acquire(), spare2 = bridge.acquire();
      require(bool(spare1) && bool(spare2) && !bridge.acquire(),
              "retained typed source/core lost slot ownership");
      spare1 = {};
      spare2 = {};
      retained = {};
      auto recycled = bridge.acquire();
      require(bool(recycled), "typed source/core did not recycle");
      const auto counters = bridge.counters();
      require(counters.received == 1 && counters.completed == 1 &&
                  counters.replaced == 0 && counters.no_slot == 0 &&
                  counters.duplicate_or_regressed == 0,
              "unexpected source counters");
      json row = {{"fixture", fixture},
                  {"cpu_sdk_depth_mismatches", cpu_sdk},
                  {"gpu_sdk_depth_mismatches", depth_bad},
                  {"alignment_max_xyz_error_m", max_xyz},
                  {"core_arrays_mismatches", arrays},
                  {"components", count},
                  {"slot_bound_and_typed_retention", true},
                  {"gpu_fences_checked_after_collect", gpu},
                  {"physical", false}};
      report["cases"].push_back(row);
      std::cout << row.dump() << "\n" << std::flush;
    }
    report["passed"] = true;
    {
      std::ofstream f(output + "/summary.json");
      f << report.dump(2) << "\n";
      require(bool(f), "summary output failed");
    }
    std::cout << report.dump() << "\n";
    return 0;
  } catch (const std::exception &e) {
    report["passed"] = false;
    report["error"] = e.what();
    std::cerr << report.dump() << "\n" << std::flush;
    return 1;
  }
}
