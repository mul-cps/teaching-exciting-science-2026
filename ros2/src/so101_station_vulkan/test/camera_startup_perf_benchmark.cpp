// CPU-only cost of the actual cold admission gate. No SDK, ROS or Vulkan.
// The out-of-line wrapper prevents replacing repeated deterministic sequences
// with their known result; its call overhead is included in the measurement.
#include "so101_station_vulkan/camera_startup_gate.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <new>

using so101_station_vulkan::CameraSourceObservation;
using so101_station_vulkan::CameraStartupGate;
namespace {
uint64_t allocations = 0;
volatile uint64_t retained_ready = 0;
constexpr unsigned cycles = 20000, trials = 7;
constexpr uint64_t step_ns = 33333334;
struct Trial { double ns; uint64_t observations, ready, heap; };

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
bool observe(CameraStartupGate& gate, const CameraSourceObservation& frame,
             uint64_t now_ns) {
  return gate.observe(frame, now_ns);
}

Trial run(bool resets) {
  uint64_t observations = 0, ready = 0;
  const auto before = allocations;
  const auto start = std::chrono::steady_clock::now();
  for (unsigned cycle = 0; cycle < cycles; ++cycle) {
    CameraStartupGate gate(0);
    for (unsigned i = 0; i < 120; ++i) {
      uint64_t depth = i + 1;
      if (resets) {
        // Captured-style startup: repeated depth, two later counter resets,
        // advancing color and nonregressing exposure timestamps throughout.
        if (i >= 15 && i < 20) depth = i == 15 ? 0 : i - 15;
        else if (i >= 20 && i <= 45) depth = 4;
        else if (i == 46) depth = 5;
        else if (i == 47 || i == 53) depth = 0;
        else if (i >= 48 && i < 53) depth = i - 47;
        else if (i > 53) depth = i - 53;
      }
      CameraSourceObservation frame{uint64_t(cycle) * 512 + i + 1, depth,
                                    i * 33.333334, i * 33.333334, 2, 2};
      ++observations;
      if (observe(gate, frame, uint64_t(i) * step_ns)) {
        ++ready;
        break;
      }
    }
  }
  const auto elapsed = std::chrono::duration<double, std::nano>(
      std::chrono::steady_clock::now() - start).count();
  const auto heap = allocations - before;
  retained_ready = ready;
  return {elapsed / double(observations), observations, ready, heap};
}
}

void* operator new(size_t size) {
  if (void* p = std::malloc(size ? size : 1)) { ++allocations; return p; }
  throw std::bad_alloc();
}
void* operator new[](size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
void operator delete[](void* p, size_t) noexcept { std::free(p); }
void* operator new(size_t size, std::align_val_t alignment) {
  void* p = nullptr;
  if (posix_memalign(&p, static_cast<size_t>(alignment), size ? size : 1))
    throw std::bad_alloc();
  ++allocations;
  return p;
}
void* operator new[](size_t size, std::align_val_t alignment) {
  return ::operator new(size, alignment);
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, size_t, std::align_val_t) noexcept { std::free(p); }

int main() {
  (void)run(false); (void)run(true); // Exclude warmup and all output allocation.
  std::cout << "{\"schema\":\"camera-startup-perf-v1\",\"cycles_per_trial\":"
            << cycles << ",\"trials\":" << trials << ",\"sequences\":[";
  for (unsigned mode = 0; mode < 2; ++mode) {
    std::array<Trial, trials> measured{};
    std::array<double, trials> timings{};
    for (unsigned i = 0; i < trials; ++i) {
      measured[i] = run(mode != 0);
      timings[i] = measured[i].ns;
      if (measured[i].ready != cycles || measured[i].heap) return 1;
    }
    std::sort(timings.begin(), timings.end());
    if (mode) std::cout << ',';
    std::cout << "{\"name\":\"" << (mode ? "counter_resets" : "valid_advancing")
              << "\",\"median_ns_per_observation\":" << timings[trials / 2]
              << ",\"samples\":[";
    for (unsigned i = 0; i < trials; ++i) {
      if (i) std::cout << ',';
      const auto& t = measured[i];
      std::cout << "{\"ns_per_observation\":" << t.ns
                << ",\"observations\":" << t.observations
                << ",\"ready\":" << t.ready << ",\"heap_allocations\":"
                << t.heap << '}';
    }
    std::cout << "]}";
  }
  std::cout << "],\"scope\":\"actual cold gate plus out-of-line call; no SDK, ROS, GPU, image copies, wall wait or control workload\"}\n";
}
