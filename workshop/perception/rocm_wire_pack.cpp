// Bounded wire packing only. Projection/occlusion compatibility is unchanged.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

extern "C" int rocm_wire_pack_abi() { return 1; }

extern "C" int rocm_wire_pack(const float* xyz, const std::uint32_t* rgb,
    const std::uint8_t* valid, std::size_t count, std::uint8_t* output,
    std::size_t capacity, std::size_t* output_count) {
    if (output_count) *output_count = 0;
    const std::uint16_t endian = 1;
    if (sizeof(float) != 4 || *reinterpret_cast<const std::uint8_t*>(&endian) != 1) return -4;
    if (!xyz || !rgb || !valid || !output || !output_count || count > 921600
        || capacity < count*20) return -1;
    std::size_t written = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (valid[i] > 1) return -2;
        if (!valid[i]) continue;
        const float* point = xyz+3*i;
        if (!std::isfinite(point[0]) || !std::isfinite(point[1])
            || !std::isfinite(point[2]) || point[2] <= 0) return -3;
        std::uint8_t* destination = output+20*written;
        std::memcpy(destination, point, 12);       // Exact IEEE float32 bits/order.
        std::memset(destination+12, 0, 4);         // Canonical XYZ padding.
        std::memcpy(destination+16, rgb+i, 4);     // Exact RGB bits, not float conversion.
        ++written;
    }
    // A partial prefix is never a result on any fault: publish only success.
    *output_count = written;
    return 0;
}
