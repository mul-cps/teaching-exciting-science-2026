#pragma once
// Strict, bounded station-export decoder. Never invokes the PCL file parser.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct StationPcdPoint { float x, y, z; std::uint32_t rgba; };
inline std::vector<StationPcdPoint> read_station_pcd(const std::string& path) {
  constexpr std::size_t max_points = 1000000, max_header = 4096;
  constexpr std::size_t max_bytes = max_header + 16*max_points;
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  auto fail = [] { throw std::runtime_error("invalid bounded station binary PCD schema/count/storage/finite XYZ"); };
  if (!input) fail();
  const auto length = input.tellg();
  if (length <= 0 || length > static_cast<std::streamoff>(max_bytes)) fail();
  input.seekg(0);
  std::string data(static_cast<std::size_t>(length), '\0');
  if (!input.read(data.data(), length)) fail();
  const std::string marker = "DATA binary\n";
  const auto end = data.find(marker);
  if (end == std::string::npos || end+marker.size() > max_header) fail();
  const auto offset = end+marker.size();
  std::map<std::string, std::vector<std::string>> fields;
  std::istringstream header(data.substr(0, offset));
  std::string line;
  while (std::getline(header, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream parts(line); std::string key, value;
    parts >> key;
    if (key.empty() || fields.count(key)) fail();
    std::vector<std::string> values;
    while (parts >> value) values.push_back(value);
    fields.emplace(key, values);
  }
  const std::map<std::string, std::vector<std::string>> fixed = {
    {"FIELDS", {"x", "y", "z", "rgba"}}, {"SIZE", {"4", "4", "4", "4"}},
    {"TYPE", {"F", "F", "F", "U"}}, {"COUNT", {"1", "1", "1", "1"}},
    {"VIEWPOINT", {"0", "0", "0", "1", "0", "0", "0"}}, {"DATA", {"binary"}}};
  if (fields.size() != 10 || (fields["VERSION"] != std::vector<std::string>{".7"}
      && fields["VERSION"] != std::vector<std::string>{"0.7"})) fail();
  for (const auto& entry : fixed) if (fields[entry.first] != entry.second) fail();
  auto number = [&](const std::string& name) {
    const auto& values = fields[name];
    if (values.size() != 1 || values[0].empty() || values[0].size() > 7) fail();
    for (char c : values[0]) if (c < '0' || c > '9') fail();
    return static_cast<std::size_t>(std::stoull(values[0]));
  };
  const auto width = number("WIDTH"), height = number("HEIGHT"), points = number("POINTS");
  if (!width || width > max_points || !height || height > max_points || !points
      || points > max_points || width*height != points || data.size()-offset != points*16) fail();
  // Decode explicitly as little endian, independent of host endianness/alignment.
  auto u32 = [&](std::size_t position) {
    const auto* b = reinterpret_cast<const unsigned char*>(data.data()+position);
    return std::uint32_t(b[0]) | (std::uint32_t(b[1])<<8) | (std::uint32_t(b[2])<<16) | (std::uint32_t(b[3])<<24);
  };
  std::vector<StationPcdPoint> result; result.reserve(points);
  for (std::size_t i = 0; i < points; ++i) {
    StationPcdPoint p{};
    float* coordinates[] = {&p.x, &p.y, &p.z};
    for (std::size_t j = 0; j < 3; ++j) {
      const auto bits = u32(offset+i*16+j*4);
      std::memcpy(coordinates[j], &bits, 4);
      if (!std::isfinite(*coordinates[j])) fail();
    }
    p.rgba = u32(offset+i*16+12); result.push_back(p);
  }
  return result;
}
