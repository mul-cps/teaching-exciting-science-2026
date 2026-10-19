// Fixed actual-image inference benchmark, excluding model/input load and file IO.
#include <gpd/classifier.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 4) return 2;
  try {
    unsetenv("STATION_GPD_CLASSIFIER_AUDIT_DIR");
    cv::setNumThreads(1);
    nlohmann::json audit; std::ifstream(argv[2]) >> audit;
    const auto shape = audit.at("shape").get<std::vector<int>>();
    if (shape.size() != 4 || shape[0] < 1 || shape[0] > 512 || shape[1] != 60 || shape[2] != 60 || shape[3] != 15)
      throw std::invalid_argument("bounded actual GPD images required");
    auto path = std::filesystem::path(argv[2]); path.replace_extension(".inputs.u8");
    if (std::filesystem::file_size(path) != static_cast<size_t>(shape[0])*60*60*15)
      throw std::invalid_argument("actual GPD image byte count mismatch");
    std::ifstream stream(path, std::ios::binary); stream.exceptions(std::ios::failbit | std::ios::badbit);
    std::vector<cv::Mat> images;
    for (int n = 0; n < shape[0]; ++n) {
      cv::Mat image(60, 60, CV_8UC(15));
      stream.read(reinterpret_cast<char*>(image.data), 60*60*15);
      images.push_back(image);
    }
    auto classifier = Classifier::create(argv[1], "", Classifier::Device::eCPU);
    std::vector<double> timings; std::vector<float> scores;
    for (int repeat = 0; repeat < 6; ++repeat) {
      const auto start = std::chrono::steady_clock::now();
      scores = classifier->classifyImages(images);
      const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
      if (repeat) timings.push_back(elapsed);
    }
    nlohmann::json result = {{"images", images.size()}, {"samples_ms", timings}, {"scores", scores},
      {"opencv", CV_VERSION}, {"batch_size", 8}, {"cpu_only", true}, {"audit_io_in_timing", false}};
    std::ofstream(argv[3]) << result.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
