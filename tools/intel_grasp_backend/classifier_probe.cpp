// Deterministic CPU compatibility check, including a final partial inference batch.
#include <gpd/classifier.h>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    cv::setNumThreads(1);
    auto classifier = Classifier::create(argv[1], "", Classifier::Device::eCPU);
    std::vector<cv::Mat> images;
    for (int n = 0; n < 9; ++n) {
      cv::Mat image(60, 60, CV_8UC(15));
      for (int i = 0; i < 60*60*15; ++i) image.data[i] = n == 0 ? 0 : (i*37+7*n)%256;
      images.push_back(image);
    }
    const auto start = std::chrono::steady_clock::now();
    const auto scores = classifier->classifyImages(images);
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
    nlohmann::json report = {{"scores", scores}, {"elapsed_ms", elapsed},
      {"cases", "zero plus 8 deterministic uint8 HWC15 images"}, {"qualified", false}};
    std::ofstream(argv[2]) << report.dump(2) << '\n';
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
