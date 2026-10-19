// Station CPU compatibility backend for the pinned GPD Classifier interface.
// Uses the original bundled Caffe weights, never a synthetic grasp classifier.
#include <gpd/classifier.h>
#include "classifier_diagnostics.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace {
StationClassifierDiagnostics diagnostics;
class OpenCvCpuClassifier final : public Classifier {
 public:
  OpenCvCpuClassifier(const std::string& model, const std::string& weights, Device device) {
    diagnostics = {};
    if (device != Device::eCPU) throw std::invalid_argument("offline reference supports CPU only");
    cv::ocl::setUseOpenCL(false);
    net_ = cv::dnn::readNetFromONNX(model);
    (void)weights;  // Exact original weights are embedded in the hash-pinned ONNX.
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    const int shape[] = {batch_size_, 15, 60, 60};
    input_ = cv::Mat(4, shape, CV_32F);
    planes_.resize(15);
    if (const char* path = std::getenv("STATION_GPD_CLASSIFIER_AUDIT_DIR")) {
      audit_root_ = path;
      if (!audit_root_.empty()) std::filesystem::create_directories(audit_root_);
    }
  }
  int getBatchSize() const override { return batch_size_; }
  std::vector<float> classifyImages(const std::vector<cv::Mat>& images) override {
    std::vector<float> scores;
    scores.reserve(images.size());
    std::vector<std::array<float, 2>> audit_logits;
    if (!audit_root_.empty()) {
      if (images.size() > 512) throw std::invalid_argument("audit image count exceeds 512");
      audit_logits.reserve(images.size());
    }
    for (size_t start = 0; start < images.size(); start += batch_size_) {
      const size_t count = std::min<size_t>(batch_size_, images.size()-start);
      input_.setTo(0);  // Fixed batch shape: final padding is not returned as a grasp.
      for (size_t n = 0; n < count; ++n) {
        const auto& image = images[start+n];
        if (image.rows != 60 || image.cols != 60 || image.type() != CV_8UC(15))
          throw std::invalid_argument("expected original GPD uint8 60x60x15 grasp image");
        image.convertTo(float_image_, CV_32F);
        for (int channel = 0; channel < 15; ++channel)
          planes_[channel] = cv::Mat(60, 60, CV_32F, input_.ptr<float>()+(n*15+channel)*3600);
        cv::split(float_image_, planes_);
      }
      // GPD's legacy classifier does not apply RGB normalization or softmax.
      net_.setInput(input_);
      const cv::Mat output = net_.forward();
      if (output.total() != batch_size_*2 || output.type() != CV_32F || !cv::checkRange(output))
        throw std::runtime_error("nonfinite or unexpected GPD classifier output");
      const float* logits = output.ptr<float>();
      for (size_t n = 0; n < count; ++n) {
        const float score = logits[2*n+1]-logits[2*n];
        if (!std::isfinite(score)) throw std::runtime_error("nonfinite classifier score");
        if (diagnostics.scored_images == 0) diagnostics.minimum_score = diagnostics.maximum_score = score;
        diagnostics.minimum_score = std::min(diagnostics.minimum_score, score);
        diagnostics.maximum_score = std::max(diagnostics.maximum_score, score);
        ++diagnostics.scored_images;
        if (score >= 500.f) ++diagnostics.images_at_original_threshold;
        scores.push_back(score);
        if (!audit_root_.empty()) audit_logits.push_back({logits[2*n], logits[2*n+1]});
      }
    }
    if (!audit_root_.empty()) {
      const auto prefix = audit_root_ / ("batch_"+std::to_string(audit_sequence_++));
      std::ofstream inputs(prefix.string()+".inputs.u8", std::ios::binary);
      inputs.exceptions(std::ios::failbit | std::ios::badbit);
      for (const auto& image : images)
        for (int row = 0; row < image.rows; ++row)
          inputs.write(reinterpret_cast<const char*>(image.ptr(row)), image.cols*image.elemSize());
      nlohmann::json metadata = {{"shape", {images.size(), 60, 60, 15}},
        {"layout", "NHWC"}, {"dtype", "uint8"}, {"input_scale", 1.},
        {"score_formula", "positive_logit[1] minus negative_logit[0]"},
        {"logits", audit_logits}, {"scores", scores}, {"qualified", false}};
      std::ofstream output(prefix.string()+".json");
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output << metadata.dump(2) << '\n';
    }
    return scores;
  }
 private:
  static constexpr int batch_size_ = 8;
  cv::dnn::Net net_;
  cv::Mat input_, float_image_;
  std::vector<cv::Mat> planes_;
  std::filesystem::path audit_root_;
  unsigned audit_sequence_ = 0;
};
}

StationClassifierDiagnostics station_classifier_diagnostics() { return diagnostics; }

std::shared_ptr<Classifier> Classifier::create(const std::string& model,
                                              const std::string& weights, Device device) {
  return std::make_shared<OpenCvCpuClassifier>(model, weights, device);
}
