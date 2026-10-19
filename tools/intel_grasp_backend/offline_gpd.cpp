// Offline actual upstream GPG/GPD reference. No ROS, camera or robot endpoints.
#include <gpd/grasp_detector.h>
#include "classifier_diagnostics.hpp"
#include "pcd_guard.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: offline_gpd SOURCE.pcd PINNED_MODEL.onnx OUTPUT.json\n";
    return 2;
  }
  try {
    const auto points = read_station_pcd(argv[1]);  // Validate/decode before PCL or model construction.
    cv::setNumThreads(1);
    GraspDetector::GraspDetectionParameters p{};
    p.model_file_ = argv[2]; p.device_ = 0;
    p.min_score_diff_ = 500.; p.create_image_batches_ = false;  // Upstream default; its image-batch branch indexes all candidates.
    p.workspace_ = {-1., 1., -1., 1., -1., 1.};
    p.gripper_width_range_ = {.03, .10}; p.min_inliers_ = 1; p.num_selected_ = 5;
    p.generator_params.voxelize_ = true;
    p.generator_params.num_samples_ = 60; p.generator_params.num_threads_ = 2;
    p.generator_params.workspace_ = p.workspace_;
    auto& hand = p.hand_search_params;
    hand.finger_width_ = .005; hand.hand_outer_diameter_ = .12;
    hand.hand_depth_ = .06; hand.hand_height_ = .02; hand.init_bite_ = .01;
    hand.nn_radius_frames_ = .01; hand.num_samples_ = 60; hand.num_threads_ = 2;
    hand.num_orientations_ = 8; hand.rotation_axis_ = 2;
    hand.cam_tf_left_.setIdentity(); hand.cam_tf_right_.setIdentity();
    p.image_params.outer_diameter_ = .12; p.image_params.depth_ = .06;
    p.image_params.height_ = .02; p.image_params.size_ = 60; p.image_params.num_channels_ = 15;
    Eigen::Matrix3Xd viewpoints(3, 1); viewpoints.setZero();
    PointCloudRGB::Ptr decoded(new PointCloudRGB);
    decoded->resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      auto& p = (*decoded)[i];
      p.x = points[i].x; p.y = points[i].y; p.z = points[i].z; p.rgba = points[i].rgba;
    }
    CloudCamera cloud(decoded, static_cast<int>(decoded->size()), viewpoints);
    if (cloud.getCloudOriginal()->empty() || cloud.getCloudOriginal()->size() > 1000000)
      throw std::runtime_error("recorded point cloud must contain 1..1000000 points");
    const auto begin = std::chrono::steady_clock::now();
    GraspDetector detector(p);
    const auto loaded = std::chrono::steady_clock::now();
    detector.preprocessPointCloud(cloud);
    const auto prepared = std::chrono::steady_clock::now();
    const auto grasps = detector.detectGrasps(cloud);
    const auto finished = std::chrono::steady_clock::now();
    auto milliseconds = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b-a).count(); };
    nlohmann::json report = {{"schema", "station-offline-upstream-gpd-v1"},
      {"backend", "upstream GPG/GPD with original weights and OpenCV DNN CPU compatibility"},
      {"qualified", false}, {"execution_enabled", false}, {"target_association_verified", false},
      {"tool_mapping_qualified", false}, {"source_pcd", argv[1]}, {"model_onnx", argv[2]},
      {"original_points", cloud.getCloudOriginal()->size()},
      {"processed_points", cloud.getCloudProcessed()->size()},
      {"parameters", {{"samples", 60}, {"orientations", 8}, {"threads", 2},
        {"min_score_diff", 500.}, {"hand_outer_diameter_m", .12}, {"hand_depth_m", .06}}},
      {"timing_ms", {{"model_load", milliseconds(begin, loaded)},
        {"preprocess", milliseconds(loaded, prepared)}, {"detect", milliseconds(prepared, finished)},
        {"total", milliseconds(begin, finished)}}}, {"grasps", nlohmann::json::array()}};
    auto xyz = [](const Eigen::Vector3d& v) {
      if (!v.allFinite()) throw std::runtime_error("nonfinite actual upstream grasp vector");
      return std::vector<double>{v.x(), v.y(), v.z()};
    };
    const auto diagnostics = station_classifier_diagnostics();
    for (const auto& grasp : grasps) {
      if (!std::isfinite(grasp.getScore()) || !std::isfinite(grasp.getGraspWidth()))
        throw std::runtime_error("nonfinite actual upstream grasp score/width");
      report["grasps"].push_back({{"bottom", xyz(grasp.getGraspBottom())},
        {"surface", xyz(grasp.getGraspSurface())}, {"approach", xyz(grasp.getApproach())},
        {"binormal", xyz(grasp.getBinormal())}, {"axis", xyz(grasp.getAxis())},
        {"width_m", grasp.getGraspWidth()}, {"score", grasp.getScore()},
        {"origin", diagnostics.images_at_original_threshold <= 2 ? "upstream_geometric_fallback" : "upstream_neural_classified"},
        {"neural_accepted", diagnostics.images_at_original_threshold > 2},
        {"qualified", false}});
    }
    // Upstream detectGrasps can fall back to unclassified candidate hypotheses.
    // Its stdout log must accompany this artifact; nonempty output is not a
    // calibrated neural-confidence or physical success claim.
    report["classification"] = {{"scored_images", diagnostics.scored_images},
      {"images_at_original_threshold", diagnostics.images_at_original_threshold},
      {"original_threshold", 500.}, {"minimum_score", diagnostics.minimum_score},
      {"maximum_score", diagnostics.maximum_score},
      {"used_geometric_fallback", diagnostics.images_at_original_threshold <= 2}};
    std::ofstream output(argv[3]); output.exceptions(std::ios::failbit | std::ios::badbit);
    output << report.dump(2) << '\n';
    return grasps.empty() ? 3 : 0;
  } catch (const std::exception& error) {
    std::cerr << "offline_gpd: " << error.what() << '\n';
    return 1;
  }
}
