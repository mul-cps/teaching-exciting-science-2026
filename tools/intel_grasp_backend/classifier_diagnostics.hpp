#pragma once
#include <cstddef>
struct StationClassifierDiagnostics {
  std::size_t scored_images = 0;
  std::size_t images_at_original_threshold = 0;
  float minimum_score = 0.f;
  float maximum_score = 0.f;
};
StationClassifierDiagnostics station_classifier_diagnostics();
