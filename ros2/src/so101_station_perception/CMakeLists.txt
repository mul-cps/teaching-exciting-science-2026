cmake_minimum_required(VERSION 3.8)
project(so101_station_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
# Reuse the repository's tested ROI-independent tag detector/tracker source in
# a private namespace, avoiding workshop/__init__.py application imports.
ament_python_install_package(so101_station_perception_tag_tracking
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../workshop/digital_twin")
ament_python_install_package(so101_station_projection
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../workshop/perception")
add_library(so101_rocm_bridge_runtime SHARED
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../workshop/perception/projection_compat.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../workshop/perception/rocm_wire_pack.cpp")
target_compile_features(so101_rocm_bridge_runtime PRIVATE cxx_std_17)
target_compile_options(so101_rocm_bridge_runtime PRIVATE -O2)
install(TARGETS so101_rocm_bridge_runtime LIBRARY DESTINATION lib)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../config/realsense.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS
  scripts/calibration_tf_publisher
  scripts/periodic_registration
  scripts/registration_capture_sources
  scripts/camera_capture_sources
  scripts/virtual_gripper_camera
  scripts/diagnostic_pointcloud
  scripts/tag_observations
  scripts/live_tag_alignment
  scripts/rgbd_evidence_sink
  scripts/charuco_world
  scripts/marker_pipeline
  scripts/rocm_rgbd
  scripts/perception_recovery
  scripts/perception_supervisor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
