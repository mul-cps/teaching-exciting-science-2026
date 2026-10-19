cmake_minimum_required(VERSION 3.16)
project(station_intel_gpd_offline LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT GPG_SOURCE OR NOT GPD_SOURCE)
  message(FATAL_ERROR "GPG_SOURCE and GPD_SOURCE must name pinned isolated upstream clones")
endif()
find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIRS}")
find_package(Boost REQUIRED COMPONENTS thread)
add_subdirectory("${GPG_SOURCE}" upstream_gpg EXCLUDE_FROM_ALL)
get_property(GPG_TARGETS DIRECTORY "${GPG_SOURCE}" PROPERTY BUILDSYSTEM_TARGETS)
foreach(target IN LISTS GPG_TARGETS)
  target_compile_options(${target} PRIVATE -include "${CMAKE_CURRENT_SOURCE_DIR}/compat.hpp")
  target_link_libraries(${target} Eigen3::Eigen Boost::thread)
endforeach()
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui)
find_package(OpenMP REQUIRED)
find_package(nlohmann_json REQUIRED)
set(GPD_IMPL "${GPD_SOURCE}/src/gpd")
add_library(station_grasp_pose_detection SHARED
  opencv_classifier.cpp
  "${GPD_IMPL}/clustering.cpp"
  "${GPD_IMPL}/grasp_detector.cpp"
  "${GPD_IMPL}/learning.cpp"
  "${GPD_IMPL}/sequential_importance_sampling.cpp"
  "${GPD_IMPL}/grasp_image.cpp"
  "${GPD_IMPL}/grasp_image_15_channels.cpp")
target_include_directories(station_grasp_pose_detection PUBLIC
  "${GPD_SOURCE}/include" "${GPG_SOURCE}/include" ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_options(station_grasp_pose_detection PRIVATE -include "${CMAKE_CURRENT_SOURCE_DIR}/compat.hpp")
target_link_libraries(station_grasp_pose_detection PUBLIC gpg_grasp_candidates_generator
  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} OpenMP::OpenMP_CXX Eigen3::Eigen nlohmann_json::nlohmann_json)
find_package(nlohmann_json REQUIRED)
add_executable(offline_gpd offline_gpd.cpp)
target_link_libraries(offline_gpd PRIVATE station_grasp_pose_detection nlohmann_json::nlohmann_json)
add_executable(classifier_probe classifier_probe.cpp)
target_link_libraries(classifier_probe PRIVATE station_grasp_pose_detection nlohmann_json::nlohmann_json)
add_executable(classifier_benchmark classifier_benchmark.cpp)
target_link_libraries(classifier_benchmark PRIVATE station_grasp_pose_detection nlohmann_json::nlohmann_json)
