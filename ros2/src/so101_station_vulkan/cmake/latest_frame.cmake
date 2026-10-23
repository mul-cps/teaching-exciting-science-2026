# CPU-only scheduling/actual lease lifetime checks. Does not construct Context.
add_executable(latest_frame_queue_test test/latest_frame_queue_test.cpp)
target_compile_features(latest_frame_queue_test PRIVATE cxx_std_17)
target_link_libraries(latest_frame_queue_test resident_rgbd)
target_compile_options(latest_frame_queue_test PRIVATE -Wall -Wextra)
if(BUILD_TESTING)
 add_test(NAME resident_latest_frame_queue COMMAND latest_frame_queue_test)
endif()
