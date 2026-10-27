#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
export PATH=/opt/ros/jazzy/bin:/usr/bin:/bin
export CMAKE_BUILD_PARALLEL_LEVEL=2
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/bjoern/.local/share/exciting-science/validation/highres-20261027/source
colcon --log-base /home/bjoern/.local/share/exciting-science/validation/highres-20261027/log build --base-paths ros2/src --packages-select so101_station_vulkan --build-base /home/bjoern/.local/share/exciting-science/validation/highres-20261027/build --install-base /home/bjoern/.local/share/exciting-science/validation/highres-20261027/install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -DPython3_EXECUTABLE=/usr/bin/python3 -DRMW_IMPLEMENTATION=rmw_fastrtps_cpp -Drealsense2_DIR=/home/bjoern/miniforge3/envs/so101-jazzy/lib/cmake/realsense2 -DPRECOMPILED_SHADER_DIR=/tmp/workshop-fwmini4-camera-shader-caches-20261027/shaders -DPRECOMPILED_ALIGNMENT_SHADER_DIR=/tmp/workshop-highres-alignment-shaders-20261027 -DPRECOMPILED_GEOMETRY_SHADER_DIR=/tmp/workshop-fwmini4-camera-shader-caches-20261027/geometry-shaders -DPRECOMPILED_PARENT_SHADER_DIR=/tmp/workshop-fwmini4-camera-shader-caches-20261027/parent-shaders -DPRECOMPILED_CHILD_SHADER_DIR=/tmp/workshop-fwmini4-camera-shader-caches-20261027/child-shaders -DRESIDENT_SLANG_CPU=OFF
