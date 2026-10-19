# Offline upstream GPD backend feasibility

Stage 1 proposal bridge is committed separately as d4b6727. This work is isolated,
uses recorded PCD/RGBD only and will not open a camera, robot or live ROS endpoint.

1. Pin Intel ros2_grasp_library 980b7ddd, its sharronliu/gpd libgpd engine fdd69930
   and GPG 3dcd656d (upstream pull/7 head), plus model/prototxt hashes and licenses.
2. Attempt a prefix-local CPU build against existing Jazzy Conda PCL/OpenCV first;
   record concrete legacy ABI/API failures. Do not modify the live environment.
3. If legacy OpenVINO 2019 cannot coexist with the isolated modern stack, evaluate
   a small OpenCV DNN CPU classifier port using the exact bundled Caffe weights.
   Its original MemoryData input may be replaced by an explicit identical NCHW
   input; pin and document that graph-only adaptation and validate inference.
4. Run actual upstream candidate generation/classification on a saved PCD; record
   finite grasp output, actual CPU latency, source/weight/data provenance and any
   fallbacks in upstream GPD. A classifier-only synthetic pass is not GPD success.
5. Only after engine proof, expose an optional offline ROS GraspPlanning service
   suitable for independent integration tests. Recorded frames never become fresh
   live sensor observations; preserve qualification/association/tool-mapping gates.
6. Separate performance review and tests precede any GPU validation. CPU-only
   reference does not establish AMD GPU acceleration or physical pickup capability.
