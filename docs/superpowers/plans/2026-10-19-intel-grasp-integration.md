# Optional Intel grasp planning integration

Assumption: reuse Intel's ROS service boundary for experimental grasp proposals;
retain the station's existing geometric planner and every commissioning/motion gate.
No robot, camera, detector or calibration process is started by this integration.

1. Pin the inspected upstream revision and document dependencies/API limitations.
2. Add a default-off `moveit_msgs/srv/GraspPlanning` station adapter for an independently
   installed Intel backend. Bound outstanding work to one request with a hard deadline.
   Reject stale, malformed or differently framed requests/results. No automatic retry.
3. Publish accepted backend poses on isolated proposal topics, with source stamps and
   explicit unqualified/non-executable metadata. Do not inject these into sorting.
   Intel's EEF offsets and jaw postures are robot-specific and remain unqualified.
4. Add an opt-in launch and ROS-only quick action. Backend absence is an explicit
   service error. Document validation handoff and hand-eye prerequisites.
5. Verify with synthetic ROS services, including timeout/late completion, and a
   focused bounded-cost benchmark/static guard. Do not claim real detector or pick/place
   qualification. Root reviews and owns commits/deployment.
