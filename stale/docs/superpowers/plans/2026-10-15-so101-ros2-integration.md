# SO-101 ROS 2 integration plan

1. Pin the upstream stack and recursively pinned Feetech driver; keep fetched source outside Git. Export the Rig01 calibration only as a read-only servo-ID/joint-name map.
2. Add a small launch overlay and host wrapper. Make mock RViz/MoveIt the default; expose real hardware only as an explicit, separately gated mode.
3. Add tests for pins, calibration provenance, actual joint/controller names, mock default, and hardware write hazards. Document Fedora/container setup and the exact limitations of the 5-DOF group.
4. Build and run the pinned workspace in the Jazzy container. Verify RViz, controller graph, and bounded 5-DOF planning/IK behavior before committing only owned files.

The upstream Feetech driver must remain out of any real-hardware startup path until its unconditional EEPROM configuration writes are guarded or replaced with an audited read-only/configuration policy.
