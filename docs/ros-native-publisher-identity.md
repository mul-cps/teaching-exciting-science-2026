# Isolated native ROS publisher identity support

The pinned Conda rclpy7.1.12 omits publisher GIDs from Python message metadata;
Cyclone RMW2.2.4 additionally reports a publication handle instead of its RTPS
GUID. The reproducible isolated builds are:

```bash
ROS_DOMAIN_ID=75 ros2/conda/build-rclpy-publisher-gid.sh
ROS_DOMAIN_ID=75 ros2/conda/build-rmw-cyclonedds-publisher-gid.sh
```

They pin upstream revisions and patch bytes, build into separate `ros2/.cache`
prefixes, and verify that installed Conda binaries remain unchanged. The wrapper
validates both provenance markers before prepending the isolated Python/native
paths after environment/workspace activation. The patched backend has the
explicit filename and implementation identifier `rmw_cyclonedds_cpp_so101_gid`;
this avoids Conda RPATH resolving the identically named stock backend first.
CDR, introspection type support and Cyclone DDS wire behavior remain unchanged.
An explicit non-Cyclone implementation selection is preserved. Existing live
processes are unchanged by building an overlay.

The native patch looks up the actual matched publication GUID and frees the
returned endpoint metadata. Missing/disappeared endpoint metadata yields an
unavailable zero GID, never a handle or graph attribution fallback. Tests cover
typed/serialized messages, recreated publishers, and stock↔patched processes;
`/proc/self/maps` proves the loaded binary paths. This transport proof does not
qualify camera registration, geometry contacts, or continuous recovery.

## Validation scope

Native tests compare actual message and graph GUIDs across recreated publishers
and bidirectional stock/patched processes. The wrapper also has a real nested
Conda activation regression. These checks use domain75 with simulated publishers;
they do not open the robot or camera.

The independent 2,000-message Python/DDS diagnostic measured stock/patched means
of51.72/54.96 microseconds and p95 values63.89/65.63 microseconds. This includes
Python publication, take and callback overhead; it is not an isolated native-call
benchmark or a physical control-loop timing guarantee.

Camera identity, registration, geometry and full recovery have separate gates.
See the [acceptance register](ros-sorting-acceptance.md).
