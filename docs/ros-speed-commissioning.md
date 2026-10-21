# Named ROS speed commissioning profiles

Select one immutable startup profile in the complete station JSON:

```json
{"arm": {"ros_motion_profile": "v250"}}
```

Merge the field into the existing `arm` object. Omitting it selects `default`.
Unknown names, null, booleans, numbers and other malformed values are rejected.
Selection does not change while an owner generation is running.

| Name | Velocity rad/s | Acceleration rad/s² | Jerk rad/s³ |
| --- | ---: | ---: | ---: |
| default | 0.125 | 0.2 | 2 |
| v187 | 0.1875 | 0.2 | 2 |
| v250 | 0.25 | 0.2 | 2 |
| v250-a300 | 0.25 | 0.3 | 2 |
| v250-a400 | 0.25 | 0.4 | 2 |
| v375-a400 | 0.375 | 0.4 | 2 |

Nondefault profiles require every body joint's configured `arm.limits` speed
and acceleration to authorize at least the selected values, converted to
degrees/s and degrees/s². The existing global ceilings remain 25 degrees/s
and 50 degrees/s². For example, the largest tier requires speed at least
21.485917318 degrees/s and acceleration at least 22.918311806 degrees/s²;
22 and 23 authorize it. These permissions do not change joint position bounds.
Gripper limits retain their existing percent-based interpretation.

The resident ROS path previously used its independent fixed dynamics envelope;
`arm.limits` speed/acceleration primarily governed the separate LeRobotArm motion
path. Nondefault tier resolution now explicitly checks these permissions. A
high permission value cannot select a faster tier by itself.

## Approved default pace

The user approved the observed `speed-clearance-baseline` move as the default
pace: its exact two-knot trajectory duration was 4.092212123 seconds and its
maximum measured endpoint error was 1.637362637 degrees. The helper paced it
at the default profile limits (0.125 rad/s velocity, 0.2 rad/s² acceleration,
2 rad/s³ jerk) under `v375-a400` commissioning admission. The selected
persistent profile is now `default` with the same native body Goal_Velocity
160. No faster pacing was physically tested.

MoveGroup and RViz now start with velocity and acceleration scaling 1.0, using
the selected profile's full planning envelope. The profile defaults and native
settings remain unchanged. This preserves trajectory construction and all
admission guards; dense RViz plans can still require uniform timing dilation
under the same C2 spline and jerk checks, so their observed pace can be slower
than this two-knot baseline.

## Owner, bridge and planner coherence

The owner and bridge independently resolve the same station JSON through
`workshop.station.ros_trajectory.resolve_ros_motion_profile`. The helper is now
included in the owner source hash map. The bridge also checks that the exact
helper bytes it loaded match the source pinned before owner startup.

The owner protocol's `execute_trajectory` command requires
`ros_motion_profile` with the exact startup-selected name. Both ordinary and
supervised bridge dispatches include it. Missing or mismatched profiles reject
before trajectory bus writes; supervised calibration retains its existing
slower dynamics. Commands do not expose arbitrary velocity or acceleration
values. Existing direct Python calls retain the default profile unless their
owner selected a different tier, which requires an explicitly matching name.

`moveit_resident.launch.py` resolves the profile before constructing its nodes
and sets matching velocity, acceleration and jerk limits for all five arm
joints in both MoveGroup and RViz. Pass the same `station_root` and
`station_config` used by the bridge. Changing generic ROS parameters does not
change the owner's selected tier. RViz scaling 1.0 requests the selected planner
ceiling; it does not guarantee that the physical trajectory reaches that speed.

Bridge collision artifacts record the profile and dynamics limits together
with the exact retimed command points and effective path digest. The owner
independently reconstructs and validates that path under its selected limits.
Original input evidence, C2 reconstruction, analytic jerk checks, collision
checking, measured-start/path-digest checks, torque limits, timing/freshness,
tracking/stop guards and the 2-degree arrival condition remain in force.
Uniform timing dilation can still reduce speed when the reconstructed spline's
jerk is limiting. Do not classify a calibration-held endpoint as success.

## Native servo settings and Phase evidence

This feature does not raise native servo settings: acceleration stays 8,
Goal_Velocity stays 160 for body/220 for gripper, and Torque_Limit stays
700 for body/250 for gripper. It makes no PID, Lock, Phase or EEPROM writes.

The existing passive `/follower/joint_diagnostics` inventory additionally reads
`Phase` after `Goal_Position`, using the SDK's address 18, one-byte register.
The same 80 ms acquisition budget and required final position/torque readback
apply. Unsupported, read-error or budget-exhausted Phase observations are
explicit; absence is not an assumed value. Values retain raw SDK units.

Phase and measured tracking evidence are prerequisites for any later proposal
to change native velocity. No native240/320 profile or runtime tier-switch API
is included here.

## Controlled cutover

Restore and verify any pending elbow P24 EEPROM journal using its original
frozen owner source before changing owner sources. Adding the trajectory helper
to the source map changes the pinned cohort; do not edit or deploy over an
active pending trial. Preserve its restoration receipt, then start the new
owner with an explicitly selected and authorized tier. This profile is a
commissioning envelope, not a qualification of maximum physical speed.
