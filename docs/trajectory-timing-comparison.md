# Offline trajectory timing comparison

Keep the current C2 quintic bridge and 50 Hz exact-sample admission. The bounded
comparison below found no benefit from adding TOTG + Ruckig before this bridge,
and rejected the direct derivative interpolation candidate. Nothing here changes
controller code, runtime configuration, gains, defaults or hardware ownership.

## Measured result (2026-10-21)

Two existing speed-qualification artifacts supplied identical geometric joint
paths to every arm. The first waypoint was anchored to the artifact's measured
start, exactly as the bridge does. All arms used v=0.125 rad/s, a=0.2 rad/s²,
j=2 rad/s³. No faster profile was tested. The second artifact was originally
recorded under a faster profile; this comparison reconstructs it at baseline
limits. A is the **current bridge reconstruction**, not a claim that the old
recording executed these exact samples.

| Frozen path | A current bridge | B TOTG → Ruckig → bridge | C final p/v/a → quintic Hermite |
|---|---:|---:|---:|
| clearance (39 input points) | 6.04 s, pass | 6.04 s, identical bytes | 9.14 s, reject |
| elbow 5° (2 input points) | 1.64 s, pass | 1.64 s, identical bytes | 6.70 s, reject |

Durations include the next 20 ms terminal hold tick. A and B share exact sample
SHA256 hashes for each path. Both paths are collinear: the existing bridge
collapses the extra TOTG/Ruckig waypoints to the same admitted rest-to-rest
quintic. This does not establish equivalence on cornered paths.

C preserves the final MoveIt waypoint positions, velocities and accelerations
using a C2 quintic Hermite polynomial between knots. It is **not** the internal
continuous Ruckig trajectory. Its command-sample finite-difference peak jerks
were 107.88 and 81.78 rad/s³; exact polynomial peak jerks were 301.21 and 230.70
rad/s³. It introduced two reversals per moving joint and approximately 0.061 rad
joint-space path deviation on the elbow path. Therefore, copying derivatives
into a generic interpolation layer does not preserve the smoother's guarantees.
Final knot accelerations were nonzero, which also needs explicit terminal-hold
handling before any direct derivative execution could be admitted.

Single-call TOTG + smoothing wall times were about 2.79 ms and 0.56 ms on this
host (MoveIt 2.12.4, Ruckig 0.9.2). They are diagnostics, not latency percentiles.
Python bridge wall times include parsing the existing source to extract the pure
function and sample/metric/export work; they are not controller tick costs.

The frozen [report](../tools/trajectory_benchmark/evidence/2026-10-21/report.json)
contains exact 50 Hz sample exports, hashes, input and model provenance, backend
versions, v/a/j, reversals, waypoint/polyline deviations and processing wall time.
No collision check, ROS runtime request or hardware motion was performed.
**Every exported candidate remains NOT owner-admissible and NOT collision-approved.**
The existing complete exact-sample collision and owner preflight must admit any
future execution; these files confer no execution authority.

## What the implementation actually calls

The standalone C++ executable constructs a `RobotModel` and `RobotTrajectory`,
then calls installed `TimeOptimalTrajectoryGeneration::computeTimeStamps` and
`RuckigSmoothing::applySmoothing` in that order. It never initializes a ROS node,
executor, service or action. No separate Ruckig state-to-state generator is used.
TOTG tolerance is 0.001 rad with 0.1 s resampling and 0.001 rad minimum angle
change. Smoothing uses explicit limits and overshoot mitigation at 0.001 rad.
These are frozen experiment settings, not changes to resident-launch adapters.

The [MoveIt tutorial](https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html)
describes TOTG followed by Ruckig response adapters and warns that fitted paths
may require new collision checks. Older request-adapter lists have different
ordering syntax. The installed 2.12.4 headers were checked, together with the
[matching smoothing source](https://github.com/moveit/moveit2/blob/2.12.4/moveit_core/trajectory_processing/src/ruckig_traj_smoothing.cpp).
This tool calls the library order explicitly; it does not infer resident adapter
configuration from documentation examples.

## Reproduce without ROS runtime

Run from repository root. Use the installed Jazzy environment; the following
commands only load environment variables, compile and run local library code.
The frozen URDF/SRDF already include the same offsets and end-effector correction
as the resident launch. `freeze_model.py` can regenerate them from installed
SO101 description/config shares and `model_offsets.yaml`; it records all source
model hashes. Meshes are not used for collision checking in this experiment.

```bash
source /home/bjoern/miniforge3/envs/so101-jazzy/setup.bash
prefix=/home/bjoern/miniforge3/envs/so101-jazzy
"$prefix/bin/cmake" -S tools/trajectory_benchmark -B /tmp/timing-build \
  -DCMAKE_PREFIX_PATH="$prefix" \
  -DCMAKE_CXX_COMPILER="$prefix/bin/x86_64-conda-linux-gnu-c++" \
  -DPython3_EXECUTABLE="$prefix/bin/python"
"$prefix/bin/cmake" --build /tmp/timing-build -j2
frozen=tools/trajectory_benchmark/evidence/2026-10-21
"$prefix/bin/python" tools/trajectory_benchmark/compare.py \
  --artifact "$frozen/case-00/input.json" \
  --artifact "$frozen/case-01/input.json" \
  --urdf "$frozen/model.urdf" --srdf "$frozen/model.srdf" \
  --backend /tmp/timing-build/moveit_timing --prefix "$prefix" \
  --config "$frozen/model-sources.json" --output /tmp/timing-results-new
"$prefix/bin/python" tools/trajectory_benchmark/verify_exports.py /tmp/timing-results-new
```

Choose a new output directory for every run. Samples should reproduce byte for
byte with the pinned sources/libraries; timing measurements can differ.
`verify_exports.py` checks this experiment's two-path acceptance/rejection result,
sample hashes and 50 Hz grid. It is not a general trajectory admission checker.
The metrics use finite differences of commanded positions, including three
stationary samples at both boundaries. C also reports exact segment polynomial
extrema; neither metric is measured motor dynamics. Path distances are L2 radians
in joint space, not Cartesian clearance.

## Simple motion modes

`tools/trajectory_benchmark/motion_profiles.yaml` supplies conservative initial
MoveIt request velocity/acceleration scalings for transfer, approach, contact,
loaded lift, placement and home. These suggestions have not been physically
qualified with a load. For example:

```bash
python tools/trajectory_benchmark/motion_request.py loaded_lift
# {"max_velocity_scaling_factor": 0.35, "max_acceleration_scaling_factor": 0.25}
```

Merge those two fields into an existing `MotionPlanRequest` before planning.
They scale the current configured limits and must never raise the baseline
admission envelope. Full bridge/owner preflight remains mandatory. In particular,
collinear collapse may select its own duration within existing bridge limits;
request scaling alone does not guarantee a slower executed phase. Where an
executed phase ceiling matters, it must be enforced and admitted by the owner
before these presets are used for physical contact. No preset enables force
control or contact detection. Adaptive clocks, online compensation, controller
gain changes and a ros2_control migration are deferred.
