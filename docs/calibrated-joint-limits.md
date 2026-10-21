# Calibrated planning bounds

The physical model uses encoder q: `(raw - calibration midpoint) * 2*pi/4095`.
Mechanical zero corrections rotate joint origins; they do not shift measured q
or its bounds. The five arm bounds are the integer intersection of calibration,
station configuration, and observed firmware limits. The gripper keeps its
existing unqualified visual proxy; this export does not invent a jaw calibration.

`tools/freeze_ros_joint_limits.py` exports that intersection from a fresh idle
owner without serial access or torque changes. It rejects stale, active,
identity-mismatched and source-mismatched evidence. A contract binds the exact
arm settings, calibration hash and owner source cohort. It remains valid across
restarts with that cohort. Source, configuration, calibration or firmware changes
require regeneration. The bridge compares actual owner admission before checking
any trajectory, and the operator panel displays a mismatch before execution.
URDF, MoveIt/RViz, collision sampling and owner interpolation use identical q
limits. Analytic spline position extrema, collision checks, dynamics, exact path
hashes, stale feedback protection and terminal tolerances remain enforced.

After building/sourcing ROS and stopping the previous owner, the normal startup
can bootstrap a passive owner, export, close it, and start the planner in one
command. Use the station Python environment and the same configuration, runtime
paths and lock as the resident deployment:

```bash
/path/to/station-python tools/freeze_ros_joint_limits.py \
  --config /path/to/station.json --state-file /dev/shm/station/arm.json \
  --output /path/to/joint-limits.json --bootstrap-owner \
  --lock /path/to/station.lock --log-dir /dev/shm/station/owner-logs \
  --launch -- enable_hardware:=true rviz:=true
```

Without `--bootstrap-owner`, the command reads an already running fresh idle
owner. An idle holding arm may remain torque enabled during this read-only
export. A new source cohort must first start its own observe-only owner: old
source evidence cannot qualify new code. If the passive owner cannot acquire
the lock, prove its identity, or exit cleanly, the launcher refuses to start.

Alternatively export once and pass `joint_limit_contract:=/path/to/joint-limits.json`
to `moveit_resident.launch.py`. The portable no-contract model retains bounded
vendor limits; differing hardware limits appear as a blocking planning status.
A state outside admitted limits still requires the existing bounded recovery
workflow; a contract never marks it normal or changes measured feedback.
Regenerate/relaunch after a contract mismatch; never bypass start-state bounds.

## All five current ranges and the manual wrist sweeps

Degrees below use the existing calibration midpoint and 4095-count mapping.
These are distinct software/evidence domains, not a new mechanical qualification.

| Joint | Vendor model | Current admitted | Observed firmware |
| --- | --- | --- | --- |
| shoulder_pan | -109.999875..109.999875 | -89.978022..89.978022 | -119.780220..119.780220 |
| shoulder_lift | -100.000043..100.000043 | -95.692308..95.692308 | -95.692308..95.692308 |
| elbow_flex | -96.829867..96.829867 | -96.747253..96.747253 | -97.010989..97.010989 |
| wrist_flex | -94.999840..94.999840 | -94.989011..94.989011 | -103.340659..103.340659 |
| wrist_roll | -157.211025..162.789342 | -89.978022..89.978022 | -180..180 |

Vendor provenance: `workshop/digital_twin/assets/so101_new_calib.urdf`, SHA256
`3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`.
Calibration: `hs_rig01_follower.json`, SHA256
`bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c`.
Current owner `arm-session-6b45c02f4aa341058590c30b90744eb4` exposed admitted raw
pan1084..3131, lift1063..3240, elbow851..3052, flex915..3076, roll1024..3071.
This source change generates from a fresh matching owner rather than embedding
these particular numbers. A separately loaded stale model can still differ
from the vendor source; the generated contract installs the actual admitted q
ranges into the description consumed by every resident planning node.

The operator's earlier manual wrist travel is supported by saved evidence:

- `2026-10-19-sorting/wrist-manual-sweep/ANALYSIS.json`: 5,712 passive samples,
  all torque off, session `arm-session-db08d4e79a544627b10afbd41b3da2af`.
  `ROLL-UNWRAPPED.json` identifies **wrist_roll**, unwrapped1218..5007 counts,
  **333.017578 degrees** using encoder modulus4096, with two seam crossings
  (4080 to13 and4 to4066). The naive min/max raw span is not a safe travel range.
- `2026-10-19-sorting/wrist-flex-manual-sweep/ANALYSIS.json`: 1,361 passive
  samples, flex826..3172, **206.191406 degrees**, no wrap; lower endpoint dwell
  0.421s and upper dwell45.252s. Both records say `calibration_applied:false`.

The configured wrist roll ±90 range is therefore a restrictive existing policy,
not established mechanical truth. This fix aligns the model with that currently
admitted policy. Promoting full observed roll travel needs a wrap-aware encoder
coordinate, chosen seam and independently checked model/collision coverage.
Promoting flex endpoints also needs endpoint/geometry qualification. Neither
expansion is inferred from the present bounds export. Physical gripper radians
remain unqualified.
