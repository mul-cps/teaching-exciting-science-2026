# ROS sorting station acceptance — 2026-10-16

The station is **not physically accepted**. The implementation branch is
`feat/sorting-station-20261016`; the original dirty checkout is preserved.
The [plan](superpowers/plans/2026-10-16-ros-sorting-station.md) defines the
release requirements; the [progress record](superpowers/plans/2026-10-16-ros-sorting-progress.md)
records commands, limitations and decisions.

| Requirement | Current evidence | Status |
| --- | --- | --- |
| Software suite | System Python: 1140 passed, 53 skipped (23.80s, at domain/GPU integration); trusted CGAL/mesh-audit subset44passed | Software passed; skipped checks remain separate |
| Native ROS checks | Root camera/GPU/domain subset42passed in domain75; typed object-depth launch subset23passed; earlier tracking/validation/worker subset94passed and station-venv twin subset19passed | Passed with test owners; no physical execution |
| Clean package installation | All 17 packages built in a fresh integration prefix; separate startup worker verified a non-symlink install and installed plane-module import | Passed |
| MoveIt planning | Mock start state valid, no collision; seven-point trajectory saved | Planning only; physical scene not qualified |
| Single station startup | One camera launch and one resident follower owner; calibrated jaw path forwarded through both launch layers | Implemented; live cold starts pending |
| Calibration | Available reports are explicitly unqualified; no held-out evidence supports a further 180-degree mount correction | Not accepted |
| Plane residual ≤3 mm / pickup position error ≤5 mm | No current held-out physical measurements prove these bounds | Pending |
| Passive twin | Live viewer renders correct root-frame TF/marker semantics; raw RSP/MoveIt hashes match; bridge interruption visibly marks STALE and restart restores FRESH | Stationary live display and stale/reconnect checked; physical registration remains separate |
| Twin error ≤1 mm / ≤0.5 degrees | One fresh synchronized stationary sample: tool disagreement1.24e-16m/0degrees; all-link/root-geometry nonidentity-transform regressions pass | Stationary mathematical parity passed; moving physical qualification pending |
| Teleop ≥100 Hz for 30 minutes | Uncapped profile regression retained; no qualifying endurance run this session | Pending |
| Twenty consecutive physical sorting cycles, ≥5 per color | No physical sorting cycle executed | Pending |
| Failure recovery and reconnect | Software tests cover stop/ownership paths; transport failure observed while torque was last verified off | Physical behavior unqualified |
| Camera-guided automatic arm recovery | Bounded planner, conservative depth adapter and owner admission/watchdog implemented and tested; production provider rejects without qualified geometry/registration | Live physical recovery pending |
| ROS isolation and rendering | Full station and twin restarted on domain76; wrapper defaults76/rejects0; localhost discovery; actual RViz amdgpu counter increments | Live checked; rebuilt container image remains separate |
| Native EV3 commissioning | Exact machine/port identity checked; one diagnostic sorter switch home completed1.859s/refhome0; baseline overspeed guard and MicroPython fault observed and recorded; feeder reference remainsNone | Sorter home once; feeder/ejection/othermoves/sorting pending |
| Three successful cold starts | Three camera-only starts passed after reconnection; complete station cold-start gate not exercised | Pending |
| Release artifacts and publication | Local camera/USB logs retained; implementation branch not a release | Pending |

## Current hardware boundary

The D435i is SDK serial `243322071783`, firmware `5.17.0.10`. After the
recorded USB outage, three native ROS camera starts each delivered approximately
200 paired frames over15seconds with zero saved-pair timestamp skew. The live
camera, RViz and sole follower owner are running. These camera-only cycles do
not prove full station cold starts or camera-to-arm registration.

Fresh read-only arm telemetry still reports elbow raw2993/91.56degrees,
outside its configured90degree bound, with all six torque states zero. The
recovery owner derives a bounded inward target but production lacks a qualified
camera scene provider; no recovery or grasp motion has been sent. Physical
camera/base/TCP/jaw qualification remains a prerequisite for execution.

Full-view color tracking now separates observation, support and pickup admission.
Contact/pregrasp previews outside pickup bounds do not authorize execution.
Depth support, original per-child shape evidence and three-frame acquisition
confirmation reject transient floating fragments without a fixed workspace
exclusion. Earlier30-second capture `tracking-confirmed-live/` preserved all eight
real bricks with zero extras in150/150tracking reports. Receipt age was below
0.327seconds. The later domain76 sample has9colored surfaces, including near hardware; all ghosts are not eliminated. This verifies the earlier stationary reproduced scene; live dynamic,
occlusion and semantic generalization remain unqualified.

Evidence is under
`~/.local/share/exciting-science/station/evidence/2026-10-16-sorting/`, including
`README.md`, camera probe logs, kernel events and
`rgbd_pair_ros_driver/{color.png,depth_raw.npy,depth_aligned.npy,info.json}`.
Native startup/build/plan artifacts are under
`/tmp/sorting-startup-validation-20261016/`; retain them with the release
evidence before clearing temporary storage.

## Sorting entry point

`tools/ros_sort_station.py` consumes the already-running ROS station and the
persistent EV3 API tunnel. It opens neither a camera SDK nor an arm serial port.
It requires explicit `--execute`, a newly created evidence directory, and a
`so101-ros-sorting-v1` JSON configuration containing:

- `enabled`, `commissioned`, and `automatic_start`, all explicitly true only
  after physical commissioning;
- `station_config` and `grasp_execution_config` paths, resolved relative to the
  sorting configuration;
- `frame_id`, six ordered `pickup_bounds_m` values (`xmin,xmax,ymin,ymax,zmin,zmax`),
  and a measured `clear_joint_pose_rad` map for all five arm joints;
- `destinations`, mapping commissioned colors to XYZ plus XYZW place poses.

The grasp execution configuration and calibrated jaw/TCP/camera/model gates
are described in [gripper registration](so101-gripper-registration.md).
The EV3 must already have valid physical references in the current daemon
session. The read-only `/follower/sorting_ready` query requires measured-home
completion in the current resident-owner session.

Once these prerequisites are physically verified:

```sh
ros2/conda/so101-conda.sh run python tools/ros_sort_station.py \
  --config /path/to/commissioned-sorting.json \
  --ev3-port 18766 --cycles 1 --execute \
  --output /path/to/new-cycle-evidence
```

Every failure prevents another dispense. A cycle completes only after distinct
fresh observations verify pickup/lift and the brick remaining at its destination
after the tool retreats, followed by measured clearance. Command acceptance,
successful planning, and jaw movement do not establish a completed physical sort.
