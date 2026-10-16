# ROS sorting station acceptance — 2026-10-16

The station is **not physically accepted**. The implementation branch is
`feat/sorting-station-20261016`; the original dirty checkout is preserved.
The [plan](superpowers/plans/2026-10-16-ros-sorting-station.md) defines the
release requirements; the [progress record](superpowers/plans/2026-10-16-ros-sorting-progress.md)
records commands, limitations and decisions.

| Requirement | Current evidence | Status |
| --- | --- | --- |
| Software suite | System Python: 865 passed, 45 skipped | Software passed; skipped checks remain separate |
| Native ROS checks | 84 passed across control, scene, objects, map, gripper, grasp and ownership tests | Passed with test owners; no physical execution |
| Clean package installation | All 17 packages built in a fresh integration prefix; separate startup worker verified a non-symlink install and installed plane-module import | Passed |
| MoveIt planning | Mock start state valid, no collision; seven-point trajectory saved | Planning only; physical scene not qualified |
| Single station startup | One camera launch and one resident follower owner; calibrated jaw path forwarded through both launch layers | Implemented; live cold starts pending |
| Calibration | Available reports are explicitly unqualified; no held-out evidence supports a further 180-degree mount correction | Not accepted |
| Plane residual ≤3 mm / pickup position error ≤5 mm | No current held-out physical measurements prove these bounds | Pending |
| Passive twin | Effective ROS description and timestamped state are consumed passively; native tests cover historical cloud TF and static TF timing | Software checked; live hash/pose gates pending |
| Twin error ≤1 mm / ≤0.5 degrees | No synchronized live qualification run completed | Pending |
| Teleop ≥100 Hz for 30 minutes | Uncapped profile regression retained; no qualifying endurance run this session | Pending |
| Twenty consecutive physical sorting cycles, ≥5 per color | No physical sorting cycle executed | Pending |
| Failure recovery and reconnect | Software tests cover stop/ownership paths; transport failure observed while torque was last verified off | Physical behavior unqualified |
| Camera-guided automatic arm recovery | Bounded inward planner checks fresh, identity-matched RGB-D/encoder state and complete path clearance | Planner only; live scene adapter, owner admission/watchdog and physical tests pending |
| Three successful cold starts | Camera repeat starts failed; complete station starts not attempted | Pending |
| Release artifacts and publication | Local camera/USB logs retained; implementation branch not a release | Pending |

## Current hardware boundary

The D435i is SDK serial `243322071783`, firmware `5.17.0.10`. One native ROS
capture produced paired RGB8 and aligned Z16 frames at zero timestamp skew.
Repeated runs failed with UVC `-71` errors and USB re-enumeration. A successful
frame does not establish a reliable camera stream or camera-to-arm registration.

The arm connected for read-only telemetry. The elbow measured 91.56 degrees,
outside its configured 90-degree bound; all six torque states were zero. No
recovery or grasp command was sent. The USB hub carrying both arm adapters and
the EV3 then disappeared, and the resident process exited with a serial write
timeout. Physical access must be restored and verified before further motion.

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
