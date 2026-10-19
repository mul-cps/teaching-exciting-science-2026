# Capture measured home through ROS

`/follower/capture_home` (`std_srvs/srv/Trigger`) saves the stationary measured
pose through the existing resident bus owner. It opens no second SDK connection
and writes no motor registers. Development qualification used fake hardware;
root/operator owns deployment and physical support.

With the arm supported, first obtain the existing explicit verified STOP as
required by the current fault state. Capture requires fresh all-six torque-off
measurements and calibrated normal raw/body bounds. A failed endpoint hold,
active operation/teleop, component scope, pending RAM restoration, observer,
recovery or source/calibration mismatch rejects capture. No limit or geometry
qualification is changed.

```
ros2/scripts/station-action.sh arm-capture-home
# Forwarding API: /station/arm/capture_home (Trigger, async station job)
# Direct resident API: /follower/capture_home (Trigger)
```

The destination is exclusively the trusted configured `arm.home_file`; callers
cannot choose a path or pose. Two exact-equal six-joint position/torque snapshots
are acquired. The new measured-home JSON and an exact-byte backup of the prior
artifact are staged and synced, then a third unchanged all-off acquisition is
required immediately before atomic replacement. Each register read must return
within100 ms; precommit operation budget is1 s. Native filesystem/SDK calls
cannot be forcibly interrupted, so this is not a hard-real-time transaction.

The artifact preserves schema1, body degrees/gripper percent, raw counts,
arm/calibration/source/session identity, original measurement timestamps,
previous SHA256 and exact-byte backup path. Geometry, grasp, calibration and
motion qualification remain false. The resident station lock is the sole-writer contract; the prior-byte comparison
is not a filesystem compare-and-swap against an unrelated external writer. Do
not edit the configured home concurrently. The file replacement is the commit point;
read/staging/validation failures before it leave the prior home bytes intact.
The containing directory is synced before replacement; post-rename directory
fsync is deliberately omitted so a postcommit sync failure cannot masquerade as
an unchanged home. Sudden-power-loss durability of the final rename is not
claimed. On unknown delivery, inspect artifact hash/command ID/status; do not
blindly retry. A malformed/late completion latches owner motion admission until
a separately explicit verified STOP. Capture itself never sends STOP.

Capture does not adopt either resident or bridge home cache. Existing explicit
`/follower/home` (or `station-action.sh arm-home`) reloads the new same-identity
home and executes its existing bounded motion, after all physical gates pass.
Capture clears the bridge's previous home-completion provenance and never marks
the station commissioned or enables automatic sorting. Do not use the old
standalone direct-bus capture command while the resident owner holds its lock.
