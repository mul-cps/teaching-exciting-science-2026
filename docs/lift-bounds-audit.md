# Rig01 shoulder lift bound correction (2026-10-19)

The previous generic ±90° lift planning bound rejected raw1095, which maps to
−92.87912087912088° under the existing midpoint encoder convention. This is an
incorrect model envelope for the pinned SO-ARM101 mechanism.

Only the lift planning envelope changes: `ros_trajectory.POSITION_LIMITS_RAD` and
`ros2/config/rig01_moveit_limits.yaml` now use official URDF ±1.74533 radians
(approximately ±100°). The identity-only `rig01_joint_map.yaml` has no position
limits and is unchanged. Pan, elbow, wrist flex and wrist roll retain their
previous limits. Speed, acceleration, jerk and analytic spline checks are unchanged.

## Evidence and narrower execution admission

- Official model: TheRobotStudio/SO-ARM100 commit
  `a758567c3978dfeefe282ede0500085a48fe8f78`.
- Repo model `workshop/digital_twin/assets/so101_new_calib.urdf` SHA256:
  `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`.
- Captured expanded model in `commissioning-live/registration-baseline-bound/robot.urdf`
  SHA256 `d8b4c027e7ee6e2ac52721345f4b0b65cb8ffc146ce57f02ed80932590fac6a8`.
  This identifies the historical audit cohort, before the separate gripper mount
  rotation correction; shoulder lift limit values agree with the official model.
- Rig01 calibration `hs_rig01_follower.json` SHA256:
  `bc688147848f386899f8f34eb09cdb285fcdb0fc8de576b78a50a8efaf7f424c`.
- Read-only firmware snapshot and detailed five-body-joint audit are retained in
  station evidence `commissioning-live/body-limit-intersection/`.

The calibration and resident lift firmware both admit raw1063..3240, midpoint
2151.5. The existing mapping is `q_rad=(raw−2151.5)*2*pi/4095` with drive_mode0;
firmware homing offsets are already represented in encoder readings.
Their intersection with the official model is ±1.670145838062266rad,
±95.6923076923077°, inclusive raw1063..3240. Raw1062 and3241 remain outside this
execution admission. Raw1095 is32counts above its lower edge. Old configured
±90° narrowed it incorrectly to raw1128..3175.

The owner must continue passing its calibrated/configured/firmware intersection
into trajectory preparation. The existing `_position_limits` narrows the model
by this supplied admission; both trajectory knots and continuous spline extrema
are checked against that intersection. The ±100° model envelope grants no
additional execution authority. This change does not alter local station
configuration, firmware, encoder calibration, owner admission logic, or hardware.

## Verification

Hardware-free focused tests cover the current raw1095 reading, old policy rejection,
both exact raw edges and one-count violations, production pure calibrated bound
calculation, inherited non-lift limits, and an interior spline overshoot with all
knots inside the admitted envelope. Independent review and measured preflight
cost are retained in station evidence `commissioning-live/lift-bounds/`.
