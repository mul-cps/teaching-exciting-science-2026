# Provisional model-only joint zero alignment

`model_offsets.yaml` records the frozen fixed-base CAD/depth candidate as
**PROVISIONAL, qualified=false** for `hs_rig01_follower`. Exact values are lift
+11.616411167163976°, elbow−10.348454041608484°, wrist flex−9.05585412227618°.
The file pins the original expanded model, calibration, training result and
independent-pose result. It retains the existing −90° gripper mounting rotation.

At description load, `apply_joint_zero_offsets` changes only selected joint
origin rotations: `R_origin_new = R_origin_old @ R_axis(offset)`. Thus
`FK(new_URDF,q) == FK(old_URDF,q+offset)` for every downstream link. Original
translations, axes, meshes, inertial/collision geometry, encoder readings,
calibration, firmware endpoints, command values and joint limits stay unchanged.
Only lift/elbow/wrist-flex finite numeric offsets within ±12° are admitted;
booleans, unsupported joints and malformed origins/axes are rejected.

The resident MoveIt launch builds one corrected description used by MoveGroup,
robot_state_publisher and its RViz leaf. Standalone physical guarded RViz uses
the same shared pipeline/configuration. The browser and virtual gripper camera
consume `/follower/robot_description` from RSP and do not apply another correction.
Plan-only and servo-mock launches remain explicitly nominal nonphysical profiles;
do not use them as an effective physical-model source.

## Evidence and limitations

The first pose fit used three selected offsets with the base fixed. Its spatial
point split participated in seed selection, so it is not independent pose
validation. The later captured pose changed lift raw by−69 (approximately−6.07°)
and wrist flex/roll by−1 count each; the other fitted parameters remained frozen.
No refit occurred for that pose. Its residual median improved8.80→4.25mm,
p9530.24→12.10mm and fraction within10mm54.5→89.9%; base residual stayed unchanged.
The moving-jaw residual did not improve and the nominal pinch remained roughly
14px from the visible distal silhouette. Neither image identifies a calibrated
physical contact midpoint.

The optimizer nearest-neighbor Jacobian condition32781 reflects discontinuous
correspondence changes. With correspondences frozen, the local rank is3 and
condition8.15; these are local numerical diagnostics, not calibration confidence.
Only one joint changed substantially in the independent pose. Further independently
excited poses, tool/contact measurements and physical registration validation
remain required. This profile is provisional visual/planning geometry, not
qualified physical TCP or a change to motor calibration. Deployment and any
physical acceptance remain separate supervised steps.

All corrections are once-at-load operations; no control tick, model polling,
file I/O or robot service calls are added by the model transformation helper.

## Later home-series diagnostic (2026-10-19)

A new, exact-timestamp RGB-D home series used expanded model SHA
`24085b067432f5ee575780c845b0f35d37fe4228927649b2112bceb5f8088fc2`.
No optimizer or new calibration correction ran. The first shoulder move completed;
the elbow move stopped short and entered a measured-pose failed-endpoint hold.
Its actual pose was recorded rather than treating the requested target as reached.

Across H00/H01/H02, the current model's median residual was 6.35/6.45/6.39 mm,
with p95 18.76/18.11/19.01 mm and 71.3/71.6/70.9% within 10 mm. For H00,
the original model's median/p95 were 7.17/14.40 mm. The earlier improvement
therefore does not establish robust alignment across poses. Nearest-surface
moving-jaw residuals were 14.09/14.40/13.50 mm for the current model, compared
with 9.73/10.31/11.45 mm for the original model. These nearest-neighbor link
labels are not physical jaw segmentation and cannot alone identify a jaw angle,
mount correction, or contact point.

Measured joint spans were shoulder lift 4.923 degrees, elbow 2.462 degrees,
and wrist flex zero, giving centered rank 2 at a one-degree threshold. There
was no independent held-out pose in this series. It is ineligible for the planned
three-offset fit. Repeated AprilTag ID 0 observations remain unassociated with
physical links; they do not provide a measured tag-to-tool transform.

Local artifacts: `~/.local/share/exciting-science/station/evidence/`
`2026-10-19-sorting/home-series-model-calibration/`, including
`CALIBRATION-STATUS.json`, per-pose residuals, raw-tag overlays, and checksums.
The model remains provisional and physical pick/place commissioning is incomplete.
