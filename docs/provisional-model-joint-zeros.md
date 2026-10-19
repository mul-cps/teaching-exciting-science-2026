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
