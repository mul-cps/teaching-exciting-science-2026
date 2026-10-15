# Offline station scene-pose candidate fitting

`workshop.perception.scene_pose_optimizer` fits a small batch of already
normalized tag observations on the CPU. It uses SciPy `least_squares`, NumPy,
and a robust loss. It does not start cameras, detect markers, calculate arm
FK, publish ROS messages, or change the live TF tree. The output is explicitly
a **candidate** for a human to inspect.

## Frames and identities

Use the metric `charuco_board` frame as `world_frame`. Its origin is the
measured ChArUco outer-board origin on the depth plane, with the board X/Y
axes, and Z normal, as described by the ChArUco-plane calibration. The plane
anchors Z=0 and fixes world heading. `charuco_board_T_camera_link` is
`world_from_camera`; camera optical depth XYZ can be converted with
`camera_point_to_world(world_from_camera, xyz_camera_m)`. This uses the actual
per-capture plane transform, not a stale static camera transform.

`track_id` is the persistent physical association of one printed marker across
frames (or an explicit declaration such as `ev3-left-face`). Printed tags all
have code ID 0, so the code ID is not a physical identity and must never be
used as `track_id`. The optimizer deliberately rejects the literal track ID
`"0"`.

## Input JSON

The problem contains `entities` with an initial board/world pose, and an
`observations` array. Each observation relates a marker to an entity through
`entity_from_tag`, which is the known physical tag mount for a fixed object or
the cached base-from-tag FK transform for that robot sample.

```json
{
  "world_frame": "charuco_board",
  "entities": [
    {"id": "so101_base", "kind": "planar_base",
     "base_origin_height_m": 0.115,
     "initial_world_from_entity": [[1,0,0,0],[0,1,0,0],[0,0,1,0.115],[0,0,0,1]]},
    {"id": "ev3_sorter", "kind": "rigid",
     "initial_world_from_entity": [[1,0,0,0.3],[0,1,0,0.1],[0,0,1,0.2],[0,0,0,1]]}
  ],
  "observations": [
    {"track_id": "ev3-sorter-front", "entity_id": "ev3_sorter",
     "pose_kind": "point_only",
     "entity_from_tag": [[1,0,0,0.04],[0,1,0,0],[0,0,1,0.02],[0,0,0,1]],
     "world_point_m": [0.341,0.101,0.221], "sigma_translation_m": 0.006},
    {"track_id": "so101-tool-tag", "entity_id": "so101_base",
     "pose_kind": "point_only", "entity_from_tag": [[1,0,0,0.3],[0,1,0,0],[0,0,1,0.31],[0,0,0,1]],
     "world_point_m": [0.5,0.16,0.425], "sigma_translation_m": 0.008}
  ]
}
```

`world_point_m` is the tag-centre point in board coordinates. A marker pipeline
that currently has only depth XYZ can provide this without inventing an
orientation: transform the camera optical XYZ using that capture's
`world_from_camera` and set `pose_kind` to `point_only`. A full pose record
uses `pose_kind: "pose"` and a homogeneous `world_from_tag` matrix. Pose
rotation is a right-handed 3x3 matrix; translation is metres. All matrices use
column-vector transform composition (`world_from_entity @ entity_from_tag`).

Planar bases optimize only X, Y, and yaw. The caller must supply their measured
base-origin height explicitly in `base_origin_height_m`; the solver will not
infer it from a wrist tag or set it from the board plane. Other rigid objects
with point-only observations optimize translation and keep the initial
orientation fixed. Full-pose observations enable all six rigid-body degrees
of freedom.

Tag mounts are fixed by default. To fit a small shared correction for a
physical tag mount, provide `optimize_mount: true` and a finite six-vector
`mount_prior_sigma` in `[tx, ty, tz, rx, ry, rz]` units of metres and radians.
Zero locks a component. An absent or unbounded prior cannot create free mount
variables; this prevents the tool-tag roll and base yaw from being silently
co-estimated as a gauge-coupled pair. Calibrate that mount independently when
possible, especially for the gripper tag's unknown rotation/constant tool roll.

The optimizer checks the numerical Jacobian rank before returning. A failed
rank check means the capture motion or tag geometry did not constrain all
requested parameters. Add arm poses with varied joints and well-spread tag
locations, or lock uncertain parameters; do not force a candidate out of a
degenerate capture set.

## Run and benchmark

SciPy is an optional dependency in the `digital-twin` extra. With the project
environment active:

```sh
python tools/optimize_scene_poses.py observations.json --output candidate.json
python tools/optimize_scene_poses.py observations.json --initial-solution candidate.json
python tools/optimize_scene_poses.py --benchmark
```

The second invocation warm-starts each entity from the prior candidate while
reusing the same cached observations. Call at a low rate or when observations
change, review residuals/observability, and explicitly promote an accepted
candidate through the station's calibration workflow. The benchmark is a
small synthetic CPU batch with one injected marker-pose outlier; it warms the
SciPy import and reports steady solve timing and recovery error, not camera or
ROS latency.

Physical printed marker dimensions remain caller metadata. This module does
not infer them from border appearance. Use the documented nominal 19.4 mm or
7.5 mm size only for the correct physical label; the EV3 visible 36 mm marker
measurement is uncertain and should not silently become a scale calibration.
