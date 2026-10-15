# SO-ARM101 MoveIt grasp pose planning

`tools/ros_plan_grasp.py` accepts a tracked detection ID or an explicit tool
contact pose, then requests sequential MoveIt paths to pregrasp, contact, lift,
and optionally place. For a tracked object, the tool uses the geometric contact
candidate in `/so101/grasp_status`; `/so101/grasp_candidates` remains the
existing 60 mm pregrasp stream. The planner uses the current resident MoveIt
scene, `manipulator` (five arm joints), and `gripper_frame_link` by default.

Example, with the station owner already running the camera, tracker, scene
updater, and resident MoveIt stack:

```bash
ros2/conda/so101-conda.sh run python tools/ros_plan_grasp.py \
  --track-id red-0003 --output /tmp/red-0003-plan.json
```

An explicit contact pose can be planned without an object tracker:

```bash
ros2/conda/so101-conda.sh run python tools/ros_plan_grasp.py \
  --frame charuco_board --pose 0.12 -0.04 0.03 0 0 0 1 \
  --place-pose 0.20 0.02 0.10 0 0 0 1 \
  --output /tmp/pose-plan.json
```

The output records every segment result and trajectory. A failed segment stops
the sequence. The start state comes from a complete fresh
`/follower/joint_states` sample; later segments start at the previous planned
endpoint. Tracked status must remain recent while the service calls are in
flight. MoveIt's pose goal uses a small positional region and tolerances on
orientation because the arm has five joints and cannot generally satisfy an
arbitrary six-axis pose exactly. A successful service response proves a path
was planned in the current MoveIt scene; it does not establish that the
camera target is calibrated, that a grasp will work, or that the retimed
trajectory passes the resident controller's dense collision preflight.

The contact path can correctly fail while a target brick remains a world
collision object. Planning-scene attach/detach diff builders are available in
`so101_station_scene.grasp_planner`, but the CLI does not apply them: the
tracker and scene updater are live owners, and the station has no validated
gripper contact links for an attachment transition. The control bridge accepts
the five arm joints only. Its gripper joint is a visual proxy and is never
commanded, so jaw-open, jaw-close, force/contact acknowledgement, attached-object
planning, and physical pick/place are not implemented. No plan from this tool
is executed.
