# SO-ARM101 MoveIt grasp planning and execution

`tools/ros_plan_grasp.py` defaults to planning only. It selects a tracked
geometric grasp or an explicit tool-link contact pose and requests sequential
MoveIt plans. The default group is the five-joint `manipulator`, with
`gripper_frame_link` as the commanded link.

With one station owner already providing the camera, tracker, scene and MoveIt:

```sh
ros2/conda/so101-conda.sh run python tools/ros_plan_grasp.py \
  --track-id red-0003 --output /tmp/red-0003-plan.json
```

Planning artifacts retain native response error codes, trajectories, and stage
results. Start state must be complete and fresh; later stages use the previous
planned endpoint. Tracking and localization must remain fresh. Planning alone
does not establish calibration, physical clearance, pickup or release.

## Opt-in execution

Execution requires a tracked target, a place **tool-link** pose, and independently
qualified jaw, physical grasp-frame/TCP, camera-arm and effective-model records.
See [gripper calibration](so101-gripper-registration.md) for their schemas and
[the acceptance register](ros-sorting-acceptance.md) for remaining physical gates.
No current calibration candidate is promoted merely by setting a flag.

```sh
ros2/conda/so101-conda.sh run python tools/ros_plan_grasp.py \
  --execute --track-id red-0003 \
  --execution-config /path/to/qualified-grasp-config.json \
  --frame charuco_board --place-pose X Y Z QX QY QZ QW \
  --return-joint-pose /path/to/commissioned-clear-joints.json \
  --timeout 120 --output /path/to/new-grasp-result.json
```

The CLI locks scene ownership before opening, plans and executes approach and
contact, closes through the resident gripper action, provisionally attaches,
lifts and checks distinct fresh RGB-D observations, then places, opens,
detaches, retreats and verifies the object remains at its destination. The
physical grasp-frame offset is composed into pickup targets; explicit place
poses already refer to the tool link. The clear-joint file maps all five arm
joint names to commissioned radians. Without that file, retreat uses the
calibrated physical grasp frame.

Arm trajectories use the resident collision preflight and dynamics checks.
The gripper action shares that resident owner and moves its full calibrated
range in bounded jaw-only chunks. Inferred jaw contact remains provisional;
it is not measured force or confirmed pickup.

Immediately before every action submission, the CLI checks localization and
its execution deadline. Target movement or loss during planning prevents the
pickup motion. Failure or cancellation stops further stages and waits for
confirmed action termination. Uncertain attachment outcomes retain ownership
for explicit scene reconciliation. Software/native tests pass with test
owners; real calibration, contact, recovery and complete sorting are not yet
physically accepted.
