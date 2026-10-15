# SO-ARM101 tracked planning scene and grasp checks

`so101_station_scene` converts the perception worker's stable color-object
tracks into incremental MoveIt world-object updates. It subscribes to
`/so101/objects` (`vision_msgs/Detection3DArray`) and transforms each box from
its stamped source frame into `base_link` using TF at the detection timestamp.
Zero, stale, future, unframed, or untransformable observations are rejected.
Boxes require stable detection IDs, finite dimensions between 2 mm and 300 mm,
and receive 6 mm padding on each side because the current extents are
provisional. Tracked IDs become `colorbrick_<escaped-id>` and expire after
750 ms without an update.

The node calls `/follower/apply_planning_scene` with `PlanningScene.is_diff`
set. It only adds, updates, or removes the IDs it owns; it never replaces the
world, clears unrelated objects, or publishes attached-object changes. Status
appears on `/so101_scene/status`. It labels boxes provisional and exposes
`frames_qualified` (false by default); this flag does not qualify the current
camera/board calibration.

The board box uses the current ChArUco world convention: x spans 0–175 mm,
y spans −125–0 mm, the top surface is z=0, and +z points upward. The board's
4 mm collision thickness extends below its top surface. A finite tabletop is
disabled by default because its measured footprint is not yet part of the
calibration record. To enable one, set `table_collision_enabled`, then replace
`table_size_xy_m` and `table_center_xy_m` with measured board-frame values.
The configured tabletop volume is split around the live `base_link` footprint
(`base_keepout_size_xy_m`) and sits below the board at `table_surface_z_m`.
Keep the table disabled until both its extent and keepout are measured.

## Launch and MoveIt wiring

Build the overlay, then start `station_scene.launch.py` in the same ROS domain
as the existing perception and resident MoveIt launches:

```bash
./ros2/conda/so101-conda.sh build
./ros2/conda/so101-conda.sh run ros2 launch so101_station_scene station_scene.launch.py
```

The scene package does not open the camera or start the robot driver. The
existing perception launch supplies `/so101/objects`; the resident MoveIt
launch supplies `base_link` TF, `/follower/apply_planning_scene`, and the IK
and state-validity services. `station_scene.launch.py validate_grasps:=false`
starts only the collision-scene updater.

`ros2/config/sensors_3d.yaml` is an optional MoveIt occupancy-map config for
the RealSense point cloud. The resident `move_group` node must receive this
YAML alongside its existing parameter dictionaries, and must use
`octomap_frame: base_link`. The official `PointCloudOctomapUpdater` filters
robot-body points against the robot model, downsamples by four, and is limited
to 1.2 m and 2 Hz. That updater has no `min_range` setting; do not add one and
assume it is applied. Its actual accepted fields are visible in the upstream
[MoveIt 2 updater source](https://github.com/moveit/moveit2/blob/main/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)
and [sensor configuration example](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/perception_pipeline/config/sensors_3d.yaml).
The OctoMap is an additional collision source, not a replacement for the
tracked-box updates.

## Grasp-candidate validation

`grasp_validator` consumes `/so101/grasp_candidates` (`geometry_msgs/PoseArray`)
in the stamped `charuco_board` frame. The perception worker's poses are
pregrasp poses offset 60 mm opposite the downward approach. For each candidate
the validator transforms its position to `base_link` at the message timestamp
and tests a base-vertical-down orientation: tool +Z points along base −Z, with
the requested jaw-closing axis projected into the base horizontal plane. This
keeps each candidate's transformed XYZ while allowing the tool approach and
closing directions to differ from the source pose; the JSON reports both
angular changes. The base vertical differs from the board normal by about five
degrees in the current unqualified setup. This is a tested IK variant, not a
claim that the source grasp orientation was preserved.

The validator requires current TF and a fresh `/follower/joint_states` sample.
It checks up to eight candidates from each batch in sequence using
`/follower/compute_ik` with collision checking and
`/follower/check_state_validity` on each returned joint state. It rate-limits
batch starts to at most one per second, while allowing candidates in that batch
to advance without an extra one-second delay. Results are published as JSON on
`/so101/grasp_validation`, one record per candidate.

`pregrasp_ik_collision_valid` means only that one instantaneous pregrasp pose
has an IK solution and passes MoveIt's state-validity check in the current
scene. It does not validate the approach path, contact pose, jaw opening,
gripper calibration, object attachment, or a physical pickup. Every record
states `geometric_only: true`, `path_validated: false`, and
`execution_enabled: false`; this package has no trajectory execution action
client.

MoveIt Task Constructor is a possible next step for a plan-only sequence. The
upstream Jazzy release includes `moveit_task_constructor_core` and
`moveit_task_constructor_msgs` at 0.1.8, and the RoboStack Jazzy channel has
matching Conda packages, but neither package is installed in the current
`so101-jazzy` environment. The official [MTC pick-and-place tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
shows the staged planner API. A future planner could consume a perception
candidate, run `ComputeIK`, plan a collision-checked `Connect` to pregrasp and
a `MoveRelative` approach, then model attachment and retreat in a
`PlanningScene` diff. It should stop before close/attach/execute until the
gripper jaw mapping and contact geometry are qualified. No MTC node or grasp
generator is included here; the current geometric candidate stream is the
input, and this package only validates pregrasp states.
