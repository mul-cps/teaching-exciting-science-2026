# Compact RGB-D surface map

`so101_station_map` turns `/camera/depth/color/points` into a small RViz voxel
overlay in `charuco_board`. It is a CPU-only view for comparing the raw camera
cloud with a readable surface summary. The cloud subscriber uses sensor-data
QoS; the node looks up `charuco_board <- cloud frame` at each cloud timestamp
and never publishes or modifies TF.

The map stores only observed surface cells. A colored cube means that a depth
sample landed in that 1 cm voxel. Empty space means **unknown**: this view does
not trace sensor rays, infer free space, or provide a planning collision map.
Cells expire after 0.8 seconds, which limits stale arm echoes when the arm
moves. No robot geometry is removed from the cloud, so recently observed robot
surfaces can briefly appear in the map.

The node limits range to 1.2 m, crops transformed points to the configurable
workspace (default X/Y -0.6..0.6 m and Z -0.05..0.8 m in the board frame), and
keeps at most 60,000 cells. It reads the PointCloud2 payload in 16-row chunks;
the one-row compact RealSense layout is accepted even if `row_step` retains its
old organized-cloud value. Input stamps must be positive, no more than 0.5 s
old, and no more than 50 ms in the future. Mapping and RViz publication are
capped at 2 Hz. `/so101_map/status` reports input/cropped point counts, current
voxel count, bounds state, and rejected input reasons.

Build with the repository's ROS container workflow:

```bash
ros2/scripts/so101.sh build
```

In a ROS Jazzy terminal with the workspace sourced, start the node:

```bash
ros2 run so101_station_map surface_voxel_map
```

In RViz, add the raw `/camera/depth/color/points` PointCloud2 display and the
`/so101_map/occupied_surface` Marker display. Their display checkboxes provide
the switch between dense raw RGB-D and the compact map. Set the fixed frame to
`charuco_board`; pass a different `target_frame` ROS parameter only when using
another published fixed frame. The station live launch and RViz config can
include this node/display when desired; the map package itself does not start
the camera, arm, or any other hardware.

The visualization uses OctoMap-like sparse occupied cells without requiring
PCL or a probabilistic octree. The official OctoMap ROS 2 server supports
incremental PointCloud2 mapping, but its ROS package depends on PCL and
publishes a probabilistic 3D occupancy model. For this surface-only comparison
view, a bounded expiring sparse cache is smaller and keeps its unknown-space
semantics explicit. See the [OctoMap ROS 2 server](https://github.com/OctoMap/octomap_mapping)
and its [ROS package overview](https://index.ros.org/p/octomap_server/).
