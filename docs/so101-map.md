# Compact RGB-D surface map

`so101_station_map` turns `/camera/depth/color/points` into a small RViz voxel
overlay in `charuco_board`. It is a CPU-only view for comparing the raw camera
cloud with a readable surface summary. The cloud subscriber uses sensor-data
QoS; the node looks up `charuco_board <- cloud frame` at each cloud timestamp
and never publishes or modifies TF.

The map stores only observed surface cells. A colored cube means that a depth
sample landed in that 1 cm voxel. Empty space means **unknown**: this view does
not trace sensor rays, infer free space, or provide a planning collision map.
Cells expire after 2 seconds. To ride through brief board-tracking gaps, cached
cells remain visible at full opacity for 0.8 seconds, then fade until expiry.
The freshness gate still blocks integration during stale tracking. No robot
geometry is removed from the cloud, so recently observed robot surfaces can
briefly appear in the map; the longer cache can leave faint moving-arm echoes
for up to 2 seconds.

The node limits range to 1.2 m, crops transformed points to the configurable
workspace (default X/Y -0.6..0.6 m and Z -0.05..0.8 m in the board frame), and
keeps at most 60,000 cells. It reads the PointCloud2 payload in blocks of no
more than 8,192 points and samples at most 120,000 points per cloud by default.
The regular point stride bounds transforms and voxel updates while preserving
95.9% of occupied 1 cm cells in an offline synthetic XYZ cloud derived from the
supplied 640x480 registration depth frame (compared with using every valid
depth sample).
Configure the cap with
`max_points_per_cloud`. Duplicate voxel indices are reduced per block before
the sparse cache update. The one-row compact RealSense layout is accepted even
if `row_step` retains its old organized-cloud value. Input stamps must be
positive, no more than 0.5 s old, and not in the future. Mapping and RViz
publication are capped at 2 Hz. TF lookup waits at most 0.15 s for the exact
cloud timestamp while a second executor thread services TF updates. By
default, integration also requires a fresh `tracking` observation from
`/so101_perception/charuco_world/status`, no older than 0.5 s and within 0.5 s
of the cloud stamp. A retained last-known camera transform alone does not
qualify for map integration; the TTL clears geometry while board tracking is
stale. Set `require_fresh_tracking:=false` only when the selected fixed frame
is independently known to remain valid.
`/so101_map/status` reports input/sampled/cropped point counts, current voxel
count, bounds state, and rejected input reasons.

An offline profile derived synthetic XYZ from the supplied RGB-D registration
bundle's 640x480 depth frame. Decoding, stride sampling, range/workspace crop,
transform, and sparse voxel insertion processed 307,200 input points in
62 ms, sampled 102,400 points, and retained 3,475 voxels; peak process RSS was
about 43 MiB. The timing excludes ROS dispatch, TF waiting, and Marker message
serialization.

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
another published fixed frame. The station live launch starts this node by default (`use_map:=false` disables
it). In the supplied RViz config, disable **Native RGB-D surface** and enable
**Sparse 1 cm surface map (toggle with native RGB-D)** to switch views; the map package itself does not start
the camera, arm, or any other hardware.

The visualization uses OctoMap-like sparse occupied cells without requiring
PCL or a probabilistic octree. The official OctoMap ROS 2 server supports
incremental PointCloud2 mapping, but its ROS package depends on PCL and
publishes a probabilistic 3D occupancy model. For this surface-only comparison
view, a bounded expiring sparse cache is smaller and keeps its unknown-space
semantics explicit. See the [OctoMap ROS 2 server](https://github.com/OctoMap/octomap_mapping)
and its [ROS package overview](https://index.ros.org/p/octomap_server/).
