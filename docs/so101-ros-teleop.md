# SO-ARM101 ROS leader teleoperation

The ROS leader reader publishes calibrated leader state from the exact device
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00` using
`hs_rig01_leader.json` in `/home/bjoern/git/soarm101-highschool-demos/.calibration`.
It runs LeRobot in the configured station Python environment because the ROS
Python environment does not provide that hardware SDK. The reader connects with
the calibration handshake disabled, verifies the saved calibration and all six
leader torque registers are already off, and fails closed if either check fails.
It only reads the leader. Shutdown disconnects without changing torque.

Build and source the ROS workspace, then start the reader:

```bash
ros2 launch so101_station_control leader_teleop.launch.py
```

Override `station_python`, `station_root`, `leader_port`, or `calibration_dir`
when the installation differs. The launch starts only the read-only leader
publisher. It does not enable follower teleoperation. The follower must already
be managed by the resident `trajectory_server` bridge with its hardware owner
ready.

`/leader/teleop_pose` is `std_msgs/msg/String` containing compact JSON schema 1:

```json
{"schema":1,"stamp_s":1234.5,"sequence":12,"units":{"body":"degrees","gripper":"percent"},"pose":{"shoulder_pan":0,"shoulder_lift":0,"elbow_flex":0,"wrist_flex":0,"wrist_roll":0,"gripper":50}}
```

`stamp_s` is the host monotonic acquisition time and `sequence` increases for
each leader sample. This timestamp is a same-host contract; remote ROS hosts
with independent monotonic clocks are unsupported. Body values use calibrated
physical degrees; gripper uses calibrated opening percent. The follower bridge
rejects malformed or older than 150 ms samples, captures fresh matched
references when teleoperation is enabled, and sends mapped follower poses
through its existing ArmSession owner. The mapping is
`follower_start + leader - leader_start`, using the per-joint calibrated
convention above. ArmSession applies the calibrated follower bounds and rate
limits. The bridge provides `std_srvs/SetBool` at `/follower/teleop_enable`;
enable fails while a trajectory action is active and reports the owner start
acknowledgement before returning success. Disable stops the session and holds
the measured pose. Do not publish a synthetic enable request at startup.

`/leader/joint_states` publishes the five body joints in radians. Its `gripper`
joint is a visual-only jaw estimate mapped from normalized percent into the
canonical `[-10, 100]` degree display range; it is not a measured angle and is
never commanded. The JSON pose topic is the teleoperation input contract.
Online teleoperation is joint-space control and does not run MoveIt collision
checks; keep the physical workspace clear and supervise the arm.

The leader ROS node does not own or open the follower serial port. Follower
commands continue through the one resident `ArmSession`, which enforces a
300 ms command watchdog. A lost leader stream therefore expires at the bridge
and resident owner rather than leaving a stale command queued for later use.
