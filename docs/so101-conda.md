# SO-ARM101 ROS 2 Jazzy with RoboStack

This setup creates a dedicated Miniforge environment for ROS 2 Jazzy and the
SO-ARM101 MoveIt workspace. It leaves Conda `base` and the system ROS install
alone. The environment definition is [environment.yml](../ros2/conda/environment.yml);
`linux-64.explicit.txt` records the resolved Linux package artifacts from the
verified host installation.

RoboStack publishes ROS packages through the `robostack-jazzy` Conda channel
and recommends Miniforge with `conda-forge` plus the matching RoboStack
channel. See the [Jazzy package list](https://robostack.github.io/jazzy.html)
and [RoboStack Conda guidance](https://robostack.github.io/conda.html).

## Create and build

From the repository root:

```bash
./ros2/conda/so101-conda.sh create
./ros2/conda/so101-conda.sh build
```

The wrapper defaults to `$HOME/miniforge3/envs/so101-jazzy`. Set
`SO101_CONDA_ROOT`, `SO101_CONDA_EXE`, or `SO101_CONDA_PREFIX` to use another
Miniforge installation or prefix. `create` refuses to overwrite an existing
prefix. The wrapper strips inherited ROS, Python, CMake, and dynamic-loader
search paths before invoking Conda, so a previously sourced system ROS setup
does not contaminate the environment. It preserves display authentication
variables and honors the caller's `ROS_DOMAIN_ID`.

The workspace build fetches the pinned upstream source through the existing
`ros2/scripts/so101.sh fetch` command and builds it with two colcon workers.
Build, install, and logs are written under the ignored `ros2/.cache/conda/`
directory.

## Mock workflows

All launch commands below use mock ros2_control and do not access the arm or
cameras:

```bash
./ros2/conda/so101-conda.sh rviz        # MoveIt and RViz visualization
./ros2/conda/so101-conda.sh servo       # Servo with mock controllers
./ros2/conda/so101-conda.sh plan-smoke  # IK and plan-only smoke; no execution
./ros2/conda/so101-conda.sh inspect     # controllers, topics, actions
./ros2/conda/so101-conda.sh packages    # installed Servo, RealSense, and TF
```

Use `run COMMAND...` to invoke tools in the built overlay, or `shell` for an
interactive ROS shell. The RealSense package being installed only confirms
that its ROS driver is available; the `packages` action does not open a camera.
The wrapper's `real` action deliberately fails closed. Hardware startup remains
owned by the station's separate safety and device-lock workflow.

This environment includes SciPy for the TagTracker and image-processing paths.
RoboStack's Jazzy control dependencies currently require OpenCV 5.0. Its Python
binding exposes ArUco but does not expose `cv2.calibrateHandEye`; offline
camera hand-eye calibration therefore remains in the station's separate base
Python environment. Downgrading to OpenCV 4.12 is unsatisfiable alongside the
Jazzy `ros2_controllers` package, so the ROS environment does not attempt it.

For a separate smoke-test ROS graph, set a domain ID such as `73` before the
command. The wrapper passes it through without overriding it:

```bash
ROS_DOMAIN_ID=73 ./ros2/conda/so101-conda.sh plan-smoke
```

### Local station discovery

The Conda wrapper defaults `ROS_AUTOMATIC_DISCOVERY_RANGE` to `LOCALHOST`.
This prevents unrelated LAN robots publishing common names such as `base_link`
from replacing the station's `charuco_board -> base_link` TF parent in RViz.
An explicit `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` remains available when a
multi-host ROS deployment is intended; use unique frame names in that case.
Changing discovery settings requires restarting affected ROS processes,
including RViz, to discard conflicting transforms already cached in TF.
