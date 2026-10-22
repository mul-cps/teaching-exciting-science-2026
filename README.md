# Teaching exciting science: robot candy factory

A classroom workshop about programming, perception and robotics. Students build a sorting factory in Python or Blockly; instructors operate the SO-ARM101, leader arm, RealSense D435i and LEGO EV3 station.

## Student

On the prepared classroom PC, from the repository root:

```sh
./station notebook
./station sim
```

The first command opens JupyterLab; open `notebooks/Robot_Candy_Factory_Student.ipynb` and save a team copy. The second opens the local Blockly simulation at <http://127.0.0.1:8877>. These are separate activities; neither starts hardware.

For a downloaded learner bundle, extract the whole ZIP and follow [notebook instructions](notebooks/README.md). The [student Blockly guide](docs/blockly-student.md), [lesson plan](docs/lesson-plan.md) and [worksheet](docs/student-worksheet.pdf) describe the exercises.

Without Jupyter, use the standard-library exercises:

```sh
python3 exercises/check_setup.py
python3 exercises/01_python.py
```

See [all exercise entrypoints](docs/student-exercises.md) for the complete sequence.

## Operator

Use the prepared graphical login and installed deployment manifest. Connect the identified devices and prepare the native EV3 service before starting:

```sh
./station doctor
./station start
./station status
```

`start` opens the complete prepared native ROS station and RViz. Startup verifies the selected deployment, device identities and fresh passive joint limits. It does not home, enable teleoperation or run sorting. The installed manifest binds services, RViz and physical Blockly to the same checkout and ROS domain. Live operation uses domain **76**; domain **177** is reserved for isolated software fixtures.

To start the full station, RViz and physical Blockly together, use the explicit instructor route:

```sh
./station start --physical --instructor
```

Use the RViz panel for arm preparation and explicit relative/absolute teleoperation. Blockly supports continuous calibrated jaw gaps, teaching fixed-jaw strokes and replay. Missing calibration or commissioning data blocks the corresponding operation.

Stop through the managed lifecycle:

```sh
./station stop
./station purge              # preview generated files; no changes
./station purge --yes        # verified STOP, then remove generated files
```

Read [startup, shutdown and troubleshooting](docs/station-startup.md) before operating hardware. [Prepared PC setup](docs/fwmini4-workshop.md) documents installation. Closing a browser alone does not stop the station.

## Developer

The [development guide](docs/development.md) covers layout, builds and focused checks. The [ROS API](docs/ros-station-api.md) lists the general services, actions and topics used by RViz, Blockly and other clients. [ROS workspace instructions](ros2/README.md) describe the native overlay and mock route.

The station has one follower serial owner, one camera owner and one EV3 session. Clients reuse their APIs. Credentials, measured calibration, homes, configuration and runtime evidence live outside Git. The prepared `station-fwmini4 ros-vulkan` wrapper selects the native ROS environment and Intel Vulkan path.

Physical pickup, release and automatic sorting require their own commissioning and independent evidence. A started GUI, simulation or passing software checks does not establish physical acceptance; see [sorting acceptance](docs/ros-sorting-acceptance.md).

[Documentation index](docs/README.md) · [teaching assets](docs/assets/README.md)
