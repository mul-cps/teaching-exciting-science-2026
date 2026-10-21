# CPS Candy Factory workshop
22 October 2026 · Four hours · First semester · No programming or robotics prerequisites

Start with [the student notebook](notebooks/Robot_Candy_Factory_Student.ipynb), [the instructor edition](notebooks/Robot_Candy_Factory_Instructor.ipynb), or [the robot and twin observation lab](notebooks/Robot_Candy_Factory_Live_SO101.ipynb). Download and extract the complete [student workshop ZIP](docs/student-workshop.zip), which includes local images, helper code and printable handouts. The prepared local Jupyter environment is the main classroom route; use [the launch instructions](notebooks/README.md). No installation is part of class.

The student factory is a simulation. The robot lab defaults to explicitly invented DEMO data and can read existing ROS topics when the instructor launches LIVE mode on the prepared station host. Learner notebooks never command physical actuators. The instructor uses the existing RViz operations panel to enable, select teleoperation mode and stop. Software checks and pictures do not qualify a physical sorting cycle.

![The sorting station](notebooks/assets/photo-examples.jpg)

Students predict, change one value, run and check. Red goes left, blue right, everything else to reject. The exact 240-minute route is introduction 25 min, Blockly 10, Python 30, perception 20, break 10, three stations 45, factory 50, challenges 35 and demos 15. The stations are robot and twin, camera and conveyor, and virtual robot.

- [Four-hour lesson plan](docs/lesson-plan.md)
- [Instructor preparation and answers](docs/instructor.md)
- [Student worksheet](docs/student-worksheet.pdf) and [complete handbook](docs/student-handbook.pdf)
- [External activities and offline alternatives](docs/workshop-resources.md)
- [Class slides](docs/workshop-slides.pptx) and [PDF](docs/workshop-slides.pdf)
- [Blockly factory challenge for students](docs/blockly-student.md) and [instructor setup, recording and ROS interfaces](docs/blockly-workshop.md)
- [Manual full station startup: RViz, MoveIt and LEGO EV3](docs/station-startup.md)
- [Prepared fwmini4 launchers and checks](docs/fwmini4-workshop.md)
- [Automated developer code reference](docs/code-reference/index.md)
- [Live ROS teleoperation contract](docs/so101-ros-teleop.md) and [resident station API](docs/ros-station-api.md)
- [Current ROS sorting acceptance and remaining gates](docs/ros-sorting-acceptance.md)
- [Blockly station commissioning evidence (2026-10-21; physical subset)](docs/blockly-station-commissioning-2026-10-21.json), [EV3 panel commissioning](docs/ev3-panel-commissioning-2026-10-21.json) and [gripper measurement](docs/gripper-measurement-2026-10-21.json)
- [Historical early-project acceptance baseline](docs/acceptance.md)

For the standard-library fallback run `python3 exercises/check_setup.py`, then scripts `01_python.py`, `03_vision.py`, `02_robot.py`, `04_sort.py`. Run `python3 exercises/05_visual_sort.py` and open `sorting-result.html` for the visual fallback. The supplied scripts need Python 3.11 or newer and no ROS, GPU or internet. Hardware instructions remain instructor material in the station documentation.

Rebuild notebooks and shared handbook with `python tools/build_notebooks.py`. Rebuild handouts with `SOFFICE=/path/to/prepared/soffice python tools/build_materials.py --handouts-only`, and bundle with `python exercises/make_student_bundle.py --output docs/student-workshop.zip`. Prepared dependencies are listed in notebooks/requirements.txt. See the instructor guide for isolated environment setup and verification.
