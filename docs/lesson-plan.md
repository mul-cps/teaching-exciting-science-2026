# Sorting factory lesson plan
22 October 2026 · Four hours · First semester · No coding or robotics prerequisites

Work in pairs. One partner predicts, one types; swap often. Every task follows predict, change one thing, run, check. Use the prepared local Jupyter notebook; no software installation during class. The complete lesson also works with offline controlled examples. Red goes left, blue goes right, everything else goes to reject.

| Elapsed minutes | Duration | Activity | Evidence of learning |
| --- | --- | --- | --- |
| 00–25 | 25 min | Introduction and notebook demonstration | Identify sensor, decision, actuator and feedback |
| 25–35 | 10 min | Blockly Maze levels 1–3 | Predict a sequence and fix one wrong step; levels 4–5 optional |
| 35–65 | 30 min | A Python names, repeats and decisions | Change one value; distinguish = from == |
| 65–85 | 20 min | B Teachable Machine perception | Compare red, blue and background; test changed lighting or background |
| 85–95 | 10 min | Break | Save and assign rotation groups |
| 95–140 | 45 min | C Three stations of 15 min | Robot and twin; camera and conveyor; virtual robot |
| 140–190 | 50 min | D Complete sorting factory | Trace stop, observe, decide, sort and compare routes and totals |
| 190–225 | 35 min | E Challenges: Blockly automation or notebook | Build and test a rule or sequence; save and explain the evidence |
| 225–240 | 15 min | Demos and reflection | Show one change, one result and one real-world limitation |

Total 240 minutes including transitions. At each rotation use 2 minutes to explain, 9 to try, 3 to explain a result and 1 to save and rotate. Three groups rotate in opposite starting positions: group 1 robot → camera → virtual; group 2 camera → virtual → robot; group 3 virtual → robot → camera.

The robot station uses Robot_Candy_Factory_Live_SO101.ipynb. Its default data are invented DEMO examples. A separately launched LIVE mode reads existing ROS topics only; the instructor controls physical enable, mode and stop using the RViz operations panel. The virtual station uses the short command sequence in the student notebook. The camera station combines observation uncertainty with paper conveyor sequencing; only the instructor demonstrates qualified hardware.

For the 190–225 minute challenge, pairs can choose the [offline Blockly factory](blockly-student.md) or the notebook challenges. The Blockly task is to predict and run, change the sequence, add color branches, then test unknown and save the result. Start in simulation on every laptop. During their supervised robot rotation or allocated challenge time, pairs can follow the [physical automation milestones](blockly-student.md#supervised-physical-automation-challenge): teach a fixed-jaw stroke using the leader, record the follower, replay once, then add an approved EV3 action. This replaces time within those slots; it adds no extra lesson slot. Dispensing, jaw control and payload pick/place each require their own commissioning. See [the instructor Blockly guide](blockly-workshop.md).

Use the worksheet to record answers; use the student notebook for explanations. External sites and their offline alternatives are in workshop-resources.md. The handbook reproduces the notebook text and code; build_notebooks.py is its source generator. Integrated physical sorting remains subject to the station's commissioning and acceptance evidence, independently of the notebook tests.
