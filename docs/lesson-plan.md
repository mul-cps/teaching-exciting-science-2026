# Autonomous Robot Sorting: four-hour workshop

Thursday, 22 October 2026. Six first-semester bachelor students, three pairs, no programming prerequisite. Student materials are in English. Explain unfamiliar terms aloud and demonstrate each task before independent work.

## Learning outcomes

Each pair runs and changes Python code, explains the observe / decide / act / check cycle, sorts five simulated objects correctly, and rejects missing or unrecognized colors. Physical demonstrations are supervised additions. A completed autonomous physical sort is not required for workshop success and is not claimed by these materials.

## Timetable (240 minutes)

| Elapsed time | Activity | Minutes | Result |
|---|---|---:|---|
| 00:00–00:25 | Eight-slide introduction and approved demonstration | 25 | Identify sensor, decision and actuator |
| 00:25–00:50 | A: First Python experiments | 25 | Predict, run and change a loop and a condition |
| 00:50–01:25 | B: Sensors and decisions | 35 | Recognize uncertainty and reject unknown readings |
| 01:25–01:35 | Break | 10 | |
| 01:35–02:20 | C: Three hardware / fallback rotations | 45 | Observe robot, camera and conveyor |
| 02:20–03:05 | D: Build sorting logic | 45 | Route five objects and explain counts |
| 03:05–03:45 | E: Independent challenges | 40 | Complete first two challenges, choose more |
| 03:45–04:00 | Pair demonstrations and reflection | 15 | Show one rule, its test and one limitation |

## Activities and pacing

A: Use `python exercises/01_python.py`. First predict the three greeting lines and the decision line. Change the text and repetition count. Try red / blue decisions. Swap the typing and predicting roles halfway through.

B: Use `python exercises/03_vision.py`. Predict red, blue and unknown results. Add missing / invalid readings. Compare the simulated decision with the prepared image examples or the instructor's live camera. Change lighting or object position only under supervision. Ask what the camera measures and what software infers.

C: Use the rotation matrix in [the instructor runbook](instructor.md). Three pairs visit robot, conveyor and camera for 15 minutes each. Only the instructor enables accepted physical demonstrations. Each pair writes one observation and one improvement. Do not calibrate or install software during the rotation.

D: Use `python exercises/04_sort.py`. Predict five-object counts, execute, reverse red and blue destinations, then test unknown readings. Use `python exercises/05_visual_sort.py` and open the generated `sorting-result.html` for animated results. Everything runs offline with standard Python and a browser.

E: Complete reverse rules and counting first. Then add a third color, handle sensor errors, test new sequences, or demonstrate a reliability improvement. All six challenges appear on the worksheet. A pair need not finish all six.

Closing: Allow 2–3 minutes per pair. Ask each pair to show a changed rule, a predicted and measured result, and a failure response. End with the most advanced physical demonstration actually accepted that day, or the simulator.

## Workshop success

Every pair changes a rule, tests a known five-object sequence, gets correct counts, and explains why unknown or missing readings go to reject. The instructor uses [the runbook](instructor.md) to record laptop and physical-station checks separately.
