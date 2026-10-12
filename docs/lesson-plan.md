# Instructor lesson plan | 4 hours

**Audience:** six first-semester technical bachelor students, no robotics or Python prerequisite. **Languages:** German and English; demonstrate, then explain. **Theme:** sorting colored candy-shaped tokens with a CPS.

## Learning objectives

Students can explain the closed CPS loop, edit `for`/`if`/functions in Python, identify sensors and actuators, and describe how to handle unexpected measurements and hardware failures. At least two teams should show a correctly configured autonomous *simulated* sorter; hardware operation is an aspirational integration objective and depends on prior acceptance testing.

## Timetable

| Time | Plan | Format |
|---|---|---|
| 00:00–00:15 | Welcome, full safe demo, CPS loop | Instructor |
| 00:15–00:30 | Python mini-lesson: print, variables, if, for | All |
| 00:30–00:45 | Arm teleoperation demo and safety briefing | Instructor |
| 00:45–01:05 | Rotation A | Three teams of two |
| 01:05–01:25 | Rotation B | Three teams of two |
| 01:25–01:45 | Rotation C | Three teams of two |
| 01:45–01:55 | Break | |
| 01:55–02:20 | Connect camera decision to simulated actuation | Pairs |
| 02:20–03:05 | Sorting integration; physical demonstration ONLY IF APPROVED | All |
| 03:05–03:35 | Change rules, count objects, reject unknown | Pairs |
| 03:35–04:00 | Five-object challenge, recap, reflection | All |

## Rotation stations

1. **Robot:** instructor operates/calibrates official LeRobot leader and follower; students manually command and inspect a safe pose. Offline fallback `python exercises/02_robot.py`.
2. **Vision:** students display GoPro/UVC if proven, otherwise synthetic classification in `exercises/03_vision.py`. Discuss false positives and fixed ROI.
3. **Conveyor:** instructor advances the EV3 by a guarded small step once accepted. Offline fallback `python -m workshop`.

## Pacing and guidance

Keep code explanation under five minutes per block. One team member types, the other predicts what will happen; then switch. Ask: what does the camera *know*? What does a robot *do*? Why must we stop the conveyor before reaching into it? What should happen when classification fails?

## Student deliverables

A working 5-cycle program, changed sorting rule, observed result count, and one sentence about a safety or sensing limitation. Bonus: inspect real camera color classification. No cloud account, model training, ROS, or GPU tasks.

## Definition of workshop success

Level 1: all students execute and modify simple Python scripts. Level 2: teams build correct simulated closed loop. Level 3: instructor-approved real sorted objects, if end-to-end integration is bench-tested. Do not make Level 3 mandatory.
