# Instructor runbook: Autonomous Robot Sorting

Thursday, 22 October 2026. Four hours, six first-semester bachelor students, three pairs. Use [the timed lesson plan](lesson-plan.md), [eight introductory slides](workshop-slides.pptx) and [one-page worksheet](student-worksheet.docx). English student instructions. No programming or ROS prerequisite.

## 1. Before students arrive

Prepare Python 3.11 or newer and a text editor on each laptop. Open a terminal in the repository or extracted student bundle. Use the same tested Python command throughout the session (`python`, or `python3` where required). No package installation or internet should be necessary in class.

Run these commands on **each** student laptop, with networking disconnected:

```sh
python --version
python exercises/check_setup.py
python exercises/01_python.py
python exercises/02_robot.py
python exercises/03_vision.py
python exercises/04_sort.py
python exercises/05_visual_sort.py
python -m workshop
```

Open `sorting-result.html` in the laptop's browser. Confirm that the animation advances, labels are readable, and unknown objects reach reject. Edit one rule, rerun the Python file and reload the browser. Confirm that the output changes. These commands use simulated hardware and standard Python.

Prepare a portable copy in advance:

```sh
python exercises/make_student_bundle.py --output student-workshop.zip
```

Extract it into a writable folder on each laptop. Rerun the checks there. Keep a USB copy, printed worksheets, the slides and a PDF copy if the presentation application differs. Test all three laptops; one passing workstation does not establish that all laptops work.

| Computer | Python and four exercises | Visual HTML changed and replayed | Offline test | Checked by |
|---|---|---|---|---|
| Pair 1 | ___ | ___ | ___ | ___ |
| Pair 2 | ___ | ___ | ___ | ___ |
| Pair 3 | ___ | ___ | ___ | ___ |

## 2. Physical-station decision

Check the actual camera, robot and conveyor separately before class. Record the accepted demonstration for each. Secure the arm, conveyor, camera and cables. Keep an accessible stop and power disconnect. Only the instructor enables actuators. Brief students before touching leader controls. Never run a second motor owner or generic calibration command alongside the station controller.

| Station | Check before enabling | Fallback |
|---|---|---|
| Robot | Fresh feedback, calibrated leader/follower, known stop response, clear workspace and supervised motion | `python exercises/02_robot.py`; explain a sequence of simulated actions |
| Conveyor | Correct direction, working stop, clear mechanism, repeatable object stopping | `python -m workshop`; inspect the unpowered mechanism |
| Camera | Advancing real images and classifications under actual lighting | `docs/assets/vision-demo.png` and `docs/assets/vision-roi.png`, plus exercise 03 |

The included images are labeled teaching schematics, not photographs or validation evidence. A physical autonomous pick/place remains a separate commissioning result. Demonstrate it only if the complete cycle has actually passed the current station checks. Component tests or a successful simulation do not prove a physical sort.

If a stream becomes stale, a person enters the work envelope or motion differs from expectations, stop the hardware and switch that station to its fallback. Do not troubleshoot powered hardware while students wait. Students must not change calibration, joint limits, motor settings or service configurations.

## 3. Introduction: eight slides, 25 minutes

1. Can a robot run a factory? (3 min): ask what it must know before moving a block.
2. Our miniature factory (3 min): point to the actual camera, LEGO belt, arm and bins. The slide is a schematic.
3. Observe, decide, act, check (4 min): classify camera as sensor and motor as actuator. Ask how to check a result.
4. A camera sees pixels (3 min): compare red, blue and unknown examples. Discuss lighting, shadows and missing information.
5. Robot joints and gripper (3 min): show joints and a target, using only an accepted motion demonstration.
6. Your first Python rule (4 min): predict a tiny loop and color condition. Avoid a long syntax lecture.
7. What can go wrong? (3 min): ask students for a failure and a safe response.
8. Your challenge (2 min): pairs must change a rule, test five objects and explain rejection.

The presentation contains speaker notes with prompts and pacing. Continue with the exact [240-minute schedule](lesson-plan.md).

## 4. Three rotations: 01:35–02:20

Use pairs A, B and C. Each visit lasts 15 minutes, including transition. The instructor supervises any powered robot activity; if the other stations require supervision too, use their offline alternatives. Swap the typing / predicting roles between activities.

| Time | Pair A | Pair B | Pair C |
|---|---|---|---|
| 01:35–01:50 | Robot | Conveyor | Camera |
| 01:50–02:05 | Conveyor | Camera | Robot |
| 02:05–02:20 | Camera | Robot | Conveyor |

Robot question: How easy is it to reach the same target twice? Conveyor question: Does a sensor trigger guarantee a repeatable stopping position? Camera question: Which change makes recognition less reliable? Each pair records one observation and one improvement.

## 5. Sorting and independent challenges

At 02:20, return all pairs to their laptops. Start with the unchanged exercise 04. Ask for predicted counts before execution, then reverse the destinations. Explain that the mapping accepts known labels and sends everything else to reject. Use exercise 05 for visible feedback. Students edit Python, rerun and reload the generated page; the page does not control real hardware.

For fast pairs, use the six graded challenges on the worksheet. Complete reversing and counting first. Then introduce green, invalid/missing readings, new sequences or a reliability improvement. Do not require every pair to finish all six.

Expected baseline counts and current examples are in the tested exercise files. After editing a sequence, count it by hand first. Counts must sum to the sequence length, unknown/missing/invalid readings must be rejected, and changing a recognized color's destination must change its route.

## 6. Closing and readiness record

At 03:45, allow 2–3 minutes per pair. Each pair shows a changed rule, one prediction versus result, and a possible failure response. Close with the most advanced physical demonstration accepted that day, or the visual simulator.

Before class, tick only what you have actually checked:

- [ ] All three laptops pass the offline commands and edit / rerun test.
- [ ] Eight slides open correctly on the projector.
- [ ] Six one-page worksheets are printed.
- [ ] Red, blue and unknown objects are prepared and fit the station.
- [ ] Camera, robot and conveyor each have an accepted demonstration or fallback.
- [ ] USB student bundle and image examples are available without networking.
- [ ] Pairs and rotation order are assigned.

Student success means a tested five-object simulated sorter and a clear explanation. Hardware integration remains instructor infrastructure; students do not need to learn ROS during this session.
