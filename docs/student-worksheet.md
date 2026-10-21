# Autonomous Robot Sorting

**Team:** __________  **Names:** ____________________  **22 October 2026**

**Mission:** Observe, decide, act, check. Sort red and blue objects. Reject unknown or missing readings. Work in pairs: one types, one predicts; swap roles after each activity.

## A. First Python experiments (25 min)

Run `python exercises/01_python.py`. Predict first: how many greeting lines? ____
Change the greeting and make it repeat five times. Actual greeting lines: ____
Change the color decision so red goes right and blue goes left. Explain your `if`: ____________________

## B. Sensors and decisions (35 min)

Run `python exercises/03_vision.py`. Predict what red / blue / unknown produce.
Try an unfamiliar color and a missing reading. What should happen? ____________________
Compare the image examples or live camera. One recognition problem: ____________________

## C. Robot, conveyor and camera (3 × 15 min)

Follow your assigned rotation. Keep hands clear of moving equipment. The instructor enables physical motion. Offline: `python exercises/02_robot.py` and `python -m workshop`.
One observation: ____________________  One improvement: ____________________

## D. Build your sorter (45 min)

Run `python exercises/04_sort.py`. Use the five-object sequence shown in the file.
Predict: left ____ right ____ reject ____  Actual: left ____ right ____ reject ____
Reverse red and blue destinations; run again. Does unknown stay rejected? ____
Run `python exercises/05_visual_sort.py`; open `sorting-result.html` in a browser.
Change a Python rule and run again. Which block moved differently? ____________________

## E. Challenge board (40 min)

Finish 1 and 2 first, then choose. Tick a box after you can explain your result.

- [ ] 1 Easy: Reverse the destinations and verify the result.
- [ ] 2 Easy: Count left, right and reject. Do they sum to the number of objects?
- [ ] 3 Medium: Add green and choose a destination. Test it.
- [ ] 4 Medium: Reject missing (`None`) and invalid color readings.
- [ ] 5 Medium: Create two sequences. Predict counts before running each one.
- [ ] 6 Advanced: Improve reliability. Demonstrate your idea in simulation.

## Show your result (2–3 min per pair)

Our rule: ____________________  Test result: ____________________
Sensor: __________  Actuator: __________  Failure and response: ____________________

**Stop and ask the instructor if equipment behaves unexpectedly.** No installation or ROS commands are needed for these activities.
