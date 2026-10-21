"""ROBOT ROTATION (15 min): Program movements of a SIMULATED robot.

Run: python exercises/02_robot.py

A robot only follows instructions: it does not know where to put things.
Predict the sequence of destinations before running.

TASKS
1. Make the robot sort LEFT, LEFT, RIGHT (edit COMMANDS).
2. Predict how many objects reach each bin. Check the printed counts.
3. Add a fourth command. What changes?
4. CHALLENGE: Add 'reject' to the sequence. What could cause a reject?

No physical robot is connected; nothing can move on your desk.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workshop import SimRobot

# ===== STUDENT AREA =====
COMMANDS = ["left", "right", "left"]
# ========================

robot = SimRobot()
bin_counts = {"left": 0, "right": 0, "reject": 0}

for destination in COMMANDS:
    if destination not in ("left", "right", "reject"):
        print("Unknown destination - reject:", destination)
        destination = "reject"
    robot.sort(destination)
    bin_counts[destination] += 1

print("\nRobot commands:", robot.actions)
print("Objects per bin:", bin_counts)
print("Did the output match your prediction?")
