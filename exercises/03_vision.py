"""CAMERA ACTIVITY (35 min): Turn camera observations into decisions.

Run: python exercises/03_vision.py

This is a SIMULATED camera: its observations are listed in COLORS.
It is not looking at a real image. That lets everyone learn without a camera.

TASKS
1. Predict the outcome for every object, then run.
2. Change the second color to 'red'. Does its destination change?
3. Add 'green'. What happens? Should the robot guess?
4. CHALLENGE: Create a rule for green. Does 'unknown' still get rejected?
5. DISCUSS: How could a real camera report a wrong color?
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workshop import SimCamera

# ===== STUDENT AREA =====
COLORS = ["red", "blue", "unknown", "red", "blue"]
RED_DESTINATION = "left"
BLUE_DESTINATION = "right"
# ========================

camera = SimCamera(COLORS)
counts = {"left": 0, "right": 0, "reject": 0}
routes = []

for object_number in range(len(COLORS)):
    color = camera.detect()
    if color == "red":
        destination = RED_DESTINATION
    elif color == "blue":
        destination = BLUE_DESTINATION
    else:
        destination = "reject"  # never guess when a sensor is uncertain

    if destination not in ("left", "right", "reject"):
        destination = "reject"
    routes.append(destination)
    counts[destination] += 1
    print("Object", object_number + 1, "->", destination)

print("\nFinal counts:", counts)
print("Were any objects rejected? Why?")
