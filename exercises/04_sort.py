"""FACTORY ACTIVITY (45 min): Program a complete SIMULATED sorting factory.

Run: python exercises/04_sort.py

Each cycle: belt moves -> belt stops -> camera measures -> robot sorts.
The provided Factory takes care of this safe sequence.

TASKS
1. Predict the counts for the five objects in OBJECTS. Run to check.
2. Change the rules: RED goes right, BLUE goes left.
3. Add an unknown object. Verify that it goes to reject.
4. Change the order of OBJECTS. Are your counts still correct?
5. CHALLENGE: Add 'green' to OBJECTS and a rule that routes it left.
6. DISCUSS: What should happen if the real camera stops sending images?

You are programming simulation only. No hardware is controlled here.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workshop import Factory, SimBelt, SimCamera, SimRobot

# ===== STUDENT AREA =====
OBJECTS = ["red", "blue", "red", "unknown", "blue"]
RULES = {"red": "left", "blue": "right"}
# ========================

factory = Factory(SimBelt(), SimCamera(OBJECTS), SimRobot())

for object_number in range(len(OBJECTS)):
    print("\n--- Object", object_number + 1, "---")
    factory.one_cycle(RULES)

print("\nFINAL FACTORY RESULT")
for destination, number in factory.counts.items():
    print(destination, ":", number)

print("Total objects:", sum(factory.counts.values()))
print("Belt advances:", factory.belt.steps)
print("Robot destinations:", factory.robot.actions)
assert sum(factory.counts.values()) == len(OBJECTS), "Some objects are missing!"
