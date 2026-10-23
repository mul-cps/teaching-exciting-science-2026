"""Generated from curriculum/workshop.json. Optional color-routing extension.
Run with the prepared Python; no physical commands are sent.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import Factory, SimBelt, SimCamera, SimRobot
OBJECTS = ["red", "blue", "red", "unknown", "blue"]
RULES = {"red": "left", "blue": "right"}
factory = Factory(SimBelt(), SimCamera(OBJECTS), SimRobot())
for _ in OBJECTS:
    factory.one_cycle(RULES)
print(factory.counts)
print("SIMULATION routes:", factory.robot.actions)
assert sum(factory.counts.values()) == len(OBJECTS)

for destination, count in factory.counts.items():
    print(destination, ":", count)
print("Total objects:", sum(factory.counts.values()))
