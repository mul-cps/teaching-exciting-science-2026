"""Team challenge: predict the counts, run, then change the rules."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import Factory, SimBelt, SimCamera, SimRobot

# First reverse left/right. Then add green. Unrecognized readings are rejected.
rules = {"red": "left", "blue": "right"}
objects = ["red", "blue", "red", "unknown", "blue"]
factory = Factory(SimBelt(), SimCamera(objects), SimRobot())
for _ in range(len(objects)):
    factory.one_cycle(rules)
print("Result:", factory.counts)
# Challenges: count each bin, add a color, try None, test another sequence.
