"""Final team challenge — Change the colors and destinations."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import Factory, SimBelt, SimCamera, SimRobot

factory = Factory(SimBelt(), SimCamera(), SimRobot())
for _ in range(5):
    factory.one_cycle({"red": "left", "blue": "right"})
print("Result:", factory.counts)
