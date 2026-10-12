"""Station 2 — Teach sorting decisions safely in simulation."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import SimRobot

robot = SimRobot()
robot.sort("left")
robot.sort("right")
print(robot.actions)
