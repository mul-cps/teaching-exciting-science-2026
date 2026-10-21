"""Predict, run, change: which readings should a robot accept?"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import SimCamera

# Change one reading to "green" or None. Predict the result first.
readings = ["red", "blue", "unknown"]
camera = SimCamera(readings)
for _ in range(len(readings)):
    color = camera.detect()
    if color in ("red", "blue"):
        print("Decision: pick")
    else:
        print("Decision: reject; ask instructor")
