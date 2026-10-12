"""Station 3 — Change the simulated colors; no GPU or internet needed."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop import SimCamera

camera = SimCamera(["red", "blue", "unknown"])
for _ in range(3):
    color = camera.detect()
    print("Decision:", "pick" if color != "unknown" else "ask instructor")
