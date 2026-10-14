#!/usr/bin/env python3
"""Launch the resident, JSON-lines SO-ARM101 controller."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop.station.arm_session import main


if __name__ == "__main__":
    raise SystemExit(main())
