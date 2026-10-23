"""Generated from curriculum/workshop.json. Offline beginner activity.
Run with the prepared Python; no physical commands are sent.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from workshop.blockly_client import Program, simulate_program
program = Program().log("Ready").ev3("eject").ev3("left_end")
result = simulate_program(program)
print("SIMULATION only:", result["complete"])
print(result["events"])
print("Shared Blockly JSON:", program.program)
