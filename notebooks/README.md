<!-- Generated from curriculum/workshop.json; edit the source, then run tools/build_notebooks.py. -->

# Offline classroom notebooks

Build an offline robot factory from ordered instructions. Start with one fixed pickup fixture and one fixed destination. Pair roles: driver edits; observer predicts and checks; swap halfway. Only the instructor enables supervised hardware.

On the prepared PC run `./station notebook`, open `Robot_Candy_Factory_Student.ipynb` and save a pair copy. Run cells from the top with Shift+Enter. Restart and rerun from the top after a confusing edit. Every executable cell uses invented inputs or explicit simulation; no learner cell imports ROS or commands hardware.

In the extracted learner ZIP, keep all folders together. Use the prepared Python to run `python notebooks/launch.py`. No installation or internet is a classroom task. Without Jupyter, run `python exercises/check_setup.py`, then scripts 01, 02 and 03. Script 04 is the optional color-routing extension, and 05 renders that extension as an offline HTML report.

Start local Blockly with `python tools/run_blockly_station.py`; open http://127.0.0.1:8877 and choose the first mission. `./station sim` is the prepared-PC shortcut. Both languages share validated command JSON and the same interpreter.

`Robot_Candy_Factory_Live_SO101.ipynb` defaults to invented DEMO observations. The prepared instructor launcher `tools/launch_live_workshop.py --live --ros-wrapper /path/to/wrapper` sets WORKSHOP_LIVE=1 for read-only live topic capture in a bounded wrapper subprocess. Missing LIVE data stay explicitly unavailable; there is no DEMO fallback. No cell sends motion or changes modes.

Simulation, paper, reports and measured physical outcomes are separate evidence. Never claim a grasp or placement from a command log alone. Unknown or stale observations require pause and inspection.
