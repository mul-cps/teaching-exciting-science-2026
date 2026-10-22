# Classroom notebooks

On the prepared PC, run `./station notebook` from the repository root. Open `Robot_Candy_Factory_Student.ipynb` in JupyterLab, save a team copy, then run cells with Shift+Enter. Read the explanations between code cells. Follow the notebook's mission order and save with Ctrl+S or Command+S.

For an extracted learner ZIP, keep notebooks, exercises, workshop and images together. Use the prepared Python: `.venv-workshop/bin/python notebooks/launch.py` on Linux/macOS or `.venv-workshop\Scripts\python.exe notebooks\launch.py` on Windows. The instructor may supply a different prepared interpreter. Initial preparation is described in [the prepared PC guide](../docs/fwmini4-workshop.md); package installation is not a classroom task.

Restarting a kernel resets in-memory variables, not edited files. Keep the original notebook as a clean copy. Rerunning a mission creates a fresh simulated mission. The student factory notebook uses controlled simulation reports; its photographs are separate real image examples, not live classifier output.

Without Jupyter, run `python3 exercises/check_setup.py`, then the exercise scripts in order 01, 03, 02, 04. `python3 exercises/05_visual_sort.py` produces `sorting-result.html`. These use Python's standard library; see [exercise instructions](../docs/student-exercises.md).

`./station sim` opens the local Blockly simulation. In the learner bundle, run `python3 tools/run_blockly_station.py` and open <http://127.0.0.1:8877>. See [the student Blockly guide](../docs/blockly-student.md).

`Robot_Candy_Factory_Live_SO101.ipynb` is the separate robot/twin observation activity. In the learner ZIP it uses clearly labelled invented DEMO data. On the prepared full station, `./station notebook --live` opens the instructor lab after hardware startup. Notebook cells do not command actuators. Use [the operator workflow](../docs/station-startup.md) for hardware and [the lesson plan](../docs/lesson-plan.md) for timings.
