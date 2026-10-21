# Start your sorting factory

Your instructor has prepared Python, JupyterLab and Matplotlib. Extract the whole student ZIP first. Keep exercises, notebooks and workshop together; the pictures and simulator are local files. No internet is needed after preparation.

Launch the prepared Python with `notebooks/launch.py`. On Linux/macOS with the standard prepared environment use `.venv-workshop/bin/python notebooks/launch.py`. On Windows use `.venv-workshop\Scripts\python.exe notebooks\launch.py`. If your classroom uses a different prepared Python, the instructor supplies its location. No package installation is part of class.

Open Robot_Candy_Factory_Student.ipynb in JupyterLab. Use File → Save Notebook As to make a team copy. Click a code cell and press Shift+Enter to run. Read the explanation cells in between; output appears under the code. Run setup once, then A → B → C rotations → D → E. Save with Ctrl+S or Command+S.

Restarting the kernel resets variables in memory. It does not restore edited files. Keep the original notebook as your clean copy and copy it again if needed. Rerunning a mission creates a fresh simulation and resets that mission's counts.

Fallback: run `python3 exercises/check_setup.py`, then the four scripts in order 01,03,02,04. Edit their STUDENT AREA, save and rerun. For visual results run `python3 exercises/05_visual_sort.py` and open sorting-result.html in a browser. These scripts use only Python's standard library. The full explanations are in docs/student-exercises.md.

The main student factory notebook activities are simulation only. The photographs show the teaching station; the simulated reports are controlled examples and do not come from a live classifier.

The robot and twin station uses Robot_Candy_Factory_Live_SO101.ipynb. In the learner ZIP it runs invented DEMO data without ROS. This is a separate 15-minute station task, not an extra lesson. LIVE requires the full prepared station checkout and the instructor launcher described in docs/instructor.md. No cell commands an actuator; only the instructor controls hardware through RViz.

The exact lesson route is introduction 00–25, Blockly 25–35, Python 35–65, perception 65–85, break 85–95, stations 95–140, factory 140–190, challenges 190–225, demos 225–240 minutes.

During the 35-minute challenge you can also build a factory with blocks. Run
`python3 tools/run_blockly_station.py` and open `http://127.0.0.1:8877`.
Everything needed is in the extracted ZIP. Follow `docs/blockly-student.md`.
This learner version runs in simulation; live robot teaching uses the full
instructor station and separate setup.
