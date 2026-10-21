# Sorting factory lesson plan
22 October 2026 · Four hours · First semester with no programming or robotics background

Students work in pairs using a prepared Jupyter notebook. By the end they can change a value, predict and run a short program, describe how a sensor report affects a decision, and test a simulated sorting factory. Students do not install software or connect hardware. The optional physical demonstrations are instructor controlled; the complete learning route works offline in simulation.

| Elapsed time | Duration | Activity | Evidence of learning |
| --- | --- | --- | --- |
| 00:00–00:25 | 25 min | Introduction with eight slides | Students identify sensing, deciding and acting |
| 00:25–00:50 | 25 min | A First Python instructions | Change name, repeat count and color; explain output |
| 00:50–01:25 | 35 min | B Camera observations | Compare actual objects with reported labels; reject uncertainty |
| 01:25–01:35 | 10 min | Break | Save notebooks |
| 01:35–02:20 | 45 min | C Three rotations of 15 min | Robot sequence, stop-before-pick, camera uncertainty |
| 02:20–03:05 | 45 min | D Integrated factory | Predict routes and baseline totals; change one rule |
| 03:05–03:45 | 40 min | E Independent challenges | Record two predictions and experimental results |
| 03:45–04:00 | 15 min | Closing | Show a change, evidence, and a simulation limitation |

Exactly 240 minutes. Activity 02 robot runs inside its 15 minute rotation, not as an additional 20 minute lesson. Keep the order A then B then C then D even though the script filenames are numbered by topic.

Use the notebook for substantial explanations and the one page worksheet for navigation and records. Do not teach a list of programming definitions before students see output. The instructor guide gives prompts, pacing and solutions. Student and instructor notebooks share their executable mission code with the four scripts; regenerate with `python tools/build_notebooks.py` after editing scripts.
