<!-- Generated from curriculum/workshop.json; edit the source, then run tools/build_notebooks.py. -->

# Offline workshop resources

Build an offline robot factory from ordered instructions. Start with one fixed pickup fixture and one fixed destination. Pair roles: driver edits; observer predicts and checks; swap halfway. Only the instructor enables supervised hardware.

| Minutes | Duration | Session |
|---|---|---|
| 0-20 | 20 | Meet our factory |
| 20-45 | 25 | Ordered instructions |
| 45-65 | 20 | Observations and uncertainty |
| 65-95 | 30 | First Python edits |
| 95-105 | 10 | Break |
| 105-120 | 15 | Teach inspect and replay demonstration |
| 120-165 | 45 | Explore three stations |
| 165-210 | 45 | Build the physical sequence |
| 210-230 | 20 | Improve one failure |
| 230-240 | 10 | Three pair demonstrations |

## Local Blockly simulation

`python tools/run_blockly_station.py; open http://127.0.0.1:8877`

## Prepared classroom notebook

`./station notebook; open Robot_Candy_Factory_Student.ipynb`

## No Jupyter fallback

`python exercises/01_python.py; then 02_robot.py, 03_vision.py and 04_sort.py`

Learner handouts: student-handbook.pdf, student-worksheet.pdf and activity-cards.pdf. Source and scripts remain usable without downloads. External training websites are optional instructor enrichment after the essential local route.
