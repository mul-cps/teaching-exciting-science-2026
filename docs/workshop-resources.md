# Workshop apps and short video

Use the prepared local Jupyter notebook on the workshop PC for Python. The printed instructor sheet supplies its actual address. The standard-library exercise files are the fallback. No public notebook service or JupyterLite deployment is required.

The two web activities below fit inside the four hours. Open their tabs before students arrive. Each pair needs a working browser and, for the perception activity, the approved object camera or laptop webcam. Keep a red object, a blue object, and an empty background available.

## Four-hour route

| Elapsed time | Activity | Minutes |
|---|---|---:|
| 00:00–00:25 | Ten-slide introduction | 25 |
| 00:25–00:35 | Blockly Maze | 10 |
| 00:35–01:05 | Python experiments | 30 |
| 01:05–01:25 | Teachable Machine perception | 20 |
| 01:25–01:35 | Break | 10 |
| 01:35–02:20 | Three station rotations | 45 |
| 02:20–03:10 | Complete the sorting factory | 50 |
| 03:10–03:45 | Challenges | 35 |
| 03:45–04:00 | Pair demonstrations | 15 |
| | **Total** | **240** |

## Blockly Maze: instructions in order

Open [Blockly Games: Maze, level 1 in English](https://blockly.games/maze?lang=en&level=1&skin=0). This is the Blockly Games site. Its [own introduction](https://blockly.games/about?lang=en) describes Maze as an activity about repetition and choices.

**Ten-minute task:**

1. Minute 0–2: instructor shows how to drag a block, attach it below another block, predict the movement, and choose **Run Program**.
2. Minute 2–7: pairs attempt levels **1–3**. The driver drags blocks. The navigator predicts what the character will do. Swap after the first completed level.
3. Minute 7–9: levels **4–5** are a stretch task for pairs who are ready. A pair can instead explain a correction to an earlier attempt.
4. Minute 9–10: ask, “What changed when you changed the order?” Connect the answer to ordered robot actions. Stop at ten minutes even if a level is unfinished.

Do not require all ten levels. Finishing a difficult puzzle is not the workshop's success criterion. If the site is unavailable, one partner reads three paper instructions, such as “forward, forward, turn left,” while the other traces them on a simple drawn path. Predict, trace, correct, and swap. This preserves the activity without installing software during class.

**Local browser check, 21 October 2026:** level 1 rendered in Chrome. A second forward block was attached, the program ran, and the page advanced to level 2. This check covers launch, dragging, and execution. Levels 3–5 were not completed during this check.

## Teachable Machine: examples and a missing object

Open [Teachable Machine: image project](https://teachablemachine.withgoogle.com/train/image). If the interface opens in another language, select English at the bottom and reload. The [Google Creative Lab project repository](https://github.com/googlecreativelab/teachablemachine-community) describes the tool as training recognition models in the browser without requiring machine-learning code.

Use **three classes** named `red`, `blue`, and `background / no object`. A class is a named group of examples. A model is the pattern the computer learns from those examples. The background class gives the exercise an explicit example of “there is no object to sort.” It does not guarantee that every unfamiliar object will be recognized correctly.

**Twenty-minute task:**

1. Minute 0–1: optionally show the one-minute excerpt below. Ask what the examples teach the computer. If playback is slow, demonstrate the app instead.
2. Minute 1–3: create and name all three classes. Explain that these labels are the possible answers being taught.
3. Minute 3–9: collect examples for each class. Aim for about 20–30 frames per class as a bounded classroom task. Move the red and blue objects slightly between examples. For background, remove the object and show the same work area. Keep faces out of the object examples.
4. Minute 9–13: choose **Train Model**. While it runs, ask pairs to predict the empty-background result. If training does not complete in this slot, move to the controlled-photo fallback.
5. Minute 13–18: first test red, blue, and empty background. Then predict and test a held-out green object and a changed background. Record the actual label and explain one mistake or limitation. The model has no green class, so adding a background class does not guarantee a correct reject decision. A large score is an output, not proof of correctness. The sorter's unknown/reject policy remains a separate rule.
6. Minute 18–20: explain one correct result and one mistake or limitation. Ask, “What should our sorter do when we cannot trust the label?” Use reject or ask for a new observation. Return to the workshop notebook; exporting or uploading a trained model is unnecessary.

Do not train only red and blue and then treat every highest-scoring answer as a reliable detection. Empty scenes and unfamiliar objects need explicit tests. Our workshop sorting rule still handles unknown or missing readings, even when a model appears certain.

**Prepared-photo alternative:** the red, blue, and background crops in `assets/lecture/tm-samples/` come from the actual station capture. Upload each folder to its matching class through the app's file picker if no webcam is connected. These nine correlated crops demonstrate the workflow; they do not establish general recognition quality. Test a new object/view separately.

**Complete twenty-minute fallback:** if web training is unavailable, use the prepared station photographs in the notebook. Identify the red, blue, and background areas, predict the supplied rule's result, and compare it with the displayed result. Record which conditions could make that result wrong. Do this alternative within the same twenty minutes, not as an additional assignment.

**Local browser check, 21 October 2026:** the image-project editor rendered in Chrome and adding/naming all three classes worked. A prepared-photo upload was attempted: the file chooser opened, but the browser automation extension rejected file selection because it lacks access to local file URLs. No broader browser permission was changed. A complete training/inference cycle, webcam access, and recognition quality remain unverified. They are part of the prepared-PC rehearsal. The text-only web fetch reported an unsupported browser; that is separate from the observed Chrome editor result.

## One optional video excerpt

[Teachable Machine 2.0: Making AI easier for everyone](https://www.youtube.com/watch?v=T2qQGqZxkD0), **Google**, total duration **2:08**. Show **00:00–01:00 only**, inside the perception activity's first minute. Pause and ask, “What examples would our red, blue, and empty-background groups need?”

The exact title, verified Google channel, and 2:08 player duration were checked on the direct YouTube watch page in Chrome on 21 October 2026. This is the only scheduled video excerpt: **one minute in total**. No SO-101 playlist is required or represented as verified. If adverts, network access, sound, or subtitles cause delay, skip the clip and use the app demonstration.

## Virtual robot station

The prepared local robot view and student notebook are the first choice. For the live robot station, the instructor demonstrates **teleoperation**, meaning a person controls the robot's movement. The **digital twin** is a screen model updated from measurements of the physical robot. Students compare a real joint with its screen counterpart. The display does not prove that a physical path is clear or that an object has been grasped.

An optional browser backup is [Interactive Robot Kinematics](https://vtavakkoli.github.io/interactive-robot-kinematics/), linked by its [actual project repository](https://github.com/vtavakkoli/interactive-robot-kinematics). Use only the **Joint Controls** area. The other mathematical panels are outside this beginner task. This generic model is not the workshop SO-101 and is not connected to its hardware.

**Fifteen-minute backup station:** predict which part will move when the first joint changes. Change `q1` from 0 to 30 and observe. Return it to 0, change one other joint, and describe what changed in ordinary words. Swap roles and repeat. Finish by explaining why a virtual movement alone does not tell us whether a real robot would hit a table. If the app does not render, use the notebook's sequence fallback. Do not spend the station installing packages or explaining equations.

**Local browser check, 21 October 2026:** the 3D arm rendered in Chrome. Changing `q1` from 0 to 30 changed the rendered arm and its displayed position from `(0.500, 0.665, 0.000)` to `(0.433, 0.665, -0.250)`. This verifies that this control responds in the checked browser. It does not qualify the local hardware, physical motion, or the teaching PC's graphics configuration.

For instructor background only, the official [LeRobot SO-101 documentation](https://huggingface.co/docs/lerobot/so101) identifies the leader and follower arms and their joints. Do not ask students to assemble, calibrate, or configure motors during this workshop.

## Before class

The instructor opens the local notebook, Maze, Teachable Machine, and the accepted robot/digital-twin view on the prepared PC. Repeat the short checks there. Set camera framing to objects, verify the three-class training example, and choose the listed fallback immediately if a site or device is unavailable. Use the remote-PC runbook for addresses and station controls. There is no public notebook-site QR code in these materials.
