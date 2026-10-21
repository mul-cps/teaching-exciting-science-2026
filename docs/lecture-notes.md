# Your first robot factory

A whole-class deck for a four-hour workshop with no programming or robotics prerequisite. Slides 1–10 teach the introduction in 25 minutes. Slides 11–18 remain on screen as the activity checkpoints. The complete timetable is 25 + 10 + 30 + 20 + 10 + 45 + 50 + 35 + 15 = 240 minutes. All students work in pairs: the driver uses the keyboard or mouse and the navigator predicts and checks. Swap after the first run and during longer tasks.

The primary Python environment is the prepared local Jupyter notebook. A code cell is a small area of runnable instructions. The standard-library exercises are the alternative. Web activities and their alternatives appear in [workshop resources](workshop-resources.md). The one optional video excerpt is a minute inside the perception activity, not extra lecture time.

## Slide 1: Your first robot factory

**00:00–03:00, 3 minutes**

Start with a question before introducing vocabulary. “Here is the factory we will build in code. What must it know before it can sort one object?” Give everyone fifteen seconds to look. Ask each pair for one answer. Expected responses include color, location, destination, and whether the object arrived. Accept everyday phrases such as “it needs eyes.” Translate that to “a way to obtain information.”

Explain the mission: “You will write decisions for a miniature sorting factory. A program is a list of instructions a computer follows. A robot can use instructions to make physical movements. Today you can begin without knowing how to program.” Distinguish the pictured concept from equipment in the room: “This is an illustration of the idea. In a moment we will look at our actual station.”

Give a concrete example. “I can point and say ‘put that over there.’ What does a machine need instead?” Expected: identify the object and destination clearly. Say that each pair will later test five objects in a simulation, which is a computer version of a task. Real equipment and its screen model will help us investigate movement and observation. Do not promise an autonomous physical sorting run.

**Transition:** “Before writing any code, let us make the destination choices ourselves.”

**Visual source:** `assets/lecture/factory-concept-hero.png`, a deliberately illustrated concept created for this workshop. It is visibly labeled Concept illustration. It is not a photograph of the real station.

## Slide 2: Where does it go?

**03:00–05:00, 2 minutes**

Read the three destinations. “Our first rule is red goes left, blue goes right, and unknown goes to reject. A destination is where we want an object to end up. Reject is the destination for an object that our rule cannot safely classify.” The native colored containers are an editable teaching diagram. They do not depict physical bins installed on the table.

Ask pairs to predict the result for red, blue, and green in that order. Expected: left, right, reject. Ask why green should go to reject. Expected: the rule does not recognize green. Do not accept “green is probably blue” as a safe response. Explain that unknown is useful information rather than a student mistake.

Tell students that we will eventually reverse destinations and test the rule again. The color of a destination label is a convention we choose; the program must follow the actual written rule.

**Transition:** “Which parts of our real station will obtain information and make movements?”

## Slide 3: The equipment in our room

**05:00–08:00, 3 minutes**

Point to the actual photo, then the corresponding equipment in the room. “This picture comes from the station camera. The camera is above the table, outside its own view. The conveyor is a moving belt that carries objects. The robot arm changes the location of the gripper. The gripper opens and closes to hold an object.” The black mechanism on the left is the hand-controlled leader; explain its role on slide 6.

Ask, “Which part gives us a picture?” Expected: camera. “Which parts can change an object's location?” Expected: conveyor and arm. If a student says the computer moves it, clarify that the computer sends instructions and the mechanism makes the movement.

Explain that left, right, and reject are destinations in the simulation. The photograph contains no sorting bins. Do not imply that the colored containers in the concept illustration are present in this capture. If physical containers are used later, identify today's actual placement explicitly.

A prepared instructor demonstration can show one accepted movement here, but keep the focus on the question: “What did you observe?” If a demonstration is unavailable, point to the parts and continue. A movement demonstration does not establish autonomous sorting.

**Transition:** “These parts must work together in a complete cycle.”

**Visual source:** `assets/lecture/station-current.png`, unmodified actual station camera capture from 21 October 2026.

## Slide 4: A complete cycle

**08:00–11:00, 3 minutes**

Read the four words slowly. “Observe means obtain information. Decide means use a rule. Act means make a movement. Check means compare what happened with what we wanted.” Trace the native editable diagram from top to bottom and back to the first step.

Use one blue object as a worked story. Ask pairs to fill each step: get a picture, choose the right destination, make a movement, then look to see whether the object arrived. Clarify that the camera gives a picture and a program gives the picture a color label. The camera does not know our destination convention.

Ask, “The program sent an instruction, but the object is still on the belt. Which step reveals that?” Expected: check. Ask, “Are we finished?” Expected: no. “Sending an instruction and achieving the result are different events. We need another observation before deciding what to do next.”

**Transition:** “We need words for the part that measures and the part that moves.”

## Slide 5: Information and movement

**11:00–13:00, 2 minutes**

“A sensor is a device that measures something. A camera is a sensor: it gives us an image, which is a picture. An actuator is a part that makes a physical movement. A motor is an actuator: it turns.” Point to the large words and their examples.

Ask students to offer an everyday sensor. Accept a thermometer measuring temperature or a microphone measuring sound. Return to the workshop example. Ask, “Can the turning motor by itself tell us the color?” Expected: no. “Can the image by itself move a brick?” Expected: no.

Keep this distinction concrete. Students do not need component specifications or acronyms. Ask one student to point at a sensor and another to point at an actuator in the room.

**Transition:** “A person can guide the robot, and we can compare the physical movement with a model on screen.”

## Slide 6: You move. It follows.

**13:00–16:00, 3 minutes**

Explain the three labels. “The leader is the handle you move. The follower is the robot arm that follows. Teleoperation means that a person guides the movement. A digital twin is a screen model updated from measurements of the real robot.” Point to the leader and follower in the room. Point to the supplied screen image as an example of the model and measured scene. Do not present this saved screenshot as a currently running connection.

“A joint is a place where parts of the arm can move relative to one another. It can bend or rotate. Several joints together change where the gripper is. The gripper opens and closes.” Invite students to bend their elbow and open their hand. This analogy introduces the mechanism without requiring geometry.

During the robot station, the instructor will select an accepted demonstration or the virtual alternative. Ask students to compare one real joint with the matching screen joint. Ask, “Does the screen prove that a real movement will miss the table?” Expected: no. It may omit an obstacle or a current detail. Explain clearance as enough empty space for a movement. Before moving sideways near an object, first provide clearance above the table and other objects.

**Transition:** “Movement needs a decision. What information does our camera actually provide?”

**Visual source:** `assets/lecture/digital-twin-view.png`, cropped from the user-supplied actual station screenshot `digital-twin-screen.png` (original clipboard image `codex-clipboard-5CgWs7.png`). It illustrates the real application view, not a current physical acceptance result.

## Slide 7: What does the camera give us?

**16:00–18:00, 2 minutes**

“The camera gives a picture. An image is made of pixels. A pixel is one tiny square of color. The enlarged middle example uses a deliberately small number of squares so we can see them. A program examines image information and attaches a label, such as red.” The reduced example is not a claim about the camera's native resolution.

Point to the pale, dark, and red squares. Ask, “Are all the pixels red?” Expected: no. There is a background, a dark edge, and reflections. Ask, “Would changing the light or object position leave exactly the same image?” Expected: no. Explain that choosing the right image region and recognizing uncertain information both matter.

**Transition:** “We can write a decision explicitly, or teach a model with examples. We will try both approaches.”

**Visual sources:** `brick-red.png` and `brick-pixels.png`, actual station crop and its deliberately reduced enlargement.

## Slide 8: Write a rule or teach examples

**18:00–20:00, 2 minutes**

“On the left, we write the choice ourselves: red goes left and blue goes right. On the right, we collect examples and teach a model to recognize a pattern. A model is the pattern the computer learns from those examples. Training means using the examples to build that model.” The destination rule and the image recognizer solve different parts of the task.

“We will teach three named groups: red, blue, and an empty background. A class is one named group. The background group gives the exercise examples where there is no object to sort.” Ask why a two-group red/blue activity would be incomplete for an empty table. Expected: it has not been taught a separate empty-background answer. Explain that an unfamiliar object can still receive a misleading label, so we must test a new view and keep the unknown/reject response in the sorter.

No equations or machine-learning code are needed. The Teachable Machine activity is later, after Python, and has a prepared-photo alternative if the camera or site is unavailable.

**Transition:** “First we will write the destination rule in a language the computer can run.”

**Resource:** [Teachable Machine](https://teachablemachine.withgoogle.com/train/image), with primary project reference and bounded activity in [workshop resources](workshop-resources.md).

## Slide 9: A rule the computer can run

**20:00–23:00, 3 minutes**

“Code is instructions written in a language a computer can run. This language is Python. Read from top to bottom.” Point to `COLOR = "red"`. “COLOR is a name that stores a value. We call it a variable. The value is the text red. Quotation marks mean text. One equals sign stores the value.” This is an example input, not a live sensor reading.

Point at `if COLOR == "red":`. “If asks a question. Two equals signs compare values: is the stored color the same as this text? The colon ends the question line. The line beginning farther to the right belongs to that choice. We call that indentation.” Read `print("left")`: “Print shows output, the result the program displays. It does not move the arm.” Read `elif`: “Otherwise, if the color is blue, show right.” Read `else`: “Otherwise, show reject.”

Ask what appears for red. Expected: left. In the prepared local notebook, point to the matching condition cell and click Run. Show the output underneath. Change only the first value to blue, ask for a prediction, then run. Expected: right. Ask what yellow would produce. Expected: reject. The following Python activity gives students time to try this themselves and introduces a short loop, meaning repetition. Do not add a loop to this slide.

**Transition:** “What should we do when the information is missing, wrong, or no longer current?”

## Slide 10: When the input is uncertain

**23:00–25:00, 2 minutes**

Use one concrete example. “Imagine the camera takes a picture and then the brick moves. The old location is no longer useful. What next?” Expected: stop, observe again, and ask the instructor before further physical action. Explain that repeating an old movement can repeat the mistake.

“For unknown or missing color information in the notebook, use reject. For unexpected physical movement, stop and ask the instructor. Do not reach into the moving station.” Ask, “Can the program announce success just because it sent the instruction?” Expected: no. It must check what happened.

Introduce pair roles: driver uses the mouse or keyboard, navigator predicts and checks. Swap after the first run. Tell each pair to choose the first driver now.

**Transition:** “Open Blockly Maze at level 1 from the resources. Your first task is to describe movement with blocks before typing Python.” The introduction ends at minute 25.

## Slide 11: Build a path

**25:00–35:00, 10 minutes**

**Prediction:** before pressing Run Program, the navigator says where the character will move. **Action:** attach blocks, run, compare, and correct. **Finish line:** explain one correction to a sequence.

Spend the first two minutes showing drag, attach, and run. Students attempt levels 1–3 for five minutes. Swap after the first completed level. Levels 4–5 are stretch for pairs who are ready, not a completion requirement. Reserve the last minute for, “What changed when you changed the order?” Expected: the path or destination changes.

If the website is unavailable, use the paper instruction exercise in [workshop resources](workshop-resources.md). One partner reads the sequence and the other traces it. Do not add setup time or require all ten Maze levels.

**Transition at minute 35:** “Blocks and Python both describe instructions. Open the first Python activity in the prepared local notebook.”

**Link and visual source:** [Blockly Maze level 1](https://blockly.games/maze?lang=en&level=1&skin=0). `blockly-view.png` is an actual browser screenshot after the checked first level, showing the next level. Students start at level 1.

## Slide 12: Change one thing

**35:00–65:00, 30 minutes**

**Prediction:** state what the cell will print. **Action:** run it, change one value or instruction, and rerun. **Finish line:** show one correctly predicted result and explain the change that caused it.

Use the prepared student notebook's Python section. First show where the cell and Run control are. Begin with a short output example, then the `COLOR` condition that matches slide 9. Do not assume students know variables, quotes, or indentation. Explain an error as information about what Python could not follow. Let students repair one small change rather than replacing the whole cell.

Introduce repetition with the notebook's short loop. Explain “repeat this action for each item” before asking students to read the loop syntax. Keep list-building and optional tasks out of the core route. Swap driver and navigator around minute 50. At minute 62 ask each pair to show a prediction and output. Use the standard-library exercise alternative if Jupyter is unavailable.

**Transition at minute 65:** “Our written choices need an input. Now we will teach a camera model using examples.”

## Slide 13: Teach three answers

**65:00–85:00, 20 minutes**

**Prediction:** what should an empty background produce? **Action:** collect red, blue, and background examples, train, and test a new view. **Finish line:** explain one mistake or limitation using an observed result.

The optional Google video excerpt occupies only minute 65–66: [Teachable Machine 2.0: Making AI easier for everyone](https://www.youtube.com/watch?v=T2qQGqZxkD0), channel Google, full duration 2:08. Show 00:00–01:00 only. It is the only scheduled video minute in the workshop. Skip immediately if playback causes delay.

Use minutes 66–68 to create and name the three classes. Use 68–74 for example collection, 74–78 for training, 78–83 for testing, and 83–85 for discussion. Prepared object photos can replace the webcam. Before testing, ask pairs to predict both a green object that was not in training and a changed background. Then test and record the actual output for each. A third background class does not guarantee that an unfamiliar object becomes reject. A high score can still be wrong. Keep the learned label separate from the sorter's unknown/reject policy. Each pair must explain one observed mistake or one limitation. The [resources](workshop-resources.md) explain the alternatives. Swap roles during collection and keep the camera aimed at objects.

If collection or training cannot finish in the slot, use the controlled-photo notebook challenge for the same twenty minutes. The fallback includes a prediction, a displayed result, and a limitation. It is a complete activity, not extra work after the web activity.

**Transition at minute 85:** “Save your observation. We have a ten-minute break, then rotate through three stations.”

## Slide 14: A real break

**85:00–95:00, 10 minutes**

State the wall-clock return time corresponding to minute 95. Leave the station controls with the instructor. Students can leave their notebooks open. During the break the instructor prepares the next approved station views without asking students to install software.

**Transition at minute 95:** assign the three pairs to different stations and explain the rotation order. Each station lasts fifteen minutes.

## Slide 15: Three stations

**95:00–140:00, 45 minutes**

Each pair visits all three stations in three fifteen-minute rounds: 95–110, 110–125, and 125–140. Swap roles halfway through each round. Each pair records one observation and one limitation per station.

**Robot and digital twin:** the instructor uses the accepted prepared demonstration. Students predict one joint movement, compare the real and screen movement, and explain a difference or limitation. The live notebook is read-only for student observation. Real motion remains instructor controlled. If a live connection is unavailable, clearly identify the virtual alternative.

**Camera and conveyor:** identify what the camera measures and what moves the belt. Predict a color or missing-reading result, compare the actual displayed reading, and identify one reason it might be wrong. Do not calibrate or change station services during the rotation.

**Virtual robot:** predict the effect of one joint change, make the change in the prepared virtual view, and compare. Use the notebook sequence alternative if necessary. The optional browser simulator and its bounded task are in [resources](workshop-resources.md). The generic browser model is not connected to the real SO-101 and is not its digital twin.

**Transition at minute 140:** “Return to your laptop pair. We will connect the observations and decisions into the complete sorting task.”

## Slide 16: Make the whole factory work

**140:00–190:00, 50 minutes**

**Prediction:** red, blue, red, unknown, blue should produce two left, two right, and one reject. **Action:** build the notebook sorting logic and run the sequence. **Finish line:** five recorded outcomes whose total equals five, with an explanation of the unknown response.

Begin in simulation. Give pairs time to read the supplied inputs and destination rule before running. Ask the navigator to predict the full sequence, not only the total count. Then compare the displayed outputs. If the total is wrong, find the first point where observation and prediction differ. Change one thing and rerun. Swap roles halfway through the activity.

Use the prepared visual result if the notebook includes it. A simulated result checks the program's rule. It does not establish that a physical arm picked an object. Any supervised physical demonstration is an instructor-selected addition, using the accepted station procedure. It does not replace the pair's simulation result or require autonomous physical success.

At minute 185, ask pairs to save their working result and identify one place where the rule handles uncertainty.

**Transition at minute 190:** “Keep the working version. Choose a change and design a test before you edit.”

## Slide 17: Choose an upgrade

**190:00–225:00, 35 minutes**

**Prediction:** state how a chosen rule change should alter the result. **Action:** change the rule and compare the predicted and observed result. **Finish line:** show a test that caught or could catch a mistake.

The core upgrades are swapping red and blue destinations and counting the results. Students who finish can choose a new color, a missing reading, or another optional notebook challenge. Do not require every challenge. Keep a copy of the working sorter before changes.

Swap the typing and predicting roles during the activity. If a pair is stuck, ask it to reduce the test to one object, say its expected destination aloud, and trace the condition. At minute 220, ask each pair to choose the single change and test it will show in the final demonstration.

**Transition at minute 225:** “Choose who will explain the rule first. Both partners will speak.”

## Slide 18: Show your factory

**225:00–240:00, 15 minutes**

Allow each of the three pairs about four minutes, leaving three minutes for the closing reflection. One partner explains the changed rule and prediction. The other runs or shows the result and explains one limitation. The pair can use its saved simulation result if a live demonstration would delay the class.

Ask: “What did you predict? What happened? What would have made the answer wrong?” Expected examples include an unknown color, missing reading, changed light, moved object, or a rule implemented incorrectly. Look for evidence in the pair's recorded outputs rather than a claim that the code works.

Close with, “What would you improve next?” Connect answers to the cycle: better observation, clearer rule, careful movement, or a stronger check. End at minute 240. Do not present unverified autonomous physical picking as the workshop outcome.
