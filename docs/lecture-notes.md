<!-- Generated from curriculum/workshop.json; edit the source, then run tools/build_notebooks.py. -->

# Workshop slide speaker notes

Eight introductory slides total 20 minutes, followed by two slide cards per practical mission. Both cards share the mission duration; they do not add session time.

## Slide 1: Our fixed-fixture factory

2 minutes

Point to actual equipment. Ask which step needs an observation. Explain fixed pickup/drop is core; colors are later.

## Slide 2: Measure decide act check

3 minutes

Define sensor and actuator with the actual station. Learners point to examples. A command acknowledgement is not object arrival.

## Slide 3: Robot and screen twin

3 minutes

Compare leader/follower/twin. A matching screen is not an independent sensor or payload proof.

## Slide 4: Put the steps in order

3 minutes

Ask what happens if the belt is moving at pickup. Fixed-fixture sequence first. Separate calibrated jaw actions and validated strokes.

## Slide 5: An observation can be wrong

2 minutes

Label the altered report clearly. Discuss lighting, occlusion and missing data. Unknown does not certify free space.

## Slide 6: Blocks run in order

2 minutes

This is a local station screenshot, not an external Maze. Show short log/repeat sequence and STOP. No internet needed.

## Slide 7: Your first Python change

3 minutes

Read four lines aloud. Quotes store text; REPEATS stores a number. Ask what changes if TEAM changes. More code follows in practical missions.

## Slide 8: Teach inspect return replay

2 minutes

Swap pair driver/observer. Point to STOP. Physical execution requires the instructor and prepared verified recordings. Begin the first local mission.

## Slide 9: Make a short conveyor story card 1

25 minutes total for this activity, shared across its two cards.

## Make a short conveyor story

Mission `blocks` · 25 minutes · Put instructions in order and repeat a small section.

**Worked example** Ready → dispense → belt to pickup endpoint → inspect. In simulation each command changes a model; no physical object is verified.

**Predict** Which message appears first? Where should the object model stop?

**Change** Change Ready to your pair name. Add repeat 2 around a message, then around one tested short sequence.

**Run** Open local Blockly simulation; select this mission; run the starter once. Use STOP and Reset before a different test. Save your workspace.

**Observe** Read the command log and model state. Compare the repeated messages with your prediction.

**Explain** Why do instruction order and a stopped belt matter before pickup?

**Success check** Your pair can trace the order, predict a repeat count and explain one corrected sequence.

**Optional challenge** Try an intentionally wrong order in simulation and explain the result.

**Offline route** Draw each instruction on paper and move a paper object only when its instruction runs.

**Hint 1** Read from the top connection down.

**Hint 2** Repeat repeats every block inside it; start with a message.

**Instructor guidance** Expected order is message, eject, endpoint, inspect. Repeat does not imply two verified physical placements.

## Slide 10: Make a short conveyor story card 2

25 minutes total for this activity, shared across its two cards.

## Make a short conveyor story

Mission `blocks` · 25 minutes · Put instructions in order and repeat a small section.

**Worked example** Ready → dispense → belt to pickup endpoint → inspect. In simulation each command changes a model; no physical object is verified.

**Predict** Which message appears first? Where should the object model stop?

**Change** Change Ready to your pair name. Add repeat 2 around a message, then around one tested short sequence.

**Run** Open local Blockly simulation; select this mission; run the starter once. Use STOP and Reset before a different test. Save your workspace.

**Observe** Read the command log and model state. Compare the repeated messages with your prediction.

**Explain** Why do instruction order and a stopped belt matter before pickup?

**Success check** Your pair can trace the order, predict a repeat count and explain one corrected sequence.

**Optional challenge** Try an intentionally wrong order in simulation and explain the result.

**Offline route** Draw each instruction on paper and move a paper object only when its instruction runs.

**Hint 1** Read from the top connection down.

**Hint 2** Repeat repeats every block inside it; start with a message.

**Instructor guidance** Expected order is message, eject, endpoint, inspect. Repeat does not imply two verified physical placements.

## Slide 11: Separate the object from its report card 1

20 minutes total for this activity, shared across its two cards.

## Separate the object from its report

Mission `observe` · 20 minutes · Recognise missing and incorrect observations.

**Worked example** An actual blue brick can have a missing report. An actual red brick can be reported blue. Both need inspection.

**Predict** Which of the three reports is missing? Which disagrees with the labelled object?

**Change** Change one report to None; then change one known report to a wrong known color.

**Run** Run the short Python example or trace its three rows on paper. Compare the real labelled photo with a deliberately changed report.

**Observe** Record actual object, reported label and your decision separately. None means missing, never zero or proof of empty space.

**Explain** Why can a fresh color label still be wrong? What evidence would justify continuing?

**Success check** You identify one missing and one wrong report and choose pause/inspect for uncertainty.

**Optional challenge** Test unfamiliar green without adding a rule.

**Offline route** Use the supplied real photo and handwritten report cards; no camera or website is needed.

**Hint 1** Compare actual and reported columns one row at a time.

**Hint 2** A known word is still only a report.

**Instructor guidance** Missing is row two; wrong known label row three. Correct words alone do not validate a classifier.

## Slide 12: Separate the object from its report card 2

20 minutes total for this activity, shared across its two cards.

## Separate the object from its report

Mission `observe` · 20 minutes · Recognise missing and incorrect observations.

**Worked example** An actual blue brick can have a missing report. An actual red brick can be reported blue. Both need inspection.

**Predict** Which of the three reports is missing? Which disagrees with the labelled object?

**Change** Change one report to None; then change one known report to a wrong known color.

**Run** Run the short Python example or trace its three rows on paper. Compare the real labelled photo with a deliberately changed report.

**Observe** Record actual object, reported label and your decision separately. None means missing, never zero or proof of empty space.

**Explain** Why can a fresh color label still be wrong? What evidence would justify continuing?

**Success check** You identify one missing and one wrong report and choose pause/inspect for uncertainty.

**Optional challenge** Test unfamiliar green without adding a rule.

**Offline route** Use the supplied real photo and handwritten report cards; no camera or website is needed.

**Hint 1** Compare actual and reported columns one row at a time.

**Hint 2** A known word is still only a report.

**Instructor guidance** Missing is row two; wrong known label row three. Correct words alone do not validate a classifier.

## Slide 13: Change a name and a number card 1

30 minutes total for this activity, shared across its two cards.

## Change a name and a number

Mission `python` · 30 minutes · Read and edit a short Python program before sharing the same program representation with blocks.

**Worked example** TEAM stores text in quotes. REPEATS stores a number. print shows text. The indented line runs once for each number.

**Predict** What greeting and how many object lines will appear?

**Change** Change TEAM, predict, run. Then change REPEATS to 2, predict, run. Keep indentation.

**Run** Open the Student notebook, save a pair copy and run cells from the top with Shift+Enter. Run the shared Program example in simulation.

**Observe** Count output lines and compare the shared Python program JSON with the Blockly exported JSON.

**Explain** Which edit changed text? Which changed how often a line ran? What stayed in the same order?

**Success check** Both partners make one edit and explain it; shared commands are validated before execution.

**Optional challenge** Change a wait or message in both Python and Blockly and compare command traces.

**Offline route** Run exercises/01_python.py using the prepared Python or trace it on paper. No package downloads.

**Hint 1** Quotes surround text; do not quote the number.

**Hint 2** Four spaces keep print inside the loop.

**Instructor guidance** Original prints three object lines. Changing REPEATS resets the next run; Program simulation is not hardware proof.

## Slide 14: Change a name and a number card 2

30 minutes total for this activity, shared across its two cards.

## Change a name and a number

Mission `python` · 30 minutes · Read and edit a short Python program before sharing the same program representation with blocks.

**Worked example** TEAM stores text in quotes. REPEATS stores a number. print shows text. The indented line runs once for each number.

**Predict** What greeting and how many object lines will appear?

**Change** Change TEAM, predict, run. Then change REPEATS to 2, predict, run. Keep indentation.

**Run** Open the Student notebook, save a pair copy and run cells from the top with Shift+Enter. Run the shared Program example in simulation.

**Observe** Count output lines and compare the shared Python program JSON with the Blockly exported JSON.

**Explain** Which edit changed text? Which changed how often a line ran? What stayed in the same order?

**Success check** Both partners make one edit and explain it; shared commands are validated before execution.

**Optional challenge** Change a wait or message in both Python and Blockly and compare command traces.

**Offline route** Run exercises/01_python.py using the prepared Python or trace it on paper. No package downloads.

**Hint 1** Quotes surround text; do not quote the number.

**Hint 2** Four spaces keep print inside the loop.

**Instructor guidance** Original prints three object lines. Changing REPEATS resets the next run; Program simulation is not hardware proof.

## Slide 15: Watch teach inspect return replay card 1

15 minutes total for this activity, shared across its two cards.

## Watch teach inspect return replay

Mission `teach-demo` · 15 minutes · Understand that a recorded stroke needs validated start/end poses and explicit jaw state.

**Worked example** Leader 10°, follower 30°, leader now 15° gives relative request 35°. A request and a measured result are different.

**Predict** Will a replay start safely from any pose? What if the jaw state changed?

**Change** Ask the instructor to change one invented angle; predict before rerunning. Learners do not change physical modes.

**Run** Watch instructor: teach a short empty stroke, stop recording, inspect metadata and path, return safely to its validated start, then replay separately.

**Observe** Name start pose, endpoint, jaw state, identity/calibration and validity evidence. An invented demo recording is never a student-taught physical skill.

**Explain** Why must teaching and playback be separate runs? Why does a changed robot or calibration require reteaching?

**Success check** You can say teach → inspect/validate → return to start → replay, and point to STOP.

**Optional challenge** Predict a relative versus absolute joint request.

**Offline route** Use the invented joint example and paper start/end diagrams. Label evidence DEMO.

**Hint 1** A recording stores movement; it does not plan a safe approach from every pose.

**Hint 2** A screen model can match commands while a physical object slips.

**Instructor guidance** Relative is 35 and absolute 15. Demonstrate only with prepared supervised hardware; if validation fails use paper and report physical gate incomplete.

## Slide 16: Watch teach inspect return replay card 2

15 minutes total for this activity, shared across its two cards.

## Watch teach inspect return replay

Mission `teach-demo` · 15 minutes · Understand that a recorded stroke needs validated start/end poses and explicit jaw state.

**Worked example** Leader 10°, follower 30°, leader now 15° gives relative request 35°. A request and a measured result are different.

**Predict** Will a replay start safely from any pose? What if the jaw state changed?

**Change** Ask the instructor to change one invented angle; predict before rerunning. Learners do not change physical modes.

**Run** Watch instructor: teach a short empty stroke, stop recording, inspect metadata and path, return safely to its validated start, then replay separately.

**Observe** Name start pose, endpoint, jaw state, identity/calibration and validity evidence. An invented demo recording is never a student-taught physical skill.

**Explain** Why must teaching and playback be separate runs? Why does a changed robot or calibration require reteaching?

**Success check** You can say teach → inspect/validate → return to start → replay, and point to STOP.

**Optional challenge** Predict a relative versus absolute joint request.

**Offline route** Use the invented joint example and paper start/end diagrams. Label evidence DEMO.

**Hint 1** A recording stores movement; it does not plan a safe approach from every pose.

**Hint 2** A screen model can match commands while a physical object slips.

**Instructor guidance** Relative is 35 and absolute 15. Demonstrate only with prepared supervised hardware; if validation fails use paper and report physical gate incomplete.

## Slide 17: Explore three stations card 1

45 minutes total for this activity, shared across its two cards.

## Explore three stations

Mission `explore` · 45 minutes · Connect observations, instructions and the robot model.

**Worked example** Three pairs rotate through arm/twin observation, conveyor/observation sequencing, and offline Blockly/Python preparation.

**Predict** At your current station, predict one direction, report or sequence before the instructor or simulator acts.

**Change** Change only one paper angle, report or message; switch driver and observer halfway.

**Run** Each slot is 15 minutes: 2 predict, 7 change/run, 4 observe/explain, 2 save and transition. Use the rotation table.

**Observe** Arm: compare measured joint and twin when available. Conveyor: inspect stopped fixture/report. Offline: compare ordered traces. Label each evidence source.

**Explain** Which part measures, decides or acts? What can your evidence establish?

**Success check** Every pair completes all three positions with one recorded prediction and evidence statement per position.

**Optional challenge** Compare a fresh report with a stale one without guessing missing values.

**Offline route** All three positions have photo, paper and simulation versions; record DEMO/OFFLINE if no measured stream.

**Hint 1** The twin is a model updated from a stream.

**Hint 2** Use the same short sequence at the offline table.

**Instructor guidance** No three physical robots implied. Instructor controls the arm path; preserve transition time.

## Slide 18: Explore three stations card 2

45 minutes total for this activity, shared across its two cards.

## Explore three stations

Mission `explore` · 45 minutes · Connect observations, instructions and the robot model.

**Worked example** Three pairs rotate through arm/twin observation, conveyor/observation sequencing, and offline Blockly/Python preparation.

**Predict** At your current station, predict one direction, report or sequence before the instructor or simulator acts.

**Change** Change only one paper angle, report or message; switch driver and observer halfway.

**Run** Each slot is 15 minutes: 2 predict, 7 change/run, 4 observe/explain, 2 save and transition. Use the rotation table.

**Observe** Arm: compare measured joint and twin when available. Conveyor: inspect stopped fixture/report. Offline: compare ordered traces. Label each evidence source.

**Explain** Which part measures, decides or acts? What can your evidence establish?

**Success check** Every pair completes all three positions with one recorded prediction and evidence statement per position.

**Optional challenge** Compare a fresh report with a stale one without guessing missing values.

**Offline route** All three positions have photo, paper and simulation versions; record DEMO/OFFLINE if no measured stream.

**Hint 1** The twin is a model updated from a stream.

**Hint 2** Use the same short sequence at the offline table.

**Instructor guidance** No three physical robots implied. Instructor controls the arm path; preserve transition time.

## Slide 19: Dispense carry pick and place card 1

45 minutes total for this activity, shared across its two cards.

## Dispense carry pick and place

Mission `integrate` · 45 minutes · Assemble the supervised fixed pickup to fixed destination sequence.

**Worked example** EV3 dispense → conveyor to pickup fixture and stop → named taught skill fixed_pick_place. That skill binds validated approach/open, calibrated grasp, lift/transfer, release and return phases. Missing or uncommissioned qualification blocks physical execution; simulation labels every transfer synthetic.

**Predict** Where must the object and arm be before each next step? What should stop the run?

**Change** First reorder paper cards. In simulation use pick_place_recording named fixed_pick_place and change one message or delay. Physical execution requires instructor-validated phase recordings, measured jaw calibration and the qualification manifest; missing evidence is a blocking result.

**Run** Rotate: supervised integration, offline trace/rehearsal, failure/evidence desk. Each 15-minute slot includes 2 minutes transition. Run dispense, conveyor/stop and the named fixed_pick_place skill only through the existing shared endpoint. Instructor alone enables a qualified physical run; otherwise complete explicit synthetic simulation.

**Observe** Record dispense, stopped fixture arrival, grasp, lift, placement/release and return separately. A completed command does not prove object transfer.

**Explain** Which observation supports each claimed step? Which step remains unverified?

**Success check** Your pair explains the full fixed-fixture sequence and records honest per-step evidence. Physical completion requires observed object transfer; offline completion is labelled rehearsal.

**Optional challenge** After the core succeeds, discuss color-dependent destinations as separately validated taught skills.

**Offline route** Use the same fixed pickup/drop paper layout and named-skill simulation. Its grasp/lift/release outcomes are synthetic assumptions with physical_evidence false. Missing recordings or qualification means physical execution remains incomplete.

**Hint 1** Stop the conveyor before the arm approaches.

**Hint 2** Start/end poses and jaw state of adjacent taught strokes must agree.

**Instructor guidance** Do not substitute demo recordings. Current historical operational notes remain evidence until a new physical rehearsal supersedes them. Never infer payload success from a log.

## Slide 20: Dispense carry pick and place card 2

45 minutes total for this activity, shared across its two cards.

## Dispense carry pick and place

Mission `integrate` · 45 minutes · Assemble the supervised fixed pickup to fixed destination sequence.

**Worked example** EV3 dispense → conveyor to pickup fixture and stop → named taught skill fixed_pick_place. That skill binds validated approach/open, calibrated grasp, lift/transfer, release and return phases. Missing or uncommissioned qualification blocks physical execution; simulation labels every transfer synthetic.

**Predict** Where must the object and arm be before each next step? What should stop the run?

**Change** First reorder paper cards. In simulation use pick_place_recording named fixed_pick_place and change one message or delay. Physical execution requires instructor-validated phase recordings, measured jaw calibration and the qualification manifest; missing evidence is a blocking result.

**Run** Rotate: supervised integration, offline trace/rehearsal, failure/evidence desk. Each 15-minute slot includes 2 minutes transition. Run dispense, conveyor/stop and the named fixed_pick_place skill only through the existing shared endpoint. Instructor alone enables a qualified physical run; otherwise complete explicit synthetic simulation.

**Observe** Record dispense, stopped fixture arrival, grasp, lift, placement/release and return separately. A completed command does not prove object transfer.

**Explain** Which observation supports each claimed step? Which step remains unverified?

**Success check** Your pair explains the full fixed-fixture sequence and records honest per-step evidence. Physical completion requires observed object transfer; offline completion is labelled rehearsal.

**Optional challenge** After the core succeeds, discuss color-dependent destinations as separately validated taught skills.

**Offline route** Use the same fixed pickup/drop paper layout and named-skill simulation. Its grasp/lift/release outcomes are synthetic assumptions with physical_evidence false. Missing recordings or qualification means physical execution remains incomplete.

**Hint 1** Stop the conveyor before the arm approaches.

**Hint 2** Start/end poses and jaw state of adjacent taught strokes must agree.

**Instructor guidance** Do not substitute demo recordings. Current historical operational notes remain evidence until a new physical rehearsal supersedes them. Never infer payload success from a log.

## Slide 21: Find one failure and test a repair card 1

20 minutes total for this activity, shared across its two cards.

## Find one failure and test a repair

Mission `improve` · 20 minutes · Use a controlled change and evidence to improve a sequence.

**Worked example** A missing fixture observation pauses the sequence after one inspected object. Fix the observation path before retrying.

**Predict** What happens when the middle report is None?

**Change** Test one failure: missing report, wrong order, missing recording or STOP. Change one cause only and reset.

**Run** Reproduce in simulation or paper; record expected versus actual. Repair and rerun the same input.

**Observe** Capture the failure message, stopped state and repaired trace. No physical fault injection by learners.

**Explain** Did your test detect the problem? What could still fail on hardware?

**Success check** You show a failing prediction, one repair and a repeated check with its limits.

**Optional challenge** Use the optional color-routing example with three reds and one blue to expose swapped rules.

**Offline route** All failures can be traced with paper command/report cards.

**Hint 1** Choose one input so cause and effect stay clear.

**Hint 2** A successful run alone cannot identify a wrong known report.

**Instructor guidance** Missing report stops after one; avoid relaxing admission or faking a report to pass.

## Slide 22: Find one failure and test a repair card 2

20 minutes total for this activity, shared across its two cards.

## Find one failure and test a repair

Mission `improve` · 20 minutes · Use a controlled change and evidence to improve a sequence.

**Worked example** A missing fixture observation pauses the sequence after one inspected object. Fix the observation path before retrying.

**Predict** What happens when the middle report is None?

**Change** Test one failure: missing report, wrong order, missing recording or STOP. Change one cause only and reset.

**Run** Reproduce in simulation or paper; record expected versus actual. Repair and rerun the same input.

**Observe** Capture the failure message, stopped state and repaired trace. No physical fault injection by learners.

**Explain** Did your test detect the problem? What could still fail on hardware?

**Success check** You show a failing prediction, one repair and a repeated check with its limits.

**Optional challenge** Use the optional color-routing example with three reds and one blue to expose swapped rules.

**Offline route** All failures can be traced with paper command/report cards.

**Hint 1** Choose one input so cause and effect stay clear.

**Hint 2** A successful run alone cannot identify a wrong known report.

**Instructor guidance** Missing report stops after one; avoid relaxing admission or faking a report to pass.

## Slide 23: Show one prediction and result card 1

10 minutes total for this activity, shared across its two cards.

## Show one prediction and result

Mission `share` · 10 minutes · Explain what your program did and what the evidence cannot prove.

**Worked example** Pair A shows prediction, one change, observed result and one limit; then pairs B and C follow.

**Predict** Which single output best supports your claim?

**Change** Shorten your demonstration to one change and one observation.

**Run** Use 1 minute preparation, 3 demonstrations of 2 minutes, 3 minutes shared reflection. Both partners speak.

**Observe** Show saved trace, worksheet or observed supervised step; label simulation, paper or physical.

**Explain** Name one next test that would increase confidence.

**Success check** All three pairs demonstrate and state a specific limitation.

**Optional challenge** Explain how a wrong known color differs from a missing report.

**Offline route** Present the worksheet and paper trace.

**Hint 1** Read your prediction before the result.

**Hint 2** A trace supports instruction order, not physical arrival.

**Instructor guidance** Keep all three demonstrations within ten minutes. Do not require successful hardware to explain evidence.

## Slide 24: Show one prediction and result card 2

10 minutes total for this activity, shared across its two cards.

## Show one prediction and result

Mission `share` · 10 minutes · Explain what your program did and what the evidence cannot prove.

**Worked example** Pair A shows prediction, one change, observed result and one limit; then pairs B and C follow.

**Predict** Which single output best supports your claim?

**Change** Shorten your demonstration to one change and one observation.

**Run** Use 1 minute preparation, 3 demonstrations of 2 minutes, 3 minutes shared reflection. Both partners speak.

**Observe** Show saved trace, worksheet or observed supervised step; label simulation, paper or physical.

**Explain** Name one next test that would increase confidence.

**Success check** All three pairs demonstrate and state a specific limitation.

**Optional challenge** Explain how a wrong known color differs from a missing report.

**Offline route** Present the worksheet and paper trace.

**Hint 1** Read your prediction before the result.

**Hint 2** A trace supports instruction order, not physical arrival.

**Instructor guidance** Keep all three demonstrations within ten minutes. Do not require successful hardware to explain evidence.
