<!-- Generated from curriculum/workshop.json; edit the source, then run tools/build_notebooks.py. -->

# Local Blockly robot factory

Start `python tools/run_blockly_station.py`, then open http://127.0.0.1:8877. Choose a mission, predict, edit one thing, run, inspect and explain. Save your workspace before changing missions. STOP ends the program; later blocks must not run. Use **Reset simulation** between experiments. Missing recordings are a reason to inspect and prepare, never a reason to invent a physical success.

Simulation, paper, reports and measured physical outcomes are separate evidence. Never claim a grasp or placement from a command log alone. Unknown or stale observations require pause and inspection.

## Make a short conveyor story

Mission `blocks` · 25 minutes · Put instructions in order and repeat a small section.

**Worked example** Ready → dispense → belt to pickup endpoint → inspect. In simulation each command changes a model; no physical object is verified.

**Predict** Which message appears first? Where should the object model stop?

**Change** Change Ready to your pair name. Add repeat 2 around a message, then around one tested short sequence.

**Run** Open local Blockly simulation; select this mission; run the starter once. Use STOP and **Reset simulation** before a different test. Save your workspace.

**Observe** Read the command log and model state. Compare the repeated messages with your prediction.

**Explain** Why do instruction order and a stopped belt matter before pickup?

**Success check** Your pair can trace the order, predict a repeat count and explain one corrected sequence.

**Optional challenge** Try an intentionally wrong order in simulation and explain the result.

**Offline route** Draw each instruction on paper and move a paper object only when its instruction runs.

**Hint 1** Read from the top connection down.

**Hint 2** Repeat repeats every block inside it; start with a message.

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