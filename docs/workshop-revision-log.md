# Beginner workshop revision evidence

## Current route from 22 October 2026

The authored source is `curriculum/workshop.json`. The current beginner route is
20 + 25 + 20 + 30 + 10 + 15 + 45 + 45 + 20 + 10 = 240 minutes, with eight visual
introductory slides totaling 20 minutes and separate practical mission cards.
Six beginners work in three pairs. Both three-position rotation circuits use
15-minute slots including transitions. The core sequence uses a fixed pickup
fixture and a fixed destination; color-dependent destinations are an extension.
The essential Blockly, Python, notebook and paper routes work offline.

| Review pass | Actual evidence for this revision | Remaining boundary |
|---|---|---|
| Beginner clarity | Each practical mission exposes worked example, predict/change/run/observe/explain, success, two hints and optional challenge. Short greeting and shared Program simulation examples replace the old color-first core. | No first-time learner observation or teaching trial has been performed. |
| Pacing | Validator checks the approved ten durations, eight intro slides totaling 20 minutes, three pairs and two circuits of three stations. Rotation cards allocate transitions. | This is a desk check, not an elapsed 240-minute classroom rehearsal. |
| Consistency | Primary lesson plan, instructor guide, worksheet, practical cards, browser curriculum, scripts and three notebooks are generated from the one JSON source; source and artifact checks detect drift. Fresh-kernel execution checks the simulated examples and altered routing inputs. | Hardware commissioning cannot be established by generators or software tests. |
| Presentation | Actual editable PPTX/PDF and five DOCX/PDF handouts are built. Full slide/page renders are reviewed; photographs retain actual station provenance and altered reports are labelled. | Classroom projector and actual-size printer review remain to be performed. |
| Full physical rehearsal | No new physical acceptance claimed. Paper and simulation explicitly identify their evidence source. | Full physical rehearsal remains pending: the 27 October USB inventory finds the D435i on fwmini4 and EV3/arm USB adapters on the workstation; taught fixed-fixture payload grasp/lift/place/release and three consecutive complete EV3-to-return cycles are unverified. |

## Evidence update — 23 October 2026

Commits `e32974d` and `f0b445a` resolve six source/runtime clarity issues:
offline executable Python using the shared AST, simulation reset, worked
examples, hints, progressive browser help, and matching example/prediction/
worksheet instructions. The schedule remains 240 minutes for six beginners in
three pairs. Generated materials are committed in `b6dc86d`.

A temporary nbclient runner executed all three canonical notebooks in separate
fresh kernels: Student 9/9, Instructor 9/9 and Live 7/7 code cells (25 total),
with no errors or skips and all three source hashes unchanged. Live used
`WORKSHOP_LIVE=0`, explicitly labelled invented data with no hardware, and
produced two inspected plots. This records default offline execution only.

The final [editable deck](workshop-slides.pptx) has 24 slides (eight introduction,
16 mission cards) and 24 speaker notes. The five handouts have 36 pages,
including the six-page [worksheet](student-worksheet.pdf); with the slide PDF,
there are 60 final PDF pages. All-page visual evidence combines 65 author image
views across initial and final renders with exact pixel comparisons; this is
not 65 unique final pages. Independent final review inspected worksheet pages
4/5 and slides 17/23. [Material source/hash checks](materials-build.json) and
[learner ZIP](student-workshop.zip) source/hash/allowlist checks passed: 90 ZIP
entries, with no instructor, private station or qualification paths.

Pacing remains a desk check: no elapsed classroom rehearsal has occurred.
Full physical rehearsal, three consecutive complete cycles and fwmini4
deployment remain pending. CUA had no available browser, so no new interactive
graphical Blockly QA is claimed. The historical evidence below retains its
original scope.

## Historical revision and operational evidence

The dated material below records the previous route and its evidence. Its
18-slide deck, 25-minute introduction and previous activity durations have been
superseded by the current source above. Historical hardware observations retain
their original scope and do not qualify the new fixed-fixture sequence.

# Five workshop revision passes

The current deliverable is an 18-slide whole-class deck: ten introductory slides for 25 minutes and eight timed activity checkpoints. The complete route is **25 + 10 + 30 + 20 + 10 + 45 + 50 + 35 + 15 = 240 minutes**. All five passes below have actual results. Physical gripper calibration and a complete autonomous sorting cycle remain separate acceptance work.

| Pass | Findings and actual changes | Check and evidence |
|---|---|---|
| **R1 — Structure** | Discarded the rejected eight-slide photo/text deck. Rebuilt a whole-class story from mission and prediction through real equipment, the control cycle, sensing/movement, leader/follower/twin, pixels/labels, rules/examples, Python, and uncertainty. Added a checkpoint slide for every workshop activity. | 18-slide PPTX/PDF and all 18 speaker-note sections. Introduction remains 25 minutes. No extra video slot changes the four-hour route. |
| **R2 — Beginner pedagogy** | Defined unfamiliar words before using them in exercises. Added questions with expected answers, a worked uppercase `COLOR` condition, pair-role swaps, a prediction/action/finish line for every activity, and explicit transitions. Separated rejecting an unknown input from stopping unexpected physical movement. | Final notes cover complete beginners. Blue produces right, yellow produces reject. Unknown input does not automatically mean the factory stops. |
| **R3 — Sources, timing, correctness** | Removed archive AI concept JPGs that had been mistaken for photographs. Used actual equipment and actual side-view screen capture. Labeled the single new hero as a concept illustration. Verified primary web links, bounded Maze to levels 1–3, used three perception classes, and required held-out green/changed-background tests. Matched the notebook's three-way rule. | Source hashes and code-execution record in the private lecture evidence. Chrome checks: Maze first level ran to level 2; three Teachable Machine classes were named; the optional virtual arm responded to a joint change. Exact Google video title/channel/2:08 checked on its direct page. Only 00:00–01:00 is scheduled. |
| **R4 — Design and rendering** | Introduced a new navy/paper/coral/cyan editorial theme, full-bleed concept/actual visuals, large type and generous space. Replaced prediction cards with open destination-bin diagrams. Preserved image aspect ratios. Changed the twin view so the whole arm and gripper are visible. Split REJECT and STOP into separate visual cases. | All 18 pages rendered and inspected individually after these changes, plus the montage. Package integrity, layout/heading geometry, and font checks must pass before integration. The editable rule is 27pt. LibreOffice rendering is checked; Microsoft PowerPoint rendering is not claimed. |
| **R5 — Rehearsal and release** | On fwmini4, executed Student, Instructor and DEMO live notebooks in fresh kernels, checked the 18-page PDF and exact PPTX hash, and ran all six extracted ZIP entry points without installed packages. The ZIP includes all nine local classifier training pictures. Authenticated Jupyter and fresh/stale/fresh fake ROS observation checks passed. Local hardware passed full reset, saved home, MoveIt Plan then Execute, and both EV3 endpoint operations. | Remote software rehearsal at `f253460` is recorded in the prepared-PC runbook; subsequent code updates receive incremental checks there. No physical devices were attached to fwmini4 during rehearsal. Browser Teachable Machine training/inference was not qualified: its prepared-photo upload was rejected by the automation extension. The complete 20-minute local notebook photo activity remains the alternative. No autonomous physical pick/place success is claimed. |

Visual provenance, generated-hero prompt, and Blockly attribution/license are committed in `docs/assets/lecture/`. The build uses only committed assets. The private lecture evidence includes individual `class-slide-01.png`–`class-slide-18.png`, the montage, package/layout reports, a media-source audit, and the exact extracted-code results.

## Evidence update — 27 October 2026

The current clarity desk review inspected the canonical curriculum, actual text
in all three notebooks, 24 slides and notes, 36 handout pages, and the browser
projection. Three gaps were repaired in the canonical source: the Python mission
now loads the same Ready/eject/left_end instructions as its shared Program example;
teach/replay terms are defined before use; and the paper observation activity
provides its three invented report rows and the existing human-labelled photo
collage. The browser copy omits document image markup, and the teaching slide
retains a real replay/jaw prediction prompt. The full relative/absolute example
remains in the activity and speaker notes.

Independent scoped review passed after both follow-up defects were corrected.
The supervisor's focused curriculum/client/offline run passed 65 tests. All three
final notebooks executed in fresh kernels using the existing system Python and a
private kernelspec. All 116 material-manifest hashes and 89 comparable files in
the 90-entry learner ZIP match. Changed slides and paper pages were rendered and
inspected; the final follow-up changes only slide 15 visually, with the other 23
slides and all 36 handout pages pixel-identical to the preceding reviewed build.
These are text clarity, consistency and changed-render checks, not a timed
240-minute rehearsal, first-time learner trial or full physical acceptance.

Canonical source SHA256:
`edb483460e6f49d06d2b8fc0a1e4c53b27b3ce2116494ba325fcb071bc5ec141`.
Durable evidence archive `workshop-clarity-repair-20261027.tar.gz` in the station's
local evidence directory has SHA256
`198c3720b34a45ec8260f6c39bac3786cbaf9d9c79a26f7295df44b4e8cfabf9`;
its receipt binds the 26 reviewed source/output files and preserved render hashes,
notebook outputs, failures and commands. Scoped review SHA256:
`a8f0396ce67fc244076e1b0705e370ded5a9011735b14045516eeccfe321021b`.

The D435i is enumerated on fwmini4 at USB 5000 Mb/s. EV3 and the two arm USB
adapters remain on the workstation. Commissioning, complete taught pick/place,
STOP/recovery, three consecutive physical cycles and the final main release
remain open.
