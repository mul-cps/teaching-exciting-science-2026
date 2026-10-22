# Build your factory with blocks

**35-minute pair challenge.** A block is an instruction. Connected blocks run
in order; a repeat block runs its instructions more than once.

## Open your factory

Keep the extracted workshop folder together. Run:

```bash
python3 tools/run_blockly_station.py
```

On Windows use `python` instead of `python3`. Open **http://127.0.0.1:8877**.
The heading says **SIMULATION — no robot motion**. No internet or extra packages
are needed. Keep the terminal open; Ctrl+C closes the simulator.

![The example completed twice in simulation](assets/blockly/simulation-complete.png)

The **Starter** menu gives you five saved programs. Choose one and press
**Load starter**; **Download starter** keeps a copy you can reopen with **Open
blocks**. Loading replaces the blocks on screen, so save your work first.

| Starter | Your experiment |
| --- | --- |
| [1 · EV3 endpoints](../web/blockly-station/examples/01_ev3.json) | Predict connect → home → left → right, then change the order in simulation. |
| [2 · Teach and replay](../web/blockly-station/examples/02_teach_replay.json) | Record `my_short_move` at the supervised station. In simulation only, change that name to the invented `demo` stroke. |
| [3 · Color factory](../web/blockly-station/examples/03_color_factory.json) | Compare red, blue and unknown. Dispensing requires a commissioned feeder in live mode. |
| [4 · SO-ARM101 teach a stroke](../web/blockly-station/examples/04_arm_teach.json) | Try in simulation first. At the supervised physical station, capture a short manual leader demonstration with the jaw fixed. |

| [5 · Reset and reinitialize](../web/blockly-station/examples/05_reset_station.json) | Instructor: restart the station and verify recovery, then use a separate program for mode changes or movement. |

## Find your blocks

- **Station controls:** restart the entire station and verify recovery, reset with
  torque off, verify recovery, change modes and restart individual services.
  The instructor supervises physical control. Reset does not move the arm home.
- **SO-ARM101:** saved home, MoveIt mode, leader teleop relative, leader teleop
  absolute and torque OFF, plus the existing metric jaw-gap block. Saved home
  moves the arm; ask the instructor before physical mode changes.
- **Recordings:** record follower stroke by name for **1–30 seconds**, or replay
  a named stroke with a speed and timing choice. Recording captures measured
  positions during manual teaching; replay moves the follower.
- **EV3 conveyor:** connection, dispensing and conveyor commands.
- **Flow & decisions:** wait, repeat, color decision, STOP and the commissioned
  sorting-cycle block. **ROS Blocky · logging** supplies messages.

Loading a starter only changes the workspace. Press **Run sequence** when ready.

## Predict, build, run, check

1. **0–5 minutes:** Look at the example. Which instructions run twice?
   Predict the action list, then click **Run sequence** and compare.
2. **5–12 minutes:** Change the repeat count. Add a conveyor movement before
   dispensing. Make one change at a time. Swap who uses the mouse.
3. **12–22 minutes:** Add a color decision. Use the **Simulated sensor** menu
   to try red and blue. Make each choose a different transfer stroke or
   conveyor destination. The supplied `demo` stroke represents an invented
   robot movement.
4. **22–30 minutes:** Try **unknown**. In that branch, log a message such as
   “Unknown object: reject” and skip dispensing and gripping. Explain why
   choosing red or blue without a measurement would be wrong.
5. **30–35 minutes:** Save your blocks. Show another pair your prediction,
   observed action list and one corrected mistake.

| Experiment | Prediction | What happened? |
| --- | --- | --- |
| Changed repeat count | | |
| Unknown sensor reading | | |

## If the station needs recovery

Ask the instructor to inspect the station and support the arm before restarting.
Save your current blocks. Choose **5 · Reset and reinitialize**, click **Load
starter**, then **Run sequence**. This calls the existing station restart and
checks recovery; it does not automatically enable teleoperation or move home.
Read the result. If verification fails, its actual reason remains visible; do not
repeat motion commands to clear it.

After recovery succeeds, load or build a **separate** program using **SO-ARM101 →
MoveIt mode** before **Saved home**, or **leader teleop relative/absolute** before
teaching. The arm's teleop blocks both select the mapping and enable following.
The Station controls mapping selectors only choose relative/absolute mapping;
**Enable leader teleop** is a separate command there.

While recovery is pending, use only a flat sequence of recovery controls, messages,
waits and STOP. Put motion and mode changes in the next run, after verification.
**Run sequence** stays available for these recovery programs; **STOP station**
remains available independently.

## Supervised physical automation challenge

This optional extension uses time within your supervised robot rotation or the
challenge slot; it is **not another 35-minute activity**. Complete the simulation
challenge first. The instructor opens the live version using the station setup
and recording guide in the full repository.

1. Try **4 · SO-ARM101 teach a stroke** in simulation first. For physical use,
   ask the instructor to check readiness and supervise switching arm modes.
   Choose a short path clear of objects and a unique recording name, for example
   `my_short_move` if it has not been recorded before. Physical recordings cannot
   overwrite an existing name. Do not repeat a record block or put it in a loop.
2. Load starter 4. It selects relative leader teleoperation, records the follower
   for **5 seconds**, then returns to **MoveIt mode**. Keep the jaw fixed and move
   the human-operated leader during that recording window. Capture is read-only
   and applies no force; it does not automatically approach a starting pose.
   Cancellation or stale follower data fails the capture without saving a partial
   stroke. The **Record follower** and **Finish recording** buttons remain an
   alternative for instructor-supervised manual recording.
3. After capture, let the instructor inspect the path and supervise returning
   the follower to its recorded starting pose. Load **2 · Teach and replay**,
   use the same recorded name (`my_short_move` in the supplied placeholder), and
   replay once at **speed 0.5** in a separate **Run sequence**. Keep recording and
   playback in separate programs. Check completion before adding another action.
4. Add **Connect EV3**, then one instructor-approved conveyor endpoint. Check that
   each action reports completion. Add dispensing only after the instructor has
   commissioned the feeder reference; add jaw instructions only with measured,
   qualified jaw calibration over the requested interval. Only one **9 mm**
   calibration point is currently recorded; the metric jaw interval is **not
   qualified**. Keep the jaw fixed during each recorded stroke.
5. Build red and blue branches, predicting a different message or approved action
   for each. In simulation try red, blue and unknown. The unknown branch must log
   and **STOP before dispensing, gripping or replaying a manipulation**. In live
   mode use the measured EV3 color; the instructor must confirm it is available
   before the chosen action.

Start with this block sketch; substitute your own recorded name and the endpoint
approved for your station:

```text
Connect EV3
If sensor is red:
  Log "Red route"
  Conveyor to approved endpoint
  Replay my_short_move (speed 0.5)
Else if sensor is blue:
  Log "Blue route"
  Conveyor to approved endpoint
Else:
  Log "Unknown: ask the instructor"
  STOP
```

A confirmed **STOP** block ends the sequence normally. If STOP fails or cannot be
confirmed, ask the instructor to inspect and recover the station before recording
or running another sequence. The recovery message shows the reported reason.

To use **Saved home**, leave teleoperation first: connect a **MoveIt mode**
block before the **Saved home** block.

Before every replay, the instructor checks the recorded starting pose. Add a
**Wait** block for a deliberate process delay; a pause accidentally recorded while
teaching is part of the stroke timing, not a separate factory instruction.

**Success:** show your prediction, the measured recording, one completed replay
and completed EV3 actions. Show that unknown stops before manipulation. This is
a demonstrated sequence without object contact; it does not establish a qualified
whole sorting cycle. Dispensing, jaw movement and payload sorting need their
own instructor commissioning before use.

**Explain your system:** What observes the object? Which block decides?
What moves? What tells you the movement finished?

### Keep or skip pauses in a taught stroke

Replay uses **original timing** unless you change its dropdown. **Skip stationary
pauses** shortens only long sections with exactly unchanged arm positions; it does
not change the taught moving positions. Use **Wait** blocks for delays your
factory actually needs. The duration shown on the block is the requested time;
the robot controller may stretch it or reject the motion. If the optional timing
check rejects a stroke, keep original timing or teach a shorter stroke.


Taught-stroke replay does not manage detected-brick object ownership. Jaw-gap
blocks position the fingers; they do not declare an object grasped or attached.
Use the physical station for supervised free-space taught motion and EV3
sequences until the instructor has qualified the complete grasp/ownership flow.

### One complete commissioned cycle

The separate **Run one commissioned sorting cycle** block lets the native station
own dispensing, detection, grasp ownership and verified placement. Try it first
in simulation and inspect the completed count. The instructor can enable a
physical attempt only after commissioning the whole station. Taught free-space
strokes and jaw-position blocks remain separate activities; they do not declare
an object grasped or attached.
