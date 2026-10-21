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

## At the real robot station

The instructor opens a separate live version. You can teach a short movement
with the leader arm, record the follower, then use a replay block. Hold the jaw
at one opening during each recording; use separate calibrated grip/release
instructions. A replay starts at the recorded starting pose. The instructor
checks the path and returns the arm there before replay.

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
