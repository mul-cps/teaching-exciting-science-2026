# Single elbow P24 commissioning trial

This opt-in resident-owner trial changes only STS3215 ID 3 (`elbow_flex`)
`P_Coefficient` from the measured value 16 to 24. I=0 and D=32 remain unchanged.
The native acceleration, velocity and torque profile remains 8 / 160 body,
220 gripper / 700 body, 250 gripper. Endpoint acceptance remains 2 degrees.
This is an experiment, not a qualified correction.

PID coefficients are in the SDK's EPROM table, not its volatile SRAM profile.
The owner never calls `SOFollower.configure()` or `torque_disabled()`, never
opens another bus, and never automatically enables torque for this feature.
Every P or Lock write requires newly measured all-six OFF and normal bounds,
matching calibration/source, and rediscovered exact USB identity/owner port.
Only ID 3 P and, when necessary, ID 3 Lock can be written. All six original
P/I/D and Lock values must match the expected inventory afterwards.

## Startup configuration

Set this field in the existing station configuration before starting a new
explicitly approved owner generation:

```json
{"arm": {"elbow_stiffness_profile": "elbow-p24-v1"}}
```

This is a field to merge into the complete station configuration, not a complete
configuration. Ordinary startup omits the field. Unknown values and using either
named mode with `--observe-only` are rejected.

The owner must observe an elbow baseline of P16/I0/D32. It exclusively creates
`<calibration filename without suffix>.elbow-p24-pending.json` beside the resolved
calibration file and fsyncs both file and directory before any unlock or P write.
The original all-six P/I/D and Lock snapshot, physical identity, calibration,
source hashes and trial ID are durable recovery authority. Existing pending
journals are never overwritten or adopted as a new baseline.

The journal remains pending for the entire trial. Motion uses the existing
admission, collision, timing, tracking and endpoint checks. The existing profile
label gains `;elbow-p24-v1`; the owner log records the trial ID, source cohort,
original inventory and exact readback. Component-only gripper commissioning is
rejected while this trial owns EEPROM restoration.

## Restoration and interrupted trials

Normal shutdown first verifies all-six torque OFF, then restores ID 3 P16 and
the original Lock state, verifies every original P/I/D and Lock register, and
only then archives the receipt and durably removes the pending journal. A
single-shot write exception may have applied; it is never blindly retried.
Failed restoration leaves the journal pending and cleanup unsuccessful. The
same process does not automatically repeat a failed restoration attempt.

A pending journal blocks both ordinary startup and another P24 startup,
regardless of the new state-file path. Restore through the same resident owner
and station lock, with the same physical identity, calibration and source:

```json
{"arm": {"elbow_stiffness_profile": "restore-elbow-p16-only"}}
```

Restore-only startup validates the journal and current all-six OFF normal state;
it accepts only original P16 or trial P24, never an arbitrary new coefficient.
The prior owner's UUID is provenance, not a requirement that the dead process
remain alive. Only read-state, passive diagnostics, stop and exit commands are
admitted after recovery; no motion is admitted by that owner generation.

Do not delete a pending journal to bypass recovery. Malformed, foreign-USB,
wrong-calibration or wrong-source journals reject recovery before coefficient
writes. An unavailable bus, abnormal torque-off pose, changed non-elbow PID,
unverified readback or failed durable receipt also prevents successful cleanup.
Resolve the cause and perform an explicitly approved restore-only startup.
A power interruption before journal creation completes cannot have performed a
coefficient write; a partial journal still blocks startup for manual inspection.

After a verified restoration receipt, remove the named field before the next
ordinary owner generation. Do not change sources while a trial is pending.
