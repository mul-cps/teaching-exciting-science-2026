# Named taught pick/place

`Program().pick_place_recording("fixture_left", speed=.5)` and the Blockly
**taught pick/place** block submit `{op:"pick_place_recording",name:"fixture_left",speed:0.5}`
to the existing shared station program service. The instructor drawer exposes the
block. Students cannot supply paths, endpoints, targets, attachment transforms or
qualification flags. Server admission session/epoch and STOP remain mandatory.

There is **no commissioned physical skill supplied with this checkout**. Start
the server with `--skills /private/instructor/skill-root` to configure an instructor
library. Missing, uncommissioned, synthetic, invalid and revalidation-required
skills have explicit status reasons. Restart loads structure; every invocation
still requires fresh live admission. Source, model, robot or calibration changes
require reteaching/revalidation, even if a recording's numeric samples still fit.

Simulation explicitly assumes contact/lift/transfer/release, moves the synthetic
brick to place, and returns `physical_evidence:false`, `simulation:true` and all
three physical verification flags false. Ordinary replay and jaw never establish
pickup. A physical result requires the exact selected track plus independent
pickup, lift and release verification.

## Instructor commissioning

Stage all inputs under a private root, separate from student recordings. Four
fixed-jaw v2 strokes are mandatory: `approach` (from taught approach start to
contact), `lift`, `transfer` (to fixed drop), and `return` (after opening and
detaching). Teach open-jaw approach/return separately from contact-jaw lift and
transfer. Each segment must start within two degrees of the current measured
arm and within 0.015 rad of its recorded jaw. The initial pregrasp is a separate
MoveIt joint plan to the taught approach start; no unplanned prefix is inserted.

Pin the existing qualified execution config, jaw-gap, jaw-joint-angle, TCP/object,
camera-to-arm and effective robot model. The resident must advertise its exact
**loaded** jaw-gap and joint-angle file digests on the existing calibration status
service; a visual jaw proxy is inadmissible. Digests are captured from the same
bytes parsed at load, so editing files does not silently alter loaded provenance.
All relative artifact paths resolve within the private root. Hashes cover exact
bytes. Manifest fields and phase names are strict; extra fields are rejected.

Supply SI checkpoint JSON with exactly `pickup`, `drop`, `approach_start`, `return`,
each `[x,y,z,qx,qy,qz,qw]` with a unit quaternion. `pickup` is the geometric grasp
frame; other checkpoints are the qualified tool-link frame. Pin fixture identity
and the SHA256 of surveyed fixed pickup/drop provenance, and independently saved
instructor physical qualification evidence. Changed fixtures require reteaching.

Use `python -m workshop.station.taught_commission --help` to build a draft from
these files. The helper writes `qualified:false`, refuses overwrite and never
accesses hardware. It has no flag to grant qualification. An instructor must
independently qualify the complete transaction and review the draft/evidence
before setting `qualified:true`. Hashes establish provenance; they do not certify
successful physical pickup. Keep all manifests private; none belongs in a student
bundle as a physically qualified example.

## Execution gates

The existing grasp CLI owns the transaction and resident FJT/gripper actions own
motion. The named session observes three fresh stable images to select one exact
eligible target. The CLI checks the fixed pickup, qualified registration and
current model; locks that target; opens the calibrated gap; plans pregrasp; and
substitutes recorded contact/lift/transfer paths **before** its existing endpoint
FK checks. Each taught phase revalidates manifests, fresh Trigger owner identity,
loaded calibration hashes, start and fixed jaw. A single-inflight owner timer
refreshes status during actions; any generation/source/calibration change aborts.
The resident bridge still retimes and collision checks every submitted curve.

Contact must report stalled and not reached-goal before correlated provisional
attachment. Attachment does not prove pickup. Independent elevated RGB-D track
observations must pass the existing lift gate before place/release. Failed lift
retains attachment and stops. The jaw must reach the open gap before detach; the
recorded return is independently endpoint-FK checked. Only fresh independent
release observations after return admit completion. Cancellation stops the goal
producer and awaits action cancellation; the shared station separately requires
correlated arm/EV3 STOP. Forced/unresolved producer shutdown cannot count as a
verified STOP. Hardware and novice-class acceptance remain unqualified.
