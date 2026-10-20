# Historical completed-adoption acknowledgement

A LOCAL adoption transaction can finish successfully while retaining its crash-recovery marker. A committed marker alone does not establish that the final admission gate passed. Task A closes only this historical metadata transaction; it does not establish current camera, image, analyzer, robot, or sorting readiness.

## Explicit startup migration

The supervisor accepts four read-only strings, all empty or all provided:

- `confirmed_adoption_acceptance_artifact`
- `confirmed_adoption_acceptance_sha256`
- `confirmed_adoption_completed_artifact`
- `confirmed_adoption_completed_sha256`

Copy the exact trusted ROOT CLI wrapper receipts **beneath the existing journal directory**, each at most 128 KiB. Acceptance must name `perception-supervisor-adopt-completed-job`; completion must name `perception-supervisor-status`. Both fixed services, successful CLI/RPC flags, parsed/raw message equality, exact adoption/job/artifact/generation/pin/epochs and historical false authority flags are checked. Arbitrary external artifacts and other jobs cannot be adopted.

Migration runs under the existing exclusive journal lock before unresolved-marker reconstruction and adopted-artifact restoration. It never writes the journal. Its candidate-descendant check permits only paired nondecreasing clocks with the existing two-second clock-continuity sanity bound and exact aged-prefix removal from the existing quota. Unknown jobs, latches, new attempts, handled events, epochs, baseline/boot changes or attributable artifact changes reject.

The active marker is archived byte-for-byte as `late-local-adoption-committed-H.json`, followed by a durable `late-local-adoption-ack-H.json`, where H hashes the original marker bytes. Exclusive/idempotent writes, fsync/readback, and marker/journal compare-before-retirement protect the transition. The active marker must be retired before policy activity. A valid ACK left alongside the active marker finishes retirement on restart. Conflict or ambiguous IO fails closed, retaining the journal. Once no active marker exists, old archived ACKs do not constrain later legitimate accounting. These are soft OS/FS operations, not hard preemptible deadlines.

For a successful adoption completed by the current resident supervisor, existing explicit `enable_supervision` performs historical ACK **before** checking live source health. It then executes the existing enable gates. A historical ACK therefore may succeed while enable fails because live source is unavailable; no readiness permission follows from the ACK. ACK failure blocks policy activity and journal writes while the unresolved marker remains.

## Launch wiring

`perception.launch.py` now actually includes the supervisor leaf only when `perception_supervisor_enabled:=true` (default false). `station_live.launch.py` forwards this flag and `perception_supervisor_`-prefixed leaf inputs: expected source cohort, expected serial, journal directory, recovery artifact directory, and the four confirmation strings. The leaf preserves string types, including leading-zero serials and YAML-looking paths/hashes. The launch flag makes the node available; its policy still starts DISABLED. Existing explicit enable and fresh five-second SOURCE-binding qualification remain separate.

The coordinator must already be separately launched with its existing reviewed configuration. Task A adds no coordinator launch, producer reset, native-camera restart, arm access, repair attempt, or automatic sorting promotion. Private unarmed repair is a separate default-off Task B and is not implemented here.
