# Metadata-only derived camera source pins

`camera_capture_sources` prepares explicit source pins for optional read-only
marker/object perception without requiring a board observation, robot model,
owner state, base/world registration, or fit eligibility. It does not change the
existing `registration_capture_sources` command or its physical registration
gates. The distinct `so101-camera-source-v1` schema is rejected by
`RegistrationMonitor`; it cannot qualify a physical fit or recovery operation.

After installing the perception package, run explicitly with a new output path:

```bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 ros2 run so101_station_perception \
  camera_capture_sources --output /absolute/evidence/camera-source.json --timeout 4
```

The wrapper equivalent is `ros2/conda/so101-conda.sh run ros2 run ...` from the
integration checkout. Domains must normalize to integers 1..232; domain0,
including `00`/`+0`, is rejected before ROS initialization. The expected camera
serial defaults to Rig01 `243322071783`; `--serial` can explicitly select another
expected serial. There is no SDK/device open or actuator access. Python callers
can construct `CameraSourceCapture(serial=...)` and call
`capture(timeout=4., output=...)`; this is a finite ROS metadata operation, not a
new continuously running daemon/service.

## What is independently observed

Only lineage and the two native CameraInfo streams are subscribed, plus the
normal TF listener. There are **no RGB, depth, aligned-image or cloud payload
subscriptions**. The graph must contain seven unique actual publisher endpoints:
native depth/RGB/both Info plus derived cloud/lineage/aligned. Their exact GIDs
must match the per-cohort lineage, including the received lineage/Info GIDs.
The helper independently calls the actual native camera's DeviceInfo service,
binding all four native GIDs before and after RPC, and checks the returned serial
against the explicit expected serial and the producer statement. It independently
checks exact-stamped native K/D/profile/frames and the color-from-depth optical
TF at the original depth stamp. Only the proven five-zero-coefficient
`plumb_bob` profiles are supported.

The helper observes two advancing original RGB/depth pairs asserted by distinct
lineage sequences. It does **not** independently observe their pixels, image
payload contents, cloud point values or geometric accuracy. Those remain trusted
local producer assertions; marker/object consumers later bind their actual
received RGB/aligned MessageInfo GIDs and exact stamps. Source hashes and graph
identity are provenance, not cryptographic security or physical qualification.

## Freshness, generation and publication

The collector retains at most four native Info references per stream and eight
64 KiB serialized lineage statements (at most512 KiB serialized data plus Python
object overhead and eight small fixed-profile Info references). No point/image buffers are retained. Duplicate Info cannot
renew a receipt. The latest lineage must have exact matching Info; older complete
statements are never substituted. Publisher, immutable implementation/core/
worker/compatibility/calibration or source-generation changes latch this capture
unavailable. A new explicit invocation is required; consumers never auto-repin.

Original image/Info stamps must be nonfuture and <=500 ms old; producer native
receipts and independently received native Info retain their original250 ms
leases. Producer DeviceInfo and the helper's independent request retain their
original1 s leases. Own RPC duration must be <=400 ms. Exact RGB/depth pairing
remains <=40 ms. All leases, graph identity and exact optical TF are rechecked
after access, after JSON serialization/staging, and after exclusive publication.
The timeout is finite `(0,4]` seconds, covering collection and normal publication;
it is not a claim that Python can interrupt a wedged native/filesystem syscall.

The artifact and `.evidence.json` sidecar contain full selected statement,
original receipt/status GID, actual native metadata/receipt GIDs, exact TF,
independent RPC response/request snapshot/clocks, and loaded helper/validator
source hashes. Neither contains invented owner/model/base/world or fit fields.
Complete JSON is staged privately, then linked exclusively; preexisting files
are never overwritten. On failure this call removes only its own published
links. The CLI can preserve a clearly distinct `.failed.json` diagnostic, never
a valid camera artifact. Consumers must reload **only after command success**;
the two paths are individually atomic, not an atomic multi-file transaction.

Then explicitly select the artifact for each opt-in perception consumer and call
its `~/reset_derived_source` service, or start the consumer with the artifact.
Default-off/direct paths remain unchanged. Root/operator owns live deployment;
this feature's tests use UUID-scoped synthetic DDS in domain177, actual native
DeviceInfo fake/TF and metadata only. They are not physical camera/GPU proof.
