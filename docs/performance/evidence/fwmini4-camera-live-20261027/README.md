# Retained fwmini4 live evidence

The original archive is retained unchanged:

- `fwmini4-camera-live-native-20261027.tar.gz`
- SHA-256 `325d00b3387ab8e4845363a22db69a26c6d32741a112195ef9935ee9f4539e33`

It contains all scalar-SDK CPU/Vulkan/auto observations, system-SDK CPU success,
strict refusal and explicit auto fallback, serial-type failure, build stdout and
runtime provenance. Thirty regular archive members match their supplied
extracted bytes; `raw-archive-inventory.json` records their sizes and hashes.
The archive was not regenerated when later evidence was added.

Supplemental files retain the runner, exact collector source, native v3 build
script, two SDK capture scripts/results, runtime provenance and final camera/owner
state. There are no ELFs, images, private calibration files or credentials.
Binary hashes are recorded provenance, not bundled executable content.

`inventory.json` lists each published evidence file except itself.
`selfcheck-result.json` records CPU arithmetic, contiguous color/depth source IDs,
archive-byte checks and final operational state. The cold gate is supported by
source invariant plus admitted status; no separate cold pixel dump exists.

Run the lightweight, artifact-only selfcheck from this checkout:

```bash
python docs/performance/evidence/fwmini4-camera-live-20261027/selfcheck.py
```

It checks retained hashes and recomputes process CPU fractions, per-output costs,
throughput and source-ID contiguity from archived JSON. It opens no camera,
creates no Vulkan context and launches no ROS process. The runner/build/capture
scripts are retained as evidence and are not executed by the selfcheck.

The [main note](../../fwmini4-camera-live-20261027.md) distinguishes the slower
scalar-SDK CPU baseline from the optimized system-SDK baseline and lists the
measurement limits. This evidence does not grant execution eligibility or full
geometry, control, soak, physical-station or high-resolution GPU qualification.
