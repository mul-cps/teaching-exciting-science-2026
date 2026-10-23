# Native raw-depth alignment boundary

The standalone `resident_alignment` library implements a deliberately bounded
640 × 480, zero-distortion RGB-D alignment candidate. It is **not connected to
CameraComponent or admitted for physical perception**. Its GPU qualification
must follow the separate performance review. The existing resident camera still
uses the SDK alignment path; strict raw camera admission remains closed.

`AlignmentCalibration` carries the two SDK intrinsics, column-major depth-to-color
extrinsics, original native Z16 depth units, calibration generation, and an
explicit SDK rasterization convention. An unspecified convention, unsupported
profile or nonzero coefficient is rejected. Zero coefficients are accepted only
for none and Brown-Conrady family models; fisheye and nonzero distortion remain
unsupported. No private camera calibration is stored in this checkout.

The numerical reference distinguishes SDK scalar and SSSE3 implementations:

- Scalar uses both half-pixel corners, left-associated transform additions,
  truncation of projected coordinates plus 0.5, and rejects a rectangle if its
  first or last corner falls outside the destination.
- SSSE3 uses precomputed corner rays, right-associated transform additions,
  nearest-even conversion after adding 0.5, and clips each destination pixel.
  Its second corner is conditional on the two cameras' pixels-per-angle ratios.
- Overlapping pixels retain the minimum original, nonzero **native Z16** value.
  Transformation changes the destination pixel, not the stored depth units.
  Empty pixels remain zero. The subsequent color-grid deprojection retains this
  aligned-depth convention and outputs metric XYZ with invalid points zero.

These differences come from librealsense `src/proc/align.cpp` and
`src/proc/sse/sse-align.cpp`, not from a tolerance adjustment. The local source
inspection used commit `9a0dd70db1a2c180b69c6c257cd2ee6120505499`. The installed
2.58.3 library's actual `rs2::align` software-device oracle agrees exactly with
**scalar** on the eight procedural cases. A uniform image alone is ambiguous;
random depths, translations, unequal focal lengths, empty frames and edge pixels
are included. This result does not establish the implementation in another SDK
build. A deployment must repeat the oracle with the exact loaded SDK binary.

The Slang shader explicitly implements ties-to-even rather than assuming the
meaning of HLSL `round`. It compiles offline with `-fp-mode precise`, and preserves
association. GPU division precision still requires exact rasterization tests on
each device. The acceptance check is zero differing Z16 pixels and at most
1 micrometre XYZ error; no existing geometry gate is relaxed.

## Ownership and transfers

The library takes the resident `shared_ptr<Context>`; it creates no Vulkan device
or queue. Its three private intermediate slots are retained by copyable leases.
It skips in-flight or retained slots, and returns an empty lease on saturation.
A copied lease keeps the buffers, fence, pipeline resources and shared device
alive. Submission rejects foreign leases, repeated submission and mismatched
calibration generations/dimensions. Submitted upload access is immutable.

The future camera adapter must retain the same three camera frame identities
through these intermediate pools, use latest-frame admission, and carry its
synchronization checks and original metadata. This library preserves supplied
metadata; it does not independently certify camera synchronization.

Corner rays are computed once per immutable calibration. Each frame uploads one
native Z16 plane. The output is a device buffer of tightly packed native Z16 and
a separate float4 XYZ device buffer; RGB is retained by the future upstream frame
lease. Future resident ingestion must consume these buffers on the shared queue
with a device copy or descriptor binding, rather than reading aligned depth back
and uploading it again. That adapter is intentionally pending.

Normal processing reads back only a 4-byte failure status. A projected footprint
larger than 4096 destination pixels faults the entire frame, never truncates a
successfully published frame. Full Z16/XYZ readback exists only when diagnostics
are explicitly enabled. Buffers, command buffers, descriptors and fences are
allocated at initialization. Submission does not wait for any device; completion
polls its own frame fence. Teardown waits only for this library's submitted
fences. The shared command-pool and queue locks match the resident context.

## Reproduction without devices

Configure the native package with an offline Slang compiler and a separate build
prefix; build target `qualify_alignment`. The CPU CTest
`resident_alignment_sdk_oracle` runs `qualify_alignment --cpu`. It creates only
librealsense software sensors; it never opens USB, starts services or actuates
hardware. `qualify_alignment --vulkan` is an explicit opt-in qualification tool,
not part of ordinary CPU CTest, and must wait for the separate performance pass.
Offline shader copies require matching source and per-SPIR-V SHA-256 files.

The historical lighting-blocker capture contains native and aligned depth from
exposures about 66.7 ms apart. Its saved aligned plane is therefore unsuitable as
a same-frame numerical oracle. Replay its native plane through the pinned SDK
if using that private fixture; do not compare those two historical planes as if
they were synchronized.

Camera integration, nonzero distortion, raw-profile GPU acceptance, sustained
frame-rate/CPU targets and full geometry/grasp validation remain separate gates.
