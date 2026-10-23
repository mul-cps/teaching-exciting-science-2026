# Resident shared-device video (qualification candidate)

The native `resident_video` library uses the resident pipeline's **same** Vulkan
instance, physical device, logical device, and host queue lock. It does not ask
FFmpeg to create another Vulkan device. The optional Context negotiation occurs
before device creation; lack of video capability retains compute support and a
reason. Current source and native Context require advertised, queried, and enabled
`VK_KHR_video_maintenance1`, together with timeline semaphores and synchronization2,
using a stable feature chain. FFmpeg 9 development headers/libraries are required
for this candidate.

Normal callers construct `VideoSession(width,height,fps,mode,record,context)` and
surface `selection().backend` and `selection().reason`. Modes are `auto`,
`vulkan` (strict), and `cpu`. Saved evidence must match the current device UUID,
driver, enabled queues/extensions/features, library versions, source digest,
shader digest and requested H.264 configuration. An unavailable device or failed
current initialization cannot be overridden by a saved record. Auto then selects
CPU and records why; strict Vulkan fails. No qualified profile record is shipped.
`VideoEncoder` is the lower-level API used by the explicit qualification probe.

Vulkan encoding reads RGB8 directly from a **completed resident FrameLease**.
A separate Slang kernel converts it to BT.601 limited-range NV12 in one of three
preallocated device buffers. Copy commands transfer its two planes to FFmpeg's
multi-plane NV12 image, preserving AVVkFrame access/layout and timeline semaphore
ownership. There is no second RGB upload, CPU color conversion, or depth access
in this path. A source lease remains pinned until the conversion fence finishes.
Three reusable input images and nonblocking fence checks provide EAGAIN
backpressure; callers drain packets and retry or drop a preview frame.

CPU comparison performs RGB-to-NV12 conversion with libswscale and H.264 encoding
with libx264. Metric Z16 depth remains in its original lossless representation;
this video API does not encode, edit or discard it. Recording depth and its
calibration/timestamps remains the caller's separate responsibility.

The H.264 decoder uses the same borrowed Vulkan device in hardware mode. Packets
are limited to 2 MiB, decoded dimensions to 640x480, and application-held output
frames to three. Callers must unref output frames before reusing them. FFmpeg's
codec DPB, bitstream staging and internal execution resources are separate from
these three application slots; their allocations and retained reference pictures
still require qualification. There is **no claim** of zero internal FFmpeg
allocation or a three-image total decoder footprint.

## Build and qualification

Build the package with `SLANGC_EXECUTABLE` as for resident compute. Video SPIR-V is
compiled offline into its own `video-shaders` directory, independent of the
resident compute shader inventory. Deployment can use
`PRECOMPILED_VIDEO_SHADER_DIR` containing `rgb_to_nv12.spv`, its sha256 sidecar,
and `source.sha256`; CMake rejects mismatched source or binary digests.

After the mandatory separate performance review, run the explicit probe:

```sh
qualify_video vulkan /path/to/resident/shaders /path/to/video-profile.json
```

It generates 60 synthetic 640x480 RGB-D frames, encodes their RGB on the shared
device, independently decodes with the strict CPU decoder, hardware-decodes the
same packets, and compares canonical NV12 frame hashes and expected luma. The
JSON contains the exact profile identity, frame counts, hashes, luma measurements
and failure reason. A CPU probe uses `cpu` instead of `vulkan`; its result can
never qualify a hardware profile. This two-second synthetic check does not
replace sustained resource cleanup, engine-use or full-stack acceptance.

## VAAPI boundary

Standalone VAAPI encoder success is not shared-device interoperability evidence.
The runtime currently **refuses the VAAPI branch** because the asynchronous
Vulkan/DRM/VAAPI ownership transfer has not been implemented and qualified.
Auto reports that refusal and uses CPU when Vulkan is unqualified.

FFmpeg's Vulkan-to-VAAPI mapper exports a DRM_PRIME frame, maps it to a VAAPI
surface, retains the source frame, and closes exported FDs/imports completion on
unmap. Depending on the FFmpeg build, image tiling and SYNC_FD export support, the
export path can wait on Vulkan timeline semaphores with an infinite timeout.
A permanently mapped image is also externally owned; reusing it for conversion
requires a proven acquire/release and implicit/explicit synchronization protocol.
The current converter rejects external ownership rather than guessing it.
The next implementation must use the render node identified by Vulkan's DRM
properties, verify same physical device, gate modifier/FD/semaphore support,
bound map/unmap costs and prove cleanup and completion in both directions.

Primary references: installed `libavutil/hwcontext_vulkan.h` and
[FFmpeg Vulkan implementation](https://github.com/FFmpeg/FFmpeg/blob/master/libavutil/hwcontext_vulkan.c),
[VAAPI implementation](https://github.com/FFmpeg/FFmpeg/blob/master/libavutil/hwcontext_vaapi.c).
The installed headers, not moving master, define the compiled ABI.

## Current evidence

The 2026-10-23 Wave5 shared-device probe passes on AMD Radeon 8060S/RADV
Mesa 26.2.3: 60 frames encoded, 60 independently strict CPU decoded, and 60
hardware decoded. Canonical NV12 hashes match for every frame and expected luma
checks pass. The exact-profile loader admits the valid current AMD record in
both auto and strict Vulkan modes; a tampered profile key is rejected. Evidence:
`/tmp/resident-wave5-amd-video-20261023.json` and
`/tmp/resident-wave5-amd-policy-20261023.json`.

On Intel ADL-N/ANV Mesa 25.2.8, enabling maintenance1 now permits H.264 encoder
initialization, but the first encoded packet fails independent strict CPU decode
(`top block unavailable for requested intra mode -1`). There is no accepted Intel
qualification: one packet encoded, zero CPU or hardware frames decoded. Its
failed record is rejected, auto explicitly falls back to CPU with a reason, and
strict Vulkan refuses. Tampered profile keys are also rejected. Evidence:
`/tmp/resident-wave5-intel-video-20261023.json` and
`/tmp/resident-wave5-intel-policy-20261023.json`. Standalone codec evidence cannot
substitute for this shared-device qualification.

No qualified record is shipped. Admission remains bound to the exact requested
profile, device, driver, libraries, native source and shaders, including the
current enabled-feature identity. The separate 60-frame CPU probe and nine native
CPU CTests remain CPU evidence; they do not qualify GPU execution. The frame-slot
and internal-allocation limits described above are unchanged.

Camera, preview and station recording-service integration, and asynchronous
VAAPI interoperability, remain unimplemented. Metric depth remains the caller's
separate lossless responsibility. These bounded synthetic results do not qualify
a full station or a 10-minute soak. See the
[software checkpoint](software-checkpoint-20261023.md) for the wider acceptance
boundary and `/tmp/workshop-gpu-wave5-qualification-20261023.md` for the retained
Wave5 report.
