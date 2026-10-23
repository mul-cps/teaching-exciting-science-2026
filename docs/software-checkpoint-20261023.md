# Software checkpoint — 2026-10-23

This is a SOFTWARE CHECKPOINT, not a complete user release or physical-station acceptance. Main merge, release push and deployment remain held.

Verified isolated worktree: `.worktrees/beginner-vulkan-20261022`, branch `feat/beginner-vulkan-20261022`. Existing shared edits were preserved; the dirty active checkout was preserved. fwmini4 had only keyboard/mouse attached at its last verification on 2026-10-23.

## Software and teaching materials

- Canonical CPU lane: **3748 passed, 133 expected skips, 13 subtests passed** (56.43 s).
- Mandatory native lane: **347 passed**, two deprecation warnings (31.84 s).
- Additional calibration/recovery/taught native matrix: **86 passed**, two warnings (37.73 s). This is separate evidence; counts are not added to the mandatory lane.
- All three notebooks passed fresh-kernel execution without ROS. Optional routing variants passed. The live-unavailable wrapper failed explicitly with empty measurements and no DEMO fallback, as required.
- `python exercises/make_student_bundle.py --check` passed against current source bytes.
- Every one of the 116 entries in [materials-build.json](materials-build.json) matched a freshly computed SHA256 of its current file. Receipt SHA256: `dea1f30d956fe2019f9ba92873257030c5076385e58c0a7641963c6d87f36d5d`.
- Materials contain 24 slides (8 introduction, 16 practical) and 35 print pages. Previous full renders were reviewed. The actual 240-minute class and printer/projector setup remain unrehearsed.
- Loaded jaw/joint calibration identity is bound to physical stroke recording/replay and taught admission. Missing legacy physical bindings require reteaching. Native evidence uses synthetic devices, not physical acceptance.

## Bounded GPU evidence

- Eight-stage compute correctness passed on AMD and Intel, including exact retained memberships/depth cuts and capacity/convergence fault behavior.
- Intel isolated dense compact p95: **47.966678 ms**, below 50 ms for that subset. Recorded compact p95 regressed to **20.905493 ms**, versus current CPU **5.151445 ms**. These results do not establish full-station 20 Hz or 30% CPU reduction.
- AMD synthetic shared-device video: **60 encoded / 60 strict CPU decoded / 60 hardware decoded**; all canonical hashes and expected luma checks passed.
- Intel video failed strict independent CPU decoding at the first packet. Runtime admission correctly selected CPU fallback; strict Vulkan refused the failed record.
- AMD geometry preparation passed all **8 injected-source fixtures**; maximum coordinate error **2.7755575615628914e-17 m**, within the unchanged 1e-12 bound. Exact flags/counts/support ordering and lease/fault checks passed.
- Intel physically unsupported and disabled FP64 was explicitly rejected before geometry dispatch. Full geometry beyond preparation is **not implemented**.
- Raw alignment remains **RED**: fixture 2 had **160 AMD / 216 Intel** exact-Z16 pixel mismatches. No tolerance was relaxed; combined preparation did not pass. Strict camera qualification remains incomplete and is not enabled in production.

## Open acceptance gates

Full-station device/camera qualification, live frame age, video/VAAPI interoperability, geometry integration, GPU cloud/OctoMap integration, a 10-minute soak, three physical cycles, the actual class rehearsal, deployment and main merge remain OPEN. Synthetic/native success does not close these gates.

## Local evidence

These `/tmp` paths are local ephemeral evidence, not repository artifacts:

- `/tmp/workshop-cpu-checkpoint-final-20261023.log`
- `/tmp/workshop-native-post-calibration-20261023.log`
- `/tmp/workshop-notebooks-checkpoint-20261023.log`
- `/tmp/workshop-recording-calibration-binding-20261023.md`
- `/tmp/workshop-gpu-wave5-qualification-20261023.md`
- `/tmp/workshop-preparation-gpu-harness-20261023.md`
