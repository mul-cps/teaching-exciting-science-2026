# Live tag alignment implementation plan

**Goal:** Continuously match depth-backed physical arm tags to their named model links, estimate rigid model alignment from fixed-link markers, and show moving-finger residuals in ROS/RViz.

**Approved scope:** User approved continuous tag-to-model matching after adding a 7.5 mm tag to the moving finger. All visible tags currently share AprilTag 25h9 code ID 0. Instance identity must therefore use explicit seeded temporal association rather than pretend that the printed ID is unique.

**Architecture:** Reuse detections and admitted RGB-D in the existing shared marker pipeline. Publish compact timestamped 3D detections once; a separate lightweight ROS node performs association, timestamped TF lookup and a small paired-point rigid fit. Fixed-link markers determine camera-from-base alignment; the moving-jaw marker is excluded from that fit and reports its own residual. A separate aligned visual model shows the candidate without silently modifying control TF or qualification records.

## Constraints

- Keep ROS domain 76, localhost transport, hardware identities and freshness checks.
- Keep current powered arm hold; do not edit or restart its source or command motion.
- No new detector pass, full-cloud fit, GPU allocation or image copies per alignment update.
- Distinguish measured marker center, provisional surface-derived mount and independently validated mount.
- Reject ambiguous association, loss, stale or repeated timestamps, wrong model/camera identity, insufficient geometry, excessive fit residual or excessive transform jumps.
- No automatic reassignment of repeated-ID markers after loss; provide explicit reseeding API.
- Do not add or run unit tests; use syntax checks, recorded-data replay, bounded timing measurements and live ROS observations for this requested calibration work.

## Tasks

1. Add `workshop/digital_twin/tag_alignment.py`: strict binding validation, seeded temporal association, paired-point Kabsch fit with observability/residual/jump checks, position-only marker residuals.
2. Extend `tag_observations.py` to publish compact depth-backed detections after the existing source-admission check. Reuse existing white-module plane estimator; retain existing RViz observations.
3. Add `live_tag_alignment.py`, executable and launch integration: subscribe to compact observations and model identity, verify camera identity, resolve link transforms at exposure time, publish candidate alignment and measured/predicted RViz marker pairs, explicit reseed service and stale-state reporting.
4. Add an aligned visual robot topic/frames or equivalent corrected RobotModel display without a second owner for canonical robot TF. Keep correction provenance and qualification visible in status/API.
5. Derive provisional link-local marker points from the captured RGB-D/CAD data, inspect fixed-point geometry, then replay across available captures. Save image/model correspondences and independent residuals.
6. Separate review and performance refinement: benchmark compact update cost, inspect for image duplication/device synchronization, review ambiguity/stale/model-mismatch behavior. Do not claim physical calibration from surface-initialized anchors.
7. Build/install only the new perception overlay, launch explicitly on domain 76, verify current tag association, fresh fitted transform/residuals and RViz overlays, record sustained timing, commit/push reviewed source.

## Completion evidence

- New moving finger tag is associated by name, with measured 3D position and exposure stamp.
- At least three non-collinear fixed marker correspondences support a bounded fit, or status reports the exact missing geometry.
- RViz displays measured and predicted points with connecting residuals and a corrected visual model when a usable candidate exists.
- Loss/ambiguity invalidates current candidate; canonical robot control and calibration qualification remain intact until physical validation supports promotion.
