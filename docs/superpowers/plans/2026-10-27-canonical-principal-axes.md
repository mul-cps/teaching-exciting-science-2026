# Canonical directions for numerically tied principal values

> **For agentic workers:** Use superpowers:subagent-driven-development. Keep the complete workshop/station goal active; this fixes a prerequisite of complete geometry qualification.

**Goal:** Make unresolved PCA directions consistent across the retained CPU implementations, without changing geometry acceptance thresholds or substituting another estimator.

**Architecture:** Keep the existing mean/median centering, SVD and covariance/eigh operations. Canonicalize only numerically tied eigenspaces using the coordinate frame of each caller. Preserve historical v1 references and introduce independently reviewed v2 references for this explicitly changed tie policy.

**Tech stack:** Python, NumPy, OpenCV, pytest; the existing local and fwmini4 CPU environments. No GPU or physical actuation is required for this change.

**Spec:** [Native geometry integration](2026-10-23-native-geometry-integration.md); [measured CPU portability failure](../../performance/geometry-cpu-budget-20261027.md). This document records the supervisor's implementation decisions within the approved workshop plan.

## Root cause and policy

Bit-identical matrices on both hosts produce different valid bases when leading singular values agree to about machine precision. The existing report exposes those arbitrary directions through grasp axes and percentile extents. Changing BLAS dispatch, retrying another estimator or relaxing report tolerances would not define the missing direction policy.

The helper accepts a descending spectrum and orthonormal **rows** in dimension 2 or 3. A cluster's complete spread must be at most `8 * eps64 * max(sample_count, dimension) * max(abs(spectrum))`. Do not chain adjacent gaps. This is an explicit numerical tie policy, not a certified universal LAPACK error bound. SVD uses singular-value units; covariance/eigh uses eigenvalue units without square-root conversion. Keep singleton rows byte-for-byte.

For a cluster covering every dimension, return coordinate identity. For a tied pair in 3D, form its projector as `I - outer(untied_row, untied_row)`, so the choice does not depend on arbitrary rotations of the tied rows. Select the remaining projected coordinate with largest squared norm; norms within `64 * eps64` of that maximum tie in X, Y, Z order. Orthogonalize in fixed dimension, normalize and keep a positive dot product with the selected coordinate. Return a tied-row mask so callers do not undo these signs. Keep existing singleton sign conventions and the visible-geometry right-handed cross product.

After the existing exact `opening <= maxopening_m` candidate filter, minimum-opening selection retains the first candidate when widths differ by at most `32 * eps64 * max(abs(widths))`; otherwise retain strict smaller-opening selection. This applies to the final ranking, does not equate materially different widths, and never admits an out-of-range opening. It supplies a separate explicit roundoff tie rule without adding public report metadata.

The policy has declared numerical boundaries and does not promise universal cross-platform bit identity. Existing full-report `1e-9`, ray/preparation `1e-12`, exact membership/order/flags and every physical admission threshold remain unchanged. No support-plane SVD, RANSAC, Huber fitting or calibration changes are in scope.

## Task 1: Implement the bounded tie policy

**Files:**

- Create `workshop/digital_twin/principal_axes.py`.
- Modify the SVD direction use in `workshop/digital_twin/grasp_geometry.py:98-130`.
- Modify the SVD direction use in `ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py:151-251`.
- Modify only `_component_geometry`, `_visible_component_geometry` and final grasp ranking in `ros2/src/so101_station_objects/so101_station_objects/objects.py`.
- Create `tests/test_canonical_principal_axes.py`.

**Interface:** `canonical_principal_axes(spectrum, axes, *, sample_count) -> (axes_rows, tied_mask)`. Inputs are finite, descending, dimension 2/3 with an orthonormal row basis and positive integer sample count. Use existing caller coordinate frames: camera coordinates for validation/footprints, the existing 2D tangent coordinates for support boxes, board coordinates for visible covariance geometry. No SVD/eigh call or per-point loop belongs in the helper.

- [x] Write and observe failing analytic tests for rotated/reflected/permuted tied bases, full-space identity, singletons, nullspaces, full-spread non-chaining, immediate cluster-boundary neighbors and projected-anchor ties.
- [x] Implement the helper and the four caller integrations, preserving each solver, centering and singleton arithmetic. Skip old sign adjustment only for tied directions.
- [x] Test complete public outputs under equivalent tied bases, including grasp selection, visibly unequal widths despite equal second moments, exact max-opening boundaries and unchanged separated-spectrum outputs. Expected axes/projectors must be analytic, not generated by the new helper.
- [x] Run the new tests and existing focused geometry/validation/tracking tests. Do not overwrite historical fixtures to make these checks pass.

## Task 2: Versioned references and cross-host proof

**Files:** `tools/native_geometry_oracle.py`, `tools/native_geometry_matrix.py`, `tools/qualify_native_preparation.py` where needed; `tests/test_native_geometry_oracle.py`, `tests/test_native_geometry_matrix.py`, `tests/test_observed_children.py`; new `procedural-v2.json`, `matrix-v2.json` and, if its current-source assertion requires it, `observed-children-v2.json` under `tests/fixtures/native_geometry/`.

**Interface:** New references identify the canonical tie policy and pin the new helper along with the existing geometry sources. Keep read-only v1 loading and every v1 payload byte. Active comparisons use explicitly versioned current references; historical checks use their immutable historical source commit, never changed live imports.

- [x] Audit every v1/v2 complete-report difference. Only the declared tied-direction/roundoff-ranking changes are allowed; centering, memberships, counts, reasons, flags and gate decisions must remain equal. Stop for an unexplained difference rather than broaden the allowlist.
- [x] Generate new references from the reviewed retained CPU source, separately verify analytic tie expectations, and retain the unchanged twelve matrix inputs/order. Do not use candidate GPU output as an oracle.
- [x] Replay the same v2 references locally and on fwmini4's private pinned environment. Require all twelve complete reports to pass unchanged `1e-9`/`1e-12` and exact discrete checks. Keep the historical target 10/12 result intact.

## Task 3: Resource review and verified checkpoint

**Files:** New bounded performance/static test and benchmark under `tests/` and `tools/performance/`; a concise dated readout under `docs/performance/`.

- [x] A separate worker reviews the data-path diff, measures actual CPU cost for distinct/tied 2D and 3D inputs and frozen complete reports, and adds static guards against new solvers, frame-sized allocation, per-point loops, host/GPU synchronization or added readbacks. Loops are bounded by dimension at most three; this does not authorize another GPU execution.
- [x] Independently review the final code, analytic assertions and allowed report changes; fix concrete findings without weakening existing safety checks.
- [ ] Run the canonical pinned CPU suite after source/reference freeze. Verify shared main is unchanged, commit/push the coherent fix and evidence, and independently verify remote SHA.

Complete resident report integration, deployment, full-stack qualification, physical commissioning/cycles and workshop release remain required after this checkpoint.

## Broad regression migration

The first full CPU run completed with 4606 passed, 133 skipped, 48 subtests and six failures. The failures compare current results with historical locality/CPU-reuse reports or the accepted timing algorithm fingerprint. Their additional complete-report differences must be audited against the same explicit tie policy before moving the regression epoch; do not loosen any comparison. Keep the historical revisions and prior AST fingerprints documented. The reviewed production source checkpoint supplies an immutable Git anchor for the new current baseline; publish the branch only after the migrated full suite passes.
