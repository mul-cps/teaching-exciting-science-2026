# Support-plane factor probe — 2026-10-27

**QR NOT ADMITTED.** Keep the unchanged CPU full-geometry report path. This is a frozen negative prerequisite result, not an implemented estimator, GPU qualification, or performance result. The [machine readout](support-plane-factor-probe-20261027.json) keeps `qualified:false`.

The CPU prototype compares retained mean-centered, weighted Nx3 SVD with centered Householder QR followed by a 3×3 R SVD. It uses fixed 256-lane-shaped reductions and actual compiled C++ scalar sqrt/div and LAPACK small SVD. It does not substitute covariance/EIGH or raw moments.

| Fixed-tree/C++ candidate check | Result |
|---|---:|
| Ordinary and targeted populations | 32/32 public reports pass; maximum error 2.22e-16 |
| Complete immutable retained reports | 12/12 pass at 1e-9; selected-coordinate error zero at the 1e-12 gate |
| Strict boundary witnesses | **151/170 public contracts pass; 19 fail** |
| Additional exact 3mm witness | **401 reference inliers versus 402 candidate inliers** |

The twelve complete reports use the same immutable matrix populations included in the 32 population checks; they are not twelve additional independent populations. Their lost original dense snapshot was restored into a fresh private directory with retained CPU `freeze_matrix`. The entire committed matrix payload, array hashes, types and order matched `assert_matrix_equal`, using only its existing runtime-version normalization. See [rehydration proof](evidence/support-plane-factor-probe-20261027/rehydration-proof.json) and [full-report proof](evidence/support-plane-factor-probe-20261027/full-matrix-report-proof.json).

The public failures are consequential despite tiny underlying rounding differences:

- Near 5°, the reference snaps to board z=0 while QR retains a measured plane; the public normal differs by 0.08715574274766051.
- At a 15mm offset plus two input ULPs, the reference's computed offset snaps and QR's does not.
- At upward-normal 0.85 minus one input ULP and at second singular value 1e-9, QR accepts an input that the retained path rejects.
- For the additional 3mm witness, residuals 0.0030000000000000027 versus 0.002999999999999999 change point401's membership and the final support count, with the same RANSAC winning iteration.

Eight other boundary cases differ only in the **internal raw normal sign at d=0**. The required upward orientation erases those signs; they are not counted as public failures. Original strict results remain archived, and [selected receipts](evidence/support-plane-factor-probe-20261027/negative-witnesses.json) explicitly separate this classification. No epsilon, threshold, failure reason, or backend was changed to make a boundary pass. The alternative retained-mean/NumPy-small-SVD variant also fails: 31/32 ordinary and 166/170 public boundary checks pass.

The source contract is [weighted fitting and rank rejection](../../workshop/digital_twin/grasp_geometry.py#L31), [120 strict-first-improvement trials, fixed winning subset, eight Huber refits and residual statistics](../../workshop/digital_twin/grasp_geometry.py#L42), [upward/snap rules](../../ros2/src/so101_station_objects/so101_station_objects/objects.py#L101), and [support sampling, cap and rejection ordering](../../ros2/src/so101_station_objects/so101_station_objects/objects.py#L118). Source, compiler, loaded-library and synthetic input hashes are in [provenance](evidence/support-plane-factor-probe-20261027/provenance.json) and the selected receipts.

The [feedback and memory accounting](evidence/support-plane-factor-probe-20261027/schedule-and-cost.json) is a **design estimate**, not a measured or approved layout: 48 scalar feedback/read boundaries and 10,300 bytes/frame; an example scratch bound is 304,576 bytes/slot, 913,728 for three slots. Up to 16,000 adaptive divisions and 16,000 square roots must remain GPU operations at 2000 points. Intel software binary64 currently lacks those operations; CPU weight/residual-vector feedback is excluded. Implementing them would not resolve the demonstrated QR branch failures.

Both independent fresh seed 17 schedules were actually generated and stream-hashed: unsorted cap choice and a separately reset 120-triple proposal stream. Packed-U16 cap lookup costs are 11,200,000 bytes for current grid maximum 4800 and 121,600,000 for a future 1080p grid 32400; proposal lookup adds 1,419,120 bytes. These future memory figures do not admit higher resolutions or establish performance.

Small synthetic NPZ witnesses and receipts are checked in. The complete frozen scripts/results/inputs remain in the private archive whose exact hash and compressed member inventory are [recorded here](evidence/support-plane-factor-probe-20261027/private-archive.json). Check this bundle with Python 3 and NumPy. The checker inspects frozen arrays; it does not run a fitter or GPU:

```sh
python3 docs/performance/evidence/support-plane-factor-probe-20261027/selfcheck.py
```

Add `--private-archive /tmp/support-plane-factor-prototype-raw-20261027.tar.gz` to verify every archived member. A future precision-ambiguity refusal needs a separate contract and proof; refusing a retained-success input is not current parity. No production architecture change follows from this probe.
