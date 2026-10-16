# Offline CGAL mesh proof implementation plan

**Goal:** Evaluate whether unchanged frozen triangles admit conservative material-interior certificates, without enabling recovery.

**Architecture:** A resource-limited child process uses CGAL EPECK for topology, intersection, parity and exact distance. A Python CLI snapshots source bytes, preserves triangle coordinates, records hashes and returns unknown on failure. No production imports or provider changes.

**Constraints:** Own only tools/recovery_cgal/ and tests/test_recovery_cgal*.py. Isolated dependency overlay; existing ROS environment read-only. No hardware. One compiler job, <=8 GiB virtual memory, <=180 CPU seconds. Each proof process <=4 GiB and a caller wall deadline. No repairs, hull filling, component union or custom geometric predicates.

- [x] Write failing tests for closed cube, hollow cavity, intersecting shells, tangency, degenerate membrane, full-cell boundary uncertainty, exact double payload preservation, scale provenance, malformed/timeout/crash fail-closed.
- [x] Download exact conda-forge CGAL/MPFR packages to an external overlay and record package SHA, compiler, flags and dependency versions.
- [x] Implement a narrow EPECK helper: preserve exact triangles; validate nondegenerate valid closed mesh; prove no self intersections before volume queries; reject zero-volume components and invalid volume orientation; parity query entire mesh; require exact squared boundary distance strictly above a rational upper bound on the entire uncertain cell radius.
- [x] Implement strict JSON CLI/wrapper with external process deadlines and resource limits; source/payload/helper/build/URDF-scale hashes in results. Unknown never certifies.
- [x] Verify fixtures, benchmark moving jaw first. Notify parent before full13 run if jaw runtime is poor. Freeze13 input STLs and audit provenance externally; run each under bounded deadline.
- [ ] Run focused and full tests, save reviewable external artifact and documentation. Commit owned files only after review.

## Review focus

Nested shells use parity, never union. Equal/boundary distances fail. Degenerate/zero-volume membranes fail before AABB queries. Incomplete/crashed/expired proofs fail closed. Source coordinates and scale are never silently called physical calibration.
