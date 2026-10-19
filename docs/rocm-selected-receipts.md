# Original selected acquisition receipts

The ROCm diagnostic producer stores the first unique arrival time with each of
its four bounded records per native stream. Repeated headers can update stream
activity but cannot refresh a buffered acquisition receipt. Nearest RGB and
CameraInfo selection propagates the selected records' own receipts into lineage,
not the receipt of a later, unselected message.

Admission drops selected receipts outside [0,250ms] before the exposure ledger,
array views or worker offer. Header age500ms and pairing40ms remain unchanged.
The independent stream-activity watchdog still uses its own latest arrival.
No new image copy, hashing, filesystem access, thread or GPU work is introduced.

This closes a provenance gap; it has not been established as the cause of a
live stream-lost fault. CPU/native regressions are not live GPU qualification.
