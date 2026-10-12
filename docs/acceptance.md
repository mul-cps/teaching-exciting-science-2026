# Validation register: never confuse testable with tested

**Status as authored:** CPU simulator and vision synthetic images tested in software. All physical integration is **NOT DONE**.

| Test | Evidence required | Status |
|---|---|---|
| Pure Python 5-cycle simulator | Expected left=2, right=2, reject=1 | PASSED (local) |
| Unit tests | `pytest -q`: 9 passing | PASSED (local) |
| CPU frame classification | Synthetic red/blue/grey frames | PASSED (local) |
| Linux USB webcam | Saved real test image at 640x480 | TODO |
| GoPro HERO10 over USB | Sustained 30-min stream and transport details | TODO |
| EV3 over chosen firmware | Correct low-speed direction, braking, disconnect | TODO |
| SO-ARM101 leader/follower | Calibration and safe teleoperation | TODO |
| Pick/replay motions | Five collision-free empty/repeated cycles | TODO |
| Belt-to-pick registration | Ten objects within gripper tolerance | TODO |
| Physical sorting | 10/10 red and blue blocks, reject unknown, safe failure | TODO |
| Classroom rehearsal | Six participants rotate without software issues | TODO |

## Missing features deliberately NOT faked

1. Automated `SO101` pick/place trajectories: needs physical pose capture, speed-limited interpolation and clearance testing. No stub silently moves the arm.
2. Integrated EV3 motion control: choose and validate physical firmware, then implement *only* bounded moves with a stop guarantee.
3. HERO10 -> Python frame source: official SDK demo proves candidate path, but actual transport exposure requires live device validation.
4. End-to-end physical sort: do **not** assert it works until all prior items pass.
5. Optional Jetson and neural vision: not necessary, not implemented.

## Hardware acceptance log template

Date: ______ Operator: ______ OS: ______ Python: ______ Camera firmware: ______ EV3 OS: ______ LeRobot version: ______

Object size/shape: ______ Camera exposure/WB: ______ ROI: ______ Belt motor port: ______ Robot ports: ______

Camera reconnect test: ______ Stop on failure test: ______ Safe clearance: ______ Last successful five-cycle test: ______
