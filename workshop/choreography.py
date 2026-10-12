"""Instructor choreography: dry-run by default, explicit --run for hardware.

Seventh-order curves join with continuous velocity, acceleration and jerk.
This joint-space demo does not implement collision detection or autonomous sorting.
"""
import argparse
from dataclasses import dataclass
import gc
import hashlib
import json
import math
from pathlib import Path
import queue
import signal
import threading
import time


JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Degrees/s and degrees/s²; gripper uses percent/s and percent/s².
SPEED = dict(zip(JOINTS, (10, 8, 8, 10, 15, 25)))
ACCELERATION = dict(zip(JOINTS, (20, 16, 16, 20, 30, 50)))
FLOW_SPEED = dict(zip(JOINTS, (14, 12, 12, 14, 25, 35)))
FLOW_ACCELERATION = dict(zip(JOINTS, (24, 24, 24, 28, 50, 70)))
FLOW_JERK = dict(zip(JOINTS, (100, 80, 80, 100, 180, 300)))
DISPLACEMENT = dict(zip(JOINTS, (20, 22, 25, 18, 35, 45)))


@dataclass
class Segment:
    name: str
    target: dict[str, float]
    duration: float
    hold: float = 0.0


def interpolate(start, end, fraction):
    u = max(0.0, min(1.0, fraction))
    blend = u**3 * (10 + u * (-15 + 6 * u))
    return {k: start[k] + (end[k] - start[k]) * blend for k in start}


def motion_duration(start, end):
    duration = 2.0
    for k in start:
        distance = abs(end[k] - start[k])
        duration = max(duration, 1.875 * distance / SPEED[k],
                       math.sqrt((10 / math.sqrt(3)) * distance / ACCELERATION[k]))
    return duration


def build_dance(start, body_wave=False):
    if set(start) != set(JOINTS) or not all(math.isfinite(v) for v in start.values()):
        raise ValueError("A finite starting position for all six joints is required")
    if not 0 <= start["gripper"] <= 100:
        raise ValueError("Gripper starting position must be in 0..100")
    # Offsets from the measured rest pose; no blind absolute home movement.
    # pan, lift, elbow, wrist flex, wrist roll, gripper opening
    poses = [
        ("rise", (0, 14, -16, -10, 0, 15), 0.5),
        ("turn left", (20, 16, -18, -12, 0, 15), 0),
        ("wave 1 out", (18, 18, -20, -14, 35, 45), 0),
        ("wave 1 in", (14, 20, -22, -16, -25, 15), 0),
        ("wave 2 out", (8, 22, -25, -18, 35, 45), 0),
        ("wave 2 in", (0, 20, -23, -16, -25, 15), 0),
        ("turn right", (-20, 18, -20, -14, 0, 30), 0),
        ("bow", (-10, 6, -8, -8, 0, 10), 0),
        ("final salute", (0, 22, -25, -18, 25, 45), 1),
        ("finish", (0, 0, 0, 0, 0, 0), 0.5),
    ]
    if body_wave:
        poses[3] = ("wave 1 in", (14, 10, -10, -6, -25, 15), 0)
        poses[5] = ("wave 2 in", (0, 12, -12, -8, -25, 15), 0)
    result, previous = [], start
    for name, offsets, hold in poses:
        target = {k: start[k] + delta for k, delta in zip(JOINTS, offsets)}
        target["gripper"] = min(100, target["gripper"])
        result.append(Segment(name, target, motion_duration(previous, target), hold))
        previous = target
    return result


def smooth_state(start, end, v0, v1, duration, fraction):
    """Septic Hermite position plus analytic velocity, acceleration and jerk."""
    u = max(0.0, min(1.0, fraction))
    p, velocity, acceleration, jerk = {}, {}, {}, {}
    for k in start:
        a1 = v0[k] * duration
        d = end[k] - start[k] - a1
        e = (v1[k] - v0[k]) * duration
        c4, c5, c6, c7 = 35*d-15*e, -84*d+39*e, 70*d-34*e, -20*d+10*e
        p[k] = start[k] + a1*u + u**4 * (c4 + u*(c5 + u*(c6 + u*c7)))
        velocity[k] = (a1 + u**3*(4*c4 + u*(5*c5 + u*(6*c6 + u*7*c7)))) / duration
        acceleration[k] = u**2*(12*c4 + u*(20*c5 + u*(30*c6 + u*42*c7))) / duration**2
        jerk[k] = u*(24*c4 + u*(60*c5 + u*(120*c6 + u*210*c7))) / duration**3
    return p, velocity, acceleration, jerk


def curve_peaks(plan):
    previous = plan["start"]
    peaks = [{k: 0.0 for k in JOINTS} for _ in range(3)]
    for stage in plan["stages"]:
        for i in range(201):
            p, *derivatives = smooth_state(previous, stage["target"], stage["v0"],
                                           stage["v1"], stage["move"], i / 200)
            for k in JOINTS:
                if not min(previous[k], stage["target"][k])-1e-6 <= p[k] <= max(previous[k], stage["target"][k])+1e-6:
                    raise ValueError("Spline overshoot outside adjacent poses")
                for peak, derivative in zip(peaks, derivatives):
                    peak[k] = max(peak[k], abs(derivative[k]))
        previous = stage["target"]
    return peaks


def make_plan(start, hz, body_wave=False):
    segments = build_dance(start, body_wave)
    positions = [start] + [s.target for s in segments]
    durations = []
    for a, b in zip(positions, positions[1:]):
        durations.append(max(1.5, max(2.1875*abs(b[k]-a[k])/FLOW_SPEED[k] for k in JOINTS),
                             max(math.sqrt((16.8/math.sqrt(5))*abs(b[k]-a[k])/FLOW_ACCELERATION[k]) for k in JOINTS)))
    velocities = [{k: 0.0 for k in JOINTS} for _ in positions]
    for i in range(1, len(positions)-1):
        if segments[i-1].hold:
            continue
        for k in JOINTS:
            left = (positions[i][k]-positions[i-1][k]) / durations[i-1]
            right = (positions[i+1][k]-positions[i][k]) / durations[i]
            if left*right > 0:
                # Conservative monotone knot velocity; zero at direction reversals.
                velocities[i][k] = math.copysign(min(abs(left), abs(right)), left)
    stages, begin = [], 0.0
    for i, segment in enumerate(segments):
        stages.append(dict(name=segment.name, begin=begin, move=durations[i], hold=segment.hold,
                           target=segment.target, v0=velocities[i], v1=velocities[i+1]))
        begin += durations[i] + segment.hold
    plan = dict(schema=2, interpolation="septic-flow", hz=hz, units="degrees; gripper percent",
                choreography="body-wave" if body_wave else "flow",
                start=start, stages=stages, duration=begin, speed_limits=FLOW_SPEED,
                acceleration_limits=FLOW_ACCELERATION, jerk_limits=FLOW_JERK)
    peaks = curve_peaks(plan)
    # Scale all motion time and knot velocities together; holds remain deliberate.
    scale = max(1.0, max(peaks[0][k]/FLOW_SPEED[k] for k in JOINTS),
                max(math.sqrt(peaks[1][k]/FLOW_ACCELERATION[k]) for k in JOINTS),
                max((peaks[2][k]/FLOW_JERK[k])**(1/3) for k in JOINTS)) * 1.01
    begin = 0.0
    for stage in stages:
        stage["begin"] = begin
        stage["move"] *= scale
        # These dictionaries share knots: replace rather than mutate in place.
        for key in ("v0", "v1"):
            stage[key] = {k: v/scale for k, v in stage[key].items()}
        begin += stage["move"] + stage["hold"]
    plan["duration"] = begin
    plan["profile_peaks"] = dict(zip(("velocity", "acceleration", "jerk"), curve_peaks(plan)))
    return plan


def plan_at(plan, elapsed):
    previous = plan["start"]
    for stage in plan["stages"]:
        if elapsed <= stage["begin"] + stage["move"] + stage["hold"]:
            if plan["interpolation"] == "septic-flow":
                return smooth_state(previous, stage["target"], stage["v0"], stage["v1"],
                                    stage["move"], (elapsed-stage["begin"])/stage["move"])[0], stage["name"]
            return interpolate(previous, stage["target"],
                               (elapsed - stage["begin"]) / stage["move"]), stage["name"]
        previous = stage["target"]
    return previous, plan["stages"][-1]["name"]


def profile_at(plan, elapsed):
    previous = plan["start"]
    for stage in plan["stages"]:
        if elapsed <= stage["begin"] + stage["move"] + stage["hold"]:
            u = (elapsed-stage["begin"]) / stage["move"]
            if plan["interpolation"] == "septic-flow":
                p, v, a, j = smooth_state(previous, stage["target"], stage["v0"], stage["v1"], stage["move"], u)
                if u > 1:
                    v = a = j = {k: 0.0 for k in JOINTS}
                return dict(t=elapsed, phase=stage["name"], position=p,
                            velocity=v, acceleration=a, jerk=j)
            return dict(t=elapsed, phase=stage["name"], position=plan_at(plan, elapsed)[0])
        previous = stage["target"]
    zero = {k: 0.0 for k in JOINTS}
    return dict(t=elapsed, phase=plan["stages"][-1]["name"], position=previous,
                velocity=zero, acceleration=zero, jerk=zero)


def validate_replay(plan, current, arm_id, calibration_sha256):
    if (plan.get("schema"), plan.get("interpolation")) not in ((1, "quintic-rest-to-rest"), (2, "septic-flow")):
        raise ValueError("Unsupported trajectory format")
    if plan.get("arm_id") != arm_id or plan.get("calibration_sha256") != calibration_sha256:
        raise ValueError("Replay arm ID or calibration differs")
    if set(plan["start"]) != set(JOINTS):
        raise ValueError("Replay must contain all six joints")
    if not all(math.isfinite(v) for v in plan["start"].values()):
        raise ValueError("Invalid replay starting pose")
    if not math.isfinite(plan["duration"]) or plan["duration"] <= 0:
        raise ValueError("Invalid replay timing")
    if max(abs(current[k] - plan["start"][k]) for k in JOINTS) > 3:
        raise ValueError("Replay starting pose differs by more than 3 degrees/percent; no motion started")
    previous, expected_begin = plan["start"], 0.0
    for stage in plan["stages"]:
        if not all(math.isfinite(stage[k]) for k in ("begin", "move", "hold")):
            raise ValueError("Invalid replay timing")
        if set(stage["target"]) != set(JOINTS) or not all(math.isfinite(v) for v in stage["target"].values()):
            raise ValueError("Invalid replay joint positions")
        if not 0 <= stage["target"]["gripper"] <= 100:
            raise ValueError("Invalid gripper position")
        if any(abs(stage["target"][k] - plan["start"][k]) > DISPLACEMENT[k]+1e-6 for k in JOINTS):
            raise ValueError("Replay displacement exceeds the choreography workspace")
        if abs(stage["begin"] - expected_begin) > 1e-6 or stage["hold"] < 0:
            raise ValueError("Invalid replay timing")
        if stage["move"] <= 0:
            raise ValueError("Invalid replay motion duration")
        if plan["schema"] == 1 and stage["move"] + 1e-6 < motion_duration(previous, stage["target"]):
            raise ValueError("Replay exceeds speed or acceleration limits")
        expected_begin += stage["move"] + stage["hold"]
        previous = stage["target"]
    if not plan["stages"] or abs(expected_begin - plan["duration"]) > 1e-6:
        raise ValueError("Invalid replay duration")
    if previous != plan["start"]:
        raise ValueError("Replay must finish at its starting pose")
    if plan["schema"] == 2:
        previous_velocity = {k: 0.0 for k in JOINTS}
        for stage in plan["stages"]:
            if stage["v0"] != previous_velocity or set(stage["v1"]) != set(JOINTS):
                raise ValueError("Discontinuous replay knot velocities")
            if not all(math.isfinite(v) for velocities in (stage["v0"], stage["v1"]) for v in velocities.values()):
                raise ValueError("Invalid replay knot velocities")
            if stage["hold"] and any(stage["v1"].values()):
                raise ValueError("Hold requires zero velocity")
            previous_velocity = stage["v1"]
        if any(previous_velocity.values()):
            raise ValueError("Replay must finish at rest")
        for peak, limits in zip(curve_peaks(plan), (FLOW_SPEED, FLOW_ACCELERATION, FLOW_JERK)):
            if any(peak[k] > limits[k] + 1e-6 for k in JOINTS):
                raise ValueError("Replay exceeds velocity, acceleration or jerk limits")


class Recorder:
    """Own camera capture and buffered log/video IO outside the motor loop."""

    def __init__(self, camera, writer, output):
        self.camera, self.writer, self.output = camera, writer, output
        self.samples = queue.SimpleQueue()
        self.stopping = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.record, name="dance-recorder")
        self.thread.start()

    def record(self):
        import cv2
        phase, last_snapshot, last_flush = "starting", 0.0, 0.0
        try:
            with (self.output / "feedback.jsonl").open("w") as log:
                while not self.stopping.is_set():
                    ok, frame = self.camera.read()
                    if not ok:
                        raise RuntimeError("Webcam frame lost")
                    changed = False
                    while not self.samples.empty():
                        sample = self.samples.get()
                        changed = changed or sample["phase"] != phase
                        phase = sample["phase"]
                        log.write(json.dumps(sample) + "\n")
                    now = time.monotonic()
                    if now - last_flush >= 1:
                        log.flush()
                        last_flush = now
                    cv2.putText(frame, phase, (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 255, 0), 2)
                    self.writer.write(frame)
                    if changed or now - last_snapshot >= 1:
                        cv2.imwrite(str(self.output / "latest.jpg"), frame)
                        if changed:
                            cv2.imwrite(str(self.output / f"phase-{phase.replace(' ', '-')}.jpg"), frame)
                        last_snapshot = now
                while not self.samples.empty():
                    log.write(json.dumps(self.samples.get()) + "\n")
        except Exception as exc:
            self.error = exc
        finally:
            self.writer.release()
            self.camera.release()

    def stop(self):
        self.stopping.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Camera recorder did not stop")


def run_hardware(args):
    import cv2
    from lerobot.robots.so_follower.so_follower import SOFollower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.utils.robot_utils import precise_sleep

    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "trajectory.json").exists():
        raise ValueError("Output already contains a trajectory; choose a new directory")
    arm = SOFollower(SOFollowerRobotConfig(
        port=args.port, id=args.id, calibration_dir=args.calibration_dir))
    bus = arm.bus
    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 25)
    writer = recorder = None
    trace = []
    original_acceleration = None
    torque_verified = False
    gc_was_enabled = gc.isenabled()
    try:
        for _ in range(10):
            ok, frame = camera.read()
        if not ok:
            raise RuntimeError("Webcam unavailable; no movement started")
        cv2.imwrite(str(args.output / "before.jpg"), frame)
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(str(args.output / "dance.avi"),
                                 cv2.VideoWriter_fourcc(*"MJPG"), 25, (width, height))
        if not writer.isOpened():
            raise RuntimeError("Video recording unavailable; no movement started")
        bus.connect()
        if not bus.is_calibrated:
            raise RuntimeError("Saved calibration does not match the arm")
        start = bus.sync_read("Present_Position", num_retry=2)
        digest = hashlib.sha256((args.calibration_dir / f"{args.id}.json").read_bytes()).hexdigest()
        if args.replay:
            replay_bytes = args.replay.read_bytes()
            plan = json.loads(replay_bytes)
            validate_replay(plan, start, args.id, digest)
            plan["source_trajectory_sha256"] = hashlib.sha256(replay_bytes).hexdigest()
        else:
            plan = make_plan(start, args.hz, args.body_wave)
            plan.update(arm_id=args.id, calibration_sha256=digest)
        servo_acceleration = args.servo_acceleration or plan.get("servo_acceleration", 50)
        if not 1 <= servo_acceleration <= 150:
            raise ValueError("Invalid saved servo acceleration")
        plan["servo_acceleration"] = servo_acceleration
        plan["hz"] = args.hz
        # Complete replayable plan is durable BEFORE torque is enabled.
        (args.output / "trajectory.json").write_text(json.dumps(plan, indent=2))
        with (args.output / "planned-profiles.jsonl").open("w") as planned:
            for tick in range(math.ceil(plan["duration"] * args.hz) + 1):
                planned.write(json.dumps(profile_at(plan, tick / args.hz)) + "\n")
        original_acceleration = {k: bus.read("Acceleration", k, num_retry=2) for k in JOINTS}
        (args.output / "hardware-before.json").write_text(json.dumps(dict(acceleration=original_acceleration)))
        bus.sync_write("Acceleration", {k: servo_acceleration for k in JOINTS}, num_retry=2)
        if any(bus.read("Acceleration", k, num_retry=2) != servo_acceleration for k in JOINTS):
            raise RuntimeError("Servo acceleration readback failed")
        recorder = Recorder(camera, writer, args.output)
        # Avoid unpredictable cyclic-GC pauses in the deadline-driven phase.
        # Samples contain no reference cycles; reference counting remains active.
        gc.collect()
        gc.disable()
        bus.sync_write("Goal_Position", start, num_retry=2)
        # Engage together after seeding current goals, avoiding drift during sequential activation.
        bus.sync_write("Torque_Enable", {k: 1 for k in JOINTS}, num_retry=2)
        routine_start = time.monotonic()
        tick, phase, last_command = 0, None, routine_start
        while True:
            deadline = routine_start + tick / args.hz
            precise_sleep(max(0, deadline - time.monotonic()), spin_threshold=0.0003, sleep_margin=0.0003)
            now = time.monotonic()
            elapsed = now - routine_start
            if now - deadline > 0.05 or now - last_command > 0.05:
                raise RuntimeError("Control deadline missed by >50 ms; stopping")
            last_command = now
            if recorder.error:
                raise RuntimeError(f"Recording failed: {recorder.error}")
            goal, current_phase = plan_at(plan, elapsed)
            if current_phase != phase:
                phase = current_phase
                print(f"{phase} @ {elapsed:.2f}s", flush=True)
            bus.sync_write("Goal_Position", goal, num_retry=2)
            actual = bus.sync_read("Present_Position", num_retry=2)
            sample = dict(t=elapsed, phase=phase, goal=goal, actual=actual,
                          deadline_late_ms=(now - deadline) * 1000,
                          bus_ms=(time.monotonic() - now) * 1000)
            trace.append(sample)
            recorder.samples.put(sample)
            if sample["bus_ms"] > 50:
                raise RuntimeError("Motor bus transaction exceeded 50 ms; stopping")
            errors = {k: abs(actual[k] - goal[k]) for k in JOINTS}
            if max(errors.values()) > 10:
                raise RuntimeError(f"Tracking limit exceeded: {errors}")
            if elapsed >= plan["duration"]:
                break
            # Skip missed ticks rather than sending a burst of stale goals.
            tick = max(tick + 1, math.floor((time.monotonic() - routine_start) * args.hz) + 1)
        print("Completed; returned to starting target", flush=True)
    finally:
        try:
            if bus.is_connected:
                try:
                    bus.disable_torque(num_retry=2)
                    if any(bus.read("Torque_Enable", k, num_retry=2) != 0 for k in JOINTS):
                        raise RuntimeError("Torque-disable readback failed")
                    torque_verified = True
                    if original_acceleration:
                        bus.sync_write("Acceleration", original_acceleration, num_retry=2)
                        if any(bus.read("Acceleration", k, num_retry=2) != original_acceleration[k] for k in JOINTS):
                            raise RuntimeError("Servo acceleration restore failed")
                finally:
                    bus.disconnect(disable_torque=False)
        finally:
            try:
                if recorder:
                    recorder.stop()
                    if recorder.error:
                        raise RuntimeError(f"Recording failed: {recorder.error}")
                else:
                    if writer is not None:
                        writer.release()
                    camera.release()
            finally:
                (args.output / "trace.json").write_text(json.dumps(trace))
                if gc_was_enabled:
                    gc.enable()
        print("Connections closed; " + ("torque disabled and read back" if torque_verified
                                        else "motor bus was not opened"), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Explicitly actuate the physical follower")
    parser.add_argument("--port")
    parser.add_argument("--id", default="hs_rig01_follower")
    parser.add_argument("--calibration-dir", type=Path)
    parser.add_argument("--camera", default="/dev/video1")
    parser.add_argument("--hz", type=int, default=100)
    parser.add_argument("--body-wave", action="store_true", help="Candidate stronger body-wave variation; see validation notes")
    parser.add_argument("--servo-acceleration", type=int,
                        help="Temporary native servo acceleration register (1..150); restored on exit")
    parser.add_argument("--replay", type=Path, help="Replay a saved trajectory.json after checking start/calibration")
    parser.add_argument("--output", type=Path, default=Path("/tmp/cps-spline-dance"))
    args = parser.parse_args()
    if not 100 <= args.hz <= 200:
        parser.error("--hz must be between 100 and 200")
    if args.servo_acceleration is not None and not 1 <= args.servo_acceleration <= 150:
        parser.error("--servo-acceleration must be between 1 and 150")
    if args.replay and args.replay.resolve() == (args.output / "trajectory.json").resolve():
        parser.error("Replay output must differ from the source trajectory directory")
    if args.replay and args.body_wave:
        parser.error("--replay preserves the saved choreography; do not combine with --body-wave")
    if args.run:
        if not args.port or args.calibration_dir is None:
            parser.error("--run requires --port and --calibration-dir")
        def stop_on_signal(signum, frame):
            raise KeyboardInterrupt(f"Received signal {signum}")
        signal.signal(signal.SIGTERM, stop_on_signal)
        run_hardware(args)
    else:
        plan = json.loads(args.replay.read_text()) if args.replay else make_plan({k: 0.0 for k in JOINTS}, args.hz, args.body_wave)
        for stage in plan["stages"]:
            print(f"{stage['name']}: move {stage['move']:.2f}s, hold {stage['hold']:.1f}s; target {stage['target']}")
        print(f"Dry run only: total {plan['duration']:.1f}s at requested {args.hz} Hz")


if __name__ == "__main__":
    main()
