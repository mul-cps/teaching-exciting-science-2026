#!/usr/bin/env python3
"""Dispense and sort finite single-brick cycles through the resident ROS owner."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from workshop.station.ev3_live_client import EV3LiveClient
from workshop.station.locking import state_dir, station_lock
from workshop.station.ros_sorting import StableTargetSelector, run_sorting
from tools.ros_plan_current import ARM_JOINTS, _joint_state_map


def load_settings(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text())
    if data.get("schema") != "so101-ros-sorting-v1":
        raise ValueError("sorting configuration must use so101-ros-sorting-v1")
    if any(data.get(field) is not True for field in ("enabled", "commissioned", "automatic_start")):
        raise ValueError("sorting must be explicitly enabled, commissioned and admitted for automatic_start")
    bounds = data.get("pickup_bounds_m")
    if (not isinstance(bounds, list) or len(bounds) != 6
            or any(type(v) not in (float, int) or not math.isfinite(v) for v in bounds)
            or any(bounds[i] >= bounds[i+1] for i in (0, 2, 4))):
        raise ValueError("pickup_bounds_m must be ordered finite XYZ minimum/maximum pairs")
    if not isinstance(data.get("frame_id"), str) or not data["frame_id"]:
        raise ValueError("sorting frame_id is required")
    if not isinstance(data.get("destinations"), dict) or not data["destinations"]:
        raise ValueError("commissioned color destinations required")
    pose = data.get("clear_joint_pose_rad")
    if (not isinstance(pose, dict) or set(pose) != set(ARM_JOINTS)
            or any(type(v) not in (float, int) or not math.isfinite(v) for v in pose.values())):
        raise ValueError("commissioned five-joint clear_joint_pose_rad is required")
    for field in ("station_config", "grasp_execution_config"):
        value = data.get(field)
        if not isinstance(value, str) or not value:
            raise ValueError(field + " path is required")
        target = Path(value)
        data[field] = str((path.parent / target).resolve() if not target.is_absolute() else target)
        if not Path(data[field]).is_file():
            raise ValueError(field + " file is unavailable")
    return data


class ROSSortingDevices:
    """Adapt ROS observations/actions and one persistent EV3 connection.

    No serial port or camera SDK is opened here. Grasp execution is a separate
    ROS client process; the resident bridge remains the one actuator owner.
    """
    def __init__(self, settings, client, output, *, node, rclpy):
        from action_msgs.srv import CancelGoal
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener
        self.settings, self.ev3, self.output = settings, client, Path(output)
        self.node, self.rclpy = node, rclpy
        self.placement_status = None
        self.placement_received = 0.
        self.status = None
        self.status_received = 0.
        self.joints = None
        self.child = None
        self.serial = 0
        self.baseline_ids = set()
        self.dispensed_after = 0.
        self.session = client.session
        node.create_subscription(String, "/station/placement_goal_observation", self._placement, 1)
        node.create_subscription(String, "/so101/grasp_status", self._status, 1)
        node.create_subscription(JointState, "/follower/joint_states", self._joints,
                                 qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        self.readiness = node.create_client(Trigger, "/follower/sorting_ready")
        self.cancel_clients = [node.create_client(CancelGoal, name + "/_action/cancel_goal")
                               for name in (
                                   "/follower/arm_trajectory_controller/follow_joint_trajectory",
                                   "/follower/gripper_controller/gripper_cmd")]
        self._action_statuses = [{}, {}]
        for index, name in enumerate((
                "/follower/arm_trajectory_controller/follow_joint_trajectory",
                "/follower/gripper_controller/gripper_cmd")):
            node.create_subscription(GoalStatusArray, name + "/_action/status",
                                     lambda message, index=index: self._action_status(index, message), 10)

    def _placement(self, message):
        try:
            self.placement_status = json.loads(message.data)
            self.placement_received = time.monotonic()
        except (ValueError, TypeError):
            self.placement_status = None

    def _action_status(self, index, message):
        for item in message.status_list:
            self._action_statuses[index][bytes(item.goal_info.goal_id.uuid)] = item.status

    def _status(self, message):
        try:
            self.status = json.loads(message.data)
            self.status_received = time.monotonic()
        except (ValueError, TypeError):
            self.status = None

    def _joints(self, message):
        self.joints = message

    def _check(self, deadline, cancel):
        if cancel.is_set() or not self.rclpy.ok():
            raise InterruptedError("sorting cancelled or ROS stopped")
        if time.monotonic() >= deadline:
            raise TimeoutError("sorting operation timed out")

    def _spin(self, deadline, cancel):
        self._check(deadline, cancel)
        self.rclpy.spin_once(self.node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))

    def _fresh_status(self):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        status = self.status
        if (not isinstance(status, dict) or status.get("state") != "tracking"
                or status.get("frame_id") != self.settings["frame_id"]
                or time.monotonic() - self.status_received > .5):
            return None
        stamp = status.get("stamp")
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp <= .5:
            return None
        return status

    def _inside(self, position):
        bounds = self.settings["pickup_bounds_m"]
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError("invalid observed object position")
        return all(bounds[2*i] <= p <= bounds[2*i+1] for i, p in enumerate(position))

    def _require_placement_ready(self, colors=None):
        selections = self.settings.get("placement_selections", {})
        if not selections:
            return
        from workshop.digital_twin.placement_region import load_qualified_frame_registration
        from workshop.station.placement_execution import resolve_tool_destination
        execution_path = Path(self.settings["grasp_execution_config"])
        execution = json.loads(execution_path.read_text())
        registration_path = Path(execution["camera_arm_calibration"])
        if not registration_path.is_absolute():
            registration_path = execution_path.parent / registration_path
        registration = load_qualified_frame_registration(registration_path)
        for color, selection in selections.items():
            if colors is not None and color not in colors:
                continue
            resolve_tool_destination(self.placement_status, selection_id=selection,
                registration=registration, received_monotonic=self.placement_received,
                now_monotonic=time.monotonic(), now_ros_ns=self.node.get_clock().now().nanoseconds,
                goal_from_tip=execution.get("placement_goal_from_tip"),
                footprint_m=execution.get("placement_footprint_m"),
                tool_footprint_m=execution.get("placement_tool_footprint_m"),
                clearance_m=execution.get("placement_clearance_m"),
                registration_uncertainty_m=execution.get("placement_registration_uncertainty_m"))

    def require_ready(self, deadline, cancel):
        from std_srvs.srv import Trigger
        # Do not establish physical EV3 references from an unobserved angle.
        status = self.ev3.call("status")
        references = status.get("references", {})
        if (status.get("session") != self.session or status.get("busy") is not False
                or not references.get("sorter") or not references.get("feeder")):
            raise RuntimeError("EV3 requires this session's confirmed sorter and feeder references")
        station = json.loads(Path(self.settings["station_config"]).read_text())
        if any(station.get(field) is not True for field in ("enabled", "commissioned", "automatic_start")):
            raise RuntimeError("station configuration has not admitted automatic physical operation")
        execution = json.loads(Path(self.settings["grasp_execution_config"]).read_text())
        if execution.get("qualified") is not True:
            raise RuntimeError("qualified grasp execution configuration required")
        # Read-only query: recovery and measured homing are separate explicit
        # operations. Readiness must never move the arm as a side effect.
        self._check(deadline, cancel)
        discovery_deadline = min(deadline, time.monotonic() + 3.)
        while not self.readiness.wait_for_service(timeout_sec=.05):
            self._check(deadline, cancel)
            if time.monotonic() >= discovery_deadline:
                raise RuntimeError("resident sorting readiness service unavailable")
        self._check(deadline, cancel)
        future = self.readiness.call_async(Trigger.Request())
        while not future.done():
            self._spin(deadline, cancel)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("station is not ready: " + (response.message if response else "no response"))
        placement_deadline = min(deadline, time.monotonic()+1.)
        while self.settings.get("placement_selections") and self.placement_status is None:
            self._spin(placement_deadline, cancel)
        self._require_placement_ready()

    def wait_clear(self, deadline, cancel):
        from rclpy.time import Time
        from tf2_ros import TransformException
        clear_stamps = set()
        while True:
            self._spin(deadline, cancel)
            status = self._fresh_status()
            if status is None:
                clear_stamps.clear()
                continue
            try:
                now = self.node.get_clock().now().nanoseconds * 1e-9
                measured = _joint_state_map(self.joints, now)
                if any(abs(measured[j] - self.settings["clear_joint_pose_rad"][j]) > math.radians(2.)
                       for j in ARM_JOINTS):
                    clear_stamps.clear()
                    continue
            except (ValueError, TypeError, AttributeError):
                clear_stamps.clear()
                continue
            if any(self._inside(obj["position_world"]) for obj in status.get("objects", [])):
                clear_stamps.clear()
                continue
            try:
                tool = self.tf.lookup_transform(self.settings["frame_id"],
                                                self.settings.get("tip_link", "gripper_frame_link"),
                                                Time(seconds=float(status["stamp"])))
            except TransformException:
                clear_stamps.clear()
                continue
            p = tool.transform.translation
            if self._inside((p.x, p.y, p.z)):
                clear_stamps.clear()
                continue
            clear_stamps.add(status["stamp"])
            if len(clear_stamps) >= 3:
                self.baseline_ids = {obj["track_id"] for obj in status.get("objects", [])}
                return True

    def dispense(self, deadline, cancel):
        self._check(deadline, cancel)
        self._require_placement_ready()
        self.dispensed_after = self.node.get_clock().now().nanoseconds * 1e-9
        reply = self.ev3.call("eject", {"cycles": 1})
        job = reply.get("job", {})
        if reply.get("accepted") is not True or type(job.get("id")) is not int:
            raise RuntimeError("EV3 did not admit a uniquely identified eject job")
        while True:
            self._spin(deadline, cancel)
            status = self.ev3.call("status")
            if status.get("session") != self.session:
                raise RuntimeError("EV3 daemon restarted; reference and delivery state unknown")
            if status.get("busy") is False:
                last = status.get("last_job") or {}
                if (last.get("id") != job["id"] or last.get("operation") != "eject"
                        or last.get("state") != "complete" or status.get("last_error")
                        or last.get("cleanup_fault")):
                    raise RuntimeError("EV3 eject did not complete cleanly")
                return {"session": self.session, "job_id": job["id"], "status": status}

    def detect(self, deadline, cancel):
        selector = StableTargetSelector(after_stamp=self.dispensed_after,
                                        excluded_ids=self.baseline_ids)
        while True:
            self._spin(deadline, cancel)
            status = self._fresh_status()
            if status is None:
                continue
            observed = dict(status)
            observed["objects"] = [obj for obj in status.get("objects", [])
                                   if self._inside(obj["position_world"])]
            target = selector.update(observed, now=self.node.get_clock().now().nanoseconds * 1e-9)
            if target:
                return {"track_id": target.track_id, "color": target.color,
                        "position": target.position, "stamp": target.stamp,
                        "size": next(item.get("size") for item in observed["objects"]
                                     if str(item["track_id"]) == str(int(target.track_id.rsplit("-", 1)[1])))}

    def pick_place(self, target, destination, deadline, cancel):
        self._check(deadline, cancel)
        self.serial += 1
        output = self.output / f"grasp-{self.serial:04d}.json"
        clear_pose_file = self.output / f"clear-joints-{self.serial:04d}.json"
        with clear_pose_file.open("x") as stream:
            json.dump(self.settings["clear_joint_pose_rad"], stream, allow_nan=False, sort_keys=True)
            stream.write("\n")
        command = [sys.executable, str(ROOT / "tools/ros_plan_grasp.py"), "--execute",
                   "--execution-config", self.settings["grasp_execution_config"],
                   "--timeout", str(min(180., max(.1, deadline-time.monotonic()))),
                   "--track-id", target["track_id"], "--frame", self.settings["frame_id"],
                   "--return-joint-pose", str(clear_pose_file),
                   "--place-pose", *(str(value) for value in destination), "--output", str(output)]
        selection = self.settings.get("placement_selections", {}).get(target["color"])
        if selection:
            execution = json.loads(Path(self.settings["grasp_execution_config"]).read_text())
            from workshop.station.placement_execution import require_observed_footprint
            require_observed_footprint(target.get("size"), execution.get("placement_footprint_m"))
            self._require_placement_ready(colors={target["color"]})
            command.extend(["--placement-selection", selection])
        with (self.output / f"grasp-{self.serial:04d}.log").open("x") as stream:
            self._check(deadline, cancel)
            self.child = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            while self.child.poll() is None:
                self._spin(deadline, cancel)
            code = self.child.returncode
            self.child = None
        if code != 0:
            raise RuntimeError(f"grasp execution failed ({code}); inspect {output.with_suffix('.log')}")
        return json.loads(output.read_text())

    def stop(self):
        from action_msgs.srv import CancelGoal
        failures = []
        # Independent shutdown attempts: an EV3 error must not skip arm cancel.
        try:
            self.ev3.call("stop")
        except BaseException as error:
            failures.append(str(error))
        # Stop the goal producer before the final cancellation requests. A
        # client still planning must not submit a new goal after cancellation.
        try:
            if self.child is not None and self.child.poll() is None:
                self.child.send_signal(signal.SIGINT)
                try:
                    self.child.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    self.child.terminate()
                    try:
                        self.child.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        self.child.kill()
                        self.child.wait(timeout=2.)
                    failures.append("grasp client required forced shutdown; server cancellation still required")
        except BaseException as error:
            failures.append("grasp client shutdown failed: " + str(error))
        for index, client in enumerate(self.cancel_clients):
            try:
                if not client.wait_for_service(timeout_sec=1.):
                    raise RuntimeError("action cancellation service unavailable")
                future = client.call_async(CancelGoal.Request())
                deadline = time.monotonic() + 3.
                while not future.done() and time.monotonic() < deadline:
                    self.rclpy.spin_once(self.node, timeout_sec=.05)
                response = future.result() if future.done() else None
                if response is None or response.return_code != CancelGoal.Response.ERROR_NONE:
                    raise RuntimeError("action cancellation was not acknowledged")
                goals = [bytes(item.goal_id.uuid) for item in response.goals_canceling]
                deadline = time.monotonic() + 5.
                while any(self._action_statuses[index].get(goal) not in (4, 5, 6) for goal in goals):
                    if time.monotonic() >= deadline:
                        raise RuntimeError("cancel accepted but action did not confirm terminal hold/stop")
                    self.rclpy.spin_once(self.node, timeout_sec=.05)
            except BaseException as error:
                failures.append(str(error))
        if failures:
            raise RuntimeError("; ".join(failures))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True, help="new evidence directory")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--execute", action="store_true", help="explicitly admit physical cycles")
    parser.add_argument("--ev3-port", type=int, default=18766)
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error("physical sorting requires explicit --execute")
    settings = load_settings(args.config)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    cancel = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: cancel.set())
    signal.signal(signal.SIGTERM, lambda *_: cancel.set())
    rclpy.init()
    node = Node("so101_sort_station")
    client = EV3LiveClient(port=args.ev3_port, timeout=.75)
    try:
        with station_lock(state_dir() / "ros-sorting.lock"):
            client.connect()
            devices = ROSSortingDevices(settings, client, output, node=node, rclpy=rclpy)
            with (output / "cycles.jsonl").open("x", buffering=1) as stream:
                def log(record):
                    stream.write(json.dumps(record, allow_nan=False, sort_keys=True) + "\n")
                result = run_sorting(devices, destinations=settings["destinations"],
                                     cycles=args.cycles, log=log, cancel=cancel)
            (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            return 0
    finally:
        client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
