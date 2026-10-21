"""Read-only leader state publisher for resident SO-ARM101 teleoperation."""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .leader_reader import JOINTS, joint_state_positions, validate_pose_packet

LEADER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9068765-if00"
DEFAULT_CALIBRATION_DIR = "/home/bjoern/git/soarm101-highschool-demos/.calibration"


class LeaderTeleopPublisher(Node):
    def __init__(self):
        super().__init__("so101_leader_teleop")
        self.declare_parameter("station_python", str(Path.home() / ".local/share/exciting-science/station/venv/bin/python"))
        self.declare_parameter("station_root", "/home/bjoern/git/teaching-exciting-science-2026")
        self.declare_parameter("leader_port", LEADER_PORT)
        self.declare_parameter("calibration_dir", DEFAULT_CALIBRATION_DIR)
        self.declare_parameter("reader_hz", 125.0)
        self.declare_parameter("pose_stale_s", 0.15)
        self.declare_parameter("pose_topic", "/leader/teleop_pose")
        self.declare_parameter("joint_states_topic", "/leader/joint_states")
        pose_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._pose_pub = self.create_publisher(
            String, str(self.get_parameter("pose_topic").value), pose_qos)
        joint_state_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                                     reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self._joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_states_topic").value), joint_state_qos)
        self._latest = queue.Queue(maxsize=1)
        self._leader_ready_reported = False
        self._process = None
        self._reader = None
        self._reader_closed = False
        self._reader_retry_at = 0.0
        self._reader_retry_delay = 0.5
        self._supervise_reader()
        self._reader_timer = self.create_timer(0.25, self._supervise_reader)
        self._timer = self.create_timer(1.0 / 125.0, self._publish_latest)

    def _start_reader(self):
        station_python = (str(self.get_parameter("station_python").value).strip() or
                          str(Path.home() / ".local/share/exciting-science/station/venv/bin/python"))
        station_root = Path(str(self.get_parameter("station_root").value)).resolve()
        reader_script = Path(__file__).with_name("leader_reader.py")
        command = [station_python, str(reader_script),
                   "--port", str(self.get_parameter("leader_port").value),
                   "--calibration-dir", str(self.get_parameter("calibration_dir").value),
                   "--hz", str(float(self.get_parameter("reader_hz").value))]
        env = os.environ.copy()
        for key in tuple(env):
            if (key.startswith(("PYTHON", "LD_", "ROS_", "AMENT_", "COLCON_", "RMW_",
                                "CONDA_", "VIRTUAL_ENV")) or
                    key in {"CMAKE_PREFIX_PATH", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"}):
                env.pop(key, None)
        env["PATH"] = os.pathsep.join((str(Path(station_python).parent), os.defpath))
        env["PYTHONPATH"] = str(station_root)
        process = subprocess.Popen(command, cwd=station_root, env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, bufsize=1)
        self._process = process

        def collect():
            assert process.stdout is not None
            for line in process.stdout:
                try:
                    packet = json.loads(line)
                    pose = validate_pose_packet(
                        packet, time.monotonic(), float(self.get_parameter("pose_stale_s").value))
                    if self._reader_closed or self._process is not process:
                        return
                    packet["pose"] = pose
                    state_positions = joint_state_positions(pose)
                    packet["joint_positions_rad"] = state_positions
                    while True:
                        try:
                            self._latest.get_nowait()
                        except queue.Empty:
                            break
                    self._latest.put_nowait(packet)
                except (ValueError, TypeError, KeyError):
                    continue

        thread = threading.Thread(target=collect, name="leader-state-reader", daemon=True)
        thread.start()
        threading.Thread(target=self._read_errors, args=(process,),
                         name="leader-reader-stderr", daemon=True).start()
        return thread

    def _supervise_reader(self):
        """Retry a missing/disconnected reader with bounded backoff; never enable teleop."""
        if self._reader_closed:
            return
        now = time.monotonic()
        process = self._process
        if process is not None:
            if process.poll() is None:
                return
            self.get_logger().warning(
                f"Leader reader exited with status {process.returncode}; retrying in {self._reader_retry_delay:g}s")
            self._process = None
            self._reader_retry_at = now + self._reader_retry_delay
            self._reader_retry_delay = min(8.0, self._reader_retry_delay * 2)
        if now < self._reader_retry_at:
            return
        # A new generation may restart its sequence, but must supply a newly
        # validated packet; never republish the previous reader's cached pose.
        while True:
            try:
                self._latest.get_nowait()
            except queue.Empty:
                break
        self._leader_ready_reported = False
        try:
            self._reader = self._start_reader()
        except OSError as error:
            self.get_logger().warning(f"Leader reader unavailable: {error}; retrying in {self._reader_retry_delay:g}s")
            self._process = None
            self._reader_retry_at = now + self._reader_retry_delay
            self._reader_retry_delay = min(8.0, self._reader_retry_delay * 2)

    def _read_errors(self, process):
        assert process.stderr is not None
        for line in process.stderr:
            self.get_logger().error("Leader reader: " + line.rstrip())

    def _publish_latest(self):
        try:
            packet = self._latest.get_nowait()
        except queue.Empty:
            return
        age = time.monotonic() - float(packet["stamp_s"])
        if age < -0.02 or age > float(self.get_parameter("pose_stale_s").value):
            return
        if not self._leader_ready_reported:
            self.get_logger().info(
                f"Leader ready: fresh calibrated pose stream from {self.get_parameter('leader_port').value}")
            self._leader_ready_reported = True
            self._reader_retry_delay = 0.5
        # The JSON stamp is the reader's monotonic acquisition time; bridge freshness checks
        # use this shared host clock and reject data older than 150 ms.
        packet["stamp_s"] = float(packet["stamp_s"])
        packet["publish_stamp_s"] = time.monotonic()
        message = String()
        message.data = json.dumps(packet, separators=(",", ":"))
        self._pose_pub.publish(message)
        joint_state = JointState()
        joint_state.header.stamp = (self.get_clock().now() - Duration(seconds=max(0.0, age))).to_msg()
        joint_state.name = list(JOINTS)
        joint_state.position = packet["joint_positions_rad"]
        self._joint_pub.publish(joint_state)

    def destroy_node(self):
        self._reader_closed = True
        timer = getattr(self, "_reader_timer", None)
        if timer is not None:
            timer.cancel()
        process = getattr(self, "_process", None)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderTeleopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
