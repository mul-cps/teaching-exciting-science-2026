#!/usr/bin/env python3
"""Real ROS RPC routing with fake services only. Explicit isolated domain required.

Run with the built operator interface overlay sourced, ROS_DOMAIN_ID=87 and
ROS_LOCALHOST_ONLY=1. No control/ArmSession modules, devices or systemd calls.
"""
import importlib.machinery
import json
import os
from pathlib import Path
import threading
import time
import unittest

if os.environ.get("ROS_DOMAIN_ID") != "87" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("fake operator checks require ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1")

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from so101_station_operator.srv import OperatorMaintenance

api_module = importlib.machinery.SourceFileLoader(
    "operator_api_test", str(Path(__file__).resolve().parents[1] / "scripts/operator_api")).load_module()
JOINTS = api_module.JOINTS
UNIT = "so101-fake-perception.service"


class FakeOwner(Node):
    def __init__(self):
        super().__init__("fake_operator_owner")
        self.group = ReentrantCallbackGroup()
        self.calls = []
        self.delay = 0.
        self.fail_restart = False
        self.recovery_mode = "ready"
        self.recovery_started = 0
        self.recovery_polls = 0
        self.state = dict(kind="follower_status", fresh=True, connected=True, observe_only=False,
                          owner_faulted=False, stop_latched=False, active_operation=None,
                          teleop_state="disabled", teleop_leader_ready=True, motion_profile_rearm_required=True,
                          torque={j: 0 for j in JOINTS}, raw={j: 2048 for j in JOINTS},
                          pose={j: 43. for j in JOINTS}, arm_id="FAKE", calibration_sha256="a"*64,
                          owner_started_monotonic=1., owner_source_sha256={"fake": "b"*64},
                          position_monotonic=time.monotonic(), native_damping_supported=False,
                          operator_restart_units=[UNIT], operator_external_job=None)
        self.services_owned = [self.create_service(Trigger, "/follower/" + name,
            lambda req, res, name=name: self.trigger(name, res), callback_group=self.group)
            for name in ("status", "stop", "home", "reconnect", "rearm_motion_profile", "damping_enable")]
        self.services_owned += [self.create_service(SetBool, "/follower/teleop_enable", self.teleop,
                                                   callback_group=self.group),
                                self.create_service(OperatorMaintenance, "/follower/operator_maintenance_enable",
                                                    self.maintenance, callback_group=self.group)]
        self.rgb = self.create_publisher(CameraInfo, "/camera/color/camera_info", 1)
        self.depth = self.create_publisher(CameraInfo, "/so101_perception/rocm/aligned_camera_info", 1)
        self.tags = self.create_publisher(String, "/so101_perception/tag_detections", 1)
        self.services_owned += [self.create_service(Trigger, "/so101_perception_recovery/" + endpoint,
            lambda req, res, endpoint=endpoint: self.recovery(endpoint, res), callback_group=self.group)
            for endpoint in ("start_recovery", "recovery_status")]
        self.publish_streams = True
        self.create_timer(.05, self.stream)

    def stream(self):
        if not self.publish_streams:
            return
        stamp = self.get_clock().now().to_msg()
        msg = CameraInfo()
        msg.header.stamp = stamp
        self.rgb.publish(msg)
        self.depth.publish(msg)
        self.tags.publish(String(data=json.dumps(dict(schema="so101-tag-detections-v1",
            stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), observations=[]))))

    def trigger(self, name, res):
        if name == "status":
            res.success, res.message = True, json.dumps(self.state)
            return res
        self.calls.append(name)
        if name == "stop":
            self.state.update(torque={j: 0 for j in JOINTS}, active_operation=None,
                              motion_profile_rearm_required=True, teleop_state="disabled", operator_maintenance_token=None)
        elif name == "reconnect":
            self.state["connected"] = True
        elif name == "rearm_motion_profile":
            assert all(v == 0 for v in self.state["torque"].values())
            self.state["motion_profile_rearm_required"] = False
        elif name == "home":
            self.state["active_operation"] = "home"
            time.sleep(self.delay)
            if self.state["active_operation"] != "home":
                res.success, res.message = False, "home interrupted by STOP"
                return res
            self.state.update(active_operation=None, home_completed_this_session=True,
                              torque={j: 1 for j in JOINTS})
        res.success, res.message = True, name + " fake acknowledged"
        return res

    def teleop(self, req, res):
        self.calls.append("teleop:" + str(req.data))
        self.state.update(teleop_state="active" if req.data else "disabled",
                          active_operation="teleop" if req.data else None)
        res.success, res.message = True, "fake teleop acknowledged"
        return res

    def maintenance(self, req, res):
        self.calls.append("maintenance:" + req.operation)
        token = "c" * 32
        if req.operation == "restart":
            assert req.token == token and req.unit == UNIT
            result = dict(job_id="fake-job", kind="restart_unit_terminal", success=not self.fail_restart)
            self.state.update(operator_restart_result=result, operator_external_job=None)
            res.message = json.dumps(dict(job_id="fake-job"))
        elif req.operation == "verify":
            self.state.update(operator_external_job=None)
            res.message = "verified"
        else:
            self.state.update(active_operation="operator_maintenance" if req.enable else None,
                              operator_maintenance_token=token if req.enable else None)
            res.message = json.dumps(dict(token=token))
        res.success = True
        return res

    def recovery(self, endpoint, res):
        if endpoint == "start_recovery":
            self.recovery_started += 1
            self.recovery_polls = 0
            res.success = self.recovery_mode != "busy"
            res.message = json.dumps(dict(accepted=res.success, job_id="f"*32, reason="one recovery job already active"))
        else:
            self.recovery_polls += 1
            ready = self.recovery_mode == "ready" or (self.recovery_mode == "stale" and self.recovery_polls >= 3)
            res.success = True
            res.message = json.dumps(dict(job_id="a"*32 if self.recovery_mode == "mismatch" else "f"*32,
                state="failed" if self.recovery_mode == "failed" else "wait_consumer_admissions" if self.recovery_mode == "waiting" else "completed",
                operation_success=self.recovery_mode in {"ready", "stale"}, ready_now=ready,
                reason="fake admission failure"))
        return res


class OperatorFakeROS(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=["--ros-args", "-p", "enable_service_restarts:=true", "-p", "camera_unit:="+UNIT,
                        "-p", "depth_unit:="+UNIT, "-p", "tags_unit:="+UNIT])
        cls.owner, cls.api, cls.client = FakeOwner(), api_module.OperatorAPI(), Node("fake_operator_client")
        cls.executor = MultiThreadedExecutor(num_threads=10)
        for node in (cls.owner, cls.api, cls.client):
            cls.executor.add_node(node)
        cls.received_status = {}
        cls.status_subscription = cls.client.create_subscription(String, "/station/operator/status",
            lambda msg: cls.received_status.update(json.loads(msg.data)), 1)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()
        cls.routes = {command: cls.client.create_client(Trigger, "/station/operator/" + command)
                      for command in api_module.COMMANDS}
        for client in cls.routes.values():
            assert client.wait_for_service(timeout_sec=5.)
        time.sleep(.7)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.thread.join()
        for node in (cls.owner, cls.api, cls.client):
            node.destroy_node()
        rclpy.shutdown()

    def call(self, command, success=True):
        future = self.routes[command].call_async(Trigger.Request())
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(future.done(), command + " timed out")
        result = future.result()
        self.assertEqual(result.success, success, command + ": " + result.message)
        return result

    def test_all_routes_and_lifecycle(self):
        self.assertEqual(len(self.api.operator_services), len(api_module.COMMANDS))
        self.assertEqual(len(self.api.stream_subscriptions), 3)  # real constructor/property proof
        self.owner.state["connected"] = False
        self.call("reset")
        self.assertIn("reconnect", self.owner.calls)
        self.assertFalse(self.owner.state["motion_profile_rearm_required"])
        self.call("start")
        self.assertTrue(self.owner.state["home_completed_this_session"])
        self.call("zero_torque")
        self.call("moveit")
        self.assertFalse(self.owner.state["motion_profile_rearm_required"])
        self.owner.state["teleop_leader_ready"] = False
        self.assertFalse(self.api.capabilities(self.owner.state)["teleop"]["enabled"])
        self.assertIn("fresh calibrated leader", self.call("teleop", False).message)
        self.owner.state["teleop_leader_ready"] = True
        self.assertTrue(self.api.capabilities(self.owner.state)["teleop"]["enabled"])
        # Authoritative start admission includes once-per-owner home provenance;
        # the UI must not compare the current pose against home independently.
        self.owner.state.update(teleop_start_ready=False, teleop_start_reason="complete saved home once for this owner")
        self.assertFalse(self.api.capabilities(self.owner.state)["teleop"]["enabled"])
        self.assertIn("saved home once", self.call("teleop", False).message)
        deadline = time.monotonic()+3.
        while "saved home once" not in self.received_status.get("summary", "") and time.monotonic() < deadline:
            time.sleep(.05)
        self.assertIn("saved home once", self.received_status["summary"])
        self.owner.state.update(teleop_start_ready=True, teleop_start_reason=None,
                                home_completed_this_session=True, pose={j: 84. for j in JOINTS})
        self.assertTrue(self.api.capabilities(self.owner.state)["teleop"]["enabled"])
        self.call("teleop")
        self.assertEqual(self.owner.state["teleop_state"], "active")
        self.call("moveit")
        self.assertEqual(self.owner.state["teleop_state"], "disabled")
        self.call("base_release")
        self.owner.state["owner_source_sha256"] = {"changed": "d"*64}
        self.call("base_confirm", False)
        self.owner.state["owner_source_sha256"] = {"fake": "b"*64}
        self.call("base_confirm")  # deliberately differs from any saved home
        self.assertEqual(self.api.manual_record["raw"], self.owner.state["raw"])
        self.assertFalse(self.api.capabilities(self.owner.state)["damping"]["enabled"])
        self.call("damping", False)
        self.assertNotIn("damping_enable", self.owner.calls)
        self.call("reset")
        for command in ("restart_camera", "restart_depth", "restart_tags", "restart_all"):
            before = self.owner.calls.count("maintenance:restart")
            recovery_before = self.owner.recovery_started
            self.call(command)
            self.assertEqual(self.owner.recovery_started, recovery_before+1)
            self.assertEqual(self.owner.calls.count("maintenance:restart"), before+1)  # aliases once
        self.call("verify_restart")
        for mode, reason in (("failed", "fake admission failure"), ("busy", "already active")):
            self.owner.recovery_mode = mode
            self.assertIn(reason, self.call("restart_camera", False).message)
            self.assertTrue(self.api.all_off(self.owner.state))
        self.owner.recovery_mode = "stale"
        self.call("restart_camera")
        self.assertGreaterEqual(self.owner.recovery_polls, 3)  # same job, no second recovery dispatch
        self.owner.recovery_mode = "mismatch"
        self.assertIn("job_id", self.call("restart_camera", False).message)
        self.assertTrue(self.api.uncertain)
        self.assertEqual(self.owner.state["active_operation"], "operator_maintenance")
        self.call("verify_restart", False)
        self.call("zero_torque")
        self.api.uncertain = self.api.uncertain_motion = False  # isolated fake explicit inspection/recovery
        self.owner.recovery_mode = "waiting"
        before = self.owner.recovery_started
        pending_restart = self.routes["restart_all"].call_async(Trigger.Request())
        deadline = time.monotonic()+3.
        while self.owner.recovery_started == before and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertGreater(self.owner.recovery_started, before)
        self.call("zero_torque")
        time.sleep(.2)
        self.assertTrue(pending_restart.done())
        self.assertFalse(pending_restart.result().success)
        self.owner.recovery_mode = "ready"
        self.owner.fail_restart = True
        self.call("restart_camera", False)
        self.assertTrue(self.api.uncertain)
        self.call("start", False)
        self.call("zero_torque")
        self.call("verify_restart")  # known terminal failure can be explicitly acknowledged
        self.owner.state["operator_external_job"] = dict(token="c"*32)
        self.api.uncertain = True
        self.call("verify_restart")
        self.assertFalse(self.api.uncertain)
        self.owner.fail_restart = False
        self.call("reset")
        self.owner.delay = .4
        home = self.routes["start"].call_async(Trigger.Request())
        time.sleep(.1)
        self.call("zero_torque")  # independent lock; delayed reply retained
        time.sleep(.5)
        self.assertTrue(home.done())
        self.assertFalse(home.result().success)
        self.assertTrue(self.api.all_off(self.owner.state))
        # A timed out RPC stays unknown when its late reply arrives; STOP never clears it.
        with self.assertRaisesRegex(RuntimeError, "outcome unknown"):
            self.api.rpc("home", timeout=.02)
        self.call("zero_torque")
        time.sleep(.5)
        self.assertTrue(self.api.uncertain)
        self.call("verify_restart", False)
        self.call("start", False)
        # Stream health is advancing source metadata, including empty valid detection lists.
        self.assertTrue(all(stamp is not None for stamp in self.api.streams.values()))
        self.owner.publish_streams = False
        time.sleep(.15)
        stamp = self.api.stream_stamps["tags"]
        previous = self.api.streams["tags"]
        self.api.stream("tags", String(data=json.dumps(dict(schema="so101-tag-detections-v1",
            stamp=dict(sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000), observations=[]))))
        self.assertEqual(self.api.streams["tags"], previous)
        self.api.stream("tags", String(data="{}"))
        self.assertEqual(self.api.streams["tags"], previous)
        self.assertIn("schema", self.api.stream_errors["tags"])
        time.sleep(1.05)
        self.assertGreater(time.monotonic()-self.api.streams["tags"], 1.)
        self.assertTrue(self.owner.calls.count("rearm_motion_profile") >= 3)
        self.owner.state.update(fresh=False, torque={j: 1 for j in JOINTS})
        deadline = time.monotonic()+3.
        while "STALE / LAST KNOWN" not in self.received_status.get("summary", "") and time.monotonic() < deadline:
            time.sleep(.05)
        self.assertIn("STALE / LAST KNOWN", self.received_status["summary"])
        self.assertIn("current torque/stop/fault UNKNOWN", self.received_status["summary"])
        print("FAKE DOMAIN87: all 13 button routes, reset/rearm/home, teleop/MoveIt, manual reference, "
              "damping unavailable, restart aliases/recovery correlation/transient readiness/failure/busy/STOP, "
              "unknown barriers, delayed reply/STOP, advancing/stale streams and stale torque labels passed")


if __name__ == "__main__":
    unittest.main()
