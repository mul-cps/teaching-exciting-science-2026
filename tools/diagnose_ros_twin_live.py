#!/usr/bin/env python3
"""Collect and evaluate read-only live ROS state for passive-twin qualification.

Collection requests robot_description from RSP and MoveIt and subscribes to
joint/TF/object state. It does not call motion, gripper, scene-write, camera, or
serial-control APIs. Run `collect` with the Jazzy ROS wrapper; run `evaluate`
with the station Python environment containing MuJoCo.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _collect(args):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from workshop.digital_twin.ros_state_node import RosStatePublisher

    def query_parameter(node, service_name, deadline_s):
        client = node.create_client(GetParameters, service_name)
        if not client.wait_for_service(timeout_sec=deadline_s):
            raise RuntimeError(f"parameter service unavailable: {service_name}")
        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = client.call_async(request)
        deadline = time.monotonic() + deadline_s
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            raise TimeoutError(f"parameter request timed out: {service_name}")
        values = future.result().values
        if not values or not values[0].string_value:
            raise RuntimeError(f"robot_description missing from {service_name}")
        return values[0].string_value

    rclpy.init(args=None)
    node = RosStatePublisher(str(args.output.with_suffix(".snapshot.tmp.json")),
                             root_frame=args.root_frame, max_age_s=args.max_age)
    try:
        rsp_xml = query_parameter(node, args.rsp_service, args.timeout)
        moveit_xml = query_parameter(node, args.moveit_service, args.timeout)
        arm_links = ("shoulder_link", "upper_arm_link", "lower_arm_link",
                     "wrist_link", "gripper_link", "gripper_frame_link")
        deadline = time.monotonic() + args.timeout
        snapshot = None
        ready_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            candidate = node.state.snapshot(time.monotonic())
            candidate["root_frame"] = args.root_frame.lstrip("/")
            if (candidate.get("robot_description") and candidate.get("joint_state")
                    and "base_link" in candidate.get("transforms", {})
                    and all(frame in candidate.get("transforms", {}) for frame in arm_links)):
                snapshot = candidate
                if ready_since is None:
                    ready_since = time.monotonic()
                if time.monotonic() - ready_since >= args.settle:
                    break
        if snapshot is None:
            raise TimeoutError("complete robot description, joint state, and arm TF chain not received")
        report = {
            "mode": "read_only_live_ros_model_parity",
            "domain_id": args.domain_id,
            "sources": {
                "rsp_parameter": args.rsp_service,
                "moveit_parameter": args.moveit_service,
                "twin_input": "/follower/robot_description + /follower/joint_states + /tf + /tf_static",
            },
            "robot_description": {"rsp_raw": rsp_xml, "moveit_raw": moveit_xml,
                                  "twin_resolved": snapshot["robot_description"]},
            "snapshot": snapshot,
            "actuation": "none; read-only parameters/subscriptions only",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
        print(json.dumps({
            "artifact": str(args.output),
            "raw_robot_descriptions_equal": rsp_xml == moveit_xml,
            "joint_count": len(snapshot["joint_state"]["positions_rad"]),
            "dynamic_tf_fresh_count": snapshot["diagnostics"]["dynamic_tf_fresh_count"],
            "dynamic_tf_count": snapshot["diagnostics"]["dynamic_tf_count"],
            "freshness": snapshot["diagnostics"]["freshness"],
            "joint_tf_delta_s": snapshot["diagnostics"]["joint_tf_delta_s"],
            "joint_tf_sync_basis": snapshot["diagnostics"]["joint_tf_sync_basis"],
            "object_count": len(snapshot["objects"]),
            "fresh_object_count": sum(snapshot["object_fresh"].values()),
            "actuation": report["actuation"],
        }, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _evaluate(args):
    import mujoco
    from workshop.digital_twin.mujoco_ros import (
        apply_ros_state, compare_tool_frame, effective_urdf_to_mjcf_model,
    )

    report = json.loads(args.input.read_text())
    snapshot = report["snapshot"]
    descriptions = report["robot_description"]
    rsp_xml, moveit_xml = descriptions["rsp_raw"], descriptions["moveit_raw"]
    twin_xml = descriptions["twin_resolved"]
    model = effective_urdf_to_mjcf_model(twin_xml)
    data = mujoco.MjData(model)
    applied = apply_ros_state(model, data, snapshot, root_frame=args.root_frame,
                              require_fresh=False)
    parity = compare_tool_frame(model, data, snapshot, twin_xml,
                                frame_id=args.tool_frame)
    sha = lambda text: hashlib.sha256(text.encode()).hexdigest()
    diagnostics = snapshot["diagnostics"]
    translation_error = float(parity["translation_error_m"])
    rotation_error = float(parity["rotation_error_deg"])
    parity_qualified = bool(
        rsp_xml == moveit_xml and diagnostics["freshness"] == "fresh"
        and math.isfinite(translation_error) and math.isfinite(rotation_error)
        and translation_error <= args.translation_tolerance
        and rotation_error <= args.rotation_tolerance_deg)
    result = {
        "mode": "read_only_live_ros_model_parity",
        "domain_id": report.get("domain_id"),
        "robot_description": {
            "rsp_raw_sha256": sha(rsp_xml),
            "moveit_raw_sha256": sha(moveit_xml),
            "raw_exact_match": rsp_xml == moveit_xml,
            "resolved_twin_sha256": sha(twin_xml),
            "mujoco_model_nq": int(model.nq),
            "mujoco_model_njnt": int(model.njnt),
            "mujoco_model_nbody": int(model.nbody),
        },
        "state": {
            "freshness": diagnostics["freshness"],
            "joint_tf_delta_s": diagnostics["joint_tf_delta_s"],
            "joint_tf_sync_basis": diagnostics["joint_tf_sync_basis"],
            "joint_state_age_s": diagnostics["joint_state_age_s"],
            "dynamic_tf_fresh_count": diagnostics["dynamic_tf_fresh_count"],
            "dynamic_tf_count": diagnostics["dynamic_tf_count"],
            "arm_tf_fresh_count": diagnostics["arm_tf_fresh_count"],
            "arm_tf_count": diagnostics["arm_tf_count"],
            "joint_names": sorted(snapshot["joint_state"]["positions_rad"]),
            "joint_stamp_ns": snapshot["joint_state"]["stamp_ns"],
            "world_from_base": applied["world_from_base"].tolist(),
            "fresh_object_count": sum(snapshot["object_fresh"].values()),
            "object_count": len(snapshot["objects"]),
        },
        "tool_frame": parity,
        "thresholds": {"translation_error_max_m": args.translation_tolerance,
                       "rotation_error_max_deg": args.rotation_tolerance_deg},
        "mathematical_parity_qualified": parity_qualified,
        "physical_registration_qualified": False,
        "qualification_scope": "effective_model_and_current_measured_state_mathematical_parity_only",
        "actuation": "none; parameter/subscription snapshot and offline MuJoCo FK",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({"artifact": str(args.output), **result}, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collect = commands.add_parser("collect", help="read live ROS parameters and state")
    collect.add_argument("--output", type=Path, required=True)
    collect.add_argument("--domain-id", type=int, default=0,
                         help="recording metadata only; use ROS_DOMAIN_ID to select the domain")
    collect.add_argument("--root-frame", default="charuco_board")
    collect.add_argument("--max-age", type=float, default=.5)
    collect.add_argument("--timeout", type=float, default=8.)
    collect.add_argument("--settle", type=float, default=1.,
                         help="continue spinning after readiness to collect fresh scene inputs")
    collect.add_argument("--rsp-service", default="/follower/robot_state_publisher/get_parameters")
    collect.add_argument("--moveit-service", default="/follower/move_group/get_parameters")
    collect.set_defaults(run=_collect)
    evaluate = commands.add_parser("evaluate", help="offline model and tool-frame parity")
    evaluate.add_argument("--input", type=Path, required=True)
    evaluate.add_argument("--output", type=Path, required=True)
    evaluate.add_argument("--root-frame", default="charuco_board")
    evaluate.add_argument("--tool-frame", default="gripper_frame_link")
    evaluate.add_argument("--translation-tolerance", type=float, default=.001)
    evaluate.add_argument("--rotation-tolerance-deg", type=float, default=.5)
    evaluate.set_defaults(run=_evaluate)
    args = parser.parse_args(argv)
    args.run(args)


if __name__ == "__main__":
    main()
