#!/usr/bin/env python3
"""Typed ROS-only quick commands for the resident sorting station."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# Fixed diagnostic endpoints only. No caller-provided service names or paths.
PERCEPTION_SERVICES = {
    'perception-recover': '/so101_perception_recovery/start_recovery',
    'perception-status': '/so101_perception_recovery/recovery_status',
    'perception-supervisor-enable': '/so101_perception_supervisor/enable',
    'perception-supervisor-disable': '/so101_perception_supervisor/disable',
    'perception-supervisor-status': '/so101_perception_supervisor/status',
    'perception-supervisor-reset-fault': '/so101_perception_supervisor/reset_fault',
}


def _perception_domain():
    try:
        domain = int(os.environ.get('ROS_DOMAIN_ID', '76'))
    except ValueError as error:
        raise ValueError('ROS_DOMAIN_ID must be an integer in 1..232') from error
    if not 1 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID must be in 1..232; domain 0 is forbidden')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')


def _diagnostic_payload(command, result):
    if type(result.success) is not bool or not isinstance(result.message, str):
        raise ValueError('diagnostic Trigger response requires boolean success and JSON text')
    if len(result.message.encode('utf-8')) > 16384:
        raise ValueError('diagnostic response exceeds 16384 bytes')
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError('duplicate diagnostic JSON key')
            value[key] = item
        return value
    def nonfinite(value):
        raise ValueError('nonfinite diagnostic JSON')
    try:
        status = json.loads(result.message, object_pairs_hook=pairs, parse_constant=nonfinite)
    except (json.JSONDecodeError, RecursionError) as error:
        raise ValueError('invalid diagnostic JSON') from error
    if not isinstance(status, dict):
        raise ValueError('diagnostic JSON must be an object')
    if any(status.get(field, False) is not False for field in ('qualified', 'execution_enabled')):
        raise ValueError('diagnostic API cannot claim physical qualification or execution authority')
    for field in ('operation_success', 'ready_now', 'accepted', 'enabled',
                  'reset_check_accepted', 'reset_check_pending'):
        if field in status and type(status[field]) is not bool:
            raise ValueError('diagnostic '+field+' must be boolean')
    if command == 'perception-recover':
        if status.get('ready_now', False) or status.get('operation_success', False):
            raise ValueError('recovery acceptance cannot claim completed readiness')
        if result.success:
            job_id = status.get('job_id')
            if (status.get('accepted') is not True or not isinstance(job_id, str)
                    or len(job_id) != 32 or any(c not in '0123456789abcdef' for c in job_id)):
                raise ValueError('recovery acceptance requires accepted job_id')
    return dict(success=result.success, rpc_success=result.success, command=command,
                service=PERCEPTION_SERVICES[command], message=result.message, status=status,
                operation_success=status.get('operation_success', False),
                ready_now=status.get('ready_now', False), qualified=False, execution_enabled=False)


def _perception_action(command):
    # One absolute budget includes discovery, RPC, access and serialization.
    # Native ROS initialization, cleanup and output writes are not preemptible.
    deadline = time.monotonic()+2.
    def remaining():
        value = deadline-time.monotonic()
        if value <= 0:
            raise TimeoutError('Perception RPC result unknown; inspect status explicitly before retrying')
        return value
    _perception_domain()  # Before ROS imports/init; parser/help needs no ROS.
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    remaining()
    node = None
    future = None
    client = None
    initialized = False
    try:
        rclpy.init()
        initialized = True
        remaining()
        node = Node('station_quick_action')
        remaining()
        client = node.create_client(Trigger, PERCEPTION_SERVICES[command])
        remaining()
        if not client.wait_for_service(timeout_sec=remaining()):
            remaining()
            raise RuntimeError(PERCEPTION_SERVICES[command]+' unavailable; start its optional perception launch explicitly')
        remaining()
        future = client.call_async(Trigger.Request())
        remaining()
        while not future.done():
            rclpy.spin_once(node, timeout_sec=min(.05, remaining()))
            remaining()
        remaining()
        result = future.result()
        remaining()
        payload = _diagnostic_payload(command, result)
        remaining()
        encoded = json.dumps(payload, allow_nan=False)
        remaining()
        print(encoded)
        remaining()
        return 0 if payload['rpc_success'] else 1
    except BaseException:
        if future is not None:
            if client is not None:
                client.remove_pending_request(future)
            future.cancel()
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in PERCEPTION_SERVICES:
        commands.add_parser(name, help='One explicit diagnostic RPC; accepted/status success is not readiness')
    for name in ('status', 'diagnostics', 'check-ready', 'cancel', 'stop', 'reset-fault',
                 'connect', 'wake', 'sleep', 'ev3-home', 'arm-home', 'arm-connect', 'arm-recover', 'arm-status',
                 'arm-prepare', 'arm-capture-home', 'arm-stop', 'gripper-diagnostics'):
        commands.add_parser(name)
    intel = commands.add_parser('intel-grasp')
    intel.add_argument('request_json', help='GraspPlanning request JSON with original source stamp')
    jaw = commands.add_parser('gripper-step')
    held_jaw = commands.add_parser('gripper-held-step')
    held_jaw.add_argument('delta_counts', type=int)
    jaw.add_argument('delta_counts', type=int)
    axis = commands.add_parser('sorter-position')
    axis.add_argument('target_deg', type=float)
    run = commands.add_parser('run')
    run.add_argument('cycles', type=int, nargs='?', default=1)
    confirm = commands.add_parser('confirm-reference')
    confirm.add_argument('name', choices=['sorter', 'feeder'])
    confirm.add_argument('--physically-verified', action='store_true', required=True)
    selected = commands.add_parser('select-destination')
    selected.add_argument('color', choices=['red', 'green', 'blue', 'yellow'])
    selected.add_argument('selection_id', help='UUID reported by placement observation/UI')
    for name in ('pin-anchor', 'pin-area'):
        selected = commands.add_parser(name)
        selected.add_argument('generation', nargs='?', default='')
    for name in ('select-anchor', 'select-area', 'clear-selection'):
        selected = commands.add_parser(name)
        selected.add_argument('generation')
        if name != 'clear-selection':
            selected.add_argument('pixels', type=int, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'))
    args = parser.parse_args(argv)
    if args.command == 'sorter-position':
        from workshop.station.sorter_position import validate_target
        try:
            validate_target(args.target_deg)
        except ValueError as error:
            parser.error(str(error))
    if args.command in {'gripper-step', 'gripper-held-step'} and not 0 < abs(args.delta_counts) <= 16:
        parser.error('delta_counts must be nonzero and within -16..16')
    if args.command == 'run' and not 1 <= args.cycles <= 100:
        parser.error('cycles must be 1..100')
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.command in PERCEPTION_SERVICES:
        return _perception_action(args.command)
    if args.command == 'intel-grasp':
        from tools.ros_intel_grasp import main as intel_main
        return intel_main([args.request_json])
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from so101_station_workflow.srv import RunCycles, ConfirmReference, SelectDestination, SelectPlacementRegion
    rclpy.init()
    node = Node('station_quick_action')
    try:
        service_type = Trigger
        request = Trigger.Request()
        names = {'gripper-diagnostics': '/follower/gripper_diagnostics', 'arm-connect': '/follower/reconnect', 'arm-recover': '/follower/recover_camera',
                 'arm-capture-home': 'arm/capture_home', 'arm-status': '/follower/status', 'arm-stop': '/follower/stop', 'arm-prepare': 'arm/prepare',
                 'connect': 'ev3/connect', 'ev3-home': 'ev3/home', 'arm-home': 'arm/home'}
        name = names.get(args.command, args.command.replace('-', '_'))
        if args.command in {'gripper-step', 'gripper-held-step'}:
            from so101_station_workflow.srv import CommissionGripperStep
            name = '/follower/commission_gripper_held_step' if args.command == 'gripper-held-step' else '/follower/commission_gripper_step'
            service_type = CommissionGripperStep
            request = CommissionGripperStep.Request(delta_counts=args.delta_counts)
        elif args.command == 'sorter-position':
            from so101_station_workflow.srv import SorterPosition
            name, service_type = 'ev3/sorter_position', SorterPosition
            request = SorterPosition.Request(target_deg=args.target_deg)
        elif args.command == 'run':
            name, service_type = 'run_cycles', RunCycles
            request = RunCycles.Request(cycles=args.cycles)
        elif args.command == 'confirm-reference':
            name, service_type = 'ev3/confirm_reference', ConfirmReference
            request = ConfirmReference.Request(name=args.name, reference='home' if args.name == 'sorter' else 'eject',
                                               physically_verified=args.physically_verified)
        elif args.command == 'select-destination':
            service_type = SelectDestination
            request = SelectDestination.Request(color=args.color, selection_id=args.selection_id)
        elif args.command in {'select-anchor', 'select-area', 'clear-selection', 'pin-anchor', 'pin-area'}:
            service_type = SelectPlacementRegion
            name = 'placement/select'
            request = SelectPlacementRegion.Request(generation=args.generation,
                mode={'select-anchor': 'feeder', 'select-area': 'goal', 'clear-selection': 'clear', 'pin-anchor': 'pin_feeder', 'pin-area': 'pin_goal'}[args.command],
                pixel_rectangle=getattr(args, 'pixels', [0, 0, 0, 0]))
        service_name = name if name.startswith('/') else '/station/'+name
        client = node.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError(service_name+' unavailable; start station_live.launch.py')
        future = client.call_async(request)
        deadline = time.monotonic()+({'arm-connect': 25., 'arm-recover': 10., 'arm-stop': 12., 'gripper-held-step': 50.}.get(args.command, 5.))
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS service result unknown; inspect status before retrying')
            rclpy.spin_once(node, timeout_sec=.05)
        result = future.result()
        payload = {'success': result.success, 'message': result.message}
        if args.command == 'arm-status':
            payload['status'] = json.loads(result.message)
        if hasattr(result, 'selection_id'):
            payload['selection_id'] = result.selection_id
        print(json.dumps(payload))
        return 0 if result.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
