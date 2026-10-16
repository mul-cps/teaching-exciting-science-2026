#!/usr/bin/env python3
"""Typed ROS-only quick commands for the resident sorting station."""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('status', 'diagnostics', 'check-ready', 'cancel', 'stop', 'reset-fault',
                 'connect', 'wake', 'sleep', 'ev3-home', 'arm-home', 'arm-connect', 'arm-recover', 'arm-status',
                 'arm-prepare', 'arm-stop'):
        commands.add_parser(name)
    jaw = commands.add_parser('gripper-step')
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
    if args.command == 'gripper-step' and not 0 < abs(args.delta_counts) <= 16:
        parser.error('delta_counts must be nonzero and within -16..16')
    if args.command == 'run' and not 1 <= args.cycles <= 100:
        parser.error('cycles must be 1..100')
    return args


def main(argv=None):
    args = parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    from so101_station_workflow.srv import RunCycles, ConfirmReference, SelectDestination, SelectPlacementRegion
    rclpy.init()
    node = Node('station_quick_action')
    try:
        service_type = Trigger
        request = Trigger.Request()
        names = {'arm-connect': '/follower/reconnect', 'arm-recover': '/follower/recover_camera',
                 'arm-status': '/follower/status', 'arm-stop': '/follower/stop', 'arm-prepare': 'arm/prepare',
                 'connect': 'ev3/connect', 'ev3-home': 'ev3/home', 'arm-home': 'arm/home'}
        name = names.get(args.command, args.command.replace('-', '_'))
        if args.command == 'gripper-step':
            from so101_station_workflow.srv import CommissionGripperStep
            name, service_type = '/follower/commission_gripper_step', CommissionGripperStep
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
        deadline = time.monotonic()+({'arm-connect': 25., 'arm-recover': 10., 'arm-stop': 12.}.get(args.command, 5.))
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
