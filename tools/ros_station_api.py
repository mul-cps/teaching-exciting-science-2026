#!/usr/bin/env python3
"""Resident ROS workflow. Owns exactly one EV3 session; never opens arm/camera."""
import json
import signal
from pathlib import Path
import threading
import time

from workshop.station.ev3_live_client import EV3LiveClient
from workshop.station.ros_workflow import StationWorkflow
from workshop.station.locking import station_lock, state_dir


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from so101_station_workflow.srv import RunCycles, ConfirmReference, SelectDestination, EV3Command
    from tools.ros_sort_station import ROSSortingDevices, load_settings
    from workshop.station.ros_sorting import run_sorting

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    shutdown = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: shutdown.set())
    signal.signal(signal.SIGTERM, lambda *_: shutdown.set())
    node = Node('station_workflow')
    for key, value in {'sorting_config': '', 'evidence_dir': str(Path.home()/'.local/state/exciting-science/sorting'),
                       'ev3_host': '127.0.0.1', 'ev3_port': 18766}.items():
        node.declare_parameter(key, value)
    get = lambda key: node.get_parameter(key).value
    api_lock = station_lock(state_dir()/'ros-workflow.lock')
    api_lock.__enter__()
    workflow = StationWorkflow()
    ev3 = EV3LiveClient(host=str(get('ev3_host')), port=int(get('ev3_port')), timeout=.75)
    cache = {'connected': False, 'error': 'EV3 not connected; call /station/ev3/connect'}
    destinations = {}
    stop_state = {'pending': False, 'ev3_stopped': None, 'arm_disarmed': None}
    follower_stop = node.create_client(Trigger, '/follower/stop')
    evidence = Path(str(get('evidence_dir')))
    evidence.mkdir(parents=True, exist_ok=True)
    publisher = node.create_publisher(String, '/station/status', 10)

    record_lock = threading.Lock()
    def record(event):
        workflow.record(event)
        with record_lock:
            with (evidence/'station-events.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event, job_id=workflow.job_id), allow_nan=False)+'\n')

    def idle():
        snap = workflow.snapshot()
        if workflow.thread and workflow.thread.is_alive():
            raise RuntimeError('station job is active')
        return snap

    def reply(response, function):
        try:
            value = function()
            response.success, response.message = True, json.dumps(value, allow_nan=False)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def connect():
        nonlocal ev3
        idle()
        if not ev3.connected:
            # Explicit reconnect only: transport failures have unknown delivery.
            ev3.close()
            ev3 = EV3LiveClient(host=str(get('ev3_host')), port=int(get('ev3_port')), timeout=.75)
            ev3.connect()
        cache.update(connected=True, error=None, status=ev3.call('status'))
        return cache

    def refresh():
        if not ev3.connected:
            cache.update(connected=False, error=str(ev3.failure or 'disconnected'))
        if ev3.connected:
            try:
                cache.update(connected=True, error=None, status=ev3.call('status'))
            except Exception as error:
                cache.update(connected=False, error=str(error))
        message = String()
        message.data = json.dumps(dict(workflow=workflow.snapshot(), ev3=cache,
                                       selected_destinations=destinations, stop=stop_state), allow_nan=False)
        publisher.publish(message)

    def ev3_command(request, response):
        try:
            operation = request.operation
            active = workflow.thread and workflow.thread.is_alive()
            if active and request.job_id != workflow.job_id and operation != 'stop':
                raise RuntimeError('EV3 reserved by active station job')
            if operation not in {'status', 'stop', 'eject', 'home_touch', 'wake', 'power_save', 'confirm_reference'}:
                raise ValueError('operation is not admitted by ROS station API')
            if operation not in {'status', 'stop'} and (not active or request.job_id != workflow.job_id):
                raise RuntimeError('EV3 motion/power command requires admitted station job')
            if operation == 'stop' and (not active or request.job_id != workflow.job_id):
                raise RuntimeError('external stop must use /station/stop for arm/EV3 cancellation')
            arguments = json.loads(request.arguments_json or '{}')
            if not isinstance(arguments, dict):
                raise ValueError('arguments must be an object')
            # Reference confirmation must pass the explicit operator service.
            if operation == 'confirm_reference':
                raise ValueError('use /station/ev3/confirm_reference with physical acknowledgement')
            result = ev3.call(operation, arguments)
            response.success, response.result_json = True, json.dumps(result, allow_nan=False)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response
    node.create_service(EV3Command, '/station/ev3/command', ev3_command)

    class Proxy:
        def __init__(self, worker, cancel):
            self.node, self.cancel = worker, cancel
            self.client = worker.create_client(EV3Command, '/station/ev3/command')
            self.session = ev3.session
        def call(self, operation, arguments=None):
            if not self.client.wait_for_service(timeout_sec=1.):
                raise RuntimeError('resident EV3 ROS bridge unavailable')
            request = EV3Command.Request(operation=operation, arguments_json=json.dumps(arguments or {}),
                                         job_id=workflow.job_id or '')
            future = self.client.call_async(request)
            deadline = time.monotonic()+3.
            while not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError('EV3 ROS command delivery unknown; no retry')
                self.node._station_executor.spin_once(timeout_sec=.02)
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            return json.loads(response.result_json)

    from types import SimpleNamespace
    worker_ros = SimpleNamespace(ok=rclpy.ok,
        spin_once=lambda node, timeout_sec: node._station_executor.spin_once(timeout_sec=timeout_sec))

    def worker_task(operation, function):
        if stop_state['pending'] and operation != 'check_ready':
            raise RuntimeError('stop/disarm terminal acknowledgements pending')
        def work(cancel):
            from rclpy.executors import SingleThreadedExecutor
            worker = Node('station_job_'+workflow.job_id.replace('-', '')[:12])
            worker._station_executor = SingleThreadedExecutor()
            worker._station_executor.add_node(worker)
            try:
                return function(worker, Proxy(worker, cancel), cancel)
            finally:
                worker._station_executor.remove_node(worker)
                worker._station_executor.shutdown()
                worker.destroy_node()
        return {'job_id': workflow.start(operation, work)}

    def run(request, response):
        def admit():
            if not 1 <= request.cycles <= 100:
                raise ValueError('cycles must be 1..100')
            if stop_state['pending']:
                raise RuntimeError('stop/disarm acknowledgements pending')
            settings = load_settings(str(get('sorting_config')))
            settings['placement_selections'] = dict(destinations)
            if not ev3.connected:
                raise RuntimeError('EV3 bridge disconnected')
            def sorting(worker, proxy, cancel):
                output = evidence/workflow.job_id
                output.mkdir(exist_ok=False)
                devices = ROSSortingDevices(settings, proxy, output, node=worker, rclpy=worker_ros)
                try:
                    with (output/'cycles.jsonl').open('x', buffering=1) as stream:
                        def log(event):
                            record(event)
                            stream.write(json.dumps(event, allow_nan=False)+'\n')
                        result = run_sorting(devices, destinations=settings['destinations'],
                                             cycles=request.cycles, log=log, cancel=cancel)
                        (output/'result.json').write_text(json.dumps(result, allow_nan=False)+'\n')
                        return result
                except BaseException as error:
                    if getattr(error, '__notes__', None):
                        stop_state['pending'] = True
                    raise
            return worker_task('sorting', sorting)
        return reply(response, admit)
    node.create_service(RunCycles, '/station/run_cycles', run)

    def stop():
        workflow.stop()
        import uuid
        stop_request = str(uuid.uuid4())
        stop_state.update(pending=True, ev3_stopped=False, arm_disarmed=False, request_id=stop_request)
        # Immediate EV3 braking; cycle worker independently cancels arm actions.
        errors = []
        try:
            acknowledgement = ev3.call('stop')
            stop_state['ev3_stopped'] = acknowledgement.get('stopped') is True
            if not stop_state['ev3_stopped']:
                errors.append('EV3 stop acknowledgement missing')
        except Exception as error:
            errors.append(str(error))
        if follower_stop.service_is_ready():
            future = follower_stop.call_async(Trigger.Request())
            def stopped(future):
                if stop_state.get('request_id') != stop_request:
                    return
                try:
                    response = future.result()
                    stop_state['arm_disarmed'] = response.success
                    stop_state['pending'] = not (response.success and stop_state['ev3_stopped'])
                    record({'event': 'arm_stop_ack', 'success': response.success, 'message': response.message})
                except Exception as error:
                    record({'event': 'arm_stop_ack', 'success': False, 'message': str(error)})
            future.add_done_callback(stopped)
        else:
            errors.append('resident /follower/stop unavailable')
        if errors:
            raise RuntimeError('; '.join(errors))
        return {'cancel_requested': True, 'arm_stop_ack_pending': True, 'job_id': workflow.job_id}

    def simple_ev3(operation):
        def work(worker, proxy, cancel):
            result = proxy.call(operation)
            job = result.get('job', {})
            if result.get('accepted'):
                deadline = time.monotonic()+20.
                while True:
                    if cancel.is_set() or time.monotonic() >= deadline:
                        proxy.call('stop')
                        raise InterruptedError('EV3 job cancelled or timed out')
                    status = proxy.call('status')
                    if status.get('session') != proxy.session:
                        raise RuntimeError('EV3 session changed')
                    if status.get('busy') is False:
                        last = status.get('last_job') or {}
                        if last.get('id') != job.get('id') or last.get('state') != 'complete' or last.get('cleanup_fault') or status.get('last_error'):
                            raise RuntimeError('EV3 job failed: '+str(status))
                        return status
                    time.sleep(.05)
            return result
        return worker_task(operation, work)

    def confirm(request, response):
        def action():
            idle()
            if request.physically_verified is not True:
                raise ValueError('operator must physically verify the requested reference')
            if (request.name, request.reference) not in {('sorter', 'home'), ('feeder', 'eject')}:
                raise ValueError('allowed references: sorter/home, feeder/eject')
            result = ev3.call('confirm_reference', {'name': request.name, 'reference': request.reference, 'angle': 0})
            record({'event': 'operator_reference_confirmation', 'name': request.name, 'reference': request.reference})
            return result
        return reply(response, action)
    node.create_service(ConfirmReference, '/station/ev3/confirm_reference', confirm)

    def select(request, response):
        def action():
            import uuid
            idle()
            if request.color not in {'red', 'green', 'blue', 'yellow'}:
                raise ValueError('unknown color')
            uuid.UUID(request.selection_id)
            destinations[request.color] = request.selection_id
            return dict(destinations)
        return reply(response, action)
    node.create_service(SelectDestination, '/station/select_destination', select)

    def arm_home():
        def home(worker, proxy, cancel):
            client = worker.create_client(Trigger, '/follower/home')
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('resident qualified home/recovery service unavailable')
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+95.
            while not future.done():
                if cancel.is_set() or time.monotonic() >= deadline:
                    # /follower/home owns its cancellation; never pretend it stopped.
                    raise RuntimeError('home cancellation/timeout requires resident stop acknowledgement')
                worker._station_executor.spin_once(timeout_sec=.05)
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
            return {'home': result.message}
        return worker_task('arm_home', home)

    def reset_fault():
        if stop_state['pending']:
            raise RuntimeError('stop/disarm terminal acknowledgements missing; fault cannot reset')
        workflow.reset()
        return {'fault_reset': True}

    def diagnostics():
        gates = []
        try:
            load_settings(str(get('sorting_config')))
        except Exception as error:
            gates.append(str(error))
        if not ev3.connected:
            gates.append('EV3 ROS bridge disconnected; connect explicitly')
        references = cache.get('status', {}).get('references', {})
        for name in ('sorter', 'feeder'):
            if not references.get(name):
                gates.append('EV3 reference missing: '+name)
        if not follower_stop.service_is_ready():
            gates.append('resident /follower/stop unavailable')
        return dict(gates=gates, qualified_for_motion=False,
                    note='Read-only transport/config diagnostics; check_ready runs all admission checks')

    def check_ready():
        settings = load_settings(str(get('sorting_config')))
        def check(worker, proxy, cancel):
            devices = ROSSortingDevices(settings, proxy, evidence, node=worker, rclpy=worker_ros)
            devices.require_ready(time.monotonic()+8., cancel)
            return {'ready': True}
        return worker_task('check_ready', check)

    for name, function in {'diagnostics': diagnostics, 'check_ready': check_ready, 'status': lambda: dict(workflow=workflow.snapshot(), ev3=cache, stop=stop_state),
                           'cancel': stop, 'stop': stop, 'reset_fault': reset_fault,
                           'ev3/connect': connect, 'ev3/home': lambda: simple_ev3('home_touch'),
                           'wake': lambda: simple_ev3('wake'), 'sleep': lambda: simple_ev3('power_save'),
                           'arm/home': arm_home}.items():
        node.create_service(Trigger, '/station/'+name, lambda request, response, fn=function: reply(response, fn))
    node.create_timer(.5, refresh)
    try:
        while rclpy.ok() and not shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        try:
            if ((workflow.thread and workflow.thread.is_alive())
                    or not stop_state['ev3_stopped'] or not stop_state['arm_disarmed']):
                stop()
            else:
                workflow.stop()
        except Exception as error:
            node.get_logger().error('shutdown stop/disarm incomplete: '+str(error))
        # Keep ROS alive for producer cancellation and terminal disarm ack.
        until = time.monotonic()+12.
        while ((workflow.thread and workflow.thread.is_alive()) or stop_state['pending']) and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if stop_state['pending']:
            node.get_logger().error('shutdown lacks terminal EV3/arm stop acknowledgement')
        ev3.close()
        node.destroy_node()
        rclpy.shutdown()
        api_lock.__exit__(None, None, None)
