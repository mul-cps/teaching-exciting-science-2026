"""Bounded ROS program commands sharing the HTTP adapter's Application."""
import json
import threading

SERVICE = '/station/program/command'
MAX_JSON_BYTES = 65536
FIELDS = {'status': (), 'run': ('program', 'admission'), 'record_start': ('name', 'admission'),
          'record_stop': ('admission',), 'recordings': (), 'stop': (), 'recover': ('admission',),
          'operator': ('command', 'admission')}


class ROSProgramAPI:
    def __init__(self, app, strict_json):
        # Import only on explicit ROS startup; offline HTTP remains ROS independent.
        import rclpy
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from so101_station_workflow.srv import StationCommand
        self.app = app
        self.strict_json = strict_json
        self.ros = rclpy
        self.node = self.executor = self.thread = None
        self.owned_context = not app.backend.physical
        self.context = None
        self.closed = False
        self.state_lock = threading.Lock()
        self.active = 0
        self.closing = threading.Event()
        # Reserve executor capacity for STOP when a slow control is waiting.
        self.slots = threading.BoundedSemaphore(2)
        try:
            if self.owned_context:
                from rclpy.context import Context
                self.context = Context()
                rclpy.init(context=self.context)
            else:
                self.context = app.backend.node.context
            self.node = Node('blockly_station_program_api', context=self.context)
            self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
            self.node.create_service(StationCommand, SERVICE, self.command,
                                     callback_group=ReentrantCallbackGroup())
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._spin, daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self):
        from rclpy.executors import ExternalShutdownException, ShutdownException
        try:
            self.executor.spin()
        except (ExternalShutdownException, ShutdownException):
            pass

    def dispatch(self, operation, body):
        if self.closed:
            raise RuntimeError('program API is closed')
        if not isinstance(operation, str) or operation not in FIELDS:
            raise ValueError('unsupported operation')
        if not isinstance(body, str) or not 0 < len(body.encode('utf-8')) <= MAX_JSON_BYTES:
            raise ValueError('arguments_json must be 1..65536 UTF-8 bytes')
        args = self.strict_json(body)
        if not isinstance(args, dict) or set(args) != set(FIELDS[operation]):
            raise ValueError('unsupported argument fields')
        if operation in {'record_start', 'operator'}:
            key = FIELDS[operation][0]
            if not isinstance(args[key], str):
                raise ValueError(key + ' must be a string')
        admitted = operation == 'stop' or self.slots.acquire(blocking=False)
        if not admitted:
            raise RuntimeError('program API is busy')
        started = False
        try:
            with self.state_lock:
                if self.closed:
                    raise RuntimeError('program API is closed')
                self.active += 1
                started = True
            if operation == 'recordings':
                with self.app.lock:
                    result = {'names': sorted(self.app.recordings)}
            elif operation == 'run':
                result = self.app.run_external(args['program'],args['admission'])
            elif operation in {'record_start','record_stop','operator','recover'}:
                result = self.app.control_external(operation,{key:value for key,value in args.items() if key!='admission'},args['admission'],cancel=self.closing)
            else:
                result = getattr(self.app, operation)()
            encoded = json.dumps(result, allow_nan=False, separators=(',', ':'))
            if len(encoded.encode('utf-8')) > MAX_JSON_BYTES:
                raise ValueError('result_json exceeds 65536 UTF-8 bytes')
            return encoded
        finally:
            if started:
                with self.state_lock:
                    self.active -= 1
            if operation != 'stop':
                self.slots.release()

    def command(self, request, response):
        try:
            response.result_json = self.dispatch(request.operation, request.arguments_json)
            response.success = True
            response.message = ''
        except Exception as error:
            response.success = False
            response.result_json = '{}'
            response.message = str(error)[:2000]
        return response

    def close(self):
        with self.state_lock:
            self.closed = True
            self.closing.set()
            active = self.active
        stop_error = None
        if active:
            try:
                self.app.stop()
            except Exception as error:
                stop_error = error
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2, wait_for_threads=False)
        if self.thread is not None:
            self.thread.join(timeout=2)
        with self.state_lock:
            active = self.active
        if stop_error is not None or active or (self.thread is not None and self.thread.is_alive()):
            raise RuntimeError('program API shutdown incomplete; retain application ownership and inspect STOP/callbacks'
                               + (': ' + str(stop_error)[:1000] if stop_error else ''))
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.owned_context and self.context is not None and self.context.ok():
            self.ros.shutdown(context=self.context)
