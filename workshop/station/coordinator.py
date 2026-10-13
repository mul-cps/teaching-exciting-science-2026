"""Serialized one-piece workflows. Backends must implement bounded acknowledgements."""
import time
import threading
import uuid

from .config import COLORS, validate_config


def _emit(log, event, **fields):
    record = {'timestamp': time.time(), 'event': event, **fields}
    if callable(log):
        log(record)
    else:
        log.append(record)


_CYCLE_LOCK = threading.Lock()


def run_cycle(config, ev3, arm, log):
    """Execute one cycle with exclusive ownership of station actuation."""
    validate_config(config)
    if not _CYCLE_LOCK.acquire(timeout=config['timeouts']['operation_s']):
        raise TimeoutError('another station cycle still owns actuation')
    try:
        return _run_cycle(config, ev3, arm, log)
    finally:
        _CYCLE_LOCK.release()


def _run_cycle(config, ev3, arm, log):
    """Sort one piece; on any fault independently stop both devices, never retry.

    A method returning normally acknowledges completion (False explicitly rejects
    it). Real backends enforce the configured deadlines at the transport/motion
    layer; the coordinator also rejects a late return before the next operation.
    """
    def operation(device, name, *args):
        timeout = config['timeouts']['stop_s' if name in ('stop', 'wait_stopped') else 'operation_s']
        operation_id = uuid.uuid4().hex
        deadline = time.monotonic() + timeout
        _emit(log, 'operation_start', operation_id=operation_id, device=device, operation=name, deadline=deadline)
        result = getattr(ev3 if device == 'ev3' else arm, name)(*args)
        if result is False:
            raise RuntimeError(f'{device}.{name} acknowledgement failed')
        if time.monotonic() > deadline:
            raise TimeoutError(f'{device}.{name} exceeded deadline')
        _emit(log, 'operation_complete', operation_id=operation_id, device=device, operation=name)
        return result

    try:
        validate_config(config)
        workflow = config['workflow']
        _emit(log, 'cycle_start', workflow=workflow)
        if workflow == 'arm_loads_sorter':
            operation('ev3', 'stop')
            operation('ev3', 'wait_stopped')
            operation('arm', 'load_piece')
            color = operation('ev3', 'read_color')
            if color not in COLORS:
                raise ValueError(f'unsupported or unstable color: {color!r}')
            operation('arm', 'clear')
            operation('arm', 'wait_clear')
            operation('ev3', 'position', color)
            operation('ev3', 'feed')
            operation('ev3', 'stop')
            operation('ev3', 'wait_stopped')
        else:
            operation('arm', 'clear')
            operation('arm', 'wait_clear')
            color = operation('ev3', 'read_color')
            if color not in COLORS:
                raise ValueError(f'unsupported or unstable color: {color!r}')
            operation('ev3', 'position', color)
            operation('ev3', 'feed')
            operation('ev3', 'stop')
            operation('ev3', 'wait_stopped')
            operation('arm', 'pick_and_place', color)
            operation('arm', 'clear')
            operation('arm', 'wait_clear')
        _emit(log, 'cycle_complete', workflow=workflow, color=color)
        return color
    except BaseException as error:
        # Includes operator cancellation. Each device gets its own cleanup attempt.
        cleanup_failures = []
        for name, device in (('ev3', ev3), ('arm', arm)):
            try:
                device.stop()
            except BaseException as cleanup_error:
                cleanup_failures.append((name, cleanup_error))
        # Finish both physical shutdown attempts before calling fallible logging.
        for name, cleanup_error in cleanup_failures:
            try:
                _emit(log, 'cleanup_failure', device=name, error=str(cleanup_error))
            except BaseException:
                pass  # Preserve the originating fault even if observability fails.
        try:
            _emit(log, 'fault', error=str(error))
        except BaseException:
            pass
        raise
