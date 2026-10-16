"""One bounded ROS preparation call, with explicit delivery/stop semantics."""
import json
import math
import time


def prepare_arm_once(worker, cancel, stop_state, record, *, timeout_s=110.,
                     trigger_type=None, clock=time.monotonic):
    """Used inside the current station job; never starts a job or calls EV3."""
    if trigger_type is None:
        from std_srvs.srv import Trigger
        trigger_type = Trigger
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
        raise ValueError('arm preparation timeout must be in (0, 120] seconds')
    deadline = clock()+timeout_s
    client = None
    future = None
    delivery_possible = False
    stop_attempted = False
    stop_confirmed = False
    failure = None

    def spin():
        worker._station_executor.spin_once(timeout_sec=.02)

    def check():
        if cancel.is_set() or stop_state.get('pending'):
            raise InterruptedError('arm preparation cancelled or station stop pending')
        if clock() >= deadline:
            raise TimeoutError('arm preparation result unknown; no retry')

    def wait_existing_stop(request_id, until):
        while clock() < until:
            if stop_state.get('request_id') != request_id:
                raise RuntimeError('station stop generation changed; disarm remains unresolved')
            if stop_state.get('arm_disarmed') is True:
                return
            spin()
        raise TimeoutError('existing station arm stop acknowledgement unresolved')

    def verified_stop():
        nonlocal stop_confirmed
        until = clock()+10.
        request_id = stop_state.get('request_id') if stop_state.get('pending') else None
        if request_id is not None:
            wait_existing_stop(request_id, until)
            stop_confirmed = True
            return
        dispatched_station_id = stop_state.get('request_id')
        stop = worker.create_client(trigger_type, '/follower/stop')
        try:
            discovery = min(until, clock()+3.)
            while not stop.wait_for_service(timeout_sec=.05):
                if clock() >= discovery:
                    raise RuntimeError('resident arm stop unavailable; disarm unresolved')
            stopped = stop.call_async(trigger_type.Request())
            while not stopped.done():
                if clock() >= until:
                    raise TimeoutError('resident arm stop acknowledgement unresolved')
                spin()
            result = stopped.result()
            current_station_id = stop_state.get('request_id')
            if current_station_id != dispatched_station_id:
                # This ACK belongs to our private request, never the newer
                # station stop. Its own correlated callback must verify it.
                if current_station_id is None:
                    raise RuntimeError('station stop correlation changed; disarm unresolved')
                wait_existing_stop(current_station_id, until)
                stop_confirmed = True
                return
            if result is None or result.success is not True:
                # A concurrent station stop may have acquired the bridge lock.
                request_id = stop_state.get('request_id') if stop_state.get('pending') else None
                if request_id is not None:
                    wait_existing_stop(request_id, until)
                else:
                    raise RuntimeError('resident arm stop did not verify quiescence: '+
                                       (result.message if result is not None else 'no result'))
            # Only a station stop's correlated callback owns shared ACK state.
            stop_confirmed = True
        finally:
            worker.destroy_client(stop)

    def ensure_stop(error):
        """Logging/disposal must not decide whether physical cleanup runs."""
        nonlocal stop_attempted
        if not delivery_possible or stop_attempted:
            return
        stop_attempted = True
        def best_effort_record(event):
            try:
                record(dict(event=event, monotonic=clock()))
            except BaseException as logging_error:
                error.add_note('cleanup event logging failed: '+str(logging_error))
        best_effort_record('arm_prepare_stop_requested')
        try:
            verified_stop()
        except BaseException as stop_error:
            if not stop_confirmed:
                stop_state.update(pending=True, arm_disarmed=False)
                error.add_note('arm stop/quiescence unresolved: '+str(stop_error))
            else:
                error.add_note('arm stop verified; client disposal failed: '+str(stop_error))
        if stop_confirmed:
            # Internal exception metadata, never accepted from bridge JSON.
            error.arm_prepare_stop_verified = True
            best_effort_record('arm_prepare_stop_verified')

    try:
        check()  # no client call, no stop, if locally rejected before dispatch
        client = worker.create_client(trigger_type, '/follower/prepare')
        discovery = min(deadline, clock()+3.)
        while not client.wait_for_service(timeout_sec=.05):
            check()
            if clock() >= discovery:
                raise RuntimeError('resident arm preparation service unavailable; request not sent')
        check()
        record(dict(event='arm_prepare_request', monotonic=clock()))
        delivery_possible = True
        future = client.call_async(trigger_type.Request())
        while not future.done():
            check()
            spin()
        result = future.result()
        if result is None:
            raise RuntimeError('arm preparation returned no result; delivery unknown')
        try:
            value = json.loads(result.message)
        except (ValueError, TypeError):
            raise RuntimeError('arm preparation returned malformed terminal evidence')
        if (not isinstance(value, dict) or value.get('schema') != 1 or
                value.get('kind') != 'arm_prepare'):
            raise RuntimeError('arm preparation terminal schema mismatch')
        if result.success is not True:
            # Trigger.success=false alone says nothing about physical delivery.
            if (value.get('state') == 'rejected' and
                    value.get('command_delivery') == 'not_sent' and
                    value.get('phases') == [] and value.get('stop_verified') is None and
                    value.get('home_completed_this_session') is False):
                delivery_possible = False
            raise RuntimeError(value.get('error', 'resident arm preparation failed'))
        check()
        stamp = value.get('owner_started_monotonic')
        if (value.get('state') not in {'ready', 'already_ready'} or
                value.get('home_completed_this_session') is not True or
                type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0):
            raise RuntimeError('arm preparation did not prove measured home/readiness')
        record(dict(event='arm_prepare_complete', monotonic=clock(), result=value))
        return value
    except BaseException as error:
        failure = error
        ensure_stop(error)
        raise
    finally:
        if client is not None:
            try:
                try:
                    if future is not None and not future.done():
                        try:
                            client.remove_pending_request(future)
                        finally:
                            future.cancel()
                finally:
                    worker.destroy_client(client)
            except BaseException as cleanup_error:
                if failure is not None:
                    failure.add_note('prepare client disposal failed: '+str(cleanup_error))
                else:
                    ensure_stop(cleanup_error)
                    raise
