"""Existing conservative sorter phase shared by typed targets and endpoint routes."""
import time
from workshop.station.sorter_position import (validate_target, stationary_snapshot,
    position_arguments, PHASE_DEADLINE_S, motion_progress, terminal_position,
    control_diagnostics, critical_status_read)


def move_phase(proxy, cancel, target, record, stop_state, *, overall_deadline=float('inf'), step_limit=None):
    validate_target(target)
    latest_observation = None
    def check(deadline):
        if cancel.is_set():
            raise InterruptedError('sorter position cancelled')
        if time.monotonic() >= deadline:
            raise TimeoutError('sorter position deadline exceeded')
    slow_reads = [0]
    latest_rpc_timing = None
    def observe(result, timing):
        nonlocal latest_observation, latest_rpc_timing
        latest_observation, latest_rpc_timing = result, timing
        record(dict(event='sorter_critical_observation', timing=timing,
            session=result.get('session'), busy=result.get('busy'),
            sampled_ms=result.get('sampled_ms'),
            encoder_sample_ms={name: control.get('angle_sample_ms')
                for name, control in result.get('motor_control', {}).items()},
            speed_sample_ms={name: control.get('speed_sample', {}).get('sampled_ms')
                for name, control in result.get('motor_control', {}).items()}))
    def status(deadline, stage):
        return critical_status_read(proxy, deadline=deadline, cancel=cancel,
            stage=stage, slow_reads=slow_reads, record=record,
            observe=observe, clock=time.monotonic)
    try:
        admission_deadline = min(overall_deadline, time.monotonic()+2.)
        first = status(admission_deadline, 'admission')
        initial_angle, initial_stamp = stationary_snapshot(first, session=proxy.session)
        while True:
            time.sleep(.05)
            current = status(admission_deadline, 'admission')
            angle, stamp = stationary_snapshot(current, session=proxy.session)
            if stamp > initial_stamp:
                if stamp-initial_stamp > 1500 or abs(angle-initial_angle) > 1.:
                    raise ValueError('sorter telemetry changed unexpectedly during admission')
                break
        if step_limit is not None:
            remaining = target-angle
            if abs(remaining) > step_limit:
                target = angle+(step_limit if remaining > 0 else -step_limit)
        arguments = position_arguments(current, session=proxy.session, target=target)
        check(admission_deadline)
        # The one native target phase also has its own six-second watchdog.
        deadline = min(overall_deadline, time.monotonic()+PHASE_DEADLINE_S)
        accepted = proxy.call('motor_target', arguments)
        job = accepted.get('job', {})
        if accepted.get('accepted') is not True or type(job.get('id')) is not int:
            raise RuntimeError('sorter target did not return a unique native job')
        diagnostics = control_diagnostics(current)
        record(dict(event='sorter_position_started', target_deg=target,
                    native_job_id=job['id'], profile=arguments,
                    diagnostics=diagnostics, native_job=job, command_timing=proxy.last_call_timing))
        last_sample, last_progress = stamp, time.monotonic()
        while True:
            observed = status(deadline, 'motion_or_terminal')
            last_sample, last_progress = motion_progress(observed, session=proxy.session,
                previous_stamp=last_sample, progress_time=last_progress, now=time.monotonic())
            if observed.get('busy') is False:
                final_angle = terminal_position(observed, session=proxy.session,
                    job_id=job['id'], target=target, after_stamp=stamp)
                result = dict(target_deg=target, measured_deg=final_angle,
                    session=proxy.session, native_job_id=job['id'],
                    diagnostics_before=diagnostics, diagnostics_after=control_diagnostics(observed),
                    native_job=observed['last_job'], angles=observed['angles'])
                record(dict(event='sorter_position_complete', **result))
                return result
            active = observed.get('job') or {}
            if active.get('id') != job['id'] or active.get('operation') != 'motor_target':
                raise RuntimeError('sorter native active job identity changed')
            time.sleep(.05)
    except BaseException as error:
        # Preserve the failing native state before cleanup changes
        # power mode, active-job state, or cached diagnostics.
        try:
            record(dict(event='sorter_position_failed', target_deg=target,
                        error=str(error), native_observation=latest_observation, rpc_timing=latest_rpc_timing))
        except Exception as logging_error:
            error.add_note('sorter failure evidence write failed: '+str(logging_error))
        try:
            reply = proxy.call('stop')
            if reply.get('stopped') is not True:
                raise RuntimeError('EV3 stop acknowledgement missing')
        except BaseException as stop_error:
            error.add_note('sorter cleanup stop failed: '+str(stop_error))
            stop_state['pending'] = True
        raise


def move_endpoint(proxy, cancel, target, record, stop_state):
    """At most three <=200 degree guarded phases under one workflow owner/job."""
    validate_target(target)
    deadline = time.monotonic()+29.
    phases = []
    try:
        if cancel.is_set():
            raise InterruptedError('conveyor endpoint cancelled')
        for index in range(3):
            if cancel.is_set():
                raise InterruptedError('conveyor endpoint cancelled between phases')
            if time.monotonic() >= deadline:
                raise TimeoutError('conveyor endpoint overall deadline exceeded')
            result = move_phase(proxy, cancel, target, record, stop_state,
                                overall_deadline=deadline, step_limit=200.)
            phases.append(result)
            angle = result['measured_deg']
            if result['target_deg'] == target:
                return dict(target_deg=target, measured_deg=angle, session=proxy.session,
                            phases=phases, phase_count=len(phases))
        raise RuntimeError('conveyor endpoint exceeded three guarded phases')
    except BaseException as error:
        try:
            reply = proxy.call('stop')
            if reply.get('stopped') is not True:
                raise RuntimeError('EV3 stop acknowledgement missing')
        except BaseException as stop_error:
            error.add_note('endpoint cleanup stop failed: '+str(stop_error))
            stop_state['pending'] = True
        raise
