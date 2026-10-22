"""Correlated native EV3 job completion; no fault reset or reference changes."""
import time


def _fault(value):
    if value is None:
        return None
    if not isinstance(value,dict) or not isinstance(value.get('code'),str) or not isinstance(value.get('message'),str):
        raise RuntimeError('EV3 fault report is malformed; keep stopped and inspect the station')
    return {'code':value['code'][:80],'message':value['message'][:240]}


def _terminal(status,session,job_id):
    if not isinstance(status,dict) or type(status.get('busy')) is not bool:
        raise RuntimeError('EV3 status requires boolean busy; inspect the station before retry')
    if status.get('session')!=session:
        raise RuntimeError('EV3 session changed; inspect and reconnect explicitly before retry')
    job=status.get('job' if status['busy'] else 'last_job')
    if not isinstance(job,dict) or type(job.get('id')) is not int or job['id']!=job_id:
        raise RuntimeError('EV3 native job identity is missing or changed; keep stopped and inspect before retry')
    cleanup=_fault(job.get('cleanup_fault'))
    if cleanup is not None:
        raise RuntimeError('EV3 cleanup failed ['+cleanup['code']+']: '+cleanup['message']+'; keep stopped and inspect brakes')
    faults=[_fault(value) for value in (job.get('fault'),status.get('last_error')) if value is not None]
    real=next((fault for fault in faults if fault['code']!='stopped'),None)
    if real is not None:
        raise RuntimeError('EV3 native fault ['+real['code']+']: '+real['message']+'; inspect and reset the inspected fault before retry')
    if not status['busy'] and job.get('state')=='aborted' and faults:
        raise InterruptedError('EV3 job interrupted by station STOP; inspect and reset the inspected fault before retry')
    if faults:
        raise RuntimeError('EV3 stopped report lacks a matching aborted terminal job; keep stopped and inspect')
    if status['busy']:
        if job.get('state')!='running':
            raise RuntimeError('EV3 active job state is unknown; keep stopped and inspect')
        return False
    if job.get('state')!='complete':
        raise RuntimeError('EV3 terminal job did not complete; inspect and reset the inspected fault before retry')
    return True


def run_ev3_operation(proxy,cancel,operation,record,stop_state,*,clock=time.monotonic,sleep=time.sleep):
    """Run once, classify fresh correlated status before post-RPC cancellation."""
    session=proxy.session;job_id=None;latest=None
    def evidence(event,error=None):
        observed=latest if isinstance(latest,dict) else {}
        job=observed.get('job' if observed.get('busy') is True else 'last_job') or observed.get('job')
        job=job if isinstance(job,dict) else {}
        def bounded_fault(value):
            try:return _fault(value)
            except RuntimeError:return {'code':'malformed','message':'Malformed native fault report'}
        observed_session=observed.get('session')
        faults=[bounded_fault(value) for value in (job.get('fault'),observed.get('last_error')) if value is not None]
        native_fault=next((fault for fault in faults if fault['code']!='stopped'),faults[0] if faults else None)
        request_id=stop_state.get('request_id')
        return dict(event=event,operation=operation,expected_native_job_id=job_id,
                    observed_native_job_id=job.get('id') if type(job.get('id')) is int else None,
                    expected_session=session[:128] if isinstance(session,str) else None,
                    observed_session=observed_session[:128] if isinstance(observed_session,str) else None,
                    native_state=str(job.get('state'))[:40],
                    native_fault=native_fault,
                    cleanup_fault=bounded_fault(job.get('cleanup_fault')),
                    time_s=time.time(),monotonic_s=clock(),
                    stop_request_id=request_id[:128] if isinstance(request_id,str) else None,
                    error=str(error)[:500] if error is not None else None)
    def check_before_dispatch():
        if cancel.is_set():raise InterruptedError('EV3 operation cancelled before dispatch')
    try:
        if not isinstance(session,str) or not session:
            raise RuntimeError('EV3 session unavailable; reconnect explicitly before retry')
        check_before_dispatch()
        if operation in ('eject','home_touch'):
            proxy.call('wake');check_before_dispatch()
        if operation=='eject':
            from workshop.station.ev3_readiness import require_fresh_stationary
            require_fresh_stationary(proxy,session=session,deadline=clock()+2.,cancel=cancel,
                                     spin=lambda deadline,cancel:sleep(.05))
            check_before_dispatch()
        result=proxy.call(operation);latest=result
        if not isinstance(result,dict):
            raise RuntimeError('EV3 admission response is malformed; inspect the station before retry')
        if 'accepted' not in result:
            check_before_dispatch();record(evidence('ev3_job_admitted'))
            record(evidence('ev3_job_terminal'));return result
        job=result.get('job')
        if result['accepted'] is not True or not isinstance(job,dict) or type(job.get('id')) is not int or job['id']<1:
            raise RuntimeError('EV3 admission lacks a unique accepted native job; inspect before retry')
        job_id=job['id'];record(evidence('ev3_job_admitted'));deadline=clock()+20.
        while True:
            if cancel.is_set() or clock()>=deadline:
                stopped=proxy.call('stop')
                if not isinstance(stopped,dict) or stopped.get('stopped') is not True:
                    raise RuntimeError('EV3 stop acknowledgement missing; keep stopped and inspect brakes')
                if cancel.is_set():raise InterruptedError('EV3 job cancelled before status RPC')
                raise TimeoutError('EV3 job timed out before status RPC')
            latest=proxy.call('status')
            # Native faults, cleanup and identity failures take precedence over
            # cancellation that arrived while this status RPC was blocked.
            complete=_terminal(latest,session,job_id)
            if cancel.is_set() or clock()>=deadline:
                if not complete:
                    stopped=proxy.call('stop')
                    if not isinstance(stopped,dict) or stopped.get('stopped') is not True:
                        raise RuntimeError('EV3 stop acknowledgement missing; keep stopped and inspect brakes')
                if cancel.is_set():raise InterruptedError('EV3 job cancelled after status RPC')
                raise TimeoutError('EV3 job timed out after status RPC')
            if complete:
                record(evidence('ev3_job_terminal'));return latest
            sleep(.05)
    except BaseException as error:
        try:record(evidence('ev3_job_failed',error))
        except Exception as logging_error:error.add_note('EV3 failure evidence write failed: '+str(logging_error)[:240])
        raise
