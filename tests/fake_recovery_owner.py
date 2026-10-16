"""Hardware-free child: production owner/factory/oracle with synthetic bus/depth.

Only native ROS subprocess integration fixtures should invoke this helper.
No hardware module is opened. Fixtures are explicitly synthetic/qualified only
inside their temporary directory, never copied into station configuration.
"""
import argparse
import json
from pathlib import Path
import threading
import time

from test_recovery_runtime import fixture, write_frame, write_arm_frame
from workshop.station.arm_session import ArmSession, run_locked_session


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-root',type=Path,required=True)
    parser.add_argument('--state-file',type=Path,required=True)
    parser.add_argument('--logdir',type=Path,required=True)
    parser.add_argument('--lock',type=Path,required=True)
    parser.add_argument('--external-depth',action='store_true',
                        help='serve immediately; bridge ROS collector is the only depth producer')
    parser.add_argument('--wait-fault',choices=('drift','torque'))
    parser.add_argument('--predispatch-proof-barrier',action='store_true')
    args=parser.parse_args()
    config,arm,_,_=fixture(args.fixture_root)
    goal_observations=[];health_observations=[];session_ref={}
    original_sync_write=arm.bus.sync_write
    def traced_sync_write(register,values,**kwargs):
        if register!='Goal_Position':return original_sync_write(register,values,**kwargs)
        item=dict(started_monotonic=time.monotonic(),target_raw=dict(values),
            history_count_before=len(arm.bus.goal_history),
            during_wait=getattr(session_ref.get('session'),'_camera_recovery_wait_active',False))
        try:return original_sync_write(register,values,**kwargs)
        finally:
            item.update(finished_monotonic=time.monotonic(),history_count_after=len(arm.bus.goal_history))
            goal_observations.append(item)
    arm.bus.sync_write=traced_sync_write
    class Session(ArmSession):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs);session_ref['session']=self
            provider=self.camera_scene_provider
            if provider is not None:
                original_health=provider.recapture_health
                def health(generation):
                    started=time.monotonic();result=original_health(generation)
                    cached_health=provider._health;prepared_scene=provider._latest
                    health_observations.append(dict(started_monotonic=started,
                        finished_monotonic=time.monotonic(),camera_generation=generation,
                        available=result is not None,
                        state=result['state'] if result is not None else None,
                        valid_until_monotonic=result['valid_until_monotonic'] if result is not None else None,
                        cached_health=dict(cached_health) if cached_health is not None else None,
                        runtime_preparation_fault=provider._runtime_preparation_fault,
                        provider_last_error=provider.last_error,
                        prepared_scene_stamp_monotonic=(prepared_scene.stamp_monotonic if prepared_scene is not None else None),
                        prepared_acquisition_arm_stamp_monotonic=(prepared_scene.arm_stamp_monotonic if prepared_scene is not None else None),
                        during_wait=getattr(self,'_camera_recovery_wait_active',False)))
                    return result
                provider.recapture_health=health

        def recover_camera(self):
            self.fake_recovery_started=time.monotonic()
            return super().recover_camera()

        def _wait_camera_predispatch_acquisition(self,started,previous_stamp):
            path=args.fixture_root/'fake-predispatch-phase.json'
            before=list(arm.bus.writes)
            def observe(state,error=None):
                context=getattr(self,'_camera_recovery_predispatch_context',None)
                item=dict(state=state,observed_monotonic=time.monotonic(),previous_encoder_stamp=previous_stamp,
                    operation_started_monotonic=started,operation_deadline_monotonic=started+6.,
                    context_entered_monotonic=context['entered'] if context else None,
                    context_deadline_monotonic=context['deadline'] if context else None,
                    raw=dict(arm.bus.data['Present_Position']),torque=dict(arm.bus.data['Torque_Enable']),
                    goal_writes=len(arm.bus.goal_history),bus_write_count=len(arm.bus.writes),error=error)
                if not hasattr(self,'fake_predispatch_observations'):self.fake_predispatch_observations=[]
                self.fake_predispatch_observations.append(item)
                item=dict(item,first_entry=self.fake_predispatch_observations[0])
                temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(item));temporary.replace(path)
            observe('entered')
            try:
                result=super()._wait_camera_predispatch_acquisition(started,previous_stamp)
            except BaseException as error:
                observe('failed',str(error));raise
            else:
                observe('exited')
                assert arm.bus.writes==before,'passive acquisition wrote a motor register'
                return result

        def _wait_camera_recapture(self,held_raw,hold_stamp,initial,started,camera_generation):
            path=args.fixture_root/'fake-wait-phase.json'
            def phase(state,error=None):
                data=dict(state=state,observed_monotonic=time.monotonic(),hold_stamp_monotonic=hold_stamp,
                    deadline_monotonic=min(hold_stamp+.500,started+6.),held_raw=dict(held_raw),
                    raw=dict(arm.bus.data['Present_Position']),torque=dict(arm.bus.data['Torque_Enable']),
                    goal_writes=len(arm.bus.goal_history),error=error)
                if not hasattr(self,'fake_wait_observations'):self.fake_wait_observations=[]
                data['event_index']=len(self.fake_wait_observations)
                if state=='entered' and not hasattr(self,'fake_first_wait_entry'):
                    self.fake_first_wait_entry=dict(data)
                data['first_entry']=self.fake_first_wait_entry
                temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(data));temporary.replace(path)
                self.fake_wait_observations.append(data)
            phase('entered')
            previous=arm.bus.on_present_read
            if args.wait_fault and not getattr(self,'fake_wait_fault_injected',False):
                def inject():
                    arm.bus.on_present_read=previous
                    self.fake_wait_fault_injected=True
                    if args.wait_fault=='drift':arm.bus.data['Present_Position']['wrist_roll']+=1
                    else:arm.bus.data['Torque_Enable']['wrist_roll']=0
                    if previous:previous()
                arm.bus.on_present_read=inject
            try:
                result=super()._wait_camera_recapture(held_raw,hold_stamp,initial,started,camera_generation)
                phase('exited');return result
            except BaseException as error:
                phase('failed',str(error));raise
            finally:arm.bus.on_present_read=previous

        def _camera_recovery_check(self,raw,stamp,target=None):
            started=time.monotonic()
            item=dict(check_started_monotonic=started,encoder_stamp_monotonic=stamp)
            index=len(getattr(self,'fake_proof_observations',[]))
            def proof_phase(state):
                if index>1:return
                path=args.fixture_root/'fake-proof-phase.json'
                entry=dict(index=index,state=state,observed_monotonic=time.monotonic(),
                    check_started_monotonic=started,encoder_stamp_monotonic=stamp,raw=dict(raw),
                    goal_writes=len(arm.bus.goal_history))
                if not hasattr(self,'fake_proof_entries'):self.fake_proof_entries={}
                if state=='entered':self.fake_proof_entries[str(index)]=dict(entry)
                entry['entries']=dict(self.fake_proof_entries)
                temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(entry));temporary.replace(path)
            proof_phase('entered')
            def provider_audit():
                provider=self.camera_scene_provider
                scene=getattr(provider,'_latest',None)
                return dict(last_error=getattr(provider,'last_error',None),
                    prepare_duration_s=getattr(provider,'prepare_duration_s',None),
                    prewarm_duration_s=getattr(provider,'prewarm_duration_s',None),
                    prepared_scene_stamp_monotonic=getattr(scene,'stamp_monotonic',None),
                    prepared_acquisition_arm_stamp_monotonic=getattr(scene,'arm_stamp_monotonic',None),
                    runtime_started_monotonic=getattr(provider,'runtime_started_monotonic',None))
            item['provider_before']=provider_audit()
            try:
                plan=super()._camera_recovery_check(raw,stamp,target)
                if args.predispatch_proof_barrier and index==1:
                    # Fixture-only synchronization. Keep the ORIGINAL proof and
                    # encoder clocks; actual new reads are observations only.
                    deadline=min(time.monotonic()+.025,stamp+.045,
                        self._camera_recovery_valid_until_monotonic,self.fake_recovery_started+6.)
                    item['coordination_samples']=[]
                    while True:
                        self._poll_busy_input();self._check_cancel()
                        measured=self._read('Present_Position');pstamp,putc=self.clock(),self.wall_clock()
                        torque=self._read('Torque_Enable');tstamp,tutc=self.clock(),self.wall_clock()
                        if measured!=raw or any(value!=0 for value in torque.values()):
                            raise RuntimeError('fixture passive proof coordination pose/torque changed')
                        self._publish_camera_observation(measured,pstamp,putc,
                            torque_observation=(torque,tstamp,tutc))
                        self._poll_busy_input();self._check_cancel()
                        now=time.monotonic()
                        if now>=deadline or now-stamp>.045 or now-min(pstamp,tstamp)>.050:
                            raise TimeoutError('fixture passive proof coordination budget exceeded')
                        item['coordination_samples'].append(dict(position_monotonic=pstamp,
                            torque_monotonic=tstamp,raw=dict(measured),torque=dict(torque)))
                        health=self.camera_scene_provider.recapture_health(self._camera_recovery_command_generation)
                        if health is None:raise RuntimeError('fixture passive proof coordination health unavailable')
                        marker=health.get('same_generation_refresh_request_started_monotonic')
                        if marker is not None and marker==health.get('identity_request_started_monotonic'):
                            break
                        self.sleep(min(.010,max(0.,deadline-time.monotonic())))
                item['scene_stamp_monotonic']=plan.scene_stamp_monotonic
                item['scene_valid_until_monotonic']=getattr(self,'_camera_recovery_valid_until_monotonic',None)
                acquisition=getattr(getattr(self,'_camera_recovery_scene',None),'acquisition',None)
                provenance=getattr(acquisition,'acquisition_clock_provenance',{})
                identity=provenance.get('camera_identity',{})
                item['original_declared_depth_stamp_monotonic']=provenance.get('declared_depth_stamp_monotonic')
                item['camera_identity_confirmed_monotonic']=identity.get('confirmed_monotonic')
                item['camera_identity_request_started_monotonic']=identity.get('request_started_monotonic')
                item['camera_generation']=identity.get('generation')
                return plan
            except Exception as error:
                item['error']=str(error)
                raise
            finally:
                item['check_finished_monotonic']=time.monotonic()
                item['provider_after']=provider_audit()
                if not hasattr(self,'fake_proof_observations'):self.fake_proof_observations=[]
                self.fake_proof_observations.append(item)
                proof_phase('exited')

        def serve(self,**kwargs):
            if args.external_depth:
                # Test-only observation: never prepares a scene or alters any
                # runtime clock/gate. Parent still sends exactly one request.
                stop=threading.Event()
                def report_prepared():
                    provider=self.camera_scene_provider
                    while not stop.is_set():
                        scene=getattr(provider,'_latest',None)
                        data=dict(observed_monotonic=time.monotonic(),
                            scene_stamp_monotonic=getattr(scene,'stamp_monotonic',None),
                            acquisition_arm_stamp_monotonic=getattr(scene,'arm_stamp_monotonic',None),
                            last_error=getattr(provider,'last_error',None),
                            prepare_duration_s=getattr(provider,'prepare_duration_s',None),
                            prewarm_duration_s=getattr(provider,'prewarm_duration_s',None))
                        path=args.fixture_root/'fake-provider-readiness.json'
                        temporary=path.with_suffix('.tmp')
                        temporary.write_text(json.dumps(data));temporary.replace(path)
                        stop.wait(.01)
                reporter=threading.Thread(target=report_prepared,daemon=True);reporter.start()
                try:return super().serve(**kwargs)
                finally:
                    stop.set();reporter.join(1);self.write_audit()
            provider=self.camera_scene_provider
            stop=threading.Event()
            def collect():
                while not stop.is_set():
                    write_arm_frame(provider,arm)
                    stop.wait(.025)
            camera=threading.Thread(target=collect,daemon=True)
            camera.start()
            try:
                until=time.monotonic()+2
                while provider._latest is None and time.monotonic()<until:time.sleep(.005)
                if provider._latest is None:
                    raise RuntimeError('synthetic recovery acquisition unavailable: '+str(provider.last_error))
                return super().serve(**kwargs)
            finally:
                stop.set();camera.join(1)
                self.write_audit()

        def write_audit(self):
            (args.fixture_root/'fake-owner-write-audit.json').write_text(json.dumps(dict(
                    hardware_access=False,physical_qualification=False,
                    goal_history=arm.bus.goal_history,eeprom_writes=arm.bus.eeprom_writes,
                    goal_write_observations=goal_observations,health_observations=health_observations,
                    proof_observations=getattr(self,'fake_proof_observations',[]),
                    predispatch_observations=getattr(self,'fake_predispatch_observations',[]),
                    wait_observations=getattr(self,'fake_wait_observations',[]),
                    final_raw=arm.bus.data['Present_Position'],final_torque=arm.bus.data['Torque_Enable'])))
    return run_locked_session(config,args.state_file,args.logdir,lock_path=args.lock,
        persistent_idle=True,arm_factory=lambda _:arm,session_factory=Session)


if __name__=='__main__':raise SystemExit(main())
