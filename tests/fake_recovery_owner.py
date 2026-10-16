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

from test_recovery_runtime import fixture, write_frame
from workshop.station.arm_session import ArmSession, run_locked_session


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-root',type=Path,required=True)
    parser.add_argument('--state-file',type=Path,required=True)
    parser.add_argument('--logdir',type=Path,required=True)
    parser.add_argument('--lock',type=Path,required=True)
    args=parser.parse_args()
    config,arm,_,_=fixture(args.fixture_root)
    class Session(ArmSession):
        def serve(self,**kwargs):
            provider=self.camera_scene_provider
            stop=threading.Event()
            def collect():
                while not stop.is_set():
                    write_frame(provider,dict(arm.bus.data['Present_Position']))
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
                (args.fixture_root/'fake-owner-write-audit.json').write_text(json.dumps(dict(
                    hardware_access=False,physical_qualification=False,
                    goal_history=arm.bus.goal_history,eeprom_writes=arm.bus.eeprom_writes,
                    final_raw=arm.bus.data['Present_Position'],final_torque=arm.bus.data['Torque_Enable'])))
    return run_locked_session(config,args.state_file,args.logdir,lock_path=args.lock,
        persistent_idle=True,arm_factory=lambda _:arm,session_factory=Session)


if __name__=='__main__':raise SystemExit(main())
