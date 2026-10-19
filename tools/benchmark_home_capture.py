#!/usr/bin/env python3
"""Cold explicit capture benchmark: fake resident bus, real temporary FS only."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root),str(root/'tests')]
from test_arm_session import make_session


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples',type=int,default=20)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1<=args.samples<=100:parser.error('samples must be1..100')
    times=[]
    with tempfile.TemporaryDirectory(prefix='so101-capture-benchmark-') as directory:
        session,arm,clock=make_session(Path(directory));session.initialize();session.handle({'command':'stop'})
        arm.bus.writes.clear();arm.bus.goal_history.clear()
        try:
            for i in range(args.samples):
                start=time.perf_counter_ns()
                reply=session.handle(dict(command='capture_home',command_id=f'{i:032x}',
                    owner_session_id=session._owner_session_id,expected_start_raw=dict(arm.bus.data['Present_Position'])))
                times.append((time.perf_counter_ns()-start)/1e6)
                assert reply['ok'],reply
            assert not arm.bus.writes and not arm.bus.goal_history
        finally:session.close_log()
    result=dict(schema='resident-home-capture-cold-benchmark-v1',samples=args.samples,
        scope=__doc__,sdk_or_hardware_opened=False,gpu_executed=False,motor_writes=0,
        mean_ms=sum(times)/len(times),p95_ms=sorted(times)[int(.95*(len(times)-1))],max_ms=max(times),
        limitations='Shared host; fake encoder clock/read cost. Real temp filesystem fsync+atomic replace, not live configured-home storage guarantee. No tick/hot path work.')
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
