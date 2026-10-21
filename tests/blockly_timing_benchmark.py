"""Offline per-replay preparation cost; never runs on the feedback/control tick."""
import math
from pathlib import Path
import statistics
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workshop.blockly_recording import JOINTS,nominal_curve_bounds,stationary_pause_points
rows=[{'time':i*.09,'positions':[0.]*5 if i<10 else [math.sin(i*.1)*.01*(j+1) for j in range(5)]} for i in range(600)]
candidate=stationary_pause_points(rows);start=dict.fromkeys(JOINTS,0.)
samples=[]
for _ in range(15):
    began=time.perf_counter_ns();nominal_curve_bounds(JOINTS,rows,candidate,start);samples.append((time.perf_counter_ns()-began)/1e6)
print({'source_knots':len(rows),'candidate_knots':len(candidate),'per_replay_p50_ms':statistics.median(samples),'per_replay_p95_ms':sorted(samples)[-1],'control_tick_work':0})
