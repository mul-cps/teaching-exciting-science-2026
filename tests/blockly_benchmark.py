"""Offline per-message microbenchmark; no ROS or devices required."""
from pathlib import Path
import statistics
import sys
import threading
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workshop.blockly_ros import ROSBackend

class Stamp: sec=100; nanosec=0
class Header: stamp=Stamp()
class Message:
    header=Header()
    name=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
    position=[0.]*6
class Now: nanoseconds=100000010000
class Clock:
    def now(self): return Now()
class Node:
    def get_clock(self): return Clock()
backend=ROSBackend.__new__(ROSBackend); backend.lock=threading.Lock();backend.latest=None;backend.node=Node()
message=Message(); samples=[]
for index in range(10000):
    Stamp.nanosec=index
    start=time.perf_counter_ns();backend._joints(message);samples.append(time.perf_counter_ns()-start)
assert backend.latest['stamp']>100
print({'callback_p50_us':statistics.median(samples)/1000,'callback_p95_us':sorted(samples)[9500]/1000,'messages':len(samples)})
