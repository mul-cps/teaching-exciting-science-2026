"""Preparation-only collector. Execute only after root releases live camera run.
No pixels, CameraInfo/calibration or serial subscription. Owns no camera/launcher.
"""
import argparse,json,os,time,pathlib,statistics
from resident_live_status import status_ready
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String

def fields(text):
 return {k.strip():v.strip() for part in text.split(';') if '=' in part for k,v in [part.split('=',1)]}
def cpu_ticks(pid):
 try:
  tail=pathlib.Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
  return int(tail[11])+int(tail[12])
 except (FileNotFoundError,ProcessLookupError):return None
def p95(values):
 return sorted(values)[int(.95*(len(values)-1))] if values else None
class Collector(Node):
 def __init__(self,args):
  super().__init__('raw_resident_private_qualifier');self.args=args;self.status=[];self.messages=[];self.errors=[];self.begin=None;self.cpu0=None;self.cpu1=None;self.initial=time.monotonic();self.last=None;self.generation=None;self.final_status=None
  q=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.s=self.create_subscription(String,'/station/resident/status',self.on_status,q)
  self.m=self.create_subscription(String,'/station/resident/measurements',self.on_measurement,10)
 def on_status(self,msg):
  f=fields(msg.data);self.status.append({'receipt_monotonic':time.monotonic(),'fields':f,'text':msg.data});self.final_status=f
  try:ready=status_ready(msg.data,self.args.mode,self.begin is not None)
  except ValueError as error:self.errors.append(str(error));return
  if not ready:return
  if self.begin is None:
   self.begin=time.monotonic();self.cpu0=cpu_ticks(self.args.container_pid)
   if self.cpu0 is None:self.errors.append('actual component container PID disappeared')
 def on_measurement(self,msg):
  if msg.data.startswith('{'):
   self.errors.append('unknown/failure measurement: '+msg.data);return
  if self.begin is None:return
  f=fields(msg.data)
  try:
   color=int(f['source_frame']);depth=int(f['depth_source_frame']);generation=int(f['calibration_generation']);source_ns=int(f['source_received_ros_ns']);now_ros=self.get_clock().now().nanoseconds
   if f.get('execution_eligible')!='false' or f.get('geometry_admission')!='unqualified':raise ValueError('unsafe eligibility')
   if now_ros<source_ns:raise ValueError('unknown/backwards received ROS time')
   if self.last is not None and (color<=self.last[0] or depth<=self.last[1]):raise ValueError('duplicate/regressed original source IDs')
   if self.generation is not None and generation!=self.generation:raise ValueError('generation changed inside one source run')
   self.last=(color,depth);self.generation=generation;self.messages.append({'receipt_monotonic':time.monotonic(),'collector_received_age_ms':(now_ros-source_ns)*1e-6,'fields':f})
  except (ValueError,KeyError) as e:self.errors.append(str(e))
 def done(self):
  now=time.monotonic()
  return bool(self.errors) or (self.begin is not None and now-self.begin>=self.args.seconds) or now-self.initial>=self.args.startup_timeout
 def result(self):
  now=time.monotonic();self.cpu1=cpu_ticks(self.args.container_pid);hz=os.sysconf('SC_CLK_TCK');cpu_ms=(self.cpu1-self.cpu0)*1000/hz if self.cpu0 is not None and self.cpu1 is not None else None;duration=now-self.begin if self.begin is not None else 0
  if self.cpu1 is None:self.errors.append('actual component container PID disappeared')
  if self.begin is None:self.errors.append('cold admission/startup timeout before required ready status')
  if not self.messages:self.errors.append('no unique measured source output')
  if duration<self.args.seconds:self.errors.append('steady window incomplete')
  return {'schema':'raw-resident-live-collector-v1','mode_requested':self.args.mode,'physical_camera_capture':True,'full_station_qualified':False,'execution_eligible':False,'geometry_admission':'unqualified','passed':not self.errors,'errors':self.errors,'steady_seconds':duration,'unique_measured_outputs':len(self.messages),'output_hz':len(self.messages)/duration if duration else 0,'collector_received_age_p95_ms':p95([m['collector_received_age_ms'] for m in self.messages]),'age_semantics':'host receipt ROS time to collector receipt; NOT hardware capture age','container_pid':self.args.container_pid,'container_cpu_ms_proc_stat':cpu_ms,'container_cpu_ms_per_unique_output':cpu_ms/len(self.messages) if cpu_ms is not None and self.messages else None,'container_cpu_method':'actual containerPID process utime+stime; startup excluded; tick quantization; driverIRQ/kworker excluded','source_generation':self.generation,'status_samples':self.status,'measurements':self.messages,'last_status_fields':self.final_status}
def main():
 p=argparse.ArgumentParser();p.add_argument('--mode',choices=('cpu','vulkan','auto'),required=True);p.add_argument('--container-pid',type=int,required=True);p.add_argument('--seconds',type=float,default=15);p.add_argument('--startup-timeout',type=float,default=45);p.add_argument('--output',type=pathlib.Path,required=True);args=p.parse_args();assert 0<args.seconds<=15 and 0<args.startup_timeout<=45
 assert os.environ.get('ROS_DOMAIN_ID')=='179','isolated qualification domain179 required'
 rclpy.init();node=Collector(args)
 try:
  while rclpy.ok() and not node.done():rclpy.spin_once(node,timeout_sec=.05)
  result=node.result();args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('status_samples','measurements')}));return 0 if result['passed'] else 1
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
