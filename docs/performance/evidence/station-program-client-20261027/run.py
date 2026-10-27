from pathlib import Path
import os,sys,json,subprocess,time,signal,hashlib
R=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022');B=Path('/tmp/station-client-native177-20261027');wrapper=R/'ros2/conda/so101-conda.sh'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,x):(B/n).write_text(json.dumps(x,indent=2)+'\n')
paths=list((R/'ros2/src/so101_station_workflow').rglob('*'))+[R/x for x in ['tests/blockly_client_native.py','tests/blockly_external_control_native.py','workshop/blockly_client.py','workshop/blockly_ros_api.py','tools/run_station_program.py','tools/run_blockly_station.py','ros2/conda/so101-conda.sh']]
pins={str(p.relative_to(R)):sha(p) for p in paths if p.is_file()};save('source-before.json',pins)
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST');env.pop('STATION_PROGRAM_LAUNCHER',None)
def run(name,cmd,timeout):
 start=time.monotonic();p=subprocess.Popen(cmd,cwd=R,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True);timed=False;signals=[]
 try:output,_=p.communicate(timeout=timeout)
 except subprocess.TimeoutExpired:
  timed=True;os.killpg(p.pid,signal.SIGTERM);signals.append('TERM')
  try:output,_=p.communicate(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);signals.append('KILL');output,_=p.communicate(timeout=5)
 (B/(name+'.log')).write_bytes(output);result={'command':cmd,'returncode':p.returncode,'timed_out':timed,'reaped':p.returncode is not None,'signals':signals,'elapsed_seconds':time.monotonic()-start,'log_sha256':sha(B/(name+'.log'))};save(name+'.json',result);print(json.dumps({'step':name,**result}),flush=True);return result
build=run('interface-build',[str(wrapper),'run','colcon','--log-base',str(B/'log'),'build','--base-paths',str(R/'ros2/src/so101_station_workflow'),'--build-base',str(B/'build'),'--install-base',str(B/'install'),'--cmake-args','-DBUILD_TESTING=OFF'],120)
if build['returncode']!=0 or build['timed_out']:sys.exit(1)
metadata="""import sys,os,json,importlib.metadata as m,hashlib
import rclpy,so101_station_workflow.srv._station_command as s
from rclpy.impl.implementation_singleton import rclpy_implementation as impl
from so101_station_workflow.srv import StationCommand
from pathlib import Path
versions={}
for name in ['rclpy','colcon-core','rosidl-generator-py']:
 try:versions[name]=m.version(name)
 except m.PackageNotFoundError:versions[name]=None
print(json.dumps(dict(python=sys.version,executable=sys.executable,versions=versions,rclpy_file=rclpy.__file__,rclpy_extension=impl.__file__,station_command_file=s.__file__,station_command_sha256=hashlib.sha256(Path(s.__file__).read_bytes()).hexdigest(),request_fields=StationCommand.Request.get_fields_and_field_types(),response_fields=StationCommand.Response.get_fields_and_field_types(),environment={k:os.getenv(k) for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']})))
"""
(B/'metadata.py').write_text(metadata)
def overlaycmd(script):return [str(wrapper),'run','bash','--noprofile','--norc','-c','source "$1/local_setup.bash"; exec python "$2"','isolated-client',str(B/'install'),str(script)]
results=[run('runtime',overlaycmd(B/'metadata.py'),30)]
for script in ['blockly_client_native.py','blockly_external_control_native.py']:results.append(run(script[:-3],overlaycmd(R/'tests'/script),120))
after={str(p.relative_to(R)):sha(p) for p in paths if p.is_file()};save('source-after.json',after);assert after==pins
save('proof.json',{'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=R,text=True).strip(),'source_unchanged':True,'domain':177,'hardware':False,'GPU':False,'service_changes':False,'results':results,'all_passed':all(x['returncode']==0 and not x['timed_out'] and x['reaped'] for x in results)})
sys.exit(0 if all(x['returncode']==0 and not x['timed_out'] for x in results) else 1)
