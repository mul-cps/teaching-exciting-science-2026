from pathlib import Path
import subprocess,os,signal,time,json,hashlib
R=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022');B=Path('/tmp/station-client-native177-20261027');name='wrapper-final';command=[str(R/'ros2/conda/so101-conda.sh'),'run','bash','--noprofile','--norc','-c','source "$1/local_setup.bash";exec python "$2" --wrapper-overlay "$1"','isolated-wrapper',str(B/'install'),str(R/'tests/blockly_client_native.py')]
files=['workshop/blockly_client.py','tools/run_station_program.py','tests/blockly_client_native.py','tests/test_blockly_client.py']
before={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in files}
start=time.monotonic();p=subprocess.Popen(command,cwd=R,env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True);timed=False;signals=[]
try:out,_=p.communicate(timeout=120)
except subprocess.TimeoutExpired:
 timed=True;os.killpg(p.pid,signal.SIGTERM);signals.append('TERM')
 try:out,_=p.communicate(timeout=5)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);signals.append('KILL');out,_=p.communicate(timeout=5)
(B/(name+'.log')).write_bytes(out);d={'command':command,'returncode':p.returncode,'timed_out':timed,'reaped':p.returncode is not None,'signals':signals,'elapsed_seconds':time.monotonic()-start,'test_source_sha256':hashlib.sha256((R/'tests/blockly_client_native.py').read_bytes()).hexdigest(),'log_sha256':hashlib.sha256(out).hexdigest()};d['source_before']=before;d['source_after']={n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in files};assert d['source_before']==d['source_after']
(B/(name+'.json')).write_text(json.dumps(d,indent=2)+'\n');print(json.dumps(d,indent=2));raise SystemExit(0 if p.returncode==0 and not timed else 1)
