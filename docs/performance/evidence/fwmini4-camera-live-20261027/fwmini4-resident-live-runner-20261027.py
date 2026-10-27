import argparse, json, os, pathlib, signal, subprocess, time
p=argparse.ArgumentParser(); p.add_argument('--mode',choices=['cpu','vulkan','auto'],required=True); a=p.parse_args()
base=pathlib.Path('/home/bjoern/.local/share/exciting-science/validation/0542aba/live'); base.mkdir(exist_ok=True)
mode=a.mode; stem=base/mode
source=pathlib.Path('/home/bjoern/git/teaching-exciting-science-2026-validation-0542aba')
log=stem.with_suffix('.launch.log').open('w')
def totals():
    values=list(map(int,pathlib.Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
    return {'total':sum(values),'idle':values[3]+values[4]}
process=subprocess.Popen(['ros2','launch','so101_station_vulkan','resident_rgbd.launch.py',f'backend:={mode}','serial:=243322071783'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=source)
record={'mode':mode,'middleware':os.environ.get('RMW_IMPLEMENTATION'),'domain':os.environ.get('ROS_DOMAIN_ID'),'logical_cpus':os.cpu_count(),'scope':'CameraComponent+ObjectsComponent only; excludes marker registration and control'}
try:
    deadline=time.monotonic()+15; child=None
    while time.monotonic()<deadline:
        if process.poll() is not None: raise RuntimeError('launcher exited before container')
        for entry in pathlib.Path('/proc').iterdir():
            if not entry.name.isdigit(): continue
            try:
                raw=(entry/'cmdline').read_bytes().split(b'\0')
                if raw and raw[0].endswith(b'/component_container') and os.getpgid(int(entry.name))==process.pid:
                    child=int(entry.name); break
            except (FileNotFoundError,ProcessLookupError,PermissionError): pass
        if child: break
        time.sleep(.1)
    if not child: raise RuntimeError('actual component container not found')
    record['container_pid']=child
    before=totals(); start=time.monotonic()
    result=subprocess.run(['/usr/bin/python3',str(source/'tools/performance/collect_resident_live.py'),'--mode',mode,'--container-pid',str(child),'--seconds','15','--startup-timeout','45','--output',str(stem.with_suffix('.json'))],stdout=stem.with_suffix('.collector.log').open('w'),stderr=subprocess.STDOUT,cwd=source,timeout=55)
    after=totals(); record.update(collector_exit=result.returncode,observed_seconds=time.monotonic()-start)
    record['host_busy_fraction_including_startup_and_other_processes']=1-(after['idle']-before['idle'])/(after['total']-before['total'])
    data=json.loads(stem.with_suffix('.json').read_text()) if stem.with_suffix('.json').exists() else {}
    record['summary']={k:v for k,v in data.items() if k not in ['status_samples','measurements']}
    record['container_fraction_of_one_core']=data['container_cpu_ms_proc_stat']/1000/data['steady_seconds'] if data.get('container_cpu_ms_proc_stat') is not None and data.get('steady_seconds') else None
except Exception as e:
    record['runner_error']=str(e)
finally:
    try: os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError: pass
    try: process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try: process.wait(timeout=4)
        except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL); process.wait()
    log.close(); record['launcher_exit']=process.returncode
    stem.with_suffix('.runner.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)
