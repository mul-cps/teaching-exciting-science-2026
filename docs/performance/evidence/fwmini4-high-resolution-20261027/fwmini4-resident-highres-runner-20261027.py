import argparse, json, os, pathlib, signal, subprocess, time
p=argparse.ArgumentParser(); p.add_argument('--mode',choices=['cpu','vulkan','auto'],required=True)
p.add_argument('--profile',choices=['vga','720p','1080p'],required=True)
p.add_argument('--prefix',type=pathlib.Path,required=True); p.add_argument('--source',type=pathlib.Path,required=True)
a=p.parse_args()
base=a.prefix/'live'/a.profile; base.mkdir(parents=True,exist_ok=True)
mode=a.mode; stem=base/mode
source=a.source
log=stem.with_suffix('.launch.log').open('w')
def totals():
    values=list(map(int,pathlib.Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
    return {'total':sum(values),'idle':values[3]+values[4]}
process=subprocess.Popen(['ros2','launch','so101_station_vulkan','resident_rgbd.launch.py',f'backend:={mode}',f'profile:={a.profile}','serial:=243322071783'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=source)
record={'profile_requested':a.profile,'mode':mode,'middleware':os.environ.get('RMW_IMPLEMENTATION'),'domain':os.environ.get('ROS_DOMAIN_ID'),'logical_cpus':os.cpu_count(),'scope':'CameraComponent+ObjectsComponent only; excludes marker registration and control'}
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
    record['rss_kib_after_steady']={k:v.strip() for line in pathlib.Path(f'/proc/{child}/status').read_text().splitlines() if ':' in line for k,v in [line.split(':',1)] if k in ('VmRSS','VmHWM')}
    expected={'vga':(640,480,640,480),'720p':(1280,720,1280,720),'1080p':(1920,1080,1280,720)}[a.profile]
    fields=data.get('last_status_fields') or {}
    actual=tuple(int(fields.get(k,0)) for k in ('color_width','color_height','depth_width','depth_height'))
    record['actual_extents']=actual;record['profile_matches']=actual==expected and fields.get('profile')==a.profile and fields.get('fps')=='30'
    allocations=sorted({int(s['fields']['vulkan_allocation_count']) for s in data.get('status_samples',[]) if s.get('fields',{}).get('backend')=='vulkan'})
    record['steady_vulkan_allocation_counts']=allocations
    record['allocation_count_stable']=len(allocations)<=1
    record['source_frame_gaps']={}
    for key in ('source_frame','depth_source_frame'):
        ids=[int(m['fields'][key]) for m in data.get('measurements',[])]
        record['source_frame_gaps'][key]=sum(y-x-1 for x,y in zip(ids,ids[1:]))
    record['passed']=bool(data.get('passed')) and record['profile_matches'] and record['allocation_count_stable']
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
