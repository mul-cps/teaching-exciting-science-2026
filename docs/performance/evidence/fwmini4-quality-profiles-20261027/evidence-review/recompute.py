"""Raw-only independent check. Does not import the offline analyzer or launch native code."""
import json,math,hashlib,tarfile
from pathlib import Path
from collections import defaultdict
B=Path('/tmp/fwmini4-quality-profiles-rviz-20261027/raw');P=Path('/tmp/quality-profiles-independent-evidence-review-20261027')
def rows(path):return [json.loads(s) for s in path.read_text().splitlines() if s.lstrip().startswith('{')]
def close(a,b):assert math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-8),(a,b)
EXT={'realtime':[640,480,640,480],'medium':[1280,720,1280,720],'high_quality':[1920,1080,1280,720]}
CUM=['received','received_unique','completed','skipped','replaced','contention_discard','stopped_discard','duplicate_or_regressed','invalid_received','selected','submitted','computed','published','capture_service_calls','sdk_color_id_gaps','sdk_depth_id_gaps','capture_gap_deadline_misses','capture_duration_deadline_misses','dropped','acquisition_gap_deadline_misses','received_age_unknown']
result={};inputs={}
for profile in EXT:
 gl=rows(B/'logs'/f'rviz-{profile}.log');ob=rows(B/'logs'/f'processes-{profile}.jsonl')
 for lane,mode in [('vulkan','vulkan'),('system-cpu','cpu')]:
  f=B/lane/'live'/profile/f'{mode}.json';d=json.loads(f.read_text());ru=json.loads(f.with_name(mode+'.runner.json').read_text());key=profile+'/'+mode
  for path in (f,f.with_name(mode+'.runner.json'),B/'logs'/f'rviz-{profile}.log',B/'logs'/f'processes-{profile}.jsonl'):inputs[str(path.relative_to(B))]=hashlib.sha256(path.read_bytes()).hexdigest()
  assert d['passed'] and not d['errors'] and ru['passed'] and ru['profile_matches'] is True and ru['actual_extents']==EXT[profile] and ru['mode']==mode and ru['profile_requested']==profile and ru['logical_cpus']==4
  statuses=[s for s in d['status_samples'] if s['text'].startswith('experimental')]
  first=statuses[0]['receipt_monotonic'];duration=d['steady_seconds'];end=first+duration
  statuses=[s for s in statuses if first<=s['receipt_monotonic']<=end]
  assert all(y['receipt_monotonic']>x['receipt_monotonic'] for x,y in zip(statuses,statuses[1:]))
  fresh=[]
  for s in statuses:
   v=s['fields'];assert v['backend']==mode and v['profile']==profile and v['profile_requested']==profile and v['alignment']==('raw_vulkan' if mode=='vulkan' else 'vendor_cpu') and v['skip_policy']=='latest_native'
   assert [int(v[k]) for k in ['color_width','color_height','depth_width','depth_height']]==EXT[profile]
   if v['counter_snapshot_fresh']!='true':continue
   assert float(v['counter_snapshot_age_ms'])==0
   c={k:int(v[k]) for k in CUM+['active','pending']};assert all(n>=0 for n in c.values());assert c['active']<=1 and c['pending']<=1
   assert c['received']==c['received_unique']+c['duplicate_or_regressed']+c['invalid_received'];assert c['received_unique']==c['completed']+c['skipped']+c['active']+c['pending'];assert c['skipped']==c['replaced']+c['contention_discard']+c['dropped']+c['stopped_discard'];assert c['selected']>=c['submitted']>=c['computed']>=c['published'];assert c['completed']==c['published'] and v['counter_conservation']=='true'
   fresh.append((s,c))
  assert len(fresh)>=2
  for (_,a),(_,b) in zip(fresh,fresh[1:]):assert all(b[k]>=a[k] for k in CUM)
  a,c=fresh[0];b,e=fresh[-1];dt=b['receipt_monotonic']-a['receipt_monotonic'];delta={k:e[k]-c[k] for k in CUM}
  pw=ru['producer_status_window'];close(dt,pw['seconds']);close(delta['received_unique']/dt,pw['received_unique_hz']);close(delta['published']/dt,pw['published_unique_hz']);assert all(delta[k]==v for k,v in pw['counter_deltas'].items())
  m=d['measurements'];assert all(first<=x['receipt_monotonic']<=end for x in m);n=len({int(x['fields']['source_frame']) for x in m});ages=sorted(x['collector_received_age_ms'] for x in m);p95=ages[math.floor(.95*(len(ages)-1))];cpu=d['container_cpu_ms_proc_stat'];close(n/duration,d['output_hz']);close(cpu/n,d['container_cpu_ms_per_unique_output']);close(p95,d['collector_received_age_p95_ms']);assert n==d['unique_measured_outputs']
  gaps={}
  for field in ('source_frame','depth_source_frame'):
   ids=[int(x['fields'][field]) for x in m];assert all(y>x for x,y in zip(ids,ids[1:]));gaps[field]=sum(y-x-1 for x,y in zip(ids,ids[1:]))
  groups=defaultdict(list)
  for s in gl:
   if s.get('event')=='interval' and s['start_ns']>=first*1e9 and s['end_ns']<=end*1e9:groups[(s['display'],s['drawable'])].append(s)
  viewports=[]
  for pair,g in groups.items():
   g.sort(key=lambda s:s['start_ns']);assert all(y['start_ns']>=x['end_ns'] for x,y in zip(g,g[1:]));assert all(s['end_ns']>s['start_ns'] and s['elapsed_ns']==s['end_ns']-s['start_ns'] and s['interval_swaps']>0 for s in g)
   metadata=[s for s in gl if s.get('event')=='renderer' and (s['display'],s['drawable'])==pair and s['monotonic_ns']<=g[0]['start_ns']];mta=max(metadata,key=lambda s:s['monotonic_ns']);assert mta['direct'] and mta['direct_known'] and mta['metadata_complete'] and mta['vendor']=='Intel' and 'ADL-N' in mta['renderer']
   seconds=sum(s['elapsed_ns'] for s in g)/1e9;swaps=sum(s['interval_swaps'] for s in g);viewports.append(dict(display=pair[0],drawable=pair[1],area=max(s['width']*s['height'] for s in g),dimensions=sorted({(s['width'],s['height']) for s in g}),intervals=len(g),covered_seconds=seconds,swaps=swaps,swap_hz=swaps/seconds,renderer=mta['renderer']))
  primary=max(viewports,key=lambda s:s['area']);assert primary['dimensions']==[(574,810)] and len(viewports)==3
  left=min(range(len(ob)),key=lambda i:abs(ob[i]['monotonic']-first));right=min(range(len(ob)),key=lambda i:abs(ob[i]['monotonic']-end));sel=ob[left:right+1];a,b=sel[0],sel[-1];obsdt=b['monotonic']-a['monotonic'];assert obsdt>0;hz=a['clock_ticks_per_second'];assert all(s['clock_ticks_per_second']==hz for s in sel);used=set();external=0;procs={}
  for name in sorted(set().union(*(s['processes'] for s in sel))):
   states=[s['processes'][name] for s in sel];assert not any(s.get('missing') for s in states);assert len({(s['pid'],s['starttime_ticks']) for s in states})==1;assert all(y['cpu_ticks']>=x['cpu_ticks'] for x,y in zip(states,states[1:]));x,y=states[0],states[-1];cms=(y['cpu_ticks']-x['cpu_ticks'])*1000/hz;pct=cms/obsdt/40
   engines={}
   for client in x.get('drm_clients',{}).keys()&y.get('drm_clients',{}).keys():
    ex,ey=x['drm_clients'][client],y['drm_clients'][client];engines[client]={k.removeprefix('drm-engine-'):int(ey[k].split()[0])-int(ex[k].split()[0]) for k in ex.keys()&ey.keys() if k.startswith('drm-engine-') and not k.startswith('drm-engine-capacity-') and ex[k].endswith(' ns') and ey[k].endswith(' ns')};assert all(v>=0 for v in engines[client].values())
   procs[name]=dict(pid=x['pid'],starttime_ticks=x['starttime_ticks'],CPU_ms=cms,total_four_core_CPU_percent=pct,RSS_end_pages=y['rss_pages'],RSS_max_pages=max(s['rss_pages'] for s in states),engine_delta_ns=engines)
   if x['pid']!=d['container_pid'] and x['pid'] not in used:external+=pct;used.add(x['pid'])
  host=[y-x for x,y in zip(a['host_cpu_ticks'],b['host_cpu_ticks'])];assert all(v>=0 for v in host);hostbusy=100*(sum(host)-host[3]-host[4])/sum(host)
  cumulative_max={k:max(float(s['fields'][k]) for s in statuses) for k in ['capture_service_gap_max_ms','capture_duration_max_ms','acquisition_gap_max_ms']}
  result[key]=dict(collector_window=dict(begin=first,end=end,seconds=duration),perception=dict(unique_outputs=n,output_hz=n/duration,CPU_ms=cpu,total_four_core_CPU_percent=cpu/duration/40,CPU_ms_per_output=cpu/n,receipt_age_p95_ms=p95,source_ID_gaps=gaps),producer=dict(snapshot_window_seconds=dt,begin=fresh[0][0]['receipt_monotonic'],end=fresh[-1][0]['receipt_monotonic'],fresh_snapshots=len(fresh),all_snapshots=len(statuses),unique_intake_hz=delta['received_unique']/dt,published_status_delta_hz=delta['published']/dt,capture_service_hz=delta['capture_service_calls']/dt,delta=delta,cumulative_maxima=cumulative_max,max_active=max(x[1]['active'] for x in fresh),max_pending=max(x[1]['pending'] for x in fresh)),primary_viewport=primary,viewports=viewports,observer=dict(begin=a['monotonic'],end=b['monotonic'],seconds=obsdt,begin_offset=a['monotonic']-first,end_offset=b['monotonic']-end,external_four_core_CPU_percent=external,processes=procs,host_busy_percent=hostbusy,sampling_wall_cost_mean=sum(s['sampling_cost_seconds'] for s in sel)/len(sel),sampling_wall_cost_max=max(s['sampling_cost_seconds'] for s in sel)),approximate_combined_four_core_CPU_percent=cpu/duration/40+external,loaded_libraries=ru['loaded_libraries'])
(P/'independent-metrics.json').write_text(json.dumps(result,indent=2)+'\n');(P/'input-sha256.json').write_text(json.dumps(inputs,indent=2)+'\n')
print(json.dumps({k:dict(input_hz=v['producer']['unique_intake_hz'],output_hz=v['perception']['output_hz'],CPU_percent=v['perception']['total_four_core_CPU_percent'],CPU_ms_output=v['perception']['CPU_ms_per_output'],p95_age_ms=v['perception']['receipt_age_p95_ms'],swap_hz=v['primary_viewport']['swap_hz'],service_gap_max_ms=v['producer']['cumulative_maxima']['capture_service_gap_max_ms'],duration_max_ms=v['producer']['cumulative_maxima']['capture_duration_max_ms'],SDKgap_delta=v['producer']['delta']['sdk_color_id_gaps'],acquisition_gap_misses=v['producer']['delta']['acquisition_gap_deadline_misses'],capture_gap_misses=v['producer']['delta']['capture_gap_deadline_misses'],external_CPU=v['observer']['external_four_core_CPU_percent']) for k,v in result.items()},indent=2))
