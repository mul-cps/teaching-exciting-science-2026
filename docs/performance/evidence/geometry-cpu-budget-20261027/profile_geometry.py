"""Unchanged retained CPU geometry timings plus separate attribution; no hardware."""
from pathlib import Path
import argparse,sys,os,json,time,hashlib,platform,functools,cProfile,pstats,contextlib
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--reps',type=int,default=20);p.add_argument('--warmup',type=int,default=3);p.add_argument('--profile-reps',type=int,default=3);a=p.parse_args();assert 1<=a.reps<=30 and 0<=a.warmup<=5 and 1<=a.profile_reps<=3
assert os.environ.get('OPENBLAS_NUM_THREADS')=='1';a.output.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(a.root));from tools import native_geometry_oracle as oracle
import numpy as np,cv2,scipy
from workshop.digital_twin import grasp_geometry as g
from so101_station_objects import colored_object_validation as v
objects=oracle.objects
cv2.setNumThreads(1)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):(a.output/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def encode(x):
 if isinstance(x,np.ndarray):return {'__array__':x.tolist(),'dtype':str(x.dtype),'shape':list(x.shape),'bytes_sha256':hashlib.sha256(x.tobytes(order='C')).hexdigest()}
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:encode(y) for k,y in x.items()}
 if isinstance(x,(tuple,list)):return [encode(y) for y in x]
 return x
def report(x):
 data=json.dumps(encode(x),sort_keys=True,separators=(',',':'),allow_nan=False).encode();return hashlib.sha256(data).hexdigest(),data
@contextlib.contextmanager
def instrument():
 stack=[];stats={};restores=[]
 def wrap(module,name,label):
  original=getattr(module,name)
  @functools.wraps(original)
  def measured(*args,**kwargs):
   key=label(stack) if callable(label) else label
   frame=[key,time.perf_counter_ns(),time.process_time_ns(),0,0];stack.append(frame)
   try:return original(*args,**kwargs)
   finally:
    cpu=time.process_time_ns()-frame[2];wall=time.perf_counter_ns()-frame[1];stack.pop()
    x=stats.setdefault(key,dict(calls=0,inclusive_wall_ns=0,inclusive_cpu_ns=0,exclusive_wall_ns=0,exclusive_cpu_ns=0));x['calls']+=1;x['inclusive_wall_ns']+=wall;x['inclusive_cpu_ns']+=cpu;x['exclusive_wall_ns']+=wall-frame[3];x['exclusive_cpu_ns']+=cpu-frame[4]
    if stack:stack[-1][3]+=wall;stack[-1][4]+=cpu
  restores.append((module,name,original));setattr(module,name,measured)
 def scope(stack):return next((x[0] for x in reversed(stack) if x[0] in ['support_weighted_fit','support_RANSAC_robust_and_residual_report','component_validation','visible_geometry','pickup_geometry','grasp_proposals']), 'other')
 def numeric(kind):return lambda stack:scope(stack)+'/'+kind
 wrap(objects,'analyze_rgbd','analyze_rgbd')
 wrap(objects.ColorMaskBackend,'__call__','color_masks')
 wrap(objects,'_fit_world_support','support_sampling_cap_and_admission')
 wrap(objects,'fit_support_plane','support_RANSAC_robust_and_residual_report')
 wrap(g,'_weighted_plane','support_weighted_fit')
 for name,label in [('split_depth_coherent_components','child_depth_preparation'),('validate_colored_component','component_validation'),('validate_brick_silhouette','silhouette_validation'),('validate_child_visibility','child_visibility'),('_visible_component_geometry','visible_geometry'),('_component_geometry','pickup_geometry'),('footprint_grasps','grasp_proposals'),('_split_elongated_contours','elongated_contour_split'),('camera_points','projection'),('transform_points','world_transform'),('_inside_roi','ROI_predicate')]:wrap(objects,name,label)
 for module in [objects,v]:wrap(module,'linear_percentile',numeric('linear_percentile'))
 for name in ['median','percentile']:wrap(np,name,numeric(name))
 wrap(np.linalg,'svd',numeric('SVD'));wrap(np.linalg,'eigh',numeric('PCA_eigh'))
 for name in ['findContours','drawContours','morphologyEx','connectedComponentsWithStatsWithAlgorithm']:wrap(cv2,name,'OpenCV/'+name)
 try:yield stats
 finally:
  for module,name,original in reversed(restores):setattr(module,name,original)
def summary(samples):
 x=np.asarray(samples);return {'n':len(x),'p50_ms':float(np.percentile(x,50))/1e6,'p95_ms':float(np.percentile(x,95))/1e6,'min_ms':float(x.min())/1e6,'max_ms':float(x.max())/1e6,'mean_ms':float(x.mean())/1e6}
manifest=json.loads((a.inputs/'manifest.json').read_text());assert len(manifest['cases'])==12
source_before={n:sha(a.root/n) for n in oracle.SOURCES};file_pins={str(p.relative_to(a.inputs)):sha(p) for p in sorted(a.inputs.rglob('*')) if p.is_file()};save('input-pins.json',file_pins)
results=[]
for case in manifest['cases']:
 name=case['name'];snap,arrays=oracle.load_verified(a.inputs/name)
 assert sha(a.inputs/name/'manifest.json')==case['oracle_manifest_sha256']
 assert all(snap[k]==case[k] for k in ('inputs','provenance','arrays'))
 for n,h in snap['provenance']['sources'].items():assert sha(a.root/n)==h,(name,n)
 args=oracle.restore_arrays(snap['inputs'],arrays);expected=oracle.restore_arrays(snap['report'],arrays)
 before=oracle.analyze_reference(args);oracle.assert_report_equal(expected,before);digest,payload=report(before);(a.output/(name+'-complete-report.json')).write_bytes(payload)
 for _ in range(a.warmup):oracle.analyze_reference(args)
 wall=[];cpu=[];hashes=[]
 for _ in range(a.reps):
  t=time.perf_counter_ns();c=time.process_time_ns();value=oracle.analyze_reference(args);cpu.append(time.process_time_ns()-c);wall.append(time.perf_counter_ns()-t)
  oracle.assert_report_equal(expected,value);hashes.append(report(value)[0]);assert hashes[-1]==digest
 profile_hashes=[]
 with instrument() as attribution:
  for _ in range(a.profile_reps):
   value=oracle.analyze_reference(args);profile_hashes.append(report(value)[0]);assert profile_hashes[-1]==digest
 # Verification/encoding runs while wrappers installed above; only numpy conversion/hash bookkeeping, not analyzer numeric helpers.
 profiler=cProfile.Profile();profiler.enable();value=oracle.analyze_reference(args);profiler.disable();oracle.assert_report_equal(expected,value);assert report(value)[0]==digest;profiler.dump_stats(str(a.output/(name+'.prof')))
 ps=pstats.Stats(profiler);records=[]
 for (file,line,func),(primitive,calls,self_seconds,inclusive_seconds,callers) in ps.stats.items():records.append(dict(file=file,line=line,function=func,calls=calls,primitive_calls=primitive,self_wall_ms=self_seconds*1000,inclusive_wall_ms=inclusive_seconds*1000))
 records.sort(key=lambda r:r['inclusive_wall_ms'],reverse=True);save(name+'-cprofile.json',records[:100])
 after=oracle.analyze_reference(args);oracle.assert_report_equal(expected,after);assert report(after)[0]==digest
 support=oracle.restore_arrays(snap['trace']['support'],arrays)
 stages={k:{**x,'per_call_inclusive_wall_ms':x['inclusive_wall_ns']/x['calls']/1e6,'per_analyze_inclusive_wall_ms':x['inclusive_wall_ns']/a.profile_reps/1e6,'per_analyze_exclusive_wall_ms':x['exclusive_wall_ns']/a.profile_reps/1e6,'per_analyze_inclusive_cpu_ms':x['inclusive_cpu_ns']/a.profile_reps/1e6,'per_analyze_exclusive_cpu_ms':x['exclusive_cpu_ns']/a.profile_reps/1e6} for k,x in attribution.items()}
 total=stages['analyze_rgbd'];assert sum(x['exclusive_cpu_ns'] for x in stages.values())==total['inclusive_cpu_ns'];assert stages['support_weighted_fit']['calls']==9*a.profile_reps
 serialization=[]
 for _ in range(5):
  t=time.perf_counter_ns();json.dumps(oracle.plain_values(after),sort_keys=True,allow_nan=False);serialization.append(time.perf_counter_ns()-t)
 row=dict(name=name,shape=list(args['depth'].shape),frame_pixels=int(args['depth'].size),support_pre_cap=len(support['inside_board']),support_selected=len(support['selected_board']),diagnostics=before['validation_diagnostics'],returned_objects=len(before['objects']),report_sha256=digest,frozen_report_tolerance=1e-9,frozen_report_matched=True,all_uninstrumented_report_hashes=hashes,instrumented_report_hashes=profile_hashes,after_report_sha256=report(after)[0],uninstrumented_wall=summary(wall),uninstrumented_process_CPU=summary(cpu),uninstrumented_samples_ns={'wall':wall,'CPU':cpu},instrumented_stages=stages,report_JSON_serialization_wall=summary(serialization))
 results.append(row);print(json.dumps({'case':name,'wall_p50_ms':row['uninstrumented_wall']['p50_ms'],'CPU_p50_ms':row['uninstrumented_process_CPU']['p50_ms'],'parity':True}),flush=True)
source_after={n:sha(a.root/n) for n in oracle.SOURCES};assert source_after==source_before
libraries={}
for line in Path('/proc/self/maps').read_text().splitlines():
 path=line.split()[-1]
 if path.startswith('/') and ('.so' in path) and any(x in path for x in ['numpy','scipy','cv2','opencv','libpython','libstdc++','libgomp','libc.so','libm.so']):
  file=Path(path)
  if file.is_file():libraries[str(file)]=dict(sha256=sha(file),bytes=file.stat().st_size)
proof=dict(scope='Unchanged retained full CPU geometry on original12 procedural matrix cases only; no native/GPU/camera/ROS/physical/whole-station or headroom claim',factor_prototype_used=False,source_pins_before=source_before,source_pins_after=source_after,input_pins_sha256=sha(a.output/'input-pins.json'),script_sha256=sha(Path(__file__)),loaded_libraries=libraries,runtime={'package_paths':{'numpy':np.__file__,'opencv':cv2.__file__,'scipy':scipy.__file__},'host':platform.node(),'machine':platform.machine(),'python':sys.version,'executable':sys.executable,'numpy':np.__version__,'opencv':cv2.__version__,'scipy':scipy.__version__,'OPENBLAS_NUM_THREADS':os.getenv('OPENBLAS_NUM_THREADS'),'OMP_NUM_THREADS':os.getenv('OMP_NUM_THREADS'),'opencv_threads':cv2.getNumThreads(),'cpu_count':os.cpu_count()},warmup=a.warmup,repetitions=a.reps,profile_repetitions=a.profile_reps,profiling_scope='Nested wrapper attribution and separate cProfile pass; overhead is included, not throughput. Exclusive wall/CPU stages are nonoverlapping and sum to analyze_rgbd inclusive; nested inclusive values must not be summed. Root exclusive contains unwrapped indexing/ROI/mask/report assembly. Support fit residual exclusive includes120 RANSAC trials and robust update/residual report work; nine weighted fits and their nine SVDs are separate nested values. Serialization measured separately, outside analyze_rgbd. Loading, validation and hashes excluded from uninstrumented timers.',cases=results)
save('result.json',proof)
