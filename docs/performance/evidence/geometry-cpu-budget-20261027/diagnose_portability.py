"""Read-only retained-call observation, never an alternative solver or accepted timing."""
from pathlib import Path
import argparse,sys,os,json,hashlib,inspect,platform
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);os.environ['SO101_STATION_ROOT']=str(a.root);sys.path.insert(0,str(a.root))
from tools import native_geometry_oracle as oracle
import numpy as np,cv2,scipy
cv2.setNumThreads(1)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def encode(x):
 if isinstance(x,np.ndarray):return {'__array__':x.tolist(),'dtype':str(x.dtype),'shape':list(x.shape),'bytes_sha256':hashlib.sha256(x.tobytes(order='C')).hexdigest()}
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:encode(v) for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [encode(v) for v in x]
 return x
def first_difference(e,v,path='report'):
 try:oracle.assert_report_equal(e,v,path);return None
 except AssertionError:pass
 if isinstance(e,dict) and isinstance(v,dict) and e.keys()==v.keys():
  for k in e:
   result=first_difference(e[k],v[k],path+'.'+str(k))
   if result is not None:return result
 if isinstance(e,(list,tuple)) and isinstance(v,(list,tuple)) and len(e)==len(v):
  for i,(left,right) in enumerate(zip(e,v)):
   result=first_difference(left,right,path+f'[{i}]')
   if result is not None:return result
 if isinstance(e,np.ndarray) and np.asarray(v).shape==e.shape:
  vv=np.asarray(v);bad=~np.isclose(e,vv,rtol=0,atol=1e-9,equal_nan=True);indices=np.argwhere(bad)
  return {'path':path,'shape':list(e.shape),'indices':indices[:8].tolist(),'expected':e[bad][:8].tolist(),'actual':vv[bad][:8].tolist(),'max_absolute_error':float(np.max(np.abs(e.astype(float)-vv)))}
 return {'path':path,'expected':encode(e),'actual':encode(v)}
arrays={};calls=[]
def record(kind,original,*args,**kwargs):
 frame=inspect.currentframe().f_back.f_back
 values=original(*args,**kwargs);number=len(calls);prefix=f'{number:02d}-{kind}';payload={'input':np.asarray(args[0]).copy()}
 for i,value in enumerate(values):payload['output'+str(i)]=np.asarray(value).copy()
 locals_seen={}
 for key in ['center','origin','points','weights','p','q','base_w','normal','axis_x','axis_y','tangent','centered','height_m','contour_height_m','projection_height','pixels','n','d']:
  value=frame.f_locals.get(key)
  if isinstance(value,np.ndarray):payload['local-'+key]=value.copy()
  elif isinstance(value,(int,float,str)) or value is None:locals_seen[key]=value
 metadata={}
 for key,value in payload.items():
  name=prefix+'-'+key;arrays[name]=value;metadata[key]={'array':name,'shape':list(value.shape),'dtype':str(value.dtype),'sha256':hashlib.sha256(value.tobytes(order='C')).hexdigest()}
 calls.append({'kind':kind,'caller':frame.f_code.co_name,'caller_file':frame.f_code.co_filename,'caller_line':frame.f_lineno,'arrays':metadata,'scalar_locals':locals_seen})
 return values
manifest=json.loads((a.inputs/'manifest.json').read_text());results=[]
for case in manifest['cases']:
 name=case['name'];snap,data=oracle.load_verified(a.inputs/name);args=oracle.restore_arrays(snap['inputs'],data);expected=oracle.restore_arrays(snap['report'],data)
 assert sha(a.inputs/name/'manifest.json')==case['oracle_manifest_sha256']
 assert all(sha(a.root/n)==h for n,h in snap['provenance']['sources'].items())
 old_svd=np.linalg.svd;old_eigh=np.linalg.eigh
 if name=='base':
  def svd(*args,**kwargs):return record('SVD',old_svd,*args,**kwargs)
  def eigh(*args,**kwargs):return record('eigh',old_eigh,*args,**kwargs)
  np.linalg.svd=svd;np.linalg.eigh=eigh
 try:actual=oracle.analyze_reference(args)
 finally:np.linalg.svd=old_svd;np.linalg.eigh=old_eigh
 payload=json.dumps(encode(actual),sort_keys=True,separators=(',',':'),allow_nan=False).encode();(a.output/(name+'-complete-report.json')).write_bytes(payload)
 diff=first_difference(expected,actual)
 results.append({'case':name,'matches_frozen_report':diff is None,'first_difference':diff,'actual_report_sha256':hashlib.sha256(payload).hexdigest(),'grasp_axes':[encode(g['closing_axis']) for item in actual['objects'] for g in item['grasps']]})
 print(json.dumps(results[-1]),flush=True)
np.savez_compressed(a.output/'base-linear-algebra-calls.npz',**arrays)
libraries={}
for line in Path('/proc/self/maps').read_text().splitlines():
 value=line.split()[-1]
 if value.startswith('/') and '.so' in value and any(s in value for s in ['numpy','scipy','cv2','opencv','libpython','libstdc++','libgomp','libc.so','libm.so']):
  f=Path(value)
  if f.is_file():libraries[value]={'sha256':sha(f),'bytes':f.stat().st_size}
summary={'scope':'READ-ONLY numerical portability trace; not an admitted benchmark or solver change','algorithm_unchanged':True,'factor_prototype_used':False,'runtime':{'host':platform.node(),'python':sys.version,'numpy':np.__version__,'opencv':cv2.__version__,'scipy':scipy.__version__},'source_pins':{n:sha(a.root/n) for n in oracle.SOURCES},'script_sha256':sha(Path(__file__)),'loaded_libraries':libraries,'cases':results,'base_call_trace':calls,'base_arrays_sha256':sha(a.output/'base-linear-algebra-calls.npz'),'all_reports_match':all(x['matches_frozen_report'] for x in results)}
(a.output/'result.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
