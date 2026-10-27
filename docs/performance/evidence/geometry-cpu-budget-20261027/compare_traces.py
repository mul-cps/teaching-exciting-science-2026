from pathlib import Path
import json,hashlib
import numpy as np
B=Path('/tmp/full-geometry-cpu-budget-20261027');rows=[]
for pair,meta_key,npz in [('portability','base_call_trace','base-linear-algebra-calls.npz'),('failing-case-traces','call_trace','linear-algebra-calls.npz')]:
 left=json.loads((B/('local-'+pair)/'result.json').read_text());right=json.loads((B/('remote-'+pair)/'result.json').read_text());la=np.load(B/('local-'+pair)/npz,allow_pickle=False);ra=np.load(B/('remote-'+pair)/npz,allow_pickle=False)
 assert len(left[meta_key])==len(right[meta_key])
 for i,(l,r) in enumerate(zip(left[meta_key],right[meta_key])):
  assert l['caller']==r['caller'] and l['kind']==r['kind']
  case='base' if pair=='portability' else ('nonplanar-two-face' if i<13 else 'sensor-edge')
  differences={}
  for key,x in l['arrays'].items():
   assert key in r['arrays'];aa=la[x['array']];bb=ra[r['arrays'][key]['array']];assert aa.shape==bb.shape
   differences[key]={'bit_exact':np.array_equal(aa.view(np.uint8),bb.view(np.uint8)),'max_absolute_error':float(np.max(np.abs(aa.astype(float)-bb))) if aa.size else 0.,'local_sha256':x['sha256'],'remote_sha256':r['arrays'][key]['sha256']}
  aa=la[l['arrays']['input']['array']];bb=ra[r['arrays']['input']['array']]
  row={'case':case,'call_index':i,'caller':l['caller'],'kind':l['kind'],'input_shape':list(aa.shape),'differences':differences,'scalar_locals_local':l['scalar_locals'],'scalar_locals_remote':r['scalar_locals']}
  if l['kind']=='SVD':
   singular_left=la[l['arrays']['output1']['array']];singular_right=ra[r['arrays']['output1']['array']]
   row.update(singular_values_local=singular_left.tolist(),singular_values_remote=singular_right.tolist(),relative_leading_singular_gap_local=float(abs(singular_left[0]-singular_left[1])/singular_left[0]),relative_leading_singular_gap_remote=float(abs(singular_right[0]-singular_right[1])/singular_right[0]),VT_local=la[l['arrays']['output2']['array']].tolist(),VT_remote=ra[r['arrays']['output2']['array']].tolist())
   for label,source,call,matrix in [('local',la,l,aa),('remote',ra,r,bb)]:
    u=source[call['arrays']['output0']['array']];sv=source[call['arrays']['output1']['array']];vt=source[call['arrays']['output2']['array']];row[label+'_relative_reconstruction_error']=float(np.linalg.norm(matrix-(u*sv)@vt)/np.linalg.norm(matrix))
  rows.append(row)
proof={'scope':'Read-only comparison of retained calls, not a solver or accepted benchmark','case_correlation':'Base trace contains13 calls. Focused trace canonical command order: first13 calls nonplanar-two-face, last11 sensor-edge. Raw focused metadata case label was shadowed by array-name variable; numeric arrays/caller order unchanged. This comparison derives case from verified canonical order and caller sequence.','rows':rows}
(B/'linear-algebra-comparison.json').write_text(json.dumps(proof,indent=2)+'\n')
for r in rows:
 if r['caller']!='_weighted_plane':print(json.dumps({k:r[k] for k in ['case','call_index','caller','input_shape','relative_leading_singular_gap_local','relative_leading_singular_gap_remote','singular_values_local','singular_values_remote'] if k in r}|{'input':r['differences']['input'],'VT_delta':r['differences'].get('output2'),'reconstruction':[r.get('local_relative_reconstruction_error'),r.get('remote_relative_reconstruction_error')]}))
