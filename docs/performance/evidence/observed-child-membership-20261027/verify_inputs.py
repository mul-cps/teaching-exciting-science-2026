from pathlib import Path
import sys,json,hashlib,subprocess,io,importlib.util
ROOT=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022');B=Path('/tmp/observed-children-native-20261027');OUT=Path('/tmp/observed-child-membership-publication-20261027');COMMIT='ccbf7a72809f9b00c182d05ac02152e0f88f734f'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
pins=load(B/'source-pins.json');names=list(pins)
r=subprocess.run(['git','-C',str(ROOT),'cat-file','--batch'],input=''.join(COMMIT+':'+f+'\n' for f in names).encode(),capture_output=True,check=True);s=io.BytesIO(r.stdout)
for f in names:
 head=s.readline().split();assert len(head)==3 and head[1]==b'blob',(f,head)
 data=s.read(int(head[2]));assert s.read(1)==b'\n';assert hashlib.sha256(data).hexdigest()==pins[f],f
assert len(pins)==119
sys.path.insert(0,str(ROOT/'tools'));spec=importlib.util.spec_from_file_location('observed_acceptance',ROOT/'tools/qualify_native_observed_children.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
corpus=Path('/tmp/observed-real-qualifier-corpus-20261027');m=load(corpus/'manifest.json');assert m['roi'] is None and len(m['cases'])==28
expected=[(name,0) for name in m['cases']]+[('scoped-640x480',n) for n in range(1,16)];summaries={}
for host,precision,identity in [('intel-software','software','remote'),('amd-native','native','local')]:
 directory=B/(host+'-matrix');run=load(directory/'run.json');comparison=load(directory/'comparison.json');build=load(B/(identity+'-binary-pins.json'))
 assert run['execution_complete'] and not run['dry_run'] and not run['smoke']
 assert comparison['all_matched'] and comparison['cases']==43 and len(run['trials'])==43
 assert comparison['acceptance_python']=='/tmp/workshop-test-pinned313-20261027/bin/python'
 assert [(x['case'],x['mutation']) for x in run['trials']]==expected
 assert [(x['case'],x['mutation']) for x in comparison['results']]==expected
 assert run['build_identity']==build
 expected_libraries={x['path']:x['sha256'] for x in build['libraries']};assert len(expected_libraries)==5
 rows=[];devices=set()
 for trial,comp in zip(run['trials'],comparison['results']):
  d=directory/trial['directory'];report=load(d/'result.json');w=trial['watchdog']
  assert comp['matched'] and comp['precision']==precision and trial['identities_verified']
  assert w['returncode']==0 and not w['timed_out'] and not w['interrupted'] and w['reaped'] and w['pid_pinned_until_final_group_signal']
  assert w['command']==trial['command']
  assert all(sha(d/name)==digest for name,digest in trial['artifacts'].items())
  assert sha(d/'result.json')==comp['report_sha256']
  assert report['executable_sha256']==build['executable']['sha256']
  assert {x['path']:x['sha256'] for x in report['loaded_libraries']}==expected_libraries
  assert report['int64_enabled'] is True and report['float64_enabled'] is (precision=='native')
  assert report['qualifier_source_sha256']==pins['ros2/src/so101_station_vulkan/test/qualify_observed_children.cpp']
  actual=module.compare(corpus,trial['case'],d/'result',precision,trial['mutation']);assert actual=={k:v for k,v in comp.items() if k!='report_sha256'}|{'report_sha256':comp['report_sha256']}
  devices.add(report['device'])
  rows.append({'case':trial['case'],'mutation':trial['mutation'],'matched':True,'status':report['status'],'report_sha256':comp['report_sha256'],'watchdog_sha256':sha(d/'watchdog.json'),'executable_and_five_library_values_matched':True,'raw_artifact_hashes':trial['artifacts']})
 summaries[host]={'precision':precision,'devices':sorted(devices),'normal_exact_membership_cases':28,'malformed_unusable_cases':15,'watchdogs_reaped_no_timeouts':43,'comparison_sha256':sha(directory/'comparison.json'),'run_sha256':sha(directory/'run.json'),'cases':rows}
proof={'source_commit':COMMIT,'git_blobs_verified':len(pins),'source_archive_sha256':sha(B/'source.tar.gz'),'corpus_manifest_sha256':sha(corpus/'manifest.json'),'acceptance_python':sys.executable,'scope':'observed preparation, original no-ROI matrix only','comparison_contract':{'integer_arrays':['observed','records','counts','child_roots'],'integer_rule':'byte exact','world_rule':{'rtol':0,'atol':1e-12,'equal_nan':True}},'hosts':summaries,'full_geometry_camera_physical_qualified':False}
(OUT/'verified-inputs.json').write_text(json.dumps(proof,indent=2)+'\n');print('Verified119committedblobs and86actualtrials with pinned numerical recomparison/executable+five-library values.')
