exec(compile((__import__('pathlib').Path(__file__).with_name('verify_inputs.py')).read_text(), 'verify_inputs.py','exec'))
import struct
cpu=load(B/'roi-qualification/roi-cpu-proof.json');final=load(B/'roi-qualification-final-proof.json');assert final['cpu_proof_sha256']==sha(B/'roi-qualification/roi-cpu-proof.json')
assert len(cpu['cases'])==7
for host,precision,identity in [('intel','software','remote'),('amd','native','local')]:
 build=load(B/(identity+'-binary-pins.json'));libs={x['path']:x['sha256'] for x in build['libraries']};rows=[]
 assert set(final['hosts'][host])==set(cpu['cases'])
 for name,expected in cpu['cases'].items():
  corpus=B/'roi-qualification'/name;directory=B/(host+'-roi-results')/name;run=load(directory/'run.json');comp=load(directory/'comparison.json');bound=final['hosts'][host][name]
  assert sha(corpus/'manifest.json')==expected['manifest_sha256'];assert run['execution_complete'] and run['smoke'] and not run['dry_run'] and len(run['trials'])==1 and run['build_identity']==build
  assert comp['all_matched'] and comp['cases']==1 and comp['acceptance_python']=='/tmp/workshop-test-pinned313-20261027/bin/python'
  trial=run['trials'][0];d=directory/trial['directory'];r=load(d/'result.json');w=trial['watchdog'];assert trial['case']=='scoped-640x480' and trial['mutation']==0 and trial['identities_verified']
  assert w['returncode']==0 and not w['timed_out'] and not w['interrupted'] and w['reaped'] and w['pid_pinned_until_final_group_signal']
  assert all(sha(d/f)==digest for f,digest in trial['artifacts'].items())
  assert sha(directory/'comparison.json')==bound['comparison_sha256'] and sha(d/'result.json')==bound['report_sha256']
  assert r['executable_sha256']==build['executable']['sha256'] and {x['path']:x['sha256'] for x in r['loaded_libraries']}==libs
  assert r['int64_enabled'] and r['float64_enabled'] is (precision=='native')
  assert module.compare(corpus,'scoped-640x480',d/'result',precision,0)==comp['results'][0]
  for key in ['roi_pixels','closed_pixels','flags']:assert bound[key]==expected[key]
  assert bound['unchanged_original_and_order'] and expected['original_and_order_unchanged']
  assert bound['roi_ieee754_u64']==[hex(struct.unpack('>Q',struct.pack('>d',v))[0]) for v in bound['roi']]==expected['roi_ieee754_u64']
  rows.append({'name':name,'matched':True,'comparison_sha256':sha(directory/'comparison.json'),'report_sha256':sha(d/'result.json'),'watchdog_sha256':sha(d/'watchdog.json'),'original_membership_sha256':bound['original_membership_sha256']})
 assert len(set(x['original_membership_sha256'] for x in rows))==1
 proof['hosts']['intel-software' if host=='intel' else 'amd-native']['roi_boundary_cases']=rows
proof['scope']='observed preparation: 28 normal, 15 malformed and seven ROI boundary cases per host';proof['total_planned_trials_verified']=100;proof['roi_final_proof_sha256']=sha(B/'roi-qualification-final-proof.json')
(OUT/'verified-inputs.json').write_text(json.dumps(proof,indent=2)+'\n');print('Verified all100 planned actual trials, including14 ROI numerical/identity/watchdog comparisons.')
