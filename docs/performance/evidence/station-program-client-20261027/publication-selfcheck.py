from pathlib import Path
import json,hashlib,subprocess,io,tarfile
from zipfile import ZipFile
R=Path(__file__).resolve().parents[4];D=R/'docs';E=D/'performance/evidence/station-program-client-20261027'
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
d=load(D/'station-program-client-checkpoint-20261027.json');commit=d['source_commit'];assert isinstance(commit,str) and len(commit)==40,'Source commit must be finalized'
pins=load(E/'source-pins.json');assert pins==d['source_pins']
for row in load(E/'readout-inventory.json')['files']:
 payload=R/row['path']
 assert payload.stat().st_size==row['bytes'] and sha(payload)==row['sha256'],row['path']
materials=load(E/'materials-build.json');nb=load(E/'notebook-proof.json')
names=sorted(set(pins)|set(materials)|{'notebooks/'+n for n in nb['notebooks']})
s=io.BytesIO(subprocess.check_output(['git','-C',str(R),'cat-file','--batch'],input=''.join(commit+':'+n+'\n' for n in names).encode()));blobs={}
for n in names:
 header=s.readline().split();assert len(header)==3 and header[1]==b'blob',(n,header)
 blobs[n]=s.read(int(header[2]));assert s.read(1)==b'\n'
for n,h in pins.items():assert hashlib.sha256(blobs[n]).hexdigest()==h,n
review=load(E/'independent-review-proof.json');native=load(E/'wrapper-final.json');assert native['source_before']==native['source_after']==review['source_pins']
assert native['returncode']==0 and native['reaped'] and not native['timed_out']
assert sha(E/'wrapper-final.json')==review['native_fixture']['receipt_sha256'];assert native['log_sha256']==review['native_fixture']['log_sha256']
assert all(pins[n]==h for n,h in native['source_after'].items())
for n in ['blockly_client_native.json','blockly_external_control_native.json']:
 x=load(E/n);assert x['returncode']==0 and x['reaped'] and not x['timed_out']
canonical=load(E/'canonical-cpu-receipt.json');assert canonical['passed']==4527 and canonical['skipped']==133 and canonical['subtests_passed']==48
assert '84 passed in 21.12s' in (E/'focused-final-cpu.log').read_text();assert review['independent_focused_tests']['observed_stdout']=='35 passed in 20.41s'
assert len(review['static_mutants_rejected'])==4;assert sum(x['n'] for x in review['host_microbenchmark_summary'].values())==43
materials=load(E/'materials-build.json');assert len(materials)==116
for n,h in materials.items():assert hashlib.sha256(blobs[n]).hexdigest()==h,n
bundle=load(E/'generated-materials-proof.json');assert bundle['ZIP_entries_changed']==['workshop/blockly_client.py'];assert set(bundle['materials_receipt_entries_changed'])=={'docs/student-workshop.zip','workshop/blockly_client.py'}
with ZipFile(io.BytesIO(blobs['docs/student-workshop.zip'])) as z:assert z.read('workshop/blockly_client.py')==blobs['workshop/blockly_client.py']
nb=load(E/'notebook-proof.json');assert nb['canonical_code_cells_executed']==25 and len(nb['notebooks'])==3 and nb['fresh_kernel_all_three'] and nb['live_unavailable_no_DEMO_fallback'] and nb['routing_variants_passed']
for n,x in nb['notebooks'].items():assert hashlib.sha256(blobs['notebooks/'+n]).hexdigest()==x['source_sha256']
archive=load(E/'private-archive-inventory.json');assert sha(Path(archive['path']))==archive['sha256'];expected={x['path']:x for x in archive['members']}
with tarfile.open(archive['path']) as t:
 assert {x.name for x in t}==set(expected)
 for n,x in expected.items():
  data=t.extractfile(n).read();assert len(data)==x['bytes'] and hashlib.sha256(data).hexdigest()==x['sha256'];assert data[:4]!=b'\x7fELF'
 import xml.etree.ElementTree as ET
 log=t.extractfile('cpu/canonical-final.log').read();xml=t.extractfile('cpu/canonical-final.xml').read()
 assert hashlib.sha256(log).hexdigest()==canonical['raw_log_sha256']
 assert hashlib.sha256(xml).hexdigest()==canonical['raw_junit_sha256']
 suite=ET.fromstring(xml).find('testsuite');assert suite is not None
 assert suite.get('errors')=='0' and suite.get('failures')=='0' and suite.get('skipped')=='133'
assert not d['qualified'] and not d['station_qualified'] and not d['physical_STOP_verified'] and not d['release_qualified']
print(json.dumps({'passed':True,'source_commit':commit,'immutable_source_and_generated_blobs_verified':len(pins),'materials_hashes_verified':116,'canonical_cpu_passed':4527,'canonical_skips_not_passes':133,'canonical_subtests_passed':48,'native_direct_scripts_passed':2,'native_wrapper_modes_passed':4,'independent_host_trials':43,'canonical_notebook_cells':25,'private_archive_members_verified':len(expected),'private_archive_sha256':archive['sha256'],'historical_payloads_from_committed_Git_only':True,'frozen_readout_inventory_verified':True,'qualified':False},indent=2))
