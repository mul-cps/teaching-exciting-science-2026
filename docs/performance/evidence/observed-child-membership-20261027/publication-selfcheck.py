from pathlib import Path
import json,hashlib,tarfile,subprocess,io
R=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022');D=R/'docs/performance';E=D/'evidence/observed-child-membership-20261027'
def load(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
pins=load(E/'source-pins.json');commit=load(D/'observed-child-membership-20261027.json')['source_commit'];out=subprocess.run(['git','-C',str(R),'cat-file','--batch'],input=''.join(commit+':'+f+'\n' for f in pins).encode(),capture_output=True,check=True);s=io.BytesIO(out.stdout)
for name,digest in pins.items():
 h=s.readline().split();assert h[1]==b'blob';assert hashlib.sha256(s.read(int(h[2]))).hexdigest()==digest;assert s.read(1)==b'\n'
assert len(pins)==119
cache=load(E/'observed-cache-pins.json')
with tarfile.open('/tmp/observed-children-native-20261027/observed-caches.tar.gz') as t:
 assert {x.name for x in t.getmembers() if x.isfile()}==set(cache)
 for n,h in cache.items():assert hashlib.sha256(t.extractfile(n).read()).hexdigest()==h
assert len(cache)==46
inv=load(E/'private-archive-inventory.json');p=Path(inv['archive_path']);assert sha(p)==inv['archive_sha256'];members={v['path']:v for v in inv['members']}
with tarfile.open(p) as t:
 assert {x.name for x in t.getmembers()}==set(members)
 for f in t:
  content=t.extractfile(f).read();assert len(content)==members[f.name]['bytes'] and hashlib.sha256(content).hexdigest()==members[f.name]['sha256'];assert content[:4]!=b'\x7fELF'
for x in load(E/'inventory.json')['files']:assert sha(E/x['path'])==x['sha256']
v=load(E/'independent-verification.json');assert v['git_blobs_verified']==119 and v['total_planned_trials_verified']==100
for h in v['hosts'].values():assert len(h['cases'])==43 and len(h['roi_boundary_cases'])==7 and all(c['matched'] for c in h['cases']+h['roi_boundary_cases'])
d=load(D/'observed-child-membership-20261027.json');assert not d['qualified'] and d['actual_gpu_matrix']['validated'] and d['actual_gpu_matrix']['total_planned_cases']==100
subprocess.run(['git','-C',str(R),'diff','--check'],check=True)
proof={'passed':True,'source_commit':commit,'immutable_git_blobs_verified':119,'cache_archive_pins_verified':46,'raw_archive_members_verified':len(members),'raw_archive_sha256':inv['archive_sha256'],'planned_trials_numerically_recomputed_and_identity_checked':100,'actual_evidence_review_sha256':sha(E/'actual-evidence-review.json'),'selected_inventory_hashes_verified':True,'diff_check_passed':True,'no_GPU_SSH_USB':True,'qualification':False}
(E/'publication-selfcheck.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof,indent=2))
