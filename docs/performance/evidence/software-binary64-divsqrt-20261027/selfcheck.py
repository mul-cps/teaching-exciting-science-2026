"""Historical evidence check only; no numerical, GPU, ROS or hardware execution."""
from pathlib import Path
import hashlib,json,subprocess,tarfile
E=Path(__file__).resolve().parent
R=E.parents[3]
def digest(data):return hashlib.sha256(data).hexdigest()
manifest=json.loads((E/'inventory.json').read_text())
for name,row in manifest.items():
 data=(R/name).read_bytes()
 assert len(data)==row['bytes'] and digest(data)==row['sha256'],name
readout=json.loads((R/'docs/performance/software-binary64-divsqrt-20261027.json').read_text())
commit=readout['source_commit']
assert commit and len(commit)==40
sources=json.loads((E/'source-pins.json').read_text())
for name,sha in sources.items():
 blob=subprocess.check_output(['git','show',commit+':'+name],cwd=R)
 assert digest(blob)==sha,name
archive=json.loads((E/'private-archive.json').read_text())
p=Path(archive['path']);checked=False
if p.exists():
 assert p.stat().st_size==archive['bytes'] and digest(p.read_bytes())==archive['sha256']
 with tarfile.open(p) as t:
  for row in archive['members']:
   member=t.getmember(row['name']);data=t.extractfile(member).read()
   assert len(data)==row['bytes'],row['name']
   if 'sha256' in row:assert digest(data)==row['sha256'],row['name']
   assert not data.startswith(b'\x7fELF'),row['name']
 checked=True
print(json.dumps({'selected_files':len(manifest),'immutable_source_commit':commit,'source_blobs':len(sources),'private_archive_checked':checked,'numerical_or_GPU_execution':False}))
