import hashlib,json,tarfile
from pathlib import Path
root=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022')
base=Path('/tmp/workshop-highres-cpu-final-20261027-source-pins-before.json')
keys=set(json.loads(base.read_text()))
for directory in ('ros2/src/so101_station_vulkan','tests','tools/performance'):
    keys.update(str(p.relative_to(root)) for p in (root/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts and '.pytest_cache' not in p.parts)
keys.add('tools/test_cpu.py')
manifest={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sorted(keys) if (root/p).is_file()}
Path('/tmp/quality-profiles-source-pins-before-20261027.json').write_text(json.dumps(manifest,indent=2)+'\n')
native_keys=[p for p in manifest if p.startswith(('ros2/src/so101_station_vulkan/','tools/performance/'))]
with tarfile.open('/tmp/fwmini4-quality-profiles-source-20261027.tar.gz','w:gz') as archive:
    for p in native_keys: archive.add(root/p,arcname=p)
Path('/tmp/quality-profiles-native-source-pins-20261027.json').write_text(json.dumps({p:manifest[p] for p in native_keys},indent=2)+'\n')
print(json.dumps({'canonical_test_source_pins':len(manifest),'native_source_files':len(native_keys)}))
