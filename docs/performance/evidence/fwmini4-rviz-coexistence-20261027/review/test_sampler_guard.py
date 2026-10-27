from pathlib import Path
import ast,hashlib,json
p=Path('/tmp/rviz-process-sampler-20261027.py');source=p.read_text();tree=ast.parse(source)
assert hashlib.sha256(p.read_bytes()).hexdigest() == '90a3ad08c31a4a07b5cebf62033e74ef60d7165ec8af5f056def171ac4eb83f3'
assert {a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names} == {'argparse','json','os','pathlib','signal','time'}
assert '/proc/stat' in source and 'pathlib.Path(\'/proc\')/str(pid)' in source and 'targets.items()' in source
for forbidden in ('subprocess','socket','requests','ctypes','rclpy','glFinish','read_pixels','readPixels','/dev/','os.kill','glob('): assert forbidden not in source,forbidden
assert 'time.sleep(max(0,.1-(time.monotonic()-begin)))' in source
assert '\'starttime_ticks\':int(tail[19])' in source and '\'cpu_ticks\':int(tail[11])+int(tail[12])' in source
print(json.dumps({'passed':True,'source':str(p),'GPU_executed':False}))
