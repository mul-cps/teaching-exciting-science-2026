#!/usr/bin/env python3
from pathlib import Path
import subprocess,json,shutil,resource
R=Path(__file__).resolve().parent
source=(R/'swap_probe.c').read_text()
mutants={
 'missing_real_swap':source.replace('    real_swap(display,drawable);','    /* missing forwarded call */'),
 'merged_drawables':source.replace(' && slots[i].drawable==drawable',''),
 'wrong_dimensions':source.replace('query_drawable(d,x,GLX_WIDTH,width);','query_drawable(d,x,GLX_HEIGHT,width);'),
 'heap_in_hot_path':source.replace('#include <errno.h>','#include <errno.h>\n#include <stdlib.h>').replace('    pthread_once(&once,resolve);','    free(malloc(1));\n    pthread_once(&once,resolve);'),
 'slot_bound_overrun':source.replace('i<SLOT_COUNT','i<=SLOT_COUNT'),
}
results={}
for name,text in mutants.items():
 assert text!=source
 d=R/'mutants'/name;d.mkdir(parents=True,exist_ok=True)
 (d/'swap_probe.c').write_text(text);shutil.copy2(R/'test_probe.py',d/'test_probe.py')
 for lib in ('libaudit_heap.so','fake_host','minimal_host','missing_host','libfake_gl.so','libfake_minimal.so'):
  p=d/lib
  if not p.exists():p.symlink_to(R/lib)
 subprocess.run(['gcc','-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fPIC','-shared','-fsanitize=undefined','-fsanitize-undefined-trap-on-error',str(d/'swap_probe.c'),'-o',str(d/'libswap_probe.so'),'-ldl','-pthread'],check=True)
 def no_core():resource.setrlimit(resource.RLIMIT_CORE,(0,0))
 p=subprocess.run(['python3',str(d/'test_probe.py')],capture_output=True,text=True,timeout=20,preexec_fn=no_core,cwd=d)
 (d/'test.log').write_text(p.stdout+p.stderr)
 assert p.returncode!=0,name
 results[name]={'rejected':True,'test_exit':p.returncode,'log':str((d/'test.log').relative_to(R))}
(R/'mutants.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
