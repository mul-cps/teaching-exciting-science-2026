#!/usr/bin/env python3
"""Single-job offline build with external dependency prefixes and recorded caps."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import time

from certify import bounded_read, sealed_snapshot, CGAL_VERSION


def compile_frozen(source_bytes, command, *, deadline, limits, env):
    with sealed_snapshot(source_bytes, 'recovery-cgal-source') as source_fd:
        frozen_command=[arg.replace('{source}',f'/proc/self/fd/{source_fd}') for arg in command]
        child=subprocess.Popen(frozen_command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                               start_new_session=True,preexec_fn=limits,env=env,pass_fds=(source_fd,))
        try:
            output,_=child.communicate(timeout=max(.001,deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGKILL)
            output,_=child.communicate()
        return child.returncode,output,frozen_command


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dependency-prefix',type=Path,required=True,help='External CGAL/MPFR overlay')
    p.add_argument('--base-prefix',type=Path,required=True,help='Read-only GCC/Boost/GMP conda prefix')
    p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args()
    overlay=args.dependency_prefix.resolve(); base=args.base_prefix.resolve()
    dest=args.output_dir.resolve(); dest.mkdir(parents=True,exist_ok=True)
    source=Path(__file__).with_name('main.cpp').resolve()
    source_bytes=bounded_read(source,1024**2,time.monotonic()+1.)
    compiler=base/'bin/g++'
    command=[str(compiler),'-std=c++17','-O1','-DNDEBUG','-frounding-math',
             '-I'+str(overlay/'include'),'-I'+str(base/'include'),'-x','c++','{source}','-x','none',
             '-L'+str(overlay/'lib'),'-L'+str(base/'lib'),
             '-Wl,-rpath,'+str(overlay/'lib'),'-Wl,-rpath,'+str(base/'lib'),
             '-lmpfr','-lgmp','-o',str(dest/'recovery-cgal')]
    def limits():
        resource.setrlimit(resource.RLIMIT_AS,(8*1024**3,)*2)
        resource.setrlimit(resource.RLIMIT_CPU,(170,170))
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        os.nice(10)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    started=time.monotonic()
    returncode,output,frozen_command=compile_frozen(source_bytes,command,deadline=started+180,limits=limits,env=env)
    (dest/'build.log').write_bytes(output)
    metadata=[]
    for path in sorted((base/'conda-meta').glob('*.json')):
        row=json.loads(path.read_text())
        if row.get('name')in {'gmp','libboost','libboost-devel','libboost-headers','libgcc','libstdcxx','gcc','gcc_impl_linux-64','gxx_impl_linux-64','eigen','eigen-abi'}:
            metadata.append({k:row.get(k)for k in ('name','version','build','sha256','url')})
    result={'command':command,'executed_command':frozen_command,'source_sha256':hashlib.sha256(source_bytes).hexdigest(),
            'cgal_version':CGAL_VERSION,'source_compiled_from_sealed_snapshot':True,
            'compiler_version':subprocess.check_output([str(compiler),'--version'],text=True).splitlines()[0],
            'base_dependency_packages':metadata,'overlay_prefix':str(overlay),
            'elapsed_s':time.monotonic()-started,
            'child_peak_rss_kib':resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
            'limits':{'virtual_memory_bytes':8*1024**3,'cpu_s':170,'wall_s':180,'compiler_jobs':1},
            'returncode':returncode,'live_environment_mutated':False}
    if returncode==0:
        result['binary_sha256']=hashlib.sha256((dest/'recovery-cgal').read_bytes()).hexdigest()
    (dest/'build.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if returncode==0 else 2


if __name__=='__main__':
    raise SystemExit(main())
