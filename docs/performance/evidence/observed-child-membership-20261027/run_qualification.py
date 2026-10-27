#!/usr/bin/env python3
"""On-target bounded process-group watchdog; run locally ON the GPU host.
No SSH launcher and no GPU execution in --dry-run, compare, or self-test.
"""
import argparse
import collections
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

ABORT_SIGNAL=None

def check_abort():
    if ABORT_SIGNAL is not None:raise InterruptedError("watchdog interrupted by signal "+str(ABORT_SIGNAL))



def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(65536),b''):h.update(block)
    return h.hexdigest()


def write(path,value):
    path=Path(path);temp=path.with_name(path.name+'.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def require(condition,reason):
    if not condition:raise ValueError(reason)


def watch(command,directory,timeout=120,grace=2):
    """Keep the leader UNREAPED until all group signaling is finished.

    WNOWAIT pins its PID (even if zombie), avoiding recycled integer-PGID races.
    Timeout TERM is followed by KILL before wait/reap. Native qualifier creates
    no intentional subprocesses; kill remaining group members on normal exit too.
    """
    directory=Path(directory);started=time.monotonic();p=None;timed_out=False;interrupted=False;actions=[];reaped=False;rc=None
    with (directory/'stdout.log').open('xb') as stdout,(directory/'stderr.log').open('xb') as stderr:
        try:
            check_abort()
            p=subprocess.Popen(command,stdout=stdout,stderr=stderr,start_new_session=True)
            write(directory/'process.json',dict(pid=p.pid,pgid=p.pid,host=socket.gethostname(),command=command,timeout_seconds=timeout))
            while True:
                check_abort()
                exited=os.waitid(os.P_PID,p.pid,os.WEXITED|os.WNOHANG|os.WNOWAIT)
                if exited is not None:break
                if time.monotonic()-started>=timeout:
                    timed_out=True
                    try:os.killpg(p.pid,signal.SIGTERM);actions.append('TERM')
                    except ProcessLookupError:pass
                    end=time.monotonic()+grace
                    while time.monotonic()<end:
                        if os.waitid(os.P_PID,p.pid,os.WEXITED|os.WNOHANG|os.WNOWAIT) is not None:break
                        time.sleep(.01)
                    break
                time.sleep(.01)
        except BaseException:
            interrupted=True
            raise
        finally:
            if p is not None:
                # No poll(), wait(), communicate() or SIGCHLD handler may reap
                # before this last group signal; leader PID is still reserved.
                try:os.killpg(p.pid,signal.SIGKILL);actions.append('KILL-before-reap')
                except ProcessLookupError:pass
                try:rc=p.wait(timeout=5);reaped=True
                except subprocess.TimeoutExpired:actions.append('unreaped-after-KILL-deadline')
            proof=dict(command=command,returncode=rc,timed_out=timed_out,interrupted=interrupted or ABORT_SIGNAL is not None,reaped=reaped,signals=actions,elapsed_wall_seconds=time.monotonic()-started,watchdog_host=socket.gethostname(),pid_pinned_until_final_group_signal=True)
            write(directory/'watchdog.json',proof)
    return proof


def shaders(directory):
    directory=Path(directory);source=(directory/'source.sha256').read_text().strip();require(len(source)==64,'source identity length');spv={}
    for file in sorted(directory.glob('*.spv')):
        h=digest(file);require((directory/(file.name+'.sha256')).read_text()==h+'  '+file.name+'\n','SPIR-V sidecar mismatch '+str(file));spv[file.name]=h
    require(spv,'empty shader cache')
    return dict(path=str(directory.resolve()),source_sha256=source,spirv=spv,provenance=(directory/'provenance.txt').read_text() if (directory/'provenance.txt').exists() else None)


def build_identity(path,executable):
    m=json.loads(Path(path).read_text());require(len(m.get('source_archive_sha256',''))==64,'source archive SHA pin required');require(Path(m['executable']['path']).resolve()==Path(executable).resolve(),'build executable path mismatch');require(digest(executable)==m['executable']['sha256'],'build executable SHA mismatch');require(len(m['libraries'])==5,'exactly five module libraries required')
    for lib in m['libraries']:require(digest(lib['path'])==lib['sha256'],'build library SHA mismatch '+lib['path'])
    return m


def validate_report(report,build,caches,fixture_sha,precision,mutation):
    require(report['executable_sha256']==build['executable']['sha256'],'reported executable SHA mismatch')
    expected=collections.Counter((str(Path(x['path']).resolve()),x['sha256']) for x in build['libraries'])
    actual=collections.Counter((str(Path(x['path']).resolve()),x['sha256']) for x in report['loaded_libraries'])
    require(actual==expected,'reported FIVE loaded library paths/SHA VALUES mismatch')
    for key,cache in caches.items():
        identity=report[key];require(identity['source_sha256']==cache['source_sha256'],'reported shader source mismatch '+key)
        normalized={name if name.endswith('.spv') else name+'.spv':value for name,value in identity['spirv'].items()}
        require(normalized==cache['spirv'],'reported SPIR-V SHA values mismatch '+key)
        require(identity.get('provenance')==cache['provenance'],'reported shader provenance mismatch '+key)
    for key,value in dict(fixture_sha256=fixture_sha,precision=precision,malformation=mutation,scope='observed_preparation_only',gpu_executed=True,physical_camera=False,objects=False,world_qualified=False,unique_inputs=1,completed_outputs=0 if mutation else 1,status='unusable' if mutation else 'valid').items():require(type(report.get(key)) is type(value) and report[key]==value,'report field mismatch '+key)


def tool(repo):
    file=Path(repo)/'tools/qualify_native_observed_children.py';spec=importlib.util.spec_from_file_location('observed_acceptance',file);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def planned_trials(corpus,smoke=False):
    if smoke:return [dict(case='scoped-640x480',mutation=0,directory='00-scoped-640x480')]
    result=[dict(case=name,mutation=0,directory=f'{index:02d}-{name}') for index,name in enumerate(corpus['cases'])]
    return result+[dict(case='scoped-640x480',mutation=n,directory=f'{27+n:02d}-malformed-{n:02d}') for n in range(1,16)]


def compare_results(args):
    m=json.loads((args.results/'run.json').read_text());oracle=tool(args.repo);proof=[]
    require(m['execution_complete'] is True,'execution not complete')
    require(digest(args.build_manifest)==m['build_manifest_sha256'] and json.loads(args.build_manifest.read_text())==m['build_identity'],'comparison initial build receipt mismatch')
    corpus=json.loads((args.corpus/'manifest.json').read_text());require(m['corpus_manifest_sha256']==digest(args.corpus/'manifest.json'),'comparison corpus manifest mismatch')
    expected=planned_trials(corpus,m['smoke']);require([(t['case'],t['mutation'],t['directory']) for t in m['trials']]==[(t['case'],t['mutation'],t['directory']) for t in expected],'exact1smoke/43full coverage/order required')
    for trial in m['trials']:
        for name,sha in trial['artifacts'].items():require(digest(args.results/trial['directory']/name)==sha,'artifact changed '+name)
        prefix=args.results/trial['directory']/'result';validate_report(json.loads(Path(str(prefix)+'.json').read_text()),m['build_identity'],m['shader_identities'],corpus['cases'][trial['case']]['fixture_sha256'],m['precision'],trial['mutation']);proof.append(oracle.compare(args.corpus,trial['case'],prefix,m['precision'],trial['mutation']))
    write(args.results/'comparison.json',dict(all_matched=True,cases=len(proof),scope='observed_preparation_only',acceptance_python=sys.executable,acceptance_tool_sha256=digest(Path(args.repo)/'tools/qualify_native_observed_children.py'),results=proof))
    print(json.dumps(dict(all_matched=True,cases=len(proof))))


def run(args):
    require(0<args.timeout<=120 and 0<args.grace<=5,'watchdog bound must be <=120s + <=5s TERM grace')
    corpus=json.loads((args.corpus/'manifest.json').read_text());require(corpus['schema']=='native-observed-diagnostic-v1' and len(corpus['cases'])==28,'expected frozen28 corpus')
    build=build_identity(args.build_manifest,args.executable)
    caches={name:shaders(value) for name,value in dict(parent_shader_identity=args.parents,child_shader_identity=args.children,geometry_shader_identity=args.geometry,observed_shader_identity=args.observed).items()}
    require([len(caches[k]['spirv']) for k in caches]==[9,21,5,6],'exact kernel counts9/21/5/6 required')
    trials=planned_trials(corpus,args.smoke)
    for trial in trials:
        case=corpus['cases'][trial['case']]
        for key in ('fixture','expected'):require(Path(case[key]).name==case[key] and digest(args.corpus/case[key])==case[key+'_sha256'],'corpus artifact hash '+key)
        command=[str(args.executable),'--vulkan',str(args.corpus/case['fixture']),str(args.parents),str(args.children),str(args.geometry),str(args.observed),str(args.results/trial['directory']/'result'),args.precision,str(trial['mutation']),'--external-watchdog-ack']
        if corpus['roi'] is not None:command+=list(map(repr,corpus['roi']))
        trial['command']=command
    args.results.mkdir(parents=True,exist_ok=False)
    report=dict(schema='observed-watchdog-v1',host=socket.gethostname(),precision=args.precision,runner_sha256=digest(__file__),corpus_manifest_sha256=digest(args.corpus/'manifest.json'),build_manifest_sha256=digest(args.build_manifest),build_identity=build,shader_identities=caches,environment={k:os.environ.get(k) for k in ('VK_ICD_FILENAMES','VK_DRIVER_FILES','LD_LIBRARY_PATH')},execution_complete=False,numerically_compared=False,smoke=args.smoke,dry_run=args.dry_run,trials=trials)
    write(args.results/'run.json',report)
    if args.dry_run:print(json.dumps(dict(dry_run=True,planned_children=len(trials),gpu_executed=False)));return
    try:
        for trial in trials:
            check_abort()
            directory=args.results/trial['directory'];directory.mkdir();case=corpus['cases'][trial['case']]
            proof=watch(trial['command'],directory,args.timeout,args.grace);trial['watchdog']=proof
            require(proof['reaped'] and not proof['timed_out'] and not proof['interrupted'] and proof['returncode']==0,'child failed/timeout '+trial['directory'])
            native=json.loads((directory/'result.json').read_text());require(native['qualifier_source_sha256']==corpus['sources']['ros2/src/so101_station_vulkan/test/qualify_observed_children.cpp'] and native['qualifier_helper_source_sha256']==corpus['sources']['tools/performance/qualify_children.cpp'],'qualifier/helper source pin mismatch');validate_report(native,build,caches,case['fixture_sha256'],args.precision,trial['mutation']);trial['identities_verified']=True
            # Recheck actual files to detect changes during this execution.
            require(digest(args.corpus/'manifest.json')==report['corpus_manifest_sha256'],'corpus manifest changed during run')
            require(digest(args.build_manifest)==report['build_manifest_sha256'],'build receipt changed during run')
            require(build_identity(args.build_manifest,args.executable)==build,'build identity drift during run')
            trial['artifacts']={file.name:digest(file) for file in sorted(directory.iterdir()) if file.is_file()}
            if args.repo:trial['comparison']=tool(args.repo).compare(args.corpus,trial['case'],directory/'result',args.precision,trial['mutation'])
            write(args.results/'run.json',report)
        report['execution_complete']=True;write(args.results/'run.json',report)
    except BaseException as e:
        report['failure']=dict(type=type(e).__name__,reason=str(e),stop_on_first_failure=True);write(args.results/'failure.json',report['failure']);write(args.results/'run.json',report);raise
    if args.repo:compare_results(args)
    else:print(json.dumps(dict(execution_complete=True,numerically_compared=False,required_next_step='copy artifacts to pinned local Python and run compare')))


def self_test():
    global ABORT_SIGNAL
    with tempfile.TemporaryDirectory(prefix='observed-watchdog-test-') as d:
        p=Path(d);(p/'success').mkdir();a=watch([sys.executable,'-c','print("CPU-only child")'],p/'success',1,.05);require(a['returncode']==0 and a['reaped'],'success watchdog')
        (p/'timeout').mkdir();b=watch([sys.executable,'-c','import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(10)'],p/'timeout',.1,.05);require(b['timed_out'] and b['reaped'] and b['returncode']==-signal.SIGKILL,'timeout watchdog')
        # Shape-only/library substitution must fail even with correct list length.
        try:validate_report({'executable_sha256':'x','loaded_libraries':[dict(path='/tmp/wrong',sha256='0'*64)]*5},{'executable':{'sha256':'x'},'libraries':[dict(path='/tmp/right',sha256='1'*64)]*5},{},'', 'software',0)
        except ValueError:pass
        else:raise ValueError('library value mutant admitted')
        interrupted=p/'interrupted';interrupted.mkdir()
        with (p/'interrupt-runner.log').open('w') as log:
            runner=subprocess.Popen([sys.executable,__file__,'self-test-interrupt-child',str(interrupted)],stdout=log,stderr=log)
            deadline=time.monotonic()+3
            while not (interrupted/'process.json').exists():
                require(time.monotonic()<deadline,'interrupt self-test startup deadline');time.sleep(.01)
            pid=json.loads((interrupted/'process.json').read_text())['pid'];runner.send_signal(signal.SIGHUP);require(runner.wait(timeout=3)!=0,'interrupted runner must fail')
        cleanup=json.loads((interrupted/'watchdog.json').read_text());require(cleanup['interrupted'] and cleanup['reaped'],'interrupted runner cleanup')
        try:os.kill(pid,0)
        except ProcessLookupError:pass
        else:raise ValueError('interrupted child survived')
        original_popen=subprocess.Popen;original_killpg=os.killpg
        try:
            def interrupted_spawn(*a,**kw):
                child=original_popen(*a,**kw)
                for _ in range(3):os.kill(os.getpid(),signal.SIGHUP)
                return child
            subprocess.Popen=interrupted_spawn;(p/'spawn-boundary').mkdir()
            try:watch([sys.executable,'-c','import time;time.sleep(10)'],p/'spawn-boundary',1,.05)
            except InterruptedError:pass
            else:raise ValueError('spawn-boundary abort lost')
            require(json.loads((p/'spawn-boundary/watchdog.json').read_text())['reaped'],'spawn-boundary child survived')
            subprocess.Popen=original_popen;ABORT_SIGNAL=None
            def interrupted_cleanup(pid,number):
                for _ in range(3):os.kill(os.getpid(),signal.SIGHUP)
                return original_killpg(pid,number)
            os.killpg=interrupted_cleanup;(p/'cleanup-boundary').mkdir();c=watch([sys.executable,'-c','pass'],p/'cleanup-boundary',1,.05);require(c['reaped'] and c['interrupted'],'cleanup-boundary interrupted cleanup')
        finally:subprocess.Popen=original_popen;os.killpg=original_killpg;ABORT_SIGNAL=None
        print(json.dumps(dict(self_test=True,gpu_executed=False,success=a,timeout=b)))


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True);sub.add_parser('self-test');a=sub.add_parser('self-test-interrupt-child');a.add_argument('directory',type=Path);a=sub.add_parser('run')
    for name in ('executable','parents','children','geometry','observed','corpus','results','build-manifest'):a.add_argument('--'+name,type=Path,required=True)
    a.add_argument('--precision',choices=('native','software'),required=True);a.add_argument('--timeout',type=float,default=120);a.add_argument('--grace',type=float,default=2);a.add_argument('--dry-run',action='store_true');a.add_argument('--smoke',action='store_true',help='only one scoped640x480 valid case, before full43 matrix');a.add_argument('--repo',type=Path,help='only use pinned local acceptance Python; omit on remote and compare locally later')
    a=sub.add_parser('compare');a.add_argument('--build-manifest',type=Path,required=True);a.add_argument('--repo',type=Path,required=True);a.add_argument('--corpus',type=Path,required=True);a.add_argument('--results',type=Path,required=True)
    args=p.parse_args()
    def interrupt(number,frame):
        global ABORT_SIGNAL
        ABORT_SIGNAL=number
    for number in (signal.SIGTERM,signal.SIGHUP,signal.SIGINT):signal.signal(number,interrupt)
    if args.mode=='self-test-interrupt-child':
        watch([sys.executable,'-c','import time;time.sleep(10)'],args.directory,1,.05);return
    if args.mode=='self-test':self_test()
    elif args.mode=='run':run(args)
    else:compare_results(args)
if __name__=='__main__':
    try:main()
    except Exception as e:print(type(e).__name__+': '+str(e),file=sys.stderr);raise SystemExit(1)
