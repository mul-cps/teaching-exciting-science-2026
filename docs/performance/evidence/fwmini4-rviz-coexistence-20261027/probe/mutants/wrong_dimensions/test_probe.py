#!/usr/bin/env python3
"""Host fake-GL only: runs real LD_PRELOAD wrapper, no display connection."""
from pathlib import Path
import os,json,subprocess,statistics,re
R=Path(__file__).resolve().parent

def run(mode, extra=None):
    env={**os.environ,'LD_PRELOAD':str(R/'libswap_probe.so')+':'+str(R/'libaudit_heap.so'),**(extra or {})}
    p=subprocess.run([str(R/'fake_host'),mode],env=env,text=True,capture_output=True,check=True)
    return [json.loads(s) for s in p.stderr.splitlines()],p

def check():
    source=(R/'swap_probe.c').read_text()
    for forbidden in ('glFinish','glReadPixels','glGetTexImage','glClientWaitSync','glWaitSync','glXWaitGL','glXWaitX','vkDeviceWaitIdle','vkQueueWaitIdle','malloc','calloc','realloc'):
        assert not re.search(r'\b'+forbidden+r'\s*\(',source),forbidden
    rows,_=run('separate'); first=[r for r in rows if r['event']=='renderer'];summary=[r for r in rows if r['event']=='interval']
    assert len(first)==2 and len(summary)==2,rows
    assert {r['drawable'] for r in first}=={11,22}
    r=next(r for r in first if r['drawable']==11)
    assert r['vendor']=='Fake "vendor"' and r['renderer']=='Fake\\renderer\nGPU' and r['version']=='4.6 fake'
    assert r['width']==1011 and r['height']==511 and r['direct'] is True and r['metadata_complete'] is True
    a=next(r for r in summary if r['drawable']==11);b=next(r for r in summary if r['drawable']==22)
    assert (a['interval_swaps'],a['total_swaps'],a['elapsed_ns'],a['swap_hz'])==(2,3,1000000000,2)
    assert (b['interval_swaps'],b['total_swaps'],b['elapsed_ns'],b['swap_hz'])==(1,2,1000000000,1)
    assert (a['width'],a['height'])==(1211,711)
    rows,_=run('bounds');assert len([r for r in rows if r['event']=='renderer'])==16
    assert len([r for r in rows if r['event']=='slot_limit'])==1
    a=next(r for r in rows if r['event']=='interval');assert a['drawable']==1 and a['total_swaps']==2
    rows,_=run('displays');assert len(rows)==2 and {r['display'] for r in rows}=={1,2}
    rows,_=run('separate',{'FAKE_MISSING_METADATA':'1'});assert all(not r['metadata_complete'] for r in rows if r['event']=='renderer')
    p=subprocess.run([str(R/'missing_host')],env={**os.environ,'LD_PRELOAD':str(R/'libswap_probe.so')},capture_output=True,text=True,check=True)
    rows=[json.loads(s) for s in p.stderr.splitlines()];assert rows==[{'event':'symbol_error','symbol':'glXSwapBuffers','telemetry_valid':False}]
    p=subprocess.run([str(R/'minimal_host'),'separate'],env={**os.environ,'LD_PRELOAD':str(R/'libswap_probe.so')},capture_output=True,text=True,check=True)
    rows=[json.loads(s) for s in p.stderr.splitlines()]
    assert len(rows)==4 and all(not r['metadata_complete'] and not r['direct_known'] for r in rows if r['event']=='renderer')
    assert all(r['width']==r['height']==0 for r in rows)
    samples={'baseline':[],'probe':[]}
    for _ in range(5):
        p=subprocess.run([str(R/'fake_host'),'bench'],env={**os.environ,'LD_PRELOAD':str(R/'libaudit_heap.so')},capture_output=True,text=True,check=True)
        samples['baseline'].append(json.loads(p.stdout))
        rows,p=run('bench');assert len(rows)==1 and rows[0]['event']=='renderer';samples['probe'].append(json.loads(p.stdout))
    base=statistics.median(x['ns_per_call'] for x in samples['baseline']);probe=statistics.median(x['ns_per_call'] for x in samples['probe'])
    report={'passed':True,'GPU_executed':False,'real_driver_used':False,'calls_per_sample':500000,'samples':samples,'baseline_median_ns':base,'probe_median_ns':probe,'incremental_median_ns':probe-base,'scope':'local host fake real-swap body; real libc monotonic clock; steady no-log hot path only; not RViz/driver/display performance'}
    (R/'host-test-results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':check()
