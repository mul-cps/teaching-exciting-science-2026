#!/usr/bin/env python3
"""CPU scalar/worker boundary microbenchmark; no ROS init or sensor payloads."""
import os
if os.environ.get('ROS_DOMAIN_ID')!='177' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise RuntimeError('benchmark requires domain177 localhost1 before imports')
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import sys
from threading import Event
from types import SimpleNamespace
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2/src/so101_station_objects'))
from so101_station_objects.analysis_worker import LatestOnlyAnalysisWorker
from so101_station_objects.timing_diagnostic import TimingRecorder,begin_node,site,worker_timing,count_analyze,finish_node


def summary(values):
    ordered=sorted(values)
    return dict(samples=len(values),mean_ns=statistics.mean(values),p95_ns=ordered[int(.95*(len(values)-1))],max_ns=max(values),all_ns=values)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-worker',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    baseline=Path(args.baseline_worker).resolve()
    raw=baseline.read_bytes()
    assert hashlib.sha256(raw).hexdigest()=='37490d2702ea3e24a19752ee5ff68c9d4c30d23e3619ef93f502b58930310889'
    spec=importlib.util.spec_from_file_location('baseline_worker',baseline)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    values={key:[] for key in ('baseline','disabled','enabled')}
    # Balanced starts, no interval wait, no analyzer work: event handoff overhead only.
    for label in ['baseline','disabled','enabled','enabled','disabled','baseline']*3:
        done=Event();cls=module.LatestOnlyAnalysisWorker if label=='baseline' else LatestOnlyAnalysisWorker
        kwargs={} if label=='baseline' else {'diagnostics':label=='enabled'}
        worker=cls(lambda x:x,on_ready=done.set,**kwargs)
        try:
            for index in range(103):
                done.clear();start=time.perf_counter_ns();worker.submit(index)
                assert done.wait(1)
                result=worker.take_result_with_diagnostics()[0] if label=='enabled' else worker.take_result()
                elapsed=time.perf_counter_ns()-start
                assert result==(index,index,None)
                if index>=3:values[label].append(elapsed)
        finally:assert worker.close(1)
    recorder=TimingRecorder();cost=[]
    owner=SimpleNamespace(timing_recorder=recorder,derived=SimpleNamespace(consumer_epoch='a'*32))
    token={'statement':{'producer_generation':'b'*32,'sequence':1,
        'original':{k+'_stamp_ns':123 for k in ('depth','color','depth_info','color_info')},
        'health':{'source_receipts_monotonic':{k:1. for k in ('depth','color','depth_info','color_info')}},
        'identity':dict(request_started_monotonic=1.,confirmed_monotonic=1.1,expires_monotonic=2.,confirmed_ros_ns=123)},
        'lineage_receipt':1.1,'records':{k:(None,'gid',1.1,123) for k in ('color','aligned','color_info','depth_info')}}
    for _ in range(1000):
        start=time.perf_counter_ns();row=begin_node(owner,token)
        for index in range(22):recorder.clock(row,index)
        site(row,1);worker_timing(row,dict(submit=1.,start=1.,analysis_enter=1.,analysis_return=1.,store_enter=1.))
        count_analyze(row);count_analyze(row);finish_node(owner,0,'expired')
        cost.append(time.perf_counter_ns()-start)
        if len(recorder.rows)==4:recorder.drain()
    serialize=[];max_bytes=0
    for _ in range(1000):
        for __ in range(4):
            row=recorder.begin(None,consumer_epoch='a'*32)
            row.update(generation='f'*32,sequence=(1<<63)-1,headers=[(1<<63)-1]*4,
                       source=[1.7976931348623157e308]*4,identity=[1.7976931348623157e308]*3+[(1<<63)-1],
                       lineage=1.7976931348623157e308,consumer=[1.7976931348623157e308]*4,
                       clocks=[1.7976931348623157e308]*22,analyze_calls=2)
            recorder.finish(row,outcome=0,site=1,error='\x00'*20)
        start=time.perf_counter_ns();text=recorder.drain();serialize.append(time.perf_counter_ns()-start)
        max_bytes=max(max_bytes,len(text.encode()));assert max_bytes<=8192
    result=dict(schema='object-timing-cpu-microbenchmark-v1',qualified=False,execution_enabled=False,
                scope='existing worker Event/public take boundary, zero analyzer/interval; actual node metadata hooks with fixed scalar token and maximum numeric-vector packet; NOT ROS executor/graph/TF/whole image path',
                worker={key:summary(value) for key,value in values.items()},
                row_begin_22clocks_finish=summary(cost),four_row_drain=summary(serialize),maximum_packet_bytes=max_bytes,
                baseline_sha256=hashlib.sha256(raw).hexdigest(),sources={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in (
                    'ros2/src/so101_station_objects/so101_station_objects/objects.py',
                    'ros2/src/so101_station_objects/so101_station_objects/analysis_worker.py',
                    'ros2/src/so101_station_objects/so101_station_objects/timing_diagnostic.py')})
    with Path(args.output).open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({key:{'mean_ns':value['mean_ns'],'p95_ns':value['p95_ns']} for key,value in result['worker'].items()}))
    print(json.dumps({'row_p95_ns':result['row_begin_22clocks_finish']['p95_ns'],'packet_p95_ns':result['four_row_drain']['p95_ns'],'maximum_packet_bytes':max_bytes}))


if __name__=='__main__':main()
