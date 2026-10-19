"""Fixed-rate source-health metadata starts; no ROS init or recovery authority."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='177'
from types import SimpleNamespace as NS,MethodType
import pytest
from test_supervisor_health_expiry import fake_node
from so101_station_perception import perception_supervisor as m


def refresh_node(monkeypatch):
    node,proof,clock,requests=fake_node(monkeypatch)
    node.health_refresh_callbacks=0;resets=[]
    node.health_timer=NS(reset=lambda:resets.append(clock[0]))
    for name in ('_health_eligible','_health_refresh'):
        setattr(node,name,MethodType(getattr(m.PerceptionSupervisor,name),node))
    return node,clock,requests,resets


@pytest.mark.parametrize('mutation',[lambda n:setattr(n,'closed',True),lambda n:setattr(n.policy,'enabled',False),
    lambda n:setattr(n.policy,'latched','fault'),lambda n:setattr(n,'reset_pending',True),
    lambda n:setattr(n.policy,'job',{'state':'accepted'}),lambda n:setattr(n.policy,'state','verifying_native'),
    lambda n:setattr(n.policy,'state','operator_preflight')])
def test_refresh_never_starts_outside_readonly_health_phase(monkeypatch,mutation):
    node,_,calls,resets=refresh_node(monkeypatch);mutation(node);node._health_refresh()
    assert not calls
    assert bool(resets) is (not node.closed)


def test_refresh_one_pending_no_catchup_and_end_anchored(monkeypatch):
    node,clock,calls,resets=refresh_node(monkeypatch)
    def request(endpoint,timeout):
        calls.append((endpoint,clock[0]));node.rpc[endpoint]=object();clock[0]+=.003
    node._request=request;node._health_refresh()
    assert len(calls)==2 and resets==[clock[0]]
    assert all(node.health_next_request[e]>=t+.1 for e,t in calls)
    clock[0]+=50;node._health_refresh();assert len(calls)==2
    node.rpc.clear();node._health_refresh();assert len(calls)==4
    node._health_wakeup();assert len(calls)==4  # Completion guard never starts.


def test_second_start_rechecks_eligibility_after_first_admission(monkeypatch):
    node,clock,calls,_=refresh_node(monkeypatch)
    def request(endpoint,timeout):
        calls.append(endpoint);node.policy.enabled=False
    node._request=request;node._health_refresh();assert len(calls)==1


def test_service_ready_stall_cannot_start_after_disable(monkeypatch):
    node,clock,calls,_=refresh_node(monkeypatch)
    endpoint=m.CONSUMERS[0]+'/derived_source_health'
    def ready():node.policy.enabled=False;return True
    node.rpc_clients={endpoint:NS(service_is_ready=ready,call_async=lambda _:calls.append('RPC'))}
    assert m.PerceptionSupervisor._request(node,endpoint,2.) is False
    assert not calls and not node.rpc


def test_close_cannot_rearm_timer_in_finally(monkeypatch):
    node,_,calls,resets=refresh_node(monkeypatch)
    node._health_queries=lambda:setattr(node,'closed',True)
    node._health_refresh();assert not resets and not calls


@pytest.mark.parametrize('period,coverage,armed',[(.1,.185,True),(.1,.135,True),(.1,.07,False),(.2,.185,False)])
def test_only_genuine_original_overlap_accumulates_five_seconds(monkeypatch,period,coverage,armed):
    node,clock,_,_=refresh_node(monkeypatch)
    for i in range(61):
        clock[0]=100.3+i*period;node.health_candidate_end=clock[0]+coverage
        node._apply_current_health(True)
    assert (node.policy.state=='armed') is armed
    assert not node.policy.attempts


def test_readonly_starter_cpu_microbenchmark(monkeypatch,tmp_path):
    import json,time,statistics
    from pathlib import Path
    node,clock,calls,_=refresh_node(monkeypatch)
    count=[0]
    node._request=lambda *a:count.__setitem__(0,count[0]+1)
    values=[]
    for i in range(1010):
        clock[0]=100.3+i*.101
        began=time.perf_counter_ns();node._health_refresh();elapsed=(time.perf_counter_ns()-began)/1000.
        if i>=10:values.append(elapsed)
    values.sort()
    assert count[0]==2020 and node.policy.job is None and not node.policy.attempts
    result=dict(scope='CPU only; two fixed fake-ready endpoints; no ROS init/RPC/GPU/SDK; callback+eligibility+caps+reanchor',
        samples=len(values),mean_us=statistics.mean(values),p95_us=values[949],max_us=max(values),
        health_refresh_period_s=.1,control_period_s=.2,starts_per_callback=2)
    if os.environ.get('HEALTH_REFRESH_BENCHMARK_OUTPUT'):
        Path(os.environ['HEALTH_REFRESH_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')


@pytest.mark.parametrize("change",["closed_during_graph","disabled_during_graph","generation","ready_with_fault"])
def test_no_readonly_rpc_after_final_producer_access_invalidates_admission(monkeypatch,change):
    node,proof,clock,requests=fake_node(monkeypatch)
    calls=[]
    endpoint=m.CONSUMERS[0]+"/derived_source_health"
    node.rpc_clients={endpoint:NS(service_is_ready=lambda:True,call_async=lambda req:calls.append(req) or NS(add_done_callback=lambda cb:None))}
    def producer_now():
        if change=="closed_during_graph":node.closed=True
        if change=="disabled_during_graph":node.policy.enabled=False
        return dict(state="ready",fault={} if change=="ready_with_fault" else None,producer_generation="f"*32 if change=="generation" else node.pin["producer_generation"])
    node._producer_now=producer_now
    try:m.PerceptionSupervisor._request(node,endpoint,2.)
    except ValueError:pass
    assert not calls


def test_whole_production_health_starter_cpu_benchmark(monkeypatch):
    import json,time,statistics
    from pathlib import Path
    node,clock,calls,resets=refresh_node(monkeypatch)
    graph_calls=[0];starts=[0]
    gid='a'*32
    node.producer=(dict(state='ready',fault=None,producer_generation=node.pin['producer_generation']),gid,clock[0])
    node.heartbeat_gid=gid
    node.get_publishers_info_by_topic=lambda topic:graph(topic)
    def graph(topic):
        assert topic==m.PRODUCER_STATUS;graph_calls[0]+=1
        namespace,name=m.PRODUCER.rsplit('/',1)
        return [NS(node_namespace=namespace,node_name=name,endpoint_gid=bytes.fromhex(gid))]
    node._producer_now=MethodType(m.PerceptionSupervisor._producer_now,node)
    node._request=MethodType(m.PerceptionSupervisor._request,node)
    class Future:
        def add_done_callback(self,callback):self.callback=callback
    def send(request):starts[0]+=1;return Future()
    node.rpc_clients={c+'/derived_source_health':NS(service_is_ready=lambda:True,call_async=send) for c in m.CONSUMERS}
    def measure(kind):
        values=[];before_graph=graph_calls[0];before_starts=starts[0]
        for i in range(1010):
            clock[0]+=.101
            node.producer=(node.producer[0],gid,clock[0])
            if kind=='admit':node.rpc.clear()
            elif kind=='cap':node.rpc.clear();node.health_next_request={e:clock[0]+.1 for e in node.rpc_clients}
            began=time.perf_counter_ns();node._health_refresh();elapsed=(time.perf_counter_ns()-began)/1000.
            if i>=10:values.append(elapsed)
        values.sort()
        return dict(samples=1000,mean_us=statistics.mean(values),p95_us=values[949],max_us=max(values),
            graph_lookups_total=graph_calls[0]-before_graph,rpc_admissions_total=starts[0]-before_starts)
    result=dict(scope='WHOLE production refresh/query/request/producer_now; fixed graph+fresh heartbeat; fake client/future only; no ROS init or actual RPC',
        admit_two=measure('admit'),pending_skip=measure('pending'),cap_skip=measure('cap'))
    assert result['admit_two']['graph_lookups_total']==2020 and result['admit_two']['rpc_admissions_total']==2020
    assert result['pending_skip']['graph_lookups_total']==result['pending_skip']['rpc_admissions_total']==0
    assert result['cap_skip']['graph_lookups_total']==result['cap_skip']['rpc_admissions_total']==0
    if os.environ.get('HEALTH_REFRESH_WHOLE_BENCHMARK_OUTPUT'):
        Path(os.environ['HEALTH_REFRESH_WHOLE_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')
