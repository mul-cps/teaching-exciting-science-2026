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
