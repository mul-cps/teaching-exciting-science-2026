"""Cold operator metadata microbenchmark; fakegraph, realtempFS, no RPC/device."""
import json,os,statistics,time
from pathlib import Path
from std_srvs.srv import Trigger
from test_supervisor_consumer_reconciliation_ros import reconciler


def test_bounded_operator_metadata_and_cold_journal(reconciler):
    node,clock=reconciler
    def transition():
        response=node._reconcile_consumers(None,Trigger.Response());assert response.success,response.message
        node._disable(None,Trigger.Response())
    transition()
    response=node._reconcile_consumers(None,Trigger.Response());assert response.success
    def measure(call,count):
        for _ in range(5):call()
        samples=[]
        for _ in range(count):
            began=time.perf_counter_ns();call();samples.append((time.perf_counter_ns()-began)/1000.)
        samples.sort();return dict(samples=count,mean_us=statistics.mean(samples),p95_us=samples[int(.95*(count-1))],max_us=max(samples))
    result=dict(scope='CPU native fake graph/source metadata; realtempFS journal through disable; zero RPC/SDK/GPU/actuator',
        pending_original_entry_guard=measure(node._preflight_deadline,1000))
    node._disable(None,Trigger.Response())
    result['cold_admission_and_disable_journal']=measure(transition,50)
    assert not node.rpc and not node.policy.attempts and node.policy.fault is None and not node.policy.enabled
    if os.environ.get('RECONCILIATION_BENCHMARK_OUTPUT'):
        Path(os.environ['RECONCILIATION_BENCHMARK_OUTPUT']).open('x').write(json.dumps(result,indent=2)+'\n')
