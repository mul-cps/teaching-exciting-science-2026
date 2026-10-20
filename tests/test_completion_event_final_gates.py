"""Regressions from independent v1 review; original RED artifacts retained."""
from copy import deepcopy
import ast,json,threading,importlib.util
from types import SimpleNamespace as NS
from pathlib import Path
import pytest
from so101_station_perception import perception_recovery as m
from so101_station_perception import completion_event as core
from test_completion_event_protocol import event,inputs,response_for
ROOT=Path(__file__).resolve().parents[1]

def test_actual_completion_commit_block_rejects_legacy_expiry_in_new_gate(monkeypatch):
    # Exact actual _run completion block, isolated from already-tested preflight/reset.
    tree=ast.parse(Path(m.__file__).read_text())
    run=next(x for x in ast.walk(tree) if isinstance(x,ast.FunctionDef) and x.name=='_run')
    commit=run.body[0].body[-1]
    assert isinstance(commit,ast.With) and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='result' for t in x.targets) for x in commit.body)
    f=ast.FunctionDef(name='actual_commit',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg='self')],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[deepcopy(commit)],decorator_list=[])
    module=ast.fix_missing_locations(ast.Module(body=[f],type_ignores=[]))
    clock=[100.49];ns=dict(m.__dict__);ns['time']=NS(monotonic=lambda:clock[0]);exec(compile(module,'actual-production-completion-block','exec'),ns)
    e=event()
    def ros():clock[0]=100.51;return NS(nanoseconds=10_510_000_000)
    n=NS(lock=threading.RLock(),result=dict(state='wait_consumer_admissions',operation_success=False),
        expose_completion_evidence=True,_prepare_event=lambda:e,_guard=lambda:None,
        _proofs_ready=lambda:core.require(clock[0]<100.5,'legacy original proof expired'),
        get_clock=lambda:NS(now=ros),closed=False,cancel=NS(is_set=lambda:False),budget=NS(deadline=139.),_evidence_error=None,
        progress={c:NS(current=v['witnesses'][-1]['proof'],witnesses=[w['proof'] for w in v['witnesses']]) for c,v in e['consumers'].items()})
    # Bind actual production gate with isolated module clock binding.
    gate=next(x for x in ast.walk(tree) if isinstance(x,ast.FunctionDef) and x.name=='_event_commit_gate')
    gm=ast.fix_missing_locations(ast.Module(body=[deepcopy(gate)],type_ignores=[]));exec(compile(gm,'actual-production-gate','exec'),ns)
    n._event_commit_gate=lambda v:ns['_event_commit_gate'](n,v)
    try:ns['actual_commit'](n)
    except ValueError:pass
    assert n.result.get('operation_success') is not True,'expired legacy latest committed success after new clock access'

@pytest.mark.parametrize('change',[dict(job_id='a'*32),dict(state='failed'),dict(ready_now=0)])
def test_offline_verifier_rejects_outer_status_contradiction(tmp_path,capsys,change):
    spec=importlib.util.spec_from_file_location('review_verify',ROOT/'tools/verify_completion_event.py');tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    e=event();ctx=inputs();v=response_for(e)
    v.update(change);a=tmp_path/'response.json';c=tmp_path/'context.json';a.write_text(json.dumps(v));c.write_text(json.dumps(ctx))
    code=tool.main(['--response',str(a),'--expected-context',str(c),'--take-monotonic','100.6','--take-ros-ns','10600000000'])
    assert code==2,'contradictory outer status was reported historically valid'


def test_false_readiness_still_rejects_structurally_invalid_known_deadlines():
    v=dict(schema=core.READINESS_SCHEMA,ready=False,checked_monotonic=100.5,checked_ros_ns=10_500_000_000,
        original_receipt_valid_until_monotonic=-1.,original_identity_valid_until_monotonic=-1.,
        original_local_lineage_valid_until_monotonic=-1.,effective_valid_until_monotonic=-1.,
        original_header_valid_until_ros_ns=-1,qualified=False,execution_enabled=False)
    with pytest.raises(ValueError):core.validate_current_readiness(v,take_mono=100.6,take_ros_ns=10_600_000_000)
