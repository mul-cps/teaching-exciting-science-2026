"""Fixed reconciliation action only; no actual service invocation."""
import json
from types import SimpleNamespace as NS
import pytest
from tools import ros_station_action as action
from test_perception_quick_actions import install_ros
COMMAND='perception-supervisor-reconcile-consumers'


def test_fixed_operator_action_one_rpc_no_path_or_readiness_inference(monkeypatch,capsys):
    value=dict(accepted=True,reconciliation_id='a'*32,enabled=False,ready_now=False,operation_success=False,qualified=False,execution_enabled=False)
    calls=install_ros(monkeypatch,value)
    assert action.main([COMMAND])==0
    out=json.loads(capsys.readouterr().out)
    assert out['service']=='/so101_perception_supervisor/reconcile_consumers' and calls.count('call')==1
    assert out['ready_now'] is False and out['operation_success'] is False
    with pytest.raises(SystemExit):action.parse_args([COMMAND,'/arbitrary/path'])


@pytest.mark.parametrize('change',[dict(reconciliation_id='bad'),dict(enabled=True),dict(ready_now=True),dict(operation_success=True),dict(accepted=False)])
def test_operator_acceptance_cannot_claim_ready_or_enable(change):
    value=dict(accepted=True,reconciliation_id='a'*32,enabled=False,ready_now=False,operation_success=False,qualified=False,execution_enabled=False);value.update(change)
    with pytest.raises(ValueError):action._diagnostic_payload(COMMAND,NS(success=True,message=json.dumps(value)))
