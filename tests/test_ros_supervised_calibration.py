"""Native service boundary, fake owner only."""
import copy
import json
import time
import pytest
pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from test_ros_follower_recovery_status import ready, state
from test_ros_stop_service import bridge


def setup_target(node, tmp_path):
    helper = node._trajectory_helper
    def snapshot():
        value = state(node)
        value['arm_bounds'] = dict(schema='arm-bounds-observation-v1', classification='normal',
            position_monotonic=value['monotonic'], admitted_min=dict.fromkeys(value['raw'], 1100),
            admitted_max=dict.fromkeys(value['raw'], 2900), firmware_min=dict.fromkeys(value['raw'], 1100),
            firmware_max=dict.fromkeys(value['raw'], 2900))
        return value
    node._read_state = lambda **kwargs: snapshot()
    value = snapshot()
    target = dict(schema='so101-supervised-calibration-target-v1', kind='planned_observation',
        qualified=False, qualification=helper.SUPERVISED_QUALIFICATION,
        arm_id=value['arm_id'], calibration_sha256=value['calibration_sha256'],
        owner_source_sha256=value['owner_source_sha256'], owner_started_monotonic=node._owner_started,
        start_raw=value['raw'], raw=dict(value['raw'], elbow_flex=1886))
    path = tmp_path/'target.json'
    node.set_parameters([Parameter('calibration_endpoint_mode', value=True),
                         Parameter('calibration_ready_pose_file', value=str(path)),
                         Parameter('calibration_ready_timeout_s', value=90.)])
    return target, path, snapshot


def test_service_uses_slow_existing_executor_no_home_promotion(ready, tmp_path):
    node, process = ready
    target, path, snapshot = setup_target(node, tmp_path)
    path.write_text(json.dumps(target))
    def write(line):
        command = json.loads(line); process.commands.append(command)
        assert command['command'] == 'execute_trajectory'
        assert command['supervised_commissioning'] is True
        assert command['expected_start_raw'] == target['start_raw']
        assert 'calibration_endpoint' not in command
        final = snapshot(); final.update(event='trajectory_complete', torque=dict.fromkeys(final['raw'], 1))
        node._response_queue.put(dict(ok=True, state=final, trajectory=dict(
            goal_id=command['goal_id'], effective_path_sha256=command['path_sha256'], qualified=False,
            qualification=node._trajectory_helper.SUPERVISED_QUALIFICATION)))
    process.write = write
    response = node._calibration_ready(Trigger.Request(), Trigger.Response())
    assert response.success, response.message
    assert json.loads(response.message)['qualified'] is False
    assert node._home_completed_owner_started is None
    assert node._active_goal is None
    assert len(process.commands) == 1


@pytest.mark.parametrize('failure', ['schema', 'kind', 'qualified', 'generation', 'identity',
    'source', 'start', 'raw_bool', 'jaw', 'bounds', 'stale', 'torque'])
def test_invalid_target_never_delivered(ready, tmp_path, failure):
    node, process = ready
    target, path, snapshot = setup_target(node, tmp_path)
    if failure == 'schema': target['schema'] = 1
    if failure == 'kind': target['kind'] = 'measured-home'
    if failure == 'qualified': target['qualified'] = True
    if failure == 'generation': target['owner_started_monotonic'] -= 1
    if failure == 'identity': target['arm_id'] = 'wrong'
    if failure == 'source': target['owner_source_sha256'] = {}
    if failure == 'start': target['start_raw']['shoulder_pan'] += 1
    if failure == 'raw_bool': target['raw']['elbow_flex'] = True
    if failure == 'jaw': target['raw']['gripper'] += 1
    if failure == 'bounds': target['raw']['elbow_flex'] = 1000
    if failure in ('stale', 'torque'):
        value = snapshot()
        if failure == 'stale': value['monotonic'] -= 1
        else: value['torque']['gripper'] = 1
        node._read_state = lambda **kwargs: value
    path.write_text(json.dumps(target))
    response = node._calibration_ready(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert not process.commands
    assert node._active_goal is None


@pytest.mark.parametrize('failure', ['reply_error', 'wrong_goal', 'stale_ack', 'timeout'])
def test_possible_delivery_failure_requests_existing_stop(ready, tmp_path, monkeypatch, failure):
    node, process = ready
    target, path, snapshot = setup_target(node, tmp_path); path.write_text(json.dumps(target))
    stops=[]
    monkeypatch.setattr(node, '_prepare_stop', lambda p, generation: stops.append((p,generation)) or True)
    def exchange(command, p, generation, deadline, reservation):
        process.commands.append(command)
        if failure == 'timeout': raise TimeoutError('unknown delivery')
        if failure == 'reply_error': return {'ok': False}
        final=snapshot(); final.update(event='trajectory_complete', torque=dict.fromkeys(final['raw'],1))
        if failure == 'stale_ack': final['monotonic'] -= 1
        return dict(ok=True, state=final, trajectory=dict(goal_id='wrong' if failure=='wrong_goal' else command['goal_id'],
            effective_path_sha256=command['path_sha256'], qualified=False,
            qualification=node._trajectory_helper.SUPERVISED_QUALIFICATION))
    monkeypatch.setattr(node, '_reserved_owner_reply', exchange)
    result=node._calibration_ready(Trigger.Request(),Trigger.Response())
    assert not result.success and json.loads(result.message)['stop_verified'] is True
    assert len(stops)==1 and len(process.commands)==1
    assert node._home_completed_owner_started is None


def test_home_service_adopts_validated_cache_only_after_owner_completion(ready, tmp_path):
    node, process = ready
    from pathlib import Path
    config=json.loads(Path(node._prepare_config_path).read_text())
    home_path=Path(config['arm']['home_file']); home=json.loads(home_path.read_text())
    home['position']['elbow_flex']=-1.; home_path.write_text(json.dumps(home))
    node._saved_home_pose={'old': 1}
    def write(line):
        request=json.loads(line); process.commands.append(request)
        assert request == {'command':'home'}
        node._response_queue.put(dict(ok=True,state=state(node,event='home_complete')))
    process.write=write
    result=node._home(Trigger.Request(),Trigger.Response())
    assert result.success, result.message
    assert node._saved_home_pose == home['position']
    assert node._home_completed_owner_started == node._owner_started


def test_home_service_invalid_reload_keeps_old_cache_no_delivery(ready):
    node, process=ready
    from pathlib import Path
    config=json.loads(Path(node._prepare_config_path).read_text())
    path=Path(config['arm']['home_file']); home=json.loads(path.read_text()); home['arm_id']='other'
    path.write_text(json.dumps(home)); node._saved_home_pose={'old':1}
    result=node._home(Trigger.Request(),Trigger.Response())
    assert not result.success and node._saved_home_pose == {'old':1}
    assert not process.commands
