"""MoveIt-only restart uses the exact manager barrier without perception/hardware work."""
import json
import time
from types import SimpleNamespace

import pytest

from test_station_emergency_reset import API, ROOT, harness, owner


def moveit_api():
    api = harness()
    api.restart = API.restart.__get__(api)
    api.reset_scene_instance = 'old'
    api.reset_scene_empty = True
    api.reset_scene_applied = None
    api.reset_scene_world_ids = []
    api.reset_samples = {}
    api.scene_client = SimpleNamespace(service_is_ready=lambda: True)
    api.requests = []
    def rpc(key, request=None, **kwargs):
        api.calls.append(key)
        if key == 'stop':
            api.state.update(active_operation=None)
            return '{}'
        assert key == 'maintenance'
        api.requests.append(request)
        if request.enable and not request.token:
            assert request.operation == 'station_reset'
            api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
            return json.dumps({'token': 'a'*32})
        assert request.token == 'a'*32
        if request.enable:
            api.state['operator_restart_result'] = dict(job_id=request.unit, kind='restart_unit_terminal', success=True)
            if request.unit == api.reset_units['scene']:
                api.reset_scene_instance = 'new'
                api.reset_samples['scene_status'] = (10., time.monotonic(), 2)
            return json.dumps({'job_id': request.unit})
        api.state.update(active_operation=None, operator_maintenance_token=None)
        return '{}'
    api.rpc = rpc
    api.current_moveit_proof = lambda token, deadline, phase: (
        phase('measured proof', deadline) or SimpleNamespace(scene=SimpleNamespace(world=SimpleNamespace(collision_objects=[]))))
    api.recover_perception = lambda *args: pytest.fail('MoveIt-only must not recover perception')
    return api


def test_restart_moveit_stops_restarts_exact_planning_units_and_releases_after_proof():
    api = moveit_api()
    identity = api.state['owner_started_monotonic']
    result = api.command('restart_moveit', None, SimpleNamespace())
    assert result.success, result.message
    assert api.calls[0] == 'stop'
    assert [r.unit for r in api.requests if getattr(r, 'operation', None) == 'restart'] == [
        'so101-movegroup.service', 'so101-scene.service']
    assert api.requests[-1].enable is False
    assert api.state['owner_started_monotonic'] == identity
    assert api.all_off(api.state)
    assert 'current measured' in result.message
    assert not {'home', 'reset_owner', 'reconnect', 'teleop', 'rearm'} & set(api.calls)


@pytest.mark.parametrize('failure', ['proof', 'stop', 'owner', 'world', 'unknown', 'failed'])
def test_failed_moveit_proof_retains_barrier(failure):
    api = moveit_api()
    proof = api.current_moveit_proof
    def fail(token, deadline, phase):
        if failure == 'proof':
            raise RuntimeError('stale current measured state')
        if failure == 'stop':
            api.stop_generation += 1
        if failure == 'owner':
            api.state['owner_started_monotonic'] += 1
        if failure == 'world':
            api.reset_scene_world_ids = ['colorbrick_missing']
        return proof(token, deadline, phase)
    api.current_moveit_proof = fail
    rpc = api.rpc
    def changed(key, request=None, **kwargs):
        result = rpc(key, request, **kwargs)
        if failure in ('unknown', 'failed') and getattr(request, 'operation', None) == 'restart':
            api.state['operator_restart_result'].update(
                kind='restart_outcome_unknown' if failure == 'unknown' else 'restart_unit_terminal', success=False)
        return result
    api.rpc = changed
    result = api.command('restart_moveit', None, SimpleNamespace())
    assert not result.success
    assert api.state['active_operation'] == 'operator_maintenance'
    assert all(request.enable for request in api.requests)
    assert api.all_off(api.state)


def test_busy_moveit_request_does_not_stop_or_dispatch_another_job():
    api = moveit_api()
    api.command_lock.acquire()
    result = api.command('restart_moveit', None, SimpleNamespace())
    assert not result.success and 'pending' in result.message
    assert not api.calls
    api.command_lock.release()


def test_moveit_restart_capability_does_not_depend_on_camera_or_recovery_services():
    api = moveit_api()
    api.service_clients = {key: SimpleNamespace(service_is_ready=lambda: True) for key in
        ('status', 'stop', 'reconnect', 'reset_owner', 'maintenance', 'rearm', 'home', 'teleop', 'teleop_mode', 'damping')}
    api.service_clients.update({key: SimpleNamespace(service_is_ready=lambda: False) for key in ('recovery_start', 'recovery_status')})
    assert api.capabilities(api.state)['restart_moveit']['enabled']
    api.state['operator_station_reset_units'] = []
    assert not api.capabilities(api.state)['restart_moveit']['enabled']


def test_restart_controls_are_full_width_and_duplicate_guard_preserves_stop():
    panel = (ROOT/'ros2/src/so101_station_operator/src/operator_panel.cpp').read_text()
    assert panel.index('"Restart entire station"') < panel.index('status_ =')
    assert panel.index('"Restart MoveIt"') < panel.index('status_ =')
    assert 'layout->addWidget(button)' in panel
    assert 'new QScrollArea(this)' in panel
    assert 'active.command == command && command != "zero_torque"' in panel
    assert 'moveit_rviz_plugin/MotionPlanning' in panel
    assert 'display->setEnabled(false)' in panel and 'display->setEnabled(true)' in panel
