"""Mode selection uses the real operator flow with an AST-only fake owner."""
from types import SimpleNamespace

import pytest

from test_station_emergency_reset import API, NS, harness, owner


def mode_api(**updates):
    api = harness(owner(teleop_start_ready=True, teleop_mode='relative', **updates))
    NS['SetBool'] = SimpleNamespace(Request=lambda **kw: SimpleNamespace(**kw))
    api.requests = []
    def rpc(key, request=None, **kwargs):
        api.calls.append(key)
        api.requests.append((key, request))
        if key == 'teleop_mode':
            api.state['teleop_mode'] = 'absolute' if request.data else 'relative'
        return 'selected'
    api.rpc = rpc
    api.service_clients = {key: SimpleNamespace(service_is_ready=lambda: True)
                           for key in ('status', 'stop', 'reconnect', 'reset_owner', 'maintenance',
                                       'recovery_start', 'recovery_status', 'home', 'rearm',
                                       'teleop', 'teleop_mode', 'damping')}
    return api


@pytest.mark.parametrize('mode,absolute', [('relative', False), ('absolute', True)])
def test_idle_mode_selection_does_not_home_or_enable(mode, absolute):
    api = mode_api()
    result = api.command('teleop_'+mode, None, SimpleNamespace())
    assert result.success, result.message
    assert api.calls == ['teleop_mode']
    assert api.requests[0][1].data is absolute
    assert api.state['teleop_mode'] == mode
    assert 'teleop_'+mode in NS['COMMANDS']


@pytest.mark.parametrize('updates,reason', [
    ({'teleop_state': 'active', 'active_operation': 'teleop_relative'}, 'disable'),
    ({'fresh': False}, 'fresh'),
    ({'owner_faulted': True}, 'faulted'),
    ({'observe_only': True}, 'observe-only'),
    ({'active_operation': 'trajectory'}, 'active'),
    ({'teleop_start_ready': False, 'teleop_start_reason': 'blocked leader'}, 'blocked leader'),
])
def test_rejected_selection_never_calls_backend(updates, reason):
    api = mode_api()
    api.state.update(updates)
    result = api.command('teleop_absolute', None, SimpleNamespace())
    assert not result.success and reason in result.message
    assert api.calls == []
    capabilities = api.capabilities(api.state)
    for command in ('teleop_relative', 'teleop_absolute'):
        assert not capabilities[command]['enabled']
        assert reason.lower() in capabilities[command]['reason'].lower()


@pytest.mark.parametrize('final', [{'fresh': False}, {'teleop_mode': 'relative'}])
def test_selection_requires_fresh_exact_final_mode(final):
    api = mode_api()
    rpc = api.rpc
    def wrong_final(*args, **kwargs):
        result = rpc(*args, **kwargs)
        api.state.update(final)
        return result
    api.rpc = wrong_final
    result = api.command('teleop_absolute', None, SimpleNamespace())
    assert not result.success and 'fresh measured absolute' in result.message
    assert api.calls == ['teleop_mode']


def test_selection_uses_existing_rearm_without_home_or_enable():
    api = mode_api(motion_profile_rearm_required=True)
    result = api.command('teleop_absolute', None, SimpleNamespace())
    assert result.success, result.message
    assert api.calls == ['rearm', 'teleop_mode']


def test_missing_mode_service_disables_both_choices():
    api = mode_api()
    api.service_clients['teleop_mode'].service_is_ready = lambda: False
    for command in ('teleop_relative', 'teleop_absolute'):
        capability = api.capabilities(api.state)[command]
        assert not capability['enabled']
        assert 'teleop_mode' in capability['reason']
