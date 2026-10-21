"""Real ROS bridge mode-selection/enable/status methods, in-memory owner only."""
import json
import time

import pytest
pytest.importorskip('rclpy')
from std_srvs.srv import SetBool, Trigger
from test_ros_stop_service import bridge, snapshot


def mode(node, absolute):
    request = SetBool.Request(); request.data = absolute
    return node._set_teleop_absolute_mode(request, SetBool.Response())


def current(node, torque=0):
    value = snapshot()
    value.update(observe_only=False, torque_monotonic=value['monotonic'],
        owner_source_sha256=node._owner_source_expected)
    value['torque'] = dict.fromkeys(value['raw'], torque)
    value['arm_bounds'] = dict(schema='arm-bounds-observation-v1', classification='normal',
        position_monotonic=value['monotonic'], admitted_min=dict.fromkeys(value['raw'],1000),
        admitted_max=dict.fromkeys(value['raw'],3000), firmware_min=dict.fromkeys(value['raw'],500),
        firmware_max=dict.fromkeys(value['raw'],3500))
    return value


def setup_owner(node):
    node._owner_started = time.monotonic() - .01
    node._home_completed_owner_started = None
    node._owner_source_expected = {'arm_session': 'a'*64}
    node._read_state = lambda **kwargs: current(node)
    node._cache_owner_state(current(node))


@pytest.mark.parametrize('absolute', [False, True])
def test_real_bridge_selects_mode_without_joint_writes_and_enables_without_home(bridge, absolute):
    node, process = bridge
    setup_owner(node)
    result = mode(node, absolute)
    assert result.success, result.message
    assert not process.commands
    node._teleop_latest = dict(stamp_s=time.monotonic(), sequence=1,
                              pose=dict.fromkeys(current(node)['raw'],10.))
    node._send = process.commands.append
    node._wait_teleop_ack = lambda key: {'ok': True, 'state': current(node, torque=1)}
    request = SetBool.Request(); request.data=True
    result = node._set_teleop_enabled(request, SetBool.Response())
    assert result.success, result.message
    selected = 'absolute' if absolute else 'relative'
    assert process.commands[0]['mode'] == selected
    assert node._home_completed_owner_started is None
    status = node._status(Trigger.Request(), Trigger.Response())
    assert json.loads(status.message)['teleop_mode'] == selected
    assert not json.loads(status.message)['home_completed_this_session']
    assert not mode(node, not absolute).success
    assert node._teleop_mode == selected
    # The bridge owns mode mapping; the owner receives final position targets.
    mapped = {joint: float(index * 3) for index, joint in enumerate(current(node)['raw'])}
    node._teleop_latest.update(stamp_s=time.monotonic(), sequence=2, pose=mapped)
    node._teleop_tick()
    step = process.commands[-1]
    assert step['command'] == 'teleop_step'
    expected = (mapped if absolute else
                {joint: node._teleop_follower_start[joint] + mapped[joint] - node._teleop_origin[joint]
                 for joint in mapped})
    assert step['pose'] == expected
    assert node._home_completed_owner_started is None


@pytest.mark.parametrize('fault', ['stale', 'fault', 'stop', 'busy', 'bounds'])
def test_real_bridge_mode_selection_preserves_idle_fresh_admission(bridge, fault):
    node, process = bridge
    setup_owner(node)
    state = current(node)
    if fault == 'stale': state['monotonic'] -= 1
    if fault == 'fault': node._owner_faulted = True
    if fault == 'stop': node._stop_latched = True
    if fault == 'busy': node._active_goal = 'home'
    if fault == 'bounds': state['arm_bounds']['classification'] = 'unrecoverable'
    node._read_state = lambda **kwargs: state
    assert not mode(node, True).success
    assert node._teleop_mode == 'relative' and not process.commands
