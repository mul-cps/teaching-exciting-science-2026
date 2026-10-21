"""Mixed owner reservations must never invoke ServerGoalHandle equality."""
import ast
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_control/so101_station_control'


class GoalHandle:
    """Match rclpy's unguarded equality and unhashable handle contract."""
    goal_id = object()
    def __eq__(self, other):
        return self.goal_id == other.goal_id
    def __ne__(self, other):
        return self.goal_id != other.goal_id


def bridge_class():
    tree = ast.parse((SOURCE / 'trajectory_server.py').read_text())
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'TrajectoryBridge')
    names = {'_active_goal_is', '_clear_teleop', '_cancel_request', '_goal_request', '_expire_operator_modes', '_gripper_cancel_request'}
    methods = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name in names]
    for method in methods:
        method.decorator_list = []
    tree = ast.Module(body=[ast.ClassDef(name='Bridge', bases=[], keywords=[], body=methods,
                                       decorator_list=[])], type_ignores=[])
    namespace = dict(GoalResponse=SimpleNamespace(REJECT=0, ACCEPT=1),
                     CancelResponse=SimpleNamespace(REJECT=0, ACCEPT=1),
                     ARM_JOINTS=('a', 'b', 'c', 'd', 'e'), time=time)
    exec(compile(ast.fix_missing_locations(tree), str(SOURCE / 'trajectory_server.py'), 'exec'), namespace)
    return namespace['Bridge']


@pytest.mark.parametrize('reservation', ['home', 'operator_maintenance', 'operator_maintenance_unknown',
    'teleop', 'teleop_starting', 'native_damping', 'pending', 'validating', object(), GoalHandle()])
def test_busy_action_admission_preserves_every_reservation(reservation):
    node = bridge_class()()
    node._active_goal = reservation
    node._active_lock = threading.Lock()
    node._owner_faulted = False
    node._external_state_only = False
    node._has_nondefault_tolerance = lambda request: False
    goal = SimpleNamespace(trajectory=SimpleNamespace(joint_names=['a', 'b', 'c', 'd', 'e'], points=[0, 1]))
    assert node._goal_request(goal) == 0
    assert node._active_goal is reservation


def test_teleop_cleanup_preserves_active_action_handle():
    node = bridge_class()()
    handle = GoalHandle()
    node._active_goal = handle
    node._active_lock = threading.Lock()
    node._teleop_stop_reason = None
    node._publish_teleop_status = lambda *args: None
    node._clear_teleop()
    assert node._active_goal is handle


def test_typed_reservations_never_call_handle_equality_and_use_identity():
    node = bridge_class()()
    handle = GoalHandle()
    node._active_goal = handle
    for name in ('home', 'operator_maintenance', 'teleop', 'gripper'):
        assert not node._active_goal_is(name)
    assert node._active_goal_is(handle)
    assert not node._active_goal_is(GoalHandle())
    node._active_goal = 'operator_maintenance'
    assert node._active_goal_is('operator_maintenance', 'home')
    assert not node._active_goal_is(handle)


def test_cancel_preserves_operator_reservation_and_cancels_only_same_action():
    node = bridge_class()()
    node._active_lock = threading.Lock()
    sent = []
    node._send = sent.append
    handle = GoalHandle()
    for reservation in ('operator_maintenance', 'home', 'teleop', object()):
        node._active_goal = reservation
        assert node._cancel_request(handle) == 0
        assert node._active_goal is reservation
    node._active_goal = handle
    assert node._cancel_request(GoalHandle()) == 0
    assert node._cancel_request(handle) == 1
    assert sent == [{'command': 'cancel_hold'}]


def test_no_raw_mixed_type_reservation_comparisons_remain():
    for filename in ('trajectory_server.py', 'operator_restart.py'):
        tree = ast.parse((SOURCE / filename).read_text())
        for method in ast.walk(tree):
            if not isinstance(method, ast.FunctionDef) or method.name == '_active_goal_is':
                continue
            for node in ast.walk(method):
                if isinstance(node, ast.Compare) and isinstance(node.left, ast.Attribute) and node.left.attr == '_active_goal':
                    assert all(isinstance(op, (ast.Is, ast.IsNot)) for op in node.ops), (filename, node.lineno)


def test_real_rclpy_handle_uses_identity_without_ros_or_hardware():
    server = pytest.importorskip('rclpy.action.server')
    handle = object.__new__(server.ServerGoalHandle)
    handle._goal_info = SimpleNamespace(goal_id=object())
    node = bridge_class()()
    node._active_goal = handle
    assert node._active_goal_is(handle)
    assert not node._active_goal_is('home', 'operator_maintenance', 'teleop')


def test_operator_expiry_monitor_preserves_running_trajectory():
    node = bridge_class()()
    handle = GoalHandle()
    node._active_goal = handle
    node._active_lock = threading.Lock()
    node._state_cache_lock = threading.Lock()
    node._cached_owner_state = None
    node._cached_owner_generation = 0.
    node._owner_started = 0.
    node._operator_maintenance_deadline = None
    node._expire_operator_modes()
    assert node._active_goal is handle


def test_gripper_cancel_cannot_clear_or_compare_a_trajectory_handle():
    node = bridge_class()()
    node._active_lock = threading.Lock()
    handle = GoalHandle()
    node._active_goal = handle
    assert node._gripper_cancel_request(handle) == 0
    assert node._active_goal is handle


def test_operator_restart_rejects_action_handle_before_external_manager_access():
    tree = ast.parse((SOURCE / 'operator_restart.py').read_text())
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == '_operator_restart_begin')
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                 str(SOURCE / 'operator_restart.py'), 'exec'), namespace)
    cls = bridge_class()
    cls._operator_restart_begin = namespace['_operator_restart_begin']
    node = cls()
    node._operator_restart_lock = threading.Lock()
    node._operator_restart_units = ['fake.service']
    node._operator_external_job = None
    node._operator_maintenance_token = 'token'
    handle = GoalHandle()
    node._active_goal = handle
    with pytest.raises(RuntimeError, match='live maintenance reservation'):
        node._operator_restart_begin('token', 'fake.service')
    assert node._active_goal is handle
