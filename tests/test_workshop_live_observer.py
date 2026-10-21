"""No hardware tests: deterministic freshness, failure and subscriber-only contract."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('observer_ros', ROOT/'tools/live_observer_ros.py')
ros = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ros)
from workshop.live_observer import observe


class ObserverTests(unittest.TestCase):
    def test_stale_future_missing_and_reconnection(self):
        cache = ros.Cache()
        self.assertEqual(cache.snapshot(20)['leader']['state'], 'not_running')
        cache.remember('leader', {'shoulder_pan': 30}, .1, 20)
        self.assertEqual(cache.snapshot(20.1)['leader']['state'], 'fresh')
        self.assertNotIn('degrees', cache.snapshot(22)['leader'])
        cache.remember('leader', {'shoulder_pan': 40}, .01, 23)
        self.assertEqual(cache.snapshot(23.1)['leader']['degrees']['shoulder_pan'], 40)
        for invalid in (-2, math.inf, math.nan):
            cache.remember('leader', {}, invalid, 25)
            self.assertEqual(cache.snapshot(25)['leader']['state'], 'stale')

    def test_demo_is_labelled_and_live_failure_never_becomes_demo(self):
        self.assertIn('invented', observe()['mode'])
        with patch('workshop.live_observer.subprocess.run', side_effect=subprocess.TimeoutExpired('observer', 23)):
            report = observe(live=True)
        self.assertEqual(report['samples'], [])
        self.assertIn('LIVE unavailable', report['mode'])

    def test_duration_bounded(self):
        for duration in (0, 6, math.nan, math.inf):
            with self.assertRaises(ValueError): observe(live=True, duration=duration)

    def test_subscription_only_contract(self):
        banned = {'create_publisher', 'create_client', 'create_service', 'ActionClient', 'publish', 'call_async', 'call', 'Serial', 'ArmSession'}
        paths = [ROOT/'tools/live_observer_ros.py', ROOT/'workshop/live_observer.py']
        paths += [ROOT/'notebooks'/name for name in ('Robot_Candy_Factory_Student.ipynb', 'Robot_Candy_Factory_Instructor.ipynb', 'Robot_Candy_Factory_Live_SO101.ipynb')]
        for path in paths:
            sources = [''.join(c['source']) for c in json.loads(path.read_text())['cells'] if c['cell_type']=='code'] if path.suffix=='.ipynb' else [path.read_text()]
            for source in sources:
                for item in ast.walk(ast.parse(source)):
                    if isinstance(item, ast.Call):
                        name = item.func.attr if isinstance(item.func, ast.Attribute) else getattr(item.func, 'id', '')
                        self.assertNotIn(name, banned, (path, name))
        shell = (ROOT/'tools/live_observer.sh').read_text()
        self.assertNotIn('--hardware', shell)
        self.assertNotIn('--device', shell)

    def test_fake_ros_collects_and_cleans_without_writes(self):
        """Emulate only three subscriptions, in-memory fake ROS domain 75."""
        node = types.SimpleNamespace()
        callbacks = []
        node.create_subscription = lambda kind, topic, callback, qos: callbacks.append((topic, callback))
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))
        destroyed = []
        node.destroy_node = lambda: destroyed.append(True)
        rclpy = types.ModuleType('rclpy')
        rclpy.init = lambda: None
        rclpy.create_node = lambda name: node
        stopped = []
        rclpy.try_shutdown = lambda: stopped.append(True)
        now = [100.0]
        def spin(node, timeout_sec):
            now[0] += .05
            for topic, callback in callbacks:
                if topic.endswith('joint_states'):
                    callback(types.SimpleNamespace(name=list(ros.JOINTS), position=[.1]*5,
                        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100, nanosec=0))))
                else:
                    callback(types.SimpleNamespace(data=json.dumps({'schema':1,'stamp_s':now[0], 'state':'active','teleop_mode':'relative'})))
        rclpy.spin_once = spin
        qos = types.ModuleType('rclpy.qos')
        qos.QoSProfile = lambda **kwargs: kwargs
        qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
        qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
        sensor = types.ModuleType('sensor_msgs.msg'); sensor.JointState = object
        std = types.ModuleType('std_msgs.msg'); std.String = object
        modules = {'rclpy':rclpy,'rclpy.qos':qos,'sensor_msgs.msg':sensor,'std_msgs.msg':std}
        with patch.dict(sys.modules, modules), patch.dict('os.environ', {'ROS_DOMAIN_ID':'75'}), patch.object(ros.time, 'monotonic', side_effect=lambda:now[0]):
            report = ros.collect(.6)
        self.assertEqual(len(callbacks), 3)
        self.assertTrue(report['samples'])
        self.assertEqual(report['samples'][-1]['leader']['state'], 'fresh')
        self.assertTrue(destroyed and stopped)


def observer_callbacks():
    """Run real observer setup against a fake ROS module, retaining its cache."""
    cache = ros.Cache()
    callbacks, cleaned = {}, []
    now = [100.]
    node = types.SimpleNamespace(
        create_subscription=lambda kind, topic, cb, qos: callbacks.update({topic: cb}),
        get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000)),
        destroy_node=lambda: cleaned.append('destroy'))
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda: None
    rclpy.create_node = lambda name: node
    rclpy.spin_once = lambda node, timeout_sec: now.__setitem__(0, now[0]+.21)
    rclpy.try_shutdown = lambda: cleaned.append('shutdown')
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    sensor = types.ModuleType('sensor_msgs.msg'); sensor.JointState = object
    std = types.ModuleType('std_msgs.msg'); std.String = object
    modules = {'rclpy':rclpy,'rclpy.qos':qos,'sensor_msgs.msg':sensor,'std_msgs.msg':std}
    with patch.dict(sys.modules, modules), patch.object(ros, 'Cache', return_value=cache), patch.object(ros.time, 'monotonic', side_effect=lambda:now[0]):
        ros.collect(.5)
    assert cleaned == ['destroy', 'shutdown']
    return callbacks, cache


def joint_message(names=None, positions=None):
    return types.SimpleNamespace(name=list(ros.JOINTS) if names is None else names,
        position=[.1]*5 if positions is None else positions,
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100,nanosec=0)))


class BoundedObserverTests(unittest.TestCase):
    def test_future_sample_never_becomes_fresh_without_a_new_valid_receipt(self):
        cache = ros.Cache()
        for invalid in (-.5, math.nan, math.inf):
            cache.remember('leader', {'shoulder_pan': 123}, invalid, 20.)
            self.assertEqual(cache.snapshot(20.6)['leader']['state'], 'stale')
            self.assertNotIn('degrees', cache.snapshot(20.6)['leader'])
            json.dumps(cache.snapshot(20.6), allow_nan=False)
        cache.remember('leader', {'shoulder_pan': 40}, .1, 21.)
        self.assertEqual(cache.snapshot(21.1)['leader']['degrees']['shoulder_pan'], 40)
        self.assertNotIn('degrees', cache.snapshot(22.1)['leader'])

    def test_joint_payload_is_bounded_before_building_values(self):
        callbacks, cache = observer_callbacks()
        callback = callbacks['/leader/joint_states']
        class TooLong:
            def __len__(self): return 17
            def __iter__(self): raise AssertionError('oversized array must not be traversed')
        for msg in (joint_message(TooLong(), TooLong()),
                    joint_message(list(ros.JOINTS), [.1]*6),
                    joint_message(list(ros.JOINTS)+[ros.JOINTS[0]], [.1]*6),
                    joint_message(positions=[.1,.1,math.nan,.1,.1]),
                    joint_message(positions=[1e308]*5)):
            callback(msg)
            self.assertNotIn('leader', cache.items)
        callback(joint_message())
        self.assertEqual(set(cache.items['leader'][0]), set(ros.JOINTS))

    def test_status_rejects_oversized_input_before_json_parse(self):
        callbacks, cache = observer_callbacks()
        with patch.object(ros.json, 'loads', side_effect=AssertionError('oversized JSON must not be parsed')):
            callbacks['/follower/teleop_status'](types.SimpleNamespace(data='x'*8193))
        self.assertNotIn('teleop', cache.items)

    def test_status_keeps_only_valid_bounded_display_fields(self):
        callbacks, cache = observer_callbacks()
        callback = callbacks['/follower/teleop_status']
        valid = {'schema':1, 'stamp_s':100., 'state':'active', 'teleop_mode':'relative', 'reason':'owner_acknowledged'}
        for change in ({'schema':True}, {'state':'unexpected'}, {'teleop_mode':'invalid'},
                       {'stamp_s':math.nan}, {'stamp_s':10**1000}, {'reason':'x'*257}, {'reason':{}}, {'state':[]}):
            callback(types.SimpleNamespace(data=json.dumps(dict(valid, **change))))
            self.assertNotIn('teleop', cache.items)
        callback(types.SimpleNamespace(data=json.dumps(dict(valid, extra='x'*1000))))
        self.assertEqual(cache.items['teleop'][0], valid)

    def test_input_bounds_and_subscription_qos_have_static_guards(self):
        tree = ast.parse((ROOT/'tools/live_observer_ros.py').read_text())
        collect = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='collect')
        callbacks = {n.name:n for n in ast.walk(collect) if isinstance(n, ast.FunctionDef)}
        joint_source = ast.unparse(callbacks['callback'])
        status_source = ast.unparse(callbacks['status'])
        self.assertLess(joint_source.index('MAX_JOINT_VALUES'), joint_source.index('zip('))
        self.assertLess(status_source.index('MAX_STATUS_CHARS'), status_source.index('json.loads('))
        subscriptions = [n for n in ast.walk(collect) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_subscription']
        self.assertEqual(len(subscriptions), 3)
        profiles = [n for n in ast.walk(collect) if isinstance(n,ast.Call) and getattr(n.func,'id','')=='QoSProfile']
        self.assertEqual(len(profiles), 2)
        self.assertTrue(all(any(k.arg=='depth' and isinstance(k.value,ast.Constant) and k.value.value==1 for k in n.keywords) for n in profiles))


if __name__ == '__main__': unittest.main()
