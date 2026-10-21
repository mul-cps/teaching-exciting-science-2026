"""Exercise real operator control flow without ROS, hardware, or manager calls."""
import ast
import functools
import json
import math
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'ros2/src/so101_station_operator/scripts/operator_api'


def load_api():
    tree = ast.parse(SCRIPT.read_text())
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.Assign))]
    namespace = dict(Node=object, json=json, math=math, re=re, threading=threading,
                     time=time, functools=functools,
                     OperatorMaintenance=SimpleNamespace(Request=lambda **kw: SimpleNamespace(**kw)))
    exec(compile(tree, str(SCRIPT), 'exec'), namespace)
    return namespace['OperatorAPI'], namespace


API, NS = load_api()


def owner(**updates):
    state = dict(kind='follower_status', fresh=True, connected=True,
                 observe_only=False, owner_faulted=False, active_operation=None,
                 torque={j: 0 for j in NS['JOINTS']}, arm_id='fake',
                 calibration_sha256='a'*64, owner_started_monotonic=1.,
                 owner_source_sha256={'fake': 'b'*64}, operator_external_job=None)
    state.update(updates)
    return state


def harness(state=None):
    api = API.__new__(API)
    api.command_lock, api.stop_lock = threading.Lock(), threading.Lock()
    api.reset_cancel = threading.Event()
    api.stop_generation = 0
    api.stop_intent_lock = threading.Lock()
    api.reset_rpc_deadline = threading.local()
    api.reset_progress, api.pending, api.last_result = None, None, None
    api.uncertain = api.uncertain_motion = False
    api.alignment_status = None
    api.recovery_binding = None
    api.reset_measured_joints = None
    api.reset_alignment = lambda *args: None
    api.activate_reset_images = lambda: None
    api.retire_reset_images = lambda: None
    api.manual_reference = api.manual_record = None
    api.manual_confirmed = False
    api.state = state or owner()
    api.units = {k: "so101-"+k+".service" for k in ("camera", "depth", "tags")}
    api.reset_units = {k: "so101-"+k+".service" for k in ("movegroup", "objects", "scene")}
    api.state.update(operator_restart_units=list(api.units.values()), operator_station_reset_units=list(api.reset_units.values()))
    api.get_parameter = lambda name: SimpleNamespace(value=name == "enable_service_restarts")
    api.calls = []
    api.snapshot = lambda: dict(api.state)
    def rpc(key, *args, **kwargs):
        api.calls.append(key)
        if key == 'stop':
            api.state.update(torque={j: 0 for j in NS['JOINTS']}, active_operation=None)
        if key == 'reset_owner':
            api.state['owner_started_monotonic'] += 1
        if key == 'reconnect':
            api.state.update(connected=True, owner_faulted=False, fault_observation=None)
        return '{}'
    api.rpc = rpc
    def restart(key, **kwargs):
        assert api.all_off(api.state)
        assert kwargs['station_reset']
        api.calls.append('restart:' + key)
        api.reset_token = 'a'*32
        api.state.update(active_operation='operator_maintenance', operator_maintenance_token=api.reset_token)
        return 'fresh acceptance'
    api.restart = restart
    return api


def test_reset_reinitializes_owner_in_order_and_finishes_off_without_home():
    api = harness(owner(torque={j: 1 for j in NS['JOINTS']}))
    result = api.command('emergency_reset', None, SimpleNamespace())
    assert result.success, result.message
    assert api.calls == ['stop', 'stop', 'reset_owner', 'stop', 'restart:all', 'maintenance']
    assert api.reset_progress['initialized_all_off'] is True
    assert api.reset_progress['pick_place_ready'] is False
    assert 'saved home pending' in result.message
    assert not {'home', 'rearm', 'teleop', 'damping'} & set(api.calls)


def test_fault_observer_reconnects_even_when_connected_and_fresh():
    api = harness(owner(fault_observation={'schema': 1}, owner_faulted=True))
    assert api.command('emergency_reset', None, SimpleNamespace()).success
    assert api.calls[:4] == ['stop', 'reconnect', 'stop', 'reset_owner']


def test_unknown_job_never_clears_or_retries():
    api = harness(owner(operator_external_job={'job_id': 'pending'}))
    assert not api.command('emergency_reset', None, SimpleNamespace()).success
    assert api.calls == ['stop']
    assert api.state['operator_external_job'] == {'job_id': 'pending'}


def test_no_owner_response_is_bounded_failure():
    api = harness()
    def unavailable():
        raise RuntimeError('status service unavailable')
    api.snapshot = unavailable
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and 'unavailable' in reply.message
    assert api.reset_progress['initialized_all_off'] is False
    assert api.calls == ['stop']


def test_stop_interrupts_reset_and_duplicate_request_is_busy():
    api = harness()
    api.pending = 'emergency_reset'
    api.command_lock.acquire()
    assert not api.command('emergency_reset', None, SimpleNamespace()).success
    assert api.command('zero_torque', None, SimpleNamespace()).success
    assert api.pending == 'emergency_reset'
    with pytest.raises(RuntimeError, match='interrupted'):
        api.reset_phase('proofs', time.monotonic()+1)
    api.command_lock.release()


def test_reset_owner_identity_change_is_not_accepted():
    api = harness()
    def restart(*args, **kwargs):
        api.state['owner_started_monotonic'] += 1
        api.reset_token = 'a'*32
        api.state.update(active_operation='operator_maintenance', operator_maintenance_token=api.reset_token)
        return 'jobs completed'
    api.restart = restart
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and 'identity changed' in reply.message


def test_payload_admission_accepts_empty_objects_but_rejects_stale_and_empty_images():
    api = harness()
    api.reset_samples = {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    header = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0))
    api.reset_stream('objects', SimpleNamespace(header=header, detections=[]))
    assert api.reset_samples['objects'][2] == 1
    api.reset_stream('objects', SimpleNamespace(header=header, detections=[]))
    assert api.reset_samples['objects'][2] == 1
    api.reset_stream('rgb_image', SimpleNamespace(header=header, width=1, height=1, data=[]))
    assert 'rgb_image' not in api.reset_samples
    api.reset_stream('rgb_image', SimpleNamespace(header=header, width=1, height=1, data=[0]))
    assert api.reset_samples['rgb_image'][2] == 1
    header.stamp.sec = 8
    api.reset_stream('depth_image', SimpleNamespace(header=header, width=1, height=1, data=[0]))
    assert 'depth_image' not in api.reset_samples


def test_endpoint_button_and_launch_configuration_wiring():
    assert 'emergency_reset' in NS['COMMANDS']
    panel = (ROOT/'ros2/src/so101_station_operator/src/operator_panel.cpp').read_text()
    assert '{"Restart entire station", "emergency_reset"}' in panel
    assert '{"Restart MoveIt", "restart_moveit"}' in panel
    assert '{"Reset (torque OFF)", "reset"}' in panel
    launch = (ROOT/'ros2/src/so101_station_operator/launch/operator.launch.py').read_text()
    for slot in ('movegroup_unit', 'objects_unit', 'scene_unit', 'leader_readonly_unit'):
        assert slot in launch


def test_reset_recipe_uses_one_token_deduplicates_camera_and_orders_before_recovery():
    api = harness()
    api.restart = API.restart.__get__(api)
    api.pending = 'emergency_reset'
    api.units = {'camera': 'so101-rgbd.service', 'depth': 'so101-rgbd.service', 'tags': 'so101-tags.service'}
    api.reset_units = {k: 'so101-'+k+'.service' for k in ('movegroup', 'objects', 'scene', 'leader_readonly')}
    api.reset_scene_instance = 'old'
    api.get_parameter = lambda name: SimpleNamespace(value=name == 'enable_service_restarts')
    api.service_clients = {k: SimpleNamespace(service_is_ready=lambda: True) for k in ('recovery_start', 'recovery_status')}
    requests = []
    def rpc(key, request=None, **kwargs):
        assert key == 'maintenance'
        assert api.all_off(api.state)
        requests.append(request)
        if request.enable and not request.token:
            assert request.operation == 'station_reset'
            api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
            return json.dumps({'token': 'a'*32})
        assert request.token == 'a'*32
        if request.enable:
            api.state['operator_restart_result'] = dict(job_id=request.unit, kind='restart_unit_terminal', success=True)
            return json.dumps({'job_id': request.unit})
        api.state['active_operation'] = None
        return '{}'
    api.rpc = rpc
    api.recover_perception = lambda token, deadline: requests.append(('recovery', token)) or 'b'*32
    api.reset_fresh_proofs = lambda token, deadline: requests.append(('fresh', token))
    api.restart('all', station_reset=True, deadline=time.monotonic()+110.)
    units = [r.unit for r in requests if not isinstance(r, tuple) and getattr(r, 'operation', None) == 'restart']
    assert units == ['so101-rgbd.service', 'so101-tags.service', 'so101-movegroup.service', 'so101-objects.service', 'so101-scene.service']
    assert requests[-2:] == [('recovery', 'a'*32), ('fresh', 'a'*32)]
    assert all(r.enable for r in requests if not isinstance(r, tuple))
    assert api.state['active_operation'] == 'operator_maintenance'


def test_scene_empty_status_needs_new_instance_and_source_stamp():
    api = harness()
    api.reset_samples = {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    api.reset_scene_instance = None
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(dict(state='empty', stamp=10., accepted=0))))
    assert not api.reset_samples
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(dict(state='empty', stamp=10., accepted=0, instance_id='a'*32, scene_busy=False, pending_count=0, scene_fault=None, live_ids=[], applied_world_ids=[]))))
    assert api.reset_scene_empty is True
    assert api.reset_samples['scene_status'][2] == 1


def test_station_reset_extended_units_rejected_under_normal_reservation():
    import importlib.util
    path = ROOT/'ros2/src/so101_station_control/so101_station_control/operator_restart.py'
    spec = importlib.util.spec_from_file_location('restart_for_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    restart = module.OperatorRestartMixin()
    restart._operator_restart_lock = threading.RLock()
    restart._operator_restart_units = {'so101-camera.service'}
    restart._operator_station_reset_units = {'so101-movegroup.service'}
    restart._operator_external_job = None
    restart._operator_maintenance_purpose = None
    with pytest.raises(RuntimeError, match='not allowlisted'):
        restart._operator_restart_begin('a'*32, 'so101-movegroup.service')


def test_empty_scene_with_pending_removal_cannot_advance_proof():
    api = harness()
    api.reset_samples = {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    data = dict(state='empty', stamp=10., accepted=0, instance_id='a'*32,
                scene_busy=True, pending_count=1, scene_fault=None,
                live_ids=[], applied_world_ids=['colorbrick_old'])
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert 'scene_status' not in api.reset_samples
    data.update(scene_busy=False, pending_count=0)
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert 'scene_status' not in api.reset_samples
    data['applied_world_ids'] = []
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert api.reset_samples['scene_status'][2] == 1


def test_failed_fresh_proofs_do_not_release_maintenance_reservation():
    api = harness()
    api.restart = API.restart.__get__(api)
    api.pending = 'emergency_reset'
    api.reset_units['leader_readonly'] = ''
    api.reset_scene_instance = None
    api.service_clients = {k: SimpleNamespace(service_is_ready=lambda: True) for k in ('recovery_start', 'recovery_status')}
    requests = []
    def rpc(key, request=None, **kwargs):
        requests.append(request)
        if request.enable and not request.token:
            api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
            return json.dumps({'token': 'a'*32})
        if request.enable:
            api.state['operator_restart_result'] = dict(job_id=request.unit, kind='restart_unit_terminal', success=True)
            return json.dumps({'job_id': request.unit})
        raise AssertionError('failed reset must retain reservation')
    api.rpc = rpc
    api.recover_perception = lambda *args: 'b'*32
    def stale(*args):
        raise RuntimeError('source stamp stale')
    api.reset_fresh_proofs = stale
    with pytest.raises(RuntimeError, match='stale'):
        api.restart('all', station_reset=True, deadline=time.monotonic()+110)
    assert api.state['active_operation'] == 'operator_maintenance'
    assert all(request.enable for request in requests)


def test_async_scene_apply_keeps_exact_request_source_stamp():
    path = ROOT/'ros2/src/so101_station_scene/so101_station_scene/scene_updater.py'
    tree = ast.parse(path.read_text())
    tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SceneUpdater']
    namespace = dict(Node=object, CollisionObject=SimpleNamespace(REMOVE=1),
                     stamp_seconds=lambda stamp: stamp.sec + stamp.nanosec*1e-9,
                     ApplyPlanningScene=SimpleNamespace(Request=lambda: SimpleNamespace()),
                     PlanningScene=lambda: SimpleNamespace(robot_state=SimpleNamespace(), world=SimpleNamespace()))
    exec(compile(tree, str(path), 'exec'), namespace)
    scene = namespace['SceneUpdater'].__new__(namespace['SceneUpdater'])
    scene._pending_by_id, scene._applied_world_ids = {}, set()
    scene._scene_busy, scene._scene_fault = False, None
    scene.max_objects = 24
    callbacks, statuses = [], []
    def call_async(request):
        return SimpleNamespace(add_done_callback=lambda callback: callbacks.append(callback))
    scene.scene_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)
    scene._status = lambda state, **fields: statuses.append((state, fields))
    item = lambda stamp: SimpleNamespace(id='colorbrick_A', operation=0, header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)))
    scene._apply([item(10)])
    scene._apply([item(11)])
    callbacks[0](SimpleNamespace(result=lambda: SimpleNamespace(success=True)))
    assert statuses[0][0] == 'applied'
    assert statuses[0][1]['source_stamp'] == 10
    callbacks[1](SimpleNamespace(result=lambda: SimpleNamespace(success=True)))
    assert statuses[1][1]['source_stamp'] == 11


@pytest.mark.parametrize("bad_scene", [None, "mismatch", "stale", "missing", "nan", "delayed"])
def test_fresh_proofs_reject_old_metadata_and_accept_advancing_empty_workspace(monkeypatch, bad_scene):
    api = harness()
    api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
    api.reset_scene_started = 0.
    api.reset_scene_instance, api.reset_previous_scene_instance = 'new', 'old'
    api.reset_scene_empty, api.reset_scene_applied = True, None
    api.reset_scene_world_ids = []
    keys = ('rgb_image', 'depth_image', 'objects', 'scene_status', 'joint_states')
    api.reset_samples = {key: (10., 0., 1) for key in keys}
    api.stream_stamps, api.streams = {'tags': 10}, {'tags': 0.}
    clock = SimpleNamespace(value=0.)
    def sleep(_):
        clock.value += .1
    monkeypatch.setitem(NS, 'time', SimpleNamespace(monotonic=lambda: clock.value, sleep=sleep))
    with pytest.raises(RuntimeError, match='deadline'):
        api.reset_fresh_proofs('a'*32, .3)
    # Actual new samples must advance each source twice; an empty workspace is valid.
    clock.value = 0.
    def advancing(_):
        clock.value += .1
        api.reset_samples = {key: (10.1+clock.value, clock.value, int(clock.value*10)+3) for key in keys}
        if bad_scene != "stale" or api.reset_measured_joints is None:
            api.reset_measured_joints = (10.1+clock.value, clock.value, int(clock.value*10)+3, dict.fromkeys(NS["JOINTS"], 0.))
        api.stream_stamps['tags'] = 11
        api.streams['tags'] = clock.value
    NS['time'].sleep = advancing
    monkeypatch.setitem(NS, 'GetPlanningScene', SimpleNamespace(Request=lambda: SimpleNamespace(components=SimpleNamespace())))
    monkeypatch.setitem(NS, 'PlanningSceneComponents', SimpleNamespace(ROBOT_STATE=1, WORLD_OBJECT_NAMES=2))
    api.fresh_source_admissions = lambda *args: None
    joints = SimpleNamespace(name=list(NS['JOINTS']), position=[0.]*6,
                             header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
    if bad_scene == "mismatch":
        joints.position[0] = .011
    if bad_scene == "missing":
        joints.name.pop(); joints.position.pop()
    if bad_scene == "nan":
        joints.position[0] = float("nan")
    result = SimpleNamespace(scene=SimpleNamespace(robot_state=SimpleNamespace(joint_state=joints), world=SimpleNamespace(collision_objects=[])))
    future = SimpleNamespace(add_done_callback=lambda callback: callback(None), result=lambda: result)
    api.scene_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_350_000_000 if bad_scene == "delayed" else 10_100_000_000))
    if bad_scene in (None, "delayed"):
        api.reset_fresh_proofs('a'*32, 1.)
        if bad_scene == "delayed":
            # The 10.3 source sample arrives after dispatch but predates its ROS floor.
            assert clock.value == pytest.approx(.3)
            assert api.reset_measured_joints[0] > 10.35
    else:
        with pytest.raises(RuntimeError, match="disagree|deadline|complete finite"):
            api.reset_fresh_proofs('a'*32, 1.)


def test_incomplete_recipe_stops_before_reporting_configuration_failure():
    api = harness(owner(torque={j: 1 for j in NS['JOINTS']}))
    api.units['camera'] = ''
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and 'allowlists' in reply.message
    assert api.calls == ['stop', 'stop']
    assert api.all_off(api.state)


@pytest.mark.parametrize("rotate_epoch", [False, True])
def test_display_alignment_reseed_requires_new_advancing_matching_source(monkeypatch, rotate_epoch):
    api = harness()
    api.reset_alignment = API.reset_alignment.__get__(api)
    api.get_parameter = lambda name: SimpleNamespace(value='/so101_perception/live_tag_alignment/reseed')
    api.stream_stamps = {'tags': 10_000_000_000}
    api.tag_source_context = dict(consumer_epoch='a'*32, source_pin_sha256='b'*64, producer_generation='c'*32, source_generation='d'*64, camera_serial='fake')
    api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_500_000_000))
    clock = SimpleNamespace(value=0.)
    exposures = iter([10_100_000_000, 10_200_000_000, 10_300_000_000] if rotate_epoch else [10_100_000_000, 10_100_000_000, 10_200_000_000])
    def sample(_):
        clock.value += .1
        exposure = next(exposures)
        if rotate_epoch and clock.value >= .2:
            api.tag_source_context = {**api.tag_source_context, "consumer_epoch": "e"*32}
        api.alignment_status = (clock.value, dict(schema='so101-live-tag-alignment-v1',
            stamp={'sec': exposure//1_000_000_000, 'nanosec': exposure%1_000_000_000},
            candidate_available=True, display_mode='tag_alignment', qualified=False,
            canonical_control_tf_modified=False, independent_device_info_identity_current=True,
            source_context=api.tag_source_context))
    monkeypatch.setitem(NS, 'time', SimpleNamespace(monotonic=lambda: clock.value, sleep=sample))
    api.reset_alignment('a'*32, 1.)
    assert api.calls == ['alignment_reseed']
    assert clock.value == pytest.approx(.3)


def test_receipt_callbacks_do_not_block_or_call_services():
    tree = ast.parse(SCRIPT.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    for method in cls.body:
        if isinstance(method, ast.FunctionDef) and method.name in {'stream', 'reset_stream', 'alignment_stream', 'reset_image'}:
            for node in ast.walk(method):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    assert node.func.attr not in {'sleep', 'wait', 'rpc', 'snapshot', 'call_async', 'run', 'read_bytes'}


def test_emergency_busy_home_lock_still_stops_without_dispatching_recipe():
    api = harness(owner(torque={j: 1 for j in NS['JOINTS']}))
    class BusyLock:
        def acquire(self, **kwargs):
            assert kwargs == {'timeout': 3.}
            return False
    api.command_lock = BusyLock()
    api.pending = 'start'
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and 'not quiesced' in reply.message
    assert api.calls == ['stop']
    assert api.all_off(api.state)
    assert api.pending == 'start'


def test_emergency_unknown_motion_stops_first_without_clearing_uncertainty():
    api = harness()
    api.uncertain = api.uncertain_motion = True
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success
    assert api.calls == ['stop']
    assert api.uncertain is api.uncertain_motion is True


def test_tag_receipt_cannot_refresh_stale_source_exposure():
    api = harness()
    api.streams, api.stream_stamps, api.stream_errors = {'tags': None}, {'tags': -1}, {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    api.stream('tags', SimpleNamespace(data=json.dumps(dict(schema='so101-tag-detections-v1',
        stamp={'sec': 8, 'nanosec': 0}, observations=[]))))
    assert api.streams['tags'] is None
    assert 'stale' in api.stream_errors['tags']


def test_external_stop_intent_while_initial_stop_lock_busy_cancels_recipe():
    api = harness()
    entered, release = threading.Event(), threading.Event()
    original = api.rpc
    def blocked(key, *args, **kwargs):
        if key == 'stop' and not entered.is_set():
            entered.set()
            assert release.wait(2.)
        return original(key, *args, **kwargs)
    api.rpc = blocked
    result = []
    thread = threading.Thread(target=lambda: result.append(api.command('emergency_reset', None, SimpleNamespace())))
    thread.start()
    assert entered.wait(1.)
    stopped = api.command('zero_torque', None, SimpleNamespace())
    assert not stopped.success  # Existing STOP write is already in flight.
    release.set()
    thread.join(2.)
    assert not thread.is_alive()
    assert not result[0].success and 'interrupted' in result[0].message
    assert api.calls == ['stop']
    assert api.stop_generation == 2


def test_final_owner_proof_cannot_complete_after_global_deadline(monkeypatch):
    api = harness()
    clock = SimpleNamespace(value=0.)
    monkeypatch.setitem(NS, 'time', SimpleNamespace(monotonic=lambda: clock.value))
    original = api.rpc
    def rpc(key, *args, **kwargs):
        reply = original(key, *args, **kwargs)
        return reply
    api.rpc = rpc
    original_snapshot = api.snapshot
    def snapshot():
        if clock.value == 109.:
            clock.value = 112.
        return original_snapshot()
    api.snapshot = snapshot
    def restart(*args, **kwargs):
        clock.value = 109.
        api.reset_token = 'a'*32
        api.state.update(active_operation='operator_maintenance', operator_maintenance_token=api.reset_token)
        return 'proofs accepted'
    api.restart = restart
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and 'deadline' in reply.message
    assert api.reset_progress['initialized_all_off'] is False


def test_bounded_status_parsers_project_and_reject_malformed_payloads():
    api = harness()
    for payload in ('[]', 'null', '"text"', 'x'*65537):
        api.alignment_stream(SimpleNamespace(data=payload))
        assert api.alignment_status is None
    context = dict(consumer_epoch='a'*32, source_pin_sha256='b'*64,
                   producer_generation='c'*32, source_generation='d'*64, camera_serial='fake', extra='discard')
    api.alignment_stream(SimpleNamespace(data=json.dumps(dict(schema='so101-live-tag-alignment-v1',
        source_context=context, arbitrary={'object': [1, 2, 3]}, reason='r'*1000))))
    projected = api.alignment_status[1]
    assert 'arbitrary' not in projected and 'extra' not in projected['source_context']
    assert len(projected['reason']) == 512
    api.reset_samples = {}
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(dict(state='empty', instance_id='a'*32,
        live_ids=['colorbrick_x']*25, applied_world_ids=[], scene_busy=False, pending_count=0, stamp=10.))))
    assert not api.reset_samples


def test_release_is_final_backend_operation_after_all_reserved_proofs():
    api = harness()
    released = SimpleNamespace(value=False)
    snapshot = api.snapshot
    rpc = api.rpc
    def checked_snapshot():
        assert not released.value, 'no dependent backend proof is allowed after release'
        return snapshot()
    def release_rpc(key, request=None, **kwargs):
        if key == 'maintenance' and request.enable is False:
            assert api.state['active_operation'] == 'operator_maintenance'
            released.value = True
            api.state.update(active_operation=None, operator_maintenance_token=None)
        return rpc(key, request, **kwargs)
    api.snapshot, api.rpc = checked_snapshot, release_rpc
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert reply.success, reply.message
    assert api.calls[-1] == 'maintenance'
    assert api.calls.count('stop') == 3


def test_unknown_release_ack_attempts_bounded_stop_without_claiming_retained_token():
    api = harness()
    rpc = api.rpc
    def lost_ack(key, request=None, **kwargs):
        if key == 'maintenance' and request.enable is False:
            api.state.update(active_operation=None, operator_maintenance_token=None)
            raise RuntimeError('maintenance acknowledgement timed out; outcome unknown')
        if key == 'stop' and api.reset_progress and api.reset_progress['phase'] == 'final acceptance before release':
            assert kwargs['timeout'] == 2.
        return rpc(key, request, **kwargs)
    api.rpc = lost_ack
    reply = api.command('emergency_reset', None, SimpleNamespace())
    assert not reply.success and api.uncertain is True
    assert 'reservation may already be released' in reply.message
    assert 'bounded STOP confirmed' in reply.message
    assert api.calls[-1] == 'stop'
    assert api.reset_progress['initialized_all_off'] is False
    assert api.state['operator_maintenance_token'] is None


def test_rpc_caps_wait_by_remaining_global_budget_and_rejects_late_ack(monkeypatch):
    api = harness()
    clock = SimpleNamespace(value=0.)
    monkeypatch.setitem(NS, 'time', SimpleNamespace(monotonic=lambda: clock.value))
    waits = []
    class Event:
        def set(self):
            pass
        def wait(self, timeout):
            waits.append(timeout)
            return True
    monkeypatch.setitem(NS, 'threading', SimpleNamespace(Event=Event))
    def result():
        clock.value = .2
        return SimpleNamespace(success=True, message='ack')
    future = SimpleNamespace(add_done_callback=lambda callback: callback(None), result=result)
    api.service_clients = {'maintenance': SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: future)}
    api.reset_rpc_deadline.value = .1
    with pytest.raises(RuntimeError, match='deadline elapsed after'):
        API.rpc(api, 'maintenance', object(), timeout=8.)
    assert waits == [.1]


def test_scene_status_projects_internal_fault_future_to_boolean():
    path = ROOT/'ros2/src/so101_station_scene/so101_station_scene/scene_updater.py'
    tree = ast.parse(path.read_text())
    tree.body = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'SceneUpdater']
    namespace = dict(Node=object, json=json, String=lambda: SimpleNamespace())
    exec(compile(tree, str(path), 'exec'), namespace)
    scene = namespace['SceneUpdater'].__new__(namespace['SceneUpdater'])
    scene._instance_id, scene._scene_busy = 'a'*32, False
    scene._pending_by_id, scene._applied_world_ids = {}, set()
    scene.registry = SimpleNamespace(_objects={})
    scene._scene_fault = {'future': object(), 'data': {'arbitrary': object()}}
    emitted = []
    scene.status_pub = SimpleNamespace(publish=lambda message: emitted.append(json.loads(message.data)))
    scene._status('empty', stamp=10., accepted=0)
    assert emitted[0]['scene_fault'] is True
    assert 'future' not in json.dumps(emitted[0])


@pytest.mark.parametrize('rejected_stamp', [8., 11., 9.5, float('nan')])
def test_rejected_scene_stamp_cannot_mutate_admitted_cohort(rejected_stamp):
    api = harness()
    api.reset_samples = {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    api.reset_scene_applied = None
    data = dict(state='updated', stamp=10., accepted=1, instance_id='a'*32,
                scene_busy=False, pending_count=0, scene_fault=False,
                live_ids=['colorbrick_A'], applied_world_ids=['colorbrick_A'])
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    admitted = (api.reset_scene_instance, list(api.reset_scene_world_ids), api.reset_scene_empty, dict(api.reset_samples))
    data.update(state='empty', stamp=rejected_stamp, accepted=0, instance_id='b'*32,
                live_ids=[], applied_world_ids=[])
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert (api.reset_scene_instance, api.reset_scene_world_ids, api.reset_scene_empty, api.reset_samples) == admitted


def test_same_admitted_scene_stamp_ack_updates_application_without_count_renewal():
    api = harness()
    api.reset_samples, api.reset_scene_applied = {}, None
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    data = dict(state='updated', stamp=10., accepted=1, instance_id='a'*32,
                scene_busy=False, pending_count=0, scene_fault=False,
                live_ids=['colorbrick_A'], applied_world_ids=['colorbrick_A'])
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    initial = api.reset_samples['scene_status']
    data.update(state='applied', source_stamp=10.)
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert api.reset_scene_applied[0] == 'a'*32
    assert api.reset_scene_applied[2] == 10.
    assert api.reset_samples['scene_status'] == initial
    data['instance_id'] = 'b'*32
    api.reset_stream('scene_status', SimpleNamespace(data=json.dumps(data)))
    assert api.reset_scene_instance == 'a'*32


def image_lifecycle_harness(monkeypatch):
    api = harness()
    api.activate_reset_images = API.activate_reset_images.__get__(api)
    api.retire_reset_images = API.retire_reset_images.__get__(api)
    api.reset_image_lock = threading.Lock()
    api.reset_image_generation, api.reset_image_source_after = 0, 0
    api.reset_image_subscriptions, api.reset_samples = [], {}
    clock = SimpleNamespace(source_ns=10_000_000_000)
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock.source_ns))
    api.get_parameter = lambda name: SimpleNamespace(value='/'+name)
    monkeypatch.setitem(NS, 'Image', object())
    monkeypatch.setitem(NS, 'QoSProfile', lambda **kwargs: kwargs)
    monkeypatch.setitem(NS, 'ReliabilityPolicy', SimpleNamespace(BEST_EFFORT='best_effort'))
    created, destroyed, active = [], [], []
    def create(kind, topic, callback, qos):
        assert qos == dict(depth=1, reliability='best_effort')
        subscription = SimpleNamespace(topic=topic, callback=callback)
        created.append(subscription)
        active.append(subscription)
        return subscription
    def destroy(subscription):
        active.remove(subscription)
        destroyed.append(subscription)
    api.create_subscription, api.destroy_subscription = create, destroy
    return api, clock, created, destroyed, active


@pytest.mark.parametrize('outcome', ['pass', 'failure', 'stop', 'unknown'])
def test_image_subscriptions_exist_only_in_reset_proof_window(monkeypatch, outcome):
    api, clock, created, destroyed, active = image_lifecycle_harness(monkeypatch)
    assert active == []
    def proofs(token, deadline):
        assert len(active) == 2
        if outcome != 'pass':
            if outcome == 'stop':
                api.reset_cancel.set()
                api.reset_phase('proof', deadline)
            if outcome == 'unknown':
                api.uncertain = True
            raise RuntimeError(outcome)
        return 'accepted'
    api._reset_fresh_proofs = proofs
    if outcome == 'pass':
        assert api.reset_fresh_proofs('a'*32, time.monotonic()+1.) == 'accepted'
    else:
        with pytest.raises(RuntimeError):
            api.reset_fresh_proofs('a'*32, time.monotonic()+1.)
    assert len(created) == len(destroyed) == 2
    assert active == api.reset_image_subscriptions == []
    api.retire_reset_images()
    assert len(destroyed) == 2


def test_retired_image_generation_and_pre_activation_exposures_cannot_prove_freshness(monkeypatch):
    api, clock, created, destroyed, active = image_lifecycle_harness(monkeypatch)
    def frame(nanosec):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=nanosec)),
                               width=1, height=1, data=[0])
    api.activate_reset_images()
    old = created[0].callback
    old(frame(0))
    assert api.reset_samples == {}
    clock.source_ns = 10_200_000_000
    old(frame(100_000_000))
    assert api.reset_samples['rgb_image'][2] == 1
    api.retire_reset_images()
    old(frame(150_000_000))
    assert api.reset_samples['rgb_image'][2] == 1
    api.activate_reset_images()
    assert 'rgb_image' not in api.reset_samples
    clock.source_ns = 10_400_000_000
    old(frame(300_000_000))
    assert api.reset_samples == {}
    created[-2].callback(frame(300_000_000))
    created[-2].callback(frame(350_000_000))
    assert api.reset_samples['rgb_image'][2] == 2
    api.retire_reset_images()
    assert len(created) == len(destroyed) == 4
    assert active == []


def test_partial_image_subscription_activation_cleans_up(monkeypatch):
    api, clock, created, destroyed, active = image_lifecycle_harness(monkeypatch)
    create = api.create_subscription
    def failing_create(*args):
        if created:
            raise RuntimeError('second subscription failed')
        return create(*args)
    api.create_subscription = failing_create
    with pytest.raises(RuntimeError, match='second subscription'):
        api.activate_reset_images()
    assert len(created) == len(destroyed) == 1
    assert active == []


def test_normal_commands_do_not_allocate_image_subscriptions(monkeypatch):
    api, clock, created, destroyed, active = image_lifecycle_harness(monkeypatch)
    assert api.command('zero_torque', None, SimpleNamespace()).success
    assert created == destroyed == active == []
    api.pending = 'emergency_reset'
    api.command_lock.acquire()
    try:
        assert not api.command('emergency_reset', None, SimpleNamespace()).success
    finally:
        api.command_lock.release()
    assert created == destroyed == active == []
    source = SCRIPT.read_text()
    init = next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef))
    init = next(node for node in init.body if isinstance(node, ast.FunctionDef) and node.name == '__init__')
    assert not any(isinstance(node, ast.Name) and node.id == 'Image' for node in ast.walk(init))


def adopted_binding():
    return dict(producer_generation='c'*32, source_pin_sha256='b'*64,
                consumer_epochs={k: 'a'*32 for k in ('/so101_charuco_world', '/so101_colored_objects')})


def completed_recovery(**updates):
    binding = adopted_binding()
    state = dict(job_id='e'*32, state='completed', operation_success=True, ready_now=False,
                 producer_generation=binding['producer_generation'], source_pin_sha256=binding['source_pin_sha256'],
                 consumers={k: dict(latest=dict(consumer_epoch=v, producer_generation=binding['producer_generation'],
                     source_pin_sha256=binding['source_pin_sha256'])) for k, v in binding['consumer_epochs'].items()})
    state.update(updates)
    return state


@pytest.mark.parametrize('failure', [None, 'failed', 'wrong_job', 'no_identity', 'unknown'])
def test_recovery_adopts_historical_completion_without_renewing_original_lease(monkeypatch, failure):
    api = harness()
    api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
    state = completed_recovery()
    if failure == 'failed': state.update(state='failed', operation_success=False)
    if failure == 'wrong_job': state['job_id'] = 'f'*32
    if failure == 'no_identity': state.pop('source_pin_sha256')
    if failure == 'unknown': state.update(state='unknown', operation_success=False)
    replies = []
    def reply(key, timeout):
        replies.append(key)
        if len(replies) > 3: raise RuntimeError('test bounded unresolved job')
        return dict(accepted=True, job_id='e'*32) if key == 'recovery_start' else state
    api.recovery_reply = reply
    if failure:
        with pytest.raises(RuntimeError):
            api.recover_perception('a'*32, time.monotonic()+50.)
        assert api.recovery_binding is None
    else:
        assert api.recover_perception('a'*32, time.monotonic()+50.) == 'e'*32
        assert api.recovery_binding == adopted_binding()
        assert replies == ['recovery_start', 'recovery_status']


@pytest.mark.parametrize('failure', [None, 'generation', 'pin', 'epoch', 'expired', 'old_header', 'tags'])
def test_current_checked_admissions_must_match_historical_recovery(failure):
    api = harness()
    api.state.update(active_operation='operator_maintenance', operator_maintenance_token='a'*32)
    api.recovery_binding = adopted_binding()
    api.tag_source_context = dict(consumer_epoch='a'*32, producer_generation='c'*32, source_pin_sha256='b'*64)
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    proof = dict(schema='so101-derived-admission-status-v1', state='ready', qualified=False, execution_enabled=False,
                 consumer_epoch='a'*32, producer_generation='c'*32, source_pin_sha256='b'*64,
                 original_receipt_expiry_monotonic=time.monotonic()+1., original_header_expiry_ros_ns=10_200_000_000,
                 checked_ros_ns=9_999_999_999)
    if failure == 'generation': proof['producer_generation'] = 'f'*32
    if failure == 'pin': proof['source_pin_sha256'] = 'f'*64
    if failure == 'epoch': proof['consumer_epoch'] = 'f'*32
    if failure == 'expired': proof['original_receipt_expiry_monotonic'] = time.monotonic()-1.
    if failure == 'old_header': proof['original_header_expiry_ros_ns'] = 9_000_000_000
    if failure == 'tags': api.tag_source_context['source_pin_sha256'] = 'f'*64
    api.recovery_reply = lambda *args: proof
    if failure:
        with pytest.raises(RuntimeError, match='admission|context'):
            api.fresh_source_admissions('a'*32, time.monotonic()+2.)
    else:
        api.fresh_source_admissions('a'*32, time.monotonic()+2.)


def test_reset_joint_payload_retains_only_bounded_admitted_advancing_measurement():
    api = harness()
    api.reset_samples = {}
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
                              name=list(NS['JOINTS']), position=[.1]*6)
    api.reset_stream('joint_states', message)
    first = api.reset_measured_joints
    message.position[0] = .2
    api.reset_stream('joint_states', message)
    assert api.reset_measured_joints == first
    message.header.stamp.nanosec = 1
    api.reset_stream('joint_states', message)
    assert api.reset_measured_joints[2] == first[2]+1
    assert api.reset_measured_joints[3][message.name[0]] == .2


def test_current_checked_admissions_expire_during_final_owner_observation(monkeypatch):
    api = harness()
    api.recovery_binding = adopted_binding()
    api.tag_source_context = dict(consumer_epoch='a'*32, producer_generation='c'*32, source_pin_sha256='b'*64)
    clock = SimpleNamespace(value=10., owners=0)
    monkeypatch.setitem(NS, 'time', SimpleNamespace(monotonic=lambda: clock.value))
    api.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock.value*1e9)))
    proof = dict(schema='so101-derived-admission-status-v1', state='ready', qualified=False, execution_enabled=False,
                 consumer_epoch='a'*32, producer_generation='c'*32, source_pin_sha256='b'*64,
                 original_receipt_expiry_monotonic=10.1, original_header_expiry_ros_ns=10_100_000_000,
                 checked_ros_ns=10_000_000_000)
    api.recovery_reply = lambda *args: proof
    def owner_observation(token):
        clock.owners += 1
        if clock.owners == 3:
            clock.value = 10.2
    api.maintenance_state = owner_observation
    with pytest.raises(RuntimeError, match='admission expired'):
        api.fresh_source_admissions('a'*32, 12.)
    assert clock.owners == 3
