"""Offline restart lifecycle tests; no ROS imports or reader processes."""
import ast
from pathlib import Path
import queue
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_control/so101_station_control/leader_teleop.py'
TREE=ast.parse(SOURCE.read_text())
CLASS=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='LeaderTeleopPublisher')
METHOD=next(n for n in CLASS.body if isinstance(n,ast.FunctionDef) and n.name=='_supervise_reader')


def supervisor(start):
    clock=SimpleNamespace(value=0.)
    namespace={'queue':queue,'time':SimpleNamespace(monotonic=lambda:clock.value)}
    exec(compile(ast.Module(body=[METHOD],type_ignores=[]),str(SOURCE),'exec'),namespace)
    messages=[]
    node=SimpleNamespace(_reader_closed=False,_process=None,_reader_retry_at=0.,
        _reader_retry_delay=.5,_latest=queue.Queue(maxsize=1),_leader_ready_reported=True,
        _start_reader=lambda:start(node),get_logger=lambda:SimpleNamespace(warning=messages.append))
    return node,clock,lambda:namespace['_supervise_reader'](node),messages


def test_missing_reader_retries_without_dead_publisher_and_clears_old_packet():
    attempts=[]
    def start(node):
        attempts.append(True)
        if len(attempts)<3: raise FileNotFoundError('leader missing')
        node._process=SimpleNamespace(poll=lambda:None)
        return 'reader-thread'
    node,clock,tick,messages=supervisor(start)
    tick()
    assert node._reader_retry_at==.5 and len(attempts)==1
    clock.value=.25;tick();assert len(attempts)==1
    clock.value=.5;tick();assert node._reader_retry_at==1.5 and len(attempts)==2
    node._latest.put({'old':True})
    clock.value=1.5;tick()
    assert len(attempts)==3 and node._process.poll() is None
    assert node._latest.empty() and node._leader_ready_reported is False
    assert len(messages)==2


def test_disconnected_reader_restarts_with_capped_backoff():
    attempts=[]
    def start(node):
        attempts.append(True)
        node._process=SimpleNamespace(poll=lambda:1,returncode=1)
    node,clock,tick,_=supervisor(start)
    tick()
    for _ in range(12):
        tick()  # Observe exited child and schedule its replacement.
        assert .5<=node._reader_retry_delay<=8.
        clock.value=node._reader_retry_at
        tick()
    assert len(attempts)==13 and node._reader_retry_delay==8.


def test_shutdown_never_starts_another_reader():
    node,_,tick,_=supervisor(lambda _:(_ for _ in ()).throw(AssertionError('restart after shutdown')))
    node._reader_closed=True
    tick()


def test_default_resident_launch_starts_one_read_only_leader_with_explicit_opt_out():
    source=(ROOT/'ros2/src/so101_station_control/launch/moveit_resident.launch.py').read_text()
    assert 'DeclareLaunchArgument("leader", default_value="true"' in source
    assert 'condition=IfCondition(LaunchConfiguration("leader"))' in source
    assert '"leader_teleop.launch.py"' in source
    wrapper=(ROOT/'ros2/src/so101_station_control/launch/leader_follower_teleop.launch.py').read_text()
    assert '"leader": "false"' in wrapper
    assert '"leader_teleop.launch.py"' in wrapper


def test_restart_retains_existing_packet_validation_and_read_only_device_contract():
    reader=(ROOT/'ros2/src/so101_station_control/so101_station_control/leader_reader.py').read_text()
    assert 'bus.connect(handshake=False)' in reader
    assert 'if not bus.is_calibrated:' in reader
    assert 'bus.disconnect(disable_torque=False)' in reader
    assert 'bus.write(' not in reader and 'sync_write(' not in reader
    source=SOURCE.read_text()
    assert '5AB9068765-if00' in source
    assert 'validate_pose_packet(' in source
    supervisor_calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id for n in ast.walk(METHOD)
        if isinstance(n,ast.Call) and isinstance(n.func,(ast.Attribute,ast.Name))}
    assert not supervisor_calls&{'validate_pose_packet','create_subscription','enable_teleop','write','sync_write'}
