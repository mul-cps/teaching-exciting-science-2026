"""Static authority/queue bounds; no ROS initialization or live calls."""
import ast
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_perception/so101_station_perception'


def test_metadata_only_and_coordinator_is_sole_recovery_authority():
    text=(SOURCE/'perception_supervisor.py').read_text();tree=ast.parse(text)
    imports={alias.name for node in ast.walk(tree) if isinstance(node,(ast.Import,ast.ImportFrom)) for alias in node.names}
    assert not imports.intersection({'Image','PointCloud2','subprocess','serial','pyrealsense2','torch'})
    assert 'reset_source' not in text and 'start_recovery' in text
    assert 'deque(maxlen=8)' in text and 'deque(maxlen=4)' in text
    assert 'create_timer(.2' in text
    dispatch=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_dispatch')
    calls=[ast.unparse(n) for n in dispatch.body]
    reserve=next(i for i,s in enumerate(calls) if 'self.policy.reserve(' in s)
    persist=next(i for i,s in enumerate(calls) if i>reserve and 'self._persist()' in s)
    rpc=next(i for i,s in enumerate(calls) if "self._request(COORDINATOR + '/start_recovery'" in s)
    assert reserve<persist<rpc


def test_typed_producer_fault_is_used_at_exact_two_existing_gap_sites():
    tree=ast.parse((SOURCE/'rocm_rgbd.py').read_text())
    raises=[n for n in ast.walk(tree) if isinstance(n,ast.Raise) and isinstance(n.exc,ast.Call) and
            isinstance(n.exc.func,ast.Name) and n.exc.func.id=='TypedSourceReceiptGap']
    assert len(raises)==2
    assert {n.exc.args[0].value for n in raises}=={'private_source_receipt_gap','private_source_stream_lost'}


def test_health_completion_guard_never_starts_queries_and_timer_queries_are_capped():
    tree=ast.parse((SOURCE/'perception_supervisor.py').read_text())
    methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    wake=ast.unparse(methods['_health_wakeup'])
    assert '_request(' not in wake and '_health_queries(' not in wake
    assert '_current_health(start_requests=False)' in wake
    query=ast.unparse(methods['_health_queries'])
    assert 'endpoint not in self.rpc' in query and '+ 0.1' in query
    complete=ast.unparse(methods['completed'])
    assert '_request(' not in complete and 'health_guard.trigger()' in complete


def test_health_starter_and_control_have_separate_fixed_cadence_and_close_gate():
    tree=ast.parse((SOURCE/'perception_supervisor.py').read_text())
    methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
    control=ast.unparse(methods['_step']);validate=ast.unparse(methods['_current_health'])
    refresh=ast.unparse(methods['_health_refresh']);close=ast.unparse(methods['close'])
    assert '_health_queries(' not in control and '_health_queries(' not in validate
    assert '_dispatch(' not in refresh and '_monitor_job(' not in refresh and '_step(' not in refresh
    assert 'self.health_timer.reset()' in refresh and 'self.timer.reset()' not in refresh
    assert close.index('self.closed = True')<close.index('self.health_timer.cancel()')
    assert not any(isinstance(n,ast.While) for n in ast.walk(methods['_health_queries']))
