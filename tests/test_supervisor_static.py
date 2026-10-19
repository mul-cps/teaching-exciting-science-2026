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
