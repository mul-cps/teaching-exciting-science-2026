"""AST guards keep cold admission small and cancellation wait non-spinning."""
import ast
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / 'tools/run_blockly_station.py'
TREE = ast.parse(SOURCE.read_text())

def method(cls, name):
    owner = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == cls)
    return next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == name)


def test_cold_admission_has_no_io_or_per_item_loop():
    for name in ('_check_admission', '_begin_control', '_check_control'):
        node = method('Application', name)
        assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.DictComp, ast.SetComp)) for n in ast.walk(node))
        names = {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}
        assert not names & {'read_bytes', 'read_text', 'open', 'sha256', 'glob', 'sleep', 'join'}


def test_combined_wait_yields_at_most_twenty_ms_between_parent_checks():
    node = method('_ControlCancellation', 'wait')
    waits = [n for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'wait']
    assert len(waits) == 1
    clamp = waits[0].args[0]
    assert isinstance(clamp, ast.Call) and isinstance(clamp.func, ast.Name) and clamp.func.id == 'min'
    assert isinstance(clamp.args[0], ast.Constant) and clamp.args[0].value == .02
    assert any(isinstance(n, ast.Compare) and isinstance(n.ops[0], ast.LtE) for n in ast.walk(node))
