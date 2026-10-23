"""Cold bounded catalog and allocation-only status fence."""
import ast
from pathlib import Path
import statistics
import time
from workshop.blockly_skills import SkillLibrary

ROOT=Path(__file__).resolve().parents[1]


def test_status_no_artifact_io_and_discovery_bound():
    tree=ast.parse((ROOT/'workshop/blockly_skills.py').read_text())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef))
    status=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='status')
    assert {ast.unparse(node.func) for node in ast.walk(status) if isinstance(node,ast.Call)}=={'copy.deepcopy'}
    init=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='__init__')
    bounded=next(node for node in ast.walk(init) if isinstance(node,ast.Call) and ast.unparse(node.func)=='islice')
    assert len(bounded.args)==2 and ast.literal_eval(bounded.args[1])==64
    assert any(isinstance(node,ast.Call) and ast.unparse(node.func)=='sorted' and node.args[0] is bounded for node in ast.walk(init))


def test_cached_status_microbenchmark(capsys):
    library=SkillLibrary(None)
    library.root=Path('/unread/private/library')
    library._catalog={f'skill{i}':{'state':'revalidation_required','reason':'live admission required'} for i in range(64)}
    samples=[]
    for _ in range(1000):
        start=time.perf_counter_ns();library.status();samples.append(time.perf_counter_ns()-start)
    p95=sorted(samples)[949]/1000
    print(f'taught_status_64 median_us={statistics.median(samples)/1000:.2f} p95_us={p95:.2f}')
    assert p95<10000 # generous CPU-only bound; no disk/camera/ROS in feedback
