"""Guard the diagnostic collector's pure classifier and steady timing boundary."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace

from tools.performance.resident_live_status import status_ready

ROOT = Path(__file__).resolve().parents[1]
COLLECTOR = ROOT / "tools/performance/collect_resident_live.py"
HELPER = ROOT / "tools/performance/resident_live_status.py"
SAFE = "execution_eligible=false; geometry_admission=unqualified"
STARTING = "starting; source_admitted=false; waiting_for_stable_source=true; " + SAFE
READY = "experimental; backend=vulkan; alignment=raw_vulkan; " + SAFE


def status_fixture():
    """Execute the production callback without importing the ROS node runtime.

    Only the external process CPU reader and monotonic clock are controlled;
    the actual classifier, field parser and callback implementation run intact.
    """
    tree = ast.parse(COLLECTOR.read_text())
    fields = next(node for node in tree.body
                  if isinstance(node, ast.FunctionDef) and node.name == "fields")
    collector = next(node for node in tree.body
                     if isinstance(node, ast.ClassDef) and node.name == "Collector")
    callback = next(node for node in collector.body
                    if isinstance(node, ast.FunctionDef) and node.name == "on_status")
    clock = iter(range(1, 100))
    cpu_reads = []

    def cpu_ticks(pid):
        cpu_reads.append(pid)
        return 77

    namespace = {"status_ready": status_ready,
                 "time": SimpleNamespace(monotonic=lambda: next(clock)),
                 "cpu_ticks": cpu_ticks}
    selected = ast.Module(body=[fields, callback], type_ignores=[])
    exec(compile(selected, str(COLLECTOR), "exec"), namespace)
    state = SimpleNamespace(args=SimpleNamespace(mode="vulkan", container_pid=12345),
                            status=[], errors=[], begin=None, cpu0=None,
                            final_status=None)
    return state, MethodType(namespace["on_status"], state), cpu_reads


def test_starting_cannot_sample_cpu_or_begin_steady_window():
    """Dropping the false-readiness return would include startup in CPU timing."""
    state, callback, cpu_reads = status_fixture()
    callback(SimpleNamespace(data=STARTING))
    assert not state.errors and len(state.status) == 1
    assert state.begin is None and state.cpu0 is None and cpu_reads == []
    callback(SimpleNamespace(data=READY))
    assert state.begin == 3 and state.cpu0 == 77 and cpu_reads == [12345]
    callback(SimpleNamespace(data=READY))
    assert state.begin == 3 and state.cpu0 == 77 and cpu_reads == [12345]
    callback(SimpleNamespace(data=STARTING))
    assert state.errors and state.begin == 3 and cpu_reads == [12345]


def test_invalid_ready_status_cannot_enter_cpu_measurement_window():
    """Failing the classifier must be terminal diagnostic evidence, never ready."""
    for text in ("unknown; backend=vulkan; alignment=raw_vulkan; " + SAFE,
                 "experimental; backend=cpu; alignment=vendor_cpu; " + SAFE,
                 READY.replace("execution_eligible=false", "execution_eligible=true"),
                 STARTING + "; restart_required=true"):
        state, callback, cpu_reads = status_fixture()
        callback(SimpleNamespace(data=text))
        assert state.errors and state.begin is None and state.cpu0 is None
        assert cpu_reads == []


def test_status_classifier_cannot_acquire_runtime_or_io_dependencies():
    """A blocking/logging/ROS/process call added here would perturb qualification."""
    tree = ast.parse(HELPER.read_text())
    # This helper's contract is string/dict classification only. No imports,
    # dynamic imports, filesystem, network, sleep, process or ROS APIs are needed.
    assert not any(isinstance(node, (ast.Import, ast.ImportFrom)) for node in ast.walk(tree))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Name):
            assert node.func.id == "ValueError", ast.unparse(node.func)
        else:
            assert isinstance(node.func, ast.Attribute), ast.unparse(node.func)
            assert node.func.attr in {"strip", "split", "get"}, ast.unparse(node.func)
