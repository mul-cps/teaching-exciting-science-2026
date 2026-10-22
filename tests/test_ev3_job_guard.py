"""Production reachability and bounded cold-path fences for EV3 completion."""
import ast
import hashlib
import json
from pathlib import Path

from ev3_job_benchmark import benchmark

ROOT = Path(__file__).resolve().parents[1]


def tree(path):
    return ast.parse((ROOT / path).read_text())


def digest(node):
    def fields(value):
        if isinstance(value, ast.AST):
            return [type(value).__name__, {key: fields(item) for key, item in ast.iter_fields(value)
                                          if item is not None and item != []}]
        if isinstance(value, list):
            return [fields(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(fields(node), sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def test_actual_main_delegates_to_helper_without_other_workflow_changes():
    main = next(node for node in tree('tools/ros_station_api.py').body
                if isinstance(node, ast.FunctionDef) and node.name == 'main')
    simple = next(node for node in main.body
                  if isinstance(node, ast.FunctionDef) and node.name == 'simple_ev3')
    work = next(node for node in simple.body
                if isinstance(node, ast.FunctionDef) and node.name == 'work')
    expected = ast.parse('''
from workshop.station.ev3_job import run_ev3_operation
return run_ev3_operation(proxy, cancel, operation, record, stop_state)
''').body
    assert [ast.dump(node, include_attributes=False) for node in work.body] == [
        ast.dump(node, include_attributes=False) for node in expected]
    assert ast.unparse(simple.body[-1]) == 'return worker_task(operation, work)'
    # Frozen ce2362c main with only simple_ev3.work removed: worker_task,
    # tickets, STOP correlation/ACKs, faults/reset, references and all other
    # admission/cleanup remain byte-for-byte equivalent at the AST level.
    work.body = [ast.Pass()]
    assert digest(main) == '1ff7b032fa4ecb47890b8f946c6e169aa97fd3a22047cdb8f13dd9752ae8c1a0'


def test_pre_rpc_preemption_and_post_rpc_fault_precedence_are_ordered():
    run = next(node for node in tree('workshop/station/ev3_job.py').body
               if isinstance(node, ast.FunctionDef) and node.name == 'run_ev3_operation')
    loop, = [node for node in ast.walk(run) if isinstance(node, ast.While)]
    early, status, classify, late, complete, sleep = loop.body
    assert isinstance(early, ast.If) and isinstance(late, ast.If)
    assert ast.unparse(early.test) == ast.unparse(late.test) == 'cancel.is_set() or clock() >= deadline'
    assert ast.unparse(status) == "latest = proxy.call('status')"
    assert ast.unparse(classify) == 'complete = _terminal(latest, session, job_id)'
    expected = ast.parse('''
stopped = proxy.call('stop')
if not isinstance(stopped, dict) or stopped.get('stopped') is not True:
    raise RuntimeError('EV3 stop acknowledgement missing; keep stopped and inspect brakes')
''').body
    assert [ast.dump(node, include_attributes=False) for node in early.body[:2]] == [
        ast.dump(node, include_attributes=False) for node in expected]
    assert all(isinstance(node, ast.Raise) or isinstance(node, ast.If) for node in early.body[2:])
    assert isinstance(late.body[0], ast.If) and ast.unparse(late.body[0].test) == 'not complete'
    assert [ast.dump(node, include_attributes=False) for node in late.body[0].body] == [
        ast.dump(node, include_attributes=False) for node in expected]
    assert ast.unparse(sleep) == 'sleep(0.05)'
    # No repeated evidence construction or serialization on active polling.
    calls = [node for node in ast.walk(loop) if isinstance(node, ast.Call)
             and ast.unparse(node.func) == 'record']
    assert len(calls) == 1 and calls[0] in list(ast.walk(complete))
    assert ast.unparse(complete.test) == 'complete'
    assert ast.unparse(calls[0]) == "record(evidence('ev3_job_terminal'))"


def test_original_eject_readiness_and_bounded_fault_projection_remain():
    module = tree('workshop/station/ev3_job.py')
    run = next(node for node in module.body
               if isinstance(node, ast.FunctionDef) and node.name == 'run_ev3_operation')
    readiness, = [node for node in ast.walk(run) if isinstance(node, ast.Call)
                  and ast.unparse(node.func) == 'require_fresh_stationary']
    assert ast.unparse(readiness) == ('require_fresh_stationary(proxy, session=session, '
                                    'deadline=clock() + 2.0, cancel=cancel, '
                                    'spin=lambda deadline, cancel: sleep(0.05))')
    dispatch = next(node for node in ast.walk(run) if isinstance(node, ast.Call)
                    and ast.unparse(node) == 'proxy.call(operation)')
    assert readiness.lineno < dispatch.lineno
    fault = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == '_fault')
    returned = next(node for node in fault.body if isinstance(node, ast.Return) and isinstance(node.value, ast.Dict))
    assert [ast.unparse(value) for value in returned.value.values] == ["value['code'][:80]", "value['message'][:240]"]
    terminal = next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == '_terminal')
    assert not any(isinstance(node, ast.Attribute) and node.attr in {'call', 'record', 'write', 'dump'}
                   for node in ast.walk(terminal))
    assert not any(isinstance(node, ast.Call) and ast.unparse(node.func) in {'eval', 'exec', 'compile', '__import__'}
                   for node in ast.walk(module))


def test_cpu_benchmark_exercises_actual_helper_and_only_two_cold_events():
    result = benchmark(iterations=5, batches=3)
    assert result['evidence_events_per_100_running_polls'] == 2
    assert len(result['evidence_event_json_bytes']) == 2
    assert all(size < 2000 for size in result['evidence_event_json_bytes'])
    assert result['results']['current_running_classifier']['median_batch_us'] >= 0
    assert result['results']['current_100_polls_and_completion_two_events']['median_batch_us'] >= 0
