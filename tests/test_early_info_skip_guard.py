"""Local skip-path/static cost fences; no ROS initialization or live calls."""
import ast
from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace as NS

import pytest

from early_info_skip_benchmark import SOURCE, adapter_class, adapter_fixture, snapshot


def digest(node):
    # ast.dump's empty-field formatting changed in Python 3.13.
    def fields(value):
        if isinstance(value, ast.AST):
            return [type(value).__name__, {key: fields(item) for key, item in ast.iter_fields(value)
                                           if item is not None and item != []}]
        if isinstance(value, list):
            return [fields(item) for item in value]
        return value
    return hashlib.sha256(json.dumps(fields(node), sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def source_class():
    return next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n, ast.ClassDef))


def test_skip_is_scalar_only_returns_before_admission_and_preserves_baseline_closure():
    cls = source_class()
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    intake = methods['_intake']
    info = next(n for n in ast.walk(intake) if isinstance(n, ast.If)
                and ast.unparse(n.test) == "kind in ('depth_info', 'color_info')")
    assert ast.unparse(info.body[0]) == 'self._validate_received_info(kind, message)'
    skip = info.body[1]
    assert ast.unparse(skip.test) == 'self.metadata_offer is not None'
    # Whitelist every call in the added hot branch. No copy/TF/sync/sleep,
    # queue, receive/take/drain, callback or progress work can hide here.
    assert sorted(ast.unparse(n.func) for n in ast.walk(skip) if isinstance(n, ast.Call)) == [
        'canonical_gid', 'min', 'type', 'type']
    shape = skip.body[1]
    assert ast.unparse(shape.test) == (
        'type(stamp.sec) is int and 0 <= stamp.sec <= 2 ** 31 - 1 and '
        '(type(stamp.nanosec) is int) and (0 <= stamp.nanosec < 1000000000)')
    assert ast.unparse(shape.body[0]) == 'header_ns = stamp.sec * 1000000000 + stamp.nanosec'
    assert not any(isinstance(n, (ast.For, ast.While, ast.comprehension, ast.Await,
                                  ast.Yield, ast.Import, ast.ImportFrom)) for n in ast.walk(skip))
    stores = [ast.unparse(n) for n in ast.walk(skip)
              if isinstance(n, (ast.Attribute, ast.Subscript)) and isinstance(n.ctx, ast.Store)]
    assert stores == ['self._early_info_drops[kind]', 'self._early_info_last_age_ns[kind]']
    decision = next(n for n in ast.walk(skip) if isinstance(n, ast.If)
                    and any(isinstance(x, ast.Return) for x in n.body))
    assert ast.unparse(decision.test) == (
        "header_ns > clocks['now_ns'] and canonical_gid(gid) == "
        "self.core.ledger.pin['source'][kind + '_publisher_gid']")
    assert isinstance(decision.body[-1], ast.Return) and decision.body[-1].value is None
    assert ast.unparse(decision.body[0]) == (
        'self._early_info_drops[kind] = min(2 ** 31 - 1, self._early_info_drops[kind] + 1)')
    assert ast.unparse(decision.body[1]) == (
        "self._early_info_last_age_ns[kind] = clocks['now_ns'] - header_ns")

    # Normalize ONLY the reviewed additions, then pin the complete adapter AST
    # to 053635c. This protects existing receipt/header/epoch/proof, timer,
    # legacy, invalidation and call ordering, including immutable Info checks.
    info.body.pop(1)
    reload = methods['reload']
    for block in ast.walk(reload):
        if isinstance(block, ast.With):
            block.body[:] = [n for n in block.body if not (
                isinstance(n, ast.Assign) and any(ast.unparse(t) in (
                    'self._early_info_drops', 'self._early_info_last_age_ns') for t in n.targets))]
    health = methods['_source_health']
    result = next(n for n in ast.walk(health) if isinstance(n, ast.Dict)
                  and any(isinstance(k, ast.Constant) and k.value == 'early_native_info_diagnostics' for k in n.keys))
    index = next(i for i, k in enumerate(result.keys) if isinstance(k, ast.Constant)
                 and k.value == 'early_native_info_diagnostics')
    diagnostics = result.values[index]
    assert {ast.unparse(n.func) for n in ast.walk(diagnostics) if isinstance(n, ast.Call)} == {
        'getattr', "getattr(self, '_early_info_drops', {}).get",
        "getattr(self, '_early_info_last_age_ns', {}).get"}
    assert not any(isinstance(n, (ast.Attribute, ast.Subscript)) and isinstance(n.ctx, ast.Store)
                   for n in ast.walk(diagnostics))
    assert len([n for n in ast.walk(diagnostics) if isinstance(n, ast.comprehension)]) == 1
    iterator = next(n.iter for n in ast.walk(diagnostics) if isinstance(n, ast.comprehension))
    assert ast.literal_eval(iterator) == ('depth_info', 'color_info')
    result.keys.pop(index); result.values.pop(index)
    assert digest(cls) == '2745efa6641c672f44d82ad675ae0f016b1524c761abfbc8b3105c6d77165b8c'


@pytest.mark.parametrize('filename,expected', [
    ('aligned_cohort.py', '6940f684518e2681cd2204df0b601294ca745b9986f7f820d9c5c56afe8a93b1'),
    ('admission_proof.py', 'e975b5d87fd9ad5bf70e7af71233c4c18e0d1b42ec8b6aeb4abdc1f8c082f95f'),
    ('derived_lineage.py', '33169b73f756bb21edb043307d3cc0d879694222e51faf630b00602cf496ba69')])
def test_transitive_admission_and_lease_implementation_unchanged(filename, expected):
    assert hashlib.sha256(SOURCE.with_name(filename).read_bytes()).hexdigest() == expected


def test_actual_skip_and_cold_health_do_not_admit_refresh_or_advance():
    obj, kind, message, gid, offered = adapter_fixture(adapter_class(SOURCE.read_text()), 'future_info')
    before = snapshot(obj)
    for _ in range(3):
        obj._intake(kind, message, gid)
        assert snapshot(obj) == before and offered == []
    assert obj._early_info_drops == {'depth_info': 3, 'color_info': 0}
    assert obj._early_info_last_age_ns == {'depth_info': -5_000_000, 'color_info': None}
    # Source-binding checks retain their existing contradiction invalidation;
    # isolate the new serialization from those checks, without weakening them.
    records = {k: rows[0] for k, rows in obj.core.streams.items() if k.endswith('_info')}
    obj._check_source_binding = lambda *args: records
    diagnostics = deepcopy((obj._early_info_drops, obj._early_info_last_age_ns))
    for ready in (True, False):
        if not ready:
            obj._check_source_binding = lambda *args: (_ for _ in ()).throw(ValueError('fixture unavailable'))
        result = obj._source_health(None, NS())
        payload = json.loads(result.message)
        assert result.success is ready
        assert snapshot(obj) == before and offered == []
        assert (obj._early_info_drops, obj._early_info_last_age_ns) == diagnostics
        diag = payload['early_native_info_diagnostics']
        assert diag['consumer_epoch'] == obj.consumer_epoch
        assert diag['input_admitted'] is diag['processing_progress'] is diag['qualified'] is False
        diag['streams']['depth_info']['discarded_unadmitted_count'] = 99
        assert obj._early_info_drops['depth_info'] == 3
