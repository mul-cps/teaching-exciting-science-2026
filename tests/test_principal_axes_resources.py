"""Fail-closed resource guards for the small principal-axis policy helper."""
import ast
import hashlib
import inspect
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin import principal_axes


def _name(node):
    return ast.unparse(node)


def _guard(source):
    tree = ast.parse(source)
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
    assert len(functions) == 1
    function = functions[0]
    assert not function.args.defaults and not any(function.args.kw_defaults)
    assert all(isinstance(n, (ast.Import, ast.FunctionDef, ast.Expr)) for n in tree.body)
    assert all(n.names[0].name == 'numpy' for n in tree.body if isinstance(n, ast.Import))
    assert not any(isinstance(n, (ast.Global, ast.Nonlocal, ast.ListComp, ast.SetComp,
                                  ast.DictComp, ast.GeneratorExp, ast.AsyncFor)) for n in ast.walk(tree))
    # The rejecting shape/cap check must precede all loops and allocations.
    cap = next(n for n in function.body if isinstance(n, ast.If))
    assert any(isinstance(n, ast.Compare) and _name(n) == 'dimension not in (2, 3)'
               for n in ast.walk(cap.test))
    assert len(cap.body) == 1 and isinstance(cap.body[0], ast.Raise)
    cap_position = function.body.index(cap)
    assert all(not isinstance(n, (ast.For, ast.While)) for statement in function.body[:cap_position]
               for n in ast.walk(statement))
    calls = {'np.asarray', 'len', 'np.isfinite', 'np.isfinite(values).all',
             'np.isfinite(rows).all', 'np.any', 'isinstance', 'ValueError',
             'np.finfo', 'max', 'float', 'np.max', 'np.abs', 'rows.copy',
             'np.zeros', 'np.eye', 'np.outer', 'range', 'projector.copy',
             'np.sum', 'int', 'np.flatnonzero', 'np.sqrt'}
    for node in ast.walk(function):
        if isinstance(node, ast.Call):
            name = _name(node.func)
            assert name in calls, f'unsupported work: {name}'
            if name == 'np.asarray':
                assert _name(node.args[0]) in {'spectrum', 'axes'}
            if name in {'np.zeros', 'np.eye'}:
                assert _name(node.args[0]) == 'dimension', 'unbounded allocation'
                assert function.body.index(next(s for s in function.body if node in ast.walk(s))) > cap_position
        if isinstance(node, ast.For):
            assert isinstance(node.iter, ast.Call) and _name(node.iter.func) == 'range'
            assert all(isinstance(arg, ast.Name) and arg.id in {'dimension', 'start', 'end', 'index'}
                       or isinstance(arg, ast.Constant) and type(arg.value) is int and 0 <= arg.value <= 3
                       for arg in node.iter.args), 'unbounded range'
        if isinstance(node, ast.While):
            test = node.test.values[0] if isinstance(node.test, ast.BoolOp) else node.test
            assert _name(test) in {'start < dimension', 'end < dimension'}
            variable = test.left.id
            writes = [n for n in ast.walk(node) if isinstance(n, (ast.Assign, ast.AugAssign))
                      and any(isinstance(t, ast.Name) and t.id == variable
                              for t in (n.targets if isinstance(n, ast.Assign) else [n.target]))]
            if variable == 'end':
                assert len(writes) == 1 and _name(writes[0]) == 'end += 1'
            else:
                assert len(writes) == 1 and _name(writes[0]) == 'start = end'
                assert _name(node.body[0]) == 'end = start + 1'
    # Population is permitted solely in scalar input validation and tie tolerance.
    for statement in function.body:
        if any(isinstance(n, ast.Name) and n.id == 'sample_count' for n in ast.walk(statement)):
            assert statement is cap or (isinstance(statement, ast.Assign)
                                       and _name(statement.targets[0]) == 'tolerance')
    # No mutable containers/cache and no extra assignments to bounded indices.
    assert not any(isinstance(n, (ast.List, ast.Dict, ast.Set)) for n in ast.walk(function))
    writes = {key: [] for key in ('dimension', 'start', 'end', 'index')}
    for node in ast.walk(function):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in writes:
                    writes[target.id].append(_name(node))
        elif isinstance(node, (ast.AugAssign, ast.NamedExpr)):
            if isinstance(node.target, ast.Name) and node.target.id in writes:
                writes[node.target.id].append(_name(node))
    assert writes == {'dimension': ['dimension = len(values)'],
                      'start': ['start = 0', 'start = end'],
                      'end': ['end = start + 1', 'end += 1'], 'index': []}


def test_helper_resource_contract():
    _guard(inspect.getsource(principal_axes))


@pytest.mark.parametrize('old,new', [
    ('dimension not in (2, 3)', 'False'),
    ('range(start, end)', 'range(sample_count)'),
    ('np.zeros(dimension, dtype=bool)', 'np.zeros(sample_count, dtype=bool)'),
    ('result = rows.copy()', 'result = np.linalg.svd(rows)'),
    ('result = rows.copy()', 'dimension += 1000000\n    result = rows.copy()'),
    ('result = rows.copy()', 'if (dimension := 1000000):\n        pass\n    result = rows.copy()'),
])
def test_guard_rejects_forbidden_mutations(old, new):
    source = inspect.getsource(principal_axes)
    assert old in source
    with pytest.raises((AssertionError, StopIteration)):
        _guard(source.replace(old, new, 1))


@pytest.mark.parametrize('dimension', [4, 5, 16])
def test_invalid_dimensions_rejected(dimension):
    with pytest.raises(ValueError):
        principal_axes.canonical_principal_axes(np.ones(dimension), np.eye(dimension), sample_count=1)


def test_large_population_only_affects_scalar_tolerance(monkeypatch):
    observed = []
    original_range = range
    def bounded_range(*args):
        assert all(0 <= value <= 3 for value in args)
        observed.append(args)
        return original_range(*args)
    monkeypatch.setattr(principal_axes, 'range', bounded_range, raising=False)
    spectrum = np.array([3., 2., 1.])
    axes = np.eye(3)
    _, small = principal_axes.canonical_principal_axes(spectrum, axes, sample_count=1)
    _, large = principal_axes.canonical_principal_axes(spectrum, axes, sample_count=10**16)
    assert not small.any() and large.all()
    assert all(max(args) <= 3 for args in observed)


def test_report_helper_hash():
    source = Path(principal_axes.__file__).read_bytes()
    print('principal_axes_sha256=' + hashlib.sha256(source).hexdigest())
