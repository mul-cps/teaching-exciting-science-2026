"""Combined native ingress/fence guards and forbidden-regression controls."""
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/performance'), str(ROOT / 'ros2/src/so101_station_objects')]
SPEC = importlib.util.spec_from_file_location('native_resource_probe', ROOT / 'tools/performance/benchmark_native_complete_report.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def test_frozen_combined_resource_contract():
    assert probe.NATIVE_GLUE.keys() == {'offer_native_complete', '_native_current', '_consume_native_complete'}
    assert probe.combined_guard(probe.SOURCE.read_text(), probe.INGRESS.read_text())


@pytest.mark.parametrize('old,new', [
    ('ingress = self.native_ingress\n', 'ingress = self.native_ingress\n                np.asarray(packet.report)\n'),
    ('ingress = self.native_ingress\n', 'ingress = self.native_ingress\n                device.readback()\n'),
    ('ingress = self.native_ingress\n', 'ingress = self.native_ingress\n                self.pending.append(packet)\n'),
    ('admitted = self.native_ingress.take()', 'self._analyze_job({})\n                admitted = self.native_ingress.take()'),
    ('admitted = self.native_ingress.take()', 'device.wait()\n                admitted = self.native_ingress.take()'),
    ('self.create_guard_condition(self._consume_native_complete)', 'self.create_guard_condition(self._consume_native_complete, callback_group=other)'),
    ('self._native_provider, self.native_ready_guard.trigger)', 'self._native_provider, self._consume_native_complete)'),
    ('if not accepting or invoked or get_ident() != owner:', 'if not accepting or invoked:'),
    ('if not accepting or invoked or get_ident() != owner:', 'if invoked or get_ident() != owner:'),
    ('accepting = False', 'accepting = True'),
    ('grasp = dict(grasp)', 'grasp = dict(obj)'),
    ('grasp["closing_axis"][:3].copy()', 'grasp["closing_axis"]'),
    ('grasp["approach_direction"][:3].copy()', 'grasp["approach_direction"]'),
    ('grasp["closing_axis"][:3].copy()', 'copy.deepcopy(grasp["closing_axis"])'),
    ('self._native_provider.publish_if_current(native_admission, publish_action)', 'self._native_provider.check(native_admission)'),
])
def test_native_glue_rejects_forbidden_mutants(old, new):
    source = probe.SOURCE.read_text()
    assert source.count(old) == 1
    with pytest.raises(AssertionError):
        probe.combined_guard(source.replace(old, new), probe.INGRESS.read_text())


@pytest.mark.parametrize('old,new', [
    ('owned = np.empty(shape, dtype=np.float64)', 'owned = np.asarray(value).copy()'),
    ('keys = {key for key, _ in entries}', 'entries = [(key, np.asarray(item)) for key, item in entries]\n    keys = {key for key, _ in entries}'),
    ('self._pending = admitted', 'self._pending.append(admitted)'),
    ('self._notify()', 'self._provider.check(admission)\n                self._notify()'),
    ('with self._lock:\n                generation, pending', 'with self._lock:\n                self._provider.check(self._pending.admission)\n                generation, pending'),
])
def test_approved_ingress_rejects_unreviewed_resource_mutants(old, new):
    ingress = probe.INGRESS.read_text()
    assert ingress.count(old) == 1
    with pytest.raises(AssertionError, match='approved ingress changed'):
        probe.combined_guard(probe.SOURCE.read_text(), ingress.replace(old, new))


def test_unknown_dense_keys_reject_before_custom_conversion():
    from test_native_complete_report import setup, retained, NativeCompleteReport
    class ForbiddenConversion:
        def __array__(self, *args):
            pytest.fail('dense/custom conversion before key admission')
        def __iter__(self):
            pytest.fail('untrusted dense iteration')
    for location in ('report', 'plane', 'object'):
        ident, _, ingress = setup()
        report = retained()
        target = {'report': report, 'plane': report['support_plane'], 'object': report['objects'][0]}[location]
        target['dense'] = ForbiddenConversion()
        assert not ingress.offer(NativeCompleteReport(ident, report))
        assert ingress.take() is None


def test_actual_publisher_grasp_call_accepts_owned_readonly_native_vectors():
    """Execute the actual publisher's call expression at its numerical boundary."""
    import ast
    from types import SimpleNamespace
    import numpy as np
    from so101_station_objects.objects import grasp_pose_rotation
    source = ast.parse(probe.SOURCE.read_text())
    publisher = next(n for n in ast.walk(source) if isinstance(n, ast.FunctionDef) and n.name == '_publish_report')
    loop = next(n for n in ast.walk(publisher) if isinstance(n, ast.For)
                and ast.unparse(n.target) == '(grasp_index, grasp)')
    stop = next(i for i,n in enumerate(loop.body) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'rotation_grasp' for t in n.targets))
    closing = np.array([1., 0., 0.]);approach = np.array([0., 0., 1.])
    closing.flags.writeable = False;approach.flags.writeable = False
    expression = compile(ast.Module(body=loop.body[:stop+1], type_ignores=[]), str(probe.SOURCE), 'exec')
    namespace = dict(grasp_pose_rotation=grasp_pose_rotation, np=np, native_admission=object(), self=SimpleNamespace(_native_mode=True),
                     grasp={'closing_axis': closing, 'approach_direction': approach})
    exec(expression, namespace)
    actual = namespace['rotation_grasp']
    expected = grasp_pose_rotation(closing.copy(), approach.copy())
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(closing, [1., 0., 0.])
    np.testing.assert_array_equal(approach, [0., 0., 1.])
    assert not closing.flags.writeable and not approach.flags.writeable


def test_native_grasp_materialization_matches_cpu_normalized_local_fields():
    """Near-unit admitted axes expose aliases used by later status/pregrasp code."""
    import ast
    from types import SimpleNamespace as NS
    import numpy as np
    from so101_station_objects.objects import grasp_pose_rotation, pregrasp_position
    tree = ast.parse(probe.SOURCE.read_text())
    publisher = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_publish_report')
    loop = next(n for n in ast.walk(publisher) if isinstance(n, ast.For)
                and ast.unparse(n.target) == '(grasp_index, grasp)')
    stop = next(i for i,n in enumerate(loop.body) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'rotation_grasp' for t in n.targets))
    boundary = compile(ast.Module(body=loop.body[:stop+1], type_ignores=[]), str(probe.SOURCE), 'exec')
    preposition = next(n for n in ast.walk(loop) if isinstance(n, ast.Assign)
                       and any(isinstance(t, ast.Name) and t.id == 'preposition' for t in n.targets))
    record = next(n for n in ast.walk(loop) if isinstance(n, ast.Expr)
                  and isinstance(n.value, ast.Call) and ast.unparse(n.value.func) == 'grasp_records.append')
    tail = compile(ast.Module(body=[preposition,record], type_ignores=[]), str(probe.SOURCE), 'exec')
    outputs = []
    for native in (False, True):
        closing = np.array([1.+2e-10, 0., 0.]);approach = np.array([0., 0., 1.+3e-10])
        assert abs(np.linalg.norm(closing)-1.) < 1e-9 and abs(np.linalg.norm(approach)-1.) < 1e-9
        if native:closing.flags.writeable = False;approach.flags.writeable = False
        grasp = dict(closing_axis=closing, approach_direction=approach, position=np.array([0., .1, .02]), opening_m=.03)
        obj = dict(track_id=1,color='red',pickup_eligible=True,eligibility_reason='inside_pickup_roi',
                   execution_eligibility_reason='not_ik_or_collision_checked')
        ns = dict(np=np, grasp_pose_rotation=grasp_pose_rotation, pregrasp_position=pregrasp_position,
                  native_admission=object() if native else None, grasp=grasp, obj=obj, grasp_index=0,
                  self=NS(_native_mode=native,board_frame='charuco_board'), det=NS(id='red-0001'), grasp_records=[])
        exec(boundary,ns);exec(tail,ns)
        outputs.append((ns['rotation_grasp'], ns['preposition'], ns['grasp_records']))
        if native:
            np.testing.assert_array_equal(closing,[1.+2e-10,0.,0.])
            np.testing.assert_array_equal(approach,[0.,0.,1.+3e-10])
            assert not closing.flags.writeable and not approach.flags.writeable
        else:
            # The original CPU helper side effects remain observable.
            np.testing.assert_array_equal(closing,[1.,0.,0.])
            np.testing.assert_array_equal(approach,[0.,0.,1.])
    np.testing.assert_array_equal(outputs[0][0],outputs[1][0])
    np.testing.assert_array_equal(outputs[0][1],outputs[1][1])
    assert outputs[0][2] == outputs[1][2]
