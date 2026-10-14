"""Static vectorization guard and hardware-free benchmark sensitivity checks."""
import ast
import inspect
from pathlib import Path
import textwrap

import pytest
import numpy as np

pytest.importorskip('scipy')

from workshop.digital_twin.scene import unproject


def projection_violations(source):
    tree = ast.parse(textwrap.dedent(source))
    violations = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.For, ast.While, ast.ListComp, ast.SetComp, ast.DictComp)):
            violations.append(type(node).__name__)
        if isinstance(node, ast.GeneratorExp):
            # Scalar validation over (fx, fy) or ('ppx', 'ppy') is deliberately
            # bounded; a generator over pixels/shape/range must fail review.
            if any(not isinstance(g.iter, ast.Tuple) or len(g.iter.elts) > 4
                   for g in node.generators):
                violations.append('unbounded generator')
        if isinstance(node, ast.Call) and ast.unparse(node.func) in (
                'np.vectorize', 'np.frompyfunc', 'numpy.vectorize', 'numpy.frompyfunc'):
            violations.append('Python scalar callback')
    return violations


def test_deprojection_keeps_array_operations_without_per_pixel_python_loops():
    assert not projection_violations(inspect.getsource(unproject))


@pytest.mark.parametrize('operation', [
    'for row in depth:\n        pass',
    'while pixel < size:\n        pixel += 1',
    'result = [project(pixel) for pixel in depth.flat]',
    'result = sum(project(pixel) for pixel in depth.flat)',
    'result = np.vectorize(project)(depth)',
    'result = np.frompyfunc(project, 1, 1)(depth)',
])
def test_projection_guard_detects_scalar_pixel_operations(operation):
    assert projection_violations('def project(depth):\n    ' + operation)


def test_offline_twin_does_not_import_hardware_control_interfaces():
    forbidden = ('serial', 'usb', 'pyrealsense2', 'scservo_sdk', 'lerobot',
                 'workshop.station', 'workshop.servo_start', 'subprocess')
    for path in Path(inspect.getfile(unproject)).parent.rglob('*.py'):
        source = path.read_text()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            names = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                     else [node.module or ''] if isinstance(node, ast.ImportFrom) else [])
            if isinstance(node, ast.Call) and node.args:
                # Catch importlib.import_module(...) and __import__(...) as well.
                func = ast.unparse(node.func)
                if func in ('__import__', 'importlib.import_module'):
                    arg = node.args[0]
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        names.append(arg.value)
            if path.name == 'ik.py' and isinstance(node, ast.ImportFrom):
                # This exact LeRobot module supplies the offline RobotKinematics
                # math model; robot and motor interfaces remain forbidden.
                if node.module == 'lerobot.model.kinematics' and [a.name for a in node.names] == ['RobotKinematics']:
                    continue
            assert not any(name == item or name.startswith(item + '.')
                           for name in names for item in forbidden), path


@pytest.mark.parametrize('source, allowed', [
    ('from lerobot.model.kinematics import RobotKinematics', True),
    ('from lerobot.robots.so_follower import SOFollower', False),
    ('from lerobot.motors import Motor', False),
    ("importlib.import_module('lerobot.robots.so_follower')", False),
    ("__import__('serial')", False),
])
def test_offline_import_guard_only_allows_the_kinematics_math_import(source, allowed):
    tree = ast.parse(source)
    names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            names.append(node.module or '')
        elif isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.Call) and node.args and ast.unparse(node.func) in (
                '__import__', 'importlib.import_module'):
            if isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
                names.append(node.args[0].value)
    forbidden = ('serial', 'usb', 'pyrealsense2', 'scservo_sdk', 'lerobot',
                 'workshop.station', 'workshop.servo_start', 'subprocess')
    exact_kinematics_import = (
        len(tree.body) == 1
        and isinstance(tree.body[0], ast.ImportFrom)
        and tree.body[0].module == 'lerobot.model.kinematics'
        and [alias.name for alias in tree.body[0].names] == ['RobotKinematics']
    )
    if allowed:
        assert exact_kinematics_import
        names = []
    is_forbidden = any(name == item or name.startswith(item + '.')
                       for name in names for item in forbidden)
    assert is_forbidden is (not allowed)


def test_cpu_benchmark_exercises_full_frame_projection_and_actual_virtual_pose():
    pytest.importorskip('mujoco')
    from tools.benchmark_digital_twin import benchmark
    result = benchmark(repetitions=1, fk_repetitions=2)
    assert result['image_shape'] == [480, 640]
    assert result['valid_pixels'] == 480 * 640
    assert result['table_samples'] == 17334
    assert result['geometries'] > 0
    assert result['table_evidence']['inlier_fraction'] == 1
    for stage in ('projection', 'table_fit', 'world_transform', 'virtual_set_pose'):
        assert 0 < result[stage]['median_ms'] <= result[stage]['p95_ms'] <= result[stage]['max_ms']


def test_pose_updates_do_not_reparse_assets_or_run_registration(monkeypatch):
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.scene import make_model
    from workshop.digital_twin import kinematics
    import xml.etree.ElementTree as ET
    model = mujoco.MjModel.from_xml_string(make_model([0, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0]))
    data = mujoco.MjData(model)
    pose = dict(shoulder_pan=-10, shoulder_lift=-88, elbow_flex=90,
                wrist_flex=73, wrist_roll=2, gripper=4)
    kinematics.set_pose(model, data, pose)  # Allow startup to initialize immutable resources.
    before = data.geom_xpos.copy()
    def forbidden(*args, **kwargs):
        pytest.fail('interactive FK must not re-read immutable assets or run a registration fit')
    monkeypatch.setattr(ET, 'parse', forbidden)
    from workshop.digital_twin import registration
    monkeypatch.setattr(registration, 'fit_registration', forbidden)
    monkeypatch.setattr(registration, 'fit_pose_series', forbidden)
    pose['shoulder_pan'] += 15
    kinematics.set_pose(model, data, pose)
    assert np.max(np.abs(data.geom_xpos-before)) > .01
    assert np.isfinite(data.qpos).all()


def test_cad_transform_and_patch_tracking_use_bulk_array_operations():
    from workshop.digital_twin.registration import transform_samples, white_arm_mask
    from workshop.digital_twin.tracking import track_patch
    for function in (transform_samples, white_arm_mask, track_patch):
        assert not projection_violations(inspect.getsource(function)), function.__name__


def test_viewer_callbacks_keep_heavy_fits_in_startup():
    pytest.importorskip('mujoco')
    pytest.importorskip('viser')
    pytest.importorskip('trimesh')
    from workshop.digital_twin import viewer
    tree = ast.parse(inspect.getsource(viewer))
    forbidden = {'fit_table', 'fit_registration', 'fit_pose_series', 'fit_tracked_tcp',
                 'fit_table_kinematics', 'load_pose_series', 'cad_samples', 'prepare'}
    callbacks = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                 and node.name in ('change', 'update_model', 'render_current')]
    assert len(callbacks) == 3
    for callback in callbacks:
        calls = {node.func.id for node in ast.walk(callback)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
        assert not calls & forbidden, callback.name


def test_registration_benchmark_is_reported_separately_from_pose_hotpath():
    pytest.importorskip('mujoco')
    from tools.benchmark_digital_twin import benchmark
    result = benchmark(repetitions=1, fk_repetitions=2, registration=True)
    assert result['registration']['status'].startswith('UNQUALIFIED')
    assert result['registration']['single_pose']['timing']['median_ms'] > 0
    assert result['registration']['single_pose']['timing']['samples'] == 1
    assert result['virtual_set_pose']['samples'] == 2
    assert result['registration']['data_kind'] == 'synthetic asymmetric surface'


def test_tracked_registration_rejects_unsynchronized_joint_frame_pairs(tmp_path):
    pytest.importorskip('mujoco')
    import json
    from workshop.digital_twin.tracking import fit_tracked_tcp
    (tmp_path / 'scene-calibration-recorded-result.json').write_text(json.dumps({
        'records': [{'label': 'home_initial', 'dwell_end_monotonic': 10.0}],
    }))
    folder = tmp_path / 'scene-calibration-recorded'
    folder.mkdir()
    (folder / 'timestamps.jsonl').write_text(json.dumps({'index': 0, 'monotonic': 11.0}) + '\n')
    with pytest.raises(ValueError, match='160ms'):
        fit_tracked_tcp(tmp_path, {}, tmp_path)


def test_virtual_gripper_uses_the_selected_compiled_model_basis():
    mujoco = pytest.importorskip('mujoco')
    from workshop.digital_twin.scene import make_model
    from workshop.digital_twin.kinematics import set_pose
    for convention in ('old', 'new'):
        model = mujoco.MjModel.from_xml_string(make_model(
            [0, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0], convention=convention))
        data = mujoco.MjData(model)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'gripper')
        # Change the compiled limits so a hardcoded other-model URDF cannot pass.
        model.jnt_range[joint] = [.1, .9]
        set_pose(model, data, {'gripper': 25})
        assert data.qpos[model.jnt_qposadr[joint]] == pytest.approx(.3)


@pytest.mark.parametrize('option,value', [('repetitions',0),('fk_repetitions',-1),
                                         ('repetitions',True),('repetitions',10001)])
def test_cpu_benchmark_rejects_empty_or_unbounded_sample_counts(option,value):
    from tools.benchmark_digital_twin import benchmark
    with pytest.raises(ValueError): benchmark(**{option:value})


@pytest.mark.parametrize('convention', ['old', 'new'])
def test_surface_registration_landmark_matches_selected_native_cad_basis(monkeypatch, convention):
    mujoco = pytest.importorskip('mujoco')
    from types import SimpleNamespace
    from workshop.digital_twin.scene import make_model
    from workshop.digital_twin.kinematics import set_pose
    from workshop.digital_twin import registration
    model = mujoco.MjModel.from_xml_string(make_model(
        [0, 0, 0], [-.2, 0, 0], [0, -.5, .3], [1, 0, 0, 0], convention=convention))
    data = mujoco.MjData(model)
    degrees = dict(shoulder_pan=-10, shoulder_lift=-88, elbow_flex=90,
                   wrist_flex=73, wrist_roll=2, gripper=4)
    set_pose(model, data, degrees)
    gripper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
    motor = next(index for index in range(model.ngeom)
                 if model.geom_bodyid[index] == gripper and model.geom_group[index] == 2
                 and 'sts3215' in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL,
                                                   int(model.geom_matid[index])) or ''))
    base_position = np.array([.15, .13, 0])
    surface = registration.cad_surface(model, data, count=3500)[::35] + base_position
    landmark = data.geom_xpos[motor] + base_position
    frames = [{'label': label, 'degrees': degrees, 'observed': surface,
               'landmark_world': landmark, 'image': 'synthetic-paired-frame.png',
               'timestamp_difference_s': 0.0}
              for label in ('home_initial', 'home_again', 'wrist_flex_minus')]
    residuals = []
    def evaluate_known_alignment(residual, initial, **kwargs):
        parameters = np.asarray(initial)
        residuals.append(residual(parameters))
        return SimpleNamespace(x=parameters, cost=0.0)
    # Evaluate the real native FK/residual at a known exact alignment, without
    # spending seconds optimizing or allowing a fit to hide a basis mismatch.
    monkeypatch.setattr(registration, 'least_squares', evaluate_known_alignment)
    registration.fit_pose_series(model, frames, initial_position=base_position, yaw_seeds=(0,))
    assert residuals
    assert max(np.max(np.abs(values)) for values in residuals) < 1e-8, (
        'surface and gripper landmark must agree in the selected native convention')
