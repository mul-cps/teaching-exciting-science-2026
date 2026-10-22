"""Production/test domain separation, without ROS, devices or systemd calls."""
import ast
from pathlib import Path
from types import SimpleNamespace

from tools import install_station_user_units as units

ROOT = Path(__file__).resolve().parents[1]


def arguments(tmp_path, domain=None):
    return SimpleNamespace(
        profile='fwmini4', wrapper=None, wrapper_command=None, domain=domain,
        config=tmp_path/'station.json',
        python=Path.home()/'.local/share/exciting-science/station/venv/bin/python',
        contract=tmp_path/'limits.json', state_file=tmp_path/'state.json',
        station_lock=tmp_path/'lock', log_dir=tmp_path/'logs',
        calibration_dir=tmp_path/'calibration',
        camera_sdk=Path.home()/'miniforge3/envs/so101-jazzy',
        color_mask_backend='vulkan', existing_tunnel=False,
        bootstrap_owner=False, background=True,
    )


def functions(relative, names):
    tree = ast.parse((ROOT/relative).read_text())
    found = {node.name: node for node in tree.body
             if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names}
    assert set(found) == set(names)
    return found


def test_live_defaults_to_76_and_explicit_test_domain_stays_177(tmp_path):
    for requested, expected in ((None, 76), (76, 76), (177, 177)):
        manifest = units.build_manifest(arguments(tmp_path, requested))
        assert manifest['domain'] == expected
        units.validate_manifest(manifest)
        for role in units.ROLES:
            assert f'Environment="ROS_DOMAIN_ID={expected}" "ROS_LOCALHOST_ONLY=1"' in units.unit_text(manifest, role)
        assert manifest['parameters']['color_mask_backend'] == 'vulkan'
        assert manifest['parameters']['operator_native_mask_backend'] == 'vulkan'


def test_ev3_terminal_helper_is_bound_to_native_source_cohort():
    assert 'workshop/station/ev3_job.py' in units.SOURCES


def test_runtime_domain_paths_do_not_hardcode_fixture_domain():
    scopes = {
        'tools/station_process.py': ['prepare_unit_start', 'check_unit_record', 'parse_arguments'],
        'tools/prepare_workshop_pc.py': ['station_command', 'start_station', 'blockly'],
        'tools/install_station_user_units.py': ['unit_text'],
    }
    for path, names in scopes.items():
        for name, node in functions(path, names).items():
            literals = [part.value for part in ast.walk(node) if isinstance(part, ast.Constant)]
            assert '177' not in literals and 177 not in literals, (path, name)


def test_rviz_blockly_start_and_stop_use_validated_manifest_domain():
    scopes = functions('tools/prepare_workshop_pc.py', ['station_command', 'start_station', 'blockly'])
    for node in scopes.values():
        source = ast.unparse(node)
        assert "str(manifest['domain'])" in source
    start = ast.unparse(scopes['start_station'])
    assert "env=rviz_env" in start
    assert "'--domain', str(manifest['domain'])" in start
    blockly = ast.unparse(scopes['blockly'])
    assert "_, manifest = station_manifest()" in blockly
    assert "ROS_DOMAIN_ID=str(manifest['domain'])" in blockly
