"""Static guard for bounded lifecycle discovery and delegated verified shutdown."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def source(path, function):
    tree = ast.parse((ROOT / path).read_text())
    return next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == function)


def test_stop_delegates_verified_station_shutdown_before_gui_cleanup():
    stop = ast.unparse(source('tools/station_cli.py', 'stop'))
    assert stop.index("prepared.blockly('blockly-close')") < stop.index('run(command)')
    assert stop.index('run(command)') < stop.index('process.cleanup_session')
    assert "'stop', '--record', str(path)" in stop
    native = ast.unparse(source('tools/station_process.py', 'stop_units'))
    assert 'fresh_unit_stop(record)' in native
    assert 'reversed(unit_tools().ROLES)' in native
    assert 'unit_binding(row) != record[\'units\'][role]' in native


def test_gui_cleanup_requires_recorded_and_current_command_identity():
    guard = ast.unparse(source('tools/station_cli.py', 'gui_record'))
    assert "record.get('root') != str(ROOT)" in guard
    assert "record.get('command') != gui_command" in guard
    assert "process.same_process(record['process'])" in guard
    assert '/cmdline' in guard and 'os.fsencode(word)' in guard


def test_purge_discovery_remains_explicit_and_preserves_cold_configuration():
    discovery = ast.unparse(source('tools/station_cli.py', 'purge_targets'))
    assert "('tools', 'tests', 'workshop', 'ros2/src')" in discovery
    assert 'os.walk(base, followlinks=False)' in discovery
    assert 'boundary(path, ROOT)' in discovery and 'tracked(path)' in discovery
    assert "ROOT / 'ros2/.cache' / name" in discovery
    assert "('conda', 'realsense-cpu-texture-bounds', 'rclpy-publisher-gid-7.1.12', 'rmw-cyclonedds-gid-2.2.4')" in discovery
    purge = ast.unparse(source('tools/station_cli.py', 'purge'))
    assert purge.index('stop(args)') < purge.index('shutil.rmtree(path)')
    assert 'if purge_targets(args) != targets:' in purge
    module = (ROOT / 'tools/station_cli.py').read_text()
    assert 'shell=True' not in module
    assert 'pkill' not in module and 'killall' not in module
    assert 'teaching-exciting-science-2026-' not in module
