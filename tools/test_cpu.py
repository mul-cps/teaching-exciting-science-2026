"""Explicit CPU/native test lanes; no test collection at module import."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('rclpy', 'torch', 'slangpy', 'pyrealsense2', 'scservo_sdk', 'lerobot', 'ev3dev2')
ENVIRONMENT = dict(ROS_DOMAIN_ID='177', ROS_LOCALHOST_ONLY='1',
                   PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1',
                   OMP_NUM_THREADS='1')
UNTRUSTED_ENV = ('PYTHONPATH', 'PYTEST_ADDOPTS', 'PYTEST_PLUGINS')


def partition(root=ROOT):
    value = json.loads((root / 'tests/native177.json').read_text())
    if set(value) != {'schema', 'native177', 'cpu_excludes', 'provenance'} or value['schema'] != 'so101-test-partition-v1':
        raise ValueError('invalid test partition schema')
    native, excluded = value['native177'], value['cpu_excludes']
    if not isinstance(native, list) or not isinstance(excluded, dict) or len(native) != len(set(native)):
        raise ValueError('invalid test partition lists')
    for name in native + list(excluded):
        if not isinstance(name, str) or Path(name).name != name or not name.startswith('test_') or not name.endswith('.py') or not (root / 'tests' / name).is_file():
            raise ValueError('invalid test module')
    if not set(excluded) <= set(native) or not all(isinstance(v, str) and v for v in excluded.values()):
        raise ValueError('invalid native dependency reasons')
    return native, excluded


def environment():
    env = os.environ.copy()
    env.update(ENVIRONMENT)
    for name in UNTRUSTED_ENV:
        env.pop(name, None)
    return env


def discovery():
    # Top-level spec lookup only: never import an SDK or enumerate devices.
    return {name: importlib.util.find_spec(name) is not None for name in PACKAGES}


def command(native, extra, root=ROOT):
    validate_extra(extra)
    modules, excluded = partition(root)
    if native:
        return [str(root / 'tools/test_perception_supervisor.sh'), '-p',
                'tools.test_cpu', '-q'] + ['tests/' + n for n in modules] + extra
    return [sys.executable, '-m', 'pytest', '-q', 'tests'] + [
        '--ignore=tests/' + n for n in excluded] + extra


def validate_extra(extra):
    """Output paths may vary; callers cannot narrow canonical test coverage."""
    seen = set()
    index = 0
    while index < len(extra):
        option, separator, value = extra[index].partition('=')
        if option not in ('--basetemp', '--junitxml') or option in seen:
            raise ValueError('only unique --basetemp/--junitxml output options are permitted')
        seen.add(option)
        if not separator:
            index += 1
            value = extra[index] if index < len(extra) else ''
        if not value or value.startswith('-'):
            raise ValueError('output path required')
        index += 1


def pytest_sessionfinish(session, exitstatus):
    """Native lane plugin: skipped mandatory coverage is a negative result."""
    if os.environ.get('SO101_REQUIRE_NATIVE_NO_SKIPS') == '1':
        reporter = session.config.pluginmanager.get_plugin('terminalreporter')
        if reporter is None or reporter.stats.get('skipped'):
            session.exitstatus = 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', action='store_true')
    parser.add_argument('--preflight', action='store_true')
    args, extra = parser.parse_known_args(argv)
    try:
        validate_extra(extra)
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2
    env = environment()
    os.environ.update(ENVIRONMENT)
    for name in UNTRUSTED_ENV:
        os.environ.pop(name, None)
    partition()
    found = discovery()
    print(json.dumps({'packages_discoverable': found, 'environment': ENVIRONMENT,
                      'python': sys.version, 'executable': sys.executable}), flush=True)
    if not args.native and any(found.values()):
        print('CPU lane blocked: optional ROS/device/GPU runtime discoverable; explicit guard review required', file=sys.stderr)
        return 2
    if args.preflight:
        return 0
    if args.native:
        env['SO101_REQUIRE_NATIVE_NO_SKIPS'] = '1'
    return subprocess.run(command(args.native, extra), cwd=ROOT, env=env).returncode


if __name__ == '__main__':
    raise SystemExit(main())
