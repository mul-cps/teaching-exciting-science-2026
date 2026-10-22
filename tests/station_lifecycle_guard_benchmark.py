"""CPU-only lifecycle planning benchmark. No processes, ROS or devices invoked."""
import json
from pathlib import Path
import statistics
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import station_cli


def measure(operation, count):
    samples = []
    for _ in range(count):
        started = time.perf_counter_ns()
        operation()
        samples.append((time.perf_counter_ns() - started) / 1000)
    samples.sort()
    return {'count': count, 'p50_us': statistics.median(samples),
            'p95_us': samples[int(.95 * (len(samples) - 1))], 'max_us': max(samples)}


def benchmark():
    args = SimpleNamespace(profile='fwmini4', manifest=Path('/fixture/manifest.json'), uninstall=False, runtime=False)
    params = {key: '/fixture/' + key for key in ('station_config', 'station_python',
              'joint_limit_contract', 'state_file', 'station_lock', 'log_dir',
              'calibration_dir', 'camera_sdk_prefix')}
    params['use_ev3_tunnel'] = 'false'
    with patch.object(station_cli, 'manifest', return_value={'domain': 76, 'parameters': params}):
        command = measure(lambda: station_cli.lifecycle_command(args, 'start'), 2000)
    with tempfile.TemporaryDirectory(prefix='station-lifecycle-benchmark-') as directory:
        root = Path(directory)
        for scope in ('tools', 'tests', 'workshop', 'ros2/src'):
            for index in range(25):
                target = root / scope / str(index)
                target.mkdir(parents=True)
                if index % 5 == 0:
                    (target / '__pycache__').mkdir()
        with patch.object(station_cli, 'ROOT', root), patch.object(station_cli, 'tracked', return_value=False):
            discovery = measure(lambda: station_cli.purge_targets(args), 100)
    # Gross regression bound only; timings are synthetic host CPU costs.
    assert command['p95_us'] < 50000
    assert discovery['p95_us'] < 50000
    return {'scope': 'CPU-only manifest command construction and 100-directory/20-cache discovery; '
                     'excludes manifest IO/git tracking checks, subprocess, ROS, systemd, STOP, GUI and devices',
            'command': command, 'purge_discovery': discovery}


if __name__ == '__main__':
    print(json.dumps(benchmark(), indent=2))
