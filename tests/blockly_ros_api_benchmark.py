"""Offline mocked ROS API dispatch benchmark; no ROS, sockets, hardware or files.

Run: python tests/blockly_ros_api_benchmark.py [--iterations 10000]
Batch means describe Python dispatch cost, not DDS or hardware latency.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import sys
import threading
import timeit
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.run_blockly_station import strict_json
from workshop.blockly_ros_api import MAX_JSON_BYTES, ROSProgramAPI


def benchmark(iterations=10000):
    api = ROSProgramAPI.__new__(ROSProgramAPI)
    api.closed = False; api.strict_json = strict_json
    api.slots = threading.BoundedSemaphore(2)
    api.state_lock = threading.Lock(); api.active = 0; api.closing = threading.Event()
    api.app = SimpleNamespace(status=lambda: {'physical': False, 'job': {'state': 'idle'}},
                              stop=lambda: {'stopped': True})
    response = SimpleNamespace()
    request = SimpleNamespace(operation='status', arguments_json='{}')
    maximum_body = '{}' + ' ' * (MAX_JSON_BYTES - 2)
    cases = {
        'dispatch_status': lambda: api.dispatch('status', '{}'),
        'service_status': lambda: api.command(request, response),
        'dispatch_stop': lambda: api.dispatch('stop', '{}'),
        'dispatch_status_max_request': lambda: api.dispatch('status', maximum_body),
    }
    results = {}
    for name, function in cases.items():
        samples = [seconds / iterations * 1e6 for seconds in timeit.repeat(function, number=iterations, repeat=7)]
        results[name] = {'median_us_per_call': round(statistics.median(samples), 3),
                         'max_batch_mean_us_per_call': round(max(samples), 3)}
    return {'python': platform.python_version(), 'platform': platform.platform(),
            'api_sha256': hashlib.sha256((ROOT / 'workshop/blockly_ros_api.py').read_bytes()).hexdigest(),
            'iterations_per_batch': iterations, 'batches': 7, 'results': results,
            'scope': 'Mocked Application; Python dispatch only. No ROS/DDS/network/hardware startup.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=10000)
    args = parser.parse_args()
    if args.iterations < 1: parser.error('--iterations must be positive')
    print(json.dumps(benchmark(args.iterations), indent=2))
