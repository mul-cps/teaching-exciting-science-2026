#!/usr/bin/env python3
"""Check retained camera evidence only; never launch any runtime/capture code."""
from pathlib import Path
import hashlib
import json
import math
import tarfile

HERE = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    inventory = json.loads((HERE/'inventory.json').read_text())
    for relative, pin in inventory.items():
        path = HERE/relative
        assert sha(path) == pin['sha256'] and path.stat().st_size == pin['bytes'], relative
    archive = HERE/'fwmini4-camera-live-native-20261027.tar.gz'
    assert sha(archive) == '325d00b3387ab8e4845363a22db69a26c6d32741a112195ef9935ee9f4539e33'
    rows = {r['path']: r for r in json.loads((HERE/'raw-archive-inventory.json').read_text())}
    with tarfile.open(archive) as tar:
        actual = {}
        data = {}
        for member in tar:
            assert not member.issym() and not member.islnk()
            assert not member.name.startswith('/') and '..' not in Path(member.name).parts
            if not member.isfile():
                continue
            content = tar.extractfile(member).read()
            assert not content.startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff'))
            actual[member.name] = dict(path=member.name, bytes=len(content),
                                       sha256=hashlib.sha256(content).hexdigest())
            data[member.name] = content
    assert rows == actual
    metrics = {}
    for group in ('live', 'live-system-sdk'):
        for mode in ('cpu', 'vulkan', 'auto'):
            name = group+'/'+mode
            runner = json.loads(data[name+'.runner.json'])
            result = json.loads(data[name+'.json'])
            assert result['execution_eligible'] is result['full_station_qualified'] is False
            if not result['passed']:
                assert runner['collector_exit'] == 1 and result['unique_measured_outputs'] == 0
                continue
            cpu = result['container_cpu_ms_proc_stat']
            seconds = result['steady_seconds']
            count = result['unique_measured_outputs']
            fraction = cpu/1000/seconds
            assert math.isclose(fraction, runner['container_fraction_of_one_core'], rel_tol=1e-12)
            assert math.isclose(cpu/count, result['container_cpu_ms_per_unique_output'], rel_tol=1e-12)
            assert math.isclose(count/seconds, result['output_hz'], rel_tol=1e-12)
            for key in ('source_frame', 'depth_source_frame'):
                ids = [int(m['fields'][key]) for m in result['measurements']]
                assert len(ids) == count and all(b == a+1 for a,b in zip(ids, ids[1:]))
            metrics[name] = dict(one_core_percent=100*fraction, cpu_ms_per_output=cpu/count,
                                 output_hz=count/seconds)
    fallback = json.loads(data['live-system-sdk/auto.runner.json'])['summary']['last_status_fields']
    assert fallback['backend'] == 'cpu'
    assert fallback['fallback_reason'] == 'loaded SDK is not the qualified scalar alignment convention'
    for filename in ('fwmini4-camera-sdk-640x480-20261027.json', 'fwmini4-camera-sdk-highest-20261027.json'):
        sdk = json.loads((HERE/'sdk'/filename).read_text())
        assert math.isclose(sdk['unique_pairs']/sdk['elapsed_s'], sdk['unique_pairs_per_s'], rel_tol=1e-12)
        assert math.isclose(1000*sdk['process_cpu_s']/sdk['unique_pairs'], sdk['cpu_ms_per_unique_pair'], rel_tol=1e-12)
    final = json.loads((HERE/'camera-live-final-state.json').read_text())
    assert final['camera_fuser_exit'] == 1 and not final['camera_fuser_stdout'] and not final['camera_fuser_stderr']
    assert not final['resident_test_container_pids']
    assert 'MainPID=181501' in final['owner_unit'] and 'ActiveState=active' in final['owner_unit']
    print(json.dumps(dict(passed=True, runtime_executed=False, files_hashed=len(inventory),
                         raw_members=len(rows), live_metrics=metrics), indent=2))


if __name__ == '__main__':
    main()
