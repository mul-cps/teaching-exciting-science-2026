#!/usr/bin/env python3
"""Read retained evidence only; never execute camera, ROS, compiler or GPU code."""
from pathlib import Path
from xml.etree import ElementTree as ET
import hashlib
import json
import math
import re
import tarfile

HERE = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    inventory = json.loads((HERE/'inventory.json').read_text())
    for relative, pin in inventory.items():
        path = HERE/relative
        assert path.stat().st_size == pin['bytes'] and sha(path) == pin['sha256'], relative
        assert not path.read_bytes().startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff')), relative
    archive = HERE/'fwmini4-highres-live-20261027.tar.gz'
    assert sha(archive) == 'bcb4b7b4f2b557ca619935673b29f77b31bf7fdad043634b6a792fd2d5023a08'
    wanted = {m['file']:m for m in json.loads((HERE/'raw-archive-inventory.json').read_text())}
    data, actual = {}, {}
    with tarfile.open(archive) as tar:
        for member in tar:
            assert not member.issym() and not member.islnk()
            assert not member.name.startswith('/') and '..' not in Path(member.name).parts
            if not member.isfile():
                continue
            content = tar.extractfile(member).read()
            assert not content.startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff'))
            data[member.name] = content
            actual[member.name] = dict(file=member.name, bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
    assert wanted == actual
    before = json.loads((HERE/'cpu/workshop-highres-cpu-final-20261027-source-pins-before.json').read_text())
    after = json.loads((HERE/'cpu/workshop-highres-cpu-final-20261027-source-pins-after.json').read_text())
    assert len(before) == 464 and before == after
    suite = ET.parse(HERE/'cpu/workshop-highres-cpu-final-20261027.xml').getroot()[0]
    assert suite.attrib['errors'] == suite.attrib['failures'] == '0' and suite.attrib['skipped'] == '133'
    assert '4333 passed, 133 skipped, 48 subtests passed in 206.10s' in (HERE/'cpu/workshop-highres-cpu-final-20261027.log').read_text()
    assert '19 errors during collection' in (HERE/'cpu/workshop-highres-cpu-20261027.log').read_text()
    assert '2 failed, 4331 passed' in (HERE/'cpu/workshop-highres-cpu-canonical-20261027.log').read_text()
    manifest = json.loads((HERE/'workshop-highres-source-manifest-20261027.json').read_text())
    check = json.loads((HERE/'commit-source-check.json').read_text())
    assert len(manifest['files']) == check['verified_files'] == 103 and check['all_match']
    assert manifest['git_revision'] == check['commit'] == '5344d665692ab2dddb0de32ef58c8adf6aedadb8'
    assert sha(HERE/'workshop-highres-source-manifest-20261027.json') == check['manifest_sha256']
    for host in ('intel','amd'):
        for profile in ('vga','720p','1080p'):
            text = data['alignment-'+profile+'.log'].decode() if host == 'intel' else (HERE/'amd'/('alignment-'+profile+'.log')).read_text()
            assert text.count('cpu_sdk_mismatch=0') == 8
            assert 'PASS cases=8 max_xyz_error_m=0 gpu=1 physical=false' in text
    shaders = HERE/'workshop-highres-alignment-shaders-20261027'
    proof = json.loads((shaders/'build-proof.json').read_text())
    assert (shaders/'source.sha256').read_text().strip() == proof['composite']
    for row in proof['commands']:
        output = Path(row['command'][-1]).name
        assert sha(shaders/output) == row['sha256']
        assert (shaders/(output+'.sha256')).read_text() == row['sha256']+'  '+output+'\n'
    metrics = {}
    for profile in ('720p','1080p'):
        for mode in ('cpu','vulkan','auto'):
            name = 'live/'+profile+'/'+mode
            runner = json.loads(data[name+'.runner.json'])
            observed = json.loads(data[name+'.json'])
            summary = runner['summary']
            assert runner['passed'] and runner['collector_exit'] == 0 and runner['profile_matches'] and runner['allocation_count_stable']
            assert summary['passed'] and not summary['execution_eligible'] and not summary['full_station_qualified']
            seconds, n, cpu = summary['steady_seconds'],summary['unique_measured_outputs'],summary['container_cpu_ms_proc_stat']
            assert seconds >= 15 and math.isclose(n/seconds, summary['output_hz'], rel_tol=1e-12)
            assert math.isclose(cpu/1000/seconds, runner['container_fraction_of_one_core'], rel_tol=1e-12)
            assert math.isclose(cpu/n, summary['container_cpu_ms_per_unique_output'], rel_tol=1e-12)
            assert summary['output_hz'] < 20
            for key in ('source_frame','depth_source_frame'):
                ids = [int(m['fields'][key]) for m in observed['measurements']]
                assert len(ids) == n and all(b>a for a,b in zip(ids,ids[1:]))
                assert sum(b-a-1 for a,b in zip(ids,ids[1:])) == runner['source_frame_gaps'][key]
            if mode != 'cpu':
                assert runner['steady_vulkan_allocation_counts'] == [35]
            metrics[profile+'/'+mode] = dict(output_hz=n/seconds, total_four_core_CPU_percent=25*cpu/1000/seconds,
                                           receipt_age_p95_ms=summary['collector_received_age_p95_ms'])
    system_archive = HERE/'fwmini4-highres-system-cpu-20261027.tar.gz'
    assert sha(system_archive) == '604b489246680602737d030d57bef9b3413e19a3101ff26f514cac1ab4397126'
    system_data, system_actual = {}, {}
    with tarfile.open(system_archive) as tar:
        for member in tar:
            assert not member.issym() and not member.islnk()
            assert not member.name.startswith('/') and '..' not in Path(member.name).parts
            if not member.isfile():
                continue
            content = tar.extractfile(member).read()
            assert not content.startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff'))
            system_data[member.name] = content
            system_actual[member.name] = dict(file=member.name, bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
    assert system_actual == {m['file']:m for m in json.loads((HERE/'system-raw-archive-inventory.json').read_text())}
    publication = json.loads((HERE.parent.parent/'fwmini4-high-resolution-20261027.json').read_text())
    for key, values in metrics.items():
        assert publication['live_runs'][key] == json.loads(data['live/'+key+'.runner.json'])
        for field in ('output_hz','total_four_core_CPU_percent'):
            assert math.isclose(publication['metrics'][key][field],values[field],rel_tol=1e-12)
    assert not publication['qualified'] and not publication['execution_eligible'] and not publication['full_station_qualified']
    assert not publication['RViz']['present_during_measurements'] and not publication['RViz']['validated']
    optimized = {}
    for profile in ('720p','1080p'):
        name = 'live/'+profile+'/cpu'
        runner = json.loads(system_data[name+'.runner.json'])
        observed = json.loads(system_data[name+'.json'])
        summary = runner['summary']
        assert runner['passed'] and runner['collector_exit'] == 0 and runner['profile_matches'] and runner['allocation_count_stable']
        assert summary['passed'] and not summary['execution_eligible'] and not summary['full_station_qualified']
        assert runner['logical_cpus'] == 4 and runner['steady_vulkan_allocation_counts'] == []
        seconds, n, cpu = summary['steady_seconds'],summary['unique_measured_outputs'],summary['container_cpu_ms_proc_stat']
        assert seconds >= 15 and math.isclose(n/seconds,summary['output_hz'],rel_tol=1e-12)
        assert math.isclose(cpu/1000/seconds,runner['container_fraction_of_one_core'],rel_tol=1e-12)
        assert math.isclose(cpu/n,summary['container_cpu_ms_per_unique_output'],rel_tol=1e-12)
        for key in ('source_frame','depth_source_frame'):
            ids = [int(m['fields'][key]) for m in observed['measurements']]
            assert len(ids) == n and all(b>a for a,b in zip(ids,ids[1:]))
            assert sum(b-a-1 for a,b in zip(ids,ids[1:])) == runner['source_frame_gaps'][key]
        published = publication['optimized_system_CPU_baseline']['profiles'][profile]
        assert published['runner'] == runner
        for key, value in [('output_hz',n/seconds),('total_four_core_CPU_percent',25*cpu/1000/seconds),('CPU_ms_per_output',cpu/n)]:
            assert math.isclose(published['metrics'][key],value,rel_tol=1e-12)
        assert publication['metrics'][profile+'/vulkan']['output_hz'] < n/seconds
        optimized[profile] = published['metrics']
    assert publication['metrics']['1080p/vulkan']['CPU_ms_per_output'] > optimized['1080p']['CPU_ms_per_output']
    recorded = json.loads((HERE/'recorded-core/summary.json').read_text())
    assert len(recorded) == 6
    for profile in ('720p','1080p'):
        published = publication['recorded_core']['profiles'][profile]
        rows = {r['mode']:r['result'] for r in recorded if r['profile'] == profile}
        assert rows == published['runs'] and set(rows) == {'compare','cpu','vulkan'}
        for mode, result in rows.items():
            assert result['frames'] == (2 if mode == 'compare' else 20)
            assert result['warmup_frames'] == (1 if mode == 'compare' else 5)
            assert result['hot_path_vulkan_allocations'] == 0 and not result['full_station_qualified']
        assert rows['compare']['dense_readback'] and not rows['vulkan']['dense_readback']
        assert rows['vulkan']['p95_ms'] > rows['cpu']['p95_ms']
        stages = rows['vulkan']['gpu_stage_mean_ms']
        assert math.isclose(100*stages['tiled_boundary_cc']/sum(stages.values()),published['CC_fraction_of_sum_of_stage_means_percent'],rel_tol=1e-12)
    final = json.loads((HERE/'final-state.json').read_text())
    assert final['source_matches_commit'] == manifest['git_revision'] and final['source_files_verified'] == 103
    assert final['camera_fuser_exit'] == 1 and not final['camera_fuser_output']
    assert 'MainPID=181501' in final['owner_unit'] and 'ActiveState=active' in final['owner_unit']
    print(json.dumps(dict(passed=True, runtime_executed=False, files_hashed=len(inventory),
                         raw_members=len(actual), optimized_CPU_raw_members=len(system_actual), native_synthetic_alignment_cases=48,
                         unchanged_CPU_pins=464, live_metrics=metrics, optimized_CPU_metrics=optimized, recorded_Core_runs=6, RViz_validated=False), indent=2))


if __name__ == '__main__':
    main()
