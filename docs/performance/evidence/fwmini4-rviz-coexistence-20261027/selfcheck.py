#!/usr/bin/env python3
"""Artifact-only verification: no ROS, GL driver, compiler, device or network."""
from pathlib import Path
import hashlib
import gzip
import importlib.util
import json
import math
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    inventory = json.loads((HERE/'inventory.json').read_text())
    for name, wanted in inventory.items():
        p = HERE/name
        assert p.stat().st_size == wanted['bytes'] and sha(p) == wanted['sha256'], name
        assert not p.read_bytes().startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff')), name
    archive = HERE/'fwmini4-rviz-coexistence-raw-20261027.tar.gz'
    assert sha(archive) == '1f33b4e14ab0db33da14031ca242e624e3b95e8fe00d0a55793034122f9a26b7'
    wanted = {r['file']:r for r in json.loads((HERE/'raw-archive-inventory.json').read_text())}
    ui_archive = HERE/'fwmini4-rviz-ui-metadata-20261027.tar.gz'
    assert sha(ui_archive) == 'db9bdee33df0eb5de5c9b421fe7cd5c331509f37d38b883455555f6bd3371167'
    ui_wanted = {r['file']:r for r in json.loads((HERE/'ui-metadata-archive-inventory.json').read_text())}
    ui_actual = {}
    with tarfile.open(ui_archive) as tar:
        for member in tar:
            assert not member.issym() and not member.islnk()
            assert not member.name.startswith('/') and '..' not in Path(member.name).parts
            if not member.isfile():
                continue
            content = tar.extractfile(member).read()
            assert not content.startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff'))
            ui_actual[member.name] = dict(file=member.name,bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
    assert ui_actual == ui_wanted and len(ui_actual) == 11
    source = HERE/'analysis/rviz-coexistence-analysis-20261027.py'
    spec = importlib.util.spec_from_file_location('retained_analysis', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    expected = json.loads((HERE/'analysis/results.json').read_text())
    published = json.loads((HERE.parent.parent/'fwmini4-rviz-coexistence-20261027.json').read_text())
    assert not published['qualified'] and not published['execution_eligible'] and not published['full_station_qualified']
    assert published['UI_content_validation'] == json.loads((HERE/'root-UI-manual-inspection.json').read_text())
    assert published['UI_content_validation']['verified'] and not published['UI_content_validation']['image_published']
    assert published['RViz_uses_actual_camera_RGB'] and not published['matched_camera_pixels_between_modes']
    actual, proof = {}, {}
    with tempfile.TemporaryDirectory(prefix='rviz-artifact-selfcheck-') as tmp:
        root = Path(tmp)
        with tarfile.open(archive) as tar:
            for member in tar:
                assert not member.issym() and not member.islnk()
                assert not member.name.startswith('/') and '..' not in Path(member.name).parts
                if not member.isfile():
                    continue
                content = tar.extractfile(member).read()
                assert not content.startswith((b'\x7fELF', b'\x89PNG', b'\xff\xd8\xff'))
                actual[member.name] = dict(file=member.name, bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
                target = root/member.name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(content)
        assert actual == wanted and len(actual) == 87
        result = module.analyze(root)
        assert result['analysis_valid'] and len(result['runs']) == 6
        for key, row in result['runs'].items():
            stored = expected['runs'][key]
            assert {k:v for k,v in row.items() if k != 'inputs'} == {k:v for k,v in stored.items() if k != 'inputs'}, key
            profile, mode = key.split('/')
            lane = 'vulkan' if mode == 'vulkan' else 'system-cpu'
            data = json.loads((root/lane/'live'/profile/(mode+'.json')).read_text())
            seconds = data['steady_seconds']
            n = len({m['fields']['source_frame'] for m in data['measurements']})
            cpu = data['container_cpu_ms_proc_stat']
            metric = published['metrics'][key]
            for field,value in [('unique_output_hz',n/seconds), ('CPU_ms_per_output',cpu/n), ('total_four_core_CPU_percent',25*cpu/1000/seconds)]:
                assert math.isclose(metric['perception'][field], value, rel_tol=1e-12), (key,field)
            start = row['perception']['window']['begin_monotonic_approx']
            end = row['perception']['window']['end_monotonic_approx']
            primary = row['GLX']['primary_viewport']
            assert primary['dimensions'] == [{'width':574,'height':810}]
            assert primary['largest_actual_area'] == max(v['largest_actual_area'] for v in row['GLX']['viewports'])
            assert len(row['GLX']['viewports']) == 3
            assert primary['renderer_record']['metadata_complete'] and primary['renderer_record']['direct']
            assert primary['renderer_record']['vendor'] == 'Intel' and 'ADL-N' in primary['renderer_record']['renderer']
            for viewport in row['GLX']['viewports']:
                intervals = viewport['intervals']
                assert all(start*1e9 <= i['start_ns'] < i['end_ns'] <= end*1e9 for i in intervals)
                elapsed = sum(i['elapsed_ns'] for i in intervals)/1e9
                count = sum(i['interval_swaps'] for i in intervals)
                assert math.isclose(viewport['weighted_swap_hz'],count/elapsed,rel_tol=1e-12)
            assert math.isclose(metric['primary_GLX']['weighted_swap_hz'],primary['weighted_swap_hz'],rel_tol=1e-12)
            assert row['perception']['allocation_safety']['counts'] == ([35] if mode=='vulkan' else [0])
            observer = row['observer']
            assert metric['observer_window'] == observer['window']
            assert metric['observer_processes'] == observer['processes']
            assert all(not p['errors'] for p in observer['processes'].values())
            assert not observer['observer_process_CPU_included_in_target_sum']
            assert metric['approximate_combined_CPU']['different_accounting_windows']
            proof[key] = dict(perception_Hz=n/seconds, primary_GLX_Hz=primary['weighted_swap_hz'],
                              GLX_intervals=primary['interval_count'], GLX_covered_seconds=primary['covered_seconds'],
                              observer_endpoint_offsets_seconds=[observer['window']['begin_offset_seconds'],observer['window']['end_offset_seconds']])
        for profile in ('vga','720p','1080p'):
            before = result['runs'][profile+'/vulkan']['perception']['window']
            after = result['runs'][profile+'/cpu']['perception']['window']
            assert before['end_monotonic_approx'] < after['begin_monotonic_approx']
    review = json.loads((HERE/'review/review-proof-final.json').read_text())
    for key,relative in [('GLX','probe/swap_probe.c'),('scene','scene/visual_scene.py'),('rviz_config','scene/rviz-coexistence.rviz'),('RSP','scene/rsp.yaml')]:
        assert sha(HERE/relative) == review['source_pins'][key][1]
    description = json.loads((HERE/'scene/description-source-manifest.json').read_text())
    assert description['asset_files_not_published'] and description['all_actual_bundle_files_match_cache']
    patch = HERE/'scene/upstream-local.patch.gz'
    assert sha(patch) == description['local_patch_gzip_sha256']
    assert hashlib.sha256(gzip.decompress(patch.read_bytes())).hexdigest() == description['local_patch_sha256']
    assert actual['rviz-process-sampler-20261027.py']['sha256'] == review['source_pins']['sampler'][1]
    analysis_review = json.loads((HERE/'review/analyzer/review-proof-final.json').read_text())
    assert sha(source) == analysis_review['source_sha256']
    assert sha(HERE/'analysis/tests/test_analysis.py') == analysis_review['test_source_sha256']
    assert analysis_review['approved'] and analysis_review['unit_tests'] == 17
    native = json.loads((HERE/'native-source/workshop-highres-source-manifest-20261027.json').read_text())
    check = json.loads((HERE/'native-source/commit-source-check.json').read_text())
    assert native['git_revision'] == published['source_commit'] == check['commit'] and len(native['files']) == 103
    assert sha(HERE/'native-source/workshop-highres-source-manifest-20261027.json') == check['manifest_sha256']
    print(json.dumps(dict(passed=True,runtime_executed=False,inventory_files=len(inventory),raw_members=len(actual),UI_metadata_members=len(ui_actual),
                         native_source_files=103,omitted_description_asset_files=len(description['files']),
                         six_run_reanalysis_matches=True,profile_mode_windows_disjoint=True,metrics=proof),indent=2))


if __name__ == '__main__':
    main()
