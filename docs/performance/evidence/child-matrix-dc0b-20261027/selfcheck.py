#!/usr/bin/env python3
"""Verify retained archive bytes/pins and rerun CPU-only stock exact comparisons."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    report = {'scope': 'CPU-only retained evidence selfcheck', 'gpu_executed': False}
    archives = json.loads((HERE/'archives.json').read_text())
    report['archives'] = {}
    with tempfile.TemporaryDirectory(prefix='child-matrix-evidence-') as work:
        work = Path(work)
        for key, record in archives.items():
            path = HERE/record['file']
            assert sha(path) == record['sha256'], path
            with tarfile.open(path) as tar:
                members = tar.getmembers()
                for member in members:
                    assert not member.issym() and not member.islnk()
                    assert not member.name.startswith('/') and '..' not in Path(member.name).parts
                    if member.isfile():
                        assert not tar.extractfile(member).read(4).startswith(b'\x7fELF'), member.name
                if key != 'historical_buggy_inputs':
                    destination = work/key
                    destination.mkdir()
                    tar.extractall(destination, filter='data')
            report['archives'][key] = dict(sha256=sha(path), members=len(members))
        common = work/'common_reproduction'
        matrix = common/'matrix'
        manifest = json.loads((matrix/'manifest.json').read_text())
        assert sha(matrix/'manifest.json') == 'cbc585b0565f905e9a9f5993e43cb7b339b03e7120d416e7ac3a26f821e3a083'
        for relative, wanted in manifest['sources'].items():
            assert sha(common/'source'/relative) == wanted, relative
        for case in manifest['cases'].values():
            assert sha(matrix/case['fixture']) == case['fixture_sha256']
            assert sha(matrix/case['expected']) == case['expected_sha256']
        for kind in ('child', 'parent'):
            identity = manifest['shader_artifacts'][kind+'_shader_identity']
            folder = common/(kind+'-shaders')
            assert sha(folder/'provenance.txt') == identity['provenance_sha256']
            assert (folder/'source.sha256').read_text().strip() == identity['source_sha256']
            for filename, wanted in identity['spirv'].items():
                assert sha(folder/filename) == wanted
        # Use the existing factory, not a duplicate comparator. Its verify command
        # checks every NPZ normal/empty array hash and frozen fixture/source pins.
        factory = ROOT/'tools/qualify_native_children.py'
        verify = subprocess.run([sys.executable, str(factory), 'verify', str(matrix)],
                                cwd=ROOT, capture_output=True, text=True)
        assert verify.returncode == 0, verify.stdout+verify.stderr
        report['factory_verify_stdout_sha256'] = hashlib.sha256(verify.stdout.encode()).hexdigest()
        report['hosts'] = {}
        for host, count in (('intel', 198), ('amd', 192)):
            aggregate = json.loads((HERE/(host+'-comparison.json')).read_text())
            outputs = work/(host+'_outputs')/('workshop-child-matrix-'+host+'-dc0b-20261027.v2')
            pins = json.loads((outputs/'post-run-pins.json').read_text())
            assert len(pins) == count and all(pins.values())
            catalog = outputs/'output-file-hashes.json'
            if catalog.exists():
                for name, wanted in json.loads(catalog.read_text()).items():
                    assert sha(outputs/name) == wanted, name
            passed = []
            for case in aggregate['compares']:
                name = case['case']
                assert sha(outputs/name/'result.json') == case['comparison']['output_json_sha256']
                comparison = common/'comparisons'/(host+'-per-case')/(name+'.json')
                assert sha(comparison) == case['stdout_sha256']
                result = subprocess.run([sys.executable, str(factory), 'compare', str(matrix), name,
                                         str(outputs/name/'result')], cwd=ROOT, capture_output=True, text=True)
                assert result.returncode == 0, (host, name, result.stdout, result.stderr)
                assert json.loads(result.stdout)['matched'] is True
                passed.append(name)
            assert len(passed) == 58
            report['hosts'][host] = dict(exact_compares_passed=len(passed), post_pins=count,
                                        status_counts=aggregate['status_counts'])
    report['passed'] = True
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
