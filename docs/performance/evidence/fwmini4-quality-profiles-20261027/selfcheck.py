"""Read-only artifact/source verification and offline recomputation; no devices."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]


def verify_source_revision(root, revision, pins):
    if not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('source revision must be a full immutable Git commit ID')
    for name, expected in pins.items():
        data = subprocess.check_output(['git', 'cat-file', 'blob', revision + ':' + name], cwd=root)
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError('source revision hash mismatch: ' + name)


def main():
    os.chdir(ROOT)
    inventory = json.loads((BASE / 'inventory.json').read_text())
    for name, expected in inventory['files'].items():
        actual = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('artifact hash mismatch: ' + name)
    native = json.loads((BASE / 'checks/quality-profiles-native-source-pins-20261027.json').read_text())
    integration = json.loads((BASE / 'integration-review/final-source-sha256.json').read_text())
    revision = json.loads((BASE / 'context.json').read_text())['source_commit']
    verify_source_revision(ROOT, revision, {**native, **integration})
    spec = importlib.util.spec_from_file_location('offline_analysis', BASE / 'scripts/quality-profiles-rviz-analysis-20261027.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actual = module.analyze((BASE / 'raw').relative_to(ROOT))
    expected = json.loads((ROOT / 'docs/performance/fwmini4-quality-profiles-20261027.json').read_text())
    if not actual['analysis_valid'] or actual != expected:
        raise ValueError('offline analysis mismatch')
    print(json.dumps({'artifact_files_verified': len(inventory['files']),
                      'native_source_files_verified': len(native),
                      'integration_source_test_pins_verified': len(integration),
                      'source_revision_verified': revision,
                      'working_tree_source_checked': False,
                      'recomputed_runs': len(actual['runs']), 'analysis_valid': True,
                      'qualified': False, 'execution_eligible': False}))


if __name__ == '__main__':
    main()
