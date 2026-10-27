"""Read-only artifact/source verification and offline recomputation; no devices."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]


def main():
    os.chdir(ROOT)
    inventory = json.loads((BASE / 'inventory.json').read_text())
    for name, expected in inventory['files'].items():
        actual = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('artifact hash mismatch: ' + name)
    native = json.loads((BASE / 'checks/quality-profiles-native-source-pins-20261027.json').read_text())
    integration = json.loads((BASE / 'integration-review/final-source-sha256.json').read_text())
    for name, expected in {**native, **integration}.items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError('source hash mismatch: ' + name)
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
                      'recomputed_runs': len(actual['runs']), 'analysis_valid': True,
                      'qualified': False, 'execution_eligible': False}))


if __name__ == '__main__':
    main()
