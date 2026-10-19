#!/usr/bin/env python3
"""Run an actual pinned offline engine and bind output to recorded source provenance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
try:
    from .runtime_env import converter_environment
    from .pcd_guard import checked_pcd
except ImportError:
    from runtime_env import converter_environment
    from pcd_guard import checked_pcd


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_inputs(build_manifest, pcd, provenance):
    build_manifest, pcd, provenance = map(Path, (build_manifest, pcd, provenance))
    build = json.loads(build_manifest.read_text())
    source = json.loads(provenance.read_text())
    if (build.get('schema') != 'station-offline-gpd-build-v1'
            or source.get('schema') != 'station-frozen-native-pcd-v1'
            or source.get('live_timestamp_refresh') is not False
            or not isinstance(source.get('acquisition_stamp_ns'), int)
            or not 0 < source['acquisition_stamp_ns'] < 2147483648*1_000_000_000
            or not source.get('frame_id') or len(source['frame_id']) > 128
            or len(source.get('pcd_sha256', '')) != 64):
        raise ValueError('pinned build and original recorded acquisition provenance required')
    checked_pcd(pcd)  # Bound decoded storage and finite XYZ before any subprocess.
    if digest(pcd) != source['pcd_sha256']:
        raise ValueError('recorded PCD hash/size mismatch')
    if digest(build['model']) != build['model_sha256']:
        raise ValueError('converted original model hash mismatch')
    for path, expected in build['binaries_sha256'].items():
        if digest(path) != expected:
            raise ValueError('isolated engine binary hash mismatch: '+path)
    return build, source


def run_once(build_manifest, pcd, provenance, output_directory, *, converter_python=None,
             audit=False, timeout_s=10.):
    if not .1 <= timeout_s <= 30.:
        raise ValueError('offline reference timeout must be .1..30 s')
    build_bytes = Path(build_manifest).read_bytes()
    source_bytes = Path(provenance).read_bytes()
    build, source = checked_inputs(build_manifest, pcd, provenance)
    if (Path(build_manifest).read_bytes(), Path(provenance).read_bytes()) != (build_bytes, source_bytes):
        raise ValueError('source/build metadata changed during admission')
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=False)
    (output/'source-build-manifest.json').write_bytes(build_bytes)
    (output/'source-pcd-provenance.json').write_bytes(source_bytes)
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2')
    environment.pop('STATION_GPD_CLASSIFIER_AUDIT_DIR', None)
    if audit:
        environment['STATION_GPD_CLASSIFIER_AUDIT_DIR'] = str(output/'classifier-audit')
    start = time.monotonic()
    with (output/'engine.log').open('w') as stream:
        try:
            result = subprocess.run([build['engine'], str(pcd), build['model'],
                                     str(output/'engine-result.json')], env=environment,
                                    stdin=subprocess.DEVNULL, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=timeout_s)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError('offline engine timed out; no automatic retry') from error
    elapsed = time.monotonic()-start
    build_after, source_after = checked_inputs(build_manifest, pcd, provenance)
    if build_after != build or source_after != source:
        raise ValueError('source/build changed during actual engine run')
    if result.returncode not in (0, 3):
        raise RuntimeError('actual offline engine failed; inspect '+str(output/'engine.log'))
    report = json.loads((output/'engine-result.json').read_text())
    if (report.get('schema') != 'station-offline-upstream-gpd-v1'
            or report.get('qualified') is not False or report.get('execution_enabled') is not False):
        raise ValueError('unexpected actual-engine schema/qualification')
    if len(report['grasps']) > 5:
        raise ValueError('unexpected unbounded actual-engine result')
    for grasp in report['grasps']:
        grasp.update(source_pcd_sha256=source['pcd_sha256'],
                     source_acquisition_stamp_ns=source['acquisition_stamp_ns'],
                     source_frame_id=source['frame_id'], target_association_verified=False,
                     tool_mapping_qualified=False, execution_enabled=False, qualified=False)
    report.update(recorded_source=True, live_timestamp_refresh=False,
                  source_dataset_id='offline-pcd:'+source['pcd_sha256'],
                  source_provenance=source,
                  source_build_manifest_sha256=hashlib.sha256(build_bytes).hexdigest(),
                  upstream_source_pins=build['sources'], converted_model_sha256=build['model_sha256'],
                  engine_binary_sha256=build['binaries_sha256'][build['engine']],
                  engine_exit_code=result.returncode, subprocess_elapsed_s=elapsed)
    if audit and report['classification']['scored_images']:
        if converter_python is None:
            raise ValueError('actual-image parity audit requires explicit isolated converter Python')
        audit_path = output/'classifier-audit/batch_0.json'
        command = [str(converter_python), str(Path(__file__).with_name('validate_classifier_audit.py')),
                   build['source_paths']['gpd'], str(audit_path),
                   str(Path(build['model']).with_suffix('.prototxt')),
                   '--output', str(output/'actual-image-parity.json')]
        with (output/'actual-image-parity.log').open('w') as stream:
            subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True,
                           timeout=timeout_s, env=converter_environment())
        report['actual_image_parity'] = json.loads((output/'actual-image-parity.json').read_text())
    (output/'recorded-result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-manifest', type=Path, required=True)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--provenance', type=Path, required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--converter-python', type=Path)
    parser.add_argument('--audit', action='store_true')
    args = parser.parse_args()
    report = run_once(args.build_manifest, args.pcd, args.provenance, args.output_directory,
                      converter_python=args.converter_python, audit=args.audit)
    print(json.dumps({'recorded_result': str(args.output_directory/'recorded-result.json'),
                      'grasps': len(report['grasps']), 'classification': report['classification'],
                      'qualified': False, 'recorded_source': True}, indent=2))
