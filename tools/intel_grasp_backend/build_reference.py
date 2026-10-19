#!/usr/bin/env python3
"""Fetch/build pinned offline GPD in a private cache; never install globally."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
try:
    from .runtime_env import converter_environment
except ImportError:
    from runtime_env import converter_environment

SOURCES = {
    'gpg': ('https://github.com/atenpas/gpg.git', '3dcd656d70f095ad1bda3d2fb597a994198466ab'),
    'gpd': ('https://github.com/sharronliu/gpd.git', 'fdd699308eee846bffd93b8efd4029469e6aef0b'),
    'intel': ('https://github.com/intel/ros2_grasp_library.git', '980b7ddd4348d9aa78c6e84c42da1984746867db'),
}
HERE = Path(__file__).resolve().parent


def run(command, *, cwd=None, log=None, env=None):
    if log:
        with Path(log).open('w') as stream:
            subprocess.run(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, check=True, env=env)
    else:
        subprocess.run(command, cwd=cwd, check=True, env=env)


def prepare_source(root, name, url, revision):
    path = root/name
    if not path.exists():
        path.mkdir(parents=True)
        run(['git', 'init', '-q', str(path)])
        run(['git', 'remote', 'add', 'origin', url], cwd=path)
        run(['git', 'fetch', '--depth', '1', 'origin', revision], cwd=path)
        run(['git', '-c', 'advice.detachedHead=false', 'checkout', '--detach', revision], cwd=path)
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=path, text=True).strip()
    remote = subprocess.check_output(['git', 'remote', 'get-url', 'origin'], cwd=path, text=True).strip()
    if actual != revision or remote != url:
        raise ValueError('refusing an unexpected source checkout: '+str(path))
    return path


def patch_comparators(source):
    name = 'include/gpg/cloud_camera.h'
    original = subprocess.check_output(['git', 'show', 'HEAD:'+name], cwd=source).decode()
    patched = original
    for n in (3, 4):
        signature = f'bool operator ()(const Eigen::Vector{n}i& a, const Eigen::Vector{n}i& b)'
        if original.count(signature) != 1:
            raise ValueError('unexpected pinned GPG comparator source')
        patched = patched.replace(signature, signature+' const')
    path = source/name
    if path.read_text() not in (original, patched):
        raise ValueError('refusing unrelated edits in GPG source')
    path.write_text(patched)
    return hashlib.sha256(patched.encode()).hexdigest()


def build(cache, converter_python, *, jobs=2):
    cache = Path(cache).resolve()
    if jobs not in (1, 2):
        raise ValueError('offline builds use one or two jobs')
    marker = cache/'station-offline-gpd-cache.json'
    if cache.exists() and any(cache.iterdir()) and not marker.is_file():
        raise ValueError('refusing nonempty directory without this task cache marker')
    cache.mkdir(parents=True, exist_ok=True)
    expected = {'schema': 'station-offline-gpd-cache-v1', 'sources': SOURCES}
    if marker.exists() and json.loads(marker.read_text()) != json.loads(json.dumps(expected)):
        raise ValueError('unexpected private cache marker')
    marker.write_text(json.dumps(expected, indent=2)+'\n')
    sources = {name: prepare_source(cache/'sources', name, *pin) for name, pin in SOURCES.items()}
    for name, source in sources.items():
        changed = subprocess.check_output(['git', 'status', '--porcelain'], cwd=source, text=True)
        allowed = ' M include/gpg/cloud_camera.h\n' if name == 'gpg' else ''
        if changed not in ('', allowed):
            raise ValueError('refusing unrelated source edits: '+str(source))
    comparator_sha = patch_comparators(sources['gpg'])
    logs = cache/'logs'; logs.mkdir(exist_ok=True)
    model = cache/'model/original-gpd.onnx'
    run([str(converter_python), str(HERE/'convert_legacy_model.py'), str(sources['gpd']),
         str(model), '--validate'], log=logs/'model-conversion.log', env=converter_environment())
    prefix = cache/'build'
    run(['cmake', '-S', str(HERE), '-B', str(prefix), '-DCMAKE_BUILD_TYPE=Release',
         '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DGPG_SOURCE='+str(sources['gpg']),
         '-DGPD_SOURCE='+str(sources['gpd'])], log=logs/'configure.log')
    run(['nice', '-n', '15', 'cmake', '--build', str(prefix), '--target', 'offline_gpd',
         'classifier_probe', 'classifier_benchmark', '--parallel', str(jobs)], log=logs/'build.log')
    binaries = [prefix/'offline_gpd', prefix/'classifier_probe', prefix/'classifier_benchmark',
                prefix/'libstation_grasp_pose_detection.so', prefix/'upstream_gpg/libgrasp_candidates_generator.so']
    if not all(path.is_file() for path in binaries):
        raise ValueError('expected isolated binary artifacts missing')
    packages = []
    env_prefix = os.environ.get('CONDA_PREFIX', '')
    if env_prefix:
        wanted = {'pcl', 'libopencv', 'eigen', 'boost-cpp', 'libboost', 'cmake', 'gxx_impl_linux-64', 'nlohmann_json'}
        for path in (Path(env_prefix)/'conda-meta').glob('*.json'):
            record = json.loads(path.read_text())
            if record.get('name') in wanted:
                packages.append({key: record.get(key) for key in ('name', 'version', 'build', 'sha256')})
    manifest = {'schema': 'station-offline-gpd-build-v1', 'sources': SOURCES,
                'source_paths': {key: str(value) for key, value in sources.items()},
                'patched_gpg_header_sha256': comparator_sha, 'model': str(model),
                'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
                'model_conversion': json.loads(model.with_suffix('.conversion.json').read_text()),
                'engine': str(prefix/'offline_gpd'), 'cache': str(cache),
                'binaries_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in binaries},
                'compat_source_sha256': {str(path.relative_to(HERE)): hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in HERE.iterdir() if path.is_file()},
                'conda_packages': packages, 'conda_prefix': env_prefix, 'jobs': jobs,
                'license_sha256': {key: hashlib.sha256((sources[key]/name).read_bytes()).hexdigest()
                                   for key, name in [('gpg', 'LICENSE'), ('gpd', 'LICENSE.md'), ('intel', 'LICENSE')]},
                'cpu_only_reference': True, 'archived_ros_node_built': False,
                'global_install': False, 'qualified': False, 'execution_enabled': False}
    encoded = (json.dumps(manifest, indent=2)+'\n').encode()
    history = cache/'build-manifests'; history.mkdir(exist_ok=True)
    version = history/(hashlib.sha256(encoded).hexdigest()+'.json')
    if not version.exists():
        version.write_bytes(encoded)
    (cache/'build-manifest.json').write_bytes(encoded)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--converter-python', type=Path, required=True)
    parser.add_argument('--jobs', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    try:
        result = build(args.cache, args.converter_python, jobs=args.jobs)
        print(json.dumps({'manifest': str(Path(args.cache).resolve()/'build-manifest.json'),
                          'engine': result['engine'], 'qualified': False}, indent=2))
    except (ValueError, subprocess.CalledProcessError) as error:
        print('offline GPD build failed: '+str(error)+'; inspect cache/logs', file=sys.stderr)
        raise SystemExit(1)
