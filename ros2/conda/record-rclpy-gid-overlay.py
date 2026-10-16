"""Write bounded provenance for the isolated native rclpy publisher-GID build."""
import argparse
import hashlib
import importlib.machinery
import json
from pathlib import Path
import subprocess
import sys


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def base_snapshot(env_prefix):
    package=env_prefix/'lib/python3.12/site-packages/rclpy'
    extension=next(package.glob('_rclpy_pybind11*.so'))
    return dict(package=str(package),extension=str(extension),extension_sha256=digest(extension),
                init_sha256=digest(package/'__init__.py'))


def validated_overlay_site(prefix,env_prefix):
    prefix=Path(prefix).resolve();env_prefix=Path(env_prefix).resolve()
    marker=prefix/'share/rclpy_gid_overlay/provenance.json'
    if marker.stat().st_size>65536:raise ValueError('oversized GID overlay provenance')
    report=json.loads(marker.read_text())
    if (sys.version_info[:2]!=(3,12) or report.get('schema')!='rclpy-publisher-gid-overlay-v1' or
            report.get('upstream_version')!='7.1.12' or
            report.get('upstream_commit')!='bd637294dd02b1c128422395c059fa06df8387a3' or
            report.get('patch_sha256')!='30a94800573e524d03eacafa7b208a3d07574902826303166f459838cb69c4b9' or
            report.get('python_abi')!='cpython-312' or report.get('env_prefix')!=str(env_prefix) or
            report.get('prefix')!=str(prefix)):
        raise ValueError('GID overlay source/environment/ABI provenance mismatch')
    if base_snapshot(env_prefix)!=report.get('base_unchanged'):
        raise ValueError('base rclpy bytes no longer match the build ABI baseline')
    site=prefix/'lib/python3.12/site-packages'
    extension=(prefix/report['extension_relative']).resolve()
    if (report.get('site_packages')!=str(site) or extension.parent!=site/'rclpy' or
            not extension.name.startswith('_rclpy_pybind11') or
            not any(extension.name.endswith(suffix) for suffix in importlib.machinery.EXTENSION_SUFFIXES) or
            digest(extension)!=report.get('extension_sha256') or
            digest(site/'rclpy/__init__.py')!=report.get('python_init_sha256')):
        raise ValueError('GID overlay extension/package bytes mismatch')
    return str(site)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env-prefix',type=Path,required=True)
    parser.add_argument('--snapshot-base',type=Path)
    parser.add_argument('--validate-prefix',type=Path)
    parser.add_argument('--prefix',type=Path)
    parser.add_argument('--source',type=Path)
    parser.add_argument('--patch',type=Path)
    parser.add_argument('--upstream-sha')
    args=parser.parse_args()
    if args.validate_prefix:
        print(validated_overlay_site(args.validate_prefix,args.env_prefix));return
    baseline=base_snapshot(args.env_prefix.resolve())
    if args.snapshot_base:
        args.snapshot_base.write_text(json.dumps(baseline,indent=2)+'\n');return
    prefix=args.prefix.resolve()
    if prefix==args.env_prefix.resolve() or args.env_prefix.resolve() in prefix.parents:
        raise ValueError('overlay must not install inside the Conda environment')
    previous=json.loads((prefix.parent/'base-before.json').read_text())
    if previous!=baseline:raise ValueError('base rclpy changed during isolated build')
    actual=subprocess.check_output(['git','-C',str(args.source),'rev-parse','HEAD'],text=True).strip()
    if actual!=args.upstream_sha:raise ValueError('unexpected upstream source')
    site=prefix/'lib/python3.12/site-packages'
    extension=next((site/'rclpy').glob('_rclpy_pybind11*.so'))
    if not any(str(extension).endswith(suffix) for suffix in importlib.machinery.EXTENSION_SUFFIXES):
        raise ValueError('overlay extension ABI does not match build interpreter')
    report=dict(schema='rclpy-publisher-gid-overlay-v1',upstream_repository='https://github.com/ros2/rclpy.git',
        upstream_version='7.1.12',upstream_commit=actual,patch_sha256=digest(args.patch),
        python_version=sys.version,python_executable=sys.executable,python_abi='cpython-312',
        env_prefix=str(args.env_prefix.resolve()),prefix=str(prefix),site_packages=str(site),
        extension_relative=str(extension.relative_to(prefix)),extension_sha256=digest(extension),
        python_init_sha256=digest(site/'rclpy/__init__.py'),
        patched_source_sha256=digest(args.source/'rclpy/src/rclpy/subscription.cpp'),base_unchanged=baseline)
    folder=prefix/'share/rclpy_gid_overlay';folder.mkdir(parents=True,exist_ok=True)
    (folder/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
