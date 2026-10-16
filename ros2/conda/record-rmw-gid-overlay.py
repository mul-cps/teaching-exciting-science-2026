"""Provenance and startup-only validation for the isolated Cyclone GUID fix."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

UPSTREAM='2659b2bf9b453bd40cf4e255da559256f388427e'
PATCH='cac5cba8f1c43d4542eee2878e38f045ede91b387b15e9208cde95d1e4b5172b'

REGISTRATION='rosidl_typesupport_c;rosidl_typesupport_introspection_c;rosidl_typesupport_cpp;rosidl_typesupport_introspection_cpp'
RESOURCE='share/ament_index/resource_index/rmw_typesupport/rmw_cyclonedds_cpp_so101_gid'

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def base_snapshot(env):
    paths=('lib/librmw_cyclonedds_cpp.so','lib/libddsc.so','include/rmw/rmw/types.h','include/dds/dds.h')
    return {p:digest(env/p) for p in paths}

def validated_overlay_lib(prefix,env):
    prefix=Path(prefix).resolve();env=Path(env).resolve()
    marker=prefix/'share/rmw_gid_overlay/provenance.json'
    if marker.stat().st_size>65536:raise ValueError('oversized RMW provenance')
    report=json.loads(marker.read_text())
    if (report.get('schema')!='rmw-cyclonedds-publisher-gid-overlay-v1' or
            report.get('upstream_commit')!=UPSTREAM or report.get('upstream_version')!='2.2.4' or
            report.get('patch_sha256')!=PATCH or report.get('implementation')!='rmw_cyclonedds_cpp_so101_gid' or report.get('prefix')!=str(prefix) or
            report.get('env_prefix')!=str(env) or report.get('base_unchanged')!=base_snapshot(env)):
        raise ValueError('RMW overlay source/environment/ABI baseline mismatch')
    library=(prefix/report['library_relative']).resolve()
    if library!=prefix/'lib/librmw_cyclonedds_cpp_so101_gid.so' or digest(library)!=report.get('library_sha256'):
        raise ValueError('RMW overlay library bytes/path mismatch')
    if (prefix/RESOURCE).read_text()!=REGISTRATION:
        raise ValueError('RMW alias typesupport registration mismatch')
    return str(prefix/'lib')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--env-prefix',type=Path,required=True)
    parser.add_argument('--snapshot-base',type=Path)
    parser.add_argument('--validate-prefix',type=Path)
    parser.add_argument('--prefix',type=Path)
    parser.add_argument('--source',type=Path)
    parser.add_argument('--patch',type=Path)
    args=parser.parse_args();env=args.env_prefix.resolve()
    if args.validate_prefix:
        print(validated_overlay_lib(args.validate_prefix,env));return
    baseline=base_snapshot(env)
    if args.snapshot_base:
        args.snapshot_base.write_text(json.dumps(baseline,indent=2)+'\n');return
    prefix=args.prefix.resolve()
    if prefix==env or env in prefix.parents:raise ValueError('RMW overlay must be isolated from Conda')
    if json.loads((prefix.parent/'base-before.json').read_text())!=baseline:
        raise ValueError('base RMW/DDS bytes changed during isolated build')
    commit=subprocess.check_output(['git','-C',str(args.source),'rev-parse','HEAD'],text=True).strip()
    if commit!=UPSTREAM or digest(args.patch)!=PATCH:raise ValueError('unexpected RMW patch/source')
    library=prefix/'lib/librmw_cyclonedds_cpp_so101_gid.so'
    if (prefix/RESOURCE).read_text()!=REGISTRATION:raise ValueError('invalid alias registration')
    report=dict(schema='rmw-cyclonedds-publisher-gid-overlay-v1',upstream_version='2.2.4',
        upstream_repository='https://github.com/ros2/rmw_cyclonedds.git',upstream_commit=commit,
        patch_sha256=PATCH,implementation='rmw_cyclonedds_cpp_so101_gid',prefix=str(prefix),env_prefix=str(env),library_relative=str(library.relative_to(prefix)),
        library_sha256=digest(library),patched_source_sha256=digest(args.source/'rmw_cyclonedds_cpp/src/rmw_node.cpp'),
        base_unchanged=baseline,lookup='dds_get_matched_publication_data; no handle/graph fallback')
    folder=prefix/'share/rmw_gid_overlay';folder.mkdir(parents=True,exist_ok=True)
    (folder/'provenance.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()
