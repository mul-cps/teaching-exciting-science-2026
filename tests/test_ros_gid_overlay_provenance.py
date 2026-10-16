"""Startup overlay validators fail closed; no ROS or native code loaded."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[1]


def module(name):
    path=ROOT/'ros2/conda'/('record-'+name+'-gid-overlay.py')
    spec=importlib.util.spec_from_file_location(name+'_overlay_fixture',path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


def rclpy_fixture(tmp_path,monkeypatch):
    value=module('rclpy');monkeypatch.setattr(value.sys,'version_info',(3,12))
    env=tmp_path/'env';base=env/'lib/python3.12/site-packages/rclpy';base.mkdir(parents=True)
    (base/'_rclpy_pybind11.so').write_bytes(b'base');(base/'__init__.py').write_bytes(b'base init')
    prefix=tmp_path/'overlay';site=prefix/'lib/python3.12/site-packages/rclpy';site.mkdir(parents=True)
    extension=site/'_rclpy_pybind11.so';extension.write_bytes(b'patched');(site/'__init__.py').write_bytes(b'init')
    marker=prefix/'share/rclpy_gid_overlay/provenance.json';marker.parent.mkdir(parents=True)
    report=dict(schema='rclpy-publisher-gid-overlay-v1',upstream_version='7.1.12',
        upstream_commit='bd637294dd02b1c128422395c059fa06df8387a3',
        patch_sha256='30a94800573e524d03eacafa7b208a3d07574902826303166f459838cb69c4b9',
        python_abi='cpython-312',env_prefix=str(env),prefix=str(prefix),site_packages=str(site.parent),
        extension_relative=str(extension.relative_to(prefix)),extension_sha256=value.digest(extension),
        python_init_sha256=value.digest(site/'__init__.py'),base_unchanged=value.base_snapshot(env))
    marker.write_text(json.dumps(report));return value,env,prefix,marker,report


@pytest.mark.parametrize('fault',[None,'patch','hash','abi','env','path','base'])
def test_rclpy_overlay_exact_source_abi_path_and_base(tmp_path,monkeypatch,fault):
    value,env,prefix,marker,report=rclpy_fixture(tmp_path,monkeypatch)
    if fault=='patch':report['patch_sha256']='f'*64
    if fault=='hash':report['extension_sha256']='f'*64
    if fault=='abi':report['python_abi']='cpython-313'
    if fault=='env':report['env_prefix']=str(tmp_path/'wrong')
    if fault=='path':report['extension_relative']='/tmp/outside.so'
    if fault=='base':(env/'lib/python3.12/site-packages/rclpy/__init__.py').write_bytes(b'changed')
    marker.write_text(json.dumps(report))
    if fault:
        with pytest.raises(ValueError):value.validated_overlay_site(prefix,env)
    else:assert value.validated_overlay_site(prefix,env)==str(prefix/'lib/python3.12/site-packages')


@pytest.mark.parametrize('fault',[None,'patch','hash','env','path','base','registration','implementation'])
def test_rmw_overlay_exact_source_library_and_base(tmp_path,fault):
    value=module('rmw');env=tmp_path/'env';prefix=tmp_path/'overlay'
    for path in ('lib/librmw_cyclonedds_cpp.so','lib/libddsc.so','include/rmw/rmw/types.h','include/dds/dds.h'):
        p=env/path;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(path.encode())
    library=prefix/'lib/librmw_cyclonedds_cpp_so101_gid.so';library.parent.mkdir(parents=True);library.write_bytes(b'patched')
    resource=prefix/value.RESOURCE;resource.parent.mkdir(parents=True);resource.write_text(value.REGISTRATION)
    report=dict(schema='rmw-cyclonedds-publisher-gid-overlay-v1',upstream_version='2.2.4',
        upstream_commit=value.UPSTREAM,patch_sha256=value.PATCH,implementation='rmw_cyclonedds_cpp_so101_gid',prefix=str(prefix),env_prefix=str(env),
        base_unchanged=value.base_snapshot(env),library_relative=str(library.relative_to(prefix)),library_sha256=value.digest(library))
    marker=prefix/'share/rmw_gid_overlay/provenance.json';marker.parent.mkdir(parents=True)
    if fault=='patch':report['patch_sha256']='f'*64
    if fault=='hash':report['library_sha256']='f'*64
    if fault=='env':report['env_prefix']=str(tmp_path/'wrong')
    if fault=='path':report['library_relative']='/tmp/outside.so'
    if fault=='base':(env/'lib/libddsc.so').write_bytes(b'changed')
    if fault=='registration':resource.write_text('wrong')
    if fault=='implementation':report['implementation']='rmw_cyclonedds_cpp'
    marker.write_text(json.dumps(report))
    if fault:
        with pytest.raises(ValueError):value.validated_overlay_lib(prefix,env)
    else:assert value.validated_overlay_lib(prefix,env)==str(prefix/'lib')


def test_overlay_validation_does_not_use_optimizable_asserts():
    for name in ('rclpy','rmw'):
        tree=ast.parse((ROOT/'ros2/conda'/('record-'+name+'-gid-overlay.py')).read_text())
        assert not any(isinstance(node,ast.Assert) for node in ast.walk(tree))


def test_native_loaned_take_uses_its_declared_subscription():
    patch=(ROOT/'ros2/patches/rmw-cyclonedds-2.2.4-publisher-gid.patch').read_text()
    assert '+        message_info_from_sample_info(cdds_subscription, info, message_info);' in patch
