"""Loaded calibration digests describe parsed bytes, never later disk edits."""
import ast
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import pytest

SOURCE=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'


def helpers():
    tree=ast.parse(SOURCE.read_text())
    selected=[node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name in {'_load_calibration_snapshot','_gripper_calibration_status'}]
    namespace={'Path':Path,'hashlib':hashlib,'json':json}
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(SOURCE),'exec'),namespace)
    return namespace


def test_loaded_digest_survives_file_edit(tmp_path):
    module=helpers();path=tmp_path/'calibration.json';raw=b'{ "qualified": true }\n';path.write_bytes(raw)
    value,digest=module['_load_calibration_snapshot'](path,lambda data:data if data['qualified'] else (_ for _ in ()).throw(ValueError()))
    obj=SimpleNamespace(_gripper_jaw=value,_gripper_joint=value,_gripper_jaw_sha256=digest,_gripper_joint_sha256=digest)
    path.write_text('{"qualified":false}')
    response=module['_gripper_calibration_status'](obj,None,SimpleNamespace())
    status=json.loads(response.message)
    assert response.success is True
    assert status['jaw_calibration_sha256']==hashlib.sha256(raw).hexdigest()
    assert status['joint_calibration_sha256']==hashlib.sha256(raw).hexdigest()
    assert status['jaw_calibration_sha256']!=hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize('raw',[b'{',b'{"qualified":false}'])
def test_malformed_or_unqualified_never_returns_digest(tmp_path,raw):
    module=helpers();path=tmp_path/'bad.json';path.write_bytes(raw)
    def validate(data):
        if data.get('qualified') is not True: raise ValueError('unqualified')
        return data
    with pytest.raises(ValueError): module['_load_calibration_snapshot'](path,validate)


def test_status_has_no_io_or_hashing():
    node=next(n for n in ast.walk(ast.parse(SOURCE.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_gripper_calibration_status')
    assert {ast.unparse(n.func) for n in ast.walk(node) if isinstance(n,ast.Call)} <= {'getattr','dict','json.dumps'}
