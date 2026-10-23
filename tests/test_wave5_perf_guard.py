"""Independent no-device regression guards for the newly reviewed boundaries."""
import ast
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2/src/so101_station_vulkan'


def identity_node():
    module = ast.parse((ROOT/'workshop/blockly_ros.py').read_text())
    cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name=='ROSBackend')
    return next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='_identity')


def cached_identity():
    node=identity_node();node.decorator_list=[]
    namespace={'time':SimpleNamespace(monotonic=lambda:100.),'math':math,'hashlib':hashlib,'json':json}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])), '<cached identity>', 'exec'),namespace)
    return namespace['_identity']


def status():
    return dict(schema=1,kind='follower_status',sampled_monotonic=100.,fresh=True,
                position_age_s=0.,torque_age_s=0.,owner_source_sha256={'module':'a'*64},
                owner_started_monotonic=99.,arm_id='synthetic',calibration_sha256='b'*64,
                jaw_calibration_sha256='c'*64,joint_calibration_sha256='d'*64)


def test_cached_identity_has_bounded_new_fields_and_no_filesystem_or_rpc_calls():
    node=identity_node()
    calls={ast.unparse(n.func) for n in ast.walk(node) if isinstance(n,ast.Call)}
    assert calls <= {'status.get','time.monotonic','type','math.isfinite','isinstance','len','any',
                     'RuntimeError','calibrations.values','hashlib.sha256','json.dumps',
                     "json.dumps(source, sort_keys=True, separators=(',', ':')).encode",
                     "hashlib.sha256(json.dumps(source, sort_keys=True, separators=(',', ':')).encode()).hexdigest"}
    text=ast.unparse(node)
    assert "('jaw_calibration_sha256', 'joint_calibration_sha256')" in text
    assert 'len(value) != 64' in text
    fn=cached_identity();sample=status();result=fn(sample)
    assert result['jaw_calibration_sha256']=='c'*64 and result['joint_calibration_sha256']=='d'*64
    sample.pop('jaw_calibration_sha256');sample.pop('joint_calibration_sha256')
    assert fn(sample)['jaw_calibration_sha256'] is None  # recovery does not need motion calibration


def test_native_loaded_digest_metadata_reads_only_cached_attributes():
    source=ROOT/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'
    tree=ast.parse(source.read_text());node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_status')
    fields={}
    for value in ast.walk(node):
        if isinstance(value,ast.Dict):
            for key,item in zip(value.keys,value.values):
                if isinstance(key,ast.Constant) and key.value in {'jaw_calibration_sha256','joint_calibration_sha256'}:
                    fields[key.value]=item
    assert set(fields)=={'jaw_calibration_sha256','joint_calibration_sha256'}
    for value in fields.values():
        assert {ast.unparse(call.func) for call in ast.walk(value) if isinstance(call,ast.Call)} <= {'getattr'}
        assert '_sha256' in ast.unparse(value)


def test_feature_negotiation_is_cold_and_fused_command_loop_is_bounded():
    context=(PACKAGE/'src/vulkan_context.cpp').read_text()
    cold=context.split('Context::Context(){',1)[1].split('Context::~Context()',1)[0]
    assert context.count('vkGetPhysicalDeviceFeatures2(')==cold.count('vkGetPhysicalDeviceFeatures2(')==1
    code=(PACKAGE/'src/gpu_pipeline.cpp').read_text().split('void Pipeline::submit(',1)[1].split('void Pipeline::finish(',1)[0]
    for forbidden in ('vkGetPhysicalDevice','vkEnumerate','vkWait','vkAllocate','vkCreate','vkMapMemory','std::vector','new '):
        assert forbidden not in code
    assert 'parent_copy' not in code and 'copy_canonical_parents' not in code
    assert 'for(unsigned i=0;i<32;i++){dispatch(3);dispatch(2);}' in code
    assert 'vkCmdCopyBuffer(s.command,s.scratch.buffer,s.scratch.buffer' not in code
    assert 'VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT|VK_ACCESS_TRANSFER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT' in code
    cmake=(PACKAGE/'CMakeLists.txt').read_text()
    inventory=cmake.split('foreach(VIDEO_SOURCE ',1)[1].split(')',1)[0]
    assert 'src/vulkan_context.cpp' in inventory and 'include/so101_station_vulkan/pipeline.hpp' in inventory


def test_dense_preparation_copy_is_diagnostic_only_and_teardown_retains_pending_transfer():
    normal=(PACKAGE/'src/geometry.cpp').read_text()
    assert 'vkCmdCopyBuffer' not in normal
    qualifier=(PACKAGE/'test/qualify_geometry.cpp').read_text()
    assert 'void snapshot(const GeometryLease& frame)' in qualifier
    assert 'frame.resident_buffer()' in qualifier
    assert 'VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_ACCESS_SHADER_WRITE_BIT,VK_ACCESS_TRANSFER_READ_BIT' in qualifier
    assert 'pending=target' in qualifier and 'pending=VK_NULL_HANDLE' in qualifier
    assert '~Transfer(){if(pending)vkWaitForFences' in qualifier
    assert 'catch(...){if(fence)vkDestroyFence' in qualifier
    cmake=(PACKAGE/'cmake/geometry.cmake').read_text()
    assert 'add_executable(qualify_geometry' in cmake
    assert 'add_test(NAME qualify_geometry' not in cmake
