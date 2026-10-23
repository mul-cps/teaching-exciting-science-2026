"""Parent diagnostic factory/parser/comparison; no GPU/device enumeration."""
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
SPEC=importlib.util.spec_from_file_location('parent_qualifier',ROOT/'tools/qualify_native_parents.py')
q=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(q)

@pytest.fixture(scope='module')
def cases(tmp_path_factory):
    directory=tmp_path_factory.mktemp('parent-qualifier')/'prepared'
    manifest=q.prepare(directory)
    return directory,manifest

@pytest.fixture(scope='module')
def offline(tmp_path_factory):
    binary=tmp_path_factory.mktemp('parent-offline')/'qualifier'
    subprocess.run(['g++','-std=c++17','-O1','-DPARENT_QUALIFIER_CPU_ONLY',
                    str(ROOT/'tools/performance/qualify_parents.cpp'),'-o',str(binary)],check=True,capture_output=True,text=True)
    return binary


def test_factory_committed_oracle_and_fresh_only(cases):
    directory,m=cases
    assert len(m['cases'])==17 and m['gpu_expected_from_native'] is False
    assert m['cases']['isolated-four-colors-64x64']['record_count']==4096
    assert m['cases']['large-all-red-640x480']['record_count']==1
    assert m['cases']['threshold-uintmax-2x2']['record_count']==0
    q.verify(directory)
    with pytest.raises(FileExistsError):q.prepare(directory)
    text=(ROOT/'tools/qualify_native_parents.py').read_text()
    assert 'connectedComponentsWithStatsWithAlgorithm' in text and 'CCL_GRANA' in text
    assert 'parent_graph' not in text and 'resident_parents' not in text.split('def reference(',1)[1].split('def scenes(',1)[0]


def test_cpp_offline_all_inputs_and_sha256(cases,offline):
    directory,m=cases
    for name,c in m['cases'].items():
        output=json.loads(subprocess.run([str(offline),'--offline',str(directory/c['fixture'])],check=True,text=True,capture_output=True).stdout)
        assert output['width']==c['width'] and output['height']==c['height']
        assert output['fixture_sha256']==q.digest(directory/c['fixture'])
        assert output['executable_sha256']==q.digest(offline)
        assert output['calibration_generation']==17 and output['configuration_generation']==23
        assert output['scope']=='parentprep_only' and not output['physical_camera']
    denied=subprocess.run([str(offline),'--vulkan','missing','missing','missing'],text=True,capture_output=True)
    assert denied.returncode and 'watchdog' in denied.stderr
    denied=subprocess.run([str(offline),'--vulkan','missing','missing','missing','--external-watchdog-ack'],text=True,capture_output=True)
    assert denied.returncode and 'CPU-only' in denied.stderr


def test_parser_bounds_and_abi_rejects_mutants(cases,offline,tmp_path):
    directory,m=cases
    raw=(directory/m['cases']['four-colors-2x2']['fixture']).read_bytes()
    assert len(raw)==48+4*4 and raw[:12]==b'PARENTQv1\0\0\0'
    mutants=[raw[:-1],raw+b'X',b'BAD'+raw[3:]]
    for offset,value in [(12,0),(16,481),(20,0),(28,0),(36,0),(40,0),(40,4097),(44,2)]:
        b=bytearray(raw);struct.pack_into('<I',b,offset,value);mutants.append(b)
    for i,b in enumerate(mutants):
        path=tmp_path/f'mutant-{i}';path.write_bytes(b)
        assert subprocess.run([str(offline),'--offline',str(path)],capture_output=True).returncode


def synthetic_valid(directory,m,name,prefix):
    c=m['cases'][name]
    with np.load(directory/c['expected']) as a:
        for key,suffix in [('labels','.labels.bin'),('records','.records.bin'),('counts','.counts.bin')]:
            prefix.with_suffix(suffix).write_bytes(a[key].astype('<u4').tobytes())
    data=dict(scope='parentprep_only',status='valid',physical_camera=False,objects=False,world_qualified=False,
              fixture_sha256=c['fixture_sha256'],width=c['width'],height=c['height'],calibration_generation=17,configuration_generation=23,
              record_count=c['record_count'],metadata_preserved=True,source_retained=True,source_pool_collision_rejected=True,
              source_fault_rejected=True,pending_source_rejected=True,generation_rejected=True,copied_lease=True,three_slot_retention=True,dropped_source_reaped=True,
              qualifier_source_sha256=m['sources']['tools/performance/qualify_parents.cpp'],
              parent_module_source_sha256=m['sources']['ros2/src/so101_station_vulkan/src/parents.cpp'],shader_source_sha256=m['sources']['ros2/src/so101_station_vulkan/shaders/parents.slang'],external_watchdog_acknowledged=True)
    data['source_metadata']=dict(width=c['width'],height=c['height'],calibration_generation=17,frame_number=91,depth_frame_number=90,timestamp_ns=12345678,serial='procedural-parent-diagnostic-only',hardware_timestamp_ms=123.25,depth_hardware_timestamp_ms=122.5,timestamp_domain=1,depth_timestamp_domain=2)
    prefix.with_suffix('.json').write_text(json.dumps(data))


def test_exact_comparison_and_mutants(cases,tmp_path):
    directory,m=cases;name='four-colors-2x2';prefix=tmp_path/'result'
    synthetic_valid(directory,m,name,prefix)
    assert q.compare(directory,name,prefix)['matched']
    path=prefix.with_suffix('.labels.bin');b=bytearray(path.read_bytes());b[0]^=1;path.write_bytes(b)
    with pytest.raises(ValueError,match='labels'):q.compare(directory,name,prefix)
    synthetic_valid(directory,m,name,prefix)
    path=prefix.with_suffix('.records.bin');a=np.fromfile(path,dtype='<u4');a[0],a[8]=a[8],a[0];path.write_bytes(a.tobytes())
    with pytest.raises(ValueError,match='records'):q.compare(directory,name,prefix)
    synthetic_valid(directory,m,name,prefix)
    path=prefix.with_suffix('.counts.bin');path.write_bytes(path.read_bytes()+b'\0\0\0\0')
    with pytest.raises(ValueError,match='counts'):q.compare(directory,name,prefix)


def test_expected_corruption_and_invalid_output_views(cases,tmp_path):
    directory,m=cases;name='invalid-mask-3';prefix=tmp_path/'negative'
    c=m['cases'][name]
    data=dict(scope='parentprep_only',status='unusable',physical_camera=False,objects=False,world_qualified=False,
              fixture_sha256=c['fixture_sha256'],width=c['width'],height=c['height'],calibration_generation=17,configuration_generation=23,
              qualifier_source_sha256=m['sources']['tools/performance/qualify_parents.cpp'],parent_module_source_sha256=m['sources']['ros2/src/so101_station_vulkan/src/parents.cpp'],shader_source_sha256=m['sources']['ros2/src/so101_station_vulkan/shaders/parents.slang'],external_watchdog_acknowledged=True,
              fault_views_rejected={key:True for key in q.FAULT_VIEWS})
    data['source_metadata']=dict(width=c['width'],height=c['height'],calibration_generation=17,frame_number=91,depth_frame_number=90,timestamp_ns=12345678,serial='procedural-parent-diagnostic-only',hardware_timestamp_ms=123.25,depth_hardware_timestamp_ms=122.5,timestamp_domain=1,depth_timestamp_domain=2)
    prefix.with_suffix('.json').write_text(json.dumps(data))
    assert q.compare(directory,name,prefix)['matched']
    data['fault_views_rejected']['records']=False;prefix.with_suffix('.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):q.compare(directory,name,prefix)
    prefix.with_suffix('.labels.bin').write_bytes(b'')
    with pytest.raises(ValueError):q.compare(directory,name,prefix)


def test_offline_publication_is_exclusive_and_corrupt_expected_fails(cases,offline,tmp_path):
    import shutil
    directory,m=cases
    case=m['cases']['four-colors-2x2'];prefix=tmp_path/'offline'
    subprocess.run([str(offline),'--offline',str(directory/case['fixture']),str(prefix)],check=True,capture_output=True)
    before=prefix.with_suffix('.offline.json').read_bytes()
    assert subprocess.run([str(offline),'--offline',str(directory/case['fixture']),str(prefix)],capture_output=True).returncode
    assert prefix.with_suffix('.offline.json').read_bytes()==before
    copied=tmp_path/'badexpected';shutil.copytree(directory,copied)
    path=copied/case['expected'];data=bytearray(path.read_bytes());data[-1]^=1;path.write_bytes(data)
    with pytest.raises(ValueError,match='hash changed'):q.verify(copied)


def test_native_source_preflight_and_bounded_failure_contract():
    source=(ROOT/'tools/performance/qualify_parents.cpp').read_text()
    assert 'PARENT_QUALIFIER_CPU_ONLY' in source
    assert '10000000000ULL' in source and 'UINT64_MAX' not in source
    assert 'if(pending)fatal(' in source
    assert 'auto* state=new GPUState' in source  # unexpected errors cannot destroy in-flight state
    assert 'std::_Exit' in source and 'real external watchdog acknowledgment required BEFORE Context' in source
    assert 'copy{lease.label_offset_bytes(),0,download.size}' in source
    assert 'lease.resident_buffer()' in source
    for forbidden in ('ParentStorage','parent_graph','cpu_reference(', 'vkDeviceWaitIdle','vkQueueWaitIdle'):
        assert forbidden not in source
