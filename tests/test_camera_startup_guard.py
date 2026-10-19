"""Startup-only repair admission; fixtures never load a camera or SDK."""
from pathlib import Path
import hashlib
import importlib.util
import json
import pytest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT/'ros2/src/so101_station_perception/so101_station_perception/camera_startup.py'


def guard():
    assert MODULE.exists(), 'camera startup repair guard missing'
    spec = importlib.util.spec_from_file_location('camera_startup', MODULE)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def test_missing_repair_blocks_before_loader_probe(tmp_path):
    calls=[]
    with pytest.raises(ValueError, match='repair|provenance'):
        guard().validate_repair(tmp_path/'missing/install', tmp_path/'sdk', loader_probe=lambda *a:calls.append(a))
    assert calls == []


def test_highest_profiles_and_finite_startup_knobs():
    m=guard()
    assert m.camera_parameters('1920,1080,30','1280,720,30','false','1.2','10','2')['enable_sync']=='false'
    for key,value in [('color','99999,1,30'),('depth','1280,720,nan'),('sync','yes'),('wait','0'),('wait','nan'),('retry','0'),('retry','.5')]:
        values=dict(color='1920,1080,30',depth='1280,720,30',sync='false',clip='1.2',wait='10',retry='2');values[key]=value
        with pytest.raises(ValueError):m.camera_parameters(*values.values())


def fixture(tmp_path, monkeypatch):
    m=guard();prefix=tmp_path/'repair/install';lib=prefix/'lib/librealsense2_camera.so';lib.parent.mkdir(parents=True);lib.write_bytes(b'synthetic SO fixture; never loaded')
    sdk=tmp_path/'sdk';sdk_lib=sdk/'lib/librealsense2.so.2.58.3';sdk_lib.parent.mkdir(parents=True);sdk_lib.write_bytes(b'synthetic SDK fixture; never loaded')
    node=sdk/'lib/realsense2_camera/realsense2_camera_node';node.parent.mkdir();node.write_bytes(b'synthetic node; never executed')
    source=prefix.parent/'source'/('realsense-ros-'+m.WRAPPER_COMMIT);source.mkdir(parents=True);(source/'fixture.cpp').write_text('fixture source')
    sources={'fixture.cpp':hashlib.sha256((source/'fixture.cpp').read_bytes()).hexdigest()}
    digest=hashlib.sha256(json.dumps(sources,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    monkeypatch.setattr(m,'SOURCE_DIGEST',digest);monkeypatch.setattr(m,'SOURCE_COUNT',1)
    record={'qualified':False,'mode':'CPU wrapper texture bounds only','wrapper_commit':m.WRAPPER_COMMIT,'builder_sha256':m.BUILDER_SHA,
            'source_final_sha256':digest,'source_file_sha256':sources,
            'sha256':{str(lib):hashlib.sha256(lib.read_bytes()).hexdigest(),str(sdk_lib):hashlib.sha256(sdk_lib.read_bytes()).hexdigest(),
                      '/fixture/realsense-ros-4.58.4-pointcloud-texture-bounds.patch':m.PATCH_SHA,
                      '/fixture/build-realsense-cpu-texture-bounds.sh':m.BUILDER_SHA}}
    cache=prefix.parent/'build-realsense2_camera/CMakeCache.txt';cache.parent.mkdir();cache.write_text('BUILD_ACCELERATE_GPU_WITH_GLSL:BOOL=OFF\n')
    record['sha256'][str(cache)]=hashlib.sha256(cache.read_bytes()).hexdigest()
    manifest=prefix.parent/'build-provenance.json';manifest.write_text(json.dumps(record))
    def probe(executable,environment):return {'wrapper':str(lib),'sdk':str(sdk_lib)}
    return m,prefix,sdk,record,manifest,probe


def test_validated_preload_is_exact_and_scoped_without_mutating_parent(tmp_path,monkeypatch):
    m,prefix,sdk,record,manifest,probe=fixture(tmp_path,monkeypatch)
    parent={'LD_PRELOAD':'/foreign/unverified.so','LD_LIBRARY_PATH':'/fixture/rpath-does-not-win'}
    report=m.validate_repair(prefix,sdk,environment=parent,loader_probe=probe)
    assert report['camera_environment']['LD_PRELOAD']==str(prefix/'lib/librealsense2_camera.so')
    assert parent['LD_PRELOAD']=='/foreign/unverified.so'
    assert report['source_files']==1 and report['qualified'] is False


@pytest.mark.parametrize('mutation',['source','so','sdk','manifest','extra_source','wrong_loader_sdk','wrong_loader_so','cmake_gpu'])
def test_mismatches_fail_before_camera_action(tmp_path,monkeypatch,mutation):
    m,prefix,sdk,record,manifest,probe=fixture(tmp_path,monkeypatch)
    if mutation=='source':next((prefix.parent/'source').rglob('fixture.cpp')).write_text('changed')
    elif mutation=='so':(prefix/'lib/librealsense2_camera.so').write_text('changed')
    elif mutation=='sdk':(sdk/'lib/librealsense2.so.2.58.3').write_text('changed')
    elif mutation=='manifest':record['wrapper_commit']='wrong';manifest.write_text(json.dumps(record))
    elif mutation=='cmake_gpu':
        cache=prefix.parent/'build-realsense2_camera/CMakeCache.txt';cache.write_text('BUILD_ACCELERATE_GPU_WITH_GLSL:BOOL=ON\n');record['sha256'][str(cache)]=hashlib.sha256(cache.read_bytes()).hexdigest();manifest.write_text(json.dumps(record))
    elif mutation=='extra_source':next((prefix.parent/'source').iterdir()).joinpath('extra.cpp').write_text('extra')
    elif mutation.startswith('wrong_loader'):
        field='sdk' if mutation.endswith('sdk') else 'wrapper';base=probe
        def probe(*args):return dict(base(*args),**{field:'/wrong/library.so'})
    with pytest.raises(ValueError):m.validate_repair(prefix,sdk,loader_probe=probe)


def test_startup_guard_has_no_ros_image_callbacks_or_sdk_imports():
    import ast
    tree=ast.parse(MODULE.read_text())
    imports={alias.name for node in ast.walk(tree) if isinstance(node,ast.Import) for alias in node.names}
    assert not imports & {'rclpy','pyrealsense2','serial','numpy','torch'}
    launch=MODULE.parents[1]/'launch/perception.launch.py'
    tree=ast.parse(launch.read_text())
    callers=[node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and any(isinstance(call,ast.Call) and isinstance(call.func,ast.Name) and call.func.id=='validate_repair' for call in ast.walk(node))]
    assert [node.name for node in callers]==['_launch_nodes']
    assert 'realsense-cpu-build)' in (ROOT/'ros2/conda/so101-conda.sh').read_text()
