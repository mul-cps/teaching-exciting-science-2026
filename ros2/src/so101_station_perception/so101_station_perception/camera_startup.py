"""One-shot private CPU repair validation. Never called from ROS/image callbacks."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

WRAPPER_COMMIT = '9a11121700cb4780e273e34141f6402fe184321d'
SOURCE_DIGEST = 'c5a1acf2488fa77f23ee2d5a84af3a0c3cd34c2a86b6b088bfd514c9fb1f3e7e'
SOURCE_COUNT = 201
BUILDER_SHA = 'bca7cb55fad342792c7738916b4115638df8fa1db209df251a7d3043ddd9653e'
PATCH_SHA = '008b46ff14fcb1c999a28935101608a18b8d41fafbd1bd4a866716bf262e3476'
MAX_FILE_BYTES = 64*1024*1024
MAX_SOURCE_BYTES = 128*1024*1024


def file_digest(path):
    path = Path(path)
    if not path.is_file() or path.is_symlink() or not 0 <= path.stat().st_size <= MAX_FILE_BYTES:
        raise ValueError('repair requires bounded regular files: '+str(path))
    digest = hashlib.sha256();count = 0
    with path.open('rb') as stream:
        while block := stream.read(1024*1024):
            count += len(block)
            if count > MAX_FILE_BYTES:raise ValueError('repair file grew beyond bound')
            digest.update(block)
    return digest.hexdigest(),count


def camera_parameters(color, depth, sync, clip, wait, retry):
    def profile(value, width, height):
        if not re.fullmatch(r'[0-9]{1,4},[0-9]{1,4},[0-9]{1,3}', value):
            raise ValueError('camera profile must be width,height,fps')
        w,h,fps=map(int,value.split(','))
        if not 1 <= w <= width or not 1 <= h <= height or not 1 <= fps <= 90:
            raise ValueError('camera profile exceeds D435i supported envelope; SDK must verify exact mode')
        return value
    if sync not in ('true','false'):raise ValueError('enable_sync must be true or false')
    clip,wait,retry=map(float,(clip,wait,retry))
    if (not all(math.isfinite(v) for v in (clip,wait,retry)) or not clip>.1
            or not 1<=wait<=120 or not 1<=retry<=10 or retry!=int(retry)):
        raise ValueError('finite clip>0.1m, wait1..120s and integer reconnect1..10s required')
    return {'rgb_camera.color_profile':profile(color,1920,1080),
            'depth_module.depth_profile':profile(depth,1280,720),'enable_sync':sync,
            'clip_distance':str(clip),'wait_for_device_timeout':str(wait),'reconnect_timeout':str(retry)}


def loader_resolution(executable, environment):
    """Trace ELF dependency resolution only: no entrypoint/SDK initializers run."""
    executable=Path(executable)
    # Host x86_64 ELF loader used by the qualified Jazzy/AMD workstation.
    loader=Path('/lib64/ld-linux-x86-64.so.2')
    if not loader.is_file() or not executable.is_file():raise ValueError('camera ELF loader/executable unavailable')
    result=subprocess.run([str(loader),'--list',str(executable)],env=environment,
                          stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=5.,check=False)
    if result.returncode or len(result.stdout)>262144 or len(result.stderr)>16384:
        raise ValueError('camera dependency loader trace failed: '+result.stderr[:512])
    wrapper=[];sdk=[]
    for line in result.stdout.splitlines():
        match=re.search(r'(?:=>\s+)?(/\S+)\s+\(',line)
        if not match:continue
        path=Path(match.group(1))
        if path.name=='librealsense2_camera.so':wrapper.append(str(path.resolve()))
        elif re.fullmatch(r'librealsense2\.so(?:\.[0-9]+)*',path.name):sdk.append(str(path.resolve()))
    if len(wrapper)!=1 or len(sdk)!=1:raise ValueError('camera loader must select exactly one wrapper and CPU SDK')
    return {'wrapper':wrapper[0],'sdk':sdk[0]}


def validate_repair(prefix, sdk_prefix, *, environment=None, loader_probe=loader_resolution):
    """Hash the exact201-source build and loader-selected SO/SDK once at launch."""
    started=time.monotonic();prefix=Path(prefix).expanduser().resolve();sdk_prefix=Path(sdk_prefix).expanduser().resolve()
    manifest=prefix.parent/'build-provenance.json'
    try:
        if not manifest.is_file() or manifest.stat().st_size>262144:raise ValueError('CPU repair provenance missing/oversized; run explicit realsense-cpu-build action')
        raw=manifest.read_bytes();record=json.loads(raw)
        sources=record['source_file_sha256'];hashes=record['sha256']
        if (record.get('mode')!='CPU wrapper texture bounds only' or record.get('wrapper_commit')!=WRAPPER_COMMIT
                or record.get('builder_sha256')!=BUILDER_SHA or record.get('source_final_sha256')!=SOURCE_DIGEST
                or len(sources)!=SOURCE_COUNT
                or hashlib.sha256(json.dumps(sources,sort_keys=True,separators=(',',':')).encode()).hexdigest()!=SOURCE_DIGEST):
            raise ValueError('CPU repair provenance source/builder pins mismatch')
        for name,expected in (('realsense-ros-4.58.4-pointcloud-texture-bounds.patch',PATCH_SHA),
                              ('build-realsense-cpu-texture-bounds.sh',BUILDER_SHA)):
            values=[value for key,value in hashes.items() if Path(key).name==name]
            if values!=[expected]:raise ValueError('CPU repair patch/builder artifact pin mismatch')
        source=prefix.parent/'source'/('realsense-ros-'+WRAPPER_COMMIT)
        inventory=set();entries=0
        for directory,dirs,files in os.walk(source,followlinks=False):
            entries+=len(dirs)+len(files)
            if entries>1024:raise ValueError('CPU repair source inventory exceeds bound')
            if any((Path(directory)/name).is_symlink() for name in dirs+files):raise ValueError('CPU repair source symlink rejected')
            inventory.update(str((Path(directory)/name).relative_to(source)) for name in files)
        if inventory!=set(sources):raise ValueError('CPU repair source inventory mismatch')
        total=0
        for name,expected in sources.items():
            path=Path(name)
            if path.is_absolute() or '..' in path.parts:raise ValueError('CPU repair source path invalid')
            actual,count=file_digest(source/path);total+=count
            if total>MAX_SOURCE_BYTES or actual!=expected:raise ValueError('CPU repair source hash/size mismatch')
        cache=prefix.parent/'build-realsense2_camera/CMakeCache.txt'
        if file_digest(cache)[0]!=hashes.get(str(cache)) or not re.search(r'^BUILD_ACCELERATE_GPU_WITH_GLSL:(?:BOOL|UNINITIALIZED)=OFF$',cache.read_text(),re.MULTILINE):
            raise ValueError('CPU repair CMake build flags/hash mismatch')
        library=prefix/'lib/librealsense2_camera.so';sdk=sdk_prefix/'lib/librealsense2.so.2.58.3'
        for path in (library,sdk):
            if file_digest(path)[0]!=hashes.get(str(path)):raise ValueError('CPU repair wrapper/SDK hash mismatch: '+str(path))
        child_environment=dict(os.environ if environment is None else environment)
        child_environment['LD_PRELOAD']=str(library)
        resolution=loader_probe(sdk_prefix/'lib/realsense2_camera/realsense2_camera_node',child_environment)
        if resolution!={'wrapper':str(library),'sdk':str(sdk)}:
            raise ValueError('actual child loader selected a different CPU wrapper/SDK')
        # Detect changed artifacts during the trace, before admitting any camera action.
        if manifest.read_bytes()!=raw or any(file_digest(path)[0]!=hashes[str(path)] for path in (library,sdk)):
            raise ValueError('CPU repair artifacts changed during startup validation')
        return {'qualified':False,'repair_validated':True,'camera_environment':{'LD_PRELOAD':str(library)},
                'loader_resolution':resolution,'validated_executable':str(sdk_prefix/'lib/realsense2_camera/realsense2_camera_node'),
                'wrapper_sha256':hashes[str(library)],'sdk_sha256':hashes[str(sdk)],
                'cmake_cache_sha256':hashes[str(cache)],'builder_sha256':BUILDER_SHA,
                'source_files':len(sources),'source_bytes':total,
                'source_final_sha256':SOURCE_DIGEST,'manifest_sha256':hashlib.sha256(raw).hexdigest(),
                'validation_ms':(time.monotonic()-started)*1000}
    except (OSError,KeyError,TypeError,json.JSONDecodeError,subprocess.SubprocessError) as error:
        raise ValueError('CPU camera repair unavailable: '+str(error)) from error


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--sdk-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args=parser.parse_args()
    report=validate_repair(args.prefix,args.sdk_prefix)
    if args.output:
        with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))
