"""CPU-only original-parent fixtures and exact diagnostic comparison.

Preparation never executes the native qualifier. Expected labels/stats come from
full original one-hot masks through explicit OpenCV GRANA, before area filtering.
Native/GPU outputs cannot generate or overwrite expected artifacts.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import subprocess

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FIXTURE='tests/fixtures/native_geometry/parent-order-v1.json'
MAGIC=b'PARENTQv1\0\0\0'
HEADER=struct.Struct('<12sIIQQIII')
FAULT_VIEWS=('finish','resident_buffer','resident_records_buffer','original_mask_buffer','records')
SOURCES=(FIXTURE,'tools/native_geometry_parent_order.py','tools/qualify_native_parents.py',
         'tools/performance/qualify_parents.cpp',
         'ros2/src/so101_station_vulkan/src/parents.cpp',
         'ros2/src/so101_station_vulkan/src/frame_pool.cpp',
         'ros2/src/so101_station_vulkan/src/vulkan_context.cpp',
         'ros2/src/so101_station_vulkan/include/so101_station_vulkan/parents.hpp',
         'ros2/src/so101_station_vulkan/include/so101_station_vulkan/pipeline.hpp',
         'ros2/src/so101_station_vulkan/shaders/parents.slang',
         'ros2/src/so101_station_vulkan/cmake/parents.cmake')


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def array_hash(a):return hashlib.sha256(np.ascontiguousarray(a,dtype='<u4').tobytes()).hexdigest()


def reference(mask,minimum):
    """Independent exact full-mask per-color CPU oracle, including tiny roots."""
    mask=np.asarray(mask,dtype='<u4');h,w=mask.shape
    labels=np.zeros((h,w),dtype='<u4');counts=np.zeros(4,dtype='<u4');records=[]
    for color in range(4):
        foreground=np.ascontiguousarray(mask==(1<<color),dtype=np.uint8)
        n,local,stats,_=cv2.connectedComponentsWithStatsWithAlgorithm(foreground,8,cv2.CV_32S,cv2.CCL_GRANA)
        counts[color]=n-1
        labels[foreground!=0]=local[foreground!=0]
        for label in range(1,n):
            area=int(stats[label,cv2.CC_STAT_AREA])
            if area<minimum:continue
            yy,xx=np.nonzero(local==label)
            key=int(np.min((yy//2)*((w+1)//2)+xx//2))
            x,y,width,height=(int(stats[label,j]) for j in (cv2.CC_STAT_LEFT,cv2.CC_STAT_TOP,cv2.CC_STAT_WIDTH,cv2.CC_STAT_HEIGHT))
            records.append((color,label,key,area,x,y,width,height))
    return dict(labels=labels,counts=counts,records=np.asarray(records,dtype='<u4').reshape(-1,8))


def scenes(frozen):
    spec=importlib.util.spec_from_file_location('parent_order_cpu_qualifier',ROOT/'tools/native_geometry_parent_order.py')
    oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
    oracle.validate_fixture(frozen)
    result=[(c['id'],oracle.mask_from_rows(c['mask_rows']).astype('<u4'),c['min_component_pixels'],4096,0) for c in frozen['cases']]
    result.extend([
        ('four-colors-2x2',np.array([[1,2],[4,8]],dtype='<u4'),1,4096,0),
        ('false-occupied-block-edge',np.array([[1,0,0,1]],dtype='<u4'),1,4096,0),
        ('diagonal-edge',np.array([[0,1,0,0],[0,0,1,0]],dtype='<u4'),1,4096,0),
        ('large-all-red-640x480',np.ones((480,640),dtype='<u4'),30,4096,0),
        ('isolated-four-colors-64x64',np.tile(np.array([[1,2],[4,8]],dtype='<u4'),(32,32)),1,4096,0),
        ('threshold-default-2x2',np.ones((2,2),dtype='<u4'),30,4096,0),
        ('threshold-uintmax-2x2',np.ones((2,2),dtype='<u4'),0xffffffff,4096,0),
        ('invalid-mask-3',np.array([[3]],dtype='<u4'),1,4096,1),
        ('overflow-two-parents-cap1',np.array([[1,0,1]],dtype='<u4'),1,1,1)])
    return result


def encode(mask,minimum=30,capacity=4096,flags=0):
    mask=np.asarray(mask)
    if mask.ndim!=2:raise ValueError('mask dimensions')
    h,w=mask.shape
    if not(1<=w<=640 and 1<=h<=480 and type(minimum) is int and 1<=minimum<=0xffffffff and type(capacity) is int and 1<=capacity<=4096 and type(flags) is int and flags in (0,1)):
        raise ValueError('fixture configuration')
    if not np.issubdtype(mask.dtype,np.integer) or np.any(mask<0) or np.any(mask>0xffffffff):raise ValueError('mask must contain U32 values')
    if not flags and not np.all(np.isin(mask,[0,1,2,4,8])):raise ValueError('unsupported unflagged mask')
    return HEADER.pack(MAGIC,w,h,17,23,minimum,capacity,flags)+np.ascontiguousarray(mask,dtype='<u4').tobytes()


def prepare(directory):
    committed=subprocess.check_output(['git','-C',str(ROOT),'show','HEAD:'+FIXTURE])
    if (ROOT/FIXTURE).read_bytes()!=committed:raise ValueError('parent-order input differs from committed fixture')
    frozen=json.loads(committed);cases=scenes(frozen)
    sources={p:digest(ROOT/p) for p in SOURCES}
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    manifest=dict(schema='native-parent-diagnostic-v1',scope='parentprep_only',physical_camera=False,objects=False,world_qualified=False,
                  gpu_expected_from_native=False,opencv=cv2.__version__,numpy=np.__version__,sources=sources,cases={})
    for name,mask,minimum,capacity,flags in cases:
        expected=reference(mask,minimum)
        invalid=not np.all(np.isin(mask,[0,1,2,4,8]))
        assert bool(flags)==bool(invalid or len(expected['records'])>capacity)
        binary=directory/(name+'.parentq');arrayfile=directory/(name+'.npz')
        with binary.open('xb') as stream:stream.write(encode(mask,minimum,capacity,flags))
        with arrayfile.open('xb') as stream:np.savez_compressed(stream,**expected,mask=np.asarray(mask,dtype='<u4'))
        h,w=mask.shape
        manifest['cases'][name]=dict(fixture=binary.name,expected=arrayfile.name,fixture_sha256=digest(binary),expected_sha256=digest(arrayfile),
            array_sha256={k:array_hash(a) for k,a in expected.items()},width=w,height=h,minimum=minimum,capacity=capacity,
            expected_unusable=bool(flags),record_count=len(expected['records']),calibration_generation=17,configuration_generation=23)
    with (directory/'manifest.json').open('x') as stream:json.dump(manifest,stream,indent=2);stream.write('\n')
    return manifest


def verify(directory):
    directory=Path(directory);m=json.loads((directory/'manifest.json').read_text())
    if m.get('schema')!='native-parent-diagnostic-v1' or m.get('gpu_expected_from_native') is not False:raise ValueError('fixture schema')
    for c in m['cases'].values():
        for key,hashkey in [('fixture','fixture_sha256'),('expected','expected_sha256')]:
            if Path(c[key]).name!=c[key] or digest(directory/c[key])!=c[hashkey]:raise ValueError('fixture/expected hash changed')
        with np.load(directory/c['expected'],allow_pickle=False) as a:
            shapes={'labels':(c['height'],c['width']),'records':(c['record_count'],8),'counts':(4,)}
            for key in ('labels','records','counts'):
                if a[key].shape!=shapes[key]:raise ValueError('expected array shape changed')
                if a[key].dtype!=np.dtype('<u4') or array_hash(a[key])!=c['array_sha256'][key]:raise ValueError('expected array changed')
    return m


def compare(directory,name,prefix):
    directory=Path(directory);m=verify(directory);c=m['cases'][name];prefix=Path(prefix)
    artifact=lambda suffix:Path(str(prefix)+suffix)
    result=json.loads(artifact('.json').read_text())
    for key,value in [('scope','parentprep_only'),('physical_camera',False),('objects',False),('world_qualified',False),
                      ('fixture_sha256',c['fixture_sha256']),('width',c['width']),('height',c['height']),
                      ('calibration_generation',17),('configuration_generation',23),
                      ('qualifier_source_sha256',m['sources']['tools/performance/qualify_parents.cpp']),
                      ('parent_module_source_sha256',m['sources']['ros2/src/so101_station_vulkan/src/parents.cpp']),
                      ('shader_source_sha256',m['sources']['ros2/src/so101_station_vulkan/shaders/parents.slang']),
                      ('external_watchdog_acknowledged',True)]:
        if type(result.get(key)) is not type(value) or result[key]!=value:raise ValueError('metadata/build provenance mismatch: '+key)
    metadata=dict(width=c['width'],height=c['height'],calibration_generation=17,frame_number=91,depth_frame_number=90,timestamp_ns=12345678,serial='procedural-parent-diagnostic-only',hardware_timestamp_ms=123.25,depth_hardware_timestamp_ms=122.5,timestamp_domain=1,depth_timestamp_domain=2)
    if result.get('source_metadata')!=metadata:raise ValueError('source metadata changed')
    if c['expected_unusable']:
        if result.get('status')!='unusable' or any(result.get('fault_views_rejected',{}).get(k) is not True for k in FAULT_VIEWS):raise ValueError('negative result exposed usable views')
        if any(artifact(s).exists() for s in ('.labels.bin','.records.bin','.counts.bin')):raise ValueError('unusable output published arrays')
    else:
        if result.get('status')!='valid' or result.get('record_count')!=c['record_count']:raise ValueError('status/count differs')
        for key in ('metadata_preserved','source_retained','source_pool_collision_rejected','source_fault_rejected','pending_source_rejected','generation_rejected','copied_lease','three_slot_retention','dropped_source_reaped'):
            if result.get(key) is not True:raise ValueError('lifecycle contract missing: '+key)
        with np.load(directory/c['expected'],allow_pickle=False) as a:
            for key,suffix in [('labels','.labels.bin'),('records','.records.bin'),('counts','.counts.bin')]:
                payload=artifact(suffix).read_bytes();expected=a[key].astype('<u4').tobytes()
                if payload!=expected:raise ValueError('exact '+key+' differs')
    return dict(matched=True,case=name,status=result['status'],scope='parentprep_only',physical_camera=False,objects=False,world_qualified=False,
                output_json_sha256=digest(artifact('.json')))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('prepare');p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('verify');p.add_argument('directory',type=Path)
    p=sub.add_parser('compare');p.add_argument('directory',type=Path);p.add_argument('case');p.add_argument('prefix',type=Path)
    args=parser.parse_args(argv)
    value=prepare(args.output) if args.mode=='prepare' else verify(args.directory) if args.mode=='verify' else compare(args.directory,args.case,args.prefix)
    print(json.dumps(value,indent=2))
if __name__=='__main__':main()
