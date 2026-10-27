"""Exclusive CPU child fixtures and exact diagnostic comparison; never runs a GPU.

Literal expectations are losslessly decoded committed CPU evidence, not a new
native model. Supplemental cases call the retained CPU splitter and GRANA only.
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
FIXTURE='tests/fixtures/native_geometry/child-evidence-v1.json'
FROZEN_SHA='00f5d69f2a297ff876ce1848fdfaa42abebf5dbe503284b33258de052e1a9ab0'
MAGIC=b'CHILDQv1\0\0\0\0'
HEADER=struct.Struct('<12s29I')
ARRAYS=('roots','flags','records','statuses','packed','counts')
VIEWS=('finish','parent','source_root_buffer','source_flags_offset_bytes','packed_flags_buffer','resident_records_buffer','records','parent_status')
SOURCES=(FIXTURE,'tools/native_geometry_child_evidence.py','tools/native_geometry_parent_order.py','tools/qualify_native_children.py','tools/performance/qualify_children.cpp',
 'ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py',
 *('ros2/src/so101_station_vulkan/'+p for p in ('include/so101_station_vulkan/children.hpp','include/so101_station_vulkan/child_graph.hpp','include/so101_station_vulkan/parents.hpp','include/so101_station_vulkan/pipeline.hpp','src/children.cpp','src/parents.cpp','src/frame_pool.cpp','src/vulkan_context.cpp','shaders/children.slang','shaders/parents.slang','cmake/children.cmake','cmake/parents.cmake')))
def digest(p):
    result=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(65536),b''):result.update(chunk)
    return result.hexdigest()
def ah(a):return hashlib.sha256(np.asarray(a,dtype='<u4').tobytes()).hexdigest()
def oracle():
    spec=importlib.util.spec_from_file_location('frozen_child_cpu_oracle',ROOT/'tools/native_geometry_child_evidence.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
def unpack(a):
    return np.array([row for count,row in a['row_runs'] for _ in range(count)],dtype={'bool':bool,'uint32':'<u4','int32':'<i4'}[a['dtype']]).reshape(a['shape'])
def u64bits(v):return struct.unpack('<Q',struct.pack('<d',v))[0]
def configuration(w,h,profile,minimum=8,P=4096,C=4096,S=0,maximum=1.2,jump=.012,flags=0):
    return dict(width=w,height=h,profile=profile,minimum=minimum,parent_minimum=1,parent_capacity=P,capacity=C,packed_capacity=S or 4*w*h,maximum=maximum,jump=jump,flags=flags)
def parent_oracle(mask):
    h,w=mask.shape;labels=np.zeros_like(mask,dtype='<u4');records=[]
    for color in range(4):
        fg=np.asarray(mask==1<<color,dtype=np.uint8)
        n,local,stats,_=cv2.connectedComponentsWithStatsWithAlgorithm(fg,8,cv2.CV_32S,cv2.CCL_GRANA)
        labels[fg!=0]=local[fg!=0]
        for lab in range(1,n):
            yy,xx=np.where(local==lab);x,y,bw,bh,area=map(int,stats[lab]);records.append([color,lab,int(((yy//2)*((w+1)//2)+xx//2).min()),area,x,y,bw,bh])
    return labels,np.asarray(records,dtype='<u4').reshape(-1,8)
def expected_parts(cfg,mask,depth,parent_records,evidences):
    h,w=mask.shape;roots=np.zeros_like(mask,dtype='<u4');z=depth.view('<f4')
    valid=np.isfinite(z)&(z>=np.float32(.1))&(z<=np.float32(cfg['maximum']))
    flags=valid.astype('<u4');statuses=[];records=[];payload=[];offset=0;jobs=0
    for index,(par,e) in enumerate(zip(parent_records,evidences)):
        _,_,_,area,x,y,bw,bh=map(int,par);labels=unpack(e['labels']);parent=unpack(e['parent']);cut=unpack(e['cut']);candidate=unpack(e['candidate'])
        flags[y:y+bh,x:x+bw]|=cut.astype('<u4')*2
        for c in e['components']:
            member=labels==c['label_id'];yy,xx=np.where(member);root=int(((yy+y)*w+xx+x).min())+1
            roots[y:y+bh,x:x+bw][member]=root
        statuses.append([int((valid[y:y+bh,x:x+bw]&parent).sum()),int(candidate.sum()),len(e['components']),len(e['accepted'])])
        for piece in e['accepted']:
            comp=next(c for c in e['components'] if c['label_id']==piece['label_id']);left,top,rw,rh=piece['local_bounds_xywh'];region=unpack(piece['region']);support=unpack(piece['support']);nearby=cv2.dilate(region.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
            own=parent[top:top+rh,left:left+rw];localcut=cut[top:top+rh,left:left+rw]&own
            packed=(region.astype('<u4')|own.astype('<u4')*2|nearby.astype('<u4')*4|localcut.astype('<u4')*8|support.astype('<u4')*16)
            member=labels==comp['label_id'];yy,xx=np.where(member);root=int(((yy+y)*w+xx+x).min())+1
            records.append([root,index,comp['min_crop_block_key'],comp['area'],*comp['bounds_xywh'],left,top,rw,rh,offset,piece['depth_cut_pixels'],int(support.sum()),0]);payload.extend(packed.ravel());offset+=rw*rh;jobs+=(rw*rh+255)//256
    return dict(roots=roots,flags=flags,records=np.asarray(records,dtype='<u4').reshape(-1,16),statuses=np.asarray(statuses,dtype='<u4').reshape(-1,4),packed=np.asarray(payload,dtype='<u4'),counts=np.array([len(parent_records),len(records),offset,jobs],dtype='<u4'))
def literal(case,translation=(0,0)):
    e=case['expected'];x,y,bw,bh=e['tight_global_bbox_xywh'];x+=translation[0];y+=translation[1];w,h=x+bw+1,y+bh+1
    mask=np.zeros((h,w),dtype='<u4');depth=np.full((h,w),np.float32(.5).view(np.uint32),dtype='<u4');mask[y:y+bh,x:x+bw]=unpack(e['parent']);depth[y:y+bh,x:x+bw]=unpack(e['depth_bits'])
    yy,xx=np.where(mask!=0);key=int(((yy//2)*((w+1)//2)+xx//2).min());pr=np.array([[0,1,key,int(mask.sum()),x,y,bw,bh]],dtype='<u4');labels=mask.copy();c=case['input'];cfg=configuration(w,h,'literal_kernel',e['effective_min_pixels'],maximum=c['max_observation_depth_m'],jump=c['max_neighbor_jump_m'])
    return cfg,mask,depth,labels,pr,expected_parts(cfg,mask,depth,pr,[e])
def supplemental(cfg,mask,depth):
    labels,pr=parent_oracle(mask);o=oracle();evidences=[]
    for p in pr:
        color,lab,_,_,x,y,w,h=map(int,p);parent=(mask[y:y+h,x:x+w]==1<<color)&(labels[y:y+h,x:x+w]==lab);bits=depth[y:y+h,x:x+w];symbols={chr(65+j):int(v) for j,v in enumerate(np.unique(bits))};inv={v:k for k,v in symbols.items()};rows=[''.join(inv[int(v)] for v in row) for row in bits]
        case={'input':{'original_parent':o.pack_array(parent),'pixel_origin_xy':[x,y],'initial_input_crop_xywh':[x,y,w,h],'depth_symbols_u32':symbols,'depth_symbol_row_runs':[[1,row] for row in rows],'min_pixels':cfg['minimum'],'caller_parent_min_pixels':None,'max_observation_depth_m':cfg['maximum'],'max_neighbor_jump_m':cfg['jump']}}
        evidences.append(o.trace_case(case))
    return cfg,mask,depth,labels,pr,expected_parts(cfg,mask,depth,pr,evidences)
def scenes(frozen):
    scenes=[]
    for case in frozen['cases']:
        value=literal(case);scenes.append(('literal-'+case['id'],value))
        cfg,mask,depth,labels,pr,expected=value
        if cfg['width']<=640 and cfg['height']<=480:
            actual,parents=parent_oracle(mask)
            if np.array_equal(parents,pr) and np.array_equal(actual,labels):
                typed=cfg.copy();typed['profile']='typed_parent';scenes.append(('typed-'+case['id'],(typed,mask,depth,labels,pr,expected)))
    case=next(c for c in frozen['cases'] if c['id']=='u-membership')
    for dx,dy in ((0,0),(1,0),(0,1),(1,1)):scenes.append((f'literal-U-parity-{dx}-{dy}',literal(case,(dx,dy))))
    for name,P,C in [('two-parents-P2C1-empty',2,1),('two-colors-boundary',2,8),('one-parent-P1C2',1,2)]:
        mask=np.ones((8,20),dtype='<u4');depth=np.full(mask.shape,np.float32(.5).view(np.uint32),dtype='<u4')
        if name.startswith('two-parents'):mask[:,9:11]=0;depth[:]=0
        elif name.startswith('two-colors'):mask[:,10:]=2;depth[:,10:]=np.float32(.6).view(np.uint32)
        else:depth[:,8:12]=0
        cfg=configuration(20,8,'typed_parent',P=P,C=C);scenes.append((name,supplemental(cfg,mask,depth)))
    mask=np.ones((12,20),dtype='<u4');mask[2:-2,2:-2]=2;depth=np.full(mask.shape,np.float32(.5).view(np.uint32),dtype='<u4');depth[2:-2,10:-2]=np.float32(.6).view(np.uint32)
    scenes.append(('overlapping-parent-cut-halos',supplemental(configuration(20,12,'typed_parent',P=2,C=8),mask,depth)))
    original=next(v for n,v in scenes if n=='one-parent-P1C2')
    for name,change in [('record-overflow',{'capacity':1}),('packed-overflow',{'packed_capacity':1})]:
        cfg,*rest=original;cfg={**cfg,**change,'flags':1};scenes.append((name,(cfg,*rest)))
    cfg,mask,depth,labels,pr,e=literal(frozen['cases'][0]);cfg={**cfg,'flags':1};labels=labels.copy();labels.flat[int(np.flatnonzero(mask)[0])]=0
    scenes.append(('literal-invalid-parent-label',(cfg,mask,depth,labels,pr,e)))
    return scenes

def encode(cfg,mask,depth,labels,pr):
    w,h=cfg['width'],cfg['height'];P,C,S=cfg['parent_capacity'],cfg['capacity'],cfg['packed_capacity'];maximum,jump=cfg['maximum'],cfg['jump']
    if not(1<=w<=2048 and 1<=h<=2048 and w*h<=307200 and cfg['profile'] in ('typed_parent','literal_kernel') and 1<=P<=4096 and 1<=C<=4096 and 1<=S<=4*w*h and 8<=cfg['minimum']<=0xffffffff and cfg['flags'] in (0,1) and np.isfinite(maximum) and maximum>.1 and np.isfinite(jump) and jump>0 and len(pr)<=P):raise ValueError('configuration bounds')
    if cfg['profile']=='typed_parent' and (w>640 or h>480):raise ValueError('typed parent dimensions')
    for a in (mask,depth,labels):
        if a.shape!=(h,w) or a.dtype!=np.dtype('<u4'):raise ValueError('input U32 shape')
    if pr.shape!=(len(pr),8) or pr.dtype!=np.dtype('<u4'):raise ValueError('parent records ABI')
    if not cfg['flags'] and not np.isin(mask,[0,1,2,4,8]).all():raise ValueError('mask values')
    mb,jb=map(u64bits,(maximum,jump));words=[w,h,1 if cfg['profile']=='typed_parent' else 2,cfg['flags'],P,C,S,cfg['minimum'],cfg['parent_minimum'],17,0,23,0,29,0,mb&0xffffffff,mb>>32,jb&0xffffffff,jb>>32,len(pr),w*h,*([0]*8)]
    return HEADER.pack(MAGIC,*words)+b''.join(np.ascontiguousarray(a,dtype='<u4').tobytes() for a in (mask,depth,labels,pr))
def shader_manifest(directory,child):
    directory=Path(directory);names=('child_lookup child_init child_hook child_jump child_validate child_reduce child_gather child_radix_hist child_radix_offsets child_radix_scatter child_scatter child_pack_scan child_pack_offsets child_jobs child_packed_init child_fill_hook child_fill_jump child_fill_validate child_boundary child_support child_finalize' if child else 'parents_init parents_hook parents_flatten parents_validate parents_scan parents_offsets parents_pixels parents_labels parents_records').split()
    source=(directory/'source.sha256').read_text().strip();expected=digest(ROOT/('ros2/src/so101_station_vulkan/shaders/children.slang' if child else 'ros2/src/so101_station_vulkan/shaders/parents.slang'))
    if source!=expected:raise ValueError('shader source identity')
    provenance=(directory/'provenance.txt').read_text()
    if 'source_sha256='+source+'\n' not in provenance:raise ValueError('shader provenance')
    result=dict(source_sha256=source,provenance=provenance,provenance_sha256=digest(directory/'provenance.txt'),spirv={})
    for name in names:
        f=name+'.spv';sha=digest(directory/f)
        if (directory/(f+'.sha256')).read_text()!=sha+'  '+f+'\n':raise ValueError('SPV digest')
        result['spirv'][f]=sha
    return result

def prepare(directory,child_shaders=None,parent_shaders=None):
    committed=subprocess.check_output(['git','-C',str(ROOT),'show','HEAD:'+FIXTURE]);assert hashlib.sha256(committed).hexdigest()==FROZEN_SHA and (ROOT/FIXTURE).read_bytes()==committed,'committed fixture changed'
    frozen=json.loads(committed);assert len(frozen['cases'])==24
    source={p:digest(ROOT/p) for p in SOURCES};shaders={}
    if child_shaders is not None:shaders['child_shader_identity']=shader_manifest(child_shaders,True)
    if parent_shaders is not None:shaders['parent_shader_identity']=shader_manifest(parent_shaders,False)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    manifest=dict(schema='native-child-diagnostic-v1',scope='childprep_only',gpu_expected_from_native=False,physical_camera=False,objects=False,world_qualified=False,numpy=np.__version__,opencv=cv2.__version__,sources=source,shader_artifacts=shaders,cases={})
    for name,(cfg,mask,depth,labels,pr,expected) in scenes(frozen):
        fixture=directory/(name+'.childq');ep=directory/(name+'.npz')
        with fixture.open('xb') as f:f.write(encode(cfg,mask,depth,labels,pr))
        empty={k:np.zeros_like(expected[k]) for k in ('roots','flags')};empty['flags']=expected['flags']&1;empty.update(records=np.empty((0,16),dtype='<u4'),statuses=np.empty((0,4),dtype='<u4'),packed=np.empty(0,dtype='<u4'),counts=np.zeros(4,dtype='<u4'))
        with ep.open('xb') as f:np.savez_compressed(f,**expected,**{'empty_'+k:v for k,v in empty.items()},mask=mask,depth=depth,parent_labels=labels,parent_records=pr)
        manifest['cases'][name]=dict(**cfg,fixture=fixture.name,fixture_sha256=digest(fixture),expected=ep.name,expected_sha256=digest(ep),array_sha256={k:ah(v) for k,v in expected.items()},empty_sha256={k:ah(v) for k,v in empty.items()},parent_count=len(pr),expected_unusable=bool(cfg['flags']))
    with (directory/'manifest.json').open('x') as f:json.dump(manifest,f,indent=2);f.write('\n')
    return manifest
def json_document(path):
    path=Path(path)
    if not 0<path.stat().st_size<=1048576:raise ValueError('JSON document bound')
    return json.loads(path.read_text())

def verify(directory):
    directory=Path(directory);m=json_document(directory/'manifest.json')
    if m.get('schema')!='native-child-diagnostic-v1' or m.get('gpu_expected_from_native') is not False:raise ValueError('schema')
    for c in m['cases'].values():
        for key in ('fixture','expected'):
            if Path(c[key]).name!=c[key] or digest(directory/c[key])!=c[key+'_sha256']:raise ValueError('artifact hash')
        with np.load(directory/c['expected'],allow_pickle=False) as e:
            for k in ARRAYS:
                if e[k].dtype!=np.dtype('<u4') or ah(e[k])!=c['array_sha256'][k] or ah(e['empty_'+k])!=c['empty_sha256'][k]:raise ValueError('expected array changed')
    return m
def compare(directory,name,prefix):
    directory=Path(directory);m=verify(directory);c=m['cases'][name];artifact=lambda suffix:Path(str(prefix)+suffix);r=json_document(artifact('.json'))
    fields={'scope':'childprep_only','physical_camera':False,'objects':False,'world_qualified':False,'profile':c['profile'],'fixture_sha256':c['fixture_sha256'],'width':c['width'],'height':c['height'],'calibration_generation':17,'parent_generation':23,'configuration_generation':29,'external_watchdog_acknowledged':True,'qualifier_source_sha256':m['sources']['tools/performance/qualify_children.cpp'],'child_module_source_sha256':m['sources']['ros2/src/so101_station_vulkan/src/children.cpp'],'shader_source_sha256':m['sources']['ros2/src/so101_station_vulkan/shaders/children.slang']}
    for k,v in fields.items():
        if type(r.get(k)) is not type(v) or r[k]!=v:raise ValueError('metadata/provenance '+k)
    if r.get('production_views_used') is not (c['profile']=='typed_parent'):raise ValueError('production profile boundary')
    if c['profile']=='literal_kernel' and ('parent_shader_identity' not in r or r['parent_shader_identity'] is not None):raise ValueError('literal parent_shader_identity must be null')
    for k,v in m.get('shader_artifacts',{}).items():
        if k=='parent_shader_identity' and c['profile']=='literal_kernel':continue
        if r.get(k)!=v:raise ValueError('shader build identity '+k)
    if c['expected_unusable']:
        if r.get('status')!='unusable':raise ValueError('expected unusable')
        if c['profile']=='typed_parent' and any(r.get('fault_views_rejected',{}).get(k) is not True for k in VIEWS):raise ValueError('fault views')
        if c['profile']=='literal_kernel' and (not r.get('fault') or r.get('production_views_used') is not False):raise ValueError('literal fault boundary')
        if any(artifact('.'+k+'.bin').exists() for k in ARRAYS):raise ValueError('fault published arrays')
    else:
        if r.get('status')!='valid':raise ValueError('expected valid')
        cycles=[('', '')] if c['profile']=='literal_kernel' else [('', ''),('.empty','empty_'),('.restore','')]
        with np.load(directory/c['expected'],allow_pickle=False) as e:
            for suffix,ek in cycles:
                for k in ARRAYS:
                    output=artifact(suffix+'.'+k+'.bin');expected=e[ek+k]
                    if output.stat().st_size!=expected.nbytes:raise ValueError('exact '+suffix+k+' size differs')
                    if output.read_bytes()!=expected.tobytes():raise ValueError('exact '+suffix+k+' differs')
        if c['profile']=='typed_parent':
            for k in ('metadata_preserved','source_retained','source_pool_collision_rejected','generation_rejected','source_fault_rejected','pending_source_rejected','copied_lease','three_slot_retention','dropped_source_reaped','reuse_empty_restore'):
                if r.get(k) is not True:raise ValueError('typed lifecycle '+k)
            if r.get('frame_numbers')!=[91,92,93]:raise ValueError('source IDs')
    return dict(matched=True,case=name,profile=c['profile'],status=r['status'],output_json_sha256=digest(artifact('.json')))
def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='mode',required=True);a=s.add_parser('prepare');a.add_argument('--output',type=Path,required=True);a.add_argument('--child-shaders',type=Path);a.add_argument('--parent-shaders',type=Path);a=s.add_parser('verify');a.add_argument('directory',type=Path);a=s.add_parser('compare');a.add_argument('directory',type=Path);a.add_argument('case');a.add_argument('prefix',type=Path);a=p.parse_args(argv);v=prepare(a.output,a.child_shaders,a.parent_shaders) if a.mode=='prepare' else verify(a.directory) if a.mode=='verify' else compare(a.directory,a.case,a.prefix);print(json.dumps(v,indent=2))
if __name__=='__main__':main()
