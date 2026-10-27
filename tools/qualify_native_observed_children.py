"""CPU-only retained oracle and exact comparison for explicit Observed Vulkan diagnostics.

Never invokes a GPU. The native executable runs only under a separately provided
process-group watchdog. Every invocation uses one original procedural cohort;
its compact timing and optional dense diagnostic timing are separate.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('child_diagnostic',ROOT/'tools/qualify_native_children.py')
q=importlib.util.module_from_spec(spec);spec.loader.exec_module(q)
SOURCES=('tools/qualify_native_observed_children.py','tools/qualify_native_children.py','tools/native_geometry_child_evidence.py','tools/performance/qualify_children.cpp','ros2/src/so101_station_vulkan/test/qualify_observed_children.cpp','ros2/src/so101_station_objects/so101_station_objects/objects.py','ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py','ros2/src/so101_station_vulkan/shaders/geometry.slang','ros2/src/so101_station_vulkan/shaders/geometry_soft.slang','ros2/src/so101_station_vulkan/shaders/observed_children.slang')


def expected(value,roi):
    cfg,mask,depth,labels,parents,child=value;h,w=mask.shape;n=w*h
    z=depth.view('<f4').astype('<f8');yy,xx=np.indices(mask.shape)
    with np.errstate(invalid='ignore'):world=np.stack((xx*z,yy*z,z),axis=-1).astype('<f8')
    world[~np.isfinite(z)]=np.nan
    records=child['records'];lookup=np.zeros(n,dtype='<u4');original=np.zeros(n,dtype='<u4');closed_count=np.zeros(n,dtype='<u4');ids=np.zeros((n,9),dtype='<u4');out=np.zeros((len(records),20),dtype='<u4');out[:,:16]=records
    inside=np.ones(mask.shape,bool)
    if roi is not None:
        inside=np.logical_and.reduce([world[:,:,a]>=roi[2*a] for a in range(3)]+[world[:,:,a]<=roi[2*a+1] for a in range(3)])
    eligible=total=0
    for i,r in enumerate(records):
        root,parent,_,area,x,y,bw,bh=map(int,r[:8]);px,py=map(int,parents[parent,4:6]);x+=px;y+=py
        member=child['roots']==root;lookup[root-1]=i+1;original[member.ravel()]=i+1;count=int((member&inside).sum());out[i,16]=count
        if count<8:continue
        eligible+=1;out[i,18]=3;closed=np.zeros(mask.shape,dtype=np.uint8);closed[y:y+bh,x:x+bw]=cv2.morphologyEx(member[y:y+bh,x:x+bw].astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));pixels=np.flatnonzero(closed)
        assert np.all(closed_count[pixels]<9)
        ids[pixels,closed_count[pixels]]=i+1;closed_count[pixels]+=1;out[i,17]=len(pixels);total+=len(pixels)
    assert total<=4*n
    arena=np.concatenate((lookup,original,closed_count,ids.ravel())).astype('<u4')
    return dict(observed=arena,records=out,counts=np.array([len(records),eligible,int(np.count_nonzero(original)),total],dtype='<u4'),child_roots=np.concatenate((child['roots'].ravel(),child['flags'].ravel())).astype('<u4'),world=world)


def prepare(directory,roi=None):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    frozen=json.loads((ROOT/q.FIXTURE).read_text());values=[]
    for name,value in q.scenes(frozen):
        cfg,mask,depth,*_=value
        if cfg['profile']!='typed_parent' or cfg['flags']:continue
        cfg={**cfg,'minimum':max(24,cfg['parent_minimum'])};values.append((name,q.supplemental(cfg,mask,depth)))
    mask=np.zeros((480,640),dtype='<u4');mask[100:140,120:160]=1;mask[200:225,300:330]=2;depth=np.full(mask.shape,np.float32(.5).view(np.uint32),dtype='<u4')
    values.append(('scoped-640x480',q.supplemental(q.configuration(640,480,'typed_parent',minimum=24),mask,depth)))
    manifest=dict(schema='native-observed-diagnostic-v1',scope='observed_preparation_only',roi=roi,sources={s:q.digest(ROOT/s) for s in SOURCES},cases={})
    for name,value in values:
        cfg,mask,depth,labels,parents,_=value;fixture=directory/(name+'.childq');npz=directory/(name+'.npz');fixture.write_bytes(q.encode(cfg,mask,depth,labels,parents))
        with npz.open('xb') as f:np.savez_compressed(f,**expected(value,roi))
        manifest['cases'][name]=dict(fixture=fixture.name,fixture_sha256=q.digest(fixture),expected=npz.name,expected_sha256=q.digest(npz),width=cfg['width'],height=cfg['height'])
    (directory/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return manifest


def compare(directory,name,prefix,precision,mutation=0):
    directory=Path(directory);m=q.json_document(directory/'manifest.json');assert m['schema']=='native-observed-diagnostic-v1';c=m['cases'][name];artifact=lambda s:Path(str(prefix)+s);r=q.json_document(artifact('.json'))
    for key in ('fixture','expected'):
        if Path(c[key]).name!=c[key] or q.digest(directory/c[key])!=c[key+'_sha256']:raise ValueError('input artifact hash')
    for source,digest in m['sources'].items():
        if q.digest(ROOT/source)!=digest:raise ValueError('source changed '+source)
    fields=dict(scope=m['scope'],gpu_executed=True,production_views_used=True,physical_camera=False,objects=False,world_qualified=False,fixture_sha256=c['fixture_sha256'],width=c['width'],height=c['height'],source_frame=91,depth_source_frame=90,publication_session_token=37,unique_inputs=1,completed_outputs=0 if mutation else 1,malformation=mutation,precision=precision,qualifier_source_sha256=m['sources']['ros2/src/so101_station_vulkan/test/qualify_observed_children.cpp'],qualifier_helper_source_sha256=m['sources']['tools/performance/qualify_children.cpp'],external_watchdog_acknowledged=True,status='unusable' if mutation else 'valid')
    for k,v in fields.items():
        if type(r.get(k)) is not type(v) or r[k]!=v:raise ValueError('metadata '+k)
    for k in ('compact_submit_finish_wall_ms','compact_submit_finish_process_cpu_ms','dense_diagnostic_wall_ms','dense_diagnostic_process_cpu_ms'):
        if not isinstance(r.get(k),(float,int)) or not np.isfinite(r[k]) or r[k]<0:raise ValueError('timing '+k)
    libraries=r.get('loaded_libraries',[])
    if len(libraries)!=5 or any(not isinstance(x,dict) or len(x.get('sha256',''))!=64 for x in libraries):raise ValueError('five loaded library identities required')
    if not mutation:
        with np.load(directory/c['expected'],allow_pickle=False) as e:
            for key in ('observed','records','counts','child_roots'):
                if artifact('.'+key+'.bin').read_bytes()!=e[key].tobytes():raise ValueError('exact '+key+' differs')
            actual=np.fromfile(artifact('.world.bin'),dtype='<f8')
            if actual.size!=e['world'].size or not np.allclose(actual,e['world'].ravel(),rtol=0,atol=1e-12,equal_nan=True):raise ValueError('geometry coordinates differ')
    elif any(artifact('.'+key+'.bin').exists() for key in ('observed','records','counts','child_roots','world')):raise ValueError('fault exposed diagnostic arrays')
    return dict(matched=True,case=name,precision=precision,mutation=mutation,report_sha256=q.digest(artifact('.json')),scope=m['scope'])


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True);a=sub.add_parser('prepare');a.add_argument('--output',type=Path,required=True);a.add_argument('--roi',type=float,nargs=6);a=sub.add_parser('compare');a.add_argument('directory',type=Path);a.add_argument('case');a.add_argument('prefix',type=Path);a.add_argument('--precision',choices=('native','software'),required=True);a.add_argument('--mutation',type=int,default=0);a=p.parse_args();print(json.dumps(prepare(a.output,a.roi) if a.mode=='prepare' else compare(a.directory,a.case,a.prefix,a.precision,a.mutation),indent=2))
if __name__=='__main__':main()
