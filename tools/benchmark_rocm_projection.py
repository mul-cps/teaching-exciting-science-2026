#!/usr/bin/env python3
"""Frozen software-frame CPU parity and opt-in ROCm whole-cohort benchmark.

No ROS, physical devices or default GPU activation. All repeats are diagnostics,
not new acquisitions. The installed-SDK oracle, not NumPy timing, is the baseline.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from workshop.perception.rocm_projection import calibration_from_oracle,numpy_raw_projection,CPUCompatibility


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_oracle(path):
    path=Path(path)
    if (path/'oracle.json').stat().st_size>65536:raise ValueError('oracle metadata exceeds bound')
    record=json.loads((path/'oracle.json').read_text());c=calibration_from_oracle(record)
    w,h=c.depth_size;cw,ch=c.color_size
    def raw(name,dtype,shape):
        p=path/name;expected=int(np.prod(shape))*np.dtype(dtype).itemsize
        if p.stat().st_size!=expected:raise ValueError('oracle file byte count mismatch: '+name)
        return np.fromfile(p,dtype=dtype).reshape(shape)
    depth=raw('source.z16','<u2',(h,w));color=raw('color.rgb8',np.uint8,(ch,cw,3))
    aligned=raw('aligned.z16','<u2',(ch,cw));xyz=raw('vertices.f32','<f4',(h,w,3));uv=raw('uv.f32','<f4',(h,w,2))
    if not np.isfinite(xyz).all() or not np.isfinite(uv).all():raise ValueError('nonfinite SDK reference')
    files={p.name:digest(p) for p in path.iterdir() if p.is_file() and p.name in
           {'source.z16','color.rgb8','aligned.z16','vertices.f32','uv.f32','oracle.json'}}
    if any(files.get(name)!=value for name,value in record.get('output_sha256',{}).items() if name in files):
        raise ValueError('CPU oracle output changed')
    return record,c,depth,color,aligned,xyz,uv,files


def compare(aligned,xyz,uv,reference):
    expected_aligned,expected_xyz,expected_uv=reference
    result={'depth_different':int(np.count_nonzero(aligned!=expected_aligned)),
        'holes_different':int(np.count_nonzero((aligned==0)!=(expected_aligned==0))),
        'vertex_zero_mask_different':int(np.count_nonzero((xyz[...,2]==0)!=(expected_xyz[...,2]==0))),
        'xyz_max_abs_m':float(np.max(np.abs(xyz-expected_xyz))),
        'uv_max_abs':float(np.max(np.abs(uv-expected_uv)))}
    result['pass']=(result['depth_different']==result['holes_different']==result['vertex_zero_mask_different']==0
                    and result['xyz_max_abs_m']<=1e-5 and result['uv_max_abs']<=1e-5
                    and np.isfinite(xyz).all() and np.isfinite(uv).all())
    result['pass']=bool(result['pass'])
    return result



def reference_texture(xyz,uv,color):
    h,w=color.shape[:2]
    px=uv[...,0]*np.float32(w);py=uv[...,1]*np.float32(h)
    valid=(xyz[...,2]>0)&np.isfinite(uv).all(-1)&(uv[...,0]>=0)&(uv[...,0]<1)&(uv[...,1]>=0)&(uv[...,1]<1)
    valid &= (px>=0)&(px<w)&(py>=0)&(py<h)
    packed=np.zeros(valid.shape,np.uint32)
    ix=np.trunc(px[valid]).astype(np.int32);iy=np.trunc(py[valid]).astype(np.int32)
    rgb=color[iy,ix].astype(np.uint32)
    packed[valid]=(rgb[:,0]<<16)|(rgb[:,1]<<8)|rgb[:,2]
    return packed,valid


def summarize(values):
    return {'samples':len(values),'mean_ms':statistics.mean(values),
            'median_ms':statistics.median(values),'p95_ms':float(np.percentile(values,95)),
            'min_ms':min(values),'max_ms':max(values)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--oracle',type=Path,action='append',required=True)
    parser.add_argument('--compatibility-library',type=Path,required=True)
    parser.add_argument('--gpu',action='store_true')
    parser.add_argument('--samples',type=int,default=10)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1<=len(args.oracle)<=4 or not 1<=args.samples<=20:parser.error('bounded4cohorts/20samples')
    paths=['workshop/perception/rocm_projection.py','workshop/perception/rocm_projection_backend.py',
           'workshop/perception/rocm_projection_kernels.py','workshop/perception/projection_compat.cpp',
           'tools/realsense_cpu_projection_oracle.cpp','tools/benchmark_rocm_projection.py']
    report={'qualified':False,'gpu_requested':args.gpu,'diagnostic_repeats_are_new_acquisitions':False,
            'source_sha256':{p:digest(ROOT/p) for p in paths},
            'compatibility_library_sha256':digest(args.compatibility_library),'cohorts':[],
            'timing_scope':'whole input freeze/upload/two kernels/one packed readback/CPUocclusion+RGB/output ownership; excludes ROS and physical capture',
            'all_pass':True}
    compatibility=CPUCompatibility(args.compatibility_library)
    try:
        for path in args.oracle:
            record,c,depth,color,expected_aligned,expected_xyz,expected_uv,files=load_oracle(path)
            reference=(expected_aligned,expected_xyz,expected_uv)
            expected_rgb,expected_valid=reference_texture(expected_xyz,expected_uv,color)
            aligned,xyz,pixels,flags=numpy_raw_projection(depth,c)
            if flags:raise ValueError('NumPy CPU contract unsupported: '+str(flags))
            uv,_,_=compatibility.finish(xyz,pixels,color,c)
            entry={'oracle':str(path),'input_sha256':files,'cpu_reference_parity':compare(aligned,xyz,uv,reference),
                'sdk_complete_ms':record.get('sdk_complete_ms',[record['complete_ms']]),
                'original_headers':{name:record.get('source_provenance',{}).get('capture',{}).get(name)
                                    for name in ('depth_header','color_header')},
                'gpu_samples':[]}
            report['cohorts'].append(entry)
            report['all_pass'] &= entry['cpu_reference_parity']['pass']
            if args.gpu:
                from workshop.perception.rocm_projection_backend import ROCmProjection
                started=time.perf_counter();backend=ROCmProjection(c,args.compatibility_library,enabled=True)
                entry['cold_initialization_ms']=(time.perf_counter()-started)*1000
                entry['hardware']=backend.hardware
                entry['per_cohort_h2d_bytes']=c.depth_size[0]*c.depth_size[1]*4
                entry['per_cohort_d2h_bytes']=(c.color_size[0]*c.color_size[1]+5*c.depth_size[0]*c.depth_size[1]+1)*4
                for sample in range(args.samples):
                    result=backend.process(depth,color);parity=compare(result.aligned_z16,result.xyz,result.uv,reference)
                    parity['texture_valid_different']=int(np.count_nonzero(result.valid!=expected_valid))
                    parity['rgb_packed_different']=int(np.count_nonzero(result.rgb_packed!=expected_rgb))
                    parity['pass'] &= parity['texture_valid_different']==parity['rgb_packed_different']==0
                    entry['gpu_samples'].append({'sample':sample,'cold':sample==0,'complete_ms':result.complete_ms,'parity':parity})
                    report['all_pass'] &= parity['pass']
                times=[r['complete_ms'] for r in entry['gpu_samples']]
                entry['all_timing']=summarize(times)
                if len(times)>1:entry['warm_timing']=summarize(times[1:])
    except Exception as error:
        report['all_pass']=False;report['error']=type(error).__name__+': '+str(error)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'all_pass':report['all_pass'],'gpu_requested':args.gpu,'error':report.get('error'),
                      'cohorts':len(report['cohorts'])}))
    return 0 if report['all_pass'] else 3


if __name__=='__main__':raise SystemExit(main())
