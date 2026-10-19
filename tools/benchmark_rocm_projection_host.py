#!/usr/bin/env python3
"""CPU-only staging/compatibility costs; never import Torch or execute GPU work.

Ordinary NumPy host buffers proxy array copies only: pinned allocation, GPU
transfers, kernels and driver initialization are explicitly excluded.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.benchmark_rocm_projection import load_oracle,compare,reference_texture,summarize
from workshop.perception.rocm_projection import CPUCompatibility,numpy_raw_projection


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--oracle',type=Path,required=True)
    parser.add_argument('--compatibility-library',type=Path,required=True)
    parser.add_argument('--samples',type=int,default=10)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 2<=args.samples<=20:parser.error('bounded2..20repeats')
    started=time.perf_counter();record,c,depth,color,aligned_ref,xyz_ref,uv_ref,pins=load_oracle(args.oracle);load_ms=(time.perf_counter()-started)*1000
    started=time.perf_counter();compat=CPUCompatibility(args.compatibility_library);library_ms=(time.perf_counter()-started)*1000
    aligned,raw_xyz,pixels,flags=numpy_raw_projection(depth,c)
    if flags:raise ValueError('frozen cohort unsupported: '+str(flags))
    w,h=c.depth_size;cw,ch=c.color_size;count=w*h;color_count=cw*ch;flag=color_count+5*count
    started=time.perf_counter();raw_host=np.empty(count,np.int32);rgb_host=np.empty_like(color);host=np.empty(flag+1,np.int32);template=np.empty_like(host)
    template[:color_count]=np.where(aligned.reshape(-1)==0,65536,aligned.reshape(-1));template[color_count:color_count+3*count]=raw_xyz.reshape(-1).view(np.int32);template[color_count+3*count:flag]=pixels.reshape(-1).view(np.int32);template[flag]=0
    allocation_ms=(time.perf_counter()-started)*1000;times={key:[] for key in ('freeze_input_host_copies','emulated_packed_host_memcpy','cpu_compatibility','post_readback_ownership')};reference=(aligned_ref,xyz_ref,uv_ref);expected_rgb,expected_valid=reference_texture(xyz_ref,uv_ref,color);previous=None;parity=[]
    for iteration in range(args.samples):
        start=time.perf_counter();np.copyto(raw_host,depth.reshape(-1),casting='safe');np.copyto(rgb_host,color);times['freeze_input_host_copies'].append((time.perf_counter()-start)*1000)
        start=time.perf_counter();np.copyto(host,template);times['emulated_packed_host_memcpy'].append((time.perf_counter()-start)*1000)
        start=time.perf_counter();raw_aligned=host[:color_count];raw_aligned[raw_aligned==65536]=0;owned_aligned=raw_aligned.astype(np.uint16).reshape(ch,cw);xyz=host[color_count:color_count+3*count].view(np.float32).reshape(h,w,3);xy=host[color_count+3*count:flag].view(np.float32).reshape(h,w,2)
        compat_start=time.perf_counter();uv,packed,valid=compat.finish(xyz,xy,rgb_host,c);times['cpu_compatibility'].append((time.perf_counter()-compat_start)*1000)
        owned_xyz=xyz.copy()
        for array in (owned_aligned,owned_xyz,uv,packed,valid):array.setflags(write=False)
        times['post_readback_ownership'].append((time.perf_counter()-start)*1000)
        result=compare(owned_aligned,owned_xyz,uv,reference);result['rgb_different']=int(np.count_nonzero(packed!=expected_rgb));result['valid_different']=int(np.count_nonzero(valid!=expected_valid));result['pass']&=result['rgb_different']==result['valid_different']==0;parity.append(result)
        if previous is not None:
            old,hashes=previous;assert all(hashlib.sha256(a.tobytes()).hexdigest()==digest for a,digest in zip(old,hashes))
        arrays=(owned_aligned,owned_xyz,uv,packed,valid);previous=arrays,[hashlib.sha256(a.tobytes()).hexdigest() for a in arrays]
    report={'gpu_executed':False,'torch_imported':'torch' in sys.modules,'qualified':False,'scope':'ordinaryNumPyhost-copyproxy +compiledCPUocclusion/RGB +ownedimmutableoutput; excludespinnedmemory/GPUtransfer/kernels/driver; NOTwholeGPUlatency','input_sha256':pins,'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),ROOT/'workshop/perception/projection_compat.cpp',ROOT/'workshop/perception/rocm_projection.py')},'compatibility_library_sha256':hashlib.sha256(args.compatibility_library.read_bytes()).hexdigest(),'depth_size':c.depth_size,'color_size':c.color_size,'cold_file_hash_calibration_ms':load_ms,'cold_library_load_ms':library_ms,'host_proxy_allocations_ms':allocation_ms,'stages':{key:summarize(values) for key,values in times.items()},'per_frame_gpu_transfer_budget_bytes':{'h2d':count*4,'d2h':(flag+1)*4},'parity_samples':parity,'all_pass':all(row['pass'] for row in parity),'old_cohort_survives_scratch_reuse':True}
    assert report['gpu_executed'] is False and report['torch_imported'] is False
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({key:report[key] for key in ('all_pass','stages','depth_size','color_size')}));return 0 if report['all_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
