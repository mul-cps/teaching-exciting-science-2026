"""Offline timestamp-aware paired teleop analysis; filtered jerk is window dependent."""
import argparse
import json
from pathlib import Path
import numpy as np
from workshop.choreography import JOINTS

def filtered_derivatives(x, dt, window):
    half=max(3,int(round(window/dt/2))); offsets=np.arange(-half,half+1)*dt
    weights=np.linalg.pinv(np.vander(offsets,6,increasing=True))
    valid=np.zeros(len(x),dtype=bool); valid[half:-half]=True
    derivatives=[]
    for order,factor in ((1,1),(2,2),(3,6)):
        values=np.full(len(x),np.nan)
        if len(x)>=len(offsets): values[half:-half]=np.correlate(x,weights[order]*factor,mode='valid')
        derivatives.append(values)
    return (*derivatives,valid)

def analyze(directory):
    rows=json.loads((directory/'trace.json').read_text())
    if len(rows)<100: raise ValueError('Too few paired samples')
    metadata=json.loads((directory/'metadata.json').read_text())
    original=np.array([r['t'] for r in rows]); gaps=np.diff(original)
    dt=1/metadata['hz']; t=np.arange(max(rows[0]['leader_t'],rows[0]['follower_t']),min(rows[-1]['leader_t'],rows[-1]['follower_t']),dt)
    limited=np.interp(t,original,[r['limited'] for r in rows])>.01
    data={}; report={'samples':len(rows),'duration_s':float(original[-1]),'mean_hz':float(1/gaps.mean()),'max_gap_ms':float(gaps.max()*1000),'limited_fraction':float(limited.mean()),'mapping':metadata['mapping'],'servo_acceleration':metadata['follower']['acceleration'],'units':'body degrees; gripper percent','filter':'degree-5 local polynomial; window sensitivity 0.16/0.24/0.40 s; edges excluded; no raw finite-difference jerk claims','joints':{}}
    for k in JOINTS:
        series={}
        for name,field,timefield in [('leader','leader_mapped','leader_t'),('command','goal','command_t'),('follower','actual','follower_t')]:
            series[name]=np.interp(t,[r[timefield] for r in rows],[r.get(field,r['goal'])[k] for r in rows])
        data[k]=series
        derivatives={n:filtered_derivatives(x,dt,.24) for n,x in series.items()}
        v=derivatives['command'][0]; # Exclude derivative windows touching any limited sample.
        pad=int(np.ceil(.4/dt/2))
        contaminated=np.convolve(limited.astype(int),np.ones(2*pad+1,dtype=int),mode='same')>0
        valid=derivatives['command'][3]&~contaminated
        metrics={'leader_span':float(np.ptp(series['leader'])),'follower_span':float(np.ptp(series['follower'])),'speed_bins':{},'filtered_jerk_rms_by_window':{}}
        for label,lo,hi in [('slow',1,5),('fast',5,30)]:
            mask=valid&(np.abs(v)>=lo)&(np.abs(v)<hi)
            error=series['follower']-series['command']
            metrics['speed_bins'][label]={'seconds':float(mask.sum()*dt),'tracking_rmse':float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None}
        # Estimate follower delay only with varying motion and ample unclipped samples.
        lag_scores=[]
        fv=derivatives['follower'][0]
        for delay in np.arange(0,.504,dt):
            shifted=np.interp(t-delay,t,np.nan_to_num(v))
            mask=valid&np.isfinite(fv)&(t>t[0]+.6)&(np.abs(v)>1)
            if mask.sum()>50 and np.std(fv[mask])>.5 and np.std(shifted[mask])>.5:
                lag_scores.append((float(np.corrcoef(shifted[mask],fv[mask])[0,1]),float(delay)))
        metrics['velocity_lag_estimate_s']=max(lag_scores)[1] if lag_scores else None
        metrics['velocity_lag_correlation']=max(lag_scores)[0] if lag_scores else None
        for window in (.16,.24,.40):
            scores={}
            for n,x in series.items():
                _,_,jerk,interior=filtered_derivatives(x,dt,window)
                mask=interior&~contaminated&(np.abs(v)>1)
                scores[n]=float(np.sqrt(np.mean(jerk[mask]**2))) if mask.any() else None
            metrics['filtered_jerk_rms_by_window'][str(window)]=scores
        report['joints'][k]=metrics
    (directory/'analysis.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(directory/'paired-resampled.npz',t=t,limited=limited,**{f'{k}_{n}':x for k,series in data.items() for n,x in series.items()})
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axs=plt.subplots(5,1,figsize=(13,12),sharex=True)
        for ax,k in zip(axs,JOINTS[:5]):
            for n,x in data[k].items(): ax.plot(t,x,label=n,lw=.8)
            ax.set_ylabel(k+'\n(degrees)'); ax.legend(loc='upper right',ncol=3)
        axs[-1].set_xlabel('seconds; telemetry reads use separate timestamps')
        fig.tight_layout(); fig.savefig(directory/'paired-tracking.png',dpi=130)
    except ImportError: pass
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args()
    print(json.dumps(analyze(a.directory),indent=2))

if __name__=='__main__':main()
