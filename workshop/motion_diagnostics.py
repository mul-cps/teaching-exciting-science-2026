"""Offline speed-conditioned diagnostics; encoder data alone cannot certify vibration."""
import argparse
import json
from pathlib import Path
import numpy as np
from workshop.choreography import JOINTS
from workshop.teleop_analysis import filtered_derivatives

BANDS=(.5,1,2,4,8,16,32,64,128)
STEP=360/4095


def reversal_metrics(t,goal,actual,step,eligible=None):
    dt=float(np.median(np.diff(t)))
    velocity,_,_,valid=filtered_derivatives(goal,dt,.16)
    mask=valid&(np.abs(velocity)>.5)
    if eligible is not None: mask &= eligible
    delta=np.diff(actual); direction=np.sign(velocity[:-1])
    backwards=delta*direction<-.1*step
    events=[]; distance=0.; begin=None
    for i in range(len(delta)):
        if not mask[i] or (i and direction[i]!=direction[i-1]) or delta[i]*direction[i]>.1*step:
            if distance>=1.5*step:events.append((float(t[begin]),distance))
            distance=0.;begin=None
        if mask[i] and backwards[i]:
            if begin is None:begin=i
            distance+=abs(delta[i])
    if distance>=1.5*step:events.append((float(t[begin]),distance))
    return {'backward_events':len(events),'backward_distance':float(sum(d for _,d in events)),
            'largest_backward_excursion':float(max((d for _,d in events),default=0)),
            'event_times':[time for time,_ in events],
            'definition':'cumulative >=1.5 encoder counts against filtered command direction while command speed >0.5; not proof of physical vibration'}


def spectrum_peak(t,residual):
    if len(t)<32:return {'peak_hz':None,'band_rms':None}
    dt=float(np.median(np.diff(t)));x=np.asarray(residual,float)
    x=x-np.polyval(np.polyfit(np.linspace(-1,1,len(x)),x,3),np.linspace(-1,1,len(x)))
    w=np.hanning(len(x));power=abs(np.fft.rfft(x*w))**2
    f=np.fft.rfftfreq(len(x),dt);band=(f>=2)&(f<=20)
    if not band.any():return {'peak_hz':None,'band_rms':None}
    # Two-sided energy correction for interior positive-frequency bins.
    rms=np.sqrt(2*power[band].sum()/(len(x)*np.sum(w*w)))
    return {'peak_hz':float(f[band][np.argmax(power[band])]),'band_rms':float(rms)}


def fit_ripple(t,x):
    """Fit a 5–10 Hz sinusoid plus cubic drift; descriptive, not a plant model."""
    centered=(t-t.mean())/(t[-1]-t[0]);poly=np.vander(centered,4,increasing=True)
    baseline=np.linalg.lstsq(poly,x,rcond=None)[0];baseline_error=np.sum((x-poly@baseline)**2)
    best=None
    for frequency in np.arange(5,10.001,.025):
        phase=2*np.pi*frequency*(t-t[0]);design=np.column_stack([poly,np.sin(phase),np.cos(phase)])
        coefficients=np.linalg.lstsq(design,x,rcond=None)[0];error=float(np.sum((x-design@coefficients)**2))
        if best is None or error<best[0]:best=(error,float(frequency),float(np.hypot(*coefficients[-2:])))
    error,frequency,amplitude=best
    return {'frequency_hz':frequency,'peak_amplitude':amplitude,'implied_peak_velocity_ripple':2*np.pi*frequency*amplitude,
            'explained_detrended_fraction':max(0.,1-error/baseline_error) if baseline_error>1e-12 else 0.,
            'duration_s':float(t[-1]-t[0]),'model':'cubic drift + best sinusoid in 5–10 Hz; fit is descriptive'}


def load(directory,phase=None,limitation_source=None):
    rows=json.loads((directory/'trace.json').read_text())
    if phase:rows=[r for r in rows if r['phase']==phase]
    if limitation_source is not None:
        source=json.loads((limitation_source/'trace.json').read_text());origin=source[0]['command_t']
        st=np.array([r['command_t']-origin for r in source])
        for row in rows:
            i=min(int(np.searchsorted(st,row['source_t'])),len(source)-1)
            row['leader_mapped']=source[i]['leader_mapped']
    if len(rows)<100:raise ValueError('Insufficient recording')
    first=rows[0]['t'];times=np.array([r['t']-first for r in rows])
    t=np.arange(max(rows[0].get('command_t',rows[0]['t']),rows[0].get('follower_t',rows[0]['t']))-first,
                min(rows[-1].get('command_t',rows[-1]['t']),rows[-1].get('follower_t',rows[-1]['t']))-first,.008)
    series={}
    for k in JOINTS:
        goal=np.interp(t,[r.get('command_t',r['t'])-first for r in rows],[r['goal'][k] for r in rows])
        actual=np.interp(t,[r.get('follower_t',r['t'])-first for r in rows],[r['actual'][k] for r in rows])
        step=STEP if k!='gripper' else 100/1494
        limited=np.zeros(len(t),bool)
        if 'leader_mapped' in rows[0]:
            limited=np.interp(t,times,[abs(r['leader_mapped'][k]-r['goal'][k])>step for r in rows])>.01
        series[k]=(goal,actual,step,limited)
    return rows,t,series


def longest_plateau(t,goal,actual,velocity,eligible,step):
    # Use native encoder samples in caller for plateau checks; interpolated
    # samples can conceal quantized holds. Applied here to resampled positions
    # with a tenth-count tolerance and labelled approximate.
    longest=0.;goal_change=0.;begin=None
    for i,delta in enumerate(np.diff(actual)):
        unchanged=abs(delta)<step*.1 and abs(velocity[i])>1 and eligible[i]
        if unchanged:
            if begin is None:begin=i
        elif begin is not None:
            duration=t[i]-t[begin]
            if duration>longest:longest=float(duration);goal_change=float(abs(goal[i]-goal[begin]))
            begin=None
    if begin is not None:
        duration=t[-1]-t[begin]
        if duration>longest:longest=float(duration);goal_change=float(abs(goal[-1]-goal[begin]))
    return {'approx_longest_feedback_hold_s':longest,'command_change_during_hold':goal_change}


def lag_estimate(t,goal,actual):
    v,_,_,valid=filtered_derivatives(goal,.008,.24);av,_,_,_=filtered_derivatives(actual,.008,.24)
    valid &= (np.abs(v)>1)&np.isfinite(av)&(t>t[0]+.5)&(t<t[-1]-.5)
    if valid.sum()<125 or np.std(av[valid])<.5:return None
    scores=[]
    for lag in np.arange(0,.352,.008):
        shifted=np.interp(t-lag,t,np.nan_to_num(v))
        if np.std(shifted[valid])<.1:continue
        scores.append((float(np.corrcoef(shifted[valid],av[valid])[0,1]),float(lag)))
    return max(scores)[1] if scores else None


def describe(directory,phase=None,limitation_source=None):
    rows,t,series=load(directory,phase,limitation_source);report={'source':str(directory),'phase':phase,'samples':len(rows),'duration_s':float(t[-1]-t[0]),'joints':{}}
    for k,(goal,actual,step,limited) in series.items():
        v,acc,_,valid=filtered_derivatives(goal,.008,.16)
        eligible=valid; moving=eligible&(np.abs(v)>.5)
        lag=lag_estimate(t,goal,actual)
        residual=actual-np.interp(t-(lag or 0),t,goal)
        m={'span':float(np.ptp(goal)),'lag_estimate_s':lag,'active_seconds':float(moving.sum()*.008),
           'moving_speed_p50':float(np.median(abs(v[moving]))) if moving.any() else None,
           'moving_speed_p90':float(np.percentile(abs(v[moving]),90)) if moving.any() else None,
           'seconds_by_speed_band':{},'stable_window_evidence':{},'backwards':reversal_metrics(t,goal,actual,step,eligible),
           'limited_fraction':float(limited.mean()),'plateau':longest_plateau(t,goal,actual,v,eligible,step)}
        windows=[];half=50
        for center in range(half,len(t)-half,16):
            sl=slice(center-half,center+half+1);speed=abs(v[sl]);median=float(np.median(speed))
            if not eligible[sl].all() or median<.5 or np.quantile(speed,.9)-np.quantile(speed,.1)>median*.75:continue
            direction=np.sign(v[sl]);sign=np.sign(v[center])
            if np.mean(direction==sign)<.98:continue
            spectral=spectrum_peak(t[sl],residual[sl])
            backward=reversal_metrics(t[sl],goal[sl],actual[sl],step)
            windows.append({'t':float(t[center]),'speed':median,'direction':float(sign),
                            'command_angle':float(goal[center]),'residual_band_rms':spectral['band_rms'],'peak_hz':spectral['peak_hz'],
                            'limiting_fraction':float(limited[sl].mean()),'backward_events':backward['backward_events'],'largest_backward_excursion':backward['largest_backward_excursion']})
        for lo,hi in zip(BANDS,BANDS[1:]):
            label=f'{lo:g}-{hi:g}';mask=moving&(abs(v)>=lo)&(abs(v)<hi)
            m['seconds_by_speed_band'][label]=float(mask.sum()*.008)
            selected=[w for w in windows if lo<=w['speed']<hi]
            m['stable_window_evidence'][label]={'overlapping_windows':len(selected),
                'median_residual_2_20hz_rms':float(np.median([w['residual_band_rms'] for w in selected])) if selected else None,
                'p90_residual_2_20hz_rms':float(np.percentile([w['residual_band_rms'] for w in selected],90)) if selected else None,
                'windows_with_backward_excursion_gt_2_counts':sum(w['largest_backward_excursion']>=2*step for w in selected)}
        m['stable_windows']=windows;report['joints'][k]=m
    return report,(t,series)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--teleop',required=True,type=Path);p.add_argument('--replay',required=True,type=Path);p.add_argument('--dance',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    report={'method':{'units':'body degrees, gripper percent','encoder_step_deg':STEP,'speed_band_units':'degrees/s or percent/s','windows':'0.808s with 0.128s stride, >98% same direction, speed spread <=75% median; analyze delivered commands, include and label original per-joint limiting identically in teleop/replay','spectral':'cubic-detrended lag-compensated position residual, 2-20 Hz Hann spectrum; quantization and servo telemetry can contribute','limitations':['No independent high-rate motion measurement: encoder oscillation is not proof of structural vibration.','Overlapping windows are not independent trials.','Lag is globally estimated and load dependent; spectral residual includes model error.','Teleop and dance differ in path, pose, speed and acceleration register; comparison is observational.','Shoulder lift/gripper have almost no active teleop motion.','No universal forbidden speed band can be certified from this coverage.']},'runs':{},'repeatability':{}}
    plots={}
    for name,path,phase in [('teleop',a.teleop,None),('replay',a.replay,'forward'),('dance',a.dance,None)]:
        report['runs'][name],plots[name]=describe(path,phase,a.teleop if name=='replay' else None)
    src=json.loads((a.teleop/'trace.json').read_text());replay=[r for r in json.loads((a.replay/'trace.json').read_text()) if r['phase']=='forward'];origin=src[0]['command_t'];source_times=np.array([r['command_t']-origin for r in src])
    indices=np.searchsorted(source_times,[r['source_t'] for r in replay]);indices=np.minimum(indices,len(src)-1)
    report['repeatability']['sent_command_max_difference']=max(abs(src[i]['goal'][k]-r['goal'][k]) for i,r in zip(indices,replay) for k in JOINTS)
    report['repeatability']['forward_samples']=len(replay)
    report['base_ripple_fits']={}
    for name,limits in [('teleop',(30.2,31.7)),('replay',(30.2,31.7)),('dance',(30.4,33.4))]:
        t,series=plots[name];goal,actual,_,_=series['shoulder_pan'];mask=(t>=limits[0])&(t<=limits[1]);lag=report['runs'][name]['joints']['shoulder_pan']['lag_estimate_s'] or 0
        residual=actual-np.interp(t-lag,t,goal)
        fit=fit_ripple(t[mask],residual[mask]);fit['interval_s']=limits;report['base_ripple_fits'][name]=fit
    report['repeatability']['feedback_rmse_by_joint']={k:float(np.sqrt(np.mean([(r['actual'][k]-src[i]['actual'][k])**2 for i,r in zip(indices,replay)]))) for k in JOINTS}
    (a.output/'diagnostics.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,2,figsize=(13,9));keys=['shoulder_pan','elbow_flex','wrist_flex','wrist_roll']
    for ax,k in zip(axs.flat,keys):
        for name,color in [('teleop','tab:blue'),('replay','tab:green'),('dance','tab:orange')]:
            windows=report['runs'][name]['joints'][k]['stable_windows']
            if windows:ax.scatter([w['speed'] for w in windows],[w['residual_band_rms'] for w in windows],s=12,alpha=.55,label=name,color=color)
        ax.axhline(STEP/np.sqrt(12),color='gray',ls=':',label='one-count uniform quantization RMS reference')
        ax.set_xscale('log');ax.set_xlabel('median commanded speed (deg/s)');ax.set_ylabel('lag-compensated residual 2–20 Hz RMS (deg)');ax.set_title(k);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(a.output/'speed-residual.png',dpi=150)
    fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
    for ax,k in zip(axs,['elbow_flex','wrist_flex','shoulder_pan']):
        for name,color in [('teleop','tab:blue'),('replay','tab:green')]:
            t,series=plots[name];goal,actual,step,_=series[k];ax.plot(t,actual-goal,label=name,color=color,lw=.7)
        ax.set_ylabel(k+'\ntracking error (deg)');ax.legend()
    axs[-1].set_xlabel('forward recording time (s)');fig.tight_layout();fig.savefig(a.output/'replay-repeatability.png',dpi=150)
    fig,axs=plt.subplots(3,1,figsize=(13,9))
    for ax,name in zip(axs,['dance','teleop','replay']):
        t,series=plots[name];goal,actual,_,_=series['shoulder_pan'];limits=report['base_ripple_fits'][name]['interval_s'];mask=(t>=limits[0])&(t<=limits[1])
        ax.plot(t[mask],goal[mask],label='sent command',lw=1.2);ax.plot(t[mask],actual[mask],label='encoder feedback',lw=.9)
        ax.set_ylabel('base angle (deg)');ax.set_title(name+' — same 6–9 Hz encoder ripple, different mean speed');ax.legend()
    axs[-1].set_xlabel('time in each forward recording (s)');fig.tight_layout();fig.savefig(a.output/'base-ripple.png',dpi=150)
    print('Saved',a.output/'diagnostics.json')

if __name__=='__main__':main()
