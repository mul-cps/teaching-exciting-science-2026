import numpy as np
from workshop.motion_diagnostics import reversal_metrics, spectrum_peak

def test_quantized_monotone_ramp_is_not_backward_oscillation():
    t=np.arange(0,4,.008);step=360/4095; goal=2*t
    actual=np.floor(goal/step)*step
    m=reversal_metrics(t,goal,actual,step)
    assert m['backward_events']==0
    assert m['backward_distance']==0

def test_visible_dither_has_backward_events_and_correct_spectrum():
    t=np.arange(0,4,.008);step=360/4095; goal=2*t
    actual=goal+.3*np.sin(2*np.pi*6*t)
    m=reversal_metrics(t,goal,actual,step)
    assert m['backward_events']>10
    assert m['backward_distance']>1
    peak=spectrum_peak(t,actual-goal)
    assert abs(peak['peak_hz']-6)<.3


def test_one_count_encoder_noise_is_below_backward_event_threshold():
    t=np.arange(0,4,.008); step=360/4095; goal=t*0
    actual=step*(np.arange(len(t))%2)
    assert reversal_metrics(t,goal,actual,step)['backward_events']==0

def test_sine_fit_recovers_ripple_amplitude_and_frequency_with_drift():
    from workshop.motion_diagnostics import fit_ripple
    t=np.arange(0,3,.008); x=1+2*t+.4*t*t+.3*np.sin(2*np.pi*7.2*t+.6)
    m=fit_ripple(t,x)
    assert abs(m['frequency_hz']-7.2)<.03
    assert abs(m['peak_amplitude']-.3)<.01
    assert m['explained_detrended_fraction']>.99
