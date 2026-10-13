import numpy as np
from workshop.teleop_analysis import filtered_derivatives

def test_derivatives_recover_cubic_interior_without_encoder_spikes():
    t=np.arange(0,2,.008)
    x=2+3*t+4*t*t+5*t*t*t
    v,a,j,valid=filtered_derivatives(x,.008,.24)
    assert np.allclose(v[valid],3+8*t[valid]+15*t[valid]**2,atol=1e-6)
    assert np.allclose(a[valid],8+30*t[valid],atol=1e-6)
    assert np.allclose(j[valid],30,atol=1e-5)
