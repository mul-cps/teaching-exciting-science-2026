import numpy as np
from workshop.system_probe import sweep_profile

def test_probe_is_rest_to_rest_with_bounded_speed_acceleration_and_jerk():
    for speed in [1,3,8,12]:
        ramp=max(.8,speed*.085)
        total=24/speed+ramp
        assert sweep_profile(0,24,speed)==(0,0,0,0)
        assert sweep_profile(total,24,speed)==(24,0,0,0)
        for t in np.linspace(0,total,2001):
            q,v,a,j=sweep_profile(t,24,speed)
            assert 0<=q<=24+1e-9
            assert abs(v)<=speed+1e-9
            assert abs(a)<=24
            assert abs(j)<=100
