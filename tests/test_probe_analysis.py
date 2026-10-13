import json
import numpy as np
from workshop.probe_analysis import analyze_probe


def test_probe_analysis_recovers_ripple_and_excludes_settling(tmp_path):
    stages=[dict(name='baseline-hold',begin=0,duration=2,speed=0,from_angle=0,to_angle=0),
            dict(name='speed-3-positive',begin=2,duration=5,speed=3,from_angle=0,to_angle=12,acceleration_register=254)]
    rows=[]
    for t in np.arange(0,7,.008):
        goal=0 if t<2 else 3*(t-2)
        actual=(.5 if t<.5 else 0) if t<2 else goal+.32*np.sin(2*np.pi*8.7*t)
        rows.append(dict(t=float(t),command_t=float(t),follower_t=float(t),phase=stages[0 if t<2 else 1]['name'],goal={'shoulder_pan':goal},actual={'shoulder_pan':actual},cycle_ms=2))
    (tmp_path/'trace.json').write_text(json.dumps(rows))
    (tmp_path/'probe-plan.json').write_text(json.dumps(dict(stages=stages,start={'shoulder_pan':0},duration_s=7)))
    (tmp_path/'result.json').write_text(json.dumps(dict(status='complete',cleanup={'torque_off':True})))
    report=analyze_probe(tmp_path)
    m=report['phases']['speed-3-positive']
    assert abs(m['ripple_fit']['frequency_hz']-8.7)<.1
    assert abs(m['ripple_fit']['peak_amplitude']-.32)<.02
    assert report['holds']['baseline-hold']['settled_range_deg']==0
    assert report['timing']['max_gap_ms']<8.01
