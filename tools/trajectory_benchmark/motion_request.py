#!/usr/bin/env python3
"""Print bounded MoveIt request scaling fields; never sends a request."""
import argparse
import json
from pathlib import Path
import yaml

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('mode',choices=('transfer','approach','contact','loaded_lift','placement','home'))
a=p.parse_args()
profiles=yaml.safe_load(Path(__file__).with_name('motion_profiles.yaml').read_text())
selected=profiles[a.mode]
if set(selected)!={'max_velocity_scaling_factor','max_acceleration_scaling_factor'} or not all(0<float(v)<=1 for v in selected.values()):
    raise ValueError('Only bounded MoveIt velocity and acceleration scaling fields allowed')
print(json.dumps(selected,indent=2))
