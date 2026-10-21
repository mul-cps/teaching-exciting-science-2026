#!/usr/bin/env python3
"""Materialize resident-launch model inputs without starting ROS."""
import argparse
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import yaml

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--description-share',type=Path,required=True)
p.add_argument('--config-share',type=Path,required=True)
p.add_argument('--offsets',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
root=Path(__file__).resolve().parents[2]
module_path=root/'ros2/src/so101_station_control/so101_station_control/model_offsets.py'
spec=importlib.util.spec_from_file_location('offline_model_offsets',module_path)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
source=a.description_share/'urdf/so101_arm.urdf.xacro'
description=xacro.process_file(str(source),mappings={'variant':'follower','use_ros2_control':'false'}).toxml()
description=m.apply_model_offsets(description,yaml.safe_load(a.offsets.read_text()))
(a.output/'model.urdf').write_text(description)
semantic=ET.fromstring((a.config_share/'config/so101_arm.srdf').read_text())
ee=semantic.find("end_effector[@name='ee']")
if ee is None or ee.get('parent_link') not in {'gripper_link','gripper_frame_link'}:raise ValueError('Unexpected end effector')
ee.set('parent_link','gripper_frame_link')
(a.output/'model.srdf').write_text(ET.tostring(semantic,encoding='unicode'))
# Hash every xacro source and include plus offsets; the resolved outputs are hashed by compare.py.
import hashlib
files=[*sorted((a.description_share/'urdf').rglob('*')),a.config_share/'config/so101_arm.srdf',a.offsets,module_path]
(a.output/'model-sources.json').write_text(json.dumps({str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()},indent=2)+'\n')
