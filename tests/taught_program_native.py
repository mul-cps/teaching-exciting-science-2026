"""Named skill through existing shared ROS program endpoint; simulation only."""
import json
import os
from pathlib import Path
import sys
import tempfile
if os.environ.get('ROS_DOMAIN_ID')!='180': raise SystemExit('Use isolated ROS_DOMAIN_ID=180')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.run_blockly_station import Application,strict_json
from workshop.blockly_ros_api import ROSProgramAPI
from workshop.blockly_client import Program,run_program
with tempfile.TemporaryDirectory() as directory:
    app=Application(Path(directory));api=ROSProgramAPI(app,strict_json)
    try:
        result=run_program(Program().ev3('eject').ev3('left_end').pick_place_recording('fixed_pick_place',.5),physical=False,timeout=10,wrapper_bridge=False)
        assert result['complete'] is True
        skill=result['taught_skills'][0]
        assert skill['simulation'] is True and skill['physical_evidence'] is False
        assert all(skill[key] is False for key in ('pickup_verified','lift_verified','release_verified'))
        assert app.backend.state.brick=='place' and app.backend.state.grasp_verified is False
        print(json.dumps({'named_shared_ROS_program_service':True,'simulation':True,'physical_evidence':False,'domain':180}))
    finally:api.close();app.close()
