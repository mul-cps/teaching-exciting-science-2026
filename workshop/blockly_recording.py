"""Read-only, bounded fixed-jaw follower stroke recordings."""
import copy
import math
import time
from .blockly_program import name, number

JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')


class Recorder:
    def __init__(self, recording_name, identity, *, clock=time.monotonic):
        self.name=name(recording_name)
        if (not isinstance(identity,dict) or not isinstance(identity.get('arm_id'),str)
                or not identity['arm_id'] or len(identity.get('calibration_sha256','')) != 64):
            raise ValueError('fresh arm identity and calibration hash required')
        self.identity=dict(identity); self.clock=clock; self.started=clock()
        self.rows=[]; self.jaw=None; self.error=None

    def add(self, positions, jaw, stamp):
        if self.error: raise ValueError(self.error)
        try:
            if not isinstance(positions,(list,tuple)) or len(positions)!=5: raise ValueError('five arm joint positions required')
            values=[number(v,-math.pi*2,math.pi*2) for v in positions]
            number(jaw,-math.pi*2,math.pi*2); number(stamp,0,1e12)
            if self.rows:
                delta=stamp-self.rows[-1]['source_stamp']
                if delta<=0 or delta>.3: raise ValueError('joint feedback stopped, regressed or has a gap >300 ms')
                if abs(jaw-self.jaw)>.015: raise ValueError('jaw changed: record fixed-jaw strokes and use separate grip/release blocks')
            if len(self.rows)>=600: raise ValueError('recording exceeds 600 samples')
            if self.rows and stamp-self.rows[0]['source_stamp']>60: raise ValueError('recording exceeds 60 seconds')
            if self.jaw is None: self.jaw=jaw
            self.rows.append({'source_stamp':stamp,'positions':values})
        except (TypeError,ValueError) as error:
            self.error=str(error); raise ValueError(self.error) from error

    def finish(self):
        if self.error: raise ValueError(self.error)
        if len(self.rows)<2: raise ValueError('record at least two fresh samples')
        origin=self.rows[0]['source_stamp']
        return {'schema':'so101-blockly-stroke-v1','name':self.name,'identity':self.identity,
                'joint_names':list(JOINTS),'jaw_radians':self.jaw,
                'points':[{'time':r['source_stamp']-origin,'positions':r['positions']} for r in self.rows]}


def replay_points(recording, identity, speed=.5):
    number(speed,.1,1)
    if recording.get('schema')!='so101-blockly-stroke-v1' or recording.get('identity')!=identity:
        raise ValueError('recording identity/calibration differs from current follower')
    if recording.get('joint_names')!=list(JOINTS): raise ValueError('recording joint names differ')
    check=Recorder(recording['name'],identity)
    for row in recording.get('points',[]):
        if set(row)!={'time','positions'}: raise ValueError('unsupported recording fields')
        check.add(row['positions'],recording['jaw_radians'],row['time'])
    validated=check.finish()
    return [{'time':row['time']/speed,'positions':list(row['positions'])} for row in validated['points']]
