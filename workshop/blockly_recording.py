"""Read-only, bounded fixed-jaw follower stroke recordings."""
import copy
import math
import time
from .blockly_program import name, number

CALIBRATION_FIELDS=('jaw_calibration_sha256','joint_calibration_sha256')

def require_physical_calibration(identity):
    for field in CALIBRATION_FIELDS:
        value=identity.get(field)
        if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('qualified loaded jaw and joint calibration digests required; reteach recording')


COHORT_FIELDS=frozenset({'owner_generation'})
CAPTURE_FIELDS=COHORT_FIELDS | {'source_sha256'}


def durable_binding(identity):
    if not isinstance(identity,dict): raise ValueError('recording identity unavailable')
    return {key:value for key,value in identity.items() if key not in COHORT_FIELDS}


def recording_binding(recording):
    schema=recording.get('schema')
    if schema not in {'so101-blockly-stroke-v1','so101-blockly-stroke-v2'}:
        raise ValueError('unsupported recording schema')
    binding=durable_binding(recording.get('identity'))
    if schema=='so101-blockly-stroke-v2':
        if recording.get('binding')!=binding or recording.get('capture_cohort')!={key:value for key,value in recording['identity'].items() if key in CAPTURE_FIELDS}:
            raise ValueError('recording binding/capture cohort disagrees with identity')
        if recording.get('origin') not in {'physical','synthetic_simulation'}: raise ValueError('recording origin unavailable')
    return binding


JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')


class Recorder:
    def __init__(self, recording_name, identity, *, clock=time.monotonic, origin=None):
        self.name=name(recording_name)
        if (not isinstance(identity,dict) or not isinstance(identity.get('arm_id'),str)
                or not identity['arm_id'] or len(identity.get('calibration_sha256','')) != 64):
            raise ValueError('fresh arm identity and calibration hash required')
        if origin is None: origin='synthetic_simulation' if identity['arm_id']=='simulation' else 'physical'
        if origin not in {'physical','synthetic_simulation'}: raise ValueError('unsupported recording origin')
        if origin=='physical': require_physical_calibration(identity)
        self.origin=origin
        self.identity=copy.deepcopy(identity); self.clock=clock; self.started=clock()
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
        return {'schema':'so101-blockly-stroke-v2','name':self.name,'identity':self.identity,
                'binding':durable_binding(self.identity),'capture_cohort':{key:value for key,value in self.identity.items() if key in CAPTURE_FIELDS},
                'origin':self.origin,
                'joint_names':list(JOINTS),'jaw_radians':self.jaw,
                'points':[{'time':r['source_stamp']-origin,'positions':r['positions']} for r in self.rows]}


def replay_points(recording, identity, speed=.5, *, timing='original', start_radians=None, physical=False):
    if timing not in ('original','skip_stationary_pauses'): raise ValueError('unknown replay timing mode')
    number(speed,.1,1)
    binding=recording_binding(recording)
    origin=recording.get('origin','synthetic_simulation' if binding.get('arm_id')=='simulation' else 'physical')
    if physical and origin!='physical': raise ValueError('synthetic recording cannot play physically')
    if physical:
        require_physical_calibration(binding)
        require_physical_calibration(identity)
    if binding!=durable_binding(identity):
        raise ValueError('recording identity/calibration differs from current follower')
    if recording.get('joint_names')!=list(JOINTS): raise ValueError('recording joint names differ')
    check=Recorder(recording['name'],identity,origin=origin)
    for row in recording.get('points',[]):
        if set(row)!={'time','positions'}: raise ValueError('unsupported recording fields')
        check.add(row['positions'],recording['jaw_radians'],row['time'])
    validated=check.finish()
    rows=validated['points']
    if timing=='skip_stationary_pauses':
        candidate=stationary_pause_points(rows)
        if len(candidate)!=len(rows):
            baseline_bound,candidate_bound=nominal_curve_bounds(JOINTS,rows,candidate,start_radians)
            if candidate_bound>baseline_bound:
                raise ValueError('skipping pauses expands the nominal curve deviation bound; keep original timing or re-record a shorter stroke')
        rows=candidate
    return [{'time':row['time']/speed,'positions':list(row['positions'])} for row in rows]


def stationary_pause_points(points):
    """Shorten only exact constant runs; retain their first two and last two knots.

    Every nonzero position edge, in order, remains unchanged. No input or saved
    recording is mutated. Timing between moving edges is deliberately changed.
    """
    result=[]; offset=0.; begin=0
    for end in range(1,len(points)+1):
        if end<len(points) and points[end]['positions']==points[begin]['positions']: continue
        indices=list(range(begin,end))
        if len(indices)>4: indices=indices[:2]+indices[-2:]
        previous=None
        for index in indices:
            if previous is not None and index>previous+1:
                offset+=points[index]['time']-points[previous+1]['time']
            result.append({'time':points[index]['time']-offset,'positions':list(points[index]['positions'])})
            previous=index
        begin=end
    return result


def recording_timing(recording):
    rows=recording['points']
    reduced=stationary_pause_points(rows)
    return {'original_duration_s':rows[-1]['time']-rows[0]['time'],
            'skip_stationary_pauses_duration_s':reduced[-1]['time']-reduced[0]['time'],
            'original_points':len(rows),'skip_stationary_pauses_points':len(reduced)}


def nominal_curve_bounds(joint_names, original, candidate, start_radians):
    """Compare threshold-free bounds of requested curves with the SAME start.

    Lazy import keeps simulation/portable bundles independent of the physical
    trajectory helpers. The bridge still owns validation of the actual final
    measured-start curve, its collision checks, timing limits and exact hash.
    """
    from .station.ros_trajectory import _spline
    if not isinstance(start_radians,dict) or set(start_radians)!=set(JOINTS):
        raise ValueError('fresh five-joint start is required for optional timing transformation')
    start=[number(start_radians[j],-math.pi*2,math.pi*2) for j in joint_names]
    def distance(point,a,b):
        direction=[y-x for x,y in zip(a,b)]; norm=sum(v*v for v in direction)
        progress=min(1.,max(0.,sum((x-y)*v for x,y,v in zip(point,a,direction))/norm)) if norm else 0.
        return math.sqrt(sum((x-(y+progress*v))**2 for x,y,v in zip(point,a,direction)))
    def bound(rows):
        times=[row['time'] for row in rows]
        values=[row['positions'] for row in rows]; values[0]=start
        coefficients=[_spline(times,[row[j] for row in values]) for j in range(5)]
        maximum=0.
        for index in range(len(rows)-1):
            duration=times[index+1]-times[index]
            for control in range(6):
                # Convert normalized power coefficients to degree-5 Bernstein controls.
                point=[sum(coefficients[j][index][power]*duration**power*
                           math.comb(control,power)/math.comb(5,power)
                           for power in range(control+1)) for j in range(5)]
                if not all(math.isfinite(value) for value in point):
                    raise ValueError('nominal spline bound is nonfinite; keep original timing')
                maximum=max(maximum,distance(point,rows[index]['positions'],rows[index+1]['positions']))
        return maximum
    # A Bezier curve lies in its control hull. Distance to the corresponding
    # taught line segment is convex, so these bound the entire nominal curve.
    try: return bound(original),bound(candidate)
    except (ZeroDivisionError,OverflowError) as error:
        raise ValueError('nominal spline bound failed; keep original timing') from error
