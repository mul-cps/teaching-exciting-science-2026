"""Private instructor-authored taught transactions; names are the student API.

File digests establish immutable provenance, not physical qualification. Only an
instructor can supply qualified physical manifests after station commissioning.
"""
import copy
from itertools import islice
import hashlib
import json
import math
from pathlib import Path
from .blockly_program import name
from .blockly_recording import durable_binding, replay_points

SCHEMA='so101-taught-pick-place-v1'
PHASES=('approach','lift','transfer','return')
ARTIFACTS=('execution_config','jaw_calibration','tcp_calibration','camera_arm_calibration','joint_calibration')
BINDING=('arm_id','calibration_sha256','source_sha256','jaw_calibration_sha256','joint_calibration_sha256','effective_model_sha256')


def _json(path):
    raw=path.read_bytes()
    if len(raw)>250000: raise ValueError('skill artifact too large')
    return _decode(raw)


def _decode(raw):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result: raise ValueError('duplicate skill field')
            result[key]=value
        return result
    try:
        return json.loads(raw,object_pairs_hook=unique,
                          parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    except RecursionError as error:
        raise ValueError('skill JSON nesting exceeds parser bound') from error


def _hash(value):
    if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('lowercase SHA256 required')
    return value


def _pose(value):
    if (not isinstance(value,list) or len(value)!=7 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in value) or
            not math.isclose(sum(v*v for v in value[3:]),1.,abs_tol=1e-5)):
        raise ValueError('checkpoint requires finite XYZ and unit XYZW')


def validate_skill_result(result,track_id=None):
    if (not isinstance(result,dict) or result.get('state')!='pick_place_complete' or
            not isinstance(result.get('track_id'),str) or not result['track_id'] or
            (track_id is not None and result['track_id']!=track_id) or
            any(result.get(field) is not True for field in
                ('pickup_verified','lift_verified','release_verified')) or
            result.get('simulation') is True):
        raise RuntimeError('taught pickup, lift, release and exact tracked target must all be verified')
    return result


def require_taught_calibration_status(status,skill):
    if (not isinstance(status,dict) or status.get('si_gap_calibrated') is not True or
            status.get('moveit_joint_calibrated') is not True or status.get('visual_proxy_only') is not False):
        raise RuntimeError('taught skill requires qualified SI gap and measured joint-angle calibration')
    for field,artifact in (('jaw_calibration_sha256','jaw_calibration'),('joint_calibration_sha256','joint_calibration')):
        if status.get(field)!=skill['artifacts'][artifact]['sha256']:
            raise RuntimeError('resident loaded calibration digest missing/changed; revalidate taught skill')


class SkillLibrary:
    def __init__(self,root):
        self.root=Path(root).resolve() if root is not None else None
        self._catalog={}
        if self.root is not None:
            # Bound discovery BEFORE sorting/materializing; status is memory-only.
            for path in sorted(islice(self.root.glob('*.json'),64)):
                try: self.load(path.stem)
                except (ValueError,TypeError,KeyError,OSError): pass

    def _artifact(self,entry,*,parsed=False):
        if not isinstance(entry,dict) or set(entry)!={'path','sha256'}:
            raise ValueError('artifact requires only path and sha256')
        _hash(entry['sha256'])
        relative=entry['path']
        if not isinstance(relative,str) or not relative or Path(relative).is_absolute():
            raise ValueError('artifact path must be relative to private skill root')
        path=(self.root/relative).resolve()
        if not path.is_relative_to(self.root) or not path.is_file():
            raise ValueError('artifact missing or outside private skill root')
        raw=path.read_bytes()
        if len(raw)>250000: raise ValueError('skill artifact too large')
        if hashlib.sha256(raw).hexdigest()!=entry['sha256']:
            raise ValueError('artifact hash changed; reteach/revalidate skill')
        return (path,_decode(raw)) if parsed else path

    def load(self,skill_name,*,physical=False,identity=None):
        try:
            data=self._load(skill_name,physical=physical,identity=identity)
        except (ValueError,TypeError,KeyError,OSError) as error:
            if isinstance(skill_name,str) and (len(self._catalog)<64 or skill_name in self._catalog):
                self._catalog[skill_name]={'state':'invalid','reason':str(error)[:1000]}
            raise
        if len(self._catalog)<64 or skill_name in self._catalog:
            self._catalog[skill_name]={'state':'revalidation_required' if data['qualified'] else 'uncommissioned',
                'reason':'fresh live identity, model, calibration and physical evidence gates required'}
        return data

    def _load(self,skill_name,*,physical=False,identity=None):
        name(skill_name)
        if self.root is None: raise ValueError('taught skill not found: private library is not configured')
        path=self.root/(skill_name+'.json')
        if not path.is_file() or not path.resolve().is_relative_to(self.root):
            raise ValueError('taught skill not found: '+skill_name)
        data=_json(path)
        fields={'schema','name','origin','qualified','qualification','binding','frame_id','tip_link',
                'fixture','checkpoints','artifacts','phases'}
        if not isinstance(data,dict) or set(data)!=fields or data['schema']!=SCHEMA or data['name']!=skill_name:
            raise ValueError('invalid taught skill schema/name/fields')
        if data['origin'] not in {'physical','synthetic_simulation'} or type(data['qualified']) is not bool:
            raise ValueError('invalid taught skill origin/qualification')
        if physical and data['origin']!='physical': raise ValueError('synthetic skill cannot run physically')
        qualification=data['qualification']
        if (not isinstance(qualification,dict) or set(qualification)!={'instructor','evidence_sha256'} or
                not isinstance(qualification['instructor'],str) or not qualification['instructor']):
            raise ValueError('instructor qualification provenance required')
        _hash(qualification['evidence_sha256'])
        if physical and data['qualified'] is not True: raise ValueError('taught skill is uncommissioned')
        binding=data['binding']
        if not isinstance(binding,dict) or set(binding)!=set(BINDING) or not isinstance(binding['arm_id'],str) or not binding['arm_id']:
            raise ValueError('complete robot/calibration/source/model binding required')
        for field in BINDING[1:]: _hash(binding[field])
        if identity is not None:
            live=durable_binding(identity)
            if any(live.get(key)!=binding[key] for key in BINDING[:-1]):
                raise ValueError('skill robot/calibration/source changed; reteach required')
        if any(not isinstance(data[key],str) or not data[key] for key in ('frame_id','tip_link')):
            raise ValueError('frame and TCP link required')
        fixture=data['fixture']
        if not isinstance(fixture,dict) or set(fixture)!={'id','provenance_sha256'} or not isinstance(fixture['id'],str) or not fixture['id']:
            raise ValueError('fixed fixture provenance required')
        _hash(fixture['provenance_sha256'])
        checkpoints=data['checkpoints']
        if not isinstance(checkpoints,dict) or set(checkpoints)!={'pickup','drop','approach_start','return'}:
            raise ValueError('fixed pickup/drop/approach/return checkpoints required')
        for value in checkpoints.values(): _pose(value)
        if not isinstance(data['artifacts'],dict) or set(data['artifacts'])!=set(ARTIFACTS):
            raise ValueError('complete calibrated execution artifacts required')
        snapshots={key:self._artifact(value,parsed=True) for key,value in data['artifacts'].items()}
        artifacts={key:value[0] for key,value in snapshots.items()}
        for field,artifact in (('jaw_calibration_sha256','jaw_calibration'),('joint_calibration_sha256','joint_calibration')):
            if binding[field]!=data['artifacts'][artifact]['sha256']:
                raise ValueError('skill calibration binding differs from pinned artifact; reteach required')
        config=snapshots['execution_config'][1]
        if not isinstance(config,dict): raise ValueError('execution config must be an object')
        if config.get('effective_model_sha256')!=binding['effective_model_sha256']:
            raise ValueError('effective model differs from taught binding')
        for key in ARTIFACTS[1:-1]:
            configured=Path(config.get(key,''))
            if not configured.is_absolute(): configured=artifacts['execution_config'].parent/configured
            if configured.resolve()!=artifacts[key]: raise ValueError('execution calibration path differs from pinned artifact')
        angular=snapshots['joint_calibration'][1]
        if (not isinstance(angular,dict) or angular.get('schema')!='so101-gripper-joint-calibration-v1' or angular.get('qualified') is not True or
                angular.get('arm_id')!=binding['arm_id'] or angular.get('calibration_sha256')!=binding['calibration_sha256']):
            raise ValueError('qualified identity-matched jaw joint-angle calibration required')
        if not isinstance(data['phases'],dict) or set(data['phases'])!=set(PHASES):
            raise ValueError('all four taught phases required')
        recordings={}
        for phase,entry in data['phases'].items():
            _,recording=self._artifact(entry,parsed=True)
            if not isinstance(recording,dict): raise ValueError('phase recording must be an object')
            expected={key:binding[key] for key in BINDING[:-1]}
            replay_points(recording,identity if identity is not None else expected,physical=physical)
            recordings[phase]=recording
        return {**data,'manifest_path':str(path),'artifact_paths':artifacts,'recordings':recordings}

    def status(self):
        if self.root is None: return {'state':'missing','reason':'private taught skill library not configured','skills':{}}
        return {'state':'configured','skills':copy.deepcopy(self._catalog)}
