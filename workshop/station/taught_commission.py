"""Build an UNCOMMISSIONED manifest from instructor-staged private artifacts.

This helper never moves hardware and cannot mark a skill qualified. Recordings,
calibrations and commissioning evidence must already exist under the root.
"""
import argparse
import hashlib
import json
from pathlib import Path
from workshop.blockly_program import name
from workshop.blockly_recording import durable_binding
from workshop.blockly_skills import ARTIFACTS,PHASES,SCHEMA,SkillLibrary


def draft(root,skill_name,*,execution_config,joint_calibration,phases,checkpoints,
          fixture_id,fixture_evidence,instructor,qualification_evidence,frame_id,tip_link):
    root=Path(root).resolve();name(skill_name)
    def artifact(path):
        path=Path(path).resolve()
        if not path.is_relative_to(root): raise ValueError('stage all artifacts under private root')
        return {'path':str(path.relative_to(root)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    config_path=Path(execution_config).resolve();config=json.loads(config_path.read_text())
    artifacts={'execution_config':artifact(config_path),'joint_calibration':artifact(joint_calibration)}
    for key in ARTIFACTS[1:-1]:
        path=Path(config[key]);path=path if path.is_absolute() else config_path.parent/path
        artifacts[key]=artifact(path)
    if set(phases)!=set(PHASES): raise ValueError('four fixed-jaw phases required')
    recordings={phase:json.loads(Path(path).read_text()) for phase,path in phases.items()}
    first=recordings['approach'];binding=durable_binding(first['identity'])
    if any(durable_binding(r['identity'])!=binding or r.get('origin')!=first.get('origin') for r in recordings.values()):
        raise ValueError('all recordings must share durable identity and origin')
    binding['effective_model_sha256']=config['effective_model_sha256']
    data={'schema':SCHEMA,'name':skill_name,'origin':first['origin'],'qualified':False,
          'qualification':{'instructor':instructor,'evidence_sha256':artifact(qualification_evidence)['sha256']},
          'binding':binding,'frame_id':frame_id,'tip_link':tip_link,
          'fixture':{'id':fixture_id,'provenance_sha256':artifact(fixture_evidence)['sha256']},
          'checkpoints':checkpoints,'artifacts':artifacts,'phases':{phase:artifact(path) for phase,path in phases.items()}}
    output=root/(skill_name+'.json')
    with output.open('x') as stream: json.dump(data,stream,indent=2,allow_nan=False)
    try: SkillLibrary(root).load(skill_name)
    except BaseException: output.unlink();raise
    return output


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for field in ('root','name','execution-config','joint-calibration','checkpoints','fixture-id',
                  'fixture-evidence','instructor','qualification-evidence','frame-id','tip-link',*PHASES):
        parser.add_argument('--'+field,required=True)
    args=vars(parser.parse_args(argv));phases={phase:args.pop(phase) for phase in PHASES}
    args['skill_name']=args.pop('name');args['phases']=phases
    args['checkpoints']=json.loads(Path(args['checkpoints']).read_text())
    print(str(draft(**args))+' (UNCOMMISSIONED; no physical authorization)')


if __name__=='__main__': main()
