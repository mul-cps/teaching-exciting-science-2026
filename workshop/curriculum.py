"""Load and validate the single authored workshop curriculum (standard library)."""
import copy
import json
import re
from pathlib import Path
from .blockly_program import validate_program
ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'curriculum/workshop.json'
SCHEDULE = [20,25,20,30,10,15,45,45,20,10]
STAGES = ('worked_example','predict','change','run','observe','explain','success','extension','offline_route')


def validate_curriculum(c):
    """Fail closed on drift in the approved beginner route and its references."""
    def require(ok, message):
        if not ok: raise ValueError(message)
    require(c.get('schema_version') == 1, 'unsupported curriculum version')
    require(c.get('language') == 'en' and c.get('pairs') == 3, 'English and three pairs required')
    sessions = c.get('sessions', [])
    require([s['minutes'] for s in sessions] == SCHEDULE, 'approved 240-minute schedule changed')
    require(len({s['id'] for s in sessions}) == len(sessions), 'duplicate session')
    slides = c.get('slides', [])
    require(len(slides) == 8 and sum(s['minutes'] for s in slides) == 20, 'eight slides totaling 20 minutes required')
    examples = c.get('examples', {})
    for slide in slides:
        require(bool(slide['lines']) and bool(slide['notes']), 'slide needs text and notes')
        if slide.get('asset'):
            require((ROOT/'docs/assets/lecture'/slide['asset']).is_file(), 'missing slide photo')
    missions = c.get('missions', [])
    require(bool(missions) and len({m['id'] for m in missions}) == len(missions), 'duplicate or missing missions')
    for m in missions:
        require(m['session_id'] in {s['id'] for s in sessions}, 'missing mission session')
        require(m.get('example_id') in examples, 'missing mission example')
        require(all(isinstance(m.get(k), str) and m[k].strip() for k in STAGES), 'mission needs every learning stage')
        require(len(m.get('hints', [])) == 2 and all(m['hints']), 'two progressive hints required')
        prompts = m.get('worksheet_prompts')
        require(isinstance(prompts, list) and bool(prompts) and all(isinstance(p, str) and p.strip() for p in prompts), 'mission needs a nonempty list of nonempty worksheet prompts')
        require(bool(m.get('instructor_notes')), 'mission needs instructor guidance')
        card=m.get('slide_card', {})
        require(all(isinstance(card.get(k), str) and 0<len(card[k])<=160 for k in STAGES if k!='offline_route'), 'slide card needs concise learning fields')
        require(len(card.get('hints', []))==2 and all(isinstance(h,str) and 0<len(h)<=55 for h in card['hints']), 'slide card needs two concise hints')
        validate_program(m['starter'])
    require(len(c.get('rotations', [])) == 2, 'two circuits required')
    for circuit in c['rotations']:
        require(len(circuit['stations']) == 3, 'three circuit positions required')
    return c


def load_curriculum(path=SOURCE):
    return validate_curriculum(json.loads(Path(path).read_text()))


def learner_projection(c):
    """Allowlist the browser projection: instructor notes never leave this layer."""
    fields = ('id','title','session_id','minutes','goal','example_id','starter','toolbox','hints','worksheet_prompts') + STAGES
    missions = [{k:copy.deepcopy(m[k]) for k in fields} for m in c['missions']]
    for mission in missions:
        # The browser uses textContent; keep document images in print/notebooks.
        mission['worked_example'] = re.sub(
            r'(?m)^!\[[^\]\n]*\]\([^\)\n]+\)[ \t]*$', '', mission['worked_example'])
    return {'schema_version':c['schema_version'], 'title':c['title'], 'intro':c['intro'],
            'evidence_rule':c['evidence_rule'], 'sessions':copy.deepcopy(c['sessions']),
            'missions':missions,
            'examples':copy.deepcopy(c['examples']), 'rotations':copy.deepcopy(c['rotations'])}
