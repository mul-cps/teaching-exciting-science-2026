"""Canonical teaching contracts and reproducible learner projections."""
import copy
import json
import subprocess
import sys
from pathlib import Path
import pytest
from workshop.curriculum import load_curriculum, validate_curriculum, learner_projection

ROOT = Path(__file__).resolve().parents[1]

def test_approved_schedule_and_intro():
    c = load_curriculum()
    assert [s['minutes'] for s in c['sessions']] == [20,25,20,30,10,15,45,45,20,10]
    assert len(c['slides']) == 8
    assert sum(s['minutes'] for s in c['slides']) == 20
    assert c['pairs'] == 3
    assert all(len(circuit['stations']) == 3 for circuit in c['rotations'])

@pytest.mark.parametrize('mutate', [
    lambda c: c['sessions'][0].update(minutes=21),
    lambda c: c['missions'][0].pop('predict'),
    lambda c: c['missions'][0].update(example_id='missing'),
    lambda c: c['missions'][0].update(starter=[{'op':'unbounded'}]),
    lambda c: c['missions'][0].update(hints=[]),
    lambda c: c['missions'][1].update(id=c['missions'][0]['id']),
])
def test_invalid_curriculum_rejected(mutate):
    c = copy.deepcopy(load_curriculum())
    mutate(c)
    with pytest.raises(ValueError): validate_curriculum(c)

def test_no_instructor_answers_in_browser():
    c = load_curriculum()
    projection = learner_projection(c)
    text = json.dumps(projection)
    assert 'instructor_notes' not in text
    assert 'answer' not in projection
    for mission in c['missions']:
        assert mission['instructor_notes'] not in text

def test_generated_artifacts_current():
    subprocess.run([sys.executable, str(ROOT/'tools/build_notebooks.py'), '--check'], check=True)

def test_notebooks_tag_examples_and_no_ros_commands():
    n = json.loads((ROOT/'notebooks/Robot_Candy_Factory_Student.ipynb').read_text())
    codes = [c for c in n['cells'] if c['cell_type']=='code']
    assert all('example_id' in c['metadata'] for c in codes)
    assert not any('import rclpy' in ''.join(c['source']) for c in codes)
    assert 'Instructor guidance' not in json.dumps(n)


def test_examples_compile_and_missions_match_session_duration():
    c = load_curriculum()
    durations = {s['id']:s['minutes'] for s in c['sessions']}
    for m in c['missions']:
        assert m['minutes'] == durations[m['session_id']]
    for eid, example in c['examples'].items():
        compile(example['code'], '<curriculum '+eid+'>', 'exec')


def test_bundled_browser_contains_no_instructor_guidance(tmp_path):
    import zipfile
    target=tmp_path/'student.zip'
    subprocess.run([sys.executable,str(ROOT/'exercises/make_student_bundle.py'),'--output',str(target)],check=True)
    with zipfile.ZipFile(target) as archive:
        browser=archive.read('web/blockly-station/curriculum.json').decode()
        assert 'instructor_notes' not in browser
        assert 'curriculum/workshop.json' not in archive.namelist()
        for mission in load_curriculum()['missions']:
            assert mission['instructor_notes'] not in browser


def test_live_notebook_respects_launch_flag_and_never_falls_back(monkeypatch):
    import subprocess
    from unittest.mock import patch
    c = load_curriculum()
    assert 'live_setup' in c['examples']
    notebook = json.loads((ROOT/'notebooks/Robot_Candy_Factory_Live_SO101.ipynb').read_text())
    tagged = {cell['metadata'].get('example_id'): ''.join(cell['source'])
              for cell in notebook['cells'] if cell['cell_type']=='code'}
    for eid in ('live_setup','live_capture','live_plot','live_recapture','joint'):
        assert tagged[eid] == c['examples'][eid]['code']
    monkeypatch.setenv('WORKSHOP_LIVE','1')
    ns = {}
    exec(tagged['setup'], ns)
    exec(tagged['live_setup'], ns)
    assert ns['LIVE'] is True
    with patch('workshop.live_observer.subprocess.run',side_effect=OSError('isolated unavailable fixture')) as transport:
        exec(tagged['live_capture'], ns)
    assert transport.call_count == 1
    assert ns['report']['samples'] == []
    assert ns['report']['mode'] == 'LIVE unavailable — no measurements'
    assert 'invented' not in ns['report']['mode']
    monkeypatch.setenv('WORKSHOP_LIVE','0')
    exec(tagged['live_setup'], ns)
    with patch('workshop.live_observer.subprocess.run',side_effect=AssertionError('offline must not launch ROS')):
        exec(tagged['live_capture'], ns)
    assert ns['LIVE'] is False and 'invented' in ns['report']['mode']


def test_projected_slide_cards_and_full_deck():
    import re
    import zipfile
    c=load_curriculum()
    fields=('worked_example','predict','change','run','observe','explain','success','extension')
    for mission in c['missions']:
        card=mission['slide_card']
        assert all(isinstance(card.get(k),str) and 0<len(card[k])<=160 for k in fields)
        assert len(card['hints'])==2 and all(len(h)<=55 for h in card['hints'])
    with zipfile.ZipFile(ROOT/'docs/workshop-slides.pptx') as deck:
        slides=[name for name in deck.namelist() if re.fullmatch(r'ppt/slides/slide\d+\.xml',name)]
        assert len(slides)==8+2*len(c['missions'])==24
        for index,mission in enumerate(c['missions']):
            first=deck.read(f'ppt/slides/slide{9+index*2}.xml').decode()
            second=deck.read(f'ppt/slides/slide{10+index*2}.xml').decode()
            assert 'Worked example' in first and 'Predict' in first and 'Change' in first and 'Run' in first
            assert 'Observe' in second and 'Explain' in second and 'Success' in second and 'Hints' in second
            assert 'sz="2200"' in first and 'sz="2400"' in first


def test_final_core_starter_named_skill_is_simulation_not_physical_evidence():
    from workshop.blockly_client import simulate_program
    c=load_curriculum()
    starter=next(m['starter'] for m in c['missions'] if m['id']=='integrate')
    assert starter == [{'op':'ev3','command':'eject'},{'op':'ev3','command':'left_end'},
                       {'op':'pick_place_recording','name':'fixed_pick_place','speed':.5}]
    result=simulate_program(starter)
    assert result['simulation'] is True
    assert 'synthetic' in str(result).lower()
    skill=result['taught_skills'][0]
    assert skill['physical_evidence'] is False
    assert skill['name']=='fixed_pick_place'
    assert all(skill[k] is False for k in ('pickup_verified','lift_verified','release_verified'))


@pytest.mark.parametrize('prompts', [None, [], 'Predict', [None], [''], ['   '], [42], {'predict': 'value'}])
def test_malformed_worksheet_prompts_rejected(prompts):
    c = copy.deepcopy(load_curriculum())
    c['missions'][0]['worksheet_prompts'] = prompts
    with pytest.raises(ValueError, match='worksheet'):
        validate_curriculum(c)


def test_worksheet_uses_each_canonical_prompt_in_order():
    from tools.build_notebooks import outputs
    c = copy.deepcopy(load_curriculum())
    c['missions'][0]['worksheet_prompts'] = ['Predict a unique direction', 'Record a unique observation']
    worksheet = outputs(c)['docs/student-worksheet.md']
    section = worksheet.split('## '+c['missions'][0]['title']+'\n\n', 1)[1].split('\n\n## ', 1)[0]
    assert 'Predict a unique direction: ______' in section
    assert 'Record a unique observation: ______' in section
    assert section.index('Predict a unique direction') < section.index('Record a unique observation')
    assert 'Our prediction was' not in section
    assert 'Evidence source (circle): PAPER / SIMULATION / PHYSICAL' in section
    assert 'Success check: '+c['missions'][0]['success'] in section
    assert 'Pair names:' in worksheet


def test_worksheet_records_each_rotation_position_and_integration_step():
    c = load_curriculum()
    missions = {m['id']: m for m in c['missions']}
    prompts = missions['explore']['worksheet_prompts']
    assert len(prompts) == 8
    for station in c['rotations'][0]['stations']:
        rows = [p.lower() for p in prompts if station in p]
        assert len(rows) == 2
        assert any('predict' in p for p in rows)
        assert any('run' in p and 'observed evidence' in p for p in rows)
    integration = missions['integrate']['worksheet_prompts']
    assert len(integration) == 9
    evidence = [p.lower() for p in integration if 'observed evidence' in p.lower()]
    assert len(evidence) == 6
    for step in ('dispense', 'arrival', 'grasp', 'lift', 'place', 'return'):
        assert sum(step in p for p in evidence) == 1
    assert any('stopped' in p and 'pickup' in p for p in evidence)
    assert any('place' in p and 'release' in p for p in evidence)


def test_beginner_comparison_names_the_command_list_view_and_reset():
    missions = {m['id']: m for m in load_curriculum()['missions']}
    assert '**Reset simulation**' in missions['blocks']['run']
    observe = missions['python']['observe']
    assert '**View executable Python**' in observe
    assert '`program.program`' in observe and '`program`' in observe
    assert 'JSON is text' in observe
    assert '**Save blocks**' in observe and 'reopen' in observe
    assert 'View executable Python' in missions['python']['slide_card']['observe']


def test_rotation_and_share_have_completed_simulation_examples(capsys):
    from workshop.blockly_client import simulate_program
    c = load_curriculum()
    missions = {m['id']: m for m in c['missions']}
    namespace = {}
    exec(c['examples']['joint']['code'], namespace)
    assert 'Relative prediction: 35 degrees' in capsys.readouterr().out
    example = missions['explore']['worked_example']
    for marker in ('SIMULATION', 'prediction', 'changed', '35', '30 + 15 - 10', 'Teach demo', 'Observe and decide'):
        assert marker in example
    assert 'physical' in example.lower()
    result = simulate_program([{'op':'repeat','count':2,'body':[{'op':'log','message':'Ready'}]}])
    assert result['simulation'] is True
    assert result['events'].count('Ready') == 2
    sample = missions['share']['worked_example']
    for marker in ('SIMULATION', 'predicted', '1 to 2', 'Ready', 'twice', 'physical'):
        assert marker in sample
    for mission_id in ('explore', 'share'):
        card = missions[mission_id]['slide_card']['worked_example']
        assert 'SIMULATION' in card
        assert ('35' if mission_id=='explore' else 'twice') in card
