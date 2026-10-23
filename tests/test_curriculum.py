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
