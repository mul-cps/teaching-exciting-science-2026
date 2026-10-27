"""Exercise the language bridge against its actual canonical Python example."""
from pathlib import Path
import re

from workshop.curriculum import load_curriculum

ROOT = Path(__file__).resolve().parents[1]


def test_python_mission_starter_matches_shared_program_and_toolbox():
    curriculum = load_curriculum()
    mission = next(m for m in curriculum['missions'] if m['id'] == 'python')
    namespace = {}
    exec(curriculum['examples']['shared']['code'], namespace)
    assert namespace['result']['complete'] is True
    assert mission['starter'] == namespace['program'].program
    # Read the browser's real category/type and opcode/type mappings, so the
    # assertion fails if the starter needs a category absent from this mission.
    browser = (ROOT / 'web/blockly-station/app.js').read_text()
    categories = dict(re.findall(r"(\w+):\[([^]]+)\]", browser.split('const categories=', 1)[1].split('};', 1)[0]))
    operation_types = dict(re.findall(r"(\w+):'([^']+)'", browser.split('function astBlock', 1)[1].split('[row.op]', 1)[0]))
    available = ' '.join(categories[name] for name in mission['toolbox'])
    for row in mission['starter']:
        assert operation_types[row['op']] in available


def test_browser_observation_keeps_plain_inputs_and_print_keeps_image():
    from workshop.curriculum import learner_projection
    curriculum = load_curriculum()
    canonical = next(m for m in curriculum['missions'] if m['id'] == 'observe')
    projected = next(m for m in learner_projection(curriculum)['missions'] if m['id'] == 'observe')
    assert '![' in canonical['worked_example']
    assert '![' not in projected['worked_example']
    assert 'notebooks/assets/photo-examples.jpg' in projected['worked_example']
    namespace = {}
    exec(curriculum['examples']['observation']['code'], namespace)
    for i, (actual, report) in enumerate(zip(namespace['ACTUAL'], namespace['REPORTED']), 1):
        assert f'Row {i}: actual {actual}, report {report}' in projected['worked_example']
    for key, value in projected.items():
        if key != 'worked_example':
            assert value == canonical[key]
    assert 'deliberately invented, not camera measurements' in projected['worked_example']


def test_teach_slide_asks_same_prediction_as_full_activity():
    mission = next(m for m in load_curriculum()['missions'] if m['id'] == 'teach-demo')
    assert mission['slide_card']['predict'] == mission['predict']
    assert '?' in mission['slide_card']['predict']
