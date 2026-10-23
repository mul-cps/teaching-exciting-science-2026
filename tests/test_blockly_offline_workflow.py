"""Learner reset and exported Python use the existing bounded simulator."""
import ast
import io
import json
import os
from pathlib import Path
import subprocess
import threading
import urllib.error
import urllib.request
import venv
from zipfile import ZipFile

import pytest

from exercises.make_student_bundle import bundle_bytes
from tools.run_blockly_station import Application, make_server
from workshop.blockly_client import export_python, simulate_program
from workshop.blockly_program import ProgramRunner, SimulationBackend

ROOT = Path(__file__).resolve().parents[1]
PROGRAM = [{'op': 'log', 'message': 'Report: Grüße "quoted"\\\n'},
           {'op': 'repeat', 'count': 2, 'body': [{'op': 'ev3', 'command': 'eject'}]},
           {'op': 'ev3', 'command': 'left_end'}, {'op': 'jaw', 'gap_mm': 12}]


@pytest.fixture
def endpoint(tmp_path):
    app = Application(tmp_path / 'recordings')
    server = make_server(app, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def request(path, payload=None):
        data = None if payload is None else json.dumps(payload).encode()
        req = urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/{path}', data=data,
                                     headers={'X-Station-Token': app.token})
        try:
            with urllib.request.urlopen(req) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as response:
            return response.code, json.load(response)
    yield app, request
    server.shutdown(); server.server_close(); app.close()


def test_reset_retest_retains_inventory_and_invalidates_admission(endpoint):
    app, request = endpoint
    initial = app.status()['simulation_state']
    inventory = dict(app.recordings)
    app.run(PROGRAM + [{'op': 'stop'}]); app.thread.join(2)
    status = app.status()
    assert status['job']['state'] == 'stopped'
    assert status['simulation_state']['dispensed_count'] == 2
    old = status['admission']
    code, fresh = request('simulation/reset', {'admission': old})
    assert code == 200, fresh
    assert fresh['simulation_state'] == initial
    assert fresh['job'] == {'state': 'idle'}
    assert fresh['events'] == [] and fresh['record_error'] is None
    assert app.recordings == inventory and app.backend.recordings is app.recordings
    assert fresh['admission']['epoch'] > old['epoch']
    with pytest.raises(RuntimeError, match='stale admission'):
        app.run_external(PROGRAM, old)
    app.run_external(PROGRAM, fresh['admission']); app.thread.join(2)
    assert app.status()['simulation_state']['dispensed_count'] == 2


def test_reset_rejects_active_stopping_and_physical_without_mutation(endpoint):
    app, request = endpoint
    app.run([{'op': 'wait', 'seconds': 10}])
    backend = app.backend
    code, _ = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 409 and app.backend is backend
    app.stop(); app.thread.join(2)
    app.stop_pending = 1
    code, _ = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 409 and app.backend is backend
    app.stop_pending = 0
    app.backend.physical = True
    code, _ = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 409 and app.backend is backend
    app.backend.physical = False


def test_stop_does_not_reset_and_reset_requires_verified_terminal_stop(endpoint):
    app, request = endpoint
    app.run(PROGRAM); app.thread.join(2)
    count = app.status()['simulation_state']['dispensed_count']
    app.stop()
    assert app.status()['simulation_state']['dispensed_count'] == count
    code, fresh = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 200, fresh
    app.job = {'state': 'stopped', 'result': {'stopped': False}}
    code, _ = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 409


def test_server_selects_export_from_launcher_context_and_rejects_client_override(endpoint):
    app, request = endpoint
    code, reply = request('python', {'program': PROGRAM})
    assert code == 200
    assert reply['mode'] == 'offline_simulation'
    assert 'simulate_program(program, recordings=recordings)' in reply['python']
    assert 'Synthetic' in reply['help'] and 'no hardware' in reply['help']
    assert 'demo' in reply['help'] and 'separate synthetic' in reply['help']
    assert app.status()['python_export']['mode'] == 'offline_simulation'
    assert request('python', {'program': PROGRAM, 'offline': False})[0] == 400


@pytest.mark.parametrize('physical', [False, True])
def test_ros_launcher_context_keeps_shared_export(endpoint, physical):
    app, request = endpoint
    app.ros_api = True
    app.backend.physical = physical
    try:
        code, reply = request('python', {'program': PROGRAM})
        assert code == 200
        assert reply['mode'] == 'shared_station'
        assert f'run_program(program, physical={physical!r})' in reply['python']
        assert 'ROS' in reply['help']
    finally:
        app.backend.physical = False


def test_offline_physical_export_rejected():
    with pytest.raises(ValueError, match='physical'):
        export_python(PROGRAM, physical=True, offline=True)


@pytest.mark.parametrize('mode', [None, 0, 'False'])
def test_export_requires_explicit_boolean_modes(mode):
    with pytest.raises(ValueError): export_python(PROGRAM, physical=mode)
    with pytest.raises(ValueError): export_python(PROGRAM, physical=False, offline=mode)


@pytest.mark.parametrize('program', [
    PROGRAM,
    [{'op': 'replay', 'name': 'demo', 'speed': .5}],
    [{'op': 'approach_recording', 'name': 'demo', 'speed': .5}],
    next(mission['starter'] for mission in json.loads((ROOT/'curriculum/workshop.json').read_text())['missions']
         if mission['example_id'] == 'fixed_pick_place'),
], ids=['unicode-sequence', 'replay-demo', 'approach-demo', 'canonical-fixed-pick-place'])
def test_offline_export_executes_from_extracted_allowlisted_zip_without_ros(tmp_path, program):
    extracted = tmp_path / 'student'
    with ZipFile(io.BytesIO(bundle_bytes(ROOT))) as archive:
        assert 'tools/run_station_program.py' not in archive.namelist()
        assert 'ros2/conda/so101-conda.sh' not in archive.namelist()
        archive.extractall(extracted)
    venv.EnvBuilder(with_pip=False).create(tmp_path / 'cpu-python')
    python = tmp_path / 'cpu-python/bin/python'
    code = export_python(program, physical=False, offline=True)
    # Execute the unmodified downloaded file, then verify imports separately.
    (extracted / 'student_export.py').write_text(code)
    harness = '''import builtins, json, runpy, sys
original = builtins.__import__
def offline_import(name, *args, **kwargs):
    if name.split('.')[0] in {'rclpy', 'serial', 'lerobot', 'scservo_sdk'} or name == 'workshop.blockly_ros':
        raise AssertionError('hardware/ROS import: ' + name)
    return original(name, *args, **kwargs)
builtins.__import__ = offline_import
runpy.run_path('student_export.py', run_name='__main__')
'''
    (extracted / 'verify_export.py').write_text(harness)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('ROS', 'AMENT', 'COLCON', 'RMW', 'STATION_program'))
           and key not in {'PYTHONPATH', 'PYTHONHOME'}}
    result = subprocess.run([str(python), '-E', 'verify_export.py'], cwd=extracted, env=env,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip(), 'Downloaded offline Python must print its synthetic result and trace'
    captured = json.loads(result.stdout)
    direct = subprocess.run([str(python), '-E', 'student_export.py'], cwd=extracted, env=env,
                            capture_output=True, text=True, timeout=30)
    assert direct.returncode == 0, direct.stderr
    assert json.loads(direct.stdout) == captured
    assert '\n  "complete": true' in direct.stdout
    assert '\n  "simulation": true' in direct.stdout
    if program == PROGRAM: assert 'Grüße' in direct.stdout
    assert captured['complete'] is True and captured['simulation'] is True
    assert captured['status']['physical'] is False
    if any(row['op'] == 'pick_place_recording' for row in program):
        assert len(captured.get('taught_skills', [])) == 1
    for skill in captured.get('taught_skills', []):
        assert skill['physical_evidence'] is False
        assert not any(skill[key] for key in ('pickup_verified', 'lift_verified', 'release_verified'))
    backend = SimulationBackend({'demo': {'schema': 'simulation-only'}})
    expected = ProgramRunner(backend).execute(program, threading.Event())
    assert captured == {**expected, 'simulation': True, 'events': backend.events, 'status': backend.status()}
    assert captured == simulate_program(program, recordings={'demo': {'schema': 'simulation-only'}})
    scope = {'__name__': 'inspect_export'}
    exec(compile(code, 'student_export.py', 'exec'), scope)
    assert scope['program'] == program
    assert scope['recordings'] == {'demo': {'schema': 'simulation-only'}}
    assert 'never a physical recording' in code
    assert 'Other named strokes require separate synthetic resources' in code
    assert ast.literal_eval(next(node.value.args[0] for node in ast.walk(ast.parse(code))
                                if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call))) == json.dumps(program, ensure_ascii=False, allow_nan=False)


def test_browser_projects_canonical_mission_fields_as_text_and_reset_keeps_workspace():
    script = r'''
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync('web/blockly-station/app.js','utf8');
const html=fs.readFileSync('web/blockly-station/index.html','utf8');
const elements={};
const $=id=>elements[id]??=( {textContent:'',children:[],append(x){this.children.push(x)},replaceChildren(){this.children=[]}} );
let workspaceLoads=0;let resetCalls=0;const handlers={};
const context={$,document:{createElement:()=>({textContent:''})},missionToolbox(){},
 loadWorkspace(){workspaceLoads++},clicked:(id,fn)=>handlers[id]=fn,
 window:{stationStatus:{admission:{session:'test',epoch:0}}},show(){},
 api:async(path)=>{assert.equal(path,'simulation/reset');resetCalls++;return {admission:{session:'test',epoch:1},job:{state:'idle'}}}};
vm.createContext(context);
vm.runInContext('let currentMission;let mutationEpoch=0;let lastJob="";'+source.slice(source.indexOf('function astBlock('),source.indexOf('async function initMissions()')),context);
const curriculum=JSON.parse(fs.readFileSync('web/blockly-station/curriculum.json'));
for(const mission of curriculum.missions){context.loadMission(mission);
 assert.equal($('mission-worked-example').textContent,mission.worked_example);
 assert.equal($('mission-extension').textContent,mission.extension);
 assert.equal($('mission-offline-route').textContent,mission.offline_route);
 assert($('mission-steps').children[0].textContent.startsWith('Predict: '));}
const hostile={...curriculum.missions[0],worked_example:'<img src=x onerror=alert(1)>',extension:'<script>bad</script>',offline_route:'<b>route</b>'};
context.loadMission(hostile);assert.equal($('mission-worked-example').textContent,hostile.worked_example);
assert(html.indexOf('id="mission-worked-example"')<html.indexOf('id="mission-steps"'));
assert(/<details><summary>Optional challenge<\/summary>/.test(html));
assert(/<details><summary>Offline route<\/summary>/.test(html));
assert(html.includes('id="reset-simulation"')&&html.includes('Reset simulation'));
const reset=source.slice(source.indexOf("clicked('reset-simulation'"),source.indexOf("clicked('record-start'"));
vm.runInContext(reset,context);
const before=workspaceLoads;
(async()=>{await handlers['reset-simulation']();assert.equal(workspaceLoads,before);assert.equal(resetCalls,1);assert.equal(context.window.stationStatus.admission.epoch,1)})().catch(error=>{console.error(error);process.exitCode=1});
'''
    result = subprocess.run(['node', '-e', script], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_completed_job_can_reset_but_failed_or_unverified_terminal_cannot(endpoint):
    app, request = endpoint
    app.run(PROGRAM); app.thread.join(2)
    assert app.job['result']['complete'] is True
    code, fresh = request('simulation/reset', {'admission': app.status()['admission']})
    assert code == 200 and fresh['simulation_state']['dispensed_count'] == 0
    assert request('simulation/reset', {'admission': fresh['admission']})[0] == 200
    for job in ({'state': 'complete', 'result': {}}, {'state': 'failed', 'error': 'cancelled'},
                {'state': 'stopping'}, {'state': 'stopped', 'result': {'stopped': 1}}):
        app.job = job
        assert request('simulation/reset', {'admission': app.status()['admission']})[0] == 409
    app.backend.stop = lambda: {'acknowledged': False, 'simulation': True}
    app.stop()
    assert request('simulation/reset', {'admission': app.status()['admission']})[0] == 409
    app.backend.stop = lambda: {'acknowledged': True, 'simulation': True}
    app.stop()
    assert request('simulation/reset', {'admission': app.status()['admission']})[0] == 200


def test_reset_rejects_actual_inflight_stop_and_old_reset_admission(endpoint):
    app, request = endpoint
    entered = threading.Event(); release = threading.Event()
    real_stop = app.backend.stop
    def stopping():
        entered.set()
        assert release.wait(2)
        return real_stop()
    app.backend.stop = stopping
    stop = threading.Thread(target=app.stop)
    stop.start(); assert entered.wait(1)
    try:
        assert request('simulation/reset', {'admission': app.status()['admission']})[0] == 409
    finally:
        release.set(); stop.join(2)
    old = app.status()['admission']
    assert request('simulation/reset', {'admission': old})[0] == 200
    code, rejected = request('simulation/reset', {'admission': old})
    assert code == 400 and 'stale admission' in rejected['error']


def test_shared_export_executes_exact_ast_through_injected_transport(monkeypatch):
    from workshop import blockly_client as client
    calls = []
    def endpoint(operation, body, timeout):
        body = json.loads(body); calls.append((operation, body))
        if operation == 'run': return {'id': 'export-fixture', 'state': 'running'}
        if operation == 'status':
            return {'physical': False, 'admission': {'session': 'shared-fixture', 'epoch': 0},
                    'job': {'id': 'export-fixture', 'state': 'complete', 'result': {'complete': True}}}
        raise AssertionError('unexpected transport operation')
    real = client.run_program
    def injected(program, *, physical):
        return real(program, physical=physical, transport=endpoint)
    monkeypatch.setattr(client, 'run_program', injected)
    scope = {'__name__': '__main__'}
    exec(compile(export_python(PROGRAM, physical=False), 'shared-export.py', 'exec'), scope)
    assert scope['program'] == PROGRAM
    assert calls[1] == ('run', {'program': PROGRAM, 'admission': {'session': 'shared-fixture', 'epoch': 0}})
    assert [operation for operation, _ in calls] == ['status', 'run', 'status']


def test_explicit_ros_context_selects_shared_python(tmp_path):
    app = Application(tmp_path, ros_api=True)
    try: assert app.status()['python_export']['mode'] == 'shared_station'
    finally: app.close()
