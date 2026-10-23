"""Only stdlib student entrypoints; no ROS, camera, network or motor required."""
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

from workshop import Factory, SimBelt, SimCamera, SimRobot

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("reading", [None, [], {}, 42, "unknown", ""])
def test_invalid_readings_rejected(reading):
    factory = Factory(SimBelt(), SimCamera([reading]), SimRobot())
    assert factory.one_cycle() == "reject"
    assert factory.counts == {"left": 0, "right": 0, "reject": 1}


@pytest.mark.parametrize("destination", [None, [], "up", 42])
def test_invalid_destination_rejected(destination):
    factory = Factory(SimBelt(), SimCamera(["red"]), SimRobot())
    assert factory.one_cycle({"red": destination}) == "reject"


def test_camera_empty_has_clear_error():
    with pytest.raises(ValueError, match="at least one"):
        SimCamera([]).detect()


def test_third_color_and_reversed_mapping():
    factory = Factory(SimBelt(), SimCamera(["red", "blue", "green", None]), SimRobot())
    rules = {"red": "right", "blue": "left", "green": "left"}
    assert [factory.one_cycle(rules) for _ in range(4)] == ["right", "left", "left", "reject"]
    assert factory.counts == {"left": 2, "right": 1, "reject": 1}


def test_portable_bundle_all_entries(tmp_path):
    bundle = tmp_path / "student.zip"
    subprocess.run([sys.executable, str(ROOT / "exercises/make_student_bundle.py"), "--output", str(bundle)], check=True)
    extracted = tmp_path / "extracted"
    with zipfile.ZipFile(bundle) as archive:
        assert not any(name.startswith("workshop/station/") or name in ("workshop/vision.py", "workshop/blockly_ros.py") for name in archive.namelist())
        assert 'web/blockly-station/vendor/blockly_compressed.js' in archive.namelist()
        archive.extractall(extracted)
    for name in ("check_setup.py", "01_python.py", "02_robot.py", "03_vision.py", "04_sort.py"):
        subprocess.run([sys.executable, "-S", str(extracted / "exercises" / name)], cwd=tmp_path, check=True)
    report = tmp_path / "sorting.html"
    subprocess.run([sys.executable, str(extracted / "exercises/05_visual_sort.py"), "--output", str(report)], cwd=tmp_path, check=True)
    html = report.read_text()
    assert "left: 2" in html and "right: 2" in html and "reject: 1" in html
    assert "http://" not in html and "https://" not in html
    assert html.count('class="object"') == 5
    simulation = '''import runpy,sys,time
from pathlib import Path
module=runpy.run_path(sys.argv[1])
app=module['Application'](Path(sys.argv[2]))
assert not app.backend.physical
app.run([{'op':'repeat','count':2,'body':[{'op':'ev3','command':'eject'}]}])
deadline=time.monotonic()+3
while app.thread.is_alive() and time.monotonic()<deadline: time.sleep(.01)
assert app.status()['job']['state']=='complete'
assert app.backend.events==['ev3:eject','ev3:eject']
app.close()
'''
    subprocess.run([sys.executable, '-S', '-c', simulation,
                    str(extracted/'tools/run_blockly_station.py'), str(tmp_path/'recordings')],
                   cwd=tmp_path, check=True, timeout=10)


def test_first_exercise_greeting_and_loop(tmp_path):
    source = (ROOT / "exercises/01_python.py").read_text()
    result = subprocess.run([sys.executable, "-S", "-c", source.replace('REPEATS = 3', 'REPEATS = 2')],
                            cwd=tmp_path, capture_output=True, text=True, check=True)
    assert "Object number 2" in result.stdout
    assert "Object number 3" not in result.stdout
    assert "Welcome to the robot factory" in result.stdout


def test_all_plain_entries_without_site_packages(tmp_path):
    for name in ("check_setup.py", "01_python.py", "02_robot.py", "03_vision.py", "04_sort.py", "05_visual_sort.py"):
        result = subprocess.run([sys.executable, "-S", str(ROOT / "exercises" / name)],
                                cwd=tmp_path, capture_output=True, text=True, check=True)
        if name == "03_vision.py":
            assert "Actual: blue Reported: None" in result.stdout
        if name == "04_sort.py":
            assert "{'left': 2, 'right': 2, 'reject': 1}" in result.stdout
    assert (tmp_path / "sorting-result.html").is_file()


def test_bundle_has_learner_notebook_no_answers(tmp_path):
    bundle = tmp_path / "student.zip"
    subprocess.run([sys.executable, str(ROOT / "exercises/make_student_bundle.py"), "--output", str(bundle)], check=True)
    with zipfile.ZipFile(bundle) as archive:
        names=archive.namelist()
        assert "notebooks/Robot_Candy_Factory_Student.ipynb" in names
        assert "exercises/student_visuals.py" in names
        assert "notebooks/assets/red.jpg" in names
        assert not any("Instructor" in n or "instructor" in n for n in names)
        assert "Instructor guidance" not in archive.read("notebooks/Robot_Candy_Factory_Student.ipynb").decode()


def test_factory_sequence_stops_before_sensor_and_action():
    events=[]
    class Belt:
        def advance(self): events.append("move")
        def stop(self): events.append("stop")
    class Camera:
        def detect(self):
            events.append("read")
            return "red"
    class Robot:
        def sort(self, destination): events.append(destination)
    factory=Factory(Belt(), Camera(), Robot())
    factory.one_cycle()
    assert events == ["move", "stop", "read", "left", "stop"]


def test_factory_stops_after_sensor_error():
    events=[]
    class Belt:
        def advance(self): events.append("move")
        def stop(self): events.append("stop")
    class Camera:
        def detect(self): raise RuntimeError("camera disconnected")
    with pytest.raises(RuntimeError, match="disconnected"):
        Factory(Belt(), Camera(), SimRobot()).one_cycle()
    assert events == ["move", "stop", "stop"]


def test_published_bundle_matches_current_allowlist_and_source_bytes():
    import runpy
    builder=runpy.run_path(str(ROOT/'exercises/make_student_bundle.py'))
    with zipfile.ZipFile(ROOT/'docs/student-workshop.zip') as archive:
        entries=builder['bundle_entries'](ROOT)
        assert set(archive.namelist())==set(entries)
        assert 'workshop/blockly_skills.py' in entries
        assert 'web/blockly-station/curriculum.json' in entries
        assert not any('instructor' in n.lower() or 'qualification' in n.lower() or n.startswith('workshop/station/') for n in entries)
        assert 'curriculum/workshop.json' not in entries
        for name, data in entries.items():
            assert archive.read(name)==data, name
            assert archive.getinfo(name).date_time==(2026,10,22,0,0,0)


def test_published_bundle_portable_admission_endpoint(tmp_path):
    extracted=tmp_path/'published'
    with zipfile.ZipFile(ROOT/'docs/student-workshop.zip') as archive: archive.extractall(extracted)
    for name in ('check_setup.py','01_python.py','02_robot.py','03_vision.py','04_sort.py'):
        subprocess.run([sys.executable,'-S',str(extracted/'exercises'/name)],cwd=tmp_path,check=True,capture_output=True)
    script='''import json,runpy,sys,threading,time,urllib.request,urllib.error
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root))
module=runpy.run_path(str(root/'tools/run_blockly_station.py'))
app=module['Application'](Path(sys.argv[2]));server=module['make_server'](app,0)
thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
url='http://127.0.0.1:'+str(server.server_port)
def post(path,body):
    req=urllib.request.Request(url+path,data=json.dumps(body).encode(),headers={'X-Station-Token':app.token})
    return json.load(urllib.request.urlopen(req))
def rejected(body):
    try:post('/api/program/run',body)
    except urllib.error.HTTPError as error:assert error.code==400
    else:raise AssertionError('Missing/stale admission was accepted')
try:
    program=[{'op':'log','message':'published current endpoint'}]
    rejected({'program':program})
    admission=app.status()['admission']
    post('/api/program/stop',{})
    rejected({'program':program,'admission':admission})
    post('/api/program/run',{'program':program,'admission':app.status()['admission']})
    app.thread.join(3);assert app.status()['job']['state']=='complete'
    assert app.backend.events[-1]=='published current endpoint'
    exported=post('/api/python',{'program':[{'op':'pick_place_recording','name':'fixed_pick_place','speed':.5}]})
    compile(exported['python'],'<published export>','exec')
    assert 'physical=False' in exported['python']
finally:
    server.shutdown();server.server_close();thread.join(3);app.close()
'''
    subprocess.run([sys.executable,'-S','-c',script,str(extracted),str(tmp_path/'recordings')],cwd=tmp_path,check=True,timeout=15)
