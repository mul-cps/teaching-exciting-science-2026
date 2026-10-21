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
        assert not any("station/" in name or name == "workshop/vision.py" for name in archive.namelist())
        archive.extractall(extracted)
    for name in ("check_setup.py", "01_python.py", "02_robot.py", "03_vision.py", "04_sort.py"):
        subprocess.run([sys.executable, "-S", str(extracted / "exercises" / name)], cwd=tmp_path, check=True)
    report = tmp_path / "sorting.html"
    subprocess.run([sys.executable, str(extracted / "exercises/05_visual_sort.py"), "--output", str(report)], cwd=tmp_path, check=True)
    html = report.read_text()
    assert "left: 2" in html and "right: 2" in html and "reject: 1" in html
    assert "http://" not in html and "https://" not in html
    assert html.count('class="object"') == 5


def test_first_exercise_greeting_and_decision(tmp_path):
    source = (ROOT / "exercises/01_python.py").read_text()
    for color, expected in (("red", "left"), ("blue", "right"), ("yellow", "reject")):
        result = subprocess.run([sys.executable, "-S", "-c", source.replace('COLOR = "red"', f'COLOR = "{color}"')],
                                cwd=tmp_path, capture_output=True, text=True, check=True)
        assert "Object number 3" in result.stdout
        assert "Decision: " + expected in result.stdout


def test_all_plain_entries_without_site_packages(tmp_path):
    for name in ("check_setup.py", "01_python.py", "02_robot.py", "03_vision.py", "04_sort.py", "05_visual_sort.py"):
        result = subprocess.run([sys.executable, "-S", str(ROOT / "exercises" / name)],
                                cwd=tmp_path, capture_output=True, text=True, check=True)
        if name == "03_vision.py":
            assert "{'left': 2, 'right': 2, 'reject': 1}" in result.stdout
        if name == "04_sort.py":
            assert "left : 2" in result.stdout and "right : 2" in result.stdout and "reject : 1" in result.stdout
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
