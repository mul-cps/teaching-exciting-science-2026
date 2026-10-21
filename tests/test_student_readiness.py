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
    subprocess.run([sys.executable, str(extracted / "exercises/check_setup.py")], cwd=tmp_path, check=True)
    report = tmp_path / "sorting.html"
    subprocess.run([sys.executable, str(extracted / "exercises/05_visual_sort.py"), "--output", str(report)], cwd=tmp_path, check=True)
    html = report.read_text()
    assert "left: 2" in html and "right: 2" in html and "reject: 1" in html
    assert "http://" not in html and "https://" not in html
    assert html.count('class="object"') == 5
