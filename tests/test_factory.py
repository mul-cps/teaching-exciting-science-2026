from workshop import Factory, SimBelt, SimCamera, SimRobot


def test_cycle_routes_and_counts():
    robot, belt = SimRobot(), SimBelt()
    system = Factory(belt, SimCamera(["red", "blue", "green"]), robot)
    for _ in range(3):
        system.one_cycle()
    assert robot.actions == ["left", "right", "reject"]
    assert system.counts == {"left": 1, "right": 1, "reject": 1}
    assert belt.steps == 3


def test_unknown_does_not_go_to_sort_bin():
    system = Factory(SimBelt(), SimCamera(["unknown"]), SimRobot())
    assert system.one_cycle() == "reject"


def test_belt_stopped_after_camera_error():
    class BrokenCamera:
        def detect(self):
            raise RuntimeError("disconnected")
    class Belt(SimBelt):
        stopped = False
        def stop(self):
            self.stopped = True
    belt = Belt()
    try:
        Factory(belt, BrokenCamera(), SimRobot()).one_cycle()
    except RuntimeError:
        pass
    assert belt.stopped


def test_student_program_runs_without_installation():
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    out = subprocess.run([sys.executable, "exercises/04_sort.py"], cwd=root, capture_output=True, text=True, check=True)
    assert "'left': 2" in out.stdout


def test_belt_stops_before_robot_action():
    events = []
    class Belt:
        def advance(self): events.append("advance")
        def stop(self): events.append("stop")
    class Robot:
        def sort(self, destination): events.append("sort")
    Factory(Belt(), SimCamera(["red"]), Robot()).one_cycle()
    assert events[:3] == ["advance", "stop", "sort"]
